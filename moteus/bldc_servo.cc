// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "bldc_servo.h"

#include <atomic>
#include <cmath>
#include <functional>

#include "mbed.h"
#include "serial_api_hal.h"

#include "PeripheralPins.h"

#include "mjlib/base/assert.h"
#include "mjlib/base/limit.h"
#include "mjlib/base/windowed_average.h"

#include "moteus/foc.h"
#include "moteus/math.h"
#include "moteus/moteus_hw.h"
#include "moteus/stm32_serial.h"

#if defined(TARGET_STM32F4)
#include "moteus/stm32f446_async_uart.h"
#elif defined(TARGET_STM32G4)
#include "moteus/stm32g4_async_uart.h"
#else
#error "Unknown target"
#endif

#ifdef wait_us
#undef wait_us
#endif

namespace micro = mjlib::micro;

namespace moteus {

namespace {

#if defined(TARGET_STM32F4)
using HardwareUart = Stm32F446AsyncUart;
#elif defined(TARGET_STM32G4)
using HardwareUart = Stm32G4AsyncUart;
#else
#error "Unknown target"
#endif


using mjlib::base::Limit;

float Threshold(float value, float lower, float upper) {
  if (value > lower && value < upper) { return 0.0f; }
  return value;
}

template <typename AdcType>
void WaitForAdc(AdcType* adc) {
#if defined(TARGET_STM32F4)
    while ((adc->SR & ADC_SR_EOC) == 0);
#elif defined(TARGET_STM32G4)
    while ((adc->ISR & ADC_ISR_EOC) == 0);
    adc->ISR |= ADC_ISR_EOC;
#else
    #error "Unknown target"
#endif
}

// From make_thermistor_table.py
constexpr float g_thermistor_lookup[] = {
  -74.17f, // 0
  -11.36f, // 128
  1.53f, // 256
  9.97f, // 384
  16.51f, // 512
  21.98f, // 640
  26.79f, // 768
  31.15f, // 896
  35.19f, // 1024
  39.00f, // 1152
  42.65f, // 1280
  46.18f, // 1408
  49.64f, // 1536
  53.05f, // 1664
  56.45f, // 1792
  59.87f, // 1920
  63.33f, // 2048
  66.87f, // 2176
  70.51f, // 2304
  74.29f, // 2432
  78.25f, // 2560
  82.44f, // 2688
  86.92f, // 2816
  91.78f, // 2944
  97.13f, // 3072
  103.13f, // 3200
  110.01f, // 3328
  118.16f, // 3456
  128.23f, // 3584
  141.49f, // 3712
  161.02f, // 3840
  197.66f, // 3968
};

template <typename Array>
int MapConfig(const Array& array, int value) {
  static_assert(sizeof(array) > 0);
  int result = 0;
  for (const auto& item : array) {
    if (value <= item) { return result; }
    result++;
  }
  // Never return past the end.
  return result - 1;
}

constexpr int kIntRateHz = 30000;
constexpr int kPwmRateHz = 60000;
constexpr int kInterruptDivisor = kPwmRateHz / kIntRateHz;
static_assert(kPwmRateHz % kIntRateHz == 0);

// This is used to determine the maximum allowable PWM value so that
// the current sampling is guaranteed to occur while the FETs are
// still low.  It was calibrated using the scope and trial and error.
//
// For some reason, with 30kHz/60kHz settings, I measure this at
// 0.8us, but with 40/40 I get 1.3us.  Whatever, 1.4 is conservative
// and not going to cause problems.
constexpr float kCurrentSampleTime = 1.4e-6f;

constexpr float kMinPwm = kCurrentSampleTime / (0.5f / static_cast<float>(kPwmRateHz));
constexpr float kMaxPwm = 1.0f - kMinPwm;

constexpr float kRateHz = kIntRateHz;
constexpr float kPeriodS = 1.0f / kRateHz;

constexpr int kCalibrateCount = 256;

// The maximum amount the absolute encoder can change in one cycle
// without triggering a fault.  Measured relative to 32767
constexpr int16_t kMaxPositionDelta = 1000;

// mbed configures the Timer clock input to 90MHz.  For any given pwm
// frequency, we want double the actual frequency since we have an up
// down counter.
constexpr uint32_t kTimerClock = 90000000;
constexpr uint32_t kPwmCounts = kTimerClock / (2 * kPwmRateHz);

// However, the control counter we want at the actual frequency.
constexpr uint32_t kControlCounts = kTimerClock / (kIntRateHz);

constexpr float kDefaultTorqueConstant = 0.1f;
constexpr float kMaxUnconfiguredCurrent = 5.0f;

IRQn_Type FindUpdateIrq(TIM_TypeDef* timer) {
#if defined(TARGET_STM32F4)
  if (timer == TIM1) {
    return TIM1_UP_TIM10_IRQn;
  } else if (timer == TIM2) {
    return TIM2_IRQn;
  } else if (timer == TIM3) {
    return TIM3_IRQn;
  } else if (timer == TIM4) {
    return TIM4_IRQn;
  } else if (timer == TIM8) {
    return TIM8_UP_TIM13_IRQn;
  } else {
    MJ_ASSERT(false);
  }
  return TIM1_UP_TIM10_IRQn;
#elif defined(TARGET_STM32G4)
  if (timer == TIM2) {
    return TIM2_IRQn;
  } else if (timer == TIM3) {
    return TIM3_IRQn;
  } else if (timer == TIM4) {
    return TIM4_IRQn;
  }
  MJ_ASSERT(false);
  return TIM2_IRQn;
#else
#error "Unknown target"
#endif
}

volatile uint32_t* FindCcr(TIM_TypeDef* timer, PinName pin) {
  const auto function = pinmap_function(pin, PinMap_PWM);

  const auto inverted = STM_PIN_INVERTED(function);
  MJ_ASSERT(!inverted);

  const auto channel = STM_PIN_CHANNEL(function);

  switch (channel) {
    case 1: { return &timer->CCR1; }
    case 2: { return &timer->CCR2; }
    case 3: { return &timer->CCR3; }
    case 4: { return &timer->CCR4; }
  }
  MJ_ASSERT(false);
  return nullptr;
}

uint32_t FindSqr(PinName pin) {
  const auto function = pinmap_function(pin, PinMap_ADC);

  const auto channel = STM_PIN_CHANNEL(function);
  return channel;
}

#if defined(TARGET_STM32F4)
volatile uint32_t* const g_adc1_cr2 = &ADC1->CR2;
#elif defined(TARGET_STM32G4)
volatile uint32_t* const g_adc1_cr2 = &ADC1->CR;
#else
#error "Unknown target"
#endif

/// Read a digital input, but without configuring it in any way.
class DigitalMonitor {
 public:
  DigitalMonitor(PinName pin) {
    const uint32_t port_index = STM_PORT(pin);
    GPIO_TypeDef* gpio = reinterpret_cast<GPIO_TypeDef*>([&]() {
      switch (port_index) {
        case PortA: return GPIOA_BASE;
        case PortB: return GPIOB_BASE;
        case PortC: return GPIOC_BASE;
        case PortD: return GPIOD_BASE;
        case PortE: return GPIOE_BASE;
        case PortF: return GPIOF_BASE;
      }
      MJ_ASSERT(false);
      return GPIOA_BASE;
      }());
    reg_in_ = &gpio->IDR;
    mask_ = static_cast<uint32_t>(1 << (static_cast<uint32_t>(pin) & 0xf));
  }

  bool read() {
    return (*reg_in_ & mask_) != 0;
  }

 private:
  volatile uint32_t* reg_in_ = nullptr;
  uint32_t mask_ = 0;
};
}

class BldcServo::Impl {
 public:
  Impl(micro::PersistentConfig* persistent_config,
       micro::TelemetryManager* telemetry_manager,
       MillisecondTimer* millisecond_timer,
       PositionSensor* position_sensor,
       MotorDriver* motor_driver,
       const Options& options)
      : options_(options),
        ms_timer_(millisecond_timer),
        position_sensor_(position_sensor),
        motor_driver_(motor_driver),
        pwm1_(options.pwm1),
        pwm2_(options.pwm2),
        pwm3_(options.pwm3),
        monitor1_(options.pwm1),
        monitor2_(options.pwm2),
        monitor3_(options.pwm3),
        current1_(options.current1),
        current2_(options.current2),
        vsense_(options.vsense),
        vsense_sqr_(FindSqr(options.vsense)),
        tsense_(options.tsense),
        tsense_sqr_(FindSqr(options.tsense)),
        debug_out_(options.debug_out),
        debug_out2_(options.debug_out2),
        debug_serial_([&]() {
            Stm32Serial::Options d_options;
            d_options.tx = options.debug_uart_out;
            d_options.baud_rate = 5450000;
            return d_options;
          }()) {

    clock_.store(0);

    persistent_config->Register("motor", &motor_,
                                std::bind(&Impl::UpdateConfig, this));
    persistent_config->Register("servo", &config_,
                                std::bind(&Impl::UpdateConfig, this));
    persistent_config->Register("servopos", &position_config_,
                                std::bind(&Impl::UpdateConfig, this));
    telemetry_manager->Register("servo_stats", &status_);
    telemetry_manager->Register("servo_cmd", &telemetry_data_);
    telemetry_manager->Register("servo_control", &control_);

    UpdateConfig();

    MJ_ASSERT(!g_impl_);
    g_impl_ = this;
  }

  void Start() {
    ConfigureADC();
    ConfigurePwmTimer();

#if defined(TARGET_STM32F4)
    if (options_.debug_uart_out != NC) {
      const auto uart = pinmap_peripheral(
          options_.debug_uart_out, PinMap_UART_TX);
      debug_uart_ = reinterpret_cast<USART_TypeDef*>(uart);
      auto dma_pair = HardwareUart::MakeDma(
          static_cast<UARTName>(uart));
      debug_uart_dma_tx_ = dma_pair.tx;

      debug_uart_dma_tx_.stream->PAR =
          reinterpret_cast<uint32_t>(&(debug_uart_->DR));
      debug_uart_dma_tx_.stream->CR =
          debug_uart_dma_tx_.channel |
          DMA_SxCR_MINC |
          DMA_MEMORY_TO_PERIPH;
    }
#elif defined(TARGET_STM32G4)
#else
#error "Unknown target"
#endif
  }

  ~Impl() {
    g_impl_ = nullptr;
  }

  void Command(const CommandData& data) {
    MJ_ASSERT(data.mode != kFault);
    MJ_ASSERT(data.mode != kEnabling);
    MJ_ASSERT(data.mode != kCalibrating);
    MJ_ASSERT(data.mode != kCalibrationComplete);

    // Actually setting values will happen in the interrupt routine,
    // so we need to update this atomically.
    CommandData* next = next_data_;
    *next = data;

    // If we have a case where the position is left unspecified, but
    // we have a velocity and stop condition, then we pick the sign of
    // the velocity so that we actually move.
    if (std::isnan(next->position) &&
        std::isfinite(next->stop_position) &&
        std::isfinite(next->velocity) &&
        next->velocity != 0.0f) {
      next->velocity = std::abs(next->velocity) *
          ((next->stop_position > status_.unwrapped_position) ?
           1.0f : -1.0f);
    }

    if (next->timeout_s == 0.0f) {
      next->timeout_s = config_.default_timeout_s;
    }

    telemetry_data_ = *next;

    std::swap(current_data_, next_data_);
  }

  const Status& status() const { return status_; }

  const Config& config() const { return config_; }

  const Motor& motor() const { return motor_; }

  bool is_torque_constant_configured() const {
    return motor_.v_per_hz != 0.0f;
  }

  void UpdateConfig() {
    // I have no idea why the second kPi is necessary here.  All my
    // torque measurements just appear to be off by about a factor of
    // pi.
    torque_constant_ =
        is_torque_constant_configured() ?
        motor_.v_per_hz / (2.0f * kPi) * kPi :
        kDefaultTorqueConstant;

    position_constant_ =
        k2Pi / 65536.0f * motor_.poles / 2.0f;

    adc_scale_ = 3.3f / (4096.0f * MOTEUS_CURRENT_SENSE_OHM * config_.i_gain);
  }

  void PollMillisecond() {
    volatile Mode* mode_volatile = &status_.mode;
    Mode mode = *mode_volatile;
    if (mode == kEnabling) {
      motor_driver_->Enable(true);
      *mode_volatile = kCalibrating;
    }
    startup_count_++;
  }

  uint32_t clock() const {
    return clock_.load();
  }

 private:
  void ConfigurePwmTimer() {
    const auto pwm1_timer = pinmap_peripheral(options_.pwm1, PinMap_PWM);
    const auto pwm2_timer = pinmap_peripheral(options_.pwm2, PinMap_PWM);
    const auto pwm3_timer = pinmap_peripheral(options_.pwm3, PinMap_PWM);

    // All three must be the same and be valid.
    MJ_ASSERT(pwm1_timer != 0 &&
              pwm1_timer == pwm2_timer &&
              pwm2_timer == pwm3_timer);
    timer_ = reinterpret_cast<TIM_TypeDef*>(pwm1_timer);
    timer_sr_ = &timer_->SR;
    timer_cr1_ = &timer_->CR1;


    pwm1_ccr_ = FindCcr(timer_, options_.pwm1);
    pwm2_ccr_ = FindCcr(timer_, options_.pwm2);
    pwm3_ccr_ = FindCcr(timer_, options_.pwm3);


    // Enable the update interrupt.
    timer_->DIER = TIM_DIER_UIE;

    // Enable the update interrupt.
    timer_->CR1 =
        // Center-aligned mode 2.  The counter counts up and down
        // alternatively.  Output compare interrupt flags of channels
        // configured in output are set only when the counter is
        // counting up.
        (2 << TIM_CR1_CMS_Pos) |

        // ARR register is buffered.
        TIM_CR1_ARPE;

    // Update once per up/down of the counter.
    timer_->RCR |= 0x01;

    // Set up PWM.

    timer_->PSC = 0; // No prescaler.
    timer_->ARR = kPwmCounts;

    // NOTE: We don't use micro::CallbackTable here because we need the
    // absolute minimum latency possible.
    const auto irqn = FindUpdateIrq(timer_);
    NVIC_SetVector(irqn, reinterpret_cast<uint32_t>(&Impl::GlobalInterrupt));
    HAL_NVIC_SetPriority(irqn, 0, 0);
    NVIC_EnableIRQ(irqn);

    // Reinitialize the counter and update all registers.
    timer_->EGR |= TIM_EGR_UG;

    // Finally, enable the timer.
    timer_->CR1 |= TIM_CR1_CEN;
  }

  void ConfigureADC() {
#if defined(TARGET_STM32F4)
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC2_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();

    // Triple mode: Regular simultaneous mode only.
    ADC->CCR = (0x16 << ADC_CCR_MULTI_Pos);

    // Turn on all the converters.
    ADC1->CR2 = ADC_CR2_ADON;
    ADC2->CR2 = ADC_CR2_ADON;
    ADC3->CR2 = ADC_CR2_ADON;

    // We rely on the AnalogIn members to configure the pins as
    // inputs.
    ADC1->SQR3 = FindSqr(options_.current1);
    ADC2->SQR3 = FindSqr(options_.current2);
    ADC3->SQR3 = vsense_sqr_;

    MJ_ASSERT(reinterpret_cast<uint32_t>(ADC1) ==
                pinmap_peripheral(options_.current1, PinMap_ADC));
    MJ_ASSERT(reinterpret_cast<uint32_t>(ADC2) ==
                pinmap_peripheral(options_.current2, PinMap_ADC));
    MJ_ASSERT(reinterpret_cast<uint32_t>(ADC3) ==
                pinmap_peripheral(options_.vsense, PinMap_ADC));

    constexpr uint16_t kCycleMap[] = {
      3, 15, 28, 56, 84, 112, 144, 480,
    };

    // Set sample times to the same thing across the board
    const uint32_t cycles = MapConfig(kCycleMap, config_.adc_cycles);
    const uint32_t all_cycles =
        (cycles << 0) |
        (cycles << 3) |
        (cycles << 6) |
        (cycles << 9) |
        (cycles << 12) |
        (cycles << 15) |
        (cycles << 18) |
        (cycles << 21) |
        (cycles << 24);
    ADC1->SMPR1 = all_cycles;
    ADC1->SMPR2 = all_cycles;
    ADC2->SMPR1 = all_cycles;
    ADC2->SMPR2 = all_cycles;
    ADC3->SMPR1 = all_cycles;
    ADC3->SMPR2 = all_cycles;
#elif defined(TARGET_STM32G4)
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC345_CLK_ENABLE();

    auto disable_adc = [](auto* adc) {
      if (adc->CR & ADC_CR_ADEN) {
        adc->CR |= ADC_CR_ADDIS;
        while (adc->CR & ADC_CR_ADEN);
      }
    };

    // First, we have to disable everything to ensure we are in a
    // known state.
    disable_adc(ADC1);
    disable_adc(ADC2);
    disable_adc(ADC5);

    ADC12_COMMON->CCR =
        (4 << ADC_CCR_PRESC_Pos);  // Prescaler /8
    ADC345_COMMON->CCR =
        (4 << ADC_CCR_PRESC_Pos);  // Prescaler /8

    // 20.4.6: ADC Deep power-down mode startup procedure
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC2->CR &= ~ADC_CR_DEEPPWD;
    ADC5->CR &= ~ADC_CR_DEEPPWD;

    ADC1->CR |= ADC_CR_ADVREGEN;
    ADC2->CR |= ADC_CR_ADVREGEN;
    ADC5->CR |= ADC_CR_ADVREGEN;

    // tADCREG_S = 20us per STM32G474xB datasheet
    ms_timer_->wait_us(20);

    // 20.4.8: Calibration
    ADC1->CR |= ADC_CR_ADCAL;
    ADC2->CR |= ADC_CR_ADCAL;
    ADC5->CR |= ADC_CR_ADCAL;

    while ((ADC1->CR & ADC_CR_ADCAL) ||
           (ADC2->CR & ADC_CR_ADCAL) ||
           (ADC5->CR & ADC_CR_ADCAL));

    ms_timer_->wait_us(1);

    // 20.4.9: Software procedure to enable the ADC
    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC2->ISR |= ADC_ISR_ADRDY;
    ADC5->ISR |= ADC_ISR_ADRDY;

    ADC1->CR |= ADC_CR_ADEN;
    ADC2->CR |= ADC_CR_ADEN;
    ADC5->CR |= ADC_CR_ADEN;

    while (!(ADC1->ISR & ADC_ISR_ADRDY) ||
           !(ADC2->ISR & ADC_ISR_ADRDY) ||
           !(ADC5->ISR & ADC_ISR_ADRDY));

    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC2->ISR |= ADC_ISR_ADRDY;
    ADC5->ISR |= ADC_ISR_ADRDY;

    ADC1->SQR1 =
        (0 << ADC_SQR1_L_Pos) |  // length 1
        FindSqr(options_.current1) << ADC_SQR1_SQ1_Pos;
    ADC2->SQR1 =
        (0 << ADC_SQR1_L_Pos) |  // length 1
        FindSqr(options_.current2) << ADC_SQR1_SQ1_Pos;
    ADC5->SQR1 =
        (0 << ADC_SQR1_L_Pos) |  // length 1
        vsense_sqr_ << ADC_SQR1_SQ1_Pos;

    ADC1->SMPR1 =
        (2 << ADC_SMPR1_SMP0_Pos);  // 12.5 ADC Cycles
    ADC2->SMPR1 =
        (2 << ADC_SMPR1_SMP0_Pos);  // 12.5 ADC Cycles
    ADC5->SMPR1 =
        (2 << ADC_SMPR1_SMP0_Pos);  // 12.5 ADC Cycles

#else
#error "Unknown target"
#endif

  }

  // CALLED IN INTERRUPT CONTEXT.
  static void GlobalInterrupt() {
    g_impl_->ISR_HandleTimer();
  }

  // CALLED IN INTERRUPT CONTEXT.
  void ISR_HandleTimer() __attribute__((always_inline)) {
    // From here, until when we finish sampling the ADC has a critical
    // speed requirement.  Any extra cycles will result in a lower
    // maximal duty cycle of the controller.  Thus there are lots of
    // micro-optimizations to try and speed things up by little bits.
    if ((*timer_sr_ & TIM_SR_UIF) &&
        (*timer_cr1_ & TIM_CR1_DIR)) {
      ISR_DoTimer();
    }

    // Reset the status register.
    timer_->SR = 0x00;
  }

  void ISR_DoTimer() __attribute__((always_inline)) {
    // We start our conversion here so that it can work while we get
    // ready.  This means we will throw away the result if our control
    // timer says it isn't our turn yet, but that is a relatively
    // minor waste.
#if defined(TARGET_STM32F4)
    *g_adc1_cr2 |= ADC_CR2_SWSTART;
#elif defined(TARGET_STM32G4)
    ADC1->CR |= ADC_CR_ADSTART;
    ADC2->CR |= ADC_CR_ADSTART;
    ADC5->CR |= ADC_CR_ADSTART;
#else
#error "Unknown target"
#endif

    phase_ = (phase_ + 1) % kInterruptDivisor;
    if (phase_ != 0) { return; }

    // No matter what mode we are in, always sample our ADC and
    // position sensors.
    ISR_DoSense();

    SinCos sin_cos{status_.electrical_theta};

    ISR_CalculateCurrentState(sin_cos);

    ISR_DoControl(sin_cos);

    ISR_MaybeEmitDebug();
    clock_++;

    debug_out_ = 0;
  }

  void ISR_DoSense() __attribute__((always_inline)) {
    // Wait for conversion to complete.
    WaitForAdc(ADC1);
#if defined(TARGET_STM32F4)
    WaitForAdc(ADC2);
#endif

    // We are now out of the most time critical portion of the ISR,
    // although it is still all pretty time critical since it runs
    // at 40kHz.  But time spent until now actually limits the
    // maximum duty cycle we can achieve, whereas time spent below
    // just eats cycles the rest of the code could be using.

    // Check to see if any motor outputs are now high.  If so, fault,
    // because we have exceeded the maximum duty cycle we can achieve
    // while still sampling current correctly.
    if (status_.mode != kFault &&
        (monitor1_.read() ||
         monitor2_.read() ||
         monitor3_.read())) {
      status_.mode = kFault;
      status_.fault = errc::kPwmCycleOverrun;
    }

    debug_out_ = 1;

    if (current_data_->rezero_position) {
      status_.position_to_set = *current_data_->rezero_position;
      current_data_->rezero_position = {};
    }

    if (!std::isfinite(current_data_->timeout_s) ||
        current_data_->timeout_s != 0.0f) {
      status_.timeout_s = current_data_->timeout_s;
      current_data_->timeout_s = 0.0;
    }

    uint32_t adc1 = ADC1->DR;
    uint32_t adc2 = ADC2->DR;
#if defined(TARGET_STM32F4)
    uint32_t adc3 = ADC3->DR;
#elif defined(TARGET_STM32G4)
    WaitForAdc(ADC5);
    uint32_t adc3 = ADC5->DR;
#else
#error "Unknown target"
#endif

    // Start sampling the temperature.
#if defined(TARGET_STM32F4)
    ADC3->SQR3 = tsense_sqr_;
    ADC3->CR2 |= ADC_CR2_SWSTART;
#elif defined(TARGET_STM32G4)
    ADC5->SQR1 =
        (0 << ADC_SQR1_L_Pos) |  // length 1
        tsense_sqr_ << ADC_SQR1_SQ1_Pos;
    ADC5->CR |= ADC_CR_ADSTART;
#else
#error "Unknown target"
#endif

    status_.adc1_raw = adc1;
    status_.adc2_raw = adc2;
    status_.adc3_raw = adc3;

    // Sample the position.
    const uint16_t old_position = status_.position;

    debug_out2_ = 1;
    status_.position_raw = position_sensor_->Sample();

    debug_out2_ = 0;
    status_.position =
        (motor_.invert ? (65535 - status_.position_raw) : status_.position_raw);

    // The temperature sensing should be done by now, but just double
    // check.
#if defined(TARGET_STM32F4)
    WaitForAdc(ADC3);
    status_.fet_temp_raw = ADC3->DR;
    // Set ADC3 back to the sense resistor.
    ADC3->SQR3 = vsense_sqr_;

#elif defined(TARGET_STM32G4)
    WaitForAdc(ADC5);
    status_.fet_temp_raw = ADC5->DR;
    ADC5->SQR1 =
        (0 << ADC_SQR1_L_Pos) |  // length 1
        (vsense_sqr_ << ADC_SQR1_SQ1_Pos);
#else
#error "Unknown target"
#endif

    // Kick off a conversion just to get the FET temp out of the system.
#if defined(TARGET_STM32F4)
    ADC3->CR2 |= ADC_CR2_SWSTART;
#elif defined(TARGET_STM32G4)
    ADC5->CR |= ADC_CR_ADSTART;
#else
#error "Unknown target"
#endif

    {
      constexpr int adc_max = 4096;
      constexpr size_t size_thermistor_table =
          sizeof(g_thermistor_lookup) / sizeof(*g_thermistor_lookup);
      size_t offset = std::max<size_t>(
          1, std::min<size_t>(
              size_thermistor_table - 2,
              status_.fet_temp_raw * size_thermistor_table / adc_max));
      const int16_t this_value = offset * adc_max / size_thermistor_table;
      const int16_t next_value = (offset + 1) * adc_max / size_thermistor_table;
      const float temp1 = g_thermistor_lookup[offset];
      const float temp2 = g_thermistor_lookup[offset + 1];
      status_.fet_temp_C = temp1 +
          (temp2 - temp1) *
          static_cast<float>(status_.fet_temp_raw - this_value) /
          static_cast<float>(next_value - this_value);
    }

    const int offset_size = motor_.offset.size();
    const int offset_index = status_.position * offset_size / 65536;
    MJ_ASSERT(offset_index >= 0 && offset_index < offset_size);

    status_.electrical_theta =
        WrapZeroToTwoPi(
            position_constant_ * (static_cast<float>(status_.position)) +
            motor_.offset[offset_index]);

    const int16_t delta_position =
        static_cast<int16_t>(status_.position - old_position);
    if ((status_.mode != kStopped && status_.mode != kFault) &&
        std::abs(delta_position) > kMaxPositionDelta) {
      // We probably had an error when reading the position.  We must fault.
      status_.mode = kFault;
      status_.fault = errc::kEncoderFault;
    }

    // While we are in the first calibrating state, our unwrapped
    // position is forced to be within one rotation of 0.  Also, the
    // AS5047 isn't guaranteed to be valid until 10ms after startup.
    if (std::isfinite(status_.position_to_set) && startup_count_.load() > 10) {
      const int16_t zero_position =
          static_cast<int16_t>(
              static_cast<int32_t>(status_.position) +
              motor_.position_offset * (motor_.invert ? -1 : 1));
      const float error = status_.position_to_set -
          zero_position * motor_.unwrapped_position_scale/ 65536.0f;;
      const float integral_offsets =
          std::round(error / motor_.unwrapped_position_scale);
      status_.unwrapped_position_raw =
          zero_position + integral_offsets * 65536.0f;
      status_.position_to_set = std::numeric_limits<float>::quiet_NaN();
    } else {
      status_.unwrapped_position_raw += delta_position;
    }

    {
      velocity_filter_.Add(delta_position * 256);
      constexpr float velocity_scale = 1.0f / (256.0f * 65536.0f);
      status_.velocity =
          static_cast<float>(velocity_filter_.average()) *
          motor_.unwrapped_position_scale * velocity_scale * kRateHz;
    }

    status_.unwrapped_position =
        status_.unwrapped_position_raw * motor_.unwrapped_position_scale *
        (1.0f / 65536.0f);
  }

  void ISR_MaybeEmitDebug() {
    if (!config_.enable_debug) { return; }
#if defined(TARGET_STM32F4)
    if (debug_uart_ == nullptr) { return; }

    debug_buf_[0] = 0x5a;
    debug_buf_[1] = static_cast<uint8_t>(255.0f * status_.electrical_theta / k2Pi);
    debug_buf_[2] = static_cast<int8_t>(status_.pid_q.desired * 2.0f);
    int16_t measured_q_a = static_cast<int16_t>(status_.q_A * 500.0f);
    std::memcpy(&debug_buf_[3], &measured_q_a, sizeof(measured_q_a));
    int16_t measured_pid_q_p = 32767.0f * status_.pid_q.p / 12.0f;
    std::memcpy(&debug_buf_[5], &measured_pid_q_p, sizeof(measured_pid_q_p));
    int16_t measured_pid_q_i = 32767.0f * status_.pid_q.integral / 12.0f;
    std::memcpy(&debug_buf_[7], &measured_pid_q_i, sizeof(measured_pid_q_i));
    int16_t control_q_V = 32767.0f * control_.q_V / 12.0f;
    std::memcpy(&debug_buf_[9], &control_q_V, sizeof(control_q_V));

    debug_buf_[11] = static_cast<int8_t>(127.0f * status_.velocity / 10.0f);
    debug_buf_[12] = static_cast<int8_t>(127.0f * status_.bus_V / 24.0f);

    *debug_uart_dma_tx_.status_clear |= debug_uart_dma_tx_.all_status();
    debug_uart_dma_tx_.stream->NDTR = sizeof(debug_buf_);
    debug_uart_dma_tx_.stream->M0AR = reinterpret_cast<uint32_t>(&debug_buf_);
    debug_uart_dma_tx_.stream->CR |= DMA_SxCR_EN;

    debug_uart_->CR3 |= USART_CR3_DMAT;
#elif defined(TARGET_STM32G4)
#else
#error "Unknown target"
#endif
  }

  // This is called from the ISR.
  void ISR_CalculateCurrentState(const SinCos& sin_cos) {
    status_.cur1_A = (status_.adc1_raw - status_.adc1_offset) * adc_scale_;
    status_.cur2_A = (status_.adc2_raw - status_.adc2_offset) * adc_scale_;
    status_.bus_V = status_.adc3_raw * config_.v_scale_V;

    auto update_filtered_bus_v = [&](float* filtered, float period_s) {
      if (!std::isfinite(*filtered)) {
        *filtered = status_.bus_V;
      } else {
        const float alpha = 1.0 / (kRateHz * period_s);
        *filtered = alpha * status_.bus_V + (1.0f - alpha) * *filtered;
      }
    };

    update_filtered_bus_v(&status_.filt_bus_V, 0.5f);
    update_filtered_bus_v(&status_.filt_1ms_bus_V, 0.001f);

    DqTransform dq{sin_cos,
          status_.cur1_A,
          0.0f - (status_.cur1_A + status_.cur2_A),
          status_.cur2_A
          };
    status_.d_A = dq.d;
    status_.q_A = dq.q;
    status_.torque_Nm = torque_on() ? (
        status_.q_A * torque_constant_ /
        motor_.unwrapped_position_scale) : 0.0f;
  }

  bool torque_on() const {
    switch (status_.mode) {
      case kNumModes: {
        MJ_ASSERT(false);
        return false;
      }
      case kFault:
      case kCalibrating:
      case kCalibrationComplete:
      case kEnabling:
      case kStopped: {
        return false;
      }
      case kPwm:
      case kVoltage:
      case kVoltageFoc:
      case kCurrent:
      case kPosition:
      case kPositionTimeout:
      case kZeroVelocity: {
        return true;
      }
    }
    return false;
  }

  void ISR_MaybeChangeMode(CommandData* data) {
    // We are requesting a different mode than we are in now.  Do our
    // best to advance if possible.
    switch (data->mode) {
      case kNumModes:
      case kFault:
      case kCalibrating:
      case kCalibrationComplete: {
        // These should not be possible.
        MJ_ASSERT(false);
        return;
      }
      case kStopped: {
        // It is always valid to enter stopped mode.
        status_.mode = kStopped;
        return;
      }
      case kEnabling: {
        // We can never change out from enabling in ISR context.
        return;
      }
      case kPwm:
      case kVoltage:
      case kVoltageFoc:
      case kCurrent:
      case kPosition:
      case kPositionTimeout:
      case kZeroVelocity: {
        switch (status_.mode) {
          case kNumModes: {
            MJ_ASSERT(false);
            return;
          }
          case kFault: {
            // We cannot leave a fault state directly into an active state.
            return;
          }
          case kStopped: {
            // From a stopped state, we first have to enter the
            // calibrating state.
            ISR_StartCalibrating();
            return;
          }
          case kEnabling:
          case kCalibrating: {
            // We can only leave this state when calibration is
            // complete.
            return;
          }
          case kCalibrationComplete:
          case kPwm:
          case kVoltage:
          case kVoltageFoc:
          case kCurrent:
          case kPosition:
          case kZeroVelocity: {
            // Yep, we can do this.
            status_.mode = data->mode;

            // Start from scratch if we are in a new mode.
            ISR_ClearPid(kAlwaysClear);

            return;
          }
          case kPositionTimeout: {
            // We cannot leave this mode except through a stop.
            return;
          }
        }
      }
    }
  }

  void ISR_StartCalibrating() {
    status_.mode = kEnabling;

    // The main context will set our state to kCalibrating when the
    // motor driver is fully enabled.

    (*pwm1_ccr_) = 0;
    (*pwm2_ccr_) = 0;
    (*pwm3_ccr_) = 0;

    // Power should already be false for any state we could possibly
    // be in, but lets just be certain.
    motor_driver_->Power(false);

    calibrate_adc1_ = 0;
    calibrate_adc2_ = 0;
    calibrate_count_ = 0;
  }

  enum ClearMode {
    kClearIfMode,
    kAlwaysClear,
  };

  void ISR_ClearPid(ClearMode force_clear) {
    const bool current_pid_active = [&]() {
      switch (status_.mode) {
        case kNumModes:
        case kStopped:
        case kFault:
        case kEnabling:
        case kCalibrating:
        case kCalibrationComplete:
        case kPwm:
        case kVoltage:
        case kVoltageFoc:
          return false;
        case kCurrent:
        case kPosition:
        case kPositionTimeout:
        case kZeroVelocity:
          return true;
      }
      return false;
    }();

    if (!current_pid_active || force_clear == kAlwaysClear) {
      status_.pid_d.Clear();
      status_.pid_q.Clear();

      // We always want to start from 0 current when initiating
      // current control of some form.
      status_.pid_d.desired = 0.0f;
      status_.pid_q.desired = 0.0f;
    }

    const bool position_pid_active = [&]() {
      switch (status_.mode) {
        case kNumModes:
        case kStopped:
        case kFault:
        case kEnabling:
        case kCalibrating:
        case kCalibrationComplete:
        case kPwm:
        case kVoltage:
        case kVoltageFoc:
        case kCurrent:
          return false;
        case kPosition:
        case kPositionTimeout:
        case kZeroVelocity:
          return true;
      }
      return false;
    }();

    if (!position_pid_active || force_clear == kAlwaysClear) {
      status_.pid_position.Clear();
      status_.control_position = std::numeric_limits<float>::quiet_NaN();
    }
  }

  void ISR_DoControl(const SinCos& sin_cos) {
    // current_data_ is volatile, so read it out now, and operate on
    // the pointer for the rest of the routine.
    CommandData* data = current_data_;

    control_.Clear();

    if (data->set_position) {
      status_.unwrapped_position_raw =
          static_cast<int32_t>(*data->set_position * 65536.0f);
      data->set_position = {};
    }

    if (std::isfinite(status_.timeout_s) && status_.timeout_s > 0.0f) {
      status_.timeout_s = std::max(0.0f, status_.timeout_s - kPeriodS);
    }

    // See if we need to update our current mode.
    if (data->mode != status_.mode) {
      ISR_MaybeChangeMode(data);
    }

    // Handle our persistent fault conditions.
    if (status_.mode != kStopped && status_.mode != kFault) {
      if (motor_driver_->fault()) {
        status_.mode = kFault;
        status_.fault = errc::kMotorDriverFault;
      }
      if (status_.bus_V > config_.max_voltage) {
        status_.mode = kFault;
        status_.fault = errc::kOverVoltage;
      }
      if (status_.fet_temp_C > config_.max_temperature) {
        status_.mode = kFault;
        status_.fault = errc::kOverTemperature;
      }
    }

    if (status_.mode == kPosition &&
        std::isfinite(status_.timeout_s) &&
        status_.timeout_s <= 0.0f) {
      status_.mode = kPositionTimeout;
    }

    // Ensure unused PID controllers have zerod state.
    ISR_ClearPid(kClearIfMode);

    if (status_.mode != kFault) {
      status_.fault = errc::kSuccess;
    }

    switch (status_.mode) {
      case kNumModes:
      case kStopped: {
        ISR_DoStopped();
        break;
      }
      case kFault: {
        ISR_DoFault();
        break;
      }
      case kEnabling: {
        break;
      }
      case kCalibrating: {
        ISR_DoCalibrating();
        break;
      }
      case kCalibrationComplete: {
        break;
      }
      case kPwm: {
        ISR_DoPwmControl(data->pwm);
        break;
      }
      case kVoltage: {
        ISR_DoVoltageControl(data->phase_v);
        break;
      }
      case kVoltageFoc: {
        ISR_DoVoltageFOC(data->theta, data->voltage);
        break;
      }
      case kCurrent: {
        ISR_DoCurrent(sin_cos, data->i_d_A, data->i_q_A);
        break;
      }
      case kPosition: {
        ISR_DoPosition(sin_cos, data);
        break;
      }
      case kPositionTimeout:
      case kZeroVelocity: {
        ISR_DoZeroVelocity(sin_cos, data);
        break;
      }
    }
  }

  void ISR_DoStopped() {
    motor_driver_->Enable(false);
    motor_driver_->Power(false);
    *pwm1_ccr_ = 0;
    *pwm2_ccr_ = 0;
    *pwm3_ccr_ = 0;
  }

  void ISR_DoFault() {
    motor_driver_->Power(false);
    *pwm1_ccr_ = 0;
    *pwm2_ccr_ = 0;
    *pwm3_ccr_ = 0;
  }

  void ISR_DoCalibrating() {
    calibrate_adc1_ += status_.adc1_raw;
    calibrate_adc2_ += status_.adc2_raw;
    calibrate_count_++;

    if (calibrate_count_ < kCalibrateCount) {
      return;
    }

    const uint16_t new_adc1_offset = calibrate_adc1_ / kCalibrateCount;
    const uint16_t new_adc2_offset = calibrate_adc2_ / kCalibrateCount;

    if (std::abs(static_cast<int>(new_adc1_offset) - 2048) > 200 ||
        std::abs(static_cast<int>(new_adc2_offset) - 2048) > 200) {
      // Error calibrating.  Just fault out.
      status_.mode = kFault;
      status_.fault = errc::kCalibrationFault;
      return;
    }

    status_.adc1_offset = new_adc1_offset;
    status_.adc2_offset = new_adc2_offset;
    status_.mode = kCalibrationComplete;
  }

  void ISR_DoPwmControl(const Vec3& pwm) {
    control_.pwm.a = LimitPwm(pwm.a);
    control_.pwm.b = LimitPwm(pwm.b);
    control_.pwm.c = LimitPwm(pwm.c);

    const uint16_t pwm1 = static_cast<uint16_t>(control_.pwm.a * kPwmCounts);
    const uint16_t pwm2 = static_cast<uint16_t>(control_.pwm.b * kPwmCounts);
    const uint16_t pwm3 = static_cast<uint16_t>(control_.pwm.c * kPwmCounts);

    *pwm1_ccr_ = pwm1;
    *pwm2_ccr_ = pwm3;
    *pwm3_ccr_ = pwm2;

    motor_driver_->Power(true);
  }

  void ISR_DoVoltageControl(const Vec3& voltage) {
    control_.voltage = voltage;

    auto voltage_to_pwm = [this](float v) {
      return 0.5f + 2.0f * v / status_.filt_bus_V;
    };

    ISR_DoPwmControl(Vec3{
        voltage_to_pwm(voltage.a),
            voltage_to_pwm(voltage.b),
            voltage_to_pwm(voltage.c)});
  }

  void ISR_DoVoltageFOC(float theta, float voltage) {
    float max_voltage = 0.5f * (0.5f - kMinPwm) * status_.filt_bus_V;
    SinCos sc(WrapZeroToTwoPi(theta));
    InverseDqTransform idt(sc, Limit(voltage, -max_voltage, max_voltage), 0);
    ISR_DoVoltageControl(Vec3{idt.a, idt.b, idt.c});
  }

  void ISR_DoCurrent(const SinCos& sin_cos, float i_d_A, float i_q_A_in) {
    if (motor_.poles == 0) {
      // We aren't configured yet.
      status_.mode = kFault;
      status_.fault = errc::kMotorNotConfigured;
      return;
    }

    auto limit_q_current = [&](float in) {
      if (status_.unwrapped_position > position_config_.position_max &&
          in > 0.0f) {
        // We derate the request in the direction that moves it
        // further outside the position limits.  This is mostly useful
        // when feedforward is applied, as otherwise, the position
        // limits could easily be exceeded.  Without feedforward, we
        // shouldn't really be trying to push outside the limits
        // anyhow.
        return in *
            std::max(0.0f,
                     1.0f - (status_.unwrapped_position -
                             position_config_.position_max) /
                     config_.position_derate);
      }
      if (status_.unwrapped_position < position_config_.position_min &&
          in < 0.0f) {
        return in *
            std::max(0.0f,
                     1.0f - (position_config_.position_min -
                             status_.unwrapped_position) /
                     config_.position_derate);
      }

      return in;
    };

    const float i_q_A = limit_q_current(i_q_A_in);

    control_.i_d_A = i_d_A;
    control_.i_q_A = i_q_A;

    control_.d_V =
        (config_.feedforward_scale * i_d_A * motor_.resistance_ohm) +
        pid_d_.Apply(status_.d_A, i_d_A, 0.0f, 0.0f, kRateHz);

    control_.q_V =
        (config_.feedforward_scale * (
            i_q_A * motor_.resistance_ohm -
            status_.velocity * motor_.v_per_hz /
            motor_.unwrapped_position_scale)) +
        pid_q_.Apply(status_.q_A, i_q_A, 0.0f, 0.0f, kRateHz);

    float max_voltage = 0.5f * (0.5f - kMinPwm) * status_.filt_bus_V;
    auto limit_v = [&](float in) {
      return Limit(in, -max_voltage, max_voltage);
    };
    InverseDqTransform idt(sin_cos, limit_v(control_.d_V),
                           limit_v(control_.q_V));

    ISR_DoVoltageControl(Vec3{idt.a, idt.b, idt.c});
  }

  void ISR_DoZeroVelocity(const SinCos& sin_cos, CommandData* data) {
    mjlib::base::PID::ApplyOptions apply_options;
    apply_options.kp_scale = 0.0;
    apply_options.kd_scale = 1.0;

    ISR_DoPositionCommon(sin_cos, data,
                         apply_options, config_.timeout_max_torque_Nm,
                         0.0f, 0.0f);
  }

  void ISR_DoPosition(const SinCos& sin_cos, CommandData* data) {
    mjlib::base::PID::ApplyOptions apply_options;
    apply_options.kp_scale = data->kp_scale;
    apply_options.kd_scale = data->kd_scale;

    ISR_DoPositionCommon(sin_cos, data, apply_options, data->max_torque_Nm,
                         data->feedforward_Nm, data->velocity);
  }

  void ISR_DoPositionCommon(
      const SinCos& sin_cos, CommandData* data,
      const mjlib::base::PID::ApplyOptions& pid_options,
      float max_torque_Nm,
      float feedforward_Nm,
      float velocity) {
    if (!std::isnan(data->position)) {
      status_.control_position = data->position;
      data->position = std::numeric_limits<float>::quiet_NaN();
    } else if (std::isnan(status_.control_position)) {
      status_.control_position = status_.unwrapped_position;
    }

    auto velocity_command = velocity;

    const auto old_position = status_.control_position;
    status_.control_position =
        Limit(status_.control_position + velocity_command / kRateHz,
              position_config_.position_min,
              position_config_.position_max);
    if (std::isfinite(data->stop_position)) {
      if ((status_.control_position -
           data->stop_position) * velocity_command > 0.0f) {
        // We are moving away from the stop position.  Force it to be there.
        status_.control_position = data->stop_position;
      }
    }
    if (status_.control_position == old_position) {
      // We have hit a limit.  Assume a velocity of 0.
      velocity_command = 0.0f;
    }

    const float measured_velocity = Threshold(
        status_.velocity, -config_.velocity_threshold,
        config_.velocity_threshold);

    const float unlimited_torque_Nm =
        pid_position_.Apply(status_.unwrapped_position,
                            status_.control_position,
                            measured_velocity, velocity_command,
                            kRateHz,
                            pid_options) +
        feedforward_Nm;

    const float limited_torque_Nm =
        Limit(unlimited_torque_Nm, -max_torque_Nm, max_torque_Nm);

    control_.torque_Nm = limited_torque_Nm;

    const float limited_q_A =
        limited_torque_Nm * motor_.unwrapped_position_scale /
        torque_constant_;

    const float q_A =
        is_torque_constant_configured() ?
        limited_q_A :
        Limit(limited_q_A, -kMaxUnconfiguredCurrent, kMaxUnconfiguredCurrent);

    const float d_A = [&]() {
      if (config_.flux_brake_min_voltage <= 0.0) {
        return 0.0f;
      }

      const auto error = (
          status_.filt_1ms_bus_V - config_.flux_brake_min_voltage);

      if (error <= 0.0f) {
        return 0.0f;
      }

      return (error / config_.flux_brake_resistance_ohm);
    }();

    ISR_DoCurrent(sin_cos, d_A, q_A);
  }

  float LimitPwm(float in) {
    // We can't go full duty cycle or we wouldn't have time to sample
    // the current.
    return Limit(in, kMinPwm, kMaxPwm);
  }

  const Options options_;
  MillisecondTimer* const ms_timer_;
  PositionSensor* const position_sensor_;
  MotorDriver* const motor_driver_;

  Motor motor_;
  Config config_;
  PositionConfig position_config_;
  TIM_TypeDef* timer_ = nullptr;
  volatile uint32_t* timer_sr_ = nullptr;
  volatile uint32_t* timer_cr1_ = nullptr;
  ADC_TypeDef* const adc1_ = ADC1;
  ADC_TypeDef* const adc2_ = ADC2;
  ADC_TypeDef* const adc3_ = ADC3;
#if defined(TARGET_STM32G4)
  ADC_TypeDef* const adc5_ = ADC5;
  ADC_Common_TypeDef* const adc12_common_ = ADC12_COMMON;
#endif

  // We create these to initialize our pins as output and PWM mode,
  // but otherwise don't use them.
  PwmOut pwm1_;
  PwmOut pwm2_;
  PwmOut pwm3_;

  DigitalMonitor monitor1_;
  DigitalMonitor monitor2_;
  DigitalMonitor monitor3_;

  volatile uint32_t* pwm1_ccr_ = nullptr;
  volatile uint32_t* pwm2_ccr_ = nullptr;
  volatile uint32_t* pwm3_ccr_ = nullptr;

  AnalogIn current1_;
  AnalogIn current2_;
  AnalogIn vsense_;
  uint32_t vsense_sqr_ = {};
  AnalogIn tsense_;
  uint32_t tsense_sqr_ = {};

  // This is just for debugging.
  DigitalOut debug_out_;
  DigitalOut debug_out2_;

  int32_t phase_ = 0;

  CommandData data_buffers_[2] = {};

  // CommandData has its data updated to the ISR by first writing the
  // new command into (*next_data_) and then swapping it with
  // current_data_.
  CommandData* volatile current_data_{&data_buffers_[0]};
  CommandData* volatile next_data_{&data_buffers_[1]};

  // This copy of CommandData exists solely for telemetry, and should
  // never be read by an ISR.
  CommandData telemetry_data_;

  // These values should only be modified from within the ISR.
  mjlib::base::WindowedAverage<int16_t, 128, int32_t> velocity_filter_;
  Status status_;
  Control control_;
  uint32_t calibrate_adc1_ = 0;
  uint32_t calibrate_adc2_ = 0;
  uint16_t calibrate_count_ = 0;

  mjlib::base::PID pid_d_{&config_.pid_dq, &status_.pid_d};
  mjlib::base::PID pid_q_{&config_.pid_dq, &status_.pid_q};
  mjlib::base::PID pid_position_{&config_.pid_position, &status_.pid_position};

  Stm32Serial debug_serial_;
#if defined(TARGET_STM32F4)
  USART_TypeDef* debug_uart_ = nullptr;
  HardwareUart::Dma debug_uart_dma_tx_;
  char debug_buf_[13] = {};
#elif defined(TARGET_STM32G4)
#else
#error "Unknown target"
#endif

  std::atomic<uint32_t> clock_;
  std::atomic<uint32_t> startup_count_{0};

  float torque_constant_ = 0.01f;
  float position_constant_ = 0.0f;
  float adc_scale_ = 0.0f;

  static Impl* g_impl_;
};

BldcServo::Impl* BldcServo::Impl::g_impl_ = nullptr;

BldcServo::BldcServo(micro::Pool* pool,
                     micro::PersistentConfig* persistent_config,
                     micro::TelemetryManager* telemetry_manager,
                     MillisecondTimer* millisecond_timer,
                     PositionSensor* position_sensor,
                     MotorDriver* motor_driver,
                     const Options& options)
    : impl_(pool,
            persistent_config, telemetry_manager,
            millisecond_timer, position_sensor, motor_driver,
            options) {}
BldcServo::~BldcServo() {}

void BldcServo::Start() {
  impl_->Start();
}

void BldcServo::PollMillisecond() {
  impl_->PollMillisecond();
}

void BldcServo::Command(const CommandData& data) {
  impl_->Command(data);
}

const BldcServo::Status& BldcServo::status() const {
  return impl_->status();
}

const BldcServo::Config& BldcServo::config() const {
  return impl_->config();
}

const BldcServo::Motor& BldcServo::motor() const {
  return impl_->motor();
}

uint32_t BldcServo::clock() const {
  return impl_->clock();
}

}
