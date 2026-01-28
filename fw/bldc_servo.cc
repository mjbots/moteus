// Copyright 2023 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include "PeripheralPins.h"

#include "mjlib/base/assert.h"
#include "mjlib/base/windowed_average.h"

#include "fw/bldc_servo_control.h"
#include "fw/bldc_servo_position.h"
#include "fw/foc.h"
#include "fw/moteus_hw.h"
#include "fw/stm32_dma.h"
#include "fw/stm32g4_adc.h"
#include "fw/thermistor.h"
#include "fw/torque_model.h"

#if defined(TARGET_STM32G4)
#include "fw/stm32g4_async_uart.h"
#else
#error "Unknown target"
#endif

#ifdef wait_us
#undef wait_us
#endif

namespace micro = mjlib::micro;

namespace moteus {

namespace {
#if defined(TARGET_STM32G4)
using HardwareUart = Stm32G4AsyncUart;
#else
#error "Unknown target"
#endif


RateConfig MakeRateConfig(int pwm_rate_hz_in) {
  const int board_min_pwm_rate_hz =
      (g_measured_hw_family == 0 &&
       g_measured_hw_rev == 2) ? 60000 :
      15000;
  return RateConfig(pwm_rate_hz_in, board_min_pwm_rate_hz);
}

constexpr int kCalibrateCount = 256;

constexpr int kMaxVelocityFilter = 256;

IRQn_Type FindUpdateIrq(TIM_TypeDef* timer) {
#if defined(TARGET_STM32G4)
  if (timer == TIM2) {
    return TIM2_IRQn;
  } else if (timer == TIM3) {
    return TIM3_IRQn;
  } else if (timer == TIM4) {
    return TIM4_IRQn;
  } else if (timer == TIM5) {
    return TIM5_IRQn;
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

/// Read a digital input, but without configuring it in any way.
class PhaseMonitors {
 public:
  PhaseMonitors(PinName pin1, PinName pin2, PinName pin3) {
    const uint32_t port_index1 = STM_PORT(pin1);
    const uint32_t port_index2 = STM_PORT(pin3);
    const uint32_t port_index3 = STM_PORT(pin3);
    MJ_ASSERT(port_index1 == port_index2);
    MJ_ASSERT(port_index2 == port_index3);

    GPIO_TypeDef* gpio = reinterpret_cast<GPIO_TypeDef*>([&]() {
      switch (port_index1) {
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
    mask_ = static_cast<uint32_t>(1 << (static_cast<uint32_t>(pin1) & 0xf)) |
        static_cast<uint32_t>(1 << (static_cast<uint32_t>(pin2) & 0xf)) |
        static_cast<uint32_t>(1 << (static_cast<uint32_t>(pin3) & 0xf));
  }

  bool read() {
    return (*reg_in_ & mask_) != 0;
  }

 private:
  volatile uint32_t* reg_in_ = nullptr;
  uint32_t mask_ = 0;
};

class ExponentialFilter {
 public:
  ExponentialFilter() {}

  ExponentialFilter(float rate_hz, float period_s)
      : alpha_(1.0f / (rate_hz * period_s)),
        one_minus_alpha_(1.0f - alpha_) {}

  void operator()(float input, float* filtered) {
    if (std::isnan(*filtered)) {
      *filtered = input;
    } else {
      *filtered = alpha_ * input + one_minus_alpha_ * *filtered;
    }
  }

 private:
  float alpha_ = 1.0;
  float one_minus_alpha_ = 0.0;
};
}

class BldcServo::Impl : public BldcServoControl<BldcServo::Impl> {
 public:
  Impl(micro::PersistentConfig* persistent_config,
       micro::TelemetryManager* telemetry_manager,
       MillisecondTimer* millisecond_timer,
       MotorDriver* motor_driver,
       AuxADC* aux_adc,
       AuxPort* aux1_port,
       AuxPort* aux2_port,
       MotorPosition* motor_position,
       const Options& options)
      : options_(options),
        ms_timer_(millisecond_timer),
        motor_driver_(motor_driver),
        aux_adc_(aux_adc),
        aux1_port_(aux1_port),
        aux2_port_(aux2_port),
        motor_position_(motor_position),
        pwm1_(options.pwm1),
        pwm2_(options.pwm2),
        pwm3_(options.pwm3),
        phase_monitors_(options.pwm1, options.pwm2, options.pwm3),
        current1_(options.current1),
        current2_(options.current2),
        current3_(options.current3),
        vsense_(options.vsense),
        vsense_sqr_(FindSqr(options.vsense)),
        tsense_(options.tsense),
        tsense_sqr_(FindSqr(options.tsense)),
        msense_(options.msense),
        msense_sqr_(FindSqr(options.msense)),
        debug_dac_(options.debug_dac),
#ifdef MOTEUS_DEBUG_OUT
        debug_out_(options.debug_out),
#endif
        vsense_adc_scale_(g_hw_pins.vsense_adc_scale) {

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
    ConfigureLPTIM1();
    ConfigureDmaLptimTrigger();
    ConfigurePwmIrq();

    if (options_.debug_uart_out != NC) {
      const auto uart = pinmap_peripheral(
          options_.debug_uart_out, PinMap_UART_TX);
      debug_uart_ = onboard_debug_uart_ =
          reinterpret_cast<USART_TypeDef*>(uart);
    }
  }

  ~Impl() {
    g_impl_ = nullptr;
  }

  void Command(const CommandData& data) {
    if (data.mode == kFault ||
        data.mode == kEnabling ||
        data.mode == kCalibrating ||
        data.mode == kCalibrationComplete) {
      // These are not valid states to command.  Ignore the command
      // entirely.
      return;
    }

    // Actually setting values will happen in the interrupt routine,
    // so we need to update this atomically.
    CommandData* next = next_data_;
    *next = data;

    if (next->timeout_s == 0.0f) {
      next->timeout_s = config_.default_timeout_s;
    }
    if (std::isnan(next->velocity_limit)) {
      next->velocity_limit = config_.default_velocity_limit;
    } else if (next->velocity_limit < 0.0f) {
      next->velocity_limit = std::numeric_limits<float>::quiet_NaN();
    }
    if (std::isnan(next->accel_limit)) {
      next->accel_limit = config_.default_accel_limit;
    } else if (next->accel_limit < 0.0f) {
      next->accel_limit = std::numeric_limits<float>::quiet_NaN();
    }
    // If we are going to limit at all, ensure that we have a velocity
    // limit, and that is is no more than the configured maximum
    // velocity.
    if (!std::isnan(next->velocity_limit) || !std::isnan(next->accel_limit)) {
      if (std::isnan(next->velocity_limit)) {
        next->velocity_limit = config_.max_velocity;
      } else {
        next->velocity_limit =
            std::min(next->velocity_limit, config_.max_velocity);
      }
    }

    // If we have a velocity command and velocity_limit, ensure that
    // the command does not violate the limit.
    if (!std::isnan(next->velocity_limit) &&
        !std::isnan(next->velocity)) {
      next->velocity = Limit(next->velocity,
                             -next->velocity_limit,
                             next->velocity_limit);
    }

    // Transform any position and stop_position command into the
    // relative raw space.
    const auto delta = static_cast<int64_t>(
        motor_position_->absolute_relative_delta.load()) << 32ll;
    if (!std::isnan(next->position)) {
      next->position_relative_raw =
          MotorPosition::FloatToInt(next->position) - delta;
    } else {
      next->position_relative_raw.reset();
    }

    if (!std::isnan(next->stop_position)) {
      next->stop_position_relative_raw =
          MotorPosition::FloatToInt(next->stop_position) - delta;
    }

    // If we have a case where the position is left unspecified, but
    // we have a velocity and stop condition, then we pick the sign of
    // the velocity so that we actually move.
    if (!next->position_relative_raw &&
        !!next->stop_position_relative_raw &&
        !std::isnan(next->velocity) &&
        next->velocity != 0.0f) {

      next->velocity = std::abs(next->velocity) *
          (((*next->stop_position_relative_raw -
             position_.position_relative_raw) > 0) ?
           1.0f : -1.0f);
    }

    telemetry_data_ = *next;

    volatile auto* mode_volatile = &status_.mode;
    volatile auto* fault_volatile = &status_.fault;

    if (!!next->stop_position_relative_raw &&
        (std::isfinite(next->accel_limit) ||
         std::isfinite(next->velocity_limit))) {
      // There is no valid use case for using a stop position along
      // with an acceleration or velocity limit.
      *fault_volatile = errc::kStopPositionDeprecated;
      *mode_volatile = kFault;
    }

    if (config_.bemf_feedforward != 0.0f &&
        !std::isfinite(next->accel_limit) &&
        !config_.bemf_feedforward_override) {
      // We normally don't allow bemf feedforward if an acceleration
      // limit is not applied, as that can easily result in output
      // currents exceeding any configured limits.  Even with limits,
      // if they are non-realistic this can happen, but we're mostly
      // trying to catch gross problems here.
      *fault_volatile = errc::kBemfFeedforwardNoAccelLimit;
      *mode_volatile = kFault;
    }

    // We pre-compute this here to save time in the ISR.
    next->synthetic_theta =
        config_.fixed_voltage_mode ||
        !std::isnan(next->fixed_voltage_override) ||
        !std::isnan(next->fixed_current_override);

    std::swap(current_data_, next_data_);
  }

  const Status& status() const { return status_; }
  const Config& config() const { return config_; }
  const Control& control() const { return control_; }
  const AuxPort::Status& aux1() const { return *aux1_port_->status(); }
  const AuxPort::Status& aux2() const { return *aux2_port_->status(); }
  const MotorPosition::Status& motor_position() const {
    return motor_position_->status();
  }
  MotorPosition::Config* motor_position_config() {
    return motor_position_->config();
  }

  const MotorPosition::Config* motor_position_config() const {
    return motor_position_->config();
  }

  bool is_torque_constant_configured() const {
    return motor_.Kv != 0.0f;
  }

  float current_to_torque(float current) const MOTEUS_CCM_ATTRIBUTE {
    TorqueModel model(torque_constant_,
                      motor_.rotation_current_cutoff_A,
                      motor_.rotation_current_scale,
                      motor_.rotation_torque_scale);
    return model.current_to_torque(current);
  }

  void UpdateConfig() {
    rate_config_ = MakeRateConfig(config_.pwm_rate_hz);
    // Update the saved config to match our limits.
    config_.pwm_rate_hz = rate_config_.pwm_rate_hz;

    flux_brake_min_voltage_ =
        config_.max_voltage - config_.flux_brake_margin_voltage;
    derate_temperature_ =
        config_.fault_temperature - config_.temperature_margin;
    motor_derate_temperature_ =
        config_.motor_fault_temperature - config_.motor_temperature_margin;

    velocity_filter_ = ExponentialFilter(rate_config_.pwm_rate_hz, 0.01f);
    temperature_filter_ = ExponentialFilter(rate_config_.pwm_rate_hz, 0.01f);
    slow_bus_v_filter_ = ExponentialFilter(rate_config_.pwm_rate_hz, 0.5f);
    fast_bus_v_filter_ = ExponentialFilter(rate_config_.pwm_rate_hz, 0.001f);

    // Ensure that our maximum current stays within the range that can
    // be sensed.
    constexpr float kSensableCurrentMargin = 0.95f;
    const float max_sensable_current_A =
        kSensableCurrentMargin *
        motor_driver_->max_sense_V() / config_.current_sense_ohm;
    config_.max_current_A = std::min(config_.max_current_A,
                                     max_sensable_current_A);

    ConfigurePwmTimer();

    // From https://things-in-motion.blogspot.com/2018/12/how-to-estimate-torque-of-bldc-pmsm.html
    constexpr float kTorqueFactor =
        (3.0f / 2.0f) * (1.0f / kSqrt3) * (60.0f / k2Pi);

    torque_constant_ =
        is_torque_constant_configured() ?
        kTorqueFactor / motor_.Kv :
        kDefaultTorqueConstant;
    v_per_hz_ = motor_.Kv == 0.0f ? 0.0f : 0.5f * 60.0f / motor_.Kv;

    adc_scale_ = 3.3f / (4096.0f *
                         config_.current_sense_ohm *
                         motor_driver_->i_gain());

    pwm_derate_ = (static_cast<float>(config_.pwm_rate_hz) / 30000.0f);

    fet_thermistor_.Reset(47000.0f);
    motor_thermistor_.Reset(config_.motor_thermistor_ohm);
    motor_position_->SetRate(rate_config_.period_s);
  }

  void PollMillisecond() {
    volatile auto* mode_volatile = &status_.mode;
    volatile auto* fault_volatile = &status_.fault;
    Mode mode = *mode_volatile;
    if (mode == kEnabling) {
      const auto enable_result = motor_driver_->StartEnable(true);
      switch (enable_result) {
        case MotorDriver::kEnabled: {
          *mode_volatile = kCalibrating;
          break;
        }
        case MotorDriver::kCalibrateFailed: {
          *fault_volatile = errc::kDriverEnableFault;
          *mode_volatile = kFault;
          break;
        }
        case MotorDriver::kDisabled:
        case MotorDriver::kEnabling1:
        case MotorDriver::kEnabling2:
        case MotorDriver::kEnabling3: {
          // Not done yet.
          break;
        }
      }
    }

    // Because the aux ports can be configured after us, we just poll
    // periodically to see if we need to point our debug uart
    // somewhere different.
    auto* desired_debug_uart =
        [&]() {
          if (aux1_port_->debug_uart()) {
            return aux1_port_->debug_uart();
          } else if (aux2_port_->debug_uart()) {
            return aux2_port_->debug_uart();
          } else {
            return onboard_debug_uart_;
          }
        }();
    if (desired_debug_uart != debug_uart_) {
      debug_uart_ = desired_debug_uart;
    }

    if (motor_position_->status().epoch !=
        main_motor_position_epoch_) {
      main_motor_position_epoch_ = motor_position_->status().epoch;
      UpdateConfig();
    }
  }

  void SetOutputPositionNearest(float position) {
    // The required function can only officially be called in an ISR
    // context.  To simplify things, we just disable IRQs to
    // deconflict.
    __disable_irq();
    motor_position_->ISR_SetOutputPositionNearest(position);
    __enable_irq();
  }

  void SetOutputPosition(float position) {
    __disable_irq();
    motor_position_->ISR_SetOutputPosition(position);
    __enable_irq();
  }

  void RequireReindex() {
    __disable_irq();
    motor_position_->ISR_RequireReindex();
    __enable_irq();
  }

  void RecapturePositionVelocity() {
    __disable_irq();

    status_.pid_position.Clear();
    status_.control_position_raw = {};
    status_.control_position = std::numeric_limits<float>::quiet_NaN();
    status_.control_velocity = {};
    status_.control_acceleration = {};

    __enable_irq();
  }

  void Fault(moteus::errc fault_code) {
    __disable_irq();

    if (status_.mode != kFault) {
      status_.mode = kFault;
      status_.fault = fault_code;
    }

    __enable_irq();
  }

 private:
  friend class BldcServoControl<Impl>;

  void ConfigurePwmIrq() {
    // NOTE: We don't use micro::CallbackTable here because we need the
    // absolute minimum latency possible.
    const auto irqn = FindUpdateIrq(timer_);
    NVIC_SetVector(irqn, reinterpret_cast<uint32_t>(&Impl::GlobalInterrupt));
    HAL_NVIC_SetPriority(irqn, 0, 0);

    // Our handler is broken up into two parts.  One which runs on the
    // timer interrupt and is the highest priority.  It makes sure the
    // current is sampled as close to the center of the PWM waveform
    // as possible.
    //
    // Then, once that is done, we trigger the PendSV interrupt to do
    // the remainder of the processing at a lower interrupt priority
    // level.  That way things like soft-GPIO handling interrupts
    // (quadrature, step-dir), can pre-empt the rest.
    NVIC_SetVector(PendSV_IRQn, reinterpret_cast<uint32_t>(&Impl::GlobalPendSv));
    // Set to the lowest priority we are using.
    HAL_NVIC_SetPriority(PendSV_IRQn, 6, 0);

    NVIC_EnableIRQ(PendSV_IRQn);
    pwm_irqn_ = irqn;
    NVIC_EnableIRQ(irqn);
  }

  void ConfigurePwmTimer() {
    // Disable PWM interrupt during reconfiguration to prevent ISR from
    // running while timer/DMA chain is in an inconsistent state.
    // Only do this if interrupts have been configured (pwm_irqn_ is set).
    const bool irq_was_enabled = (pwm_irqn_ != IRQn_Type{}) &&
                                  NVIC_GetEnableIRQ(pwm_irqn_);
    if (irq_was_enabled) {
      NVIC_DisableIRQ(pwm_irqn_);
    }

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

    // Enable DMA request on CC4 event. The DMA will trigger LPTIM1
    // which then triggers all ADCs simultaneously.
    // Using CC4 instead of Update because in center-aligned mode 2 (CMS=2),
    // the CC4 flag is set only when counting down, giving us exactly one
    // DMA trigger per PWM cycle. Update events would trigger twice per cycle.
    timer_->DIER |= TIM_DIER_CC4DE;

    // Update once per up/down of the counter.
    timer_->RCR |= 0x01;

    // Set up PWM.

    timer_->PSC = 0; // No prescaler.
    pwm_counts_ = HAL_RCC_GetPCLK1Freq() * 2 / (2 * rate_config_.pwm_rate_hz);
    timer_->ARR = pwm_counts_;

    // Set CCR4 to trigger at the top of the count (when counting
    // down).  Current sensing requires sampling when low-side
    // switches are ON, which happens at the peak of the PWM cycle
    // (counter near ARR).  We set CCR4 = ARR so the DMA triggers
    // right at the peak when counting down.
    timer_->CCR4 = pwm_counts_;

    // Reinitialize the counter and update all registers.
    timer_->EGR |= TIM_EGR_UG;

    // Finally, enable the timer.
    timer_->CR1 |= TIM_CR1_CEN;

    // Re-enable PWM interrupt now that reconfiguration is complete.
    if (irq_was_enabled) {
      NVIC_EnableIRQ(pwm_irqn_);
    }
  }

  void ConfigureADC() {
    constexpr uint16_t kCycleMap[] = {
      2, 6, 12, 24, 47, 92, 247, 640,
    };

    const uint32_t cur_cycles = MapConfig(kCycleMap, config_.adc_cur_cycles);
    const uint32_t aux_cycles = MapConfig(kCycleMap, config_.adc_aux_cycles);
    auto make_cycles = [](auto v) {
      return
        (v << 0) |
        (v << 3) |
        (v << 6) |
        (v << 9) |
        (v << 12) |
        (v << 15) |
        (v << 18) |
        (v << 21) |
        (v << 24);
    };
    const uint32_t all_cur_cycles = make_cycles(cur_cycles);
    const uint32_t all_aux_cycles = make_cycles(aux_cycles);

    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC345_CLK_ENABLE();

    // First, we have to disable everything to ensure we are in a
    // known state.
    DisableAdc(ADC1);
    DisableAdc(ADC2);
    DisableAdc(ADC3);
    DisableAdc(ADC4);
    DisableAdc(ADC5);

    // Per STM32G4 errata ES0430 section 2.7.11, ADC instances can impact
    // each other's accuracy when conversions are concurrent. To avoid this,
    // all ADCs must use the same clock configuration and be triggered
    // simultaneously by the same timer. We use synchronous AHB/2 mode
    // and ensure each ADC is started in an exact phase relationship to
    // the global cycle counter.
    ADC12_COMMON->CCR =
        (2 << ADC_CCR_CKMODE_Pos) |  // synchronous AHB/2
        (1 << ADC_CCR_DUAL_Pos); // dual mode, regular + injected
    ADC345_COMMON->CCR =
        (2 << ADC_CCR_CKMODE_Pos) |  // synchronous AHB/2
        (1 << ADC_CCR_DUAL_Pos); // dual mode, regular + injected

    constexpr int kAdcPrescale = 2;  // from the CKMODE above

    // Enable all ADCs with LPTIM1 external trigger for synchronized sampling.
    // Per STM32G4 errata ES0430 section 2.7.11, all ADCs must be triggered
    // simultaneously to avoid accuracy issues between ADC instances.
    // LPTIM1 is triggered via DMA from the PWM timer update event.
    EnableAdc(ms_timer_, ADC1, kAdcPrescale, 0, AdcTriggerMode::kLptim1);
    EnableAdc(ms_timer_, ADC2, kAdcPrescale, 0, AdcTriggerMode::kLptim1);
    EnableAdc(ms_timer_, ADC3, kAdcPrescale, 0, AdcTriggerMode::kLptim1);
    EnableAdc(ms_timer_, ADC4, kAdcPrescale, 0, AdcTriggerMode::kLptim1);
    EnableAdc(ms_timer_, ADC5, kAdcPrescale, 0, AdcTriggerMode::kLptim1);

    if (family0_) {
      adc1_sqr_ = ADC1->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          FindSqr(options_.current2) << ADC_SQR1_SQ1_Pos;
      adc2_sqr_ = ADC2->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          FindSqr(options_.current3) << ADC_SQR1_SQ1_Pos;
      adc3_sqr_ = ADC3->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          FindSqr(options_.current1) << ADC_SQR1_SQ1_Pos;
    } else {
      adc1_sqr_ = ADC1->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          FindSqr(options_.current1) << ADC_SQR1_SQ1_Pos;
      adc2_sqr_ = ADC2->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          FindSqr(options_.current2) << ADC_SQR1_SQ1_Pos;
      adc3_sqr_ = ADC3->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          FindSqr(options_.current3) << ADC_SQR1_SQ1_Pos;
    }
    if (family0_rev4_and_older_) {
      // For version <=4, we sample the motor temperature and the
      // battery sense first.
      adc4_sqr_ = ADC4->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          (msense_sqr_ << ADC_SQR1_SQ1_Pos);
      ADC5->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          (vsense_sqr_ << ADC_SQR1_SQ1_Pos);
    } else if (g_measured_hw_family == 0) {
      // For 5+, ADC4 always stays on the battery.
      adc4_sqr_ = ADC4->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          (vsense_sqr_ << ADC_SQR1_SQ1_Pos);
      ADC5->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          (tsense_sqr_ << ADC_SQR1_SQ1_Pos);
    } else if (family1or2or3_) {
      // For family 1, ADC4 always stays on temperature sense.
      adc4_sqr_ = ADC4->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          (tsense_sqr_ << ADC_SQR1_SQ1_Pos);
      ADC5->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          (vsense_sqr_ << ADC_SQR1_SQ1_Pos);
    }

    ADC1->SMPR1 = all_cur_cycles;
    ADC1->SMPR2 = all_cur_cycles;
    ADC2->SMPR1 = all_cur_cycles;
    ADC2->SMPR2 = all_cur_cycles;
    ADC3->SMPR1 = all_cur_cycles;
    ADC3->SMPR2 = all_cur_cycles;

    ADC4->SMPR1 = all_aux_cycles;
    ADC4->SMPR2 = all_aux_cycles;
    ADC5->SMPR1 = all_aux_cycles;
    ADC5->SMPR2 = all_aux_cycles;
  }

  // Get the DMAMUX request input number for the given timer's CH4 event.
  // Using CH4 instead of Update because in center-aligned mode 2, the CC4
  // flag is set only when counting down, giving us exactly one DMA trigger
  // per PWM cycle. Update events would trigger on both edges.
  static uint32_t GetTimerCh4DmamuxInput(TIM_TypeDef* timer) {
    // From STM32G4 reference manual Table 91 (DMAMUX request MUX inputs)
    if (timer == TIM2) return 59;   // TIM2_CH4
    if (timer == TIM3) return 64;   // TIM3_CH4
    if (timer == TIM4) return 70;   // TIM4_CH4
    if (timer == TIM5) return 75;   // TIM5_CH4
    mbed_die();
    return 0;
  }

  void ConfigureLPTIM1() {
    // Configure LPTIM1 to generate a pulse on LPTIM1_OUT when triggered.
    // This pulse triggers all ADCs simultaneously.

    // Configure LPTIM1 clock source using proper HAL method
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPTIM1;
    PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
      return;
    }

    // Reset and enable LPTIM1 peripheral
    __HAL_RCC_LPTIM1_FORCE_RESET();
    __HAL_RCC_LPTIM1_RELEASE_RESET();
    __HAL_RCC_LPTIM1_CLK_ENABLE();

    // Initialize LPTIM1 for ADC triggering
    static LPTIM_HandleTypeDef lptim_adc;
    lptim_adc.Instance = LPTIM1;
    lptim_adc.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    lptim_adc.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV4;
    lptim_adc.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
    lptim_adc.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
    lptim_adc.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
    lptim_adc.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
    lptim_adc.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
    lptim_adc.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;

    if (HAL_LPTIM_Init(&lptim_adc) != HAL_OK) {
      while(true);
    }

    // Configure ARR/CMP values once during initialization and leave enabled
    LPTIM1->CR |= LPTIM_CR_ENABLE;
    LPTIM1->ICR = LPTIM_ICR_ARROKCF;
    LPTIM1->ARR = 4;
    while (!(LPTIM1->ISR & LPTIM_ISR_ARROK));
    LPTIM1->ICR = LPTIM_ICR_CMPOKCF;
    LPTIM1->CMP = 1;
    while (!(LPTIM1->ISR & LPTIM_ISR_CMPOK));
  }

  void ConfigureDmaLptimTrigger() {
    // Configure DMA to transfer SNGSTRT to LPTIM1->CR on each PWM timer
    // update event. This triggers LPTIM1 which then triggers all ADCs.
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    MJ_ASSERT(options_.lptim_trigger_dma != nullptr);
    DMA_Channel_TypeDef* dma = options_.lptim_trigger_dma;
    DMAMUX_Channel_TypeDef* dmamux = Stm32Dma::SelectDmamux(dma);

    // Disable DMA channel before configuration
    dma->CCR = 0;

    // Configure DMAMUX to route PWM timer CH4 event to this DMA channel
    dmamux->CCR = GetTimerCh4DmamuxInput(timer_);

    // Store the value to be transferred (ENABLE | SNGSTRT)
    lptim1_sngstrt_value_ = LPTIM_CR_ENABLE | LPTIM_CR_SNGSTRT;

    // Configure DMA channel:
    // - Memory to peripheral
    // - Memory address: &lptim1_sngstrt_value_
    // - Peripheral address: &LPTIM1->CR
    // - Transfer size: 32-bit
    // - No increment (single value, single destination)
    // - Circular mode (repeat on each trigger)
    dma->CPAR = reinterpret_cast<uint32_t>(&LPTIM1->CR);
    dma->CMAR = reinterpret_cast<uint32_t>(&lptim1_sngstrt_value_);
    dma->CNDTR = 1;

    dma->CCR =
        DMA_CCR_CIRC |           // Circular mode
        (0x2 << DMA_CCR_MSIZE_Pos) |  // 32-bit memory size
        (0x2 << DMA_CCR_PSIZE_Pos) |  // 32-bit peripheral size
        DMA_CCR_DIR |            // Memory to peripheral
        DMA_CCR_EN;              // Enable channel
  }

  static void WaitForAdc(ADC_TypeDef* adc) MOTEUS_CCM_ATTRIBUTE {
    while ((adc->ISR & ADC_ISR_EOS) == 0);
  }

  // CALLED IN INTERRUPT CONTEXT.
  static void GlobalInterrupt() MOTEUS_CCM_ATTRIBUTE {
    g_impl_->ISR_HandleTimer();
  }

  // CALLED IN INTERRUPT CONTEXT.
  void ISR_HandleTimer() __attribute__((always_inline)) MOTEUS_CCM_ATTRIBUTE {
    // From here, until when we finish sampling the ADC has a critical
    // speed requirement.  Any extra cycles will result in a lower
    // maximal duty cycle of the controller.  Thus there are lots of
    // micro-optimizations to try and speed things up by little bits.
    const auto sr = *timer_sr_;
    const auto cr = *timer_cr1_;

    // Reset the status register.
    timer_->SR = 0x00;

    if ((sr & TIM_SR_UIF) &&
        (cr & TIM_CR1_DIR)) {
      ISR_DoTimer();
    }
  }

  void ISR_DoTimer() __attribute__((always_inline)) MOTEUS_CCM_ATTRIBUTE {
    phase_ = (phase_ + 1) & rate_config_.interrupt_mask;
    if (phase_) { return; }

#ifdef MOTEUS_PERFORMANCE_MEASURE
    DWT->CYCCNT = 0;
#endif

    // No matter what mode we are in, always sample our ADC and
    // position sensors.
    ISR_DoSenseCritical();

    // We should not re-enter into this interrupt until our low
    // priority portion is complete.
    NVIC_DisableIRQ(pwm_irqn_);

    SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
  }

  static void GlobalPendSv() MOTEUS_CCM_ATTRIBUTE {
    g_impl_->ISR_DoTimerLowerPriority();
  }

  void ISR_DoTimerLowerPriority() __attribute__((always_inline)) MOTEUS_CCM_ATTRIBUTE {
    SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;

    ISR_DoSense();
#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.sense = DWT->CYCCNT;
#endif

    // current_data_ is volatile, so read it out now, and operate on
    // the pointer for the rest of the routine.
    CommandData* data = current_data_;

    const float electrical_theta = !data->synthetic_theta ?
        position_.electrical_theta :
        WrapZeroToTwoPi(
            motor_position_config()->output.sign *
            MotorPosition::IntToFloat(*status_.control_position_raw)
            / motor_position_->config()->rotor_to_output_ratio
            * motor_.poles
            * 0.5f
            * k2Pi);

    status_.electrical_theta = electrical_theta;

    SinCos sin_cos = cordic_(RadiansToQ31(electrical_theta));
    status_.sin = sin_cos.s;
    status_.cos = sin_cos.c;

    ISR_CalculateCurrentState(sin_cos);

    if (config_.fixed_voltage_mode) {
      // Don't pretend we know where we are.
      status_.position = 0.0f;
      status_.velocity = 0.0f;
      status_.torque_Nm = 0.0f;
      status_.torque_error_Nm = 0.0f;
    }

#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.curstate = DWT->CYCCNT;
#endif

    ISR_DoControl(sin_cos, data);

#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.control = DWT->CYCCNT;
#endif

    ISR_MaybeEmitDebug();

#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.done = DWT->CYCCNT;
#endif

    const uint32_t cnt = timer_->CNT;
    status_.final_timer =
        ((*timer_cr1_) & TIM_CR1_DIR) ?
        (pwm_counts_ - cnt) :
        (pwm_counts_ + cnt);
    status_.total_timer = 2 * pwm_counts_ * rate_config_.interrupt_divisor;

#ifdef MOTEUS_DEBUG_OUT
    debug_out_ = 0;
#endif

    NVIC_EnableIRQ(pwm_irqn_);
  }

  void ISR_DoSenseCritical() __attribute__((always_inline)) MOTEUS_CCM_ATTRIBUTE {
    // ADCs are triggered by hardware: Timer Update -> DMA -> LPTIM1
    // -> LPTIM1_OUT -> ADCs.  This satisfies STM32G4 errata ES0430
    // section 2.7.11 (simultaneous triggering).

    // Wait for ADC sampling to complete.
    while ((ADC3->ISR & ADC_ISR_EOSMP) == 0);

#ifdef MOTEUS_DEBUG_OUT
    // We would like to set this debug pin as soon as possible.
    // However, if we flip it while the current ADCs are sampling,
    // they can get a lot more noise in some situations.  Thus just
    // wait until now.
    debug_out_ = 1;
#endif

    // We are now out of the most time critical portion of the ISR,
    // although it is still all pretty time critical since it runs at
    // up to 30kHz.  But time spent until now actually limits the
    // maximum duty cycle we can achieve, whereas time spent below
    // just eats cycles the rest of the code could be using.

    // Check to see if any motor outputs are now high.  If so, fault,
    // because we have exceeded the maximum duty cycle we can achieve
    // while still sampling current correctly.

#ifndef MOTEUS_DISABLE_PWM_CYCLE_OVERRUN
    if (status_.mode != kFault &&
        phase_monitors_.read()) {
      status_.mode = kFault;
      status_.fault = errc::kPwmCycleOverrun;
    }
#endif
  }

  void ISR_DoSense() __attribute__((always_inline)) MOTEUS_CCM_ATTRIBUTE {
    // With sampling done, we can kick off our encoder read.
    aux1_port_->ISR_MaybeStartSample();
    aux2_port_->ISR_MaybeStartSample();

    if (std::isnan(current_data_->timeout_s) ||
        current_data_->timeout_s != 0.0f) {
      status_.timeout_s = current_data_->timeout_s;
      current_data_->timeout_s = 0.0;
    }

    // And now, wait for the entire conversion to complete.  We
    // started ADC3 last, so we just wait for it.
    WaitForAdc(ADC3);

#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.adc_done = DWT->CYCCNT;
#endif

    if (family0_) {
      status_.adc_cur1_raw = ADC3->DR;
      status_.adc_cur2_raw = ADC1->DR;
      status_.adc_cur3_raw = ADC2->DR;
    } else if (family1_) {
      status_.adc_cur1_raw = ADC1->DR;
      status_.adc_cur2_raw = ADC2->DR;
      status_.adc_cur3_raw = ADC3->DR;
    } else if (family2_ || family3_) {
      status_.adc_cur1_raw = ADC3->DR;
      status_.adc_cur2_raw = ADC2->DR;
      status_.adc_cur3_raw = ADC1->DR;
    }

    // With hardware LPTIM1 triggering, ADC5 samples one channel per
    // control cycle. We alternate between the two channels
    // (vsense/tsense or tsense/msense depending on board version)
    // each cycle.
    WaitForAdc(ADC4);
    WaitForAdc(ADC5);

    // Clear the end of sample flag for the ADCs we check.
    ADC3->ISR |= (ADC_ISR_EOSMP | ADC_ISR_EOS);
    ADC4->ISR |= (ADC_ISR_EOSMP | ADC_ISR_EOS);
    ADC5->ISR |= (ADC_ISR_EOSMP | ADC_ISR_EOS);

    // Read ADC4 (same every cycle)
    if (family0_rev4_and_older_) {
      status_.adc_motor_temp_raw = ADC4->DR;
    } else if (family0_) {
      status_.adc_voltage_sense_raw = ADC4->DR;
    } else if (family1or2or3_) {
      status_.adc_fet_temp_raw = ADC4->DR;
    }

    // Read ADC5 based on which phase we're in, then switch to other channel
    if (adc5_phase_ == 0) {
      // First channel: vsense (family0_rev4_and_older, family1or2or3) or
      //                fet_temp (family0)
      if (family0_rev4_and_older_) {
        status_.adc_voltage_sense_raw = ADC5->DR;
        ADC5->SQR1 = (0 << ADC_SQR1_L_Pos) | (tsense_sqr_ << ADC_SQR1_SQ1_Pos);
      } else if (family0_) {
        status_.adc_fet_temp_raw = ADC5->DR;
        ADC5->SQR1 = (0 << ADC_SQR1_L_Pos) | (msense_sqr_ << ADC_SQR1_SQ1_Pos);
      } else if (family1or2or3_) {
        status_.adc_voltage_sense_raw = ADC5->DR;
        ADC5->SQR1 = (0 << ADC_SQR1_L_Pos) | (msense_sqr_ << ADC_SQR1_SQ1_Pos);
      }
    } else {
      // Second channel: tsense (family0_rev4_and_older) or
      //                 msense (family0, family1or2or3)
      if (family0_rev4_and_older_) {
        status_.adc_fet_temp_raw = ADC5->DR;
        ADC5->SQR1 = (0 << ADC_SQR1_L_Pos) | (vsense_sqr_ << ADC_SQR1_SQ1_Pos);
      } else if (family0_) {
        status_.adc_motor_temp_raw = ADC5->DR;
        ADC5->SQR1 = (0 << ADC_SQR1_L_Pos) | (tsense_sqr_ << ADC_SQR1_SQ1_Pos);
      } else if (family1or2or3_) {
        status_.adc_motor_temp_raw = ADC5->DR;
        ADC5->SQR1 = (0 << ADC_SQR1_L_Pos) | (vsense_sqr_ << ADC_SQR1_SQ1_Pos);
      }
    }
    adc5_phase_ = 1 - adc5_phase_;

#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.start_pos_sample = DWT->CYCCNT;
#endif

    aux1_port_->ISR_MaybeFinishSample();
    aux2_port_->ISR_MaybeFinishSample();
    aux_adc_->ISR_StartSample();

#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.done_pos_sample = DWT->CYCCNT;
#endif
    motor_position_->ISR_Update();

    velocity_filter_(position_.velocity, &status_.velocity_filt);


#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.done_temp_sample = DWT->CYCCNT;
#endif

    {
      status_.fet_temp_C = fet_thermistor_.Calculate(status_.adc_fet_temp_raw);
      temperature_filter_(status_.fet_temp_C, &status_.filt_fet_temp_C);

      if (config_.enable_motor_temperature) {
        status_.motor_temp_C = motor_thermistor_.Calculate(status_.adc_motor_temp_raw);
        temperature_filter_(status_.motor_temp_C, &status_.filt_motor_temp_C);
      } else {
        status_.motor_temp_C = status_.filt_motor_temp_C = 0.0f;
      }
    }

    status_.position = position_.position;
    status_.velocity = position_.velocity;

    aux_adc_->ISR_EndSample();
    aux1_port_->ISR_EndAnalogSample();
    aux2_port_->ISR_EndAnalogSample();
  }

  // This is called from the ISR.
  void ISR_CalculateCurrentState(const SinCos& sin_cos) MOTEUS_CCM_ATTRIBUTE {
    status_.cur1_A = (status_.adc_cur1_raw - status_.adc_cur1_offset) * adc_scale_;
    status_.cur2_A = (status_.adc_cur2_raw - status_.adc_cur2_offset) * adc_scale_;
    status_.cur3_A = (status_.adc_cur3_raw - status_.adc_cur3_offset) * adc_scale_;
    if (motor_.phase_invert) {
      std::swap(status_.cur2_A, status_.cur3_A);
    }
    status_.bus_V = status_.adc_voltage_sense_raw * vsense_adc_scale_;

    slow_bus_v_filter_(status_.bus_V, &status_.filt_bus_V);
    fast_bus_v_filter_(status_.bus_V, &status_.filt_1ms_bus_V);

    DqTransform dq{sin_cos,
          status_.cur1_A,
          status_.cur3_A,
          status_.cur2_A
          };
    status_.d_A = dq.d;
    status_.q_A = motor_position_config()->output.sign * dq.q;
    const bool is_torque_on = torque_on();
    status_.torque_Nm = is_torque_on ? (
        current_to_torque(status_.q_A) /
        motor_position_->config()->rotor_to_output_ratio) : 0.0f;
    if (!is_torque_on) {
      status_.torque_error_Nm = 0.0f;
    }

    // As of firmware ABI 0x010a moteus records motor Kv values that
    // correspond roughly with open loop oscilloscope measurements and
    // motor manufacturer's ratings.  They don't perfectly correspond
    // to the speed that can actually be achieved under control.  For
    // stable control loops, we need to limit the maximum controlled
    // velocity to be a modest amount under what is actually capable,
    // which tends to be around 90% of the speed expected based on
    // input voltage, Kv, and modulation depth alone.
    constexpr float kVelocityMargin = 0.87f;

    status_.motor_max_velocity =
        motor_position_->config()->rotor_to_output_ratio *
        rate_config_.max_voltage_ratio *
        kVelocityMargin * 0.5f * status_.filt_1ms_bus_V / v_per_hz_;

    status_.max_power_W = [&]() {
      if (config_.override_board_max_power &&
          std::isfinite(config_.max_power_W)) {
        return config_.max_power_W;
      }
      const float board_power_limit =
          Limit(
              Interpolate(status_.filt_bus_V,
                          g_hw_pins.power_V_l, g_hw_pins.power_V_h,
                          g_hw_pins.power_P_l_W, g_hw_pins.power_P_h_W),
              g_hw_pins.power_P_h_W,
              g_hw_pins.power_P_l_W) * pwm_derate_;
      if (!std::isfinite(config_.max_power_W)) {
        return board_power_limit;
      }
      return std::min(config_.max_power_W, board_power_limit);
    }();
#ifdef MOTEUS_EMIT_CURRENT_TO_DAC
    DAC1->DHR12R1 = static_cast<uint32_t>(dq.d * 400.0f + 2048.0f);
#endif
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
      case kVoltageDq:
      case kCurrent:
      case kPosition:
      case kZeroVelocity:
      case kStayWithinBounds:
      case kMeasureInductance:
      case kBrake: {
        return true;
      }
      case kPositionTimeout: {
        return config_.timeout_mode != 0;
      }
    }
    return false;
  }


  void ISR_DoCalibrating() {
    calibrate_adc1_ += status_.adc_cur1_raw;
    calibrate_adc2_ += status_.adc_cur2_raw;
    calibrate_adc3_ += status_.adc_cur3_raw;
    calibrate_count_++;

    if (calibrate_count_ < kCalibrateCount) {
      return;
    }

    const uint16_t new_adc1_offset = calibrate_adc1_ / kCalibrateCount;
    const uint16_t new_adc2_offset = calibrate_adc2_ / kCalibrateCount;
    const uint16_t new_adc3_offset = calibrate_adc3_ / kCalibrateCount;

    if (std::abs(static_cast<int>(new_adc1_offset) - 2048) > 200 ||
        std::abs(static_cast<int>(new_adc2_offset) - 2048) > 200 ||
        std::abs(static_cast<int>(new_adc3_offset) - 2048) > 200) {
      // Error calibrating.  Just fault out.
      status_.mode = kFault;
      status_.fault = errc::kCalibrationFault;
      return;
    }

    status_.adc_cur1_offset = new_adc1_offset;
    status_.adc_cur2_offset = new_adc2_offset;
    status_.adc_cur3_offset = new_adc3_offset;
    status_.mode = kCalibrationComplete;
  }

  void ISR_MaybeEmitDebug() MOTEUS_CCM_ATTRIBUTE {
    if (config_.emit_debug == 0 || !debug_uart_) { return; }

    debug_buf_[0] = 0x5a;

    int pos = 1;

    auto write_scalar =
        [&](auto value) {
          if ((pos + sizeof(value)) > sizeof(debug_buf_)) {
            return;
          }
          std::memcpy(&debug_buf_[pos], &value, sizeof(value));
          pos += sizeof(value);
        };

    if (config_.emit_debug & (1 << 0)) {
      write_scalar(static_cast<uint16_t>(aux1_port_->status()->spi.value * 4));
    }

    if (config_.emit_debug & (1 << 1)) {
      write_scalar(static_cast<int16_t>(32767.0f * status_.velocity / 100.0f));
    }

    if (config_.emit_debug & (1 << 2)) {
      write_scalar(static_cast<int16_t>(32767.0f * status_.d_A / 100.0f));
    }
    if (config_.emit_debug & (1 << 3)) {
      write_scalar(static_cast<int16_t>(32767.0f * status_.q_A / 100.0f));
    }
    if (config_.emit_debug & (1 << 4)) {
      write_scalar(status_.adc_cur1_raw);
    }
    if (config_.emit_debug & (1 << 5)) {
      write_scalar(status_.adc_cur2_raw);
    }
    if (config_.emit_debug & (1 << 6)) {
      write_scalar(status_.adc_cur3_raw);
    }

    if (config_.emit_debug & (1 << 7)) {
      write_scalar(static_cast<int16_t>(32767.0f * status_.cur1_A / 100.0f));
    }

    if (config_.emit_debug & (1 << 8)) {
      write_scalar(static_cast<int16_t>(32767.0f * status_.cur2_A / 100.0f));
    }

    if (config_.emit_debug & (1 << 9)) {
      write_scalar(static_cast<int16_t>(32767.0f * status_.cur3_A / 100.0f));
    }

    if (config_.emit_debug & (1 << 10)) {
      write_scalar(static_cast<int16_t>(32767.0f * control_.torque_Nm / 30.0f));
    }

    if (config_.emit_debug & (1 << 11)) {
      write_scalar(static_cast<uint16_t>(position_.sources[0].raw));
    }

    if (config_.emit_debug & (1 << 12)) {
      write_scalar(static_cast<uint16_t>(position_.sources[1].raw));
    }

    if (config_.emit_debug & (1 << 13)) {
      write_scalar(static_cast<uint16_t>(position_.sources[2].raw));
    }

    if (config_.emit_debug & (1 << 14)) {
      write_scalar(static_cast<uint16_t>(status_.final_timer));
    }

    if (config_.emit_debug & (1 << 15)) {
      write_scalar(static_cast<int16_t>(32767.0f * control_.d_V / 64.0f));
    }

    if (config_.emit_debug & (1 << 16)) {
      write_scalar(static_cast<int16_t>(32767.0f * control_.q_V / 64.0f));
    }

    if (config_.emit_debug & (1 << 17)) {
      write_scalar(static_cast<int16_t>(control_.pwm.a * 32767.0f));
    }

    if (config_.emit_debug & (1 << 18)) {
      write_scalar(static_cast<int16_t>(control_.pwm.b * 32767.0f));
    }

    if (config_.emit_debug & (1 << 19)) {
      write_scalar(static_cast<int16_t>(control_.pwm.c * 32767.0f));
    }

    if (config_.emit_debug & (1 << 20)) {
      write_scalar(static_cast<int16_t>(32767.0f * status_.torque_Nm / 30.0f));
    }

    if (config_.emit_debug & (1 << 21)) {
      write_scalar(static_cast<uint16_t>(32767.0f * status_.power_W / 3000.0f));
    }

    // We rely on the FIFO to queue these things up.
    for (int i = 0; i < pos; i++) {
      debug_uart_->TDR = debug_buf_[i];
    }
  }

  void DoPwmControl(const Vec3& pwm) MOTEUS_CCM_ATTRIBUTE {
    const uint16_t pwm1 =
        static_cast<uint16_t>(pwm.a * pwm_counts_);
    const uint16_t pwm2 =
        static_cast<uint16_t>(pwm.b * pwm_counts_);
    const uint16_t pwm3 =
        static_cast<uint16_t>(pwm.c * pwm_counts_);

    *pwm1_ccr_ = pwm1;
    if (!motor_.phase_invert) {
      *pwm2_ccr_ = pwm3;
      *pwm3_ccr_ = pwm2;
    } else {
      *pwm2_ccr_ = pwm2;
      *pwm3_ccr_ = pwm3;
    }

    motor_driver_->PowerOn();
  }

  void DoHardStop() MOTEUS_CCM_ATTRIBUTE {
    const auto result = motor_driver_->StartEnable(false);
    MJ_ASSERT(result == MotorDriver::kDisabled);
    motor_driver_->PowerOff();
    *pwm1_ccr_ = 0;
    *pwm2_ccr_ = 0;
    *pwm3_ccr_ = 0;
  }

  void DoFault() MOTEUS_CCM_ATTRIBUTE {
    motor_driver_->PowerOff();
    *pwm1_ccr_ = 0;
    *pwm2_ccr_ = 0;
    *pwm3_ccr_ = 0;

    status_.power_W = 0.0f;
  }

  void DoCalibrating() MOTEUS_CCM_ATTRIBUTE {
    ISR_DoCalibrating();
  }

  void DoBrake() MOTEUS_CCM_ATTRIBUTE {
    *pwm1_ccr_ = 0;
    *pwm2_ccr_ = 0;
    *pwm3_ccr_ = 0;
    motor_driver_->PowerOn();
  }

  void StartCalibrating() MOTEUS_CCM_ATTRIBUTE {
    (*pwm1_ccr_) = 0;
    (*pwm2_ccr_) = 0;
    (*pwm3_ccr_) = 0;
    motor_driver_->PowerOff();

    calibrate_adc1_ = 0;
    calibrate_adc2_ = 0;
    calibrate_adc3_ = 0;
    calibrate_count_ = 0;
  }

  bool motor_driver_fault() const MOTEUS_CCM_ATTRIBUTE {
    return motor_driver_->fault();
  }

  SinCos cordic(int32_t radians_q31) const MOTEUS_CCM_ATTRIBUTE {
    return cordic_(radians_q31);
  }

  int64_t absolute_relative_delta() const MOTEUS_CCM_ATTRIBUTE {
    return static_cast<int64_t>(
        motor_position_->absolute_relative_delta.load());
  }

  const Options options_;
  MillisecondTimer* const ms_timer_;
  MotorDriver* const motor_driver_;
  AuxADC* const aux_adc_;
  AuxPort* const aux1_port_;
  AuxPort* const aux2_port_;
  MotorPosition* const motor_position_;


  Motor& motor_ = *motor_position_->motor();
  const MotorPosition::Status& position_ = motor_position_->status();
  Config config_;
  PositionConfig position_config_;

  // This copy of the current epoch is intended to be accessed from the ISR.
  uint8_t isr_motor_position_epoch_ = 0;

  // This copy is only accessed from the main loop.
  uint8_t main_motor_position_epoch_ = 0;

  TIM_TypeDef* timer_ = nullptr;
  volatile uint32_t* timer_sr_ = nullptr;
  volatile uint32_t* timer_cr1_ = nullptr;
  ADC_TypeDef* const adc1_ = ADC1;
  ADC_TypeDef* const adc2_ = ADC2;
  ADC_TypeDef* const adc3_ = ADC3;
#if defined(TARGET_STM32G4)
  ADC_TypeDef* const adc4_ = ADC4;
  ADC_TypeDef* const adc5_ = ADC5;
  ADC_Common_TypeDef* const adc12_common_ = ADC12_COMMON;
  ADC_Common_TypeDef* const adc345_common_ = ADC345_COMMON;
#endif
  DAC_TypeDef* const dac_ = DAC;

  // We create these to initialize our pins as output and PWM mode,
  // but otherwise don't use them.
  PwmOut pwm1_;
  PwmOut pwm2_;
  PwmOut pwm3_;

  PhaseMonitors phase_monitors_;

  volatile uint32_t* pwm1_ccr_ = nullptr;
  volatile uint32_t* pwm2_ccr_ = nullptr;
  volatile uint32_t* pwm3_ccr_ = nullptr;

  AnalogIn current1_;
  AnalogIn current2_;
  AnalogIn current3_;
  AnalogIn vsense_;
  uint32_t vsense_sqr_ = {};
  AnalogIn tsense_;
  uint32_t tsense_sqr_ = {};
  AnalogIn msense_;
  uint32_t msense_sqr_ = {};

  AnalogOut debug_dac_;

#ifdef MOTEUS_DEBUG_OUT
  // This is just for debugging.
  DigitalOut debug_out_;
#endif

  RateConfig rate_config_;

  int32_t phase_ = 0;

  // Tracks which ADC5 channel is being sampled (alternates each control cycle).
  // 0 = first channel (vsense/tsense/fet_temp depending on board)
  // 1 = second channel (tsense/msense depending on board)
  int32_t adc5_phase_ = 0;

  // Value transferred by DMA to LPTIM1->CR to trigger ADC sampling.
  // Must be in memory accessible by DMA (not CCM).
  uint32_t lptim1_sngstrt_value_ = 0;

  Thermistor fet_thermistor_;
  Thermistor motor_thermistor_;

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
  Status status_;
  Control control_;
  uint32_t calibrate_adc1_ = 0;
  uint32_t calibrate_adc2_ = 0;
  uint32_t calibrate_adc3_ = 0;
  uint16_t calibrate_count_ = 0;

  SimplePI pid_d_{&config_.pid_dq, &status_.pid_d};
  SimplePI pid_q_{&config_.pid_dq, &status_.pid_q};
  PID pid_position_{&config_.pid_position, &status_.pid_position};

  USART_TypeDef* debug_uart_ = nullptr;
  USART_TypeDef* onboard_debug_uart_ = nullptr;

  // 7 bytes is the max that we can get out at 3Mbit running at
  // 40000Hz.
  uint8_t debug_buf_[7] = {};

  float torque_constant_ = 0.01f;
  float v_per_hz_ = 0.0f;
  float flux_brake_min_voltage_ = 0.0f;
  float derate_temperature_ = 0.0f;
  float motor_derate_temperature_ = 0.0f;

  float adc_scale_ = 0.0f;
  float pwm_derate_ = 1.0f;

  float old_d_V_ = 0.0f;
  float old_q_V_ = 0.0f;

  float vsense_adc_scale_ = 0.0f;

  uint32_t pwm_counts_ = 0;
  Cordic cordic_;

  uint32_t adc1_sqr_ = 0;
  uint32_t adc2_sqr_ = 0;
  uint32_t adc3_sqr_ = 0;
  uint32_t adc4_sqr_ = 0;

  IRQn_Type pwm_irqn_ = {};

  ExponentialFilter velocity_filter_;
  ExponentialFilter temperature_filter_;
  ExponentialFilter slow_bus_v_filter_;
  ExponentialFilter fast_bus_v_filter_;

  const bool family0_rev4_and_older_ = (
      g_measured_hw_family == 0 &&
      g_measured_hw_rev <= 4);
  const bool family0_ = (g_measured_hw_family == 0);
  const bool family1or2or3_ = (g_measured_hw_family == 1 ||
                               g_measured_hw_family == 2 ||
                               g_measured_hw_family == 3);
  const bool family1_ = (g_measured_hw_family == 1);
  const bool family2_ = (g_measured_hw_family == 2);
  const bool family3_ = (g_measured_hw_family == 3);

  static Impl* g_impl_;
};

BldcServo::Impl* BldcServo::Impl::g_impl_ = nullptr;

BldcServo::BldcServo(micro::Pool* pool,
                     micro::PersistentConfig* persistent_config,
                     micro::TelemetryManager* telemetry_manager,
                     MillisecondTimer* millisecond_timer,
                     MotorDriver* motor_driver,
                     AuxADC* aux_adc,
                     AuxPort* aux1_port,
                     AuxPort* aux2_port,
                     MotorPosition* motor_position,
                     const Options& options)
    : impl_(pool,
            persistent_config, telemetry_manager,
            millisecond_timer, motor_driver,
            aux_adc, aux1_port, aux2_port, motor_position,
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

const BldcServo::Control& BldcServo::control() const {
  return impl_->control();
}

const AuxPort::Status& BldcServo::aux1() const {
  return impl_->aux1();
}

const AuxPort::Status& BldcServo::aux2() const {
  return impl_->aux2();
}

const MotorPosition::Status& BldcServo::motor_position() const {
  return impl_->motor_position();
}

MotorPosition::Config* BldcServo::motor_position_config() {
  return impl_->motor_position_config();
}

const MotorPosition::Config* BldcServo::motor_position_config() const {
  return impl_->motor_position_config();
}

void BldcServo::SetOutputPositionNearest(float position) {
  impl_->SetOutputPositionNearest(position);
}

void BldcServo::SetOutputPosition(float position) {
  impl_->SetOutputPosition(position);
}

void BldcServo::RequireReindex() {
  impl_->RequireReindex();
}

void BldcServo::RecapturePositionVelocity() {
  impl_->RecapturePositionVelocity();
}

void BldcServo::Fault(moteus::errc fault_code) {
  impl_->Fault(fault_code);
}

}
