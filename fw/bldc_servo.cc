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

#include "fw/bldc_servo_position.h"
#include "fw/foc.h"
#include "fw/math.h"
#include "fw/moteus_hw.h"
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


float Limit(float, float, float) MOTEUS_CCM_ATTRIBUTE;

float Limit(float a, float min, float max) {
  if (a < min) { return min; }
  if (a > max) { return max; }
  return a;
}

// This variant, in addition to performing the limiting, will return
// the `code_limit` if limiting was performed and `code_nolimit` if no
// limiting was performed.  It can be used to chain operations, to
// determine which operation caused any limiting to occur.
std::pair<float, errc> LimitCode(float, float, float, errc, errc) MOTEUS_CCM_ATTRIBUTE;

std::pair<float, errc> LimitCode(float a, float min, float max, errc code_limit, errc code_nolimit) {
  if (a < min) { return {min, code_limit}; }
  if (a > max) { return {max, code_limit}; }
  return {a, code_nolimit};
}

float Threshold(float, float, float) MOTEUS_CCM_ATTRIBUTE;

float Threshold(float value, float lower, float upper) {
  if (value > lower && value < upper) { return 0.0f; }
  return value;
}

float Interpolate(float, float, float, float, float) MOTEUS_CCM_ATTRIBUTE;

float Interpolate(float x, float xl, float xh, float vl, float vh) {
  return (x - xl) / (xh - xl) * (vh - vl) + vl;
}

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

// This is used to determine the maximum allowable PWM value so that
// the current sampling is guaranteed to occur while the FETs are
// still low.  It was calibrated using the scope and trial and error.
//
// The primary test is a high torque pulse with absolute position
// limits in place of +-1.0.  Something like "d pos nan 0 1 p0 d0 f1".
// This all but ensures the current controller will saturate.
//
// As of 2020-09-13, 0.98 was the highest value that failed.
constexpr float kCurrentSampleTime = 1.03e-6f;


// All of these constants depend upon the pwm rate.
struct RateConfig {
  int int_rate_hz;
  int interrupt_divisor;
  uint32_t interrupt_mask;
  int pwm_rate_hz;
  float min_pwm;
  float max_pwm;
  float max_voltage_ratio;
  float rate_hz;
  float period_s;
  int16_t max_position_delta;

  RateConfig(int pwm_rate_hz_in = 30000) {
    const int board_min_pwm_rate_hz =
        (g_measured_hw_family == 0 &&
         g_measured_hw_rev == 2) ? 60000 :
        15000;

    // Limit our PWM rate to even frequencies between 15kHz and 60kHz.
    pwm_rate_hz =
        ((std::max(board_min_pwm_rate_hz,
                   std::min(60000, pwm_rate_hz_in))) / 2) * 2;

    interrupt_divisor = (pwm_rate_hz > 30000) ? 2 : 1;
    interrupt_mask = [&]() {
                       switch (interrupt_divisor) {
                         case 1: return 0;
                         case 2: return 1;
                         default: mbed_die();
                       }
                     }();

    // The maximum interrupt rate is 30kHz, so if our PWM rate is
    // higher than that, then set up the interrupt at half rate.
    int_rate_hz = pwm_rate_hz / interrupt_divisor;

    min_pwm = kCurrentSampleTime / (0.5f / static_cast<float>(pwm_rate_hz));
    max_pwm = 1.0f - min_pwm;
    max_voltage_ratio = ((max_pwm - 0.5f) * 2.0f);

    rate_hz = int_rate_hz;
    period_s = 1.0f / rate_hz;

    // The maximum amount the absolute encoder can change in one cycle
    // without triggering a fault.  Measured as a fraction of a uint16_t
    // and corresponds to roughly 28krpm, which is the limit of the AS5047
    // encoder.
    //  28000 / 60 = 467 Hz
    //  467 Hz * 65536 / kIntRate ~= 763
    max_position_delta = 28000 / 60 * 65536 / int_rate_hz;
  }
};

constexpr int kCalibrateCount = 256;

constexpr float kDefaultTorqueConstant = 0.1f;
constexpr float kMaxUnconfiguredCurrent = 5.0f;

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

class BldcServo::Impl {
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

  float torque_to_current(float torque) const MOTEUS_CCM_ATTRIBUTE {
    TorqueModel model(torque_constant_,
                      motor_.rotation_current_cutoff_A,
                      motor_.rotation_current_scale,
                      motor_.rotation_torque_scale);
    return model.torque_to_current(torque);
  }

  void UpdateConfig() {
    rate_config_ = RateConfig(config_.pwm_rate_hz);
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
    pwm_counts_ = HAL_RCC_GetPCLK1Freq() * 2 / (2 * rate_config_.pwm_rate_hz);
    timer_->ARR = pwm_counts_;

    // Reinitialize the counter and update all registers.
    timer_->EGR |= TIM_EGR_UG;

    // Finally, enable the timer.
    timer_->CR1 |= TIM_CR1_CEN;
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

    // Per "ES0430 - Rev 8, 2.7.9" the ADCs can only be used
    // simultaneously if they are in synchronous mode with a divider
    // no more than 1.  Yay.  We can't use synchronous mode with a
    // divider of 1, since that would run the ADCs too fast.  Instead,
    // we use a divider of 2, and ensure that each ADC is started in
    // an exact phase relationship to the global cycle counter.
    ADC12_COMMON->CCR =
        (2 << ADC_CCR_CKMODE_Pos) |  // synchronous AHB/2
        (1 << ADC_CCR_DUAL_Pos); // dual mode, regular + injected
    ADC345_COMMON->CCR =
        (2 << ADC_CCR_CKMODE_Pos) |  // synchronous AHB/2
        (1 << ADC_CCR_DUAL_Pos); // dual mode, regular + injected

    constexpr int kAdcPrescale = 2;  // from the CKMODE above

    EnableAdc(ms_timer_, ADC1, kAdcPrescale, 0);
    EnableAdc(ms_timer_, ADC2, kAdcPrescale, 0);
    EnableAdc(ms_timer_, ADC3, kAdcPrescale, 0);
    EnableAdc(ms_timer_, ADC4, kAdcPrescale, 0);
    EnableAdc(ms_timer_, ADC5, kAdcPrescale, 0);

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

  static void WaitForAdc(ADC_TypeDef* adc) MOTEUS_CCM_ATTRIBUTE {
    while ((adc->ISR & ADC_ISR_EOC) == 0);
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
    // We start our conversion here so that it can work while we get
    // ready.  This means we will throw away the result if our control
    // timer says it isn't our turn yet, but that is a relatively
    // minor waste.


    {
      // To start the ADCs, for now we resort to some inline assembly.
      // The below is roughly equivalent to:
      //
      //  auto tmp = ADC1->CR;
      //  tmp |= ADC_CR_ADSTART;
      //  ADC1->CR = tmp;
      //  ADC3->CR = tmp;
      //  ADC5->CR = tmp;
      //
      // Note: Since ADC1/2 and ADC3/4 are in dual mode, we don't have
      // to explitly start ADC2 or ADC4.
      //
      // We perform this using inline assembly so as to attempt to
      // start the trigger process of all 5 ADCs as closely as
      // possible.  Per STM32G474 errata "ES0430 Rev 8 - 2.7.9", the
      // ADCs only have good performance if they are started at
      // exactly the same time.  Ideally we'd do that through a
      // hardware, i.e. timer trigger.  However, getting that
      // integrated here is a bigger project.  For now, this seems to
      // give pretty good results.
      uint32_t temp_reg1;
      uint32_t temp_reg2;
      asm volatile (
          "mov %[temp_reg1], %[adstart];"
          "ldr %[temp_reg2], [%[adc1_cr], #0];"
          "orr %[temp_reg1], %[temp_reg2];"
          "str %[temp_reg1], [%[adc1_cr], #0];"
          "str %[temp_reg1], [%[adc3_cr], #0];"
          "str %[temp_reg1], [%[adc5_cr], #0];"
          : [temp_reg1]"=&r"(temp_reg1),
            [temp_reg2]"=&r"(temp_reg2)
          : [adstart]"r"(ADC_CR_ADSTART),
            [adc1_cr]"r"(&ADC1->CR),
            [adc3_cr]"r"(&ADC3->CR),
            [adc5_cr]"r"(&ADC5->CR)
          :
      );
    }

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
    // Wait for sampling to complete.
    while ((ADC3->ISR & ADC_ISR_EOS) == 0);

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

    // TODO: Since we have to let ADC4/5 sample for much longer, we
    // could save a lot of time by switching ADC5's targets every
    // other cycle and not even reading it until the position sampling
    // was done.  For now though, we read all the things every cycle.
    WaitForAdc(ADC4);
    WaitForAdc(ADC5);

    if (family0_rev4_and_older_) {
      status_.adc_motor_temp_raw = ADC4->DR;
      status_.adc_voltage_sense_raw = ADC5->DR;
    } else if (family0_) {
      status_.adc_voltage_sense_raw = ADC4->DR;
      status_.adc_fet_temp_raw = ADC5->DR;
    } else if (family1or2or3_) {
      status_.adc_fet_temp_raw = ADC4->DR;
      status_.adc_voltage_sense_raw = ADC5->DR;
    }

    // Start sampling the other thing on ADC5, what that is depends
    // upon our board version.
    if (family0_rev4_and_older_) {
      ADC5->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          tsense_sqr_ << ADC_SQR1_SQ1_Pos;
    } else {  // family 0 || family 1
      ADC5->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          msense_sqr_ << ADC_SQR1_SQ1_Pos;
    }

    ADC5->CR |= ADC_CR_ADSTART;

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

    // The temperature sensing should be done by now, but just double
    // check.
    WaitForAdc(ADC5);
    if (family0_rev4_and_older_) {
      status_.adc_fet_temp_raw = ADC5->DR;
    } else {
      status_.adc_motor_temp_raw = ADC5->DR;
    }

    if (family0_rev4_and_older_) {
      // Switch back to the voltage sense resistor.
      ADC5->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          (vsense_sqr_ << ADC_SQR1_SQ1_Pos);
    } else if (family0_) {
      // Switch back to FET temp sense.
      ADC5->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          (tsense_sqr_ << ADC_SQR1_SQ1_Pos);
    } else if (family1or2or3_) {
      ADC5->SQR1 =
          (0 << ADC_SQR1_L_Pos) |  // length 1
          (vsense_sqr_ << ADC_SQR1_SQ1_Pos);
    }

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

  bool current_control() const {
    switch (status_.mode) {
      case kNumModes: {
        MJ_ASSERT(false);
        return false;
      }
      case kFault:
      case kCalibrating:
      case kCalibrationComplete:
      case kEnabling:
      case kStopped:
      case kPwm:
      case kVoltage:
      case kVoltageFoc:
      case kVoltageDq:
      case kMeasureInductance:
      case kBrake: {
        return false;
      }
      case kCurrent:
      case kPosition:
      case kZeroVelocity:
      case kStayWithinBounds: {
        return true;
      }
      case kPositionTimeout: {
        return (config_.timeout_mode == BldcServoMode::kZeroVelocity ||
                config_.timeout_mode == BldcServoMode::kPosition);
      }
    }
    return false;
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

  void ISR_MaybeChangeMode(CommandData* data) MOTEUS_CCM_ATTRIBUTE {
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
      case kVoltageDq:
      case kCurrent:
      case kPosition:
      case kPositionTimeout:
      case kZeroVelocity:
      case kStayWithinBounds:
      case kMeasureInductance:
      case kBrake: {
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
          case kVoltageDq:
          case kCurrent:
          case kPosition:
          case kZeroVelocity:
          case kStayWithinBounds:
          case kMeasureInductance:
          case kBrake: {
            if ((data->mode == kPosition || data->mode == kStayWithinBounds) &&
                !data->ignore_position_bounds &&
                ISR_IsOutsideLimits()) {
              status_.mode = kFault;
              status_.fault = errc::kStartOutsideLimit;
            } else if ((data->mode == kPosition ||
                        data->mode == kStayWithinBounds) &&
                       !data->ignore_position_bounds &&
                       ISR_InvalidLimits()) {
              status_.mode = kFault;
              status_.fault = errc::kInvalidLimits;
            } else {
              // Yep, we can do this.
              status_.mode = data->mode;

              // We are entering a new active control mode.  Require
              // our PID loops to start from scratch.
              ISR_ClearPid(kAlwaysClear);
            }

            if (data->mode == kMeasureInductance) {
              status_.meas_ind_phase = 0;
              status_.meas_ind_integrator = 0.0f;
              status_.meas_ind_old_d_A = status_.d_A;
            }

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

  bool ISR_IsOutsideLimits() {
    return ((!std::isnan(position_config_.position_min) &&
             position_.position < position_config_.position_min) ||
            (!std::isnan(position_config_.position_max) &&
             position_.position > position_config_.position_max));
  }

  bool ISR_InvalidLimits() {
    return ((!std::isnan(position_config_.position_min) && (
                 std::abs(position_config_.position_min) > 32768.0f)) ||
            (!std::isnan(position_config_.position_max) && (
                std::abs(position_config_.position_max) > 32768.0f)));
  }

  void ISR_StartCalibrating() {
    // Capture the current motor position epoch.
    isr_motor_position_epoch_ = position_.epoch;

    status_.mode = kEnabling;

    // The main context will set our state to kCalibrating when the
    // motor driver is fully enabled.

    (*pwm1_ccr_) = 0;
    (*pwm2_ccr_) = 0;
    (*pwm3_ccr_) = 0;

    // Power should already be false for any state we could possibly
    // be in, but lets just be certain.
    motor_driver_->PowerOff();

    calibrate_adc1_ = 0;
    calibrate_adc2_ = 0;
    calibrate_adc3_ = 0;
    calibrate_count_ = 0;
  }

  enum ClearMode {
    kClearIfMode,
    kAlwaysClear,
  };

  void ISR_ClearPid(ClearMode force_clear) MOTEUS_CCM_ATTRIBUTE {
    const bool current_pid_active = [&]() MOTEUS_CCM_ATTRIBUTE {
      switch (status_.mode) {
        case kNumModes:
        case kFault:
        case kEnabling:
        case kCalibrating:
        case kCalibrationComplete:
        case kPwm:
        case kVoltage:
        case kVoltageFoc:
        case kVoltageDq:
        case kMeasureInductance:
        case kBrake:
          return false;
        case kCurrent:
        case kPosition:
        case kPositionTimeout:
        case kZeroVelocity:
        case kStayWithinBounds:
          return true;
        case kStopped: {
          return status_.cooldown_count != 0;
        }
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

    const bool position_pid_active = [&]() MOTEUS_CCM_ATTRIBUTE {
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
        case kVoltageDq:
        case kCurrent:
        case kMeasureInductance:
        case kBrake:
          return false;
        case kPosition:
        case kPositionTimeout:
        case kZeroVelocity:
        case kStayWithinBounds:
          return true;
      }
      return false;
    }();

    if (!position_pid_active || force_clear == kAlwaysClear) {
      status_.pid_position.Clear();
      status_.control_position_raw = {};
      status_.control_position = std::numeric_limits<float>::quiet_NaN();
      status_.control_velocity = {};
    }
  }

  void ISR_DoControl(const SinCos& sin_cos,
                     CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    old_d_V = control_.d_V;
    old_q_V = control_.q_V;

    control_.Clear();

    if (!std::isnan(status_.timeout_s) && status_.timeout_s > 0.0f) {
      status_.timeout_s =
          std::max(0.0f, status_.timeout_s - rate_config_.period_s);
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
      // NOTE: This is mostly to identify faulty voltage sense
      // components.  Actual undervolts are more likely to trigger the
      // drv8323 first.  If we erroneously use a very low voltage
      // here, we can command a very large current due to the voltage
      // compensation.
      if (status_.bus_V < 4.0f) {
        status_.mode = kFault;
        status_.fault = errc::kUnderVoltage;
      }
      if (status_.filt_fet_temp_C > config_.fault_temperature) {
        status_.mode = kFault;
        status_.fault = errc::kOverTemperature;
      }
      if (std::isfinite(config_.motor_fault_temperature) &&
          status_.filt_motor_temp_C > config_.motor_fault_temperature) {
        status_.mode = kFault;
        status_.fault = errc::kOverTemperature;
      }
    }

    if ((status_.mode == kPosition || status_.mode == kStayWithinBounds) &&
        !std::isnan(status_.timeout_s) &&
        status_.timeout_s <= 0.0f) {
      status_.mode = kPositionTimeout;
    }

    // Ensure unused PID controllers have zerod state.
    ISR_ClearPid(kClearIfMode);

    if (status_.mode != kFault) {
      status_.fault = errc::kSuccess;
    }

#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.control_sel_mode = DWT->CYCCNT;
#endif

    if (current_control()) {
      status_.cooldown_count = config_.cooldown_cycles;
    }

    switch (status_.mode) {
      case kNumModes:
      case kStopped: {
        ISR_DoStopped(sin_cos);
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
        ISR_DoBalancedVoltageControl(data->phase_v);
        break;
      }
      case kVoltageFoc: {
        ISR_DoVoltageFOC(data);
        break;
      }
      case kVoltageDq: {
        ISR_DoVoltageDQCommand(sin_cos, data->d_V, data->q_V);
        break;
      }
      case kCurrent: {
        ISR_DoCurrent(sin_cos, data->i_d_A, data->i_q_A, 0.0f,
                      data->ignore_position_bounds);
        break;
      }
      case kPosition: {
        ISR_DoPosition(sin_cos, data);
        break;
      }
      case kPositionTimeout: {
        ISR_DoPositionTimeout(sin_cos, data);
        break;
      }
      case kZeroVelocity: {
        ISR_DoZeroVelocity(sin_cos, data);
        break;
      }
      case kStayWithinBounds: {
        ISR_DoStayWithinBounds(sin_cos, data);
        break;
      }
      case kMeasureInductance: {
        ISR_DoMeasureInductance(sin_cos, data);
        break;
      }
      case kBrake: {
        ISR_DoBrake();
        break;
      }
    }
  }

  void ISR_DoStopped(const SinCos& sin_cos) MOTEUS_CCM_ATTRIBUTE {
    if (status_.cooldown_count) {
      status_.cooldown_count--;
      ISR_DoCurrent(sin_cos, 0.0f, 0.0f, 0.0f, false);
      return;
    }

    const auto result = motor_driver_->StartEnable(false);
    // We should always be able to disable immediately.
    MJ_ASSERT(result == MotorDriver::kDisabled);
    motor_driver_->PowerOff();
    *pwm1_ccr_ = 0;
    *pwm2_ccr_ = 0;
    *pwm3_ccr_ = 0;

    status_.power_W = 0.0f;
  }

  void ISR_DoFault() MOTEUS_CCM_ATTRIBUTE {
    motor_driver_->PowerOff();

    *pwm1_ccr_ = 0;
    *pwm2_ccr_ = 0;
    *pwm3_ccr_ = 0;

    status_.power_W = 0.0f;
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

  void ISR_DoPwmControl(const Vec3& pwm) MOTEUS_CCM_ATTRIBUTE {
    control_.pwm.a = LimitPwm(pwm.a);
    control_.pwm.b = LimitPwm(pwm.b);
    control_.pwm.c = LimitPwm(pwm.c);

    const uint16_t pwm1 = static_cast<uint16_t>(control_.pwm.a * pwm_counts_);
    const uint16_t pwm2 = static_cast<uint16_t>(control_.pwm.b * pwm_counts_);
    const uint16_t pwm3 = static_cast<uint16_t>(control_.pwm.c * pwm_counts_);

    // NOTE(jpieper): The default ordering has pwm2 and pwm3 flipped.
    // Why you may ask?  No good reason.  It does require that the
    // currents be similarly swapped in ISR_CalculateCurrentState.
    // Changing it back now would reverse the sign of position for any
    // existing motor, so it isn't an easy change to make.
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

  /// Assume that the voltages are intended to be balanced around the
  /// midpoint and can be shifted accordingly.
  void ISR_DoBalancedVoltageControl(const Vec3& voltage) MOTEUS_CCM_ATTRIBUTE {
    control_.voltage = voltage;

    const float bus_V = status_.filt_bus_V;
    const Vec3 pwm_in = {voltage.a / bus_V, voltage.b / bus_V, voltage.c / bus_V};

    const float pwmmin = std::min(pwm_in.a, std::min(pwm_in.b, pwm_in.c));
    const float pwmmax = std::max(pwm_in.a, std::max(pwm_in.b, pwm_in.c));

    // Balance the three phases so that the highest and lowest are
    // equidistant from the midpoint.  Note, this results in a
    // waveform that is identical to SVPWM, or min/max injection.
    const float offset = 0.5f * (pwmmin + pwmmax) - 0.5f;

    ISR_DoPwmControl(Vec3{
        pwm_in.a - offset,
        pwm_in.b - offset,
        pwm_in.c - offset});
  }

  void ISR_DoVoltageFOC(CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    data->theta += data->theta_rate * rate_config_.period_s;
    SinCos sc = cordic_(RadiansToQ31(data->theta));
    const float max_voltage = (0.5f - rate_config_.min_pwm) *
        status_.filt_bus_V * kSvpwmRatio;
    InverseDqTransform idt(sc, Limit(data->voltage, -max_voltage, max_voltage), 0);
    ISR_DoBalancedVoltageControl(Vec3{idt.a, idt.b, idt.c});
  }

  void ISR_DoCurrent(const SinCos& sin_cos, float i_d_A_in, float i_q_A_in,
                     float feedforward_velocity_rotor,
                     bool ignore_position_bounds) MOTEUS_CCM_ATTRIBUTE {
    if (motor_.poles == 0) {
      // We aren't configured yet.
      status_.mode = kFault;
      status_.fault = errc::kMotorNotConfigured;
      return;
    }
    if (!position_.theta_valid) {
      status_.mode = kFault;
      status_.fault = errc::kThetaInvalid;
      return;
    }

    auto limit_q_current = [&](std::pair<float, errc> in_pair) -> std::pair<float, errc> MOTEUS_CCM_ATTRIBUTE {
      const auto in = in_pair.first;
      if (ignore_position_bounds) { return {in, in_pair.second}; }

      if (!std::isnan(position_config_.position_max) &&
          position_.position > position_config_.position_max &&
          in > 0.0f) {
        // We derate the request in the direction that moves it
        // further outside the position limits.  This is mostly useful
        // when feedforward is applied, as otherwise, the position
        // limits could easily be exceeded.  Without feedforward, we
        // shouldn't really be trying to push outside the limits
        // anyhow.
        return std::make_pair(
            in *
            std::max(0.0f,
                     1.0f - (position_.position -
                             position_config_.position_max) /
                     config_.position_derate),
            errc::kLimitPositionBounds);
      }
      if (!std::isnan(position_config_.position_min) &&
          position_.position < position_config_.position_min &&
          in < 0.0f) {
        return std::make_pair(
            in *
            std::max(0.0f,
                     1.0f - (position_config_.position_min -
                             position_.position) /
                     config_.position_derate),
            errc::kLimitPositionBounds);
      }

      return in_pair;
    };

    auto limit_q_velocity = [&](std::pair<float, errc> in_pair) -> std::pair<float, errc> MOTEUS_CCM_ATTRIBUTE {
      const auto in = in_pair.first;
      const float abs_velocity = std::abs(position_.velocity);
      if (abs_velocity < config_.max_velocity ||
          position_.velocity * in < 0.0f) {
        return in_pair;
      }
      const float derate_fraction =
          1.0f - ((abs_velocity - config_.max_velocity) /
                  config_.max_velocity_derate);
      const float current_limit =
          std::max(0.0f, derate_fraction * config_.max_current_A);
      const errc maybe_fault = derate_fraction < 1.0f ? errc::kLimitMaxVelocity : errc::kLimitMaxCurrent;
      return LimitCode(in, -current_limit, current_limit,
                       maybe_fault, in_pair.second);
    };

    float derate_fraction =
        (status_.filt_fet_temp_C - derate_temperature_) /
        config_.temperature_margin;
    errc derate_fault = errc::kLimitMaxCurrent;
    if (derate_fraction > 0.0f) {
      derate_fault = errc::kLimitFetTemperature;
    }
    if (std::isfinite(config_.motor_fault_temperature)) {
      const float motor_derate =
          ((status_.filt_motor_temp_C - motor_derate_temperature_) /
           config_.motor_temperature_margin);
      if (motor_derate > derate_fraction) {
        derate_fraction = motor_derate;
        derate_fault = errc::kLimitMotorTemperature;
      }
    }

    const float derate_current_A =
        std::max<float>(
            0.0f,
            derate_fraction *
            (config_.derate_current_A - config_.max_current_A) +
            config_.max_current_A);

    const float temp_limit_A = std::min<float>(
        config_.max_current_A, derate_current_A);

    auto limit_either_current = [&](std::pair<float, errc> in_pair) MOTEUS_CCM_ATTRIBUTE {
      return LimitCode(in_pair.first, -temp_limit_A, temp_limit_A, derate_fault, in_pair.second);
    };


    const auto almost_i_q_A_pair =
        limit_either_current(
            limit_q_velocity(
                limit_q_current(
                    std::make_pair(i_q_A_in, errc::kSuccess))));
    const auto almost_i_d_A_pair = limit_either_current(
        std::make_pair(i_d_A_in, errc::kSuccess));

    const auto almost_i_q_A = almost_i_q_A_pair.first;
    const auto almost_i_d_A = almost_i_d_A_pair.first;
    const auto almost_fault =
        (almost_i_q_A_pair.second == errc::kSuccess) ?
        almost_i_d_A_pair.second : almost_i_q_A_pair.second;

    // Apply our power limits by limiting the maximum current command.
    // This has a feedback loop from the previous cycle's voltage
    // output, which is not ideal, but is what we've got.

    // Applying the limit here, rather than at the voltage stage has
    // proven to be more stable when activated under load.
    const float used_d_power_W = 1.5f * old_d_V * almost_i_d_A;
    const float used_q_power_W = 1.5f * old_q_V * almost_i_q_A;
    const float used_power = used_q_power_W + used_d_power_W;

    const auto [i_d_A, i_q_A, limit_code] = [&]() {
      // If we have slack, then no limiting needs to occur.
      if (std::abs(used_power) < status_.max_power_W) {
        return std::make_tuple(almost_i_d_A, almost_i_q_A, almost_fault);
      }

      // Scale both currents equally in power terms.
      const float scale = status_.max_power_W / std::abs(used_power);

      const float scaled_d_power = used_d_power_W * scale;
      const float scaled_q_power = used_q_power_W * scale;

      return std::make_tuple(
          scaled_d_power / (1.5f * old_d_V),
          scaled_q_power / (1.5f * old_q_V),
          errc::kLimitMaxPower);
    }();

    control_.i_d_A = i_d_A;
    control_.i_q_A = i_q_A;

    const float max_V =
        rate_config_.max_voltage_ratio * kSvpwmRatio *
        0.5f * status_.filt_bus_V;

    const auto limit_to_max_voltage = [max_V](float denorm_d_V, float denorm_q_V, errc inlimit) {
      const float max_V_sq = max_V * max_V;
      const float denorm_len =
          denorm_d_V * denorm_d_V + denorm_q_V * denorm_q_V;
      if (denorm_len < max_V_sq) {
        return std::make_tuple(denorm_d_V, denorm_q_V, inlimit);
      }

      const float scale = sqrtf(max_V_sq / denorm_len);
      return std::make_tuple(denorm_d_V * scale, denorm_q_V * scale, errc::kLimitMaxVoltage);
    };

    if (!config_.voltage_mode_control) {
      const float denorm_d_V =
          pid_d_.Apply(status_.d_A, i_d_A, rate_config_.rate_hz) +
          i_d_A * config_.current_feedforward * motor_.resistance_ohm;

      const float denorm_q_V =
          pid_q_.Apply(status_.q_A, i_q_A, rate_config_.rate_hz) +
          i_q_A * config_.current_feedforward * motor_.resistance_ohm +
          (feedforward_velocity_rotor *
           config_.bemf_feedforward *
           v_per_hz_);

      auto [d_V, q_V, final_limit_code] =
          limit_to_max_voltage(denorm_d_V, denorm_q_V, limit_code);

      if (final_limit_code != errc::kSuccess) {
        status_.fault = final_limit_code;
      }

      // We also limit the integral to be no more than the maximal
      // applied voltage for each phase independently.  This helps to
      // more quickly recover if things saturate in the case of no or
      // incomplete feedforward terms.
      status_.pid_d.integral = Limit(
          status_.pid_d.integral,
          -max_V, max_V);

      status_.pid_q.integral = Limit(
          status_.pid_q.integral,
          -max_V, max_V);

      // eq 2.28 from "DYNAMIC MODEL OF PM SYNCHRONOUS MOTORS" D. Ohm,
      // 2000
      status_.power_W =
          1.5f * (status_.d_A * d_V +
                  status_.q_A * q_V);

      ISR_DoVoltageDQ(sin_cos, d_V, q_V);
    } else {
      status_.power_W =
          1.5f * (i_d_A * i_d_A * motor_.resistance_ohm +
                  i_q_A * i_q_A * motor_.resistance_ohm);

      auto [d_V, q_V, final_limit_code] = limit_to_max_voltage(
          i_d_A * motor_.resistance_ohm,
          i_q_A * motor_.resistance_ohm +
          (feedforward_velocity_rotor *
           config_.bemf_feedforward *
           v_per_hz_),
          limit_code);

      if (final_limit_code != errc::kSuccess) {
        status_.fault = final_limit_code;
      }

      ISR_DoVoltageDQ(sin_cos, d_V, q_V);
    }
  }

  // The idiomatic thing to do in DoMeasureInductance would be to just
  // call DoVoltageDQ.  However, because of
  // https://gcc.gnu.org/bugzilla/show_bug.cgi?id=41091 that results
  // in a compile time error.  Instead, we construct a similar
  // factorization by delegating most of the work to this helper
  // function.
  Vec3 ISR_CalculatePhaseVoltage(const SinCos& sin_cos, float d_V, float q_V) MOTEUS_CCM_ATTRIBUTE {
    if (position_.epoch != isr_motor_position_epoch_) {
      status_.mode = kFault;
      status_.fault = errc::kConfigChanged;

      return Vec3{0.f, 0.f, 0.f};
    }

    control_.d_V = d_V;
    control_.q_V = q_V;

    InverseDqTransform idt(
        sin_cos, control_.d_V,
        motor_position_config()->output.sign * control_.q_V);

#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.control_done_cur = DWT->CYCCNT;
#endif

    return Vec3{idt.a, idt.b, idt.c};
  }

  void ISR_DoVoltageDQ(const SinCos& sin_cos, float d_V, float q_V) MOTEUS_CCM_ATTRIBUTE {
    ISR_DoBalancedVoltageControl(ISR_CalculatePhaseVoltage(sin_cos, d_V, q_V));
  }

  void ISR_DoVoltageDQCommand(const SinCos& sin_cos, float d_V, float q_V) MOTEUS_CCM_ATTRIBUTE {
    if (motor_.poles == 0) {
      // We aren't configured yet.
      status_.mode = kFault;
      status_.fault = errc::kMotorNotConfigured;
      return;
    }
    if (!position_.theta_valid) {
      status_.mode = kFault;
      status_.fault = errc::kThetaInvalid;
      return;
    }

    // We could limit maximum voltage further down the call stack in a
    // common place, however current mode control limits it
    // inherently, and is the most expensive of the control modes.
    // Thus all other users of CalculatePhaseVoltage are required to
    // limit voltage beforehand.
    const float max_V =
        rate_config_.max_voltage_ratio * kSvpwmRatio *
        0.5f * status_.filt_bus_V;

    ISR_DoBalancedVoltageControl(
        ISR_CalculatePhaseVoltage(
            sin_cos,
            Limit(d_V, -max_V, max_V),
            Limit(q_V, -max_V, max_V)));
  }

  void ISR_DoPositionTimeout(const SinCos& sin_cos, CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    if (config_.timeout_mode == kStopped) {
      ISR_DoStopped(sin_cos);
    } else if (config_.timeout_mode == kPosition) {
      CommandData timeout_data;
      timeout_data.mode = kPosition;
      timeout_data.position = std::numeric_limits<float>::quiet_NaN();
      timeout_data.velocity_limit = config_.default_velocity_limit;
      timeout_data.accel_limit = config_.default_accel_limit;
      timeout_data.timeout_s = std::numeric_limits<float>::quiet_NaN();

      PID::ApplyOptions apply_options;
      ISR_DoPositionCommon(
          sin_cos, &timeout_data, apply_options,
          timeout_data.max_torque_Nm,
          0.0f,
          0.0f);
    } else if (config_.timeout_mode == kZeroVelocity) {
      ISR_DoZeroVelocity(sin_cos, data);
    } else if (config_.timeout_mode == kBrake) {
      ISR_DoBrake();
    } else {
      ISR_DoStopped(sin_cos);
    }
  }

  void ISR_DoZeroVelocity(const SinCos& sin_cos, CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    CommandData zero_velocity;

    zero_velocity.mode = kPosition;
    zero_velocity.position = std::numeric_limits<float>::quiet_NaN();
    zero_velocity.velocity = 0.0f;
    zero_velocity.timeout_s = std::numeric_limits<float>::quiet_NaN();

    PID::ApplyOptions apply_options;
    apply_options.kp_scale = 0.0f;
    apply_options.kd_scale = data->kd_scale;
    apply_options.ilimit_scale = 0.0f;

    ISR_DoPositionCommon(sin_cos, &zero_velocity,
                         apply_options, config_.timeout_max_torque_Nm,
                         0.0f, 0.0f);
  }

  void ISR_DoPosition(const SinCos& sin_cos, CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    PID::ApplyOptions apply_options;
    apply_options.kp_scale = data->kp_scale;
    apply_options.kd_scale = data->kd_scale;
    apply_options.ilimit_scale = data->ilimit_scale;

    ISR_DoPositionCommon(sin_cos, data, apply_options, data->max_torque_Nm,
                         data->feedforward_Nm, data->velocity);
  }

  void ISR_DoPositionCommon(
      const SinCos& sin_cos, CommandData* data,
      const PID::ApplyOptions& pid_options,
      float max_torque_Nm,
      float feedforward_Nm,
      float velocity) MOTEUS_CCM_ATTRIBUTE {
    const int64_t absolute_relative_delta =
        static_cast<int64_t>(
            motor_position_->absolute_relative_delta.load()) << 32ll;

    const float velocity_command =
        BldcServoPosition::UpdateCommand(
            &status_,
            &config_,
            &position_config_,
            &position_,
            absolute_relative_delta,
            rate_config_.rate_hz,
            data,
            velocity);

    // At this point, our control position and velocity are known.

    if (config_.fixed_voltage_mode ||
        !std::isnan(data->fixed_voltage_override) ||
        !std::isnan(data->fixed_current_override)) {
      status_.position =
          static_cast<float>(
              static_cast<int32_t>(
                  *status_.control_position_raw >> 32)) /
          65536.0f;
      status_.velocity = velocity_command;

      // For "fixed voltage" and "fixed current" mode, we skip all
      // position PID loops and all their associated calculations,
      // including everything that uses the encoder, further for
      // "fixed voltage" mode we also skip the current control loop.
      //
      // In either case, we just burn power with a fixed voltage or
      // current drive based on the desired position.
      if (!std::isnan(data->fixed_current_override)) {
        ISR_DoCurrent(sin_cos,
                      data->fixed_current_override,
                      0.0f, 0.0f, data->ignore_position_bounds);
      } else {
        const float fixed_voltage =
            std::isnan(data->fixed_voltage_override) ?
            config_.fixed_voltage_control_V +
            (std::abs(status_.velocity) *
             v_per_hz_ *
             config_.bemf_feedforward) :
            data->fixed_voltage_override;
        ISR_DoVoltageDQ(sin_cos, fixed_voltage, 0.0f);
      }
      return;
    }

    // From this point, we require actual valid position.
    if (!position_.position_relative_valid) {
      status_.mode = kFault;
      status_.fault = errc::kPositionInvalid;
      return;
    }
    if (position_.error != MotorPosition::Status::kNone) {
      status_.mode = kFault;
      status_.fault = errc::kEncoderFault;
      return;
    }

    const float measured_velocity = velocity_command +
        Threshold(
            position_.velocity - velocity_command, -config_.velocity_threshold,
            config_.velocity_threshold);

    // We always control relative to the control position of 0, so
    // that we get equal performance across the entire viable integral
    // position range.
    const float unlimited_torque_Nm =
        (pid_position_.Apply(
            (static_cast<int32_t>(
                (position_.position_relative_raw -
                 *status_.control_position_raw) >> 32) /
             65536.0f),
            0.0,
            measured_velocity, velocity_command,
            rate_config_.rate_hz,
            pid_options) +
         feedforward_Nm);

    const auto limited_torque_Nm_pair =
        LimitCode(unlimited_torque_Nm, -max_torque_Nm, max_torque_Nm,
                  errc::kLimitMaxTorque, errc::kSuccess);
    const auto limited_torque_Nm = limited_torque_Nm_pair.first;
    if (limited_torque_Nm_pair.second != errc::kSuccess) {
      status_.fault = limited_torque_Nm_pair.second;
    }

    control_.torque_Nm = limited_torque_Nm;
    status_.torque_error_Nm = status_.torque_Nm - control_.torque_Nm;

    const float limited_q_A =
        torque_to_current(limited_torque_Nm *
                          motor_position_->config()->rotor_to_output_ratio);

    {
      const auto& pos_config = motor_position_->config();
      const auto commutation_source = pos_config->commutation_source;
      const float cpr = static_cast<float>(
          pos_config->sources[commutation_source].cpr);
      const float commutation_position =
          position_.sources[commutation_source].filtered_value / cpr;

      auto sample =
          [&](const auto& table, float scale) {
            const int left_index = std::min<int>(
                table.size() - 1,
                static_cast<int>(table.size() * commutation_position));
            const int right_index = (left_index + 1) % table.size();
            const float comp_fraction =
                (commutation_position -
                 static_cast<float>(left_index) / table.size()) *
                static_cast<float>(table.size());
            const float left_comp = table[left_index] * scale;
            const float right_comp = table[right_index] * scale;

            return (right_comp - left_comp) * comp_fraction + left_comp;
          };
      const float q_comp_A = sample(motor_.cogging_dq_comp,
                                    motor_.cogging_dq_scale);

      control_.q_comp_A = q_comp_A;
    }

    const float compensated_q_A = limited_q_A + control_.q_comp_A;

    const float q_A =
        is_torque_constant_configured() ?
        compensated_q_A :
        Limit(compensated_q_A, -kMaxUnconfiguredCurrent, kMaxUnconfiguredCurrent);

    const float d_A = [&]() MOTEUS_CCM_ATTRIBUTE {
      const auto error = (
          status_.filt_1ms_bus_V - flux_brake_min_voltage_);

      if (error <= 0.0f) {
        return 0.0f;
      }

      return (error / config_.flux_brake_resistance_ohm);
    }();

#ifdef MOTEUS_PERFORMANCE_MEASURE
    status_.dwt.control_done_pos = DWT->CYCCNT;
#endif

    ISR_DoCurrent(
        sin_cos, d_A, q_A,
        velocity_command / motor_position_->config()->rotor_to_output_ratio,
        data->ignore_position_bounds);
  }

  void ISR_DoStayWithinBounds(const SinCos& sin_cos, CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    const auto target_position = [&]() MOTEUS_CCM_ATTRIBUTE -> std::optional<float> {
      if (!std::isnan(data->bounds_min) &&
          position_.position < data->bounds_min) {
        return data->bounds_min;
      }
      if (!std::isnan(data->bounds_max) &&
          position_.position > data->bounds_max) {
        return data->bounds_max;
      }
      return {};
    }();

    if (!target_position) {
      status_.pid_position.Clear();
      status_.control_position_raw = {};
      status_.control_position = std::numeric_limits<float>::quiet_NaN();
      status_.control_velocity = {};

      // In this region, we still apply feedforward torques if they
      // are present.
      PID::ApplyOptions apply_options;
      apply_options.kp_scale = 0.0;
      apply_options.kd_scale = 0.0;
      apply_options.ilimit_scale = 0.0;

      ISR_DoPositionCommon(
          sin_cos, data, apply_options,
          data->max_torque_Nm, data->feedforward_Nm, 0.0f);
      return;
    }

    // Control position to whichever bound we are currently violating.
    PID::ApplyOptions apply_options;
    apply_options.kp_scale = data->kp_scale;
    apply_options.kd_scale = data->kd_scale;
    apply_options.ilimit_scale = data->ilimit_scale;

    const int64_t absolute_relative_delta =
        (static_cast<int64_t>(
            motor_position_->absolute_relative_delta.load()) << 32ll);
    data->position_relative_raw =
        MotorPosition::FloatToInt(*target_position) -
        absolute_relative_delta;
    data->velocity = 0.0;
    status_.control_position_raw = data->position_relative_raw;
    status_.control_position = *target_position;
    status_.control_velocity = 0.0f;

    ISR_DoPositionCommon(
        sin_cos, data, apply_options,
        data->max_torque_Nm, data->feedforward_Nm, 0.0f);
  }

  void ISR_DoMeasureInductance(const SinCos& sin_cos, CommandData* data) MOTEUS_CCM_ATTRIBUTE {
    // While we do use the sin_cos here, it doesn't really matter as
    // this should only be done with the motor stationary.  Thus we
    // won't bother checking if the motor is configured or the encoder
    // is valid.
    //
    // Newer moteus_tool will probably use the fixed_voltage_override
    // method anyways, which will force the sin_cos to be valid
    // regardless of encoder status.

    const int8_t old_sign = status_.meas_ind_phase > 0 ? 1 : -1;
    const float old_sign_float = old_sign > 0 ? 1.0f : -1.0f;

    status_.meas_ind_phase += -old_sign;

    // When measuring inductance, we just drive a 0 centered square
    // wave at some integral multiple of the control period.
    if (status_.meas_ind_phase == 0) {
      status_.meas_ind_phase = -old_sign * data->meas_ind_period;
    }

    const float offset = std::isfinite(data->fixed_voltage_override) ?
        data->fixed_voltage_override :
        0.0f;

    // We could limit maximum voltage further down the call stack in a
    // common place, however current mode control limits it
    // inherently, and is the most expensive of the control modes.
    // Thus all other users of CalculatePhaseVoltage are required to
    // limit voltage beforehand.
    const float max_V = (0.5f - rate_config_.min_pwm) *
        status_.filt_bus_V * kSvpwmRatio;

    const float d_V =
        Limit(
            offset +
            data->d_V * (status_.meas_ind_phase > 0 ? 1.0f : -1.0f),
            -max_V,
            max_V);

    // We also integrate the difference in current.
    status_.meas_ind_integrator +=
        (status_.d_A - status_.meas_ind_old_d_A) *
        old_sign_float;
    status_.meas_ind_old_d_A = status_.d_A;

    ISR_DoBalancedVoltageControl(ISR_CalculatePhaseVoltage(sin_cos, d_V, 0.0f));
  }

  void ISR_DoBrake() MOTEUS_CCM_ATTRIBUTE {
    *pwm1_ccr_ = 0;
    *pwm2_ccr_ = 0;
    *pwm3_ccr_ = 0;

    motor_driver_->PowerOn();
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

    // We rely on the FIFO to queue these things up.
    for (int i = 0; i < pos; i++) {
      debug_uart_->TDR = debug_buf_[i];
    }
  }

  float LimitPwm(float in) MOTEUS_CCM_ATTRIBUTE {
    // We can't go full duty cycle or we wouldn't have time to sample
    // the current.
    return Limit(in, rate_config_.min_pwm, rate_config_.max_pwm);
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

  float old_d_V = 0.0f;
  float old_q_V = 0.0f;

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
