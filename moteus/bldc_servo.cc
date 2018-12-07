// Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include <cmath>
#include <functional>

#include "mbed.h"
#include "PeripheralPins.h"

#include "mjlib/base/assert.h"
#include "mjlib/base/windowed_average.h"

#include "moteus/irq_callback_table.h"
#include "moteus/foc.h"
#include "moteus/math.h"

namespace micro = mjlib::micro;

namespace moteus {

namespace {

float Limit(float a, float min, float max) {
  if (a < min) { return min; }
  if (a > max) { return max; }
  return a;
}

constexpr float kRateHz = 40000.0;

// mbed seems to configure the Timer clock input to 90MHz.  We want
// 80kHz up/down rate for 40kHz freqency, so:
constexpr uint32_t kPwmCounts = 90000000 / 80000;

IRQn_Type FindUpdateIrq(TIM_TypeDef* timer) {
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
}

class BldcServo::Impl {
 public:
  Impl(micro::PersistentConfig* persistent_config,
       micro::TelemetryManager* telemetry_manager,
       PositionSensor* position_sensor,
       const Options& options)
      : options_(options),
        position_sensor_(position_sensor),
        pwm1_(options.pwm1),
        pwm2_(options.pwm2),
        pwm3_(options.pwm3),
        current1_(options.current1),
        current2_(options.current2),
        vsense_(options.vsense),
        debug_out_(options.debug_out) {

    persistent_config->Register("servo", &config_,
                                std::bind(&Impl::UpdateConfig, this));
    telemetry_manager->Register("servo_stats", &status_);
    telemetry_manager->Register("servo_cmd", &telemetry_data_);
    telemetry_manager->Register("servo_control", &control_);

    MJ_ASSERT(!g_impl_);
    g_impl_ = this;

    ConfigureADC();
    ConfigureTimer();
  }

  ~Impl() {
    g_impl_ = nullptr;
  }

  void Command(const CommandData& data) {
    // We can't enter current mode if we haven't zeroed our offsets
    // yet.

    if (!status_.zero_applied && (data.mode == kFoc || data.mode == kPosition)) {
      // TODO(jpieper): Flag this somehow in an error bit.
      return;
    }

    // Actually setting values will happen in the interrupt routine,
    // so we need to update this atomically.
    CommandData* next = next_data_;
    *next = data;

    telemetry_data_ = data;

    std::swap(current_data_, next_data_);
  }

  void ZeroOffset() {
    // We can only zero the offset if we are currently disabled.
    if (current_data_->mode != kDisabled) { return; }

    (*pwm1_ccr_) = 0;
    (*pwm2_ccr_) = 0;
    (*pwm3_ccr_) = 0;

    wait_us(100);

    // Now disable the timer for the duration of our sampling.
    timer_->CR1 &= ~(TIM_CR1_CEN);

    auto sample_adc = []() {
      ADC1->CR2 |= ADC_CR2_SWSTART;

      while ((ADC1->SR & ADC_SR_EOC) == 0);

      return std::make_pair(ADC1->DR, ADC2->DR);
    };

    std::pair<uint32_t, uint32_t> total = { 0, 0 };
    constexpr int kSampleCount = 256;
    for (int i = 0; i < kSampleCount; i++) {
      const auto this_sample = sample_adc();
      total = {total.first + this_sample.first,
               total.second + this_sample.second};
    }

    // We require that the results be somewhat close to the expected
    // value, to prevent doing a calibration when the driver is
    // entirely off.
    const uint16_t new_offset1 = total.first / kSampleCount;
    const uint16_t new_offset2 = total.second / kSampleCount;

    if (std::abs(static_cast<int>(new_offset1) - 2048) < 200 &&
        std::abs(static_cast<int>(new_offset2) - 2048) < 200) {
      status_.adc1_offset = new_offset1;
      status_.adc2_offset = new_offset2;
      status_.zero_applied = true;
    }

    // Start the timer back up again.
    timer_->CR1 |= TIM_CR1_CEN;
  }

  Status status() const { return status_; }

  void UpdateConfig() {
  }

 private:
  void ConfigureTimer() {
    const auto pwm1_timer = pinmap_peripheral(options_.pwm1, PinMap_PWM);
    const auto pwm2_timer = pinmap_peripheral(options_.pwm2, PinMap_PWM);
    const auto pwm3_timer = pinmap_peripheral(options_.pwm3, PinMap_PWM);

    // All three must be the same and be valid.
    MJ_ASSERT(pwm1_timer != 0 &&
                pwm1_timer == pwm2_timer &&
                pwm2_timer == pwm3_timer);
    timer_ = reinterpret_cast<TIM_TypeDef*>(pwm1_timer);


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

    // NOTE: We don't use IrqCallbackTable here because we need the
    // absolute minimum latency possible.
    const auto irqn = FindUpdateIrq(timer_);
    NVIC_SetVector(irqn, reinterpret_cast<uint32_t>(&Impl::GlobalInterrupt));
    NVIC_SetPriority(irqn, 0);
    NVIC_EnableIRQ(irqn);

    // Reinitialize the counter and update all registers.
    timer_->EGR |= TIM_EGR_UG;

    // Finally, enable the timer.
    timer_->CR1 |= TIM_CR1_CEN;
  }

  void ConfigureADC() {
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
    // inputs, however they won't
    ADC1->SQR3 = FindSqr(options_.current1);
    ADC2->SQR3 = FindSqr(options_.current2);
    ADC3->SQR3 = FindSqr(options_.vsense);

    MJ_ASSERT(reinterpret_cast<uint32_t>(ADC1) ==
                pinmap_peripheral(options_.current1, PinMap_ADC));
    MJ_ASSERT(reinterpret_cast<uint32_t>(ADC2) ==
                pinmap_peripheral(options_.current2, PinMap_ADC));
    MJ_ASSERT(reinterpret_cast<uint32_t>(ADC3) ==
                pinmap_peripheral(options_.vsense, PinMap_ADC));

    // Set sample times to 15 cycles across the board
    constexpr uint32_t kCycles = 0x1;  // 15 cycles
    constexpr uint32_t kAll15Cycles =
        (kCycles << 0) |
        (kCycles << 3) |
        (kCycles << 6) |
        (kCycles << 9) |
        (kCycles << 12) |
        (kCycles << 15) |
        (kCycles << 18) |
        (kCycles << 21) |
        (kCycles << 24);
    ADC1->SMPR1 = kAll15Cycles;
    ADC1->SMPR2 = kAll15Cycles;
    ADC2->SMPR1 = kAll15Cycles;
    ADC2->SMPR2 = kAll15Cycles;
    ADC3->SMPR1 = kAll15Cycles;
    ADC3->SMPR2 = kAll15Cycles;
  }

  // CALLED IN INTERRUPT CONTEXT.
  static void GlobalInterrupt() {
    g_impl_->ISR_HandleTimer();
  }

  // CALLED IN INTERRUPT CONTEXT.
  void ISR_HandleTimer() {

    if ((timer_->SR & TIM_SR_UIF) &&
        (timer_->CR1 & TIM_CR1_DIR)) {
      debug_out_ = 1;

      // Start conversion.
      ADC1->CR2 |= ADC_CR2_SWSTART;

      while ((ADC1->SR & ADC_SR_EOC) == 0);

      status_.adc1_raw = ADC1->DR;
      status_.adc2_raw = ADC2->DR;
      status_.adc3_raw = ADC3->DR;

      // We are now out of the most time critical portion of the ISR,
      // although it is still all pretty time critical since it runs
      // at 40kHz.  But time spent until now actually limits the
      // maximum duty cycle we can achieve, whereas time spent below
      // just eats cycles the rest of the code could be using.

      // Sample the position.
      const uint16_t old_position_raw = status_.position_raw;
      status_.position_raw = position_sensor_->Sample();

      status_.electrical_theta =
          k2Pi * ::fmodf(
              (static_cast<float>(status_.position_raw) / 65536.0f *
               (config_.motor_poles / 2.0f)) - config_.motor_offset, 1.0f);
      const int16_t delta_position =
          static_cast<int16_t>(status_.position_raw - old_position_raw);
      status_.unwrapped_position_raw += delta_position;
      velocity_filter_.Add(delta_position * config_.unwrapped_position_scale *
                           (1.0f / 65536.0f) * kRateHz);
      status_.velocity = velocity_filter_.average();

      status_.unwrapped_position =
          status_.unwrapped_position_raw * config_.unwrapped_position_scale *
          (1.0f / 65536.0f);

      SinCos sin_cos{status_.electrical_theta};

      ISR_CalculateCurrentState(sin_cos);

      ISR_DoControl(sin_cos);

      debug_out_ = 0;
    }

    // Reset the status register.
    timer_->SR = 0x00;
  }

  // This is called from the ISR.
  void ISR_CalculateCurrentState(const SinCos& sin_cos) {
    status_.cur1_A = (status_.adc1_raw - status_.adc1_offset) * config_.i_scale_A;
    status_.cur2_A = (status_.adc2_raw - status_.adc2_offset) * config_.i_scale_A;
    status_.bus_V = status_.adc3_raw * config_.v_scale_V;

    DqTransform dq{sin_cos,
          status_.cur1_A,
          0.0f - (status_.cur1_A + status_.cur2_A),
          status_.cur2_A
          };
    status_.d_A = dq.d;
    status_.q_A = dq.q;
  }

  void ISR_DoControl(const SinCos& sin_cos) {
    // current_data_ is volatile, so read it out now, and operate on
    // the pointer for the rest of the routine.
    CommandData* data = current_data_;

    control_ = {};

    if (data->mode != kFoc && data->mode != kPosition) {
      // If we are not running PID controllers, keep their state
      // zeroed out.
      status_.pid_d = {};
      status_.pid_q = {};
    }

    switch (data->mode) {
      case kNumModes:  // fall-through
      case kDisabled: {
        *pwm1_ccr_ = 0;
        *pwm2_ccr_ = 0;
        *pwm3_ccr_ = 0;
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
      case kFoc: {
        ISR_DoFOC(sin_cos, data->i_d_A, data->i_q_A);
        break;
      }
      case kPosition: {
        ISR_DoPosition(sin_cos, data->position,
                       data->velocity, data->max_current);
        break;
      }
    }
  }

  void ISR_DoPwmControl(const Vec3& pwm) {
    control_.pwm.a = LimitPwm(pwm.a);
    control_.pwm.b = LimitPwm(pwm.b);
    control_.pwm.c = LimitPwm(pwm.c);

    *pwm1_ccr_ = static_cast<uint16_t>(control_.pwm.a * kPwmCounts);
    *pwm3_ccr_ = static_cast<uint16_t>(control_.pwm.b * kPwmCounts);
    *pwm2_ccr_ = static_cast<uint16_t>(control_.pwm.c * kPwmCounts);
  }

  void ISR_DoVoltageControl(const Vec3& voltage) {
    control_.voltage = voltage;

    auto voltage_to_pwm = [this](float v) {
      return 0.5f + 2.0f * v / status_.bus_V;
    };

    ISR_DoPwmControl(Vec3{
        voltage_to_pwm(voltage.a),
            voltage_to_pwm(voltage.b),
            voltage_to_pwm(voltage.c)});
  }

  void ISR_DoVoltageFOC(float theta, float voltage) {
    SinCos sc(theta);
    InverseDqTransform idt(sc, 0, voltage);
    ISR_DoVoltageControl(Vec3{idt.a, idt.b, idt.c});
  }

  void ISR_DoFOC(const SinCos& sin_cos, float i_d_A, float i_q_A) {
    control_.i_d_A = i_d_A;
    control_.i_q_A = i_q_A;

    const float d_V = pid_d_.Apply(status_.d_A, i_d_A, 0.0f, 0.0f, kRateHz);
    const float q_V = pid_q_.Apply(status_.q_A, i_q_A, 0.0f, 0.0f, kRateHz);

    InverseDqTransform idt(sin_cos, d_V, q_V);

    ISR_DoVoltageControl(Vec3{idt.a, idt.b, idt.c});
  }

  void ISR_DoPosition(const SinCos& sin_cos, float position, float velocity, float max_current) {
    const float measured_velocity = status_.velocity;

    const float unlimited_d_A =
        pid_position_.Apply(status_.unwrapped_position, position,
                            measured_velocity, velocity,
                            kRateHz);
    const float d_A = Limit(unlimited_d_A, -max_current, max_current);

    ISR_DoFOC(sin_cos, d_A, 0.0f);
  }

  float LimitPwm(float in) {
    // We can't go full duty cycle or we wouldn't have time to sample
    // the current.
    return Limit(in, 0.1f, 0.9f);
  }

  const Options options_;
  PositionSensor* const position_sensor_;

  Config config_;
  TIM_TypeDef* timer_ = nullptr;
  ADC_TypeDef* const adc1_ = ADC1;
  ADC_TypeDef* const adc2_ = ADC2;
  ADC_TypeDef* const adc3_ = ADC3;

  // We create these to initialize our pins as output and PWM mode,
  // but otherwise don't use them.
  PwmOut pwm1_;
  PwmOut pwm2_;
  PwmOut pwm3_;

  volatile uint32_t* pwm1_ccr_ = nullptr;
  volatile uint32_t* pwm2_ccr_ = nullptr;
  volatile uint32_t* pwm3_ccr_ = nullptr;

  AnalogIn current1_;
  AnalogIn current2_;
  AnalogIn vsense_;

  // This is just for debugging.
  DigitalOut debug_out_;

  CommandData data_buffers_[2] = {};

  // CommandData has its data updated to the ISR by first writing the
  // new command into (*next_data_) and then swapping it with
  // current_data_.
  CommandData* volatile current_data_{&data_buffers_[0]};
  CommandData* volatile next_data_{&data_buffers_[1]};

  // This copy of CommandData exists solely for telemetry, and should
  // never be read by an ISR.
  CommandData telemetry_data_;

  mjlib::base::WindowedAverage<float, 32> velocity_filter_;
  Status status_;
  Control control_;

  mjlib::base::PID pid_d_{&config_.pid_dq, &status_.pid_d};
  mjlib::base::PID pid_q_{&config_.pid_dq, &status_.pid_q};
  mjlib::base::PID pid_position_{&config_.pid_position, &status_.pid_position};

  static Impl* g_impl_;
};

BldcServo::Impl* BldcServo::Impl::g_impl_ = nullptr;

BldcServo::BldcServo(micro::Pool* pool,
                     micro::PersistentConfig* persistent_config,
                     micro::TelemetryManager* telemetry_manager,
                     PositionSensor* position_sensor,
                     const Options& options)
    : impl_(pool,
            persistent_config, telemetry_manager, position_sensor, options) {}
BldcServo::~BldcServo() {}

void BldcServo::Command(const CommandData& data) {
  impl_->Command(data);
}

void BldcServo::ZeroOffset() {
  impl_->ZeroOffset();
}

BldcServo::Status BldcServo::status() const {
  return impl_->status();
}

}
