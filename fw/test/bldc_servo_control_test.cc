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

#include "fw/bldc_servo_control.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace tt = boost::test_tools;

namespace {

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

struct Context : public BldcServoControl<Context> {
  // State members accessed by BldcServoControl via impl_.
  BldcServoStatus status_;
  BldcServoControl_Control control_;
  BldcServoConfig config_;
  BldcServoPositionConfig position_config_;
  MotorPosition::Status position_;
  BldcServoMotor motor_;

  SimplePI::Config pid_dq_config;
  PID::Config pid_position_config;

  SimplePI pid_d_{&pid_dq_config, &status_.pid_d};
  SimplePI pid_q_{&pid_dq_config, &status_.pid_q};
  PID pid_position_{&pid_position_config, &status_.pid_position};

  RateConfig rate_config_{30000, 15000};

  float torque_constant_ = 0.1f;
  float v_per_hz_ = 0.0f;
  float flux_brake_min_voltage_ = 0.0f;
  float derate_temperature_ = 50.0f;
  float motor_derate_temperature_ = 80.0f;
  uint8_t isr_motor_position_epoch_ = 0;
  uint32_t pwm_counts_ = 4000;
  float old_d_V_ = 0.0f;
  float old_q_V_ = 0.0f;

  // HW mock state for verification.
  int pwm_control_count = 0;
  Vec3 last_pwm;

  int hard_stop_count = 0;
  int fault_count = 0;
  int calibrating_count = 0;
  int brake_count = 0;
  int start_calibrating_count = 0;

  bool fault_state = false;

  MotorPosition::Config motor_position_config_val;
  int64_t absolute_relative_delta_val = 0;

  // HW methods required by BldcServoControl.
  void DoPwmControl(const Vec3& pwm) {
    pwm_control_count++;
    last_pwm = pwm;
  }

  void DoHardStop() {
    hard_stop_count++;
  }

  void DoFault() {
    fault_count++;
  }

  void DoCalibrating() {
    calibrating_count++;
  }

  void DoBrake() {
    brake_count++;
  }

  void StartCalibrating() {
    start_calibrating_count++;
  }

  bool motor_driver_fault() const {
    return fault_state;
  }

  SinCos cordic(int32_t radians_q31) const {
    // Simple sin/cos implementation for tests.
    float theta = static_cast<float>(radians_q31) * kPi *
                  (1.0f / 2147483648.0f);
    SinCos result;
    result.s = std::sin(theta);
    result.c = std::cos(theta);
    return result;
  }

  const MotorPosition::Config* motor_position_config() const {
    return &motor_position_config_val;
  }

  int64_t absolute_relative_delta() const {
    return absolute_relative_delta_val;
  }

  Context() {
    position_config_.position_min = NaN;
    position_config_.position_max = NaN;
    pid_dq_config.kp = 0.005f;
    pid_dq_config.ki = 30.0f;
    pid_position_config.kp = 4.0f;
    pid_position_config.ki = 1.0f;
    pid_position_config.kd = 0.05f;
    pid_position_config.sign = -1.0f;
  }
};

using TestControl = BldcServoControl<Context>;

}  // namespace

BOOST_AUTO_TEST_CASE(BldcServoControlBasic) {
  Context ctx;
  // Just verify construction works.
  BOOST_TEST(ctx.is_torque_constant_configured() == false);

  ctx.motor_.Kv = 380.0f;
  BOOST_TEST(ctx.is_torque_constant_configured() == true);
}

BOOST_AUTO_TEST_CASE(BldcServoControlCurrentControl) {
  Context ctx;

  // Stopped mode: no current control.
  ctx.status_.mode = kStopped;
  BOOST_TEST(ctx.current_control() == false);

  // Current mode: yes.
  ctx.status_.mode = kCurrent;
  BOOST_TEST(ctx.current_control() == true);

  // Position mode: yes.
  ctx.status_.mode = kPosition;
  BOOST_TEST(ctx.current_control() == true);

  // ZeroVelocity: yes.
  ctx.status_.mode = kZeroVelocity;
  BOOST_TEST(ctx.current_control() == true);

  // Voltage modes: no.
  ctx.status_.mode = kVoltageDq;
  BOOST_TEST(ctx.current_control() == false);

  ctx.status_.mode = kVoltageFoc;
  BOOST_TEST(ctx.current_control() == false);

  // PositionTimeout depends on timeout_mode.
  ctx.status_.mode = kPositionTimeout;
  ctx.config_.timeout_mode = kStopped;
  BOOST_TEST(ctx.current_control() == false);

  ctx.config_.timeout_mode = kZeroVelocity;
  BOOST_TEST(ctx.current_control() == true);

  ctx.config_.timeout_mode = kPosition;
  BOOST_TEST(ctx.current_control() == true);

  ctx.config_.timeout_mode = kBrake;
  BOOST_TEST(ctx.current_control() == false);
}

BOOST_AUTO_TEST_CASE(BldcServoControlTorqueOn) {
  Context ctx;

  ctx.status_.mode = kStopped;
  BOOST_TEST(ctx.torque_on() == false);

  ctx.status_.mode = kFault;
  BOOST_TEST(ctx.torque_on() == false);

  ctx.status_.mode = kPwm;
  BOOST_TEST(ctx.torque_on() == true);

  ctx.status_.mode = kCurrent;
  BOOST_TEST(ctx.torque_on() == true);

  ctx.status_.mode = kPosition;
  BOOST_TEST(ctx.torque_on() == true);

  ctx.status_.mode = kBrake;
  BOOST_TEST(ctx.torque_on() == true);

  // PositionTimeout depends on timeout_mode.
  ctx.status_.mode = kPositionTimeout;
  ctx.config_.timeout_mode = 0;  // kStopped
  BOOST_TEST(ctx.torque_on() == false);

  ctx.config_.timeout_mode = kZeroVelocity;
  BOOST_TEST(ctx.torque_on() == true);
}

BOOST_AUTO_TEST_CASE(BldcServoControlIsOutsideLimits) {
  Context ctx;

  // No limits set (NaN): never outside.
  ctx.position_.position = 100.0f;
  BOOST_TEST(ctx.ISR_IsOutsideLimits() == false);

  // Set limits.
  ctx.position_config_.position_min = -1.0f;
  ctx.position_config_.position_max = 1.0f;

  ctx.position_.position = 0.5f;
  BOOST_TEST(ctx.ISR_IsOutsideLimits() == false);

  ctx.position_.position = 1.5f;
  BOOST_TEST(ctx.ISR_IsOutsideLimits() == true);

  ctx.position_.position = -1.5f;
  BOOST_TEST(ctx.ISR_IsOutsideLimits() == true);

  ctx.position_.position = -1.0f;
  BOOST_TEST(ctx.ISR_IsOutsideLimits() == false);
}

BOOST_AUTO_TEST_CASE(BldcServoControlInvalidLimits) {
  Context ctx;

  // NaN limits are not invalid.
  BOOST_TEST(ctx.ISR_InvalidLimits() == false);

  ctx.position_config_.position_min = -1.0f;
  ctx.position_config_.position_max = 1.0f;
  BOOST_TEST(ctx.ISR_InvalidLimits() == false);

  // Limits with magnitude > 32768 are invalid.
  ctx.position_config_.position_min = -40000.0f;
  BOOST_TEST(ctx.ISR_InvalidLimits() == true);

  ctx.position_config_.position_min = -1.0f;
  ctx.position_config_.position_max = 40000.0f;
  BOOST_TEST(ctx.ISR_InvalidLimits() == true);
}

BOOST_AUTO_TEST_CASE(BldcServoControlLimitPwm) {
  Context ctx;

  // Values within range should pass through.
  BOOST_TEST(ctx.LimitPwm(0.5f) == 0.5f, tt::tolerance(1e-6f));

  // Values below min_pwm should be clamped.
  BOOST_TEST(ctx.LimitPwm(0.0f) == ctx.rate_config_.min_pwm,
             tt::tolerance(1e-6f));

  // Values above max_pwm should be clamped.
  BOOST_TEST(ctx.LimitPwm(1.0f) == ctx.rate_config_.max_pwm,
             tt::tolerance(1e-6f));
}

BOOST_AUTO_TEST_CASE(BldcServoControlTorqueConversion) {
  Context ctx;
  ctx.torque_constant_ = 0.1f;

  // Basic torque/current round-trip.
  const float torque = 1.0f;
  const float current = ctx.torque_to_current(torque);
  const float torque_back = ctx.current_to_torque(current);
  BOOST_TEST(torque_back == torque, tt::tolerance(1e-4f));

  // Zero torque.
  BOOST_TEST(ctx.torque_to_current(0.0f) == 0.0f,
             tt::tolerance(1e-6f));
  BOOST_TEST(ctx.current_to_torque(0.0f) == 0.0f,
             tt::tolerance(1e-6f));
}

BOOST_AUTO_TEST_CASE(BldcServoControlClearPid) {
  Context ctx;

  // Set some state in the PIDs.
  ctx.status_.pid_d.integral = 1.0f;
  ctx.status_.pid_q.integral = 2.0f;
  ctx.status_.pid_position.integral = 3.0f;
  ctx.status_.control_position_raw = 12345;
  ctx.status_.control_velocity = 1.0f;

  // In stopped mode with no cooldown, kClearIfMode should clear everything.
  ctx.status_.mode = kStopped;
  ctx.status_.cooldown_count = 0;
  ctx.ISR_ClearPid(TestControl::kClearIfMode);

  BOOST_TEST(ctx.status_.pid_d.integral == 0.0f);
  BOOST_TEST(ctx.status_.pid_q.integral == 0.0f);
  BOOST_TEST(ctx.status_.pid_d.desired == 0.0f);
  BOOST_TEST(ctx.status_.pid_q.desired == 0.0f);
  BOOST_TEST(ctx.status_.pid_position.integral == 0.0f);
  BOOST_TEST(!ctx.status_.control_position_raw.has_value());
  BOOST_TEST(!ctx.status_.control_velocity.has_value());

  // In current mode, kClearIfMode should NOT clear current PID but
  // SHOULD clear position PID.
  ctx.status_.pid_d.integral = 1.0f;
  ctx.status_.pid_q.integral = 2.0f;
  ctx.status_.pid_position.integral = 3.0f;
  ctx.status_.control_position_raw = 12345;
  ctx.status_.control_velocity = 1.0f;
  ctx.status_.mode = kCurrent;
  ctx.ISR_ClearPid(TestControl::kClearIfMode);

  BOOST_TEST(ctx.status_.pid_d.integral == 1.0f);
  BOOST_TEST(ctx.status_.pid_q.integral == 2.0f);
  BOOST_TEST(ctx.status_.pid_position.integral == 0.0f);
  BOOST_TEST(!ctx.status_.control_position_raw.has_value());

  // In position mode, kClearIfMode should clear neither.
  ctx.status_.pid_d.integral = 1.0f;
  ctx.status_.pid_q.integral = 2.0f;
  ctx.status_.pid_position.integral = 3.0f;
  ctx.status_.control_position_raw = 12345;
  ctx.status_.control_velocity = 1.0f;
  ctx.status_.mode = kPosition;
  ctx.ISR_ClearPid(TestControl::kClearIfMode);

  BOOST_TEST(ctx.status_.pid_d.integral == 1.0f);
  BOOST_TEST(ctx.status_.pid_q.integral == 2.0f);
  BOOST_TEST(ctx.status_.pid_position.integral == 3.0f);
  BOOST_TEST(ctx.status_.control_position_raw.has_value());

  // kAlwaysClear should always clear.
  ctx.status_.mode = kPosition;
  ctx.ISR_ClearPid(TestControl::kAlwaysClear);

  BOOST_TEST(ctx.status_.pid_d.integral == 0.0f);
  BOOST_TEST(ctx.status_.pid_q.integral == 0.0f);
  BOOST_TEST(ctx.status_.pid_position.integral == 0.0f);
  BOOST_TEST(!ctx.status_.control_position_raw.has_value());
}

BOOST_AUTO_TEST_CASE(BldcServoControlClearPidCooldown) {
  Context ctx;

  // In stopped mode WITH cooldown, current PID should not be cleared.
  ctx.status_.pid_d.integral = 1.0f;
  ctx.status_.pid_q.integral = 2.0f;
  ctx.status_.mode = kStopped;
  ctx.status_.cooldown_count = 10;
  ctx.ISR_ClearPid(TestControl::kClearIfMode);

  // Current PID not cleared because cooldown is active.
  BOOST_TEST(ctx.status_.pid_d.integral == 1.0f);
  BOOST_TEST(ctx.status_.pid_q.integral == 2.0f);
}

// Test free helper functions.

BOOST_AUTO_TEST_CASE(TestLimit) {
  BOOST_TEST(Limit(5.0f, 0.0f, 10.0f) == 5.0f);
  BOOST_TEST(Limit(-1.0f, 0.0f, 10.0f) == 0.0f);
  BOOST_TEST(Limit(15.0f, 0.0f, 10.0f) == 10.0f);
}

BOOST_AUTO_TEST_CASE(TestLimitCode) {
  auto [v1, c1] = LimitCode(5.0f, 0.0f, 10.0f,
                             errc::kLimitMaxCurrent, errc::kSuccess);
  BOOST_TEST(v1 == 5.0f);
  BOOST_CHECK(c1 == errc::kSuccess);

  auto [v2, c2] = LimitCode(-1.0f, 0.0f, 10.0f,
                             errc::kLimitMaxCurrent, errc::kSuccess);
  BOOST_TEST(v2 == 0.0f);
  BOOST_CHECK(c2 == errc::kLimitMaxCurrent);

  auto [v3, c3] = LimitCode(15.0f, 0.0f, 10.0f,
                             errc::kLimitMaxCurrent, errc::kSuccess);
  BOOST_TEST(v3 == 10.0f);
  BOOST_CHECK(c3 == errc::kLimitMaxCurrent);
}

BOOST_AUTO_TEST_CASE(TestThreshold) {
  BOOST_TEST(Threshold(5.0f, -1.0f, 1.0f) == 5.0f);
  BOOST_TEST(Threshold(-5.0f, -1.0f, 1.0f) == -5.0f);
  BOOST_TEST(Threshold(0.5f, -1.0f, 1.0f) == 0.0f);
  BOOST_TEST(Threshold(-0.5f, -1.0f, 1.0f) == 0.0f);
}

BOOST_AUTO_TEST_CASE(TestInterpolate) {
  BOOST_TEST(Interpolate(5.0f, 0.0f, 10.0f, 0.0f, 100.0f) == 50.0f,
             tt::tolerance(1e-6f));
  BOOST_TEST(Interpolate(0.0f, 0.0f, 10.0f, 0.0f, 100.0f) == 0.0f,
             tt::tolerance(1e-6f));
  BOOST_TEST(Interpolate(10.0f, 0.0f, 10.0f, 0.0f, 100.0f) == 100.0f,
             tt::tolerance(1e-6f));
}

BOOST_AUTO_TEST_CASE(TestRateConfig) {
  RateConfig rc(30000, 15000);
  BOOST_TEST(rc.pwm_rate_hz == 30000);
  BOOST_TEST(rc.interrupt_divisor == 1);
  BOOST_TEST(rc.int_rate_hz == 30000);
  BOOST_TEST(rc.period_s > 0.0f);
  BOOST_TEST(rc.min_pwm > 0.0f);
  BOOST_TEST(rc.max_pwm < 1.0f);

  // High rate gets divided.
  RateConfig rc2(60000, 15000);
  BOOST_TEST(rc2.pwm_rate_hz == 60000);
  BOOST_TEST(rc2.interrupt_divisor == 2);
  BOOST_TEST(rc2.int_rate_hz == 30000);
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoPwmControl) {
  Context ctx;

  Vec3 pwm;
  pwm.a = 0.5f;
  pwm.b = 0.6f;
  pwm.c = 0.7f;

  ctx.ISR_DoPwmControl(pwm);

  // Should have called DoPwmControl.
  BOOST_TEST(ctx.pwm_control_count == 1);
  // PWM values should be limited.
  BOOST_TEST(ctx.control_.pwm.a == 0.5f, tt::tolerance(1e-6f));
  BOOST_TEST(ctx.control_.pwm.b == 0.6f, tt::tolerance(1e-6f));
  BOOST_TEST(ctx.control_.pwm.c == 0.7f, tt::tolerance(1e-6f));

  // Test clamping.
  Vec3 pwm2;
  pwm2.a = 0.0f;
  pwm2.b = 1.0f;
  pwm2.c = 0.5f;
  ctx.ISR_DoPwmControl(pwm2);
  BOOST_TEST(ctx.control_.pwm.a == ctx.rate_config_.min_pwm,
             tt::tolerance(1e-6f));
  BOOST_TEST(ctx.control_.pwm.b == ctx.rate_config_.max_pwm,
             tt::tolerance(1e-6f));
  BOOST_TEST(ctx.pwm_control_count == 2);
}

BOOST_AUTO_TEST_CASE(BldcServoControlBalancedVoltage) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;

  Vec3 voltage;
  voltage.a = 6.0f;
  voltage.b = 0.0f;
  voltage.c = -6.0f;

  ctx.ISR_DoBalancedVoltageControl(voltage);

  // Should have stored voltage in control_.
  BOOST_TEST(ctx.control_.voltage.a == 6.0f, tt::tolerance(1e-6f));
  BOOST_TEST(ctx.control_.voltage.b == 0.0f, tt::tolerance(1e-6f));
  BOOST_TEST(ctx.control_.voltage.c == -6.0f, tt::tolerance(1e-6f));

  // Should have called DoPwmControl.
  BOOST_TEST(ctx.pwm_control_count == 1);

  // PWM values should be balanced: min+max shift so that
  // (min+max)/2 = 0.5.
  // pwm_in = {6/24, 0/24, -6/24} = {0.25, 0, -0.25}
  // offset = 0.5*(0.25 + -0.25) - 0.5 = -0.5
  // result = {0.75, 0.5, 0.25}
  BOOST_TEST(ctx.control_.pwm.a > ctx.control_.pwm.b);
  BOOST_TEST(ctx.control_.pwm.b > ctx.control_.pwm.c);
}

BOOST_AUTO_TEST_CASE(BldcServoControlVoltageDQ) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.position_.epoch = 0;
  ctx.isr_motor_position_epoch_ = 0;
  ctx.motor_position_config_val.output.sign = 1;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  ctx.ISR_DoVoltageDQ(sc, 1.0f, 0.0f);

  // Should set control d_V and q_V.
  BOOST_TEST(ctx.control_.d_V == 1.0f, tt::tolerance(1e-6f));
  BOOST_TEST(ctx.control_.q_V == 0.0f, tt::tolerance(1e-6f));
  BOOST_TEST(ctx.pwm_control_count == 1);
}

BOOST_AUTO_TEST_CASE(BldcServoControlVoltageDQCommand) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.position_.epoch = 0;
  ctx.isr_motor_position_epoch_ = 0;
  ctx.position_.theta_valid = true;
  ctx.motor_.poles = 14;
  ctx.motor_position_config_val.output.sign = 1;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  // Normal voltage command.
  ctx.ISR_DoVoltageDQCommand(sc, 1.0f, 2.0f);
  BOOST_TEST(ctx.pwm_control_count == 1);
  BOOST_TEST(ctx.control_.d_V == 1.0f, tt::tolerance(1e-6f));
  BOOST_TEST(ctx.control_.q_V == 2.0f, tt::tolerance(1e-6f));

  // Voltage exceeding max should be clamped.
  ctx.ISR_DoVoltageDQCommand(sc, 100.0f, 100.0f);
  const float max_V =
      ctx.rate_config_.max_voltage_ratio * kSvpwmRatio * 0.5f * 24.0f;
  BOOST_TEST(ctx.control_.d_V == max_V, tt::tolerance(1e-3f));
  BOOST_TEST(ctx.control_.q_V == max_V, tt::tolerance(1e-3f));
}

BOOST_AUTO_TEST_CASE(BldcServoControlVoltageDQCommandFault) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.motor_.poles = 0;  // Not configured.

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  ctx.ISR_DoVoltageDQCommand(sc, 1.0f, 2.0f);
  BOOST_CHECK(ctx.status_.mode == kFault);
  BOOST_CHECK(ctx.status_.fault == errc::kMotorNotConfigured);
  BOOST_TEST(ctx.pwm_control_count == 0);

  // Reset and test theta_valid fault.
  ctx.status_.mode = kVoltageDq;
  ctx.status_.fault = errc::kSuccess;
  ctx.motor_.poles = 14;
  ctx.position_.theta_valid = false;

  ctx.ISR_DoVoltageDQCommand(sc, 1.0f, 2.0f);
  BOOST_CHECK(ctx.status_.mode == kFault);
  BOOST_CHECK(ctx.status_.fault == errc::kThetaInvalid);
}

BOOST_AUTO_TEST_CASE(BldcServoControlCalculatePhaseVoltageEpochMismatch) {
  Context ctx;
  ctx.position_.epoch = 1;
  ctx.isr_motor_position_epoch_ = 0;  // Different epoch.

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  auto result = ctx.ISR_CalculatePhaseVoltage(sc, 1.0f, 0.0f);
  BOOST_CHECK(ctx.status_.mode == kFault);
  BOOST_CHECK(ctx.status_.fault == errc::kConfigChanged);
  BOOST_TEST(result.a == 0.0f);
  BOOST_TEST(result.b == 0.0f);
  BOOST_TEST(result.c == 0.0f);
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoCurrent) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.status_.filt_1ms_bus_V = 24.0f;
  ctx.status_.filt_fet_temp_C = 25.0f;
  ctx.status_.max_power_W = 500.0f;
  ctx.position_.epoch = 0;
  ctx.isr_motor_position_epoch_ = 0;
  ctx.position_.theta_valid = true;
  ctx.motor_.poles = 14;
  ctx.motor_.resistance_ohm = 0.1f;
  ctx.config_.max_current_A = 40.0f;
  ctx.config_.max_velocity = 100.0f;
  ctx.config_.max_velocity_derate = 20.0f;
  ctx.motor_position_config_val.output.sign = 1;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  // Basic current control - should drive voltage output.
  ctx.control_.i_q_A = -999.0f;  // sentinel
  ctx.ISR_DoCurrent(sc, 0.0f, 5.0f, 0.0f, false);

  // Verify no fault occurred.
  BOOST_CHECK(ctx.status_.mode != kFault);

  BOOST_TEST(ctx.pwm_control_count == 1);
  BOOST_TEST(ctx.control_.i_q_A == 5.0f, tt::tolerance(1e-3f));
  BOOST_TEST(ctx.control_.i_d_A == 0.0f, tt::tolerance(1e-3f));
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoCurrentFault) {
  Context ctx;
  ctx.motor_.poles = 0;
  ctx.status_.mode = kCurrent;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  ctx.ISR_DoCurrent(sc, 0.0f, 5.0f, 0.0f, false);
  BOOST_CHECK(ctx.status_.mode == kFault);
  BOOST_CHECK(ctx.status_.fault == errc::kMotorNotConfigured);

  // Test theta invalid.
  ctx.status_.mode = kCurrent;
  ctx.status_.fault = errc::kSuccess;
  ctx.motor_.poles = 14;
  ctx.position_.theta_valid = false;

  ctx.ISR_DoCurrent(sc, 0.0f, 5.0f, 0.0f, false);
  BOOST_CHECK(ctx.status_.mode == kFault);
  BOOST_CHECK(ctx.status_.fault == errc::kThetaInvalid);
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoCurrentLimits) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.status_.filt_1ms_bus_V = 24.0f;
  ctx.status_.filt_fet_temp_C = 25.0f;
  ctx.status_.max_power_W = 500.0f;
  ctx.position_.epoch = 0;
  ctx.isr_motor_position_epoch_ = 0;
  ctx.position_.theta_valid = true;
  ctx.motor_.poles = 14;
  ctx.motor_.resistance_ohm = 0.1f;
  ctx.config_.max_current_A = 10.0f;
  ctx.config_.max_velocity = 100.0f;
  ctx.config_.max_velocity_derate = 20.0f;
  ctx.motor_position_config_val.output.sign = 1;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  // Request current above max_current_A - should be clamped.
  ctx.ISR_DoCurrent(sc, 0.0f, 50.0f, 0.0f, false);
  BOOST_TEST(ctx.control_.i_q_A <= ctx.config_.max_current_A);
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoPosition) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.status_.filt_1ms_bus_V = 24.0f;
  ctx.status_.filt_fet_temp_C = 25.0f;
  ctx.status_.max_power_W = 500.0f;
  ctx.position_.epoch = 0;
  ctx.isr_motor_position_epoch_ = 0;
  ctx.position_.theta_valid = true;
  ctx.position_.position_relative_valid = true;
  ctx.position_.error = MotorPosition::Status::kNone;
  ctx.motor_.poles = 14;
  ctx.motor_.resistance_ohm = 0.1f;
  ctx.config_.max_current_A = 40.0f;
  ctx.config_.max_velocity = 100.0f;
  ctx.config_.max_velocity_derate = 20.0f;
  ctx.motor_position_config_val.output.sign = 1;
  ctx.motor_position_config_val.rotor_to_output_ratio = 1.0f;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  BldcServoCommandData data;
  data.mode = BldcServoMode::kPosition;
  data.position = 0.5f;
  data.velocity = 0.0f;
  data.max_torque_Nm = 0.5f;
  data.timeout_s = NaN;

  ctx.ISR_DoPosition(sc, &data);

  // Should have driven PWM.
  BOOST_TEST(ctx.pwm_control_count == 1);

  // Control torque should be set.
  BOOST_TEST(std::isfinite(ctx.control_.torque_Nm));
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoZeroVelocity) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.status_.filt_1ms_bus_V = 24.0f;
  ctx.status_.filt_fet_temp_C = 25.0f;
  ctx.status_.max_power_W = 500.0f;
  ctx.position_.epoch = 0;
  ctx.isr_motor_position_epoch_ = 0;
  ctx.position_.theta_valid = true;
  ctx.position_.position_relative_valid = true;
  ctx.position_.error = MotorPosition::Status::kNone;
  ctx.motor_.poles = 14;
  ctx.motor_.resistance_ohm = 0.1f;
  ctx.config_.max_current_A = 40.0f;
  ctx.config_.max_velocity = 100.0f;
  ctx.config_.max_velocity_derate = 20.0f;
  ctx.motor_position_config_val.output.sign = 1;
  ctx.motor_position_config_val.rotor_to_output_ratio = 1.0f;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  BldcServoCommandData data;
  data.kd_scale = 1.0f;

  ctx.ISR_DoZeroVelocity(sc, &data);

  // Should have driven PWM.
  BOOST_TEST(ctx.pwm_control_count == 1);
  BOOST_CHECK(ctx.status_.mode != BldcServoMode::kFault);
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoStayWithinBoundsInside) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.status_.filt_1ms_bus_V = 24.0f;
  ctx.status_.filt_fet_temp_C = 25.0f;
  ctx.status_.max_power_W = 500.0f;
  ctx.position_.epoch = 0;
  ctx.isr_motor_position_epoch_ = 0;
  ctx.position_.theta_valid = true;
  ctx.position_.position_relative_valid = true;
  ctx.position_.error = MotorPosition::Status::kNone;
  ctx.position_.position = 0.5f;
  ctx.motor_.poles = 14;
  ctx.motor_.resistance_ohm = 0.1f;
  ctx.config_.max_current_A = 40.0f;
  ctx.config_.max_velocity = 100.0f;
  ctx.config_.max_velocity_derate = 20.0f;
  ctx.motor_position_config_val.output.sign = 1;
  ctx.motor_position_config_val.rotor_to_output_ratio = 1.0f;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  BldcServoCommandData data;
  data.mode = BldcServoMode::kStayWithinBounds;
  data.bounds_min = 0.0f;
  data.bounds_max = 1.0f;
  data.max_torque_Nm = 0.5f;
  data.timeout_s = NaN;

  ctx.ISR_DoStayWithinBounds(sc, &data);

  // Position is within bounds, so PID should be cleared and
  // zero torque essentially applied.
  BOOST_TEST(ctx.pwm_control_count == 1);
  BOOST_CHECK(ctx.status_.mode != BldcServoMode::kFault);
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoStayWithinBoundsOutside) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.status_.filt_1ms_bus_V = 24.0f;
  ctx.status_.filt_fet_temp_C = 25.0f;
  ctx.status_.max_power_W = 500.0f;
  ctx.position_.epoch = 0;
  ctx.isr_motor_position_epoch_ = 0;
  ctx.position_.theta_valid = true;
  ctx.position_.position_relative_valid = true;
  ctx.position_.error = MotorPosition::Status::kNone;
  ctx.position_.position = -0.5f;
  ctx.motor_.poles = 14;
  ctx.motor_.resistance_ohm = 0.1f;
  ctx.config_.max_current_A = 40.0f;
  ctx.config_.max_velocity = 100.0f;
  ctx.config_.max_velocity_derate = 20.0f;
  ctx.motor_position_config_val.output.sign = 1;
  ctx.motor_position_config_val.rotor_to_output_ratio = 1.0f;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  BldcServoCommandData data;
  data.mode = BldcServoMode::kStayWithinBounds;
  data.bounds_min = 0.0f;
  data.bounds_max = 1.0f;
  data.max_torque_Nm = 0.5f;
  data.timeout_s = NaN;

  ctx.ISR_DoStayWithinBounds(sc, &data);

  // Position is below bounds_min, should try to move back.
  BOOST_TEST(ctx.pwm_control_count == 1);
  BOOST_CHECK(ctx.status_.mode != BldcServoMode::kFault);
  // Control position should be set to the violated bound.
  BOOST_TEST(ctx.status_.control_position == 0.0f, tt::tolerance(0.01f));
}

BOOST_AUTO_TEST_CASE(BldcServoControlVoltageFOC) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;

  BldcServoCommandData data;
  data.theta = 0.0f;
  data.theta_rate = 100.0f;
  data.voltage = 5.0f;

  ctx.ISR_DoVoltageFOC(&data);

  // theta should have advanced.
  BOOST_TEST(data.theta != 0.0f);
  BOOST_TEST(data.theta == 100.0f * ctx.rate_config_.period_s,
             tt::tolerance(1e-6f));
  // Should have driven voltage.
  BOOST_TEST(ctx.pwm_control_count == 1);
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoStopped) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.status_.filt_fet_temp_C = 25.0f;
  ctx.status_.max_power_W = 500.0f;
  ctx.position_.theta_valid = true;
  ctx.motor_.poles = 14;
  ctx.motor_.resistance_ohm = 0.1f;
  ctx.config_.max_current_A = 40.0f;
  ctx.config_.max_velocity = 100.0f;
  ctx.config_.max_velocity_derate = 20.0f;
  ctx.motor_position_config_val.output.sign = 1;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  // With no cooldown, should call DoHardStop.
  ctx.status_.cooldown_count = 0;
  ctx.ISR_DoStopped(sc);
  BOOST_TEST(ctx.hard_stop_count == 1);
  BOOST_TEST(ctx.status_.power_W == 0.0f);

  // With cooldown, should call ISR_DoCurrent instead.
  ctx.status_.cooldown_count = 3;
  ctx.ISR_DoStopped(sc);
  BOOST_TEST(ctx.hard_stop_count == 1);  // unchanged
  BOOST_TEST(ctx.status_.cooldown_count == 2);
  BOOST_TEST(ctx.pwm_control_count == 1);  // ISR_DoCurrent drove PWM
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoMeasureInductance) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.position_.epoch = 0;
  ctx.isr_motor_position_epoch_ = 0;
  ctx.motor_position_config_val.output.sign = 1;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  BldcServoCommandData data;
  data.d_V = 2.0f;
  data.meas_ind_period = 4;
  ctx.status_.meas_ind_phase = 4;

  ctx.ISR_DoMeasureInductance(sc, &data);

  // Phase should have changed.
  BOOST_TEST(ctx.status_.meas_ind_phase == 3);
  // Should have driven PWM.
  BOOST_TEST(ctx.pwm_control_count == 1);
}

BOOST_AUTO_TEST_CASE(BldcServoControlMaybeChangeMode) {
  Context ctx;

  BldcServoCommandData data;

  // From stopped, requesting current should start calibrating.
  ctx.status_.mode = BldcServoMode::kStopped;
  data.mode = BldcServoMode::kCurrent;
  ctx.ISR_MaybeChangeMode(&data);
  BOOST_CHECK(ctx.status_.mode == BldcServoMode::kEnabling);
  BOOST_TEST(ctx.start_calibrating_count == 1);

  // From calibration complete, requesting current should succeed.
  ctx.status_.mode = BldcServoMode::kCalibrationComplete;
  data.mode = BldcServoMode::kCurrent;
  ctx.ISR_MaybeChangeMode(&data);
  BOOST_CHECK(ctx.status_.mode == BldcServoMode::kCurrent);

  // From fault, requesting current should fail (stay in fault).
  ctx.status_.mode = BldcServoMode::kFault;
  data.mode = BldcServoMode::kCurrent;
  ctx.ISR_MaybeChangeMode(&data);
  BOOST_CHECK(ctx.status_.mode == BldcServoMode::kFault);

  // Requesting stopped should always work.
  ctx.status_.mode = BldcServoMode::kCurrent;
  data.mode = BldcServoMode::kStopped;
  ctx.ISR_MaybeChangeMode(&data);
  BOOST_CHECK(ctx.status_.mode == BldcServoMode::kStopped);
}

BOOST_AUTO_TEST_CASE(BldcServoControlDoControl) {
  Context ctx;
  ctx.status_.filt_bus_V = 24.0f;
  ctx.status_.filt_1ms_bus_V = 24.0f;
  ctx.status_.filt_fet_temp_C = 25.0f;
  ctx.status_.bus_V = 24.0f;
  ctx.status_.max_power_W = 500.0f;
  ctx.position_.epoch = 0;
  ctx.isr_motor_position_epoch_ = 0;
  ctx.position_.theta_valid = true;
  ctx.motor_.poles = 14;
  ctx.motor_.resistance_ohm = 0.1f;
  ctx.config_.max_current_A = 40.0f;
  ctx.config_.max_velocity = 100.0f;
  ctx.config_.max_velocity_derate = 20.0f;
  ctx.motor_position_config_val.output.sign = 1;

  SinCos sc;
  sc.s = 0.0f;
  sc.c = 1.0f;

  // Test stopped mode dispatch.
  ctx.status_.mode = BldcServoMode::kStopped;
  BldcServoCommandData data;
  data.mode = BldcServoMode::kStopped;

  ctx.ISR_DoControl(sc, &data);
  BOOST_TEST(ctx.hard_stop_count == 1);

  // Test fault detection - over voltage.
  ctx.status_.mode = BldcServoMode::kCurrent;
  data.mode = BldcServoMode::kCurrent;
  ctx.status_.bus_V = 1000.0f;
  ctx.ISR_DoControl(sc, &data);
  BOOST_CHECK(ctx.status_.mode == BldcServoMode::kFault);
  BOOST_CHECK(ctx.status_.fault == errc::kOverVoltage);

  // Test fault detection - motor driver fault.
  ctx.status_.mode = BldcServoMode::kCurrent;
  ctx.status_.bus_V = 24.0f;
  ctx.fault_state = true;
  ctx.ISR_DoControl(sc, &data);
  BOOST_CHECK(ctx.status_.mode == BldcServoMode::kFault);
  BOOST_CHECK(ctx.status_.fault == errc::kMotorDriverFault);
}
