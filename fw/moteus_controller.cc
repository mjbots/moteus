// Copyright 2018-2022 Josh Pieper, jjp@pobox.com.
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

#include "fw/moteus_controller.h"

#include "mjlib/base/limit.h"

#include "fw/aux_port.h"
#include "fw/drv8323.h"
#include "fw/math.h"
#include "fw/moteus_hw.h"
#include "fw/motor_position.h"

namespace micro = mjlib::micro;
namespace multiplex = mjlib::multiplex;
using mjlib::base::Limit;

namespace moteus {

using Value = multiplex::MicroServer::Value;

namespace {

// Version 3: The enumerated values for the control mode changed
// meaning and voltage scale changed.
//
// Version 4: The scaling factors associated with velocity and
// position changed.
//
// Version 5: The "rezero state" was renamed to "home state" and the
// enumeration has new meaning.

constexpr int kRegisterMapVersion = 5;

template <typename T>
Value IntMapping(T value, size_t type) {
  switch (type) {
    case 0: return static_cast<int8_t>(value);
    case 1: return static_cast<int16_t>(value);
    case 2: return static_cast<int32_t>(value);
    case 3: return static_cast<float>(value);
  }
  MJ_ASSERT(false);
  return static_cast<int8_t>(0);
}

template <typename T>
Value ScaleSaturate(float value, float scale) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<T>::min();
  }

  const float scaled = value / scale;
  const auto max = std::numeric_limits<T>::max();
  const auto float_max = static_cast<float>(max);

  // We purposefully limit to +- max, rather than to min.  The minimum
  // value for our two's complement types is reserved for NaN.
  return static_cast<T>(Limit<float>(scaled, -float_max, float_max));
}

Value ScaleMapping(float value,
                   float int8_scale, float int16_scale, float int32_scale,
                   size_t type) {
  switch (type) {
    case 0: return ScaleSaturate<int8_t>(value, int8_scale);
    case 1: return ScaleSaturate<int16_t>(value, int16_scale);
    case 2: return ScaleSaturate<int32_t>(value, int32_scale);
    case 3: return Value(value);
  }
  MJ_ASSERT(false);
  return Value(static_cast<int8_t>(0));
}

Value ScalePosition(float value, size_t type) {
  return ScaleMapping(value, 0.01f, 0.0001f, 0.00001f, type);
}

Value ScaleVelocity(float value, size_t type) {
  return ScaleMapping(value, 0.1f, 0.00025f, 0.00001f, type);
}

Value ScaleAcceleration(float value, size_t type) {
  return ScaleMapping(value, 0.05f, 0.001f, 0.00001f, type);
}

Value ScaleTemperature(float value, size_t type) {
  return ScaleMapping(value, 1.0f, 0.1f, 0.001f, type);
}

Value ScalePwm(float value, size_t type) {
  return ScaleMapping(value, 1.0f / 127.0f, 1.0f / 32767.0f,
                      1.0f / 2147483647.0f,
                      type);
}

Value ScaleCurrent(float value, size_t type) {
  // For now, current and temperature have identical scaling.
  return ScaleTemperature(value, type);
}

Value ScaleVoltage(float value, size_t type) {
  return ScaleMapping(value, 0.5f, 0.1f, 0.001f, type);
}

Value ScaleTorque(float value, size_t type) {
  return ScaleMapping(value, 0.5f, 0.01f, 0.001f, type);
}

Value ScaleTime(float value, size_t type) {
  return ScaleMapping(value, 0.01f, 0.001f, 0.000001f, type);
}

int8_t ReadIntMapping(Value value) {
  return std::visit([](auto a) {
      return static_cast<int8_t>(a);
    }, value);
}

struct ValueScaler {
  float int8_scale;
  float int16_scale;
  float int32_scale;

  float operator()(int8_t value) const {
    if (value == std::numeric_limits<int8_t>::min()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return value * int8_scale;
  }

  float operator()(int16_t value) const {
    if (value == std::numeric_limits<int16_t>::min()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return value * int16_scale;
  }

  float operator()(int32_t value) const {
    if (value == std::numeric_limits<int32_t>::min()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return value * int32_scale;
  }

  float operator()(float value) const {
    return value;
  }
};

float ReadScaleMapping(Value value,
                       float int8_scale,
                       float int16_scale,
                       float int32_scale) {
  return std::visit(ValueScaler{int8_scale, int16_scale, int32_scale}, value);
}

float ReadPwm(Value value) {
  return ReadScaleMapping(value, 1.0f / 127.0f, 1.0f / 32767.0f,
                          1.0f / 2147483647.0f);
}

float ReadVoltage(Value value) {
  return ReadScaleMapping(value, 0.5f, 0.1f, 0.001f);
}

float ReadPosition(Value value) {
  return ReadScaleMapping(value, 0.01f, 0.0001f, 0.00001f);
}

float ReadVelocity(Value value) {
  return ReadScaleMapping(value, 0.1f, 0.00025f, 0.00001f);
}

float ReadAcceleration(Value value) {
  return ReadScaleMapping(value, 0.05f, 0.001f, 0.00001f);
}

float ReadCurrent(Value value) {
  return ReadScaleMapping(value, 1.0f, 0.1f, 0.001f);
}

float ReadTorque(Value value) {
  return ReadScaleMapping(value, 0.5f, 0.01f, 0.001f);
}

float ReadTime(Value value) {
  return ReadScaleMapping(value, 0.01f, 0.001f, 0.000001f);
}

template <typename T, size_t N>
int8_t PinsToBits(const std::array<T, N>& array) {
  static_assert(N <= 7);
  int8_t result = 0;
  for (size_t i = 0; i < array.size(); i++) {
    result |= (array[i] ? 1 : 0) << i;
  }
  return result;
}

enum class Register {
  kMode = 0x000,
  kPosition = 0x001,
  kVelocity = 0x002,
  kTorque = 0x003,
  kQCurrent = 0x004,
  kDCurrent = 0x005,
  kAbsPosition = 0x006,

  kTrajectoryComplete = 0x00b,
  kHomeState = 0x00c,
  kVoltage = 0x00d,
  kTemperature = 0x00e,
  kFault = 0x00f,

  kPwmPhaseA = 0x010,
  kPwmPhaseB = 0x011,
  kPwmPhaseC = 0x012,

  kVoltagePhaseA = 0x014,
  kVoltagePhaseB = 0x015,
  kVoltagePhaseC = 0x016,

  kVFocTheta = 0x018,
  kVFocVoltage = 0x019,
  kVoltageDqD = 0x01a,
  kVoltageDqQ = 0x01b,

  kCommandQCurrent = 0x01c,
  kCommandDCurrent = 0x01d,

  kVFocThetaRate = 0x01e,

  kCommandPosition = 0x020,
  kCommandVelocity = 0x021,
  kCommandFeedforwardTorque = 0x022,
  kCommandKpScale = 0x023,
  kCommandKdScale = 0x024,
  kCommandPositionMaxTorque = 0x025,
  kCommandStopPosition = 0x026,
  kCommandTimeout = 0x027,
  kCommandVelocityLimit = 0x028,
  kCommandAccelLimit = 0x029,
  kCommandFixedVoltageOverride = 0x02a,

  kPositionKp = 0x030,
  kPositionKi = 0x031,
  kPositionKd = 0x032,
  kPositionFeedforward = 0x033,
  kPositionCommandTorque = 0x034,

  kStayWithinLower = 0x040,
  kStayWithinUpper = 0x041,
  kStayWithinFeedforward = 0x042,
  kStayWithinKpScale = 0x043,
  kStayWithinKdScale = 0x044,
  kStayWithinMaxTorque = 0x045,
  kStayWithinTimeout = 0x046,

  kEncoder0Position = 0x050,
  kEncoder0Velocity = 0x051,
  kEncoder1Position = 0x052,
  kEncoder1Velocity = 0x053,
  kEncoder2Position = 0x054,
  kEncoder2Velocity = 0x055,
  kEncoderValidity = 0x058,
  kAux1GpioCommand = 0x05c,
  kAux2GpioCommand = 0x05d,
  kAux1GpioStatus = 0x05e,
  kAux2GpioStatus = 0x05f,

  kAux1AnalogIn1 = 0x060,
  kAux1AnalogIn2 = 0x061,
  kAux1AnalogIn3 = 0x062,
  kAux1AnalogIn4 = 0x063,
  kAux1AnalogIn5 = 0x064,

  kAux2AnalogIn1 = 0x068,
  kAux2AnalogIn2 = 0x069,
  kAux2AnalogIn3 = 0x06a,
  kAux2AnalogIn4 = 0x06b,
  kAux2AnalogIn5 = 0x06c,

  kModelNumber = 0x100,
  kFirmwareVersion = 0x101,
  kRegisterMapVersion = 0x102,
  kMultiplexId = 0x110,

  kSerialNumber1 = 0x120,
  kSerialNumber2 = 0x121,
  kSerialNumber3 = 0x122,

  kSetOutputNearest = 0x130,
  kSetOutputExact = 0x131,
  kRequireReindex = 0x132,
};

constexpr aux::AuxHardwareConfig kAux1PortHardwareConfig = {
  {{
      { MOTEUS_EXTERNAL_ENCODER_CS, -1, 0,  aux::kNoI2c },
      { MOTEUS_AS5047_SCK,           2, 5,  aux::kNoI2c },
      { MOTEUS_AS5047_MISO,          0, 5,  aux::kNoI2c },
      { MOTEUS_AS5047_MOSI,          1, 15, aux::kNoI2c },
      { NC, -1, -1 },
    }},
  {{
    { SPI2, MOTEUS_AS5047_SCK, MOTEUS_AS5047_MISO, MOTEUS_AS5047_MOSI },
    { nullptr, NC, NC, NC },
    }},
  {{
      { nullptr, NC, NC },
      { nullptr, NC, NC },
    }},
};

constexpr aux::AuxHardwareConfig kAux2PortHardwareConfig = {
  {{
      { MOTEUS_ABS_SCL, -1, 0,  aux::kScl },
      { MOTEUS_ABS_SDA, -1, 0,  aux::kSda },
      { MOTEUS_DEBUG1,  -1, 0,  aux::kNoI2c },
      { MOTEUS_DEBUG2,  -1, 0,  aux::kNoI2c },
      { NC,             -1, 0,  aux::kNoI2c },
    }},
  {{
    { nullptr, NC, NC, NC },
    { nullptr, NC, NC, NC },
    }},
  {{
      { USART3, MOTEUS_ABS_SCL, MOTEUS_ABS_SDA },
      { nullptr, NC, NC },
    }},
};
}

class MoteusController::Impl : public multiplex::MicroServer::Server {
 public:
  Impl(micro::Pool* pool,
       micro::PersistentConfig* persistent_config,
       micro::CommandManager* command_manager,
       micro::TelemetryManager* telemetry_manager,
       MillisecondTimer* timer,
       FirmwareInfo* firmware)
      : aux1_port_("aux1", "ic_pz1", kAux1PortHardwareConfig,
                   persistent_config, command_manager, telemetry_manager, timer,
                   AuxPort::kDefaultOnboardSpi),
        aux2_port_("aux2", "ic_pz2", kAux2PortHardwareConfig,
                   persistent_config, command_manager, telemetry_manager, timer,
                   AuxPort::kNoDefaultSpi),
        motor_position_(persistent_config, telemetry_manager,
                        aux1_port_.status(),
                        aux2_port_.status(),
                        aux1_port_.config(),
                        aux2_port_.config()),
        drv8323_(pool, persistent_config, telemetry_manager, timer, []() {
            Drv8323::Options options;
            options.mosi = DRV8323_MOSI;
            options.miso = DRV8323_MISO;
            options.sck = DRV8323_SCK;
            options.cs = DRV8323_CS;
            options.enable = DRV8323_ENABLE;
            options.fault = DRV8323_FAULT;
            options.hiz = DRV8323_HIZ;
            return options;
          }()),
        bldc_(pool, persistent_config, telemetry_manager,
              timer, &drv8323_, &aux1_port_, &aux2_port_, &motor_position_,
              []() {
            BldcServo::Options options;
            options.pwm1 = PA_0;
            options.pwm2 = PA_1;
            options.pwm3 = PA_2;

            options.current1 = MOTEUS_CURRENT1;
            options.current2 = MOTEUS_CURRENT2;
            options.current3 = MOTEUS_CURRENT3;
            options.vsense =
                (g_measured_hw_rev < 5) ?
                MOTEUS_VSENSE : MOTEUS_VSENSE_5_AND_LATER;

            options.tsense = MOTEUS_TSENSE;
            options.msense =
                (g_measured_hw_rev < 5) ?
                MOTEUS_MSENSE : MOTEUS_MSENSE_5_AND_LATER;

            options.debug_dac = MOTEUS_DEBUG_DAC;
            options.debug_out = MOTEUS_DEBUG1;
            options.debug_out2 = MOTEUS_DEBUG2;
            options.debug_uart_out = MOTEUS_UART_TX;

            return options;
          }()),
        firmware_(firmware) {}

  void Start() {
    bldc_.Start();
  }

  void Poll() {
    // Check to see if we have a command to send out.
    if (command_valid_) {
      command_valid_ = false;
      bldc_.Command(command_);
    }
    aux1_port_.Poll();
    aux2_port_.Poll();
  }

  void PollMillisecond() {
    aux1_port_.PollMillisecond();
    aux2_port_.PollMillisecond();
    drv8323_.PollMillisecond();
    bldc_.PollMillisecond();
  }

  uint32_t Write(multiplex::MicroServer::Register reg,
                 const multiplex::MicroServer::Value& value) override
      __attribute__ ((optimize("O3"))){
    switch (static_cast<Register>(reg)) {
      case Register::kMode: {
        const auto new_mode_int = ReadIntMapping(value);
        if (new_mode_int > static_cast<int8_t>(BldcServo::Mode::kNumModes)) {
          return 3;
        }
        command_valid_ = true;
        const auto new_mode = static_cast<BldcServo::Mode>(new_mode_int);
        command_ = {};
        command_.mode = new_mode;
        return 0;
      }

      case Register::kPwmPhaseA: {
        command_.pwm.a = ReadPwm(value);
        return 0;
      }
      case Register::kPwmPhaseB: {
        command_.pwm.b = ReadPwm(value);
        return 0;
      }
      case Register::kPwmPhaseC: {
        command_.pwm.c = ReadPwm(value);
        return 0;
      }
      case Register::kVoltagePhaseA: {
        command_.phase_v.a = ReadVoltage(value);
        return 0;
      }
      case Register::kVoltagePhaseB: {
        command_.phase_v.b = ReadVoltage(value);
        return 0;
      }
      case Register::kVoltagePhaseC: {
        command_.phase_v.c = ReadVoltage(value);
        return 0;
      }
      case Register::kVFocTheta: {
        command_.theta = ReadPwm(value) * kPi;
        return 0;
      }
      case Register::kVFocVoltage: {
        command_.voltage = ReadVoltage(value);
        return 0;
      }
      case Register::kVoltageDqD: {
        command_.d_V = ReadVoltage(value);
        return 0;
      }
      case Register::kVoltageDqQ: {
        command_.q_V = ReadVoltage(value);
        return 0;
      }
      case Register::kCommandQCurrent: {
        command_.i_q_A = ReadCurrent(value);
        return 0;
      }
      case Register::kCommandDCurrent: {
        command_.i_d_A = ReadCurrent(value);
        return 0;
      }
      case Register::kVFocThetaRate: {
        command_.theta_rate = ReadVelocity(value) * kPi;
        return 0;
      }
      case Register::kCommandPosition: {
        command_.position = ReadPosition(value);
        return 0;
      }
      case Register::kCommandVelocity: {
        command_.velocity = ReadVelocity(value);
        return 0;
      }
      case Register::kCommandPositionMaxTorque:
      case Register::kStayWithinMaxTorque: {
        command_.max_torque_Nm = ReadTorque(value);
        return 0;
      }
      case Register::kCommandStopPosition: {
        command_.stop_position = ReadPosition(value);
        return 0;
      }
      case Register::kCommandTimeout:
      case Register::kStayWithinTimeout: {
        command_.timeout_s = ReadTime(value);
        return 0;
      }
      case Register::kCommandAccelLimit: {
        command_.accel_limit = ReadAcceleration(value);
        return 0;
      }
      case Register::kCommandVelocityLimit: {
        command_.velocity_limit = ReadVelocity(value);
        return 0;
      }
      case Register::kCommandFixedVoltageOverride: {
        command_.fixed_voltage_override = ReadVoltage(value);
        return 0;
      }
      case Register::kCommandFeedforwardTorque:
      case Register::kStayWithinFeedforward: {
        command_.feedforward_Nm = ReadTorque(value);
        return 0;
      }
      case Register::kCommandKpScale:
      case Register::kStayWithinKpScale: {
        command_.kp_scale = ReadPwm(value);
        return 0;
      }
      case Register::kCommandKdScale:
      case Register::kStayWithinKdScale: {
        command_.kd_scale = ReadPwm(value);
        return 0;
      }
      case Register::kStayWithinLower: {
        command_.bounds_min = ReadPosition(value);
        return 0;
      }
      case Register::kStayWithinUpper: {
        command_.bounds_max = ReadPosition(value);
        return 0;
      }

      case Register::kAux1GpioCommand: {
        aux1_port_.WriteDigitalOut(ReadIntMapping(value));
        return 0;
      }
      case Register::kAux2GpioCommand: {
        aux2_port_.WriteDigitalOut(ReadIntMapping(value));
        return 0;
      }

      case Register::kSetOutputNearest: {
        const float position = ReadPosition(value);
        bldc_.SetOutputPositionNearest(position);
        return 0;
      }
      case Register::kSetOutputExact: {
        const float position = ReadPosition(value);
        bldc_.SetOutputPosition(position);
        return 0;
      }
      case Register::kRequireReindex: {
        bldc_.RequireReindex();
        return 0;
      }

      case Register::kPosition:
      case Register::kVelocity:
      case Register::kTemperature:
      case Register::kQCurrent:
      case Register::kDCurrent:
      case Register::kAbsPosition:
      case Register::kTrajectoryComplete:
      case Register::kHomeState:
      case Register::kVoltage:
      case Register::kTorque:
      case Register::kFault:
      case Register::kPositionKp:
      case Register::kPositionKi:
      case Register::kPositionKd:
      case Register::kPositionFeedforward:
      case Register::kPositionCommandTorque:
      case Register::kEncoder0Position:
      case Register::kEncoder0Velocity:
      case Register::kEncoder1Position:
      case Register::kEncoder1Velocity:
      case Register::kEncoder2Position:
      case Register::kEncoder2Velocity:
      case Register::kEncoderValidity:
      case Register::kAux1GpioStatus:
      case Register::kAux2GpioStatus:
      case Register::kAux1AnalogIn1:
      case Register::kAux1AnalogIn2:
      case Register::kAux1AnalogIn3:
      case Register::kAux1AnalogIn4:
      case Register::kAux1AnalogIn5:
      case Register::kAux2AnalogIn1:
      case Register::kAux2AnalogIn2:
      case Register::kAux2AnalogIn3:
      case Register::kAux2AnalogIn4:
      case Register::kAux2AnalogIn5:
      case Register::kModelNumber:
      case Register::kSerialNumber1:
      case Register::kSerialNumber2:
      case Register::kSerialNumber3:
      case Register::kRegisterMapVersion:
      case Register::kFirmwareVersion:
      case Register::kMultiplexId: {
        // Not writeable
        return 2;
      }
    }

    // If we got here, then we had an unknown register.
    return 1;
  }

  multiplex::MicroServer::ReadResult Read(
      multiplex::MicroServer::Register reg,
      size_t type) const override
      __attribute__ ((optimize("O3"))) {
    auto vi32 = [](auto v) { return Value(static_cast<int32_t>(v)); };

    switch (static_cast<Register>(reg)) {
      case Register::kMode: {
        return IntMapping(static_cast<int8_t>(bldc_.status().mode), type);
      }
      case Register::kPosition: {
        return ScalePosition(bldc_.status().position, type);
      }
      case Register::kVelocity: {
        return ScaleVelocity(bldc_.status().velocity, type);
      }
      case Register::kTemperature: {
        return ScaleTemperature(bldc_.status().fet_temp_C, type);
      }
      case Register::kQCurrent: {
        return ScaleCurrent(bldc_.status().q_A, type);
      }
      case Register::kDCurrent: {
        return ScaleCurrent(bldc_.status().d_A, type);
      }
      case Register::kAbsPosition: {
        return ScalePosition(bldc_.motor_position().sources[1].filtered_value, type);
      }
      case Register::kTrajectoryComplete: {
        return IntMapping(bldc_.status().trajectory_done ? 1 : 0, type);
      }
      case Register::kHomeState: {
        return IntMapping(
            static_cast<int>(bldc_.motor_position().homed), type);
      }
      case Register::kVoltage: {
        return ScaleVoltage(bldc_.status().bus_V, type);
      }
      case Register::kTorque: {
        return ScaleTorque(bldc_.status().torque_Nm, type);
      }
      case Register::kFault: {
        return IntMapping(bldc_.status().fault, type);
      }

      case Register::kPwmPhaseA: {
        return ScalePwm(command_.pwm.a, type);
      }
      case Register::kPwmPhaseB: {
        return ScalePwm(command_.pwm.b, type);
      }
      case Register::kPwmPhaseC: {
        return ScalePwm(command_.pwm.c, type);
      }
      case Register::kVoltagePhaseA: {
        return ScaleVoltage(command_.phase_v.a, type);
      }
      case Register::kVoltagePhaseB: {
        return ScaleVoltage(command_.phase_v.b, type);
      }
      case Register::kVoltagePhaseC: {
        return ScaleVoltage(command_.phase_v.c, type);
      }
      case Register::kVFocTheta: {
        return ScalePwm(command_.theta / kPi, type);
      }
      case Register::kVFocVoltage: {
        return ScaleVoltage(command_.voltage, type);
      }
      case Register::kVoltageDqD: {
        return ScaleVoltage(command_.d_V, type);
      }
      case Register::kVoltageDqQ: {
        return ScaleVoltage(command_.q_V, type);
      }
      case Register::kCommandQCurrent: {
        return ScaleCurrent(command_.i_q_A, type);
      }
      case Register::kCommandDCurrent: {
        return ScaleCurrent(command_.i_d_A, type);
      }
      case Register::kVFocThetaRate: {
        return ScaleVelocity(command_.theta_rate / kPi, type);
      }
      case Register::kCommandPosition: {
        return ScalePosition(command_.position, type);
      }
      case Register::kCommandVelocity: {
        return ScaleVelocity(command_.velocity, type);
      }
      case Register::kCommandPositionMaxTorque:
      case Register::kStayWithinMaxTorque: {
        return ScaleTorque(command_.max_torque_Nm, type);
      }
      case Register::kCommandStopPosition: {
        return ScalePosition(command_.stop_position, type);
      }
      case Register::kCommandTimeout:
      case Register::kStayWithinTimeout: {
        return ScaleTime(command_.timeout_s, type);
      }
      case Register::kCommandVelocityLimit: {
        return ScaleVelocity(command_.velocity_limit, type);
      }
      case Register::kCommandAccelLimit: {
        return ScaleAcceleration(command_.accel_limit, type);
      }
      case Register::kCommandFixedVoltageOverride: {
        return ScaleVoltage(command_.fixed_voltage_override, type);
      }
      case Register::kCommandFeedforwardTorque:
      case Register::kStayWithinFeedforward: {
        return ScaleTorque(command_.feedforward_Nm, type);
      }
      case Register::kCommandKpScale:
      case Register::kStayWithinKpScale: {
        return ScalePwm(command_.kp_scale, type);
      }
      case Register::kCommandKdScale:
      case Register::kStayWithinKdScale: {
        return ScalePwm(command_.kd_scale, type);
      }

      case Register::kPositionKp: {
        return ScaleTorque(bldc_.status().pid_position.p, type);
      }
      case Register::kPositionKi: {
        return ScaleTorque(bldc_.status().pid_position.integral, type);
      }
      case Register::kPositionKd: {
        return ScaleTorque(bldc_.status().pid_position.d, type);
      }
      case Register::kPositionFeedforward: {
        return ScaleTorque(command_.feedforward_Nm, type);
      }
      case Register::kPositionCommandTorque: {
        return ScaleTorque(bldc_.control().torque_Nm, type);
      }

      case Register::kStayWithinLower: {
        return ScalePosition(command_.bounds_min, type);
      }
      case Register::kStayWithinUpper: {
        return ScalePosition(command_.bounds_max, type);
      }

      case Register::kEncoder0Position: {
        return ScalePosition(bldc_.motor_position().sources[0].filtered_value, type);
      }
      case Register::kEncoder0Velocity: {
        return ScaleVelocity(bldc_.motor_position().sources[0].velocity, type);
      }
      case Register::kEncoder1Position: {
        return ScalePosition(bldc_.motor_position().sources[1].filtered_value, type);
      }
      case Register::kEncoder1Velocity: {
        return ScaleVelocity(bldc_.motor_position().sources[1].velocity, type);
      }
      case Register::kEncoder2Position: {
        return ScalePosition(bldc_.motor_position().sources[2].filtered_value, type);
      }
      case Register::kEncoder2Velocity: {
        return ScaleVelocity(bldc_.motor_position().sources[2].velocity, type);
      }
      case Register::kEncoderValidity: {
        const auto& status = bldc_.motor_position();

        const int8_t validity =
            ((status.sources[0].active_theta ? 1 : 0) << 0) |
            ((status.sources[0].active_velocity ? 1 : 0) << 1) |
            ((status.sources[1].active_theta ? 1 : 0) << 2) |
            ((status.sources[1].active_velocity ? 1 : 0) << 3) |
            ((status.sources[2].active_theta ? 1 : 0) << 4);
            ((status.sources[2].active_velocity ? 1 : 0) << 5);
        return IntMapping(validity, type);
      }
      case Register::kAux1GpioCommand: {
        return IntMapping(PinsToBits(bldc_.aux1().pins), type);
      }
      case Register::kAux2GpioCommand: {
        return IntMapping(PinsToBits(bldc_.aux2().pins), type);
      }
      case Register::kAux1GpioStatus: {
        return IntMapping(PinsToBits(bldc_.aux1().pins), type);
      }
      case Register::kAux2GpioStatus: {
        return IntMapping(PinsToBits(bldc_.aux2().pins), type);
      }
      case Register::kAux1AnalogIn1:
      case Register::kAux1AnalogIn2:
      case Register::kAux1AnalogIn3:
      case Register::kAux1AnalogIn4:
      case Register::kAux1AnalogIn5: {
        const int pin =
            static_cast<int>(reg) - static_cast<int>(Register::kAux1AnalogIn1);
        return ScalePwm(bldc_.aux1().analog_inputs[pin], type);
      }
      case Register::kAux2AnalogIn1:
      case Register::kAux2AnalogIn2:
      case Register::kAux2AnalogIn3:
      case Register::kAux2AnalogIn4:
      case Register::kAux2AnalogIn5: {
        const int pin =
            static_cast<int>(reg) - static_cast<int>(Register::kAux2AnalogIn1);
        return ScalePwm(bldc_.aux2().analog_inputs[pin], type);
      }

      case Register::kModelNumber: {
        if (type != 2) { break; }

        return Value(vi32(MOTEUS_MODEL_NUMBER));
      }
      case Register::kFirmwareVersion: {
        if (type != 2) { break; }

        return Value(vi32(firmware_->firmware_version()));
      }
      case Register::kRegisterMapVersion: {
        if (type != 2) { break; }

        return Value(vi32(kRegisterMapVersion));
      }
      case Register::kSerialNumber1:
      case Register::kSerialNumber2:
      case Register::kSerialNumber3: {
        if (type != 2) { break; }

        const auto serial_number = firmware_->serial_number();
        const auto index =
            static_cast<int>(reg) -
            static_cast<int>(Register::kSerialNumber1);
        return Value(vi32(serial_number.number[index]));
      }
      case Register::kMultiplexId: {
        break;
      }
      case Register::kSetOutputNearest:
      case Register::kSetOutputExact:
      case Register::kRequireReindex: {
        break;
      }
    }

    // If we made it here, then we had an unknown register.
    return static_cast<uint32_t>(1);
  }

  AuxPort aux1_port_;
  AuxPort aux2_port_;
  MotorPosition motor_position_;
  Drv8323 drv8323_;
  BldcServo bldc_;
  FirmwareInfo* const firmware_;

  bool command_valid_ = false;
  BldcServo::CommandData command_;
};

MoteusController::MoteusController(micro::Pool* pool,
                                   micro::PersistentConfig* persistent_config,
                                   micro::CommandManager* command_manager,
                                   micro::TelemetryManager* telemetry_manager,
                                   MillisecondTimer* timer,
                                   FirmwareInfo* firmware)
    : impl_(pool, pool, persistent_config, command_manager, telemetry_manager,
            timer, firmware) {}

MoteusController::~MoteusController() {}

void MoteusController::Start() {
  impl_->Start();
}

void MoteusController::Poll() {
  impl_->Poll();
}

void MoteusController::PollMillisecond() {
  impl_->PollMillisecond();
}

BldcServo* MoteusController::bldc_servo() {
  return &impl_->bldc_;
}

multiplex::MicroServer::Server* MoteusController::multiplex_server() {
  return impl_.get();
}

}
