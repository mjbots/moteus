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

/// @file
///
/// This file contains primitives used to encode and decode CAN-FD
/// messages for the mjbots moteus brushless controller.  While C++,
/// it is designed to be usable in "minimal-C++" environments like
/// Arduino, in addition to fully standards conforming environments.

#pragma once

#include <limits>

#include "mjbots/moteus/moteus_multiplex.h"

namespace mjbots {
namespace moteus {

/// The expected version associated with register 0x102.  If it
/// differs from this, then semantics of one or more registers may
/// have changed.
enum {
  kCurrentRegisterMapVersion = 5,
};

enum Register : uint32_t {
  kMode = 0x000,
  kPosition = 0x001,
  kVelocity = 0x002,
  kTorque = 0x003,
  kQCurrent = 0x004,
  kDCurrent = 0x005,
  kAbsPosition = 0x006,

  kMotorTemperature = 0x00a,
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

  kCommandPosition = 0x020,
  kCommandVelocity = 0x021,
  kCommandFeedforwardTorque = 0x022,
  kCommandKpScale = 0x023,
  kCommandKdScale = 0x024,
  kCommandPositionMaxTorque = 0x025,
  kCommandStopPosition = 0x026,
  kCommandTimeout = 0x027,

  kPositionKp = 0x030,
  kPositionKi = 0x031,
  kPositionKd = 0x032,
  kPositionFeedforward = 0x033,
  kPositionCommand = 0x034,

  kControlPosition = 0x038,
  kControlVelocity = 0x039,
  kControlTorque = 0x03a,
  kControlPositionError = 0x03b,
  kControlVelocityError = 0x03c,
  kControlTorqueError = 0x03d,

  kCommandStayWithinLowerBound = 0x040,
  kCommandStayWithinUpperBound = 0x041,
  kCommandStayWithinFeedforwardTorque = 0x042,
  kCommandStayWithinKpScale = 0x043,
  kCommandStayWithinKdScale = 0x044,
  kCommandStayWithinPositionMaxTorque = 0x045,
  kCommandStayWithinTimeout = 0x046,

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

  kMillisecondCounter = 0x070,
  kClockTrim = 0x071,

  kRegisterMapVersion = 0x102,
  kSerialNumber = 0x120,
  kSerialNumber1 = 0x120,
  kSerialNumber2 = 0x121,
  kSerialNumber3 = 0x122,

  kRezero = 0x130,
  kSetOutputNearest = 0x130,
  kSetOutputExact = 0x131,
  kRequireReindex = 0x132,

  kDriverFault1 = 0x140,
  kDriverFault2 = 0x141,
};

enum class Mode {
  kStopped = 0,
  kFault = 1,
  kEnabling = 2,
  kCalibrating = 3,
  kCalibrationComplete = 4,
  kPwm = 5,
  kVoltage = 6,
  kVoltageFoc = 7,
  kVoltageDq = 8,
  kCurrent = 9,
  kPosition = 10,
  kPositionTimeout = 11,
  kZeroVelocity = 12,
  kStayWithin = 13,
  kMeasureInd = 14,
  kBrake = 15,
  kNumModes,
};

enum class HomeState {
  kRelative = 0,
  kRotor = 1,
  kOutput = 2,
};

struct Query {
  struct Result {
    Mode mode = Mode::kStopped;
    double position = std::numeric_limits<double>::quiet_NaN();
    double velocity = std::numeric_limits<double>::quiet_NaN();
    double torque = std::numeric_limits<double>::quiet_NaN();
    double q_current = std::numeric_limits<double>::quiet_NaN();
    double d_current = std::numeric_limits<double>::quiet_NaN();
    double abs_position = std::numeric_limits<double>::quiet_NaN();
    double motor_temperature = std::numeric_limits<double>::quiet_NaN();
    HomeState home_state = HomeState::kRelative;
    double voltage = std::numeric_limits<double>::quiet_NaN();
    double temperature = std::numeric_limits<double>::quiet_NaN();
    int fault = 0;

    int8_t aux1_gpio = 0;
    int8_t aux2_gpio = 0;
  };

  struct Format {
    Resolution mode = kInt8;
    Resolution position = kFloat;
    Resolution velocity = kFloat;
    Resolution torque = kFloat;
    Resolution q_current = kIgnore;
    Resolution d_current = kIgnore;
    Resolution abs_position = kIgnore;
    Resolution motor_temperature = kIgnore;
    Resolution trajectory_complete = kIgnore;
    Resolution rezero_state = kIgnore;
    Resolution home_state = kIgnore;
    Resolution voltage = kInt8;
    Resolution temperature = kInt8;
    Resolution fault = kInt8;

    Resolution aux1_gpio = kIgnore;
    Resolution aux2_gpio = kIgnore;
  };

  static void Make(WriteCanFrame* frame, const Format& format) {
    {
      const Resolution kResolutions[] = {
        format.mode,
        format.position,
        format.velocity,
        format.torque,
        format.q_current,
        format.d_current,
        format.abs_position,
      };
      const auto kResolutionsSize = sizeof(kResolutions) / sizeof(*kResolutions);
      WriteCombiner combiner(
          frame, 0x10, Register::kMode,
          kResolutions, kResolutionsSize);
      for (size_t i = 0; i < kResolutionsSize; i++) {
        combiner.MaybeWrite();
      }
    }

    {
      const Resolution kResolutions[] = {
        format.motor_temperature,
        format.trajectory_complete,
        format.home_state,
        format.voltage,
        format.temperature,
        format.fault,
      };
      const auto kResolutionsSize = sizeof(kResolutions) / sizeof(*kResolutions);
      WriteCombiner combiner(
          frame, 0x10, Register::kMotorTemperature,
          kResolutions, kResolutionsSize);
      for (size_t i = 0; i < kResolutionsSize; i++) {
        combiner.MaybeWrite();
      }
    }

    {
      const Resolution kResolutions[] = {
        format.aux1_gpio,
        format.aux2_gpio,
      };
      const auto kResolutionsSize = sizeof(kResolutions) / sizeof(*kResolutions);
      WriteCombiner combiner(
          frame, 0x10, Register::kAux1GpioStatus,
          kResolutions, kResolutionsSize);
      for (size_t i = 0; i < kResolutionsSize; i++) {
        combiner.MaybeWrite();
      }
    }
  }

  static Result Parse(const CanFrame* frame) {
    MultiplexParser parser(frame);

    Result result;

    while (true) {
      const auto current = parser.next();
      if (current.done) { return result; }

      const auto res = current.resolution;
      switch (static_cast<Register>(current.value)) {
        case Register::kMode: {
          result.mode = static_cast<Mode>(parser.ReadInt(res));
          break;
        }
        case Register::kPosition: {
          result.position = parser.ReadPosition(res);
          break;
        }
        case Register::kVelocity: {
          result.velocity = parser.ReadVelocity(res);
          break;
        }
        case Register::kTorque: {
          result.torque = parser.ReadTorque(res);
          break;
        }
        case Register::kQCurrent: {
          result.q_current = parser.ReadCurrent(res);
          break;
        }
        case Register::kDCurrent: {
          result.d_current = parser.ReadCurrent(res);
          break;
        }
        case Register::kAbsPosition: {
          result.abs_position = parser.ReadPosition(res);
          break;
        }
        case Register::kMotorTemperature: {
          result.motor_temperature = parser.ReadTemperature(res);
          break;
        }
        case Register::kHomeState: {
          result.home_state = static_cast<HomeState>(parser.ReadInt(res));
          break;
        }
        case Register::kVoltage: {
          result.voltage = parser.ReadVoltage(res);
          break;
        }
        case Register::kTemperature: {
          result.temperature = parser.ReadTemperature(res);
          break;
        }
        case Register::kFault: {
          result.fault = parser.ReadInt(res);
          break;
        }
        case Register::kAux1GpioStatus: {
          result.aux1_gpio = parser.ReadInt(res);
          break;
        }
        case Register::kAux2GpioStatus: {
          result.aux2_gpio = parser.ReadInt(res);
          break;
        }
      }
    }
  }
};

struct PositionMode {
  struct Command {
    double position = 0.0;
    double velocity = 0.0;
    double feedforward_torque = 0.0;
    double kp_scale = 1.0;
    double kd_scale = 1.0;
    double maximum_torque = std::numeric_limits<double>::quiet_NaN();
    double stop_position = std::numeric_limits<double>::quiet_NaN();
    double watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
    double velocity_limit = std::numeric_limits<double>::quiet_NaN();
    double accel_limit = std::numeric_limits<double>::quiet_NaN();
    double fixed_voltage_override = std::numeric_limits<double>::quiet_NaN();
  };

  struct Format {
    Resolution position = kFloat;
    Resolution velocity = kFloat;
    Resolution feedforward_torque = kFloat;
    Resolution kp_scale = kFloat;
    Resolution kd_scale = kFloat;
    Resolution maximum_torque = kFloat;
    Resolution stop_position = kFloat;
    Resolution watchdog_timeout = kFloat;
    Resolution velocity_limit = kFloat;
    Resolution accel_limit = kFloat;
    Resolution fixed_voltage_override = kFloat;
  };

  static void Make(WriteCanFrame* frame,
                   const Command& command,
                   const Format& format) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kPosition);

    // Now we use some heuristics to try and group consecutive registers
    // of the same resolution together into larger writes.
    const Resolution kResolutions[] = {
      format.position,
      format.velocity,
      format.feedforward_torque,
      format.kp_scale,
      format.kd_scale,
      format.maximum_torque,
      format.stop_position,
      format.watchdog_timeout,
      format.velocity_limit,
      format.accel_limit,
      format.fixed_voltage_override,
    };
    WriteCombiner combiner(
        frame, 0x00,
        Register::kCommandPosition,
        kResolutions,
        sizeof(kResolutions) / sizeof(*kResolutions));

    if (combiner.MaybeWrite()) {
      frame->WritePosition(command.position, format.position);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteVelocity(command.velocity, format.velocity);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTorque(command.feedforward_torque, format.feedforward_torque);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePwm(command.kp_scale, format.kp_scale);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePwm(command.kd_scale, format.kd_scale);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTorque(command.maximum_torque, format.maximum_torque);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePosition(command.stop_position, format.stop_position);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTime(command.watchdog_timeout, format.watchdog_timeout);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteVelocity(command.velocity_limit, format.velocity_limit);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteAccel(command.accel_limit, format.accel_limit);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteVoltage(command.fixed_voltage_override,
                          format.fixed_voltage_override);
    }
  }
};

/// The voltage-mode FOC command.
struct VFOCMode {
  struct Command {
    double theta = 0.0;
    double voltage = 0.0;
    double theta_rate = 0.0;
  };

  struct Format {
    Resolution theta = kFloat;
    Resolution voltage = kFloat;
    Resolution theta_rate = kFloat;
  };

  static void Make(WriteCanFrame* frame,
                   const Command& command,
                   const Format& format) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kVoltageFoc);

    const Resolution kResolutions[] = {
      format.theta,
      format.voltage,
      kIgnore,
      kIgnore,
      kIgnore,
      kIgnore,
      format.theta_rate,
    };
    WriteCombiner combiner(
        frame, 0x00,
        Register::kCommandPosition,
        kResolutions,
        sizeof(kResolutions) / sizeof(*kResolutions));

    if (combiner.MaybeWrite()) {
      frame->WritePwm(command.theta / M_PI, format.theta);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteVoltage(command.voltage, format.voltage);
    }
    if (combiner.MaybeWrite()) {
      abort();
    }
    if (combiner.MaybeWrite()) {
      abort();
    }
    if (combiner.MaybeWrite()) {
      abort();
    }
    if (combiner.MaybeWrite()) {
      abort();
    }
    if (combiner.MaybeWrite()) {
      frame->WriteVelocity(command.theta / M_PI, format.theta_rate);
    }
  }
};

/// DQ phase current command.
struct CurrentMode {
  struct Command {
    double d_A = 0.0;
    double q_A = 0.0;
  };

  struct Format {
    Resolution d_A = kFloat;
    Resolution q_A = kFloat;
  };

  static void Make(WriteCanFrame* frame,
                   const Command& command,
                   const Format& format) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kCurrent);

    const Resolution kResolutions[] = {
      format.d_A,
      format.q_A,
    };

    WriteCombiner combiner(
        frame, 0x00,
        Register::kCommandPosition,
        kResolutions,
        sizeof(kResolutions) / sizeof(*kResolutions));

    if (combiner.MaybeWrite()) {
      frame->WriteCurrent(command.d_A, format.d_A);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteCurrent(command.q_A, format.q_A);
    }
  }
};

struct StayWithinMode {
  struct Command {
    double lower_bound = std::numeric_limits<double>::quiet_NaN();
    double upper_bound = std::numeric_limits<double>::quiet_NaN();
    double feedforward_torque = 0.0;
    double kp_scale = 1.0;
    double kd_scale = 1.0;
    double maximum_torque = 0.0;
    double watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
  };

  struct Format {
    Resolution lower_bound = kFloat;
    Resolution upper_bound = kFloat;
    Resolution feedforward_torque = kIgnore;
    Resolution kp_scale = kIgnore;
    Resolution kd_scale = kIgnore;
    Resolution maximum_torque = kIgnore;
    Resolution watchdog_timeout = kIgnore;
  };

  static void Write(WriteCanFrame* frame,
                    const Command& command,
                    const Format& format) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kStayWithin);

    const Resolution kResolutions[] = {
      format.lower_bound,
      format.upper_bound,
      format.feedforward_torque,
      format.kp_scale,
      format.kd_scale,
      format.maximum_torque,
      format.watchdog_timeout,
    };

    WriteCombiner combiner(
        frame, 0x00,
        Register::kCommandStayWithinLowerBound,
        kResolutions,
        sizeof(kResolutions) / sizeof(*kResolutions));

    if (combiner.MaybeWrite()) {
      frame->WritePosition(command.lower_bound, format.lower_bound);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePosition(command.upper_bound, format.upper_bound);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTorque(command.feedforward_torque,
                         format.feedforward_torque);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePwm(command.kp_scale, format.kp_scale);
    }
    if (combiner.MaybeWrite()) {
      frame->WritePwm(command.kd_scale, format.kd_scale);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTorque(command.maximum_torque, format.maximum_torque);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteTime(command.watchdog_timeout, format.watchdog_timeout);
    }
  }
};

struct BrakeMode {
  struct Command {};
  struct Format {};

  static void Make(WriteCanFrame* frame,
                   const Command&,
                   const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kBrake);
  }
};

struct StopMode {
  struct Command {};
  struct Format {};

  static void Make(WriteCanFrame* frame,
                   const Command&,
                   const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->Write<int8_t>(Register::kMode);
    frame->Write<int8_t>(Mode::kStopped);
  }
};

struct GpioWrite {
  struct Command {
    int8_t aux1 = 0;
    int8_t aux2 = 0;
  };

  struct Format {
    Resolution aux1 = kInt8;
    Resolution aux2 = kInt8;
  };

  static void Write(WriteCanFrame* frame,
                    const Command& command,
                    const Format& format) {
    const Resolution kResolutions[] = {
      format.aux1,
      format.aux2,
    };

    WriteCombiner combiner(
        frame, 0x00,
        Register::kAux1GpioCommand,
        kResolutions,
        sizeof(kResolutions) / sizeof(*kResolutions));

    if (combiner.MaybeWrite()) {
      frame->WriteInt(command.aux1, format.aux1);
    }
    if (combiner.MaybeWrite()) {
      frame->WriteInt(command.aux2, format.aux2);
    }
  }
};

struct OutputNearest {
  struct Command {
    double position = 0.0;
  };

  static void Make(WriteCanFrame* frame, const Command& command) {
    frame->Write<int8_t>(Multiplex::kWriteFloat | 0x01);
    frame->WriteVaruint(Register::kSetOutputNearest);
    frame->Write<float>(command.position);
  }
};

struct OutputExact {
  struct Command {
    double position = 0.0;
  };

  struct Format {};

  static void Make(WriteCanFrame* frame, const Command& command, const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteFloat | 0x01);
    frame->WriteVaruint(Register::kSetOutputExact);
    frame->Write<float>(command.position);
  }
};

struct RequireReindex {
  struct Command {};
  struct Format {};

  static void Make(WriteCanFrame* frame, const Command&, const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteInt8 | 0x01);
    frame->WriteVaruint(Register::kRequireReindex);
    frame->Write<int8_t>(1);
  }
};

struct DiagnosticWrite {
  struct Command {
    int8_t channel = 1;
    const char* data = nullptr;
    size_t size = 0;
  };

  struct Format {};

  static void Make(WriteCanFrame* frame, const Command& command, const Format&) {
    frame->Write<int8_t>(Multiplex::kClientToServer);
    frame->Write<int8_t>(command.channel);
    frame->Write<int8_t>(command.size);
    frame->Write(command.data, command.size);
  }
};

struct DiagnosticRead {
  struct Command {
    int8_t channel = 1;
    int8_t max_length = 48;
  };

  struct Format {};

  static void Make(WriteCanFrame* frame, const Command& command, const Format&) {
    frame->Write<int8_t>(Multiplex::kClientPollServer);
    frame->Write<int8_t>(command.channel);
    frame->Write<int8_t>(command.max_length);
  }
};

struct ClockTrim {
  struct Command {
    int32_t trim = 0;
  };

  struct Format {};

  static void Make(WriteCanFrame* frame, const Command& command, const Format&) {
    frame->Write<int8_t>(Multiplex::kWriteInt32 | 0x01);
    frame->WriteVaruint(Register::kClockTrim);
    frame->Write<int32_t>(command.trim);
  }
};

}
}
