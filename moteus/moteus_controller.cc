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

#include "moteus/moteus_controller.h"

#include "mjlib/base/limit.h"

#include "moteus/moteus_hw.h"

namespace micro = mjlib::micro;
using mjlib::base::Limit;

namespace moteus {

using Value = micro::MultiplexProtocol::Value;

namespace {
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
  const float scaled = value / scale;
  const auto max = std::numeric_limits<T>::max();
  // We purposefully limit to +- max, rather than to min.  The minimum
  // value for our two's complement types is reserved.
  return Limit<T>(static_cast<T>(scaled), -max, max);
}

enum class Register {
  kMode = 0x000,
  kPosition = 0x001,
  kVelocity = 0x002,
  kTemperature = 0x003,
  kQCurrent = 0x004,
  kDCurrent = 0x005,
  kVoltage = 0x006,
  kFault = 0x007,

  kPwmPhaseA = 0x010,
  kPwmPhaseB = 0x011,
  kPwmPhaseC = 0x012,

  kVoltagePhaseA = 0x014,
  kVoltagePhaseB = 0x015,
  kVoltagePhaseC = 0x016,

  kVFocTheta = 0x018,
  kVFocVoltage = 0x019,

  kCommandQCurrent = 0x01c,
  kCommandDCurrent = 0x01d,

  kCommandPosition = 0x020,
  kCommandVelocity = 0x021,
  kCommandPositionMaxCurrent = 0x022,
  kCommandStopPosition = 0x023,
  kCommandFeedforwardCurrent = 0x024,
  kCommandKpScale = 0x025,
  kCommandKdScale = 0x026,

  // kCommand3dForceX = 0x030,
  // kCommand3dForceY = 0x031,
  // kCommand3dForceZ = 0x032,

  // kCommand3dPositionX = 0x040,
  // kCommand3dPositionY = 0x041,
  // kCommand3dPositionZ = 0x042,
  // kCommand3dVelocityX = 0x043,
  // kCommand3dVelocityY = 0x044,
  // kCommand3dVelocityZ = 0x045,

  // kCommand3dStopPlaneX = 0x046,
  // kCommand3dStopPlaneY = 0x047,
  // kCommand3dStopPlaneZ = 0x048,

  // kCommand3dStopPlaneNX = 0x049,
  // kCommand3dStopPlaneNY = 0x04a,
  // kCommand3dStopPlaneNZ = 0x04b,

  // kCommand3dMaximumForce = 0x04c,

  // kCommand3dFeedforwardX = 0x04d,
  // kCommand3dFeedforwardY = 0x04e,
  // kCommand3dFeedforwardZ = 0x04f,

  // kCommand3dKpScale00 = 0x050,
  // kCommand3dKpScale11 = 0x051,
  // kCommand3dKpScale22 = 0x052,
  // kCommand3dKpScale01 = 0x053,
  // kCommand3dKpScale12 = 0x054,
  // kCommand3dKpScale02 = 0x055,

  // kCommand3dKdScale00 = 0x056,
  // kCommand3dKdScale11 = 0x057,
  // kCommand3dKdScale22 = 0x058,
  // kCommand3dKdScale01 = 0x059,
  // kCommand3dKdScale12 = 0x05a,
  // kCommand3dKdScale02 = 0x05b,

  kModelNumber = 0x100,
  kSerialNumber = 0x101,
  kRegisterMapVersion = 0x103,
  kMultiplexId = 0x104,
};
}

class MoteusController::Impl : public micro::MultiplexProtocolServer::Server {
 public:
  Impl(micro::Pool* pool,
       micro::PersistentConfig* persistent_config,
       micro::TelemetryManager* telemetry_manager,
       MillisecondTimer* timer)
      : as5047_([]() {
          AS5047::Options options;
          options.mosi = PB_15;
          options.miso = PB_14;
          options.sck = PB_13;
          options.cs = PB_12;
          return options;
        }()),
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
        bldc_(pool, persistent_config, telemetry_manager, &as5047_, &drv8323_, []() {
            BldcServo::Options options;
            options.pwm1 = PA_0;
            options.pwm2 = PA_1;
            options.pwm3 = PA_2;

            options.current1 = PC_5;
            options.current2 = PB_0_ALT0;
            options.vsense = MOTEUS_VSENSE;
            options.tsense = MOTEUS_TSENSE;

            options.debug_out = PB_8;
            options.debug_uart_out = PC_10;

            return options;
          }()) {}

  void Start() {
    bldc_.Start();
  }

  void Poll() {
  }

  void PollMillisecond() {
    drv8323_.PollMillisecond();
    bldc_.PollMillisecond();
  }

  uint32_t Write(micro::MultiplexProtocol::Register,
                 const micro::MultiplexProtocol::Value&) override {
    return 0;
  }

  micro::MultiplexProtocol::ReadResult Read(
      micro::MultiplexProtocol::Register reg,
      size_t type) const override {
    switch (static_cast<Register>(reg)) {
      case Register::kMode: {
        return IntMapping(static_cast<int8_t>(bldc_.status().mode), type);
      }
      case Register::kPosition: {
        const auto position = bldc_.status().unwrapped_position;
        switch (type) {
          case 0: return ScaleSaturate<int8_t>(position, 0.01f);
          case 1: return ScaleSaturate<int16_t>(position, 0.001f);
          case 2: return ScaleSaturate<int32_t>(position, 0.00001f);
          case 3: return Value(position);
        }
        MJ_ASSERT(false);
        return Value(static_cast<int8_t>(0));
      }
      case Register::kVelocity: {
        const auto velocity = bldc_.status().velocity;
        switch (type) {
          case 0: return ScaleSaturate<int8_t>(velocity, 0.1f);
          case 1: return ScaleSaturate<int16_t>(velocity, 0.001f);
          case 2: return ScaleSaturate<int32_t>(velocity, 0.00001f);
          case 3: return Value(velocity);
        }
        MJ_ASSERT(false);
        return Value(static_cast<int8_t>(0));
      }
      case Register::kTemperature:
      case Register::kQCurrent:
      case Register::kDCurrent:
      case Register::kVoltage:
      case Register::kFault:
      case Register::kPwmPhaseA:
      case Register::kPwmPhaseB:
      case Register::kPwmPhaseC:
      case Register::kVoltagePhaseA:
      case Register::kVoltagePhaseB:
      case Register::kVoltagePhaseC:
      case Register::kVFocTheta:
      case Register::kVFocVoltage:
      case Register::kCommandQCurrent:
      case Register::kCommandDCurrent:
      case Register::kCommandPosition:
      case Register::kCommandVelocity:
      case Register::kCommandPositionMaxCurrent:
      case Register::kCommandStopPosition:
      case Register::kCommandFeedforwardCurrent:
      case Register::kCommandKpScale:
      case Register::kCommandKdScale: {
        break;
      }

      case Register::kModelNumber: {
        if (type == 2) {
          return Value(static_cast<int32_t>(MOTEUS_MODEL_NUMBER));
        }
        break;
      }
      case Register::kSerialNumber:
      case Register::kRegisterMapVersion:
      case Register::kMultiplexId: {
        break;
      }
    }

    // If we made it here, then we had an unknown register.
    return static_cast<uint32_t>(1);
  }

  AS5047 as5047_;
  Drv8323 drv8323_;
  BldcServo bldc_;
};

MoteusController::MoteusController(micro::Pool* pool,
                                   micro::PersistentConfig* persistent_config,
                                   micro::TelemetryManager* telemetry_manager,
                                   MillisecondTimer* timer)
    : impl_(pool, pool, persistent_config, telemetry_manager, timer) {}

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

AS5047* MoteusController::as5047() {
  return &impl_->as5047_;
}

Drv8323* MoteusController::drv8323() {
  return &impl_->drv8323_;
}

BldcServo* MoteusController::bldc_servo() {
  return &impl_->bldc_;
}

micro::MultiplexProtocolServer::Server* MoteusController::multiplex_server() {
  return impl_.get();
}

}
