// Copyright 2021 Josh Pieper, jjp@pobox.com.
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

#pragma once

#include "PinNames.h"

#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/millisecond_timer.h"

namespace moteus {

/// Manages the ABS port on the moteus controller.
class AbsPort {
 public:
  struct Options {
    PinName scl = NC;
    PinName sda = NC;
  };

  AbsPort(mjlib::micro::Pool*,
          mjlib::micro::PersistentConfig* persistent_config,
          mjlib::micro::TelemetryManager* telemetry_manager,
          MillisecondTimer* timer,
          const Options&);
  ~AbsPort();

  void PollMillisecond();
  void Poll();

  enum Mode {
    kDisabled = 0,
    kAs5048 = 1,
    kAs5600 = 2,
    kNumModes = 3,
  };

  struct Config {
    Mode mode = kDisabled;

    int32_t i2c_hz = 400000;
    // 0 = standard, 1 = fast, 2 = fast+
    int32_t i2c_mode = 1;

    uint8_t encoder_i2c_address = 0x40;
    int32_t encoder_poll_ms = 10;

    // These are used to convert the "raw" encoder value into a
    // position measured in revolutions.
    //
    // revolutions = wrap_i16(raw + offset) / 65536.0 * scale
    uint16_t position_offset = 0;
    float position_scale = 1.0f;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(i2c_hz));
      a->Visit(MJ_NVP(i2c_mode));
      a->Visit(MJ_NVP(encoder_i2c_address));
      a->Visit(MJ_NVP(encoder_poll_ms));
      a->Visit(MJ_NVP(position_offset));
      a->Visit(MJ_NVP(position_scale));
    }
  };

  struct Status {
    float position = 0.0f;

    uint16_t encoder_raw = 0;
    bool encoder_valid = false;

    // Status and diagnostic values for AMS style encoders.
    uint8_t ams_agc = 0;
    uint8_t ams_diag = 0;
    uint16_t ams_mag = 0;

    uint32_t encoder_error_count = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(position));
      a->Visit(MJ_NVP(encoder_raw));
      a->Visit(MJ_NVP(encoder_valid));
      a->Visit(MJ_NVP(ams_agc));
      a->Visit(MJ_NVP(ams_diag));
      a->Visit(MJ_NVP(ams_mag));
      a->Visit(MJ_NVP(encoder_error_count));
    }
  };

  const Status& status() const;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};
}

namespace mjlib {
namespace base {

template <>
struct IsEnum<moteus::AbsPort::Mode> {
  static constexpr bool value = true;

  using M = moteus::AbsPort::Mode;
  static std::array<std::pair<M, const char*>,
                    static_cast<int>(M::kNumModes)> map() {
    return { {
        { M::kDisabled, "disabled" },
        { M::kAs5048, "as5048" },
        { M::kAs5600, "as5600" },
    } };
  }
};

}
}
