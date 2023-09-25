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

#pragma once

#include <array>

#include "mjlib/base/visitor.h"

/// @file
///
/// Subsets of structures that the moteus controller reports used in
/// host side tools.

namespace moteus {
namespace tool {

struct ServoStats {
  enum Mode {
    kStopped = 0,
    kFault =1 ,
    kPwm = 5,
    kVoltage = 6,
    kVoltageFoc = 7,
    kVoltageDq = 8,
    kCurrent = 9,
    kPosition = 10,
    kPositionTimeout = 11,
    kZeroVelocity = 12,
  };
  Mode mode = kStopped;

  enum Fault {
    kNone = 0,
  };
  Fault fault = kNone;

  float position = 0.0f;

  float torque_Nm = 0.0f;

  float d_A = 0.0f;
  float q_A = 0.0f;

  float velocity = 0.0f;
  float velocity_filt = 0.0f;

  struct PidPosition {
    float error = 0.0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(error));
    }
  };

  PidPosition pid_position;

  uint32_t final_timer = 0;
  uint32_t total_timer = 0;


  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(mode));
    a->Visit(MJ_NVP(fault));
    a->Visit(MJ_NVP(position));
    a->Visit(MJ_NVP(torque_Nm));

    a->Visit(MJ_NVP(d_A));
    a->Visit(MJ_NVP(q_A));

    a->Visit(MJ_NVP(velocity));
    a->Visit(MJ_NVP(velocity_filt));

    a->Visit(MJ_NVP(pid_position));

    a->Visit(MJ_NVP(final_timer));
    a->Visit(MJ_NVP(total_timer));
  }
};

struct Firmware {
  uint32_t version = 0;
  std::array<uint32_t, 3> serial_number = {};
  uint32_t model = 0;
  uint8_t hwrev = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(version));
    a->Visit(MJ_NVP(serial_number));
    a->Visit(MJ_NVP(model));
    a->Visit(MJ_NVP(hwrev));
  }
};

struct Git {
  std::array<uint8_t, 20> hash = {{}};
  bool dirty = false;
  uint64_t timestamp = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(hash));
    a->Visit(MJ_NVP(dirty));
    a->Visit(MJ_NVP(timestamp));
  }
};

}  // namespace tool
}  // namespace moteus

namespace mjlib {
namespace base {
template <>
struct IsEnum<moteus::tool::ServoStats::Mode> {
  static constexpr bool value = true;

  using M = moteus::tool::ServoStats::Mode;
  static std::array<std::pair<M, const char*>, 1> map() {
    return { {
        { M::kStopped, "stopped" },
            }};
  }
};

template <>
struct IsEnum<moteus::tool::ServoStats::Fault> {
  static constexpr bool value = true;

  using M = moteus::tool::ServoStats::Fault;
  static std::array<std::pair<M, const char*>, 1> map() {
    return { {
        { M::kNone, "none" },
            }};
  }
};
}  // namespace base
}  // namespace mjlib
