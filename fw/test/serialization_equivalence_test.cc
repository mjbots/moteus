// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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
/// Verify that the compact mjlib::micro::SerializableHandler produces
/// output identical to the original templated implementation for the
/// firmware's persistent configuration and telemetry structures.  In
/// particular, the configuration schema must be byte-for-byte
/// identical, as its CRC gates loading configuration from flash.

#include <boost/test/auto_unit_test.hpp>

#include <cstdio>
#include <string>

#include "mjlib/micro/test/serializable_handler_equivalence.h"

#include "fw/aux_common.h"
#include "fw/bldc_servo_structs.h"
#include "fw/motor_position.h"

namespace micro = mjlib::micro;

namespace {

bool IsOptionalKey(std::string_view key, std::string_view) {
  for (const auto optional : {
           "control_position_raw",
           "control_velocity",
           "position_relative_raw",
           "stop_position_relative_raw",
         }) {
    if (key.substr(0, std::string_view(optional).size()) == optional) {
      return true;
    }
  }
  return false;
}

/// Check the default instance, then one where every field has been
/// set to a pseudo-random value.
template <typename T>
void Check(const char* name) {
  BOOST_TEST_CONTEXT(name) {
    micro::test::EquivalenceOptions options;
    options.skip_set = IsOptionalKey;
    options.max_corruptions = 600;

    const T initial;
    const auto keys = micro::test::CheckEquivalence<T>(initial, options);
    BOOST_TEST(keys.size() > 0);

    T modified;
    micro::SerializableHandler<T> handler(&modified);
    uint32_t state = 0x12345678;
    for (const auto& key : keys) {
      state = state * 1664525u + 1013904223u;
      char value[32] = {};
      // A mix of small integers (valid for enums and bools) and
      // arbitrary values.
      if (state & 0x80000000u) {
        ::snprintf(value, sizeof(value), "%d", (state >> 8) % 3);
      } else {
        ::snprintf(value, sizeof(value), "%d.%d",
                   static_cast<int>((state >> 8) % 2000) - 1000,
                   static_cast<int>(state % 100));
      }
      BOOST_TEST(handler.Set(key, value) == 0);
      // An enumeration outside of its range of values would be
      // undefined behavior for the original implementation.
      if (micro::test::HasInvalidValue(&modified)) {
        BOOST_TEST(handler.Set(key, "0") == 0);
      }
    }
    micro::test::CheckEquivalence<T>(modified, options);
  }
}

}

BOOST_AUTO_TEST_CASE(SerializationEquivalenceTest) {
  Check<moteus::BldcServoConfig>("BldcServoConfig");
  Check<moteus::BldcServoMotor>("BldcServoMotor");
  Check<moteus::BldcServoPositionConfig>("BldcServoPositionConfig");
  Check<moteus::BldcServoStatus>("BldcServoStatus");
  Check<moteus::BldcServoCommandData>("BldcServoCommandData");
  Check<moteus::BldcServoControl_Control>("BldcServoControl_Control");
  Check<moteus::aux::AuxConfig>("AuxConfig");
  Check<moteus::aux::AuxStatus>("AuxStatus");
  Check<moteus::MotorPosition::Config>("MotorPosition::Config");
  Check<moteus::MotorPosition::Status>("MotorPosition::Status");
}
