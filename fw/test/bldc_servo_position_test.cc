// Copyright 2022 Josh Pieper, jjp@pobox.com.
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

#include "fw/bldc_servo_position.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace tt = boost::test_tools;

namespace {
constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

struct Context {
  BldcServoStatus status;
  BldcServoConfig config;
  BldcServoPositionConfig position_config;
  float motor_scale16 = 65536.0f / 1.0f;
  float rate_hz = 40000.0f;
  BldcServoCommandData data;

  Context() {
    position_config.position_min = NaN;
    position_config.position_max = NaN;

    data.mode = BldcServoMode::kPosition;
    set_position(3.2f);
  }

  void set_position(float val) {
    status.unwrapped_position = val;
    status.unwrapped_position_raw = to_raw(status.unwrapped_position);
  }

  int64_t to_raw(float val) const {
    return static_cast<int64_t>(val * motor_scale16) << 32;
  }

  float from_raw(int64_t val) const {
    return (val >> 32) / motor_scale16;
  }

  float Call() {
    return BldcServoPosition::UpdateCommand(
        &status,
        &config,
        &position_config,
        motor_scale16,
        rate_hz,
        &data,
        data.velocity);
  }
};
}


BOOST_AUTO_TEST_CASE(StartupPositionCapture) {
  // When starting, we capture the current position if no command
  // position is given.
  Context ctx;

  BOOST_TEST(!ctx.status.control_position);
  ctx.data.position = NaN;
  ctx.data.velocity = 0.0f;
  ctx.Call();
  BOOST_TEST(ctx.status.control_position.value() == ctx.status.unwrapped_position_raw);
}

BOOST_AUTO_TEST_CASE(StartupPositionSet) {
  // When starting, if a command position is given, we use that.
  Context ctx;

  BOOST_TEST(!ctx.status.control_position);
  ctx.data.position = 2.0f;
  ctx.data.velocity = 0.0f;
  ctx.Call();
  BOOST_TEST(ctx.status.control_position.value() != ctx.status.unwrapped_position_raw);
  BOOST_TEST(ctx.status.control_position.value() == ctx.to_raw(2.0f));
}

BOOST_AUTO_TEST_CASE(RunningPositionSet) {
  // When running, we still take actual command positions immediately.
  Context ctx;

  ctx.status.control_position = ctx.to_raw(4.0f);
  ctx.data.position = 2.0f;
  ctx.data.velocity = 0.0f;
  ctx.Call();

  BOOST_TEST(ctx.status.control_position.value() == ctx.to_raw(2.0f));
}

BOOST_AUTO_TEST_CASE(RunningPositionCapture) {
  // When running, an unset position means we keep the old one.
  Context ctx;

  ctx.status.control_position = ctx.to_raw(4.0f);
  ctx.data.position = NaN;
  ctx.data.velocity = 0.0f;
  ctx.Call();

  BOOST_TEST(ctx.status.control_position.value() == ctx.to_raw(4.0f));
}

BOOST_AUTO_TEST_CASE(PositionLimit) {
  struct TestCase {
    float position_min;
    float position_max;
    float control_position;

    float expected_output;
  };

  TestCase test_cases[] = {
    { NaN, NaN, 0.0f, 0.0f },
    { NaN, NaN, 100.0f, 100.0f },
    { NaN, NaN, -100.0f, -100.0f },
    { NaN, 3.0f, 0.0f, 0.0f },
    { NaN, 3.0f, 3.0f, 3.0f },
    { NaN, 3.0f, 10.0f, 3.0f },
    { NaN, 3.0f, -10.0f, -10.0f },
    { 1.0f, 3.0f, -10.0f, 1.0f },
    { 1.0f, 3.0f, 1.0f, 1.0f },
    { 1.0f, 3.0f, 1.5f, 1.5f },
    { 1.0f, 3.0f, 3.0f, 3.0f },
    { 1.0f, 3.0f, 4.0f, 3.0f },
    { 1.0f, NaN, 4.0f, 4.0f },
  };

  for (const auto& test_case : test_cases) {
    BOOST_TEST_CONTEXT("Case "
                       << test_case.position_min
                       << " " << test_case.position_max
                       << " " << test_case.control_position) {
      Context ctx;
      ctx.position_config.position_min = test_case.position_min;
      ctx.position_config.position_max = test_case.position_max;
      ctx.data.position = NaN;
      ctx.data.velocity = 0.0f;
      ctx.status.control_position = ctx.to_raw(test_case.control_position);

      ctx.Call();

      BOOST_TEST(ctx.status.control_position.value() ==
                 ctx.to_raw(test_case.expected_output));
    }
  }
}

BOOST_AUTO_TEST_CASE(PositionVelocity, * boost::unit_test::tolerance(1e-3f)) {
  Context ctx;

  ctx.data.position = NaN;
  ctx.data.velocity = 1.0f;
  ctx.status.control_position = ctx.to_raw(3.0f);
  for (int i = 0; i < ctx.rate_hz; i++) {
    const float result = ctx.Call();
    BOOST_TEST(result == 1.0f);
  }

  BOOST_TEST(ctx.from_raw(ctx.status.control_position.value()) == 4.0f);

  ctx.data.stop_position = 4.5f;
  for (int i = 0; i < ctx.rate_hz; i++) {
    const float result = ctx.Call();
    if (i < ctx.rate_hz / 2) {
      BOOST_TEST(result == 1.0f);
    } else {
      BOOST_TEST(result == 0.0f);
    }
  }
  BOOST_TEST(ctx.from_raw(ctx.status.control_position.value()) == 4.5f);
}

BOOST_AUTO_TEST_CASE(PositionSlip, * boost::unit_test::tolerance(1e-3f)) {
  struct TestCase {
    float max_position_slip;
    float control_position;
    float observed_position;

    float expected_position;
  };

  TestCase test_cases[] = {
    { NaN, 0.0f, 0.0f, 0.0f },
    { NaN, 1.0f, 0.0f, 1.0f },
    { NaN, -5.0f, 2.0f, -5.0f },
    { 2.0f, -5.0f, 2.0f, 0.0f },
    { 3.0f, -5.0f, 2.0f, -1.0f },
    { 3.0f, 6.0f, 2.0f, 5.0f },
  };

  for (const TestCase& test_case : test_cases) {
    BOOST_TEST_CONTEXT("Case "
                       << test_case.max_position_slip << " "
                       << test_case.control_position << " "
                       << test_case.observed_position) {
      Context ctx;
      ctx.data.position = NaN;

      ctx.config.max_position_slip = test_case.max_position_slip;
      ctx.status.control_position = ctx.to_raw(test_case.control_position);
      ctx.set_position(test_case.observed_position);

      ctx.Call();

      BOOST_TEST(ctx.from_raw(ctx.status.control_position.value()) ==
                 test_case.expected_position);
    }
  }
}
