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

#include "fw/bldc_servo_position.h"

#include <fstream>

#include <fmt/format.h>

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace tt = boost::test_tools;

// Set to true to enable verbose trajectory debug output in tests.
#ifndef TRAJECTORY_DEBUG_OUTPUT
#define TRAJECTORY_DEBUG_OUTPUT false
#endif

namespace {
constexpr bool kTrajectoryDebug = TRAJECTORY_DEBUG_OUTPUT;
constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

struct Context {
  BldcServoStatus status;
  BldcServoConfig config;
  BldcServoPositionConfig position_config;
  MotorPosition::Status position;
  float rate_hz = 40000.0f;
  BldcServoCommandData data;

  Context() {
    position_config.position_min = NaN;
    position_config.position_max = NaN;
    status.motor_max_velocity = 100.0f;

    data.mode = BldcServoMode::kPosition;
    set_position(3.2f);
  }

  void set_position(float val) {
    position.position = val;
    position.position_raw = to_raw(position.position);
    position.position_relative = val;
    position.position_relative_raw = position.position_raw;
  }

  void set_stop_position(float val) {
    data.stop_position = val;
    data.stop_position_relative_raw = to_raw(val);
  }

  void set_velocity(float val) {
    position.velocity = val;
    status.velocity_filt = val;
  }

  int64_t to_raw(double val) const {
    return static_cast<int64_t>(val * (1ll << 48));
  }

  double from_raw(int64_t val) const {
    return static_cast<double>(val) / static_cast<double>(1ll << 48);
  }

  float Call() {
    if (!std::isnan(data.position) && !data.position_relative_raw) {
      data.position_relative_raw = MotorPosition::FloatToInt(data.position);
    } else if (std::isnan(data.position)) {
      data.position_relative_raw.reset();
    }
    return BldcServoPosition::UpdateCommand(
        &status,
        &config,
        &position_config,
        &position,
        0,
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

  BOOST_TEST(!ctx.status.control_position_raw);
  ctx.data.position = NaN;
  ctx.data.velocity = 0.0f;
  ctx.Call();
  BOOST_TEST(ctx.status.control_position_raw.value() == ctx.position.position_raw);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
}

BOOST_AUTO_TEST_CASE(StartupVelocityCapture,
                     * boost::unit_test::tolerance(1e-2)) {
  struct TestCase {
    double input_velocity;
    double capture_threshold;

    double expected_capture;
  };

  TestCase test_cases[] = {
    // a variance of 0.001 is a stddev of 0.0316, 6x that is 0.190
    { 0.0,  0.1,    0.00, },
    { 1.0,  0.1,    1.00, },
    {-1.0,  0.1,   -1.00, },
    { 0.01, 0.1,    0.00, },
    { 0.09, 0.1,    0.00, },
    { 0.11, 0.1,    0.11, },
    {-0.09, 0.1,    0.00, },
    {-0.11, 0.1,   -0.11, },

    { 0.11, 0.2,    0.00,  },
    { 0.19, 0.2,    0.00,  },
    { 0.21, 0.2,    0.21, },

    // and with the threshold set to 0, anything is captured
    { 0.01, 0.0,    0.01 },
    {-0.01, 0.0,   -0.01 },
  };

  for (const auto& test_case : test_cases) {
    BOOST_TEST_CONTEXT("Case "
                       << test_case.input_velocity << " "
                       << test_case.capture_threshold) {
      Context ctx;

      ctx.data.position = 10.0;
      ctx.data.velocity = 0.0;
      ctx.data.accel_limit = 1.0f;
      ctx.data.velocity_limit = 1.0f;

      ctx.position.velocity = test_case.input_velocity;
      ctx.status.velocity_filt = test_case.input_velocity;
      ctx.config.velocity_zero_capture_threshold = test_case.capture_threshold;

      BOOST_TEST(!ctx.status.control_velocity);
      ctx.Call();
      BOOST_TEST(ctx.status.control_velocity.value() ==
                 test_case.expected_capture);
    }
  }
}

BOOST_AUTO_TEST_CASE(StartupPositionSet) {
  // When starting, if a command position is given, we use that.
  Context ctx;

  BOOST_TEST(!ctx.status.control_position_raw);
  ctx.data.position = 2.0f;
  ctx.data.velocity = 0.0f;
  ctx.Call();
  BOOST_TEST(ctx.status.control_position_raw.value() != ctx.position.position_raw);
  BOOST_TEST(ctx.status.control_position_raw.value() == ctx.to_raw(2.0f));
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);

  BOOST_TEST(std::isfinite(ctx.data.velocity));
  BOOST_TEST(!ctx.data.position_relative_raw);
}

BOOST_AUTO_TEST_CASE(StartupVelocityUnset) {
  // When starting with no limits and a set position if we have a
  // non-finite velocity, then 0 is assumed.
  Context ctx;
  ctx.data.position = 2.0f;
  ctx.data.velocity = NaN;

  ctx.position.velocity = 3.0f;
  ctx.status.velocity_filt = 3.0f;

  ctx.Call();

  BOOST_TEST(ctx.status.control_position_raw.value() == ctx.to_raw(2.0f));
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
}

BOOST_AUTO_TEST_CASE(RunningPositionSet) {
  // When running, we still take actual command positions immediately.
  Context ctx;

  ctx.status.control_position_raw = ctx.to_raw(4.0f);
  ctx.data.position = 2.0f;
  ctx.data.velocity = 0.0f;
  ctx.Call();

  BOOST_TEST(ctx.status.control_position_raw.value() == ctx.to_raw(2.0f));
}

BOOST_AUTO_TEST_CASE(RunningPositionCapture) {
  // When running, an unset position means we keep the old one.
  Context ctx;

  ctx.status.control_position_raw = ctx.to_raw(4.0f);
  ctx.data.position = NaN;
  ctx.data.velocity = 0.0f;
  ctx.Call();

  BOOST_TEST(ctx.status.control_position_raw.value() == ctx.to_raw(4.0f));
}

BOOST_AUTO_TEST_CASE(RunningVelocityCapture) {
  // When running, an unset velocity is treated the same as 0.0.
  Context ctx;

  ctx.status.control_position_raw = ctx.to_raw(4.0f);
  ctx.status.control_velocity = 2.0f;
  ctx.data.position = 0.5f;
  ctx.data.velocity = NaN;
  ctx.Call();

  BOOST_TEST(ctx.from_raw(ctx.status.control_position_raw.value()) == 0.5f,
             boost::test_tools::tolerance(1e-3));
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
}

BOOST_AUTO_TEST_CASE(PositionLimit) {
  struct TestCase {
    float position_min;
    float position_max;
    float control_position_raw;

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
                       << " " << test_case.control_position_raw) {
      Context ctx;
      ctx.position_config.position_min = test_case.position_min;
      ctx.position_config.position_max = test_case.position_max;
      ctx.data.position = NaN;
      ctx.data.velocity = 0.0f;
      ctx.status.control_position_raw = ctx.to_raw(test_case.control_position_raw);

      ctx.Call();

      BOOST_TEST(ctx.status.control_position_raw.value() ==
                 ctx.to_raw(test_case.expected_output));
    }
  }
}

BOOST_AUTO_TEST_CASE(PositionVelocity, * boost::unit_test::tolerance(1e-3)) {
  Context ctx;

  ctx.data.position = NaN;
  ctx.data.velocity = 1.0f;
  ctx.status.control_position_raw = ctx.to_raw(3.0f);
  for (int i = 0; i < ctx.rate_hz; i++) {
    const float result = ctx.Call();
    BOOST_TEST(result == 1.0f);
  }

  BOOST_TEST(ctx.from_raw(ctx.status.control_position_raw.value()) == 4.0);

  ctx.set_stop_position(4.5f);
  for (int i = 0; i < ctx.rate_hz; i++) {
    const float result = ctx.Call();
    if (i < ctx.rate_hz / 2) {
      BOOST_TEST(result == 1.0f);
    } else {
      BOOST_TEST(result == 0.0f);
    }
  }
  BOOST_TEST(ctx.from_raw(ctx.status.control_position_raw.value()) == 4.5);
}

BOOST_AUTO_TEST_CASE(PositionSlip, * boost::unit_test::tolerance(1e-3f)) {
  struct TestCase {
    float max_position_slip;
    float control_position_raw;
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
                       << test_case.control_position_raw << " "
                       << test_case.observed_position) {
      Context ctx;
      ctx.data.position = NaN;

      ctx.config.max_position_slip = test_case.max_position_slip;
      ctx.status.control_position_raw = ctx.to_raw(test_case.control_position_raw);
      ctx.set_position(test_case.observed_position);

      ctx.Call();

      BOOST_TEST(ctx.from_raw(ctx.status.control_position_raw.value()) ==
                 test_case.expected_position);
    }
  }
}


// Limit things to test
//
// * All works with an unwrapped_position_scale configured
// * wraparound when running in "velocity mode"


BOOST_AUTO_TEST_CASE(AccelVelocityLimits, * boost::unit_test::tolerance(1e-3)) {
  const bool write_logs = false;

  struct TestCase {
    double x0;
    double v0;

    double xf;
    double vf;

    double a;
    double v;
    double rate_khz;

    double expected_coast_duration;
    double expected_total_duration;
  };

  TestCase test_cases[] = {
    ///////////////////////////////////
    // "velocity mode"
    { 0.0,  0.0,   NaN,  0.5,   1.0, 2.0, 40,    0.000, 0.500 },
    { 0.0,  1.0,   NaN, -0.5,   1.0, 2.0, 40,    0.000, 1.500 },
    { 0.0, -2.0,   NaN,  0.0,   1.0, 2.0, 40,    0.000, 2.000 },
    { 0.0, -2.0,   NaN,  0.0,   2.0, 2.0, 40,    0.000, 1.000 },
    { 0.0,  0.5,   NaN,  2.0,   2.0, 1.0, 40,    1.000, 0.250 },
    { 0.0,  0.5,   NaN,  2.0,   2.0, NaN, 40,    0.000, 0.750 },

    { 0.0,  0.5,   NaN,  2.0,   NaN, 4.0, 40,    0.000, 0.000 },
    { 0.0,  0.5,   NaN, -2.0,   NaN, 4.0, 40,    0.000, 0.000 },
    { 0.0,  0.5,   NaN,  6.0,   NaN, 4.0, 40,    1.000, 0.000 },
    { 0.0,  0.5,   NaN, -6.0,   NaN, 4.0, 40,    1.000, 0.000 },

    /////////////////////////////////
    // No accel limit.
    { 0.0,  0.0,   5.0, 0.0,    NaN, 1.0, 40,    5.000, 5.000 },
    { 0.0,  0.5,   5.0, 0.0,    NaN, 1.0, 40,    5.000, 5.000 },
    { 0.0,  1.0,   5.0, 0.0,    NaN, 1.0, 40,    5.000, 5.000 },
    { 0.0, -1.0,   5.0, 0.0,    NaN, 1.0, 40,    5.000, 5.000 },
    { 10.0, 1.0,   5.0, 0.0,    NaN, 1.0, 40,    5.000, 5.000 },
    { 10.0,-1.0,   5.0, 0.0,    NaN, 1.0, 40,    5.000, 5.000 },

    { 0.0,  0.0,   5.0, 0.5,    NaN, 1.0, 40,    10.000, 10.000 },
    { 0.0,  0.0,   5.0, 0.0,    NaN, 2.0, 40,    2.500, 2.500 },
    { 4.0,  0.0,   5.0, 0.0,    NaN, 1.0, 40,    1.000, 1.000 },

    /////////////////////////////////
    // No velocity limit.
    { 0.0,  0.0,    5.0, 0.0,   1.0, NaN, 40,   0.000, 4.514 },
    { 0.0,  1.0,    5.0, 0.0,   1.0, NaN, 40,   0.000, 3.730 },
    { 0.0,  1.0,    5.0, 1.5,   1.0, NaN, 40,   0.000, 5.025 },
    { 0.0, -1.0,   -5.0,-1.5,   1.0, NaN, 40,   0.000, 5.025 },
    { 5.0,  0.0,    0.0, 0.0,   1.0, NaN, 40,   0.000, 4.514 },
    { 5.0,  0.0,    0.0, 0.0,   2.0, NaN, 40,   0.000, 3.1994 },

    /////////////////////////////////
    // Accel and velocity limits
    { 0.0,  0.0,    3.0, 0.0,   1.0, 0.5, 40,   5.502, 6.500 },
    { 0.0,  0.0,    3.0, 0.0,   1.0, 0.7, 40,   3.588, 5.0048 },
    { 0.0,  0.0,    3.0, 0.0,   2.0, 0.7, 40,   3.937, 4.6358 },
    { 0.0,  0.3,    3.0, 0.0,   2.0, 0.7, 40,   3.969, 4.522 },
    { 0.3,  0.3,    3.0, 0.0,   2.0, 0.7, 40,   3.540, 4.0894 },
    // overspeed
    { 0.3,  2.0,    3.0, 0.0,   2.0, 0.7, 40,   2.429, 3.4281 },
    { -0.3, -2.0,  -3.0, 0.0,   2.0, 0.7, 40,   2.429, 3.4282 },
    // overshoot
    { 0.3,  4.0,    3.0, 0.0,   2.0, 0.7, 40,   1.504, 4.2016 },

    // non-zero final velocity
    { 0.0, 0.0,     3.0, 0.5,   1.0, 0.8, 40,  10.119, 11.217 },
    { 0.0, 0.0,     3.0, 0.3,   1.0, 0.8, 40,   5.592, 6.912 },

    // A command velocity that exceeds the limit.  Note, this will
    // never complete as it is not possible to catch up.
    { 0.0, 0.0,     3.0, 1.0,   1.0, 0.5, 40,   21.501, NaN },
    { 0.0, 0.0,     -3.0, -1.0, 1.0, 0.5, 40,   21.501, NaN },

    // non-zero targets
    { 0.0, 0.0,     0.0, 0.5,   1.0, 0.6, 40,   1.152, 1.850 },
    {-0.03, 0.5,    0.0, 0.3,   1.0, 0.6, 40,   0.000975, 0.2474 },
    // The same as the previous, but shifted to be near the wraparound
    // point and at a lower PWM rate to maximize numerical problems.
    {3275.97, 0.5,    3276.0, 0.3,   1.0, 0.6, 40,   0.000975, 0.2474 },
    {32765.97, 0.5,  32766.0, 0.3,   1.0, 0.6, 40,   0.000975, 0.2441 },
    {3275.97, 0.5,    3276.0, 0.3,   1.0, 0.6, 15,   0.000933, 0.2474 },
    {32765.97, 0.5,  32766.0, 0.3,   1.0, 0.6, 15,   0.000933, 0.2441 },

    // // Actually wrap around.
    {32767.98, 0.5,  -32767.99, 0.3,  1.0, 0.6, 15,   0.000933, 0.2441 },

    { 0.0, 0.0,     0.0, -0.5,  1.0, 0.6, 40,   1.152, 1.850 },

  };

  int case_num = 0;

  for (const auto& test_case : test_cases) {
    const double expected_vf =
        [&]() {
          if (test_case.vf > test_case.v) { return test_case.v; }
          if (test_case.vf < -test_case.v) { return -test_case.v; }
          return test_case.vf;
        }();

    case_num++;

    std::ofstream out_file;
    if (write_logs) {
      out_file.open(
          fmt::format("/tmp/moteus_test_{}.log", case_num));
    }

    BOOST_TEST_CONTEXT("Case " << case_num << " : "
                       << test_case.x0 << " "
                       << test_case.v0 << " "
                       << test_case.xf << " "
                       << test_case.vf << " "
                       << test_case.a << " "
                       << test_case.v) {
      Context ctx;
      ctx.rate_hz = test_case.rate_khz * 1000.0;
      ctx.data.position = test_case.xf;
      ctx.data.velocity = test_case.vf;
      ctx.data.accel_limit = test_case.a;
      ctx.data.velocity_limit = test_case.v;
      ctx.set_position(test_case.x0);
      ctx.set_velocity(test_case.v0);

      double old_vel = test_case.v0;
      double old_pos = test_case.x0;

      int done_count = 0;
      int consecutive_accel_violation = 0;

      double current_duration = 0.0;
      double total_duration = 0.0;
      double coast_duration = 0.0;
      bool initial_overspeed = std::isfinite(test_case.v) ?
          (std::abs(test_case.v0) > test_case.v) :
          false;

      const double extra_time = 1.0;
      const int64_t extra_count = extra_time * ctx.rate_hz;

      const int64_t max_count =
          (2.0 + (std::isnan(test_case.expected_total_duration) ?
                  20.0 : test_case.expected_total_duration)) * ctx.rate_hz;

      for (int64_t i = 0; i < max_count; i++) {
        ctx.Call();

        current_duration += (1.0 / ctx.rate_hz);

        const double this_pos =
            ctx.from_raw(ctx.status.control_position_raw.value());
        const double measured_vel =
            (ctx.from_raw(ctx.to_raw(this_pos) -
                          ctx.to_raw(old_pos))) * ctx.rate_hz;

        const double this_vel =
            ctx.status.control_velocity.value();
        const double measured_accel =
            (this_vel - old_vel) * ctx.rate_hz;

        if (write_logs) {
          out_file <<
              fmt::format(
                  "{},{:.9f},{},{},{}\n",
                  i / ctx.rate_hz, this_pos, measured_vel, measured_accel,
                  ctx.status.trajectory_done ? "1" : "0");
        }

        if (std::isfinite(ctx.data.velocity_limit)) {
          if (!initial_overspeed) {
            BOOST_TEST(std::abs(this_vel) <=
                       (ctx.data.velocity_limit + 0.001));
            if (std::abs(std::abs(this_vel) -
                         ctx.data.velocity_limit) < 0.001) {
              coast_duration += (1.0 / ctx.rate_hz);
            }
          } else {
            if (std::abs(this_vel) < (ctx.data.velocity_limit + 0.001)) {
              initial_overspeed = false;
            }
          }
        }

        if (i != 0) {
          BOOST_TEST(std::abs(this_vel - measured_vel) < 0.02);
        }

        if (std::isfinite(ctx.data.accel_limit)) {
          // No single reading can be more than 2.5x our limit, and no
          // two consecutive can be more than a tiny amount over.
          BOOST_TEST(std::abs(measured_accel) <= (2.5 * ctx.data.accel_limit));
          if (std::abs(measured_accel) > (1.02 * ctx.data.accel_limit)) {
            consecutive_accel_violation++;
            BOOST_TEST(consecutive_accel_violation <= 2);
          } else {
            consecutive_accel_violation = 0;
          }
        }

        if (ctx.status.trajectory_done) {
          BOOST_TEST(ctx.status.control_velocity.value() == expected_vf);

          if (done_count == 0) {
            if (std::isfinite(test_case.xf)) {
              BOOST_TEST(ctx.from_raw(
                             ctx.status.control_position_raw.value()) ==
                         test_case.xf +
                         test_case.vf * test_case.expected_total_duration);
            }
            total_duration = current_duration;
          }
          if (++done_count > extra_count) { break; }
        }

        old_vel = this_vel;
        old_pos = this_pos;
      }

      if (std::isfinite(test_case.xf) &&
          std::isfinite(test_case.expected_total_duration)) {
        const double expected_final =
            test_case.xf +
            test_case.expected_total_duration * test_case.vf +
            expected_vf * extra_time;
        BOOST_TEST(ctx.from_raw(
                       ctx.status.control_position_raw.value()) == expected_final);
      }
      BOOST_TEST(ctx.status.control_velocity.value() == expected_vf);
      BOOST_TEST(ctx.status.trajectory_done ==
                 std::isfinite(test_case.expected_total_duration));

      if (std::isfinite(test_case.expected_total_duration)) {
        BOOST_TEST(total_duration == test_case.expected_total_duration);
      } else {
        BOOST_TEST(total_duration == 0.0);
      }
      BOOST_TEST(coast_duration == test_case.expected_coast_duration);
    }
  }
}

BOOST_AUTO_TEST_CASE(StopPositionWithLimits, * boost::unit_test::tolerance(1e-3)) {
  Context ctx;

  ctx.data.position = 3.0f;
  ctx.set_stop_position(1.0f);
  ctx.data.velocity = 1.0f;
  ctx.data.accel_limit = 2.0f;
  ctx.data.velocity_limit = 3.0f;
  ctx.set_position(0.0f);

  for (int i = 0; i < 3.0 * ctx.rate_hz; i++) {
    ctx.Call();
  }

  BOOST_TEST(ctx.from_raw(ctx.status.control_position_raw.value()) == 1.0);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0);
  BOOST_TEST(ctx.status.trajectory_done == true);
}

BOOST_AUTO_TEST_CASE(StopPositionWithLimitOvershoot, * boost::unit_test::tolerance(1e-3)) {
  Context ctx;

  ctx.data.position = 0.0f;
  ctx.set_stop_position(0.2f);
  ctx.data.velocity = 1.0f;
  ctx.data.accel_limit = 2.0f;
  ctx.data.velocity_limit = 3.0f;
  ctx.set_velocity(2.0f);
  ctx.set_position(0.0f);

  // Here, we'll get stopped at 0.2 as try to slow down and come back
  // to 0.0.

  for (int i = 0; i < 3.0 * ctx.rate_hz; i++) {
    ctx.Call();
  }

  BOOST_TEST(ctx.from_raw(ctx.status.control_position_raw.value()) == 0.2);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0);
  BOOST_TEST(ctx.status.trajectory_done == true);
}

BOOST_AUTO_TEST_CASE(ControlAccelerationConsistent) {
  Context ctx;

  constexpr float kAccel = 5000.0f;

  ctx.data.position = 0.25f;
  ctx.data.accel_limit = kAccel;
  ctx.data.velocity_limit = NaN;
  ctx.rate_hz = 30000.0f;

  ctx.set_velocity(0.0f);
  ctx.set_position(0.0f);

  int steps_to_complete = 0;

  enum ExpectedAccel {
    kStrictlyPositive,
    kStrictlyNegative,
    kZero,
  };
  ExpectedAccel expected_accel = kStrictlyPositive;

  // The control acceleration should be strictly positive, then
  // strictly negative, and then 0, with no other transitions.

  int negative_violations = 0;
  int zero_violations = 0;

  constexpr int kMaxSteps = 50000;
  for (; steps_to_complete < kMaxSteps; steps_to_complete++) {
    BOOST_TEST_CONTEXT("Step: " << steps_to_complete) {
      ctx.Call();
      if (ctx.status.trajectory_done) { break; }

      const float this_accel = ctx.status.control_acceleration;
      switch (expected_accel) {
        case kStrictlyPositive: {
          if (this_accel > 0.0f) {
            // As expected, break.
            BOOST_TEST(this_accel == kAccel);
            break;
          } else if (this_accel < 0.0f) {
            // Advance.
            expected_accel = kStrictlyNegative;
            break;
          } else {
            // Not expected.
            BOOST_TEST(expected_accel != 0.0f);
          }
          break;
        }
        case kStrictlyNegative: {
          if (this_accel < 0.0f) {
            // With adaptive deceleration, allow 90-100% of kAccel
            BOOST_TEST(this_accel <= -0.9f * kAccel);
            BOOST_TEST(this_accel >= -kAccel - 1.0f);  // Allow tiny overshoot
            break;
          } else if (this_accel == 0.0f) {
            expected_accel = kZero;
            break;
          } else {
            negative_violations++;
          }
          break;
        }
        case kZero: {
          if (this_accel != 0.0f) {
            zero_violations++;
          }
          break;
        }
      }
    }
  }

  // With adaptive deceleration, trajectory may complete slightly earlier
  BOOST_TEST(steps_to_complete >= 420);
  BOOST_TEST(steps_to_complete <= 500);

  // Because of floating point precision issues in the current
  // implementation, we aren't perfect.  Still, verify that it
  // *mostly* does what we want.
  BOOST_TEST(negative_violations <= 15);
  BOOST_TEST(zero_violations <= 2);
}

// Test with debug trace output for analyzing trajectory behavior.
BOOST_AUTO_TEST_CASE(TrajectoryDebugTrace) {
  Context ctx;

  // Test parameters: 0.25 rev at 4000 rev/s², 30kHz control rate.
  constexpr float kDistance = 0.25f;
  constexpr float kAccel = 4000.0f;
  constexpr float kRateHz = 30000.0f;

  // Theoretical time for bang-bang trajectory: 2 * sqrt(dx / a)
  // = 2 * sqrt(0.25 / 4000) = 2 * 0.00790569 = 0.01581 s = 15.811 ms
  const float theoretical_time = 2.0f * std::sqrt(kDistance / kAccel);

  ctx.data.position = kDistance;
  ctx.data.accel_limit = kAccel;
  ctx.data.velocity_limit = NaN;
  ctx.rate_hz = kRateHz;

  ctx.set_velocity(0.0f);
  ctx.set_position(0.0f);

  int steps_to_complete = 0;
  constexpr int kMaxSteps = 50000;

  float prev_accel = 0.0f;
  int accel_sign_changes = 0;

  if (kTrajectoryDebug) {
    std::cout << "\n=== TrajectoryDebugTrace ===\n";
    std::cout << "Distance: " << kDistance << " rev, Accel: " << kAccel
              << " rev/s², Rate: " << kRateHz << " Hz\n";
    std::cout << "Theoretical time: " << (theoretical_time * 1000) << " ms\n";
    std::cout << "step,time_ms,pos,vel,accel,dx,stop_dist,done\n";
  }

  for (; steps_to_complete < kMaxSteps; steps_to_complete++) {
    ctx.Call();
    const float pos = ctx.from_raw(ctx.status.control_position_raw.value());
    const float vel = ctx.status.control_velocity.value();
    const float accel = ctx.status.control_acceleration;
    const float dx = kDistance - pos;
    const float stop_dist = (vel * vel) / (2.0f * kAccel);
    const float t = (steps_to_complete + 1) / kRateHz;
    const float time_ms = t * 1000.0f;

    // Track acceleration sign changes
    if (steps_to_complete > 0 && prev_accel != 0.0f && accel != 0.0f) {
      if ((prev_accel > 0) != (accel > 0)) {
        accel_sign_changes++;
        if (kTrajectoryDebug) {
          std::cout << ">>> SIGN CHANGE at step " << steps_to_complete << "\n";
        }
      }
    }

    if (kTrajectoryDebug) {
      // Print on sign change, near switch point, or near end
      const float switch_time = std::sqrt(kDistance / kAccel);
      bool near_switch = std::abs(t - switch_time) < 0.001f;
      bool near_end = steps_to_complete >= 460 || ctx.status.trajectory_done;
      bool is_transition = (prev_accel * accel < 0);

      if (is_transition || near_switch || near_end ||
          steps_to_complete < 5 || steps_to_complete % 50 == 0) {
        std::cout << steps_to_complete << "," << time_ms << ","
                  << pos << "," << vel << "," << accel << ","
                  << dx << "," << stop_dist << ","
                  << (ctx.status.trajectory_done ? "1" : "0") << "\n";
      }
    }

    prev_accel = accel;
    if (ctx.status.trajectory_done) { break; }
  }

  if (kTrajectoryDebug) {
    const float actual_time = steps_to_complete / kRateHz;
    std::cout << "\nCompleted in " << steps_to_complete << " steps ("
              << (actual_time * 1000) << " ms)\n";
    std::cout << "Overhead: " << ((actual_time - theoretical_time) * 1000) << " ms\n";
    std::cout << "Acceleration sign changes: " << accel_sign_changes << "\n";
    std::cout << "=== End TrajectoryDebugTrace ===\n\n";
  }

  BOOST_TEST(ctx.status.trajectory_done == true);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);

  const float actual_time = steps_to_complete / kRateHz;
  std::cout << "TrajectoryDebugTrace: steps=" << steps_to_complete
            << " actual=" << (actual_time * 1000) << "ms"
            << " theoretical=" << (theoretical_time * 1000) << "ms"
            << " accel_sign_changes=" << accel_sign_changes << std::endl;

  // The trajectory should complete within reasonable overhead
  BOOST_TEST(actual_time < theoretical_time + 0.010f);  // Within 10ms

  // Should have at most 1 sign change (the switch from accel to decel)
  BOOST_TEST(accel_sign_changes <= 1);
}

// Test short trajectories in the "gap region" where 4 ≤ u < 19.
// u = √(dx/a)/dt, so gap region is dx ∈ [16·a·dt², 361·a·dt²]
// At 30kHz, a=4000: dx ∈ [0.00007, 0.0016] rev = [0.025°, 0.58°]
// At 15kHz, a=4000: dx ∈ [0.00028, 0.0064] rev = [0.1°, 2.3°]
// These trajectories previously risked oscillation.
BOOST_AUTO_TEST_CASE(ShortTrajectoryGapRegion) {
  struct TestCase {
    float distance;     // in revolutions
    float target_vel;   // target velocity (rev/s)
    float accel;        // rev/s²
    float rate_khz;     // kHz
    const char* desc;
  };

  TestCase test_cases[] = {
    // At 30kHz, gap region tests (vf = 0)
    { 0.0001f, 0.0f, 4000.0f, 30.0f, "30kHz low gap (u~5)" },
    { 0.0005f, 0.0f, 4000.0f, 30.0f, "30kHz mid gap (u~11)" },
    { 0.001f,  0.0f, 4000.0f, 30.0f, "30kHz high gap (u~15)" },
    { 0.0015f, 0.0f, 4000.0f, 30.0f, "30kHz near gap edge (u~19)" },

    // At 15kHz, gap region tests (vf = 0)
    { 0.0005f, 0.0f, 4000.0f, 15.0f, "15kHz low gap (u~5)" },
    { 0.002f,  0.0f, 4000.0f, 15.0f, "15kHz mid gap (u~10)" },
    { 0.004f,  0.0f, 4000.0f, 15.0f, "15kHz high gap (u~14)" },
    { 0.006f,  0.0f, 4000.0f, 15.0f, "15kHz near gap edge (u~17)" },

    // Edge cases at different accel rates (vf = 0)
    { 0.0002f, 0.0f, 2000.0f, 30.0f, "lower accel 30kHz" },
    { 0.001f,  0.0f, 8000.0f, 30.0f, "higher accel 30kHz" },

    // Very short moves (below gap region, vf = 0)
    { 0.00002f, 0.0f, 4000.0f, 30.0f, "below gap 30kHz (u~2)" },
    { 0.00008f, 0.0f, 4000.0f, 15.0f, "below gap 15kHz (u~2)" },

    // Non-zero target velocity cases in gap region
    // These test that the closing/position_near checks work with vf != 0
    { 0.0005f, 0.5f, 4000.0f, 30.0f, "30kHz gap with vf=0.5" },
    { 0.001f,  0.3f, 4000.0f, 30.0f, "30kHz gap with vf=0.3" },
    { 0.002f,  0.4f, 4000.0f, 15.0f, "15kHz gap with vf=0.4" },
    { 0.0005f, -0.5f, 4000.0f, 30.0f, "30kHz gap with vf=-0.5" },
  };

  for (const auto& tc : test_cases) {
    BOOST_TEST_CONTEXT(tc.desc << " dx=" << tc.distance << " vf=" << tc.target_vel
                       << " a=" << tc.accel << " rate=" << tc.rate_khz) {
      Context ctx;
      const float rate_hz = tc.rate_khz * 1000.0f;
      const float dt = 1.0f / rate_hz;

      // Compute u for diagnostic purposes
      const float u = std::sqrt(tc.distance / tc.accel) / dt;

      ctx.data.position = tc.distance;
      ctx.data.velocity = tc.target_vel;
      ctx.data.accel_limit = tc.accel;
      ctx.data.velocity_limit = NaN;
      ctx.rate_hz = rate_hz;

      ctx.set_velocity(0.0f);
      ctx.set_position(0.0f);

      // Theoretical time for bang-bang trajectory (approximation for vf=0 case)
      const float theoretical_time = 2.0f * std::sqrt(tc.distance / tc.accel);

      int steps = 0;
      constexpr int kMaxSteps = 10000;
      int accel_sign_changes = 0;
      float prev_accel = 0.0f;
      int eot_violations = 0;

      for (; steps < kMaxSteps; steps++) {
        ctx.Call();

        const float pos = ctx.from_raw(ctx.status.control_position_raw.value());
        const float accel = ctx.status.control_acceleration;

        // Count accel sign changes (oscillation indicator)
        if (steps > 0 && prev_accel != 0.0f && accel != 0.0f) {
          if ((prev_accel > 0) != (accel > 0)) {
            accel_sign_changes++;
          }
        }

        // Check for overshoot/oscillation (accounting for target movement)
        const float expected_pos = tc.distance + tc.target_vel * (steps + 1) / rate_hz;
        if (pos > expected_pos + 0.0001f) {
          eot_violations++;
        }

        prev_accel = accel;
        if (ctx.status.trajectory_done) { break; }
      }

      const float actual_time = steps / rate_hz;
      const float overhead_ms = (actual_time - theoretical_time) * 1000.0f;

      if (kTrajectoryDebug) {
        std::cout << tc.desc << ": u=" << u
                  << " steps=" << steps
                  << " overhead=" << overhead_ms << "ms"
                  << " sign_changes=" << accel_sign_changes
                  << " eot_violations=" << eot_violations << "\n";
      }

      // Must complete
      BOOST_TEST(ctx.status.trajectory_done == true);
      // Velocity should reach target velocity (within tolerance for float comparison)
      BOOST_TEST(std::abs(ctx.status.control_velocity.value() - tc.target_vel) < 0.001f);

      // Should complete with reasonable overhead (< 5ms for short moves)
      BOOST_TEST(overhead_ms < 5.0f);

      // Should have at most 1 sign change (accel to decel)
      BOOST_TEST(accel_sign_changes <= 1);

      // No overshoot
      BOOST_TEST(eot_violations == 0);
    }
  }
}

// Test that position_near check behavior at different rates.
// The position_near threshold is: dx <= v * 10 * period
// With absolute velocity, this threshold scales with both velocity AND period.
// At 10kHz (period=100µs): threshold = 5 * 10 * 100µs = 5000 µrev
// At 40kHz (period=25µs): threshold = 5 * 10 * 25µs = 1250 µrev
//
// This test verifies behavior is reasonable across different configurations.
BOOST_AUTO_TEST_CASE(HighTargetVelocityPositionError) {
  struct TestCase {
    float target_vel;
    float vel_limit;
    float accel;
    float rate_khz;
    const char* desc;
  };

  TestCase cases[] = {
    { 5.0f, 10.0f, 100.0f, 40.0f, "40kHz, vf=5" },
    { 5.0f, 10.0f, 100.0f, 10.0f, "10kHz, vf=5" },
    { 2.0f, 5.0f, 50.0f, 40.0f, "40kHz, vf=2" },
    { 2.0f, 5.0f, 50.0f, 10.0f, "10kHz, vf=2" },
  };

  for (const auto& tc : cases) {
    BOOST_TEST_CONTEXT(tc.desc) {
      Context ctx;
      const float rate_hz = tc.rate_khz * 1000.0f;
      const float period = 1.0f / rate_hz;

      ctx.data.position = 0.0f;
      ctx.data.velocity = tc.target_vel;
      ctx.data.accel_limit = tc.accel;
      ctx.data.velocity_limit = tc.vel_limit;
      ctx.rate_hz = rate_hz;

      ctx.set_velocity(0.0f);
      ctx.set_position(0.0f);

      int steps = 0;
      constexpr int kMaxSteps = 500000;

      for (; steps < kMaxSteps; steps++) {
        ctx.Call();
        if (ctx.status.trajectory_done) { break; }
      }

      BOOST_TEST(ctx.status.trajectory_done == true);

      const float final_pos = ctx.from_raw(ctx.status.control_position_raw.value());
      const float final_time = steps * period;
      const float expected_target_pos = tc.target_vel * final_time;
      const float position_error = std::abs(final_pos - expected_target_pos);
      const float closing_rate = ctx.status.control_velocity.value() - tc.target_vel;

      // Compute theoretical thresholds
      const float abs_threshold = tc.target_vel * 10.0f * period;
      const float rel_threshold = (closing_rate != 0) ?
          std::abs(closing_rate) * 10.0f * period : 0.0f;

      if (kTrajectoryDebug) {
        std::cout << tc.desc << ":\n";
        std::cout << "  position_error=" << (position_error * 1e6) << " µrev\n";
        std::cout << "  abs_threshold=" << (abs_threshold * 1e6) << " µrev\n";
        std::cout << "  rel_threshold=" << (rel_threshold * 1e6) << " µrev\n";
        std::cout << "  closing_rate=" << closing_rate << "\n";
      }

      // The position error should be reasonably small.
      // We're checking that it's not hitting the absolute threshold limit.
      // At 10kHz with vf=5, abs_threshold = 5000 µrev - if error is near that,
      // the absolute velocity check is too permissive.
      BOOST_TEST(position_error < 500e-6f);  // 500 µrev = 0.18 degrees
    }
  }
}

// Test long trajectory behavior (50x normal distance).
BOOST_AUTO_TEST_CASE(LongTrajectoryBehavior) {
  Context ctx;

  // 50x the distance: 12.5 rev at 4000 rev/s²
  constexpr float kDistance = 12.5f;
  constexpr float kAccel = 4000.0f;
  constexpr float kRateHz = 30000.0f;

  // Theoretical time: 2 * sqrt(12.5 / 4000) = 111.8 ms
  const float theoretical_time = 2.0f * std::sqrt(kDistance / kAccel);

  ctx.data.position = kDistance;
  ctx.data.accel_limit = kAccel;
  ctx.data.velocity_limit = NaN;
  ctx.rate_hz = kRateHz;

  ctx.set_velocity(0.0f);
  ctx.set_position(0.0f);

  int steps_to_complete = 0;
  constexpr int kMaxSteps = 500000;
  int eot_violations = 0;
  float prev_accel = 0.0f;
  int accel_sign_changes = 0;

  if (kTrajectoryDebug) {
    std::cout << "\n=== LongTrajectoryBehavior ===\n";
    std::cout << "Distance: " << kDistance << " rev, Accel: " << kAccel
              << " rev/s², Rate: " << kRateHz << " Hz\n";
    std::cout << "Theoretical time: " << (theoretical_time * 1000) << " ms\n";
    std::cout << "step,time_ms,pos,vel,accel,dx,stop_dist,done\n";
  }

  for (int i = 0; i < kMaxSteps; i++) {
    ctx.Call();

    const float pos = ctx.from_raw(ctx.status.control_position_raw.value());
    const float vel = ctx.status.control_velocity.value();
    const float accel = ctx.status.control_acceleration;
    const float dx = kDistance - pos;

    // Count acceleration sign changes (oscillation indicator)
    if (i > 0 && prev_accel != 0.0f && accel != 0.0f) {
      if ((prev_accel > 0) != (accel > 0)) {
        accel_sign_changes++;
        if (kTrajectoryDebug) {
          const float time_ms = (i + 1) / kRateHz * 1000.0f;
          std::cout << ">>> SIGN CHANGE #" << accel_sign_changes
                    << " at step " << i << " (" << time_ms << " ms)"
                    << " pos=" << pos << " vel=" << vel << " dx=" << dx << "\n";
        }
      }
    }

    // Check for end-of-trajectory violations
    if (std::abs(dx) < 0.001f && !ctx.status.trajectory_done) {
      if ((dx > 0 && vel < -0.01f) || (dx < 0 && vel > 0.01f)) {
        eot_violations++;
        if (kTrajectoryDebug && eot_violations <= 5) {
          std::cout << ">>> EOT VIOLATION at step " << i
                    << " dx=" << dx << " vel=" << vel << "\n";
        }
      }
    }

    if (kTrajectoryDebug) {
      // Print near the end
      bool near_end = i >= 3300 || ctx.status.trajectory_done;
      if (near_end || i % 500 == 0) {
        const float time_ms = (i + 1) / kRateHz * 1000.0f;
        const float stop_dist = (vel * vel) / (2.0f * kAccel);
        std::cout << i << "," << time_ms << ","
                  << pos << "," << vel << "," << accel << ","
                  << dx << "," << stop_dist << ","
                  << (ctx.status.trajectory_done ? "1" : "0") << "\n";
      }
    }

    prev_accel = accel;
    if (ctx.status.trajectory_done) {
      steps_to_complete = i + 1;
      break;
    }
  }

  BOOST_TEST(ctx.status.trajectory_done == true);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);

  const float actual_time = steps_to_complete / kRateHz;
  const float overhead = actual_time - theoretical_time;

  std::cout << "LongTrajectoryBehavior: steps=" << steps_to_complete
            << " actual=" << (actual_time * 1000) << "ms"
            << " theoretical=" << (theoretical_time * 1000) << "ms"
            << " overhead=" << (overhead * 1000) << "ms"
            << " accel_sign_changes=" << accel_sign_changes
            << " eot_violations=" << eot_violations << std::endl;

  if (kTrajectoryDebug) {
    std::cout << "=== End LongTrajectoryBehavior ===\n\n";
  }

  // Allow reasonable overhead
  BOOST_TEST(overhead < 0.050f);  // Within 50ms

  // Should have at most 1 sign change
  BOOST_TEST(accel_sign_changes <= 1);

  // No end-of-trajectory violations
  BOOST_TEST(eot_violations == 0);
}
