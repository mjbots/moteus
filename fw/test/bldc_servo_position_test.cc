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
  float period_s = 1.0f / 40000.0f;
  BldcServoCommandData data;

  void set_rate_hz(float hz) {
    rate_hz = hz;
    period_s = 1.0f / hz;
  }

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
        period_s,
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
      ctx.set_rate_hz(test_case.rate_khz * 1000.0f);
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
  ctx.set_rate_hz(30000.0f);

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
            // Allow 90-100% of kAccel during deceleration
            BOOST_TEST(this_accel <= -0.9f * kAccel);
            BOOST_TEST(this_accel >= -kAccel);
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

  BOOST_TEST(steps_to_complete >= 420);
  BOOST_TEST(steps_to_complete <= 440);

  // There should be no oscillations.
  BOOST_TEST(negative_violations <= 0);
  BOOST_TEST(zero_violations <= 0);
}

// Test with debug trace output for analyzing trajectory behavior.
BOOST_AUTO_TEST_CASE(TrajectoryDebugTrace) {
  Context ctx;

  // Test parameters: 0.25 rev at 4000 rev/s², 30kHz control rate.
  constexpr float kDistance = 0.25f;
  constexpr float kAccel = 4000.0f;
  constexpr float kRateHz = 30000.0f;

  // Theoretical time for bang-bang trajectory.
  const float theoretical_time = 2.0f * std::sqrt(kDistance / kAccel);

  ctx.data.position = kDistance;
  ctx.data.accel_limit = kAccel;
  ctx.data.velocity_limit = NaN;
  ctx.set_rate_hz(kRateHz);

  ctx.set_velocity(0.0f);
  ctx.set_position(0.0f);

  int steps_to_complete = 0;
  constexpr int kMaxSteps = 50000;

  float prev_accel = 0.0f;
  int accel_sign_changes = 0;

  if (kTrajectoryDebug) {
    fmt::print("\n=== TrajectoryDebugTrace ===\n");
    fmt::print("Distance: {} rev, Accel: {} rev/s^2, Rate: {} Hz\n",
               kDistance, kAccel, kRateHz);
    fmt::print("Theoretical time: {} ms\n", theoretical_time * 1000);
    fmt::print("step,time_ms,pos,vel,accel,dx,stop_dist,done\n");
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
          fmt::print(">>> SIGN CHANGE at step {}\n", steps_to_complete);
        }
      }
    }

    if (kTrajectoryDebug) {
      // Print on sign change, near switch point, or near end
      const float switch_time = std::sqrt(kDistance / kAccel);
      const bool near_switch = std::abs(t - switch_time) < 0.001f;
      const bool near_end = steps_to_complete >= 460 || ctx.status.trajectory_done;
      const bool is_transition = (prev_accel * accel < 0);

      if (is_transition || near_switch || near_end ||
          steps_to_complete < 5 || steps_to_complete % 50 == 0) {
        fmt::print("{},{},{},{},{},{},{},{}\n",
                   steps_to_complete, time_ms, pos, vel, accel,
                   dx, stop_dist, ctx.status.trajectory_done ? "1" : "0");
      }
    }

    prev_accel = accel;
    if (ctx.status.trajectory_done) { break; }
  }

  if (kTrajectoryDebug) {
    const float actual_time = steps_to_complete / kRateHz;
    fmt::print("\nCompleted in {} steps ({} ms)\n",
               steps_to_complete, actual_time * 1000);
    fmt::print("Overhead: {} ms\n", (actual_time - theoretical_time) * 1000);
    fmt::print("Acceleration sign changes: {}\n", accel_sign_changes);
    fmt::print("=== End TrajectoryDebugTrace ===\n\n");
  }

  BOOST_TEST(ctx.status.trajectory_done == true);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);

  const float actual_time = steps_to_complete / kRateHz;
  fmt::print("TrajectoryDebugTrace: steps={} actual={}ms theoretical={}ms accel_sign_changes={}\n",
             steps_to_complete, actual_time * 1000, theoretical_time * 1000, accel_sign_changes);

  // The trajectory should complete within reasonable overhead
  BOOST_TEST(actual_time < theoretical_time + 0.010f);  // Within 10ms

  // Should have at most 1 sign change (the switch from accel to decel)
  BOOST_TEST(accel_sign_changes <= 1);
}

// Test short trajectories in the "settling region" where 4 <= u < 9.
// u = sqrt(dx/a)/dt, so settling region is dx ~= [16·a·dt^2, 80·a·dt^2]
// At 30kHz, a=4000: dx ~= [0.00007, 0.00036] rev = [0.026°, 0.13°]
// At 15kHz, a=4000: dx ~= [0.00028, 0.0014] rev = [0.1°, 0.51°]
// These short trajectories require special handling to avoid oscillation.
BOOST_AUTO_TEST_CASE(ShortTrajectorySettlingRegion) {
  struct TestCase {
    float distance;     // in revolutions
    float target_vel;   // target velocity (rev/s)
    float accel;        // rev/s^2
    float rate_khz;     // kHz
    const char* desc;
  };

  TestCase test_cases[] = {
    // At 30kHz, settling region tests (vf = 0)
    { 0.0001f,  0.0f, 4000.0f, 30.0f, "30kHz settling (u~5)" },
    { 0.00016f, 0.0f, 4000.0f, 30.0f, "30kHz settling (u~6)" },
    { 0.00022f, 0.0f, 4000.0f, 30.0f, "30kHz settling (u~7)" },
    { 0.00028f, 0.0f, 4000.0f, 30.0f, "30kHz settling edge (u~8)" },

    // At 15kHz, settling region tests (vf = 0)
    { 0.0005f,  0.0f, 4000.0f, 15.0f, "15kHz settling (u~5)" },
    { 0.00064f, 0.0f, 4000.0f, 15.0f, "15kHz settling (u~6)" },
    { 0.00087f, 0.0f, 4000.0f, 15.0f, "15kHz settling (u~7)" },
    { 0.0011f,  0.0f, 4000.0f, 15.0f, "15kHz settling edge (u~8)" },

    // Edge cases at different accel rates (vf = 0)
    { 0.0001f,  0.0f, 2000.0f, 30.0f, "lower accel 30kHz" },
    { 0.0004f,  0.0f, 8000.0f, 30.0f, "higher accel 30kHz" },

    // Very short moves (below settling region, vf = 0)
    { 0.00002f, 0.0f, 4000.0f, 30.0f, "below settling 30kHz (u~2)" },
    { 0.00008f, 0.0f, 4000.0f, 15.0f, "below settling 15kHz (u~2)" },

    // Non-zero target velocity cases in settling region
    // These test that the closing/position_near checks work with vf != 0
    { 0.00016f, 0.5f, 4000.0f, 30.0f, "30kHz settling with vf=0.5" },
    { 0.00022f, 0.3f, 4000.0f, 30.0f, "30kHz settling with vf=0.3" },
    { 0.00064f, 0.4f, 4000.0f, 15.0f, "15kHz settling with vf=0.4" },
    { 0.00016f, -0.5f, 4000.0f, 30.0f, "30kHz settling with vf=-0.5" },
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
      ctx.set_rate_hz(rate_hz);

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
        fmt::print("{}: u={} steps={} overhead={}ms sign_changes={} eot_violations={}\n",
                   tc.desc, u, steps, overhead_ms, accel_sign_changes, eot_violations);
      }

      // Must complete
      BOOST_TEST(ctx.status.trajectory_done == true);

      // Velocity should reach target velocity (within tolerance for
      // float comparison)
      BOOST_TEST(std::abs(ctx.status.control_velocity.value() - tc.target_vel) < 0.001f);

      // Should complete with reasonable overhead
      BOOST_TEST(overhead_ms < 0.15f);

      // But it should not complete *too* early.
      BOOST_TEST(overhead_ms > -0.30f);

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
      ctx.set_rate_hz(rate_hz);

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
        fmt::print("{}:\n", tc.desc);
        fmt::print("  position_error={} µrev\n", position_error * 1e6);
        fmt::print("  abs_threshold={} µrev\n", abs_threshold * 1e6);
        fmt::print("  rel_threshold={} µrev\n", rel_threshold * 1e6);
        fmt::print("  closing_rate={}\n", closing_rate);
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
  ctx.set_rate_hz(kRateHz);

  ctx.set_velocity(0.0f);
  ctx.set_position(0.0f);

  int steps_to_complete = 0;
  constexpr int kMaxSteps = 500000;
  int eot_violations = 0;
  float prev_accel = 0.0f;
  int accel_sign_changes = 0;

  if (kTrajectoryDebug) {
    fmt::print("\n=== LongTrajectoryBehavior ===\n");
    fmt::print("Distance: {} rev, Accel: {} rev/s², Rate: {} Hz\n",
               kDistance, kAccel, kRateHz);
    fmt::print("Theoretical time: {} ms\n", theoretical_time * 1000);
    fmt::print("step,time_ms,pos,vel,accel,dx,stop_dist,done\n");
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
          fmt::print(">>> SIGN CHANGE #{} at step {} ({} ms) pos={} vel={} dx={}\n",
                     accel_sign_changes, i, time_ms, pos, vel, dx);
        }
      }
    }

    // Check for end-of-trajectory violations
    if (std::abs(dx) < 0.001f && !ctx.status.trajectory_done) {
      if ((dx > 0 && vel < -0.01f) || (dx < 0 && vel > 0.01f)) {
        eot_violations++;
        if (kTrajectoryDebug && eot_violations <= 5) {
          fmt::print(">>> EOT VIOLATION at step {} dx={} vel={}\n", i, dx, vel);
        }
      }
    }

    if (kTrajectoryDebug) {
      // Print near the end
      bool near_end = i >= 3300 || ctx.status.trajectory_done;
      if (near_end || i % 500 == 0) {
        const float time_ms = (i + 1) / kRateHz * 1000.0f;
        const float stop_dist = (vel * vel) / (2.0f * kAccel);
        fmt::print("{},{},{},{},{},{},{},{}\n",
                   i, time_ms, pos, vel, accel, dx, stop_dist,
                   ctx.status.trajectory_done ? "1" : "0");
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

  fmt::print("LongTrajectoryBehavior: steps={} actual={}ms theoretical={}ms overhead={}ms accel_sign_changes={} eot_violations={}\n",
             steps_to_complete, actual_time * 1000, theoretical_time * 1000,
             overhead * 1000, accel_sign_changes, eot_violations);

  if (kTrajectoryDebug) {
    fmt::print("=== End LongTrajectoryBehavior ===\n\n");
  }

  // Allow reasonable overhead
  BOOST_TEST(overhead < 0.050f);  // Within 50ms

  // Should have at most 1 sign change
  BOOST_TEST(accel_sign_changes <= 1);

  // No end-of-trajectory violations
  BOOST_TEST(eot_violations == 0);
}

// Test that extending the target position while decelerating causes
// re-acceleration. This verifies that the trajectory logic properly
// handles target changes mid-trajectory.
BOOST_AUTO_TEST_CASE(ExtendTargetDuringDeceleration) {
  struct TestCase {
    float initial_target;
    float accel;
    float rate_khz;
    const char* desc;
  };

  TestCase test_cases[] = {
    { 0.25f, 4000.0f, 30.0f, "30kHz standard" },
    { 0.25f, 4000.0f, 15.0f, "15kHz standard" },
    { 0.10f, 2000.0f, 30.0f, "30kHz lower accel" },
    { 0.50f, 8000.0f, 30.0f, "30kHz higher accel" },
  };

  for (const auto& tc : test_cases) {
    BOOST_TEST_CONTEXT(tc.desc) {
      Context ctx;
      const float rate_hz = tc.rate_khz * 1000.0f;

      ctx.data.position = tc.initial_target;
      ctx.data.accel_limit = tc.accel;
      ctx.data.velocity_limit = NaN;
      ctx.set_rate_hz(rate_hz);

      ctx.set_velocity(0.0f);
      ctx.set_position(0.0f);

      // Run until we're decelerating
      int steps_to_decel = 0;
      float vel_at_decel = 0.0f;
      float pos_at_decel = 0.0f;
      float dx_at_decel = 0.0f;

      constexpr int kMaxSteps = 50000;
      for (int i = 0; i < kMaxSteps; i++) {
        ctx.Call();
        const float accel = ctx.status.control_acceleration;
        if (accel < 0.0f) {
          steps_to_decel = i;
          vel_at_decel = ctx.status.control_velocity.value();
          pos_at_decel = ctx.from_raw(ctx.status.control_position_raw.value());
          dx_at_decel = tc.initial_target - pos_at_decel;
          break;
        }
      }

      BOOST_TEST(steps_to_decel > 0);

      // Now test various target extensions to find the threshold
      // for re-acceleration. We'll binary search for the minimum
      // extension that triggers re-acceleration.

      // The stopping distance from current velocity is v^2 / (2*a)
      const float stopping_dist = (vel_at_decel * vel_at_decel) / (2.0f * tc.accel);

      // Current distance to target
      // dx_at_decel is already computed above

      // Test extensions as fractions of the stopping distance
      float min_extension_that_reaccels = NaN;
      float max_extension_that_decels = 0.0f;

      // Test a range of extensions
      for (float extension_frac = 0.01f; extension_frac <= 2.0f; extension_frac += 0.01f) {
        const float extension = extension_frac * stopping_dist;

        // Reset and run to deceleration point again
        ctx.status = BldcServoStatus{};
        ctx.status.motor_max_velocity = 100.0f;
        ctx.data.position = tc.initial_target;
        ctx.set_velocity(0.0f);
        ctx.set_position(0.0f);

        // Run to deceleration
        for (int i = 0; i < steps_to_decel + 1; i++) {
          ctx.Call();
        }

        // Skip this extension if we're not decelerating
        // (can happen due to timing variations)
        if (ctx.status.control_acceleration >= 0.0f) {
          continue;
        }

        // Now extend the target
        ctx.data.position = tc.initial_target + extension;
        ctx.data.position_relative_raw.reset();  // Force re-evaluation of new target

        // Take one more step and check if we re-accelerate
        ctx.Call();
        const float accel_after_extend = ctx.status.control_acceleration;

        if (accel_after_extend > 0.0f) {
          if (std::isnan(min_extension_that_reaccels)) {
            min_extension_that_reaccels = extension;
          }
        } else {
          max_extension_that_decels = extension;
        }
      }

      // Report the threshold
      const float threshold_as_fraction_of_stop_dist =
          min_extension_that_reaccels / stopping_dist;
      const float threshold_as_fraction_of_dx =
          min_extension_that_reaccels / dx_at_decel;

      fmt::print("{}: vel_at_decel={:.4f} dx_at_decel={:.6f} stop_dist={:.6f}\n",
                 tc.desc, vel_at_decel, dx_at_decel, stopping_dist);
      fmt::print("  min_extension_for_reaccel={:.6f} ({:.1f}% of stop_dist, {:.1f}% of dx)\n",
                 min_extension_that_reaccels,
                 threshold_as_fraction_of_stop_dist * 100.0f,
                 threshold_as_fraction_of_dx * 100.0f);
      fmt::print("  max_extension_still_decel={:.6f}\n", max_extension_that_decels);

      // Verify we found a threshold (i.e., large extensions do cause re-accel)
      BOOST_TEST(!std::isnan(min_extension_that_reaccels));

      // Verify the threshold is small - re-acceleration should trigger with
      // a modest extension, not require extending by the full stopping distance.
      BOOST_TEST(min_extension_that_reaccels < 0.20f * stopping_dist);

      // Now run a complete trajectory with a large extension to verify
      // the behavior end-to-end
      ctx.status = BldcServoStatus{};
      ctx.status.motor_max_velocity = 100.0f;
      ctx.data.position = tc.initial_target;
      ctx.set_velocity(0.0f);
      ctx.set_position(0.0f);

      int accel_sign_changes = 0;
      float prev_accel = 0.0f;
      bool extended = false;

      for (int i = 0; i < kMaxSteps; i++) {
        ctx.Call();
        const float accel = ctx.status.control_acceleration;

        // Track sign changes
        if (i > 0 && prev_accel != 0.0f && accel != 0.0f) {
          if ((prev_accel > 0) != (accel > 0)) {
            accel_sign_changes++;
          }
        }

        // Extend target when we start decelerating
        if (!extended && accel < 0.0f) {
          ctx.data.position = tc.initial_target * 2.0f;  // Double the target
          ctx.data.position_relative_raw.reset();  // Force re-evaluation
          extended = true;
        }

        prev_accel = accel;
        if (ctx.status.trajectory_done) { break; }
      }

      BOOST_TEST(ctx.status.trajectory_done == true);

      // With target extension, we should see:
      // 1. Initial accel (positive)
      // 2. Start decel (negative) - 1st sign change
      // 3. Re-accel after extension (positive) - 2nd sign change
      // 4. Final decel (negative) - 3rd sign change
      // So we expect 3 sign changes for a successful re-acceleration
      fmt::print("  end-to-end test: accel_sign_changes={}\n", accel_sign_changes);
      BOOST_TEST(accel_sign_changes == 3);
    }
  }
}

// Test the precise threshold for re-acceleration by measuring at what
// point extending the target switches from continued deceleration to
// re-acceleration. This helps characterize the deceleration behavior.
BOOST_AUTO_TEST_CASE(ReaccelerationThreshold) {
  Context ctx;

  constexpr float kAccel = 4000.0f;
  constexpr float kRateHz = 30000.0f;
  constexpr float kInitialTarget = 0.25f;

  ctx.data.position = kInitialTarget;
  ctx.data.accel_limit = kAccel;
  ctx.data.velocity_limit = NaN;
  ctx.set_rate_hz(kRateHz);

  ctx.set_velocity(0.0f);
  ctx.set_position(0.0f);

  // Run partway through trajectory, sampling at different points during
  // deceleration to see how the threshold changes
  fmt::print("\nReaccelerationThreshold analysis:\n");
  fmt::print("step,vel,min_ext_urev,min_ext_us,min_ext_pct\n");

  // First find when deceleration starts
  int decel_start_step = 0;
  for (int i = 0; i < 50000; i++) {
    ctx.Call();
    if (ctx.status.control_acceleration < 0.0f) {
      decel_start_step = i;
      break;
    }
  }

  // Track threshold statistics in three units:
  // - pct: percentage of stopping distance
  // - dist: absolute distance in micro-revolutions
  // - time: time to traverse at current velocity in microseconds
  struct Stats {
    float min = 1e9f;
    float max = 0.0f;
    float first = 0.0f;
    float last = 0.0f;
  };
  Stats pct_stats, dist_stats, time_stats;
  int num_samples = 0;

  // Now sample at various points during deceleration
  for (int sample_step = decel_start_step;
       sample_step < decel_start_step + 200;
       sample_step += 10) {

    // Reset and run to sample point
    ctx.status = BldcServoStatus{};
    ctx.status.motor_max_velocity = 100.0f;
    ctx.data.position = kInitialTarget;
    ctx.set_velocity(0.0f);
    ctx.set_position(0.0f);

    for (int i = 0; i <= sample_step; i++) {
      ctx.Call();
    }

    if (ctx.status.trajectory_done) break;

    const float vel = ctx.status.control_velocity.value();
    const float stop_dist = (vel * vel) / (2.0f * kAccel);

    // Binary search for minimum extension that causes re-acceleration
    float lo = 0.0f;
    float hi = stop_dist;

    for (int iter = 0; iter < 20; iter++) {
      const float mid = (lo + hi) / 2.0f;

      // Reset and run to sample point
      ctx.status = BldcServoStatus{};
      ctx.status.motor_max_velocity = 100.0f;
      ctx.data.position = kInitialTarget;
      ctx.set_velocity(0.0f);
      ctx.set_position(0.0f);

      for (int i = 0; i <= sample_step; i++) {
        ctx.Call();
      }

      // Extend target
      ctx.data.position = kInitialTarget + mid;
      ctx.data.position_relative_raw.reset();  // Force re-evaluation

      // Check next step
      ctx.Call();

      if (ctx.status.control_acceleration > 0.0f) {
        hi = mid;  // Re-accelerated, try smaller extension
      } else {
        lo = mid;  // Still decelerating, try larger extension
      }
    }

    const float min_ext = hi;
    const float min_ext_pct = (min_ext / stop_dist) * 100.0f;
    const float min_ext_urev = min_ext * 1e6f;  // micro-revolutions
    const float min_ext_us = (vel > 0.001f) ? (min_ext / vel) * 1e6f : 0.0f;  // microseconds

    // Track statistics
    auto update_stats = [&](Stats& s, float val) {
      if (num_samples == 0) s.first = val;
      s.last = val;
      s.min = std::min(s.min, val);
      s.max = std::max(s.max, val);
    };
    update_stats(pct_stats, min_ext_pct);
    update_stats(dist_stats, min_ext_urev);
    update_stats(time_stats, min_ext_us);
    num_samples++;

    fmt::print("{},{:.2f},{:.1f},{:.1f},{:.1f}\n",
               sample_step, vel, min_ext_urev, min_ext_us, min_ext_pct);
  }

  fmt::print("Summary (pct):  min={:.1f}% max={:.1f}% first={:.1f}% last={:.1f}%\n",
             pct_stats.min, pct_stats.max, pct_stats.first, pct_stats.last);
  fmt::print("Summary (urev): min={:.0f} max={:.0f} first={:.0f} last={:.0f}\n",
             dist_stats.min, dist_stats.max, dist_stats.first, dist_stats.last);
  fmt::print("Summary (us):   min={:.0f} max={:.0f} first={:.0f} last={:.0f}\n",
             time_stats.min, time_stats.max, time_stats.first, time_stats.last);

  // Verify we collected enough samples
  BOOST_TEST(num_samples >= 15);

  // === Percentage of stopping distance ===
  // At decel start, threshold should be small (near the ideal curve)
  BOOST_TEST(pct_stats.first < 5.0f);
  BOOST_TEST(pct_stats.min < 5.0f);
  // Max should be under 100% (never need full stopping distance)
  BOOST_TEST(pct_stats.max < 100.0f);
  // Threshold increases as we progress through deceleration
  BOOST_TEST(pct_stats.last > pct_stats.first);

  // === Absolute distance (micro-revolutions) ===
  // At decel start with high velocity, threshold is ~3900 urev
  BOOST_TEST(dist_stats.first > 3000.0f);
  BOOST_TEST(dist_stats.first < 5000.0f);
  // Min should be similar (occurs at decel start)
  BOOST_TEST(dist_stats.min > 3000.0f);
  BOOST_TEST(dist_stats.min < 5000.0f);
  // Max occurs mid-trajectory when velocity is still significant
  BOOST_TEST(dist_stats.max > 30000.0f);
  BOOST_TEST(dist_stats.max < 50000.0f);
  // At end, absolute distance decreases as velocity drops
  BOOST_TEST(dist_stats.last < 10000.0f);

  // === Absolute time (microseconds) ===
  // At decel start, time threshold is ~125 us (distance / high velocity)
  BOOST_TEST(time_stats.first > 100.0f);
  BOOST_TEST(time_stats.first < 150.0f);
  // Time threshold stays relatively stable through deceleration
  // because both distance and velocity decrease together
  BOOST_TEST(time_stats.min > 100.0f);
  BOOST_TEST(time_stats.max < 2000.0f);
  // At end, time threshold increases as we're further below the curve
  BOOST_TEST(time_stats.last > time_stats.first);
}
