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
#include <vector>

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
            (ctx.from_raw(MotorPosition::WrappingSub(
                ctx.to_raw(this_pos), ctx.to_raw(old_pos)))) * ctx.rate_hz;

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

// When the trapezoidal step crosses the velocity limit on a single
// cycle from a starting velocity of 0, the cruise-injection branch in
// DoVelocityAndAccelLimits must pick the cruise sign from the new
// velocity (the direction the trajectory is heading), not the old.
// Otherwise a move toward a negative target with v0 == +0.0 has the
// controller stamp control_velocity to +velocity_limit on the first
// cycle and enters a sustained ±limit oscillation that walks toward
// the target at a fraction of the requested rate.
BOOST_AUTO_TEST_CASE(VelocityLimitOverrideHonorsApproachDirection) {
  Context ctx;
  ctx.set_rate_hz(40000.0f);
  ctx.set_position(0.0f);
  ctx.status.control_position_raw = ctx.to_raw(0.0f);
  ctx.status.control_velocity = 0.0f;
  ctx.data.position = -10.0f;
  ctx.data.velocity = 0.0f;
  ctx.data.velocity_limit = 1.0f;
  ctx.data.accel_limit = 100000.0f;
  ctx.data.position_relative_raw.reset();

  ctx.Call();

  // The only legitimate trajectory direction here is negative.
  BOOST_TEST(ctx.status.control_velocity.value() <= 0.0f);
}

// === Jerk-limited trajectory tests ===
//
// These exercise the slew-rate-limited acceleration path that
// activates when `data.jerk_limit` is finite.

// Velocity-only mode: ramp v from 0 to vf with a jerk limit; verify
// that the ramp is smooth and v does not overshoot.
BOOST_AUTO_TEST_CASE(JerkLimitVelocityModeRamp) {
  Context ctx;
  constexpr float kRateHz = 30000.0f;
  constexpr float kAccel = 1000.0f;
  constexpr float kJerk = 20000.0f;
  constexpr float kVTarget = 1.5f;

  ctx.set_rate_hz(kRateHz);
  ctx.data.position = NaN;
  ctx.data.velocity = kVTarget;
  ctx.data.accel_limit = kAccel;
  ctx.data.jerk_limit = kJerk;
  ctx.data.velocity_limit = NaN;
  ctx.status.control_position_raw = ctx.to_raw(0.0f);
  ctx.status.control_velocity = 0.0f;

  const float dt = 1.0f / kRateHz;
  // Per-cycle delta-a bound for the jerk-limited slew, with unit of
  // least precision (ULP) slop.
  const float jerk_bound =
      kJerk * dt + 4.0f * kAccel *
      std::numeric_limits<float>::epsilon();
  // Theoretical max overshoot is bounded by a_peak * dt where
  // a_peak = sqrt(j*v_target) for the triangular-in-a profile.
  // Allow 2x safety for the discrete-cycle phase.
  const float overshoot_bound =
      2.0f * std::sqrt(kJerk * kVTarget) * dt;
  float prev_a = 0.0f;
  double max_v = 0.0;

  for (int i = 0; i < 100000; i++) {
    ctx.Call();
    const double v = ctx.status.control_velocity.value();
    if (v > max_v) { max_v = v; }
    if (ctx.status.trajectory_done) { break; }
    const float a = ctx.status.control_acceleration;
    BOOST_TEST(std::abs(a - prev_a) <= jerk_bound);
    prev_a = a;
  }

  BOOST_TEST(ctx.status.trajectory_done == true);
  // Should reach the commanded velocity exactly (the terminal slew
  // sets control_velocity = vf at trajectory_done).
  BOOST_TEST(ctx.status.control_velocity.value() == kVTarget);
  BOOST_TEST(max_v - kVTarget < overshoot_bound);
}

// Sanity-check the analytical stop-distance helper against forward
// integration of the same 3-phase profile.
BOOST_AUTO_TEST_CASE(JerkStopDistanceMatchesIntegration,
                     * boost::unit_test::tolerance(1e-6)) {
  struct Case {
    float v;
    float a;
    float a_max;
    float j;
  };
  Case cases[] = {
    { 10.0f, 0.0f,    100.0f, 5000.0f },
    { 20.0f, 0.0f,    100.0f, 5000.0f },
    { 10.0f, 50.0f,   100.0f, 5000.0f },
    { 10.0f, -50.0f,  100.0f, 5000.0f },
    { 10.0f, 100.0f,  100.0f, 5000.0f },
    { 10.0f, -100.0f, 100.0f, 5000.0f },
    // Triangle regime: small v.
    { 0.5f,  0.0f,    100.0f, 5000.0f },
  };

  for (const auto& c : cases) {
    BOOST_TEST_CONTEXT("v=" << c.v << " a=" << c.a
                       << " amax=" << c.a_max << " j=" << c.j) {
      // Forward-integrate the same profile in tiny steps and verify
      // we stop within a tight tolerance of `stop_d`.  Each step uses
      // the constant-jerk closed-form integrals so the reference
      // matches the analytical trajectory to O(j*dt^3) per step
      // (instead of O(j*dt^2) for the previous semi-implicit Euler):
      //
      //   a_new = a_old + j_step * dt
      //   v_new = v_old + a_old*dt + j_step*dt^2/2
      //         = v_old + (a_old + a_new)/2 * dt        (trapezoid)
      //   x_new = x_old + v_old*dt + a_old*dt^2/2 + j_step*dt^3/6
      //
      // To avoid overshoot at phase boundaries -- where the last step
      // would otherwise carry a or v past the switching condition --
      // each step's duration is clamped to the time-to-boundary.
      double v = c.v;
      double a = c.a;
      double x = 0.0;
      const double dt = 1e-5;
      const float v3 = c.a_max * c.a_max / (2.0f * c.j);
      const float v_after_0 =
          c.v + (c.a * c.a - c.a_max * c.a_max) / (2.0f * c.j);
      const bool trapezoidal = v_after_0 >= v3;
      const float a_peak_sq =
          0.5f * (c.a * c.a + 2.0f * c.j * c.v);
      const float a_peak = std::sqrt(std::max(a_peak_sq, 0.0f));
      const float a_brake =
          trapezoidal ? c.a_max : a_peak;

      // Phase A: ramp a from c.a toward -a_brake with j_step = -c.j.
      // Stops exactly at -a_brake by clamping the final step.
      while (a > -a_brake) {
        const double a_old = a;
        const double j_step = -c.j;
        const double t = std::min(dt, (a + a_brake) / c.j);
        a += j_step * t;
        if (a < -a_brake) { a = -a_brake; }
        x += v * t + a_old * 0.5 * t * t + j_step * t * t * t / 6.0;
        v += (a_old + a) * 0.5 * t;
      }
      // Phase B (only trapezoidal): hold at -a_max until v == v3.
      // j_step = 0 here, so the cubic correction vanishes and the
      // trapezoid v-update reduces to v += a*t.  Final step lands
      // exactly at v3.
      if (trapezoidal) {
        while (v > v3) {
          const double t = std::min(dt, (v - v3) / a_brake);
          x += v * t + a * 0.5 * t * t;
          v += a * t;
          if (v < v3) { v = v3; }
        }
      }
      // Phase C: ramp a from -a_brake back to 0 with j_step = +c.j.
      // Stops exactly at a = 0 by clamping the final step.
      while (a < 0.0) {
        const double a_old = a;
        const double j_step = c.j;
        const double t = std::min(dt, -a / c.j);
        a += j_step * t;
        if (a > 0.0) { a = 0.0; }
        x += v * t + a_old * 0.5 * t * t + j_step * t * t * t / 6.0;
        v += (a_old + a) * 0.5 * t;
      }

      // Now compare to the online estimated version.

      const float stop_d = BldcServoPosition::ComputeJerkStopDistance(
          c.v, c.a, c.a_max, c.j, 1.0f / c.j);

      BOOST_TEST(static_cast<double>(stop_d) == x);
      // Final v should be near 0 (small absolute drift accumulated
      // by the forward integration over many tiny steps).
      BOOST_TEST(std::abs(v) < 1e-12);
    }
  }
}

// Compare jerk-limited vs unlimited: both should reach the target,
// the unlimited (constant-accel) version should finish first, and
// the jerk-limited version should have no acceleration sign
// discontinuity exceeding |jerk*dt| per cycle.
BOOST_AUTO_TEST_CASE(JerkLimitedTakesLongerThanTrapezoidal) {
  constexpr float kRateHz = 30000.0f;
  constexpr float kAccel = 1000.0f;
  constexpr float kJerk = 20000.0f;
  auto run = [&](float jerk_limit) {
    Context ctx;
    ctx.set_rate_hz(kRateHz);
    ctx.data.position = 1.0f;
    ctx.data.velocity = 0.0f;
    ctx.data.accel_limit = kAccel;
    ctx.data.jerk_limit = jerk_limit;
    ctx.data.velocity_limit = NaN;
    ctx.set_position(0.0f);
    ctx.set_velocity(0.0f);
    // For the jerk-limited run, verify the per-cycle |Δa| bound
    // throughout the trajectory.  The unlimited run is allowed
    // arbitrary Δa (that's the whole point of the jerk limit).
    const bool check_jerk_bound = std::isfinite(jerk_limit);
    const float dt = 1.0f / kRateHz;
    const float jerk_bound =
        check_jerk_bound
            ? jerk_limit * dt +
                  kAccel * std::numeric_limits<float>::epsilon()
            : 0.0f;
    float prev_a = 0.0f;
    int steps = 0;
    for (; steps < 200000; steps++) {
      ctx.Call();
      if (check_jerk_bound) {
        const float a = ctx.status.control_acceleration;
        BOOST_TEST(std::abs(a - prev_a) <= jerk_bound);
        prev_a = a;
      }
      if (ctx.status.trajectory_done) { return steps + 1; }
    }
    return -1;
  };

  const int steps_no_jerk = run(NaN);
  const int steps_jerk = run(kJerk);

  BOOST_TEST(steps_no_jerk > 0);
  BOOST_TEST(steps_jerk > 0);
  BOOST_TEST(steps_jerk > steps_no_jerk);
}

// A battery of termination cases that span hostile parameter
// regimes for the jerk-limited trajectory generator.  Each case must
// terminate (trajectory_done = true) within a generous time budget.
BOOST_AUTO_TEST_CASE(JerkLimitTerminationSweep) {
  struct TestCase {
    float xf;
    float vf;
    float accel;
    float jerk;
    float rate_hz;
    float vel_limit;
    float max_time_s;
    const char* desc;
  };

  TestCase cases[] = {
    // The user-reported low-jerk failure case.
    { 5.0f, 0.0f, 5.0f, 2.0f, 30000.0f, NaN, 15.0f, "low j (a=5,j=2)" },
    // Even lower jerk.
    { 1.0f, 0.0f, 2.0f, 0.5f, 30000.0f, NaN, 15.0f, "very low j" },
    // Very long trajectory: even more cycles of accumulated drift.
    { 50.0f, 0.0f, 10.0f, 5.0f, 30000.0f, NaN, 20.0f, "long trajectory" },
    // High-jerk / fast move (the canonical case).
    { 1.0f, 0.0f, 1000.0f, 20000.0f, 30000.0f, NaN, 2.0f, "fast move" },
    // Trajectory hitting the velocity limit (forces cruise phase).
    { 10.0f, 0.0f, 100.0f, 5000.0f, 30000.0f, 2.0f, 10.0f, "v-limit cruise" },
    // Non-zero final velocity (tracking a moving target).
    { 2.0f, 0.3f, 50.0f, 1000.0f, 30000.0f, NaN, 5.0f, "vf != 0" },
    // Backwards move.
    { -3.0f, 0.0f, 50.0f, 1000.0f, 30000.0f, NaN, 5.0f, "backwards" },
    // Low control rate.
    { 1.0f, 0.0f, 100.0f, 2000.0f, 15000.0f, NaN, 3.0f, "15 kHz rate" },
    // Tiny move that may settle in a few cycles.
    { 0.05f, 0.0f, 100.0f, 5000.0f, 30000.0f, NaN, 2.0f, "tiny move" },
    // Initial velocity in the same direction as target.
    { 5.0f, 0.0f, 50.0f, 1000.0f, 30000.0f, NaN, 5.0f, "co-aligned v0" },
  };

  for (const auto& tc : cases) {
    BOOST_TEST_CONTEXT(tc.desc) {
      Context ctx;
      ctx.set_rate_hz(tc.rate_hz);
      ctx.data.position = tc.xf;
      ctx.data.velocity = tc.vf;
      ctx.data.accel_limit = tc.accel;
      ctx.data.jerk_limit = tc.jerk;
      ctx.data.velocity_limit = tc.vel_limit;
      ctx.set_position(0.0f);
      ctx.set_velocity(0.0f);

      const int max_steps =
          static_cast<int>(tc.max_time_s * tc.rate_hz);
      // Track the worst per-cycle overshoot past the moving-target
      // reference `target_at_t = xf + vf*t`.  Without the rest-
      // curve pre-switch the trajectory can pass the target before
      // braking, which a regression-tester would notice as a
      // non-zero max_overshoot in the direction of motion.
      double max_overshoot = 0.0;
      int steps = 0;
      for (; steps < max_steps; steps++) {
        ctx.Call();
        const double pos =
            ctx.from_raw(ctx.status.control_position_raw.value());
        const double target_at_t =
            tc.xf + tc.vf * ((steps + 1) / static_cast<double>(tc.rate_hz));
        const double overshoot =
            (tc.xf >= 0.0f) ? (pos - target_at_t) : (target_at_t - pos);
        if (overshoot > max_overshoot) { max_overshoot = overshoot; }
        if (ctx.status.trajectory_done) { break; }
      }

      const double final_pos =
          ctx.from_raw(ctx.status.control_position_raw.value());
      BOOST_TEST(ctx.status.trajectory_done == true);
      BOOST_TEST(ctx.status.control_velocity.value() == tc.vf);
      const double expected =
          tc.xf + tc.vf * (steps / static_cast<double>(tc.rate_hz));
      const float kFloatEps = std::numeric_limits<float>::epsilon();
      const float kinematic_floor =
          tc.accel * tc.accel / (tc.jerk * tc.rate_hz);
      const float drift_floor =
          tc.accel / tc.jerk * tc.rate_hz * kFloatEps;
      const float bound = 1.0f * (kinematic_floor + drift_floor);
      BOOST_CHECK_LT(std::abs(final_pos - expected), bound);
      // The trajectory must not overshoot the moving target by
      // more than the kinematic residual either.
      BOOST_CHECK_LT(max_overshoot, bound);
    }
  }
}

// Verify the jerk bound is respected at the velocity-limit cruise
// transitions (entering and exiting the velocity-limited coast),
// including the cycle straddling trajectory_done.
BOOST_AUTO_TEST_CASE(JerkLimitVelocityLimitJerkBound) {
  Context ctx;
  const float rate_hz = 30000.0f;
  const float accel = 100.0f;
  const float jerk = 5000.0f;
  const float vel_limit = 2.0f;
  ctx.set_rate_hz(rate_hz);
  ctx.data.position = 10.0f;
  ctx.data.velocity = 0.0f;
  ctx.data.accel_limit = accel;
  ctx.data.jerk_limit = jerk;
  ctx.data.velocity_limit = vel_limit;
  ctx.set_position(0.0f);
  ctx.set_velocity(0.0f);

  const float dt = 1.0f / rate_hz;
  const float bound =
      jerk * dt + accel * std::numeric_limits<float>::epsilon();

  float prev_a = 0.0f;
  int max_violation_count = 0;
  float max_violation = 0.0f;
  bool done = false;
  // Iterate one cycle past trajectory_done so the termination
  // discontinuity is included in the bound check.
  for (int i = 0; i < 200000; i++) {
    ctx.Call();
    const float a = ctx.status.control_acceleration;
    const float da = std::abs(a - prev_a);
    if (da > bound) {
      max_violation_count++;
      if (da > max_violation) { max_violation = da; }
    }
    prev_a = a;
    if (done) { break; }
    if (ctx.status.trajectory_done) { done = true; }
  }
  BOOST_TEST(done == true);
  BOOST_TEST(max_violation_count == 0);
  BOOST_TEST(max_violation == 0.0f);
}

// === Mid-trajectory retargeting tests ===
//
// The jerk-limited trajectory generator latches into a terminal
// "slew acceleration to 0" phase once it reaches the rest-curve
// manifold for the current target.  If the host changes the target
// while the latch is held, the controller MUST drop the latch and
// re-plan -- otherwise it coasts to the old predicted endpoint and
// ignores the new command.
//
// These tests exercise:
//   - Retargeting during the terminal slew (the original bug).
//   - Retargeting in the opposite direction during the slew.
//   - Retargeting before the latch fires (sanity).
//   - Streaming the same target every cycle (latch must NOT clear).
//   - Streaming with small position increments (smooth tracking).
//   - Streaming with bit-equal floats from a coarse host resolution.
//   - Changing the velocity target during the slew.

namespace {

// Step a context for up to `max_steps` cycles or until `predicate`
// returns true.  Returns the number of cycles actually executed.
template <typename Pred>
int StepUntil(Context* ctx, int max_steps, Pred predicate) {
  for (int i = 0; i < max_steps; i++) {
    ctx->Call();
    if (predicate()) { return i + 1; }
  }
  return max_steps;
}

// Set up `ctx` for a fresh position move to `target`.  Mirrors the
// retargeting pattern used by JerkLimitRetargetingTerminates: clear
// position_relative_raw so Call() derives a fresh value from the new
// `data.position`.
void Retarget(Context* ctx, float new_position, float new_velocity) {
  ctx->data.position = new_position;
  ctx->data.velocity = new_velocity;
  ctx->data.position_relative_raw.reset();
}

float kRetargetRateHz = 30000.0f;
float kRetargetAccel = 50.0f;
float kRetargetJerk = 1000.0f;

void StartRetargetMove(Context* ctx, float position) {
  ctx->set_rate_hz(kRetargetRateHz);
  ctx->data.accel_limit = kRetargetAccel;
  ctx->data.jerk_limit = kRetargetJerk;
  ctx->data.velocity_limit = NaN;
  ctx->set_position(0.0f);
  ctx->set_velocity(0.0f);
  ctx->data.position = position;
  ctx->data.velocity = 0.0f;
}

float ResidualBound(float accel, float jerk, float rate_hz) {
  const float kFloatEps = std::numeric_limits<float>::epsilon();
  const float kinematic_floor = accel * accel / (jerk * rate_hz);
  const float drift_floor = accel / jerk * rate_hz * kFloatEps;
  // The unscaled (kinematic_floor + drift_floor) is the analytical
  // upper bound on the final-residual after a single trajectory
  // completes.  Empirically the worst case across all jerk tests is
  // ~30% of that bound (multi-retarget stress patterns), so a 1x
  // factor gives ~3x safety; tighter than that risks spurious
  // failures from minor controller refactors.
  return 1.0f * (kinematic_floor + drift_floor);
}

}

// (The mid-slew / reversal / early-retarget scenarios are all
// covered as rows of JerkLimitRetargetAtEachPhase below.)

// A host that re-sends the SAME literal `position` / `velocity`
// every cycle (e.g. a 1 kHz "hold here" supervisor talking to a
// 30 kHz ISR) must not interfere with the rest-commit latch.  The
// trajectory should complete in the same number of cycles as one
// where the host writes a single command and walks away.  Runs in
// both position and velocity mode -- the position mode case
// exercises the position_relative_raw re-derivation each cycle
// (data.position_relative_raw is reset before every Call), and the
// velocity mode case exercises the equivalent code path for a
// velocity-only target stream.
BOOST_AUTO_TEST_CASE(JerkLimitStreamingSameTargetCompletes) {
  struct TestCase {
    bool position_mode;
    const char* desc;
  };
  TestCase modes[] = {
    {  true, "position mode" },
    { false, "velocity mode" },
  };

  for (const auto& mode : modes) {
    BOOST_TEST_CONTEXT(mode.desc) {
      auto run = [&](bool stream_every_cycle) {
        Context ctx;
        if (mode.position_mode) {
          StartRetargetMove(&ctx, 5.0f);
        } else {
          ctx.set_rate_hz(30000.0f);
          ctx.data.position = NaN;
          ctx.data.velocity = 1.5f;
          ctx.data.accel_limit = 1000.0f;
          ctx.data.jerk_limit = 20000.0f;
          ctx.data.velocity_limit = NaN;
          ctx.status.control_position_raw = ctx.to_raw(0.0f);
          ctx.status.control_velocity = 0.0f;
        }

        const float rate_hz = mode.position_mode ? kRetargetRateHz : 30000.0f;
        const int max_steps = static_cast<int>(5.0f * rate_hz);
        int steps = 0;
        for (; steps < max_steps; steps++) {
          if (stream_every_cycle) {
            if (mode.position_mode) {
              // Re-issue the same target.  Reset position_relative_raw
              // so Call() derives a fresh value from data.position;
              // this is what PrepareCommand does on every real
              // command frame.
              ctx.data.position = 5.0f;
              ctx.data.velocity = 0.0f;
              ctx.data.position_relative_raw.reset();
            } else {
              ctx.data.velocity = 1.5f;
            }
          }
          ctx.Call();
          if (ctx.status.trajectory_done) { break; }
        }
        return std::make_tuple(steps + 1,
                               ctx.from_raw(
                                   ctx.status.control_position_raw.value()),
                               ctx.status.control_velocity.value(),
                               ctx.status.trajectory_done);
      };

      auto [single_steps, single_pos, single_v, single_done] = run(false);
      auto [stream_steps, stream_pos, stream_v, stream_done] = run(true);

      BOOST_TEST(single_done == true);
      BOOST_TEST(stream_done == true);

      if (mode.position_mode) {
        const float bound =
            ResidualBound(kRetargetAccel, kRetargetJerk, kRetargetRateHz);
        BOOST_CHECK_LT(std::abs(single_pos - 5.0), bound);
        BOOST_CHECK_LT(std::abs(stream_pos - 5.0), bound);
      } else {
        // Termination forces control_velocity = vf exactly.
        BOOST_TEST(single_v == 1.5f);
        BOOST_TEST(stream_v == 1.5f);
      }

      // Streaming the same target must not lengthen the trajectory
      // at all (empirically exact match) -- if a single command-
      // frame re-entry slipped past the latch-tolerance, this would
      // be off by a measurable number of cycles.
      BOOST_TEST(stream_steps == single_steps);
    }
  }
}

// Two distinct streaming patterns that exercise the latch-
// invalidation kinematic tolerance:
//
//   - Noisy: a host re-issues a target with small ±1 µ rev
//     jitter (host-clock or encoder-quantisation noise).  Each
//     step the latch invalidation is consulted; the kinematic
//     tolerance must absorb the sub-LSB noise rather than
//     thrash.  Verifies the trajectory still terminates and the
//     residual is bounded by the kinematic floor + 2× noise
//     peak-to-peak.
//
//   - Advancing: a host streams a monotonically growing target
//     (tracking a moving setpoint) until reaching a hold-at
//     threshold.  Verifies the trajectory follows the advancing
//     target and settles on the latched value.
BOOST_AUTO_TEST_CASE(JerkLimitStreamingPatterns) {
  enum Pattern { kNoisy, kAdvancing };
  struct TestCase {
    Pattern pattern;
    const char* desc;
  };
  TestCase cases[] = {
    { kNoisy,     "noisy: target = 5.0 +/- 1e-6 every cycle" },
    { kAdvancing, "advancing: target ramps to 2.0 at 0.5 rev/s" },
  };

  for (const auto& tc : cases) {
    BOOST_TEST_CONTEXT(tc.desc) {
      Context ctx;
      const float dt = 1.0f / kRetargetRateHz;
      const float advancing_target_v = 0.5f;
      const float advancing_stop_at = 2.0f;
      const float noisy_base_target = 5.0f;
      const float noise_amplitude = 1e-6f;
      StartRetargetMove(&ctx,
                        tc.pattern == kAdvancing ? 0.0f : noisy_base_target);

      uint32_t rng = 0x1234abcd;
      auto next_noise = [&]() {
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return (static_cast<int32_t>(rng) /
                static_cast<float>(std::numeric_limits<int32_t>::max())) *
            noise_amplitude;
      };

      float last_target = 0.0f;
      float advancing_target = 0.0f;
      const int max_steps = static_cast<int>(8.0f * kRetargetRateHz);
      for (int i = 0; i < max_steps; i++) {
        if (tc.pattern == kNoisy) {
          last_target = noisy_base_target + next_noise();
        } else {  // kAdvancing
          if (advancing_target < advancing_stop_at) {
            advancing_target += advancing_target_v * dt;
            if (advancing_target > advancing_stop_at) {
              advancing_target = advancing_stop_at;
            }
          }
          last_target = advancing_target;
        }
        Retarget(&ctx, last_target, 0.0f);
        ctx.Call();
        if (ctx.status.trajectory_done &&
            (tc.pattern != kAdvancing ||
             advancing_target >= advancing_stop_at)) {
          break;
        }
      }

      BOOST_TEST(ctx.status.trajectory_done == true);
      BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
      const double final_pos =
          ctx.from_raw(ctx.status.control_position_raw.value());
      const double expected =
          tc.pattern == kAdvancing ? advancing_stop_at : last_target;
      // The advancing path settles exactly on its held target;
      // the noisy path settles within the kinematic floor +
      // peak-to-peak noise.
      const float bound =
          ResidualBound(kRetargetAccel, kRetargetJerk, kRetargetRateHz) +
          (tc.pattern == kNoisy ? 2.0f * noise_amplitude : 0.0f);
      BOOST_CHECK_LT(std::abs(final_pos - expected), bound);
    }
  }
}

// Changing the velocity target during the terminal slew must
// invalidate the latch and let the controller re-plan to the new
// (position, velocity) endpoint.  Without invalidation the
// controller would freeze velocity at 0 even though the host now
// wants a non-zero tracking velocity.
BOOST_AUTO_TEST_CASE(JerkLimitMidSlewVelocityChangeReplan) {
  Context ctx;
  StartRetargetMove(&ctx, 5.0f);

  const int max_pre_steps = static_cast<int>(2.0f * kRetargetRateHz);
  StepUntil(&ctx, max_pre_steps,
            [&]() { return ctx.status.trajectory_rest_committed; });
  BOOST_TEST(ctx.status.trajectory_rest_committed == true);

  // Change the velocity target.  The position target stays at 5.0
  // but we now want to be moving at 0.3 rev/s when we arrive.
  ctx.data.velocity = 0.3f;
  // Don't reset position_relative_raw: only the velocity changed in
  // the host's intent.  The latch must still invalidate because
  // data.velocity differs from the snapshot.
  ctx.Call();
  BOOST_TEST(ctx.status.trajectory_rest_committed == false);

  const int max_post_steps = static_cast<int>(5.0f * kRetargetRateHz);
  StepUntil(&ctx, max_post_steps,
            [&]() { return ctx.status.trajectory_done; });
  BOOST_TEST(ctx.status.trajectory_done == true);
  // Final velocity should match the new target velocity exactly
  // (termination forces control_velocity = vf).
  BOOST_TEST(ctx.status.control_velocity.value() == 0.3f);
}

// === Adversarial retargeting ===
//
// The existing mid-slew retarget tests cover the common cases.
// These add scenarios that stress the trajectory generator's
// stability and correctness when the host retargets aggressively:
//   - retargets at each phase of the 7-phase profile (not just
//     during the terminal slew);
//   - the "stop here NOW" command (retarget to current position
//     while a_curr is large);
//   - alternating retargets between two distant targets;
//   - pseudo-random retargets every K cycles;
//   - simultaneous position + velocity retarget;
//   - cross-mode switch (NaN out position mid-trajectory to fall
//     into the velocity-only path, and back).

// Run a trajectory toward 5.0 rev, retarget to `new_target`
// after `delay` cycles (or after the rest-commit latch fires /
// the trajectory completes, if `delay < 0`), and verify the
// controller reaches new_target within the kinematic residual.
// Parameterized across the full range of trajectory phases AND
// retarget direction.  Subsumes:
//
//   - "retarget mid-slew, same direction"        (after_latch, new=10)
//   - "retarget mid-slew, reverse direction"     (after_latch, new=-5)
//   - "retarget early, before latch fires"       (delay=100, new=10)
//   - "retarget after the first move completes"  (after_done,  new=-2)
//
// In the reverse-direction case the controller must brake from
// the terminal slew, reverse direction past the initial target,
// and re-engage toward the new (negative) target.  The latched
// cases are critical for the 1.1 regression: with the latch
// invalidation disabled, the controller silently coasts to the
// old predicted endpoint instead of following the new target.
BOOST_AUTO_TEST_CASE(JerkLimitRetargetAtEachPhase) {
  struct TestCase {
    // `delay == -1`: step until trajectory_rest_committed.
    // `delay == -2`: step until trajectory_done.
    // `delay >= 0`:  step exactly that many cycles.
    int delay;
    float new_target;
    const char* desc;
  };
  TestCase cases[] = {
    {    1, 10.0f, "cycle 1 (a == 0)" },
    {  100, 10.0f, "early slew-up (pre-latch)" },
    {  750, 10.0f, "mid slew-up" },
    { 1500, 10.0f, "a near a_max" },
    { 3000, 10.0f, "cruise at a_max" },
    { 5000, 10.0f, "brake slew" },
    { 7000, 10.0f, "brake cruise" },
    {   -1, 10.0f, "after latch, same direction" },
    {   -1, -5.0f, "after latch, reverse direction" },
    {   -2, -2.0f, "after done, reverse direction" },
  };

  for (const auto& tc : cases) {
    BOOST_TEST_CONTEXT(tc.desc << " (delay=" << tc.delay
                       << ", new_target=" << tc.new_target << ")") {
      Context ctx;
      StartRetargetMove(&ctx, 5.0f);

      if (tc.delay == -1) {
        // Step until the rest-commit latch fires, then retarget.
        const int max_pre = static_cast<int>(3.0f * kRetargetRateHz);
        StepUntil(&ctx, max_pre,
                  [&]() { return ctx.status.trajectory_rest_committed; });
        BOOST_TEST(ctx.status.trajectory_rest_committed == true);
      } else if (tc.delay == -2) {
        // Step until the trajectory fully completes, then retarget.
        const int max_pre = static_cast<int>(3.0f * kRetargetRateHz);
        StepUntil(&ctx, max_pre,
                  [&]() { return ctx.status.trajectory_done; });
        BOOST_TEST(ctx.status.trajectory_done == true);
      } else {
        for (int i = 0; i < tc.delay; i++) {
          ctx.Call();
          if (ctx.status.trajectory_done) { break; }
        }
      }

      Retarget(&ctx, tc.new_target, 0.0f);

      const int max_post_steps = static_cast<int>(10.0f * kRetargetRateHz);
      StepUntil(&ctx, max_post_steps,
                [&]() { return ctx.status.trajectory_done; });
      BOOST_TEST(ctx.status.trajectory_done == true);
      BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
      const double final_pos =
          ctx.from_raw(ctx.status.control_position_raw.value());
      BOOST_CHECK_LT(
          std::abs(final_pos - tc.new_target),
          ResidualBound(kRetargetAccel, kRetargetJerk, kRetargetRateHz));
    }
  }
}

// "Stop here NOW": host retargets to a position equal to the
// current control_position while a_curr is large (mid-acceleration).
// The trajectory generator must coast smoothly to a stop, slewing
// `a` back to 0.  The final position will not exactly match the
// retarget moment's control_position (v keeps integrating during
// the slew), but the residual must be bounded.
BOOST_AUTO_TEST_CASE(JerkLimitRetargetToCurrentPositionMidAccel) {
  Context ctx;
  StartRetargetMove(&ctx, 20.0f);

  // Run until we're mid-acceleration (a substantially non-zero).
  // For these params, a hits a_max around cycle 1500.
  for (int i = 0; i < 1200; i++) {
    ctx.Call();
  }
  BOOST_TEST(std::abs(ctx.status.control_acceleration) >
             0.3f * kRetargetAccel);

  // Retarget to where we currently are.  This is the "abort and
  // stay close" pattern: not the same as a velocity command, since
  // the trajectory generator will integrate v out for some cycles
  // before a reaches 0.
  const int64_t pos_at_retarget =
      ctx.status.control_position_raw.value();
  const float pos_at_retarget_f = ctx.from_raw(pos_at_retarget);
  Retarget(&ctx, pos_at_retarget_f, 0.0f);

  const int max_post_steps = static_cast<int>(10.0f * kRetargetRateHz);
  StepUntil(&ctx, max_post_steps,
            [&]() { return ctx.status.trajectory_done; });

  BOOST_TEST(ctx.status.trajectory_done == true);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
  // Position drifts by at most the slew-time integral of v.  For
  // a = a_max during the slew, v gain is a_max^2/(2j) and the
  // position integral is bounded by (a_max/j) * (v_at_retarget +
  // a_max^2/(2j)) -- a few revolutions max for typical params.
  const double final_pos =
      ctx.from_raw(ctx.status.control_position_raw.value());
  // Empirically the drift is ~4e-5; the analytical bound is
  // ResidualBound + (a/j) * v_at_retarget but the v term dominates
  // and is parameter-dependent.  Use 2x observed for safety.
  BOOST_CHECK_LT(std::abs(final_pos - pos_at_retarget_f), 1e-4);
}

// Alternating retargets between two distant positions at fixed
// interval.  The trajectory generator should not get stuck or
// run away in either coordinate, and (after the final retarget)
// should converge to the last-commanded target.
BOOST_AUTO_TEST_CASE(JerkLimitAlternatingRetargets) {
  Context ctx;
  StartRetargetMove(&ctx, 5.0f);

  // Alternate the position target every 200 cycles for 10 flips,
  // then let the trajectory settle.
  const float target_a = 5.0f;
  const float target_b = -5.0f;
  const int interval = 200;
  const int flips = 10;

  // Track that we never accelerate without bound.
  double max_abs_v = 0.0;
  double max_abs_x = 0.0;
  float current_target = target_a;
  for (int flip = 0; flip < flips; flip++) {
    current_target = (flip % 2 == 0) ? target_a : target_b;
    Retarget(&ctx, current_target, 0.0f);
    for (int i = 0; i < interval; i++) {
      ctx.Call();
      const double v = std::abs(ctx.status.control_velocity.value());
      const double x = std::abs(
          ctx.from_raw(ctx.status.control_position_raw.value()));
      if (v > max_abs_v) { max_abs_v = v; }
      if (x > max_abs_x) { max_abs_x = x; }
    }
  }

  // Velocity must stay bounded.  With accel=50 and an interval of
  // 200 cycles (6.67 ms), |v| can grow at most accel*interval*dt =
  // 50 * 200/30000 = 0.333 rev/s.  Empirically peak is ~0.22 rev/s
  // (the trajectory never reaches steady-state slew over a single
  // interval).  Allow 1.5x for safety.
  BOOST_TEST(
      max_abs_v < 1.5f * kRetargetAccel * interval / kRetargetRateHz);
  // Position never advances far from 0 between flips because each
  // flip kicks the velocity back toward the opposite sign before
  // significant displacement accumulates.  Empirically peak |x| is
  // ~0.007 rev; allow 5x for safety.
  BOOST_TEST(max_abs_x < 0.05);

  // Let trajectory settle on the final target.
  const int max_settle_steps = static_cast<int>(10.0f * kRetargetRateHz);
  StepUntil(&ctx, max_settle_steps,
            [&]() { return ctx.status.trajectory_done; });
  BOOST_TEST(ctx.status.trajectory_done == true);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
  const double final_pos =
      ctx.from_raw(ctx.status.control_position_raw.value());
  BOOST_CHECK_LT(
      std::abs(final_pos - current_target),
      ResidualBound(kRetargetAccel, kRetargetJerk, kRetargetRateHz));
}

// Pseudo-random retargets every K cycles for a fixed number of
// iterations, then let the trajectory settle on the final target.
// Verifies the trajectory generator stays stable under arbitrary
// retargeting and converges to the last command.
BOOST_AUTO_TEST_CASE(JerkLimitRandomRetargetingStress) {
  Context ctx;
  StartRetargetMove(&ctx, 0.0f);

  uint32_t rng = 0xdeadbeef;
  auto next_target = [&]() {
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    // Range: [-5, 5] rev.
    return 10.0f * (static_cast<int32_t>(rng) /
                    static_cast<float>(
                        std::numeric_limits<int32_t>::max())) - 5.0f;
  };

  const int retargets = 50;
  const int cycles_between = 300;
  float last_target = 0.0f;
  for (int i = 0; i < retargets; i++) {
    last_target = next_target();
    Retarget(&ctx, last_target, 0.0f);
    for (int j = 0; j < cycles_between; j++) {
      ctx.Call();
    }
  }

  // Now hold the last target and wait for completion.
  const int max_settle_steps = static_cast<int>(15.0f * kRetargetRateHz);
  StepUntil(&ctx, max_settle_steps,
            [&]() { return ctx.status.trajectory_done; });
  BOOST_TEST(ctx.status.trajectory_done == true);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
  const double final_pos =
      ctx.from_raw(ctx.status.control_position_raw.value());
  BOOST_CHECK_LT(
      std::abs(final_pos - last_target),
      ResidualBound(kRetargetAccel, kRetargetJerk, kRetargetRateHz));
}

// Mid-trajectory simultaneous change of position AND velocity
// targets.  The latch must invalidate on the position change (the
// velocity change alone is independently verified by
// JerkLimitMidSlewVelocityChangeReplan).  Final state must match
// the new (position, velocity) endpoint.
BOOST_AUTO_TEST_CASE(JerkLimitCombinedPositionVelocityRetarget) {
  Context ctx;
  StartRetargetMove(&ctx, 5.0f);

  // Step until rest-commit fires.
  const int max_pre_steps = static_cast<int>(2.0f * kRetargetRateHz);
  StepUntil(&ctx, max_pre_steps,
            [&]() { return ctx.status.trajectory_rest_committed; });
  BOOST_TEST(ctx.status.trajectory_rest_committed == true);

  // Change BOTH position and target velocity.
  const float new_pos = -3.0f;
  const float new_vel = 0.5f;
  Retarget(&ctx, new_pos, new_vel);
  ctx.Call();
  BOOST_TEST(ctx.status.trajectory_rest_committed == false);

  const int max_post_steps = static_cast<int>(15.0f * kRetargetRateHz);
  StepUntil(&ctx, max_post_steps,
            [&]() { return ctx.status.trajectory_done; });
  BOOST_TEST(ctx.status.trajectory_done == true);
  // Termination forces control_velocity = new_vel exactly.
  BOOST_TEST(ctx.status.control_velocity.value() == new_vel);
}

// Cross-mode switch: start a position-mode trajectory, mid-flight
// switch to velocity-only mode by setting data.position = NaN.
// The trajectory generator should track the velocity command via
// DoVelocityModeLimits and not fall back into the position path.
BOOST_AUTO_TEST_CASE(JerkLimitCrossModePositionToVelocity) {
  Context ctx;
  StartRetargetMove(&ctx, 10.0f);

  // Run for a bit to establish a position-mode trajectory.
  for (int i = 0; i < 500; i++) {
    ctx.Call();
  }
  BOOST_TEST(ctx.status.trajectory_done == false);

  // Switch to velocity-only mode.
  ctx.data.position = NaN;
  ctx.data.position_relative_raw.reset();
  ctx.data.velocity = 1.0f;

  // Continue running.  The trajectory should ramp v to 1.0 rev/s
  // via the velocity-mode rest-curve commit and terminate there.
  const int max_post_steps = static_cast<int>(5.0f * kRetargetRateHz);
  StepUntil(&ctx, max_post_steps,
            [&]() { return ctx.status.trajectory_done; });
  BOOST_TEST(ctx.status.trajectory_done == true);
  // Termination forces control_velocity = vf exactly.
  BOOST_TEST(ctx.status.control_velocity.value() == 1.0f);
}

// Cross-mode switch the other direction: velocity-mode in progress,
// then host supplies a position command.  The trajectory should
// pick up the position target on the next cycle.
BOOST_AUTO_TEST_CASE(JerkLimitCrossModeVelocityToPosition) {
  Context ctx;
  ctx.set_rate_hz(kRetargetRateHz);
  ctx.data.accel_limit = kRetargetAccel;
  ctx.data.jerk_limit = kRetargetJerk;
  ctx.data.velocity_limit = NaN;
  ctx.set_position(0.0f);
  ctx.set_velocity(0.0f);
  ctx.data.position = NaN;
  ctx.data.velocity = 1.5f;

  // Run until velocity-mode ramps to target.
  for (int i = 0; i < 50000; i++) {
    ctx.Call();
    if (ctx.status.trajectory_done) { break; }
  }
  BOOST_TEST(ctx.status.trajectory_done == true);
  BOOST_TEST(ctx.status.control_velocity.value() == 1.5f);

  // Now command a position move with vf=0.
  const float new_target = 5.0f;
  Retarget(&ctx, new_target, 0.0f);

  const int max_post_steps = static_cast<int>(15.0f * kRetargetRateHz);
  StepUntil(&ctx, max_post_steps,
            [&]() { return ctx.status.trajectory_done; });
  BOOST_TEST(ctx.status.trajectory_done == true);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
  const double final_pos =
      ctx.from_raw(ctx.status.control_position_raw.value());
  BOOST_CHECK_LT(
      std::abs(final_pos - new_target),
      ResidualBound(kRetargetAccel, kRetargetJerk, kRetargetRateHz));
}

// === Jerk-bound-through-termination tests ===
//
// The jerk-limited trajectory generator must respect |Δa| <= j*dt
// per cycle for EVERY cycle, including the final cycle where
// trajectory_done fires.  The legacy completion paths
// (`final_sign != initial_sign` in DoVelocityModeLimits, and the
// position-mode `|a| < j*dt` gate) snapped control_acceleration to
// 0 from a potentially non-zero value, producing a single-cycle
// jerk step that exceeded the bound by up to a_max.
//
// These tests differ from the existing JerkLimit*JerkBound tests in
// that they assert on Δa AT the termination cycle, not just on the
// cycles before it.  The earlier tests use `if (trajectory_done)
// break;` immediately before the bound check, which silently
// permits arbitrary jerk violations on the terminating cycle.

namespace {

// Run a trajectory to completion and verify |Δa| <= bound for every
// cycle, including the cycle on which trajectory_done first becomes
// true AND the cycle immediately after it.  Returns the number of
// cycles executed (>= 1).
//
// The slop on the bound accounts for the fact that the slew step `Δa
// = j*dt` is computed in the controller as `a_prev + j_dt`, and an
// external observer recovers Δa via subtraction.  The unit of least
// precision (ULP) at |a_prev| ~ a_max is `a_max * eps`, so the
// recovered Δa can disagree with the analytical j*dt by O(a_max *
// eps).  Empirically `accel * eps` is sufficient; we use the same
// value JerkBoundFor does for consistency.
int RunAndCheckJerkBound(Context* ctx, int max_steps,
                         float jerk, float accel, float period_s) {
  const float kFloatEps = std::numeric_limits<float>::epsilon();
  const float bound = jerk * period_s + accel * kFloatEps;
  float prev_a = ctx->status.control_acceleration;
  bool done = false;
  int steps = 0;
  for (; steps < max_steps; steps++) {
    ctx->Call();
    const float a = ctx->status.control_acceleration;
    const float da = std::abs(a - prev_a);
    BOOST_TEST(da <= bound);
    prev_a = a;
    if (done) { steps++; break; }
    if (ctx->status.trajectory_done) { done = true; }
  }
  return steps;
}

}

// Parameter sweep for jerk-bound-through-termination, in both
// position and velocity modes.  Covers a range of accel/jerk ratios
// and targets, including cases where peak `a` would not be a clean
// multiple of j*dt (which is the regime where the legacy `< j*dt`
// gate -- as opposed to the strict `== 0` gate -- snapped from a
// small non-zero value).  Velocity-mode rows set `xf = NaN`;
// position-mode rows set `xf` to the target position.
BOOST_AUTO_TEST_CASE(JerkLimitJerkBoundSweep) {
  struct TestCase {
    float xf;   // NaN -> velocity mode
    float vf;
    float accel;
    float jerk;
    float rate_hz;
    const char* desc;
  };
  TestCase cases[] = {
    // Velocity-mode rows.
    {   NaN,  1.5f,  1000.0f,  20000.0f, 30000.0f, "vel: canonical" },
    {   NaN, -1.5f,  1000.0f,  20000.0f, 30000.0f, "vel: negative target" },
    {   NaN,  0.5f,  1000.0f,  20000.0f, 30000.0f, "vel: small (triangle)" },
    {   NaN, 10.0f,   100.0f,   5000.0f, 30000.0f, "vel: trapezoidal-in-a" },
    {   NaN,  2.0f,   500.0f,  10000.0f, 15000.0f, "vel: low rate" },
    {   NaN,  3.0f,    50.0f,   1000.0f, 30000.0f, "vel: low j, asymm quanta" },
    {   NaN,  0.05f,  100.0f,   2000.0f, 30000.0f, "vel: tiny target" },
    // Position-mode rows.
    {  1.0f,  0.0f, 1000.0f,  20000.0f, 30000.0f, "pos: canonical" },
    {  5.0f,  0.0f, 2000.0f,  50000.0f, 30000.0f, "pos: fast move" },
    { -3.0f,  0.0f, 1500.0f,  30000.0f, 30000.0f, "pos: negative target" },
    {  0.5f,  0.0f,  500.0f,  10000.0f, 15000.0f, "pos: low rate" },
    {  5.0f,  0.0f,    5.0f,      2.0f, 30000.0f, "pos: low j (a=5,j=2)" },
    {  1.0f,  0.5f, 1000.0f,  20000.0f, 30000.0f, "pos: non-zero vf" },
  };

  for (const auto& tc : cases) {
    BOOST_TEST_CONTEXT(tc.desc) {
      Context ctx;
      ctx.set_rate_hz(tc.rate_hz);
      ctx.data.position = tc.xf;
      ctx.data.velocity = tc.vf;
      ctx.data.accel_limit = tc.accel;
      ctx.data.jerk_limit = tc.jerk;
      ctx.data.velocity_limit = NaN;
      const bool position_mode = std::isfinite(tc.xf);
      if (position_mode) {
        ctx.set_position(0.0f);
        ctx.set_velocity(0.0f);
      } else {
        ctx.status.control_position_raw = ctx.to_raw(0.0f);
        ctx.status.control_velocity = 0.0f;
      }

      const float max_time_s = position_mode ? 20.0f : 10.0f;
      const int max_steps =
          static_cast<int>(max_time_s * tc.rate_hz);
      const int steps =
          RunAndCheckJerkBound(
              &ctx, max_steps, tc.jerk, tc.accel, 1.0f / tc.rate_hz);
      BOOST_TEST(steps < max_steps);
      BOOST_TEST(ctx.status.trajectory_done == true);
      // Termination forces control_velocity = vf exactly.
      BOOST_TEST(ctx.status.control_velocity.value() == tc.vf);
      BOOST_TEST(ctx.status.control_acceleration == 0.0f);
    }
  }
}


// Mid-trajectory retargeting in velocity-only mode: change the
// velocity target while the controller is in the terminal slew, and
// verify the controller follows the new target (analog of
// JerkLimitMidSlewRetargetReachesNewTarget but for velocity mode).
BOOST_AUTO_TEST_CASE(JerkLimitVelocityModeMidSlewRetargetReachesNewTarget) {
  Context ctx;
  constexpr float kRateHz = 30000.0f;
  constexpr float kAccel = 100.0f;
  constexpr float kJerk = 2000.0f;

  ctx.set_rate_hz(kRateHz);
  ctx.data.position = NaN;
  ctx.data.velocity = 2.0f;
  ctx.data.accel_limit = kAccel;
  ctx.data.jerk_limit = kJerk;
  ctx.data.velocity_limit = NaN;
  ctx.status.control_position_raw = ctx.to_raw(0.0f);
  ctx.status.control_velocity = 0.0f;

  // Step until rest-commit fires.
  const int max_pre_steps = static_cast<int>(5.0f * kRateHz);
  int pre_steps = 0;
  for (; pre_steps < max_pre_steps; pre_steps++) {
    ctx.Call();
    if (ctx.status.trajectory_rest_committed) { break; }
  }
  BOOST_TEST(ctx.status.trajectory_rest_committed == true);
  BOOST_TEST(ctx.status.trajectory_done == false);
  BOOST_TEST(pre_steps < max_pre_steps);

  // Now retarget to a different velocity.
  ctx.data.velocity = -1.0f;
  ctx.Call();
  BOOST_TEST(ctx.status.trajectory_rest_committed == false);

  const int max_post_steps = static_cast<int>(10.0f * kRateHz);
  for (int i = 0; i < max_post_steps; i++) {
    ctx.Call();
    if (ctx.status.trajectory_done) { break; }
  }
  BOOST_TEST(ctx.status.trajectory_done == true);
  // Termination forces control_velocity = vf exactly.
  BOOST_TEST(ctx.status.control_velocity.value() == -1.0f);
}

// Verify the jerk bound is respected even when the host retargets
// repeatedly mid-trajectory.  The trajectory should still terminate
// and never violate |Δa| <= j*dt + slop, even on the cycles
// where the latch invalidates and the rest-curve check fires again.
BOOST_AUTO_TEST_CASE(JerkLimitVelocityModeRetargetJerkBoundPreserved) {
  Context ctx;
  constexpr float kRateHz = 30000.0f;
  constexpr float kAccel = 100.0f;
  constexpr float kJerk = 2000.0f;

  ctx.set_rate_hz(kRateHz);
  ctx.data.position = NaN;
  ctx.data.velocity = 2.0f;
  ctx.data.accel_limit = kAccel;
  ctx.data.jerk_limit = kJerk;
  ctx.data.velocity_limit = NaN;
  ctx.status.control_position_raw = ctx.to_raw(0.0f);
  ctx.status.control_velocity = 0.0f;

  const float dt = 1.0f / kRateHz;
  const float kFloatEps = std::numeric_limits<float>::epsilon();
  const float bound =
      kJerk * dt + std::max(1e-5f, 4.0f * kAccel * kFloatEps);
  float prev_a = 0.0f;

  // Retarget every 10ms.
  const int retarget_interval = static_cast<int>(0.01f * kRateHz);
  const float retarget_velocities[] = { 1.0f, -0.5f, 2.5f, 0.5f, -1.5f };
  int retarget_index = 0;

  const int max_steps = static_cast<int>(5.0f * kRateHz);
  for (int i = 0; i < max_steps; i++) {
    if (i > 0 && (i % retarget_interval) == 0 &&
        retarget_index < static_cast<int>(
            sizeof(retarget_velocities) / sizeof(retarget_velocities[0]))) {
      ctx.data.velocity = retarget_velocities[retarget_index++];
    }
    ctx.Call();
    const float a = ctx.status.control_acceleration;
    const float da = std::abs(a - prev_a);
    BOOST_TEST(da <= bound);
    prev_a = a;
  }

  // After the last retarget, the trajectory should eventually
  // settle on the final retarget value.
  const float final_target =
      retarget_velocities[
          (sizeof(retarget_velocities) /
           sizeof(retarget_velocities[0])) - 1];
  // Allow some settling time for the last target.
  const int settle_steps = static_cast<int>(5.0f * kRateHz);
  for (int i = 0; i < settle_steps; i++) {
    ctx.Call();
    const float a = ctx.status.control_acceleration;
    const float da = std::abs(a - prev_a);
    BOOST_TEST(da <= bound);
    prev_a = a;
    if (ctx.status.trajectory_done) { break; }
  }
  BOOST_TEST(ctx.status.trajectory_done == true);
  // Termination forces control_velocity = vf exactly.
  BOOST_TEST(ctx.status.control_velocity.value() == final_target);
}

// === Degenerate-startup tests ===
//
// Issue 1.3: when the host commands a position move whose target
// matches the current (x, v) state, the jerk-limited trajectory
// generator must terminate cleanly without an excursion.  The
// legacy bang-bang path is rescued from the same edge case by its
// "v crosses target" cross-detection -- the jerk path has no such
// rescue and was launching a phantom +/-a_max trajectory.
//
// These tests also exercise the broader "trajectory restart at
// rest" class of problems: repeated identical commands, host
// streaming, and position-equals-current with non-zero target
// velocity.

// Streaming the same position+velocity command every cycle (the
// pattern a 1 kHz "hold here" supervisor produces against a 30 kHz
// ISR) must keep the controller stable: position must not drift,
// velocity must remain at the commanded value, and acceleration
// must stay at 0.  Without the degenerate-startup short-circuit,
// every cycle would launch a fresh phantom +/-a_max trajectory.
// Variants of the degenerate-startup scenario: each case where
// target == current state must terminate immediately without
// motion.  Cases that ARE legitimate trajectories must NOT
// terminate on the first cycle (this is the regression guard).
//
// Immediate-termination cases additionally run 1000 cycles of
// re-streaming the same command (with position_relative_raw reset
// each cycle, mirroring what PrepareCommand does on every real
// command frame) and verify the controller stays stable over time.
// Without the degenerate-startup short-circuit, every cycle would
// launch a fresh phantom +/-a_max trajectory.
BOOST_AUTO_TEST_CASE(JerkLimitDegenerateStartupSweep) {
  struct TestCase {
    float target_pos;
    float target_vel;
    float initial_pos;
    float initial_vel;
    bool expect_immediate_termination;
    const char* desc;
  };
  TestCase cases[] = {
    // Degenerate -- should terminate in 1 cycle.
    { 0.0f, 0.0f, 0.0f, 0.0f, true,  "all zeros" },
    { 5.0f, 0.0f, 5.0f, 0.0f, true,  "non-origin same position" },
    { 2.5f, 0.0f, 2.5f, 0.0f, true,  "non-origin same position 2" },
    { -2.0f, 0.0f, -2.0f, 0.0f, true, "negative same position" },
    { 1.5f, 0.5f, 1.5f, 0.5f, true,  "matching nonzero velocity" },
    { 1.5f, -0.7f, 1.5f, -0.7f, true, "matching negative velocity" },
    // Non-degenerate (sanity / regression for "real trajectories
    // still run").
    { 1.0f, 0.0f, 0.0f, 0.0f, false, "real position move" },
    { 0.0f, 0.0f, 0.0f, 1.0f, false, "at position but moving" },
    { 0.0f, 1.0f, 0.0f, 0.0f, false, "at position, velocity mismatch" },
    { 1.0f, 1.0f, 1.0f, 0.0f, false, "at position, velocity wrong sign" },
  };

  for (const auto& tc : cases) {
    BOOST_TEST_CONTEXT(tc.desc) {
      Context ctx;
      ctx.set_rate_hz(30000.0f);
      ctx.data.position = tc.target_pos;
      ctx.data.velocity = tc.target_vel;
      ctx.data.accel_limit = 1000.0f;
      ctx.data.jerk_limit = 20000.0f;
      ctx.data.velocity_limit = NaN;
      ctx.set_position(tc.initial_pos);
      ctx.set_velocity(tc.initial_vel);
      ctx.status.control_position_raw = ctx.position.position_raw;
      ctx.status.control_velocity = tc.initial_vel;
      ctx.status.control_acceleration = 0.0f;
      const int64_t initial_pos_raw = ctx.position.position_raw;

      ctx.Call();

      if (tc.expect_immediate_termination) {
        BOOST_TEST(ctx.status.trajectory_done == true);
        BOOST_TEST(ctx.status.control_velocity.value() == tc.target_vel);
        BOOST_TEST(ctx.status.control_acceleration == 0.0f);
        // While tracking a moving target the controller must advance
        // position by exactly v*dt each cycle (within the fixed-point
        // step's int32 truncation, ~1 part in 2^32 of the per-cycle
        // step).  Worst observed relative error is ~1.1e-5 at
        // |target_vel| = 0.5, dominated by the 32-bit step
        // truncation; allow 2x for safety.
        const double advance_rev =
            ctx.from_raw(ctx.status.control_position_raw.value() -
                         initial_pos_raw);
        BOOST_TEST(advance_rev == tc.target_vel / 30000.0,
                   boost::test_tools::tolerance(2.5e-5));

        // Now stream the same command for 1000 cycles and verify
        // stability under host repetition.  Re-resetting
        // position_relative_raw mimics what PrepareCommand does on
        // every real command frame.  Only meaningful when
        // target_vel == 0 (otherwise the controller legitimately
        // advances control_position by v*dt per cycle, so re-sending
        // the same target_pos would race against the position
        // integration and the latch invalidation would trigger).
        if (tc.target_vel == 0.0f) {
          for (int i = 0; i < 1000; i++) {
            ctx.data.position = tc.target_pos;
            ctx.data.velocity = tc.target_vel;
            ctx.data.position_relative_raw.reset();
            ctx.Call();
            BOOST_TEST(ctx.status.trajectory_done == true);
            BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
            BOOST_TEST(ctx.status.control_acceleration == 0.0f);
            BOOST_TEST(
                ctx.status.control_position_raw.value() == initial_pos_raw);
          }
        }
      } else {
        // The trajectory must not be done after a single cycle for
        // any of these cases (each requires real motion to settle).
        BOOST_TEST(ctx.status.trajectory_done == false);
      }
    }
  }
}

// After a jerk-limited trajectory terminates, the next cycle re-
// enters the trajectory generator with (a, v, x) forced to
// (0, vf, xf).  Without the degenerate-startup early-out (position
// mode) and already-at-target early-out (velocity mode), the
// generator would launch a phantom trajectory and oscillate around
// vf.  Both modes are exercised here:
//
//   - position mode: idle cycles after completion (data.position_
//     relative_raw was reset by termination; data.position stays
//     set so the test mimics a host that keeps re-sending the same
//     command frame).  control_position_raw must not drift.
//
//   - velocity mode: explicit re-streaming of the same velocity
//     target every cycle.  control_velocity must remain at the
//     target.
BOOST_AUTO_TEST_CASE(JerkLimitPostCompletionStable) {
  struct TestCase {
    bool position_mode;
    float target_pos;
    float target_vel;
    const char* desc;
  };
  TestCase cases[] = {
    {  true, 1.0f, 0.0f, "position mode" },
    { false,  NaN, 1.5f, "velocity mode" },
  };

  for (const auto& tc : cases) {
    BOOST_TEST_CONTEXT(tc.desc) {
      Context ctx;
      constexpr float kRateHz = 30000.0f;
      ctx.set_rate_hz(kRateHz);
      ctx.data.position = tc.target_pos;
      ctx.data.velocity = tc.target_vel;
      ctx.data.accel_limit = 1000.0f;
      ctx.data.jerk_limit = 20000.0f;
      ctx.data.velocity_limit = NaN;
      if (tc.position_mode) {
        ctx.set_position(0.0f);
        ctx.set_velocity(0.0f);
      } else {
        ctx.status.control_position_raw = ctx.to_raw(0.0f);
        ctx.status.control_velocity = 0.0f;
      }

      // Run trajectory to completion.
      for (int i = 0; i < 200000; i++) {
        ctx.Call();
        if (ctx.status.trajectory_done) { break; }
      }
      BOOST_TEST(ctx.status.trajectory_done == true);
      BOOST_TEST(ctx.status.control_velocity.value() == tc.target_vel);

      // Latch the completion position; further cycles must not move it.
      const int64_t completion_pos_raw =
          ctx.status.control_position_raw.value();

      // Run 1000 more cycles.  Position mode: idle (data.position is
      // already NaN'd by termination, so no fresh command).  Velocity
      // mode: re-stream the same target every cycle to exercise the
      // already-at-target early-out under host repetition.
      //
      // Position assertion only applies to position mode -- velocity
      // mode legitimately keeps advancing control_position_raw by
      // target_vel*dt per cycle.
      for (int i = 0; i < 1000; i++) {
        if (!tc.position_mode) {
          ctx.data.velocity = tc.target_vel;
        }
        ctx.Call();
        BOOST_TEST(ctx.status.trajectory_done == true);
        BOOST_TEST(ctx.status.control_velocity.value() == tc.target_vel);
        BOOST_TEST(ctx.status.control_acceleration == 0.0f);
        if (tc.position_mode) {
          BOOST_TEST(
              ctx.status.control_position_raw.value() == completion_pos_raw);
        }
      }
    }
  }
}

// Position mode: after a jerk-limited trajectory completes, the
// host keeps re-sending the SAME position command at a low rate
// (the pattern produced by moteus.move_to() polling every ~2 ms
// against a 30 kHz ISR).  In production data.position is NaN'd by
// termination and stays NaN across the many ISR cycles between
// host polls, so the latch invalidation must NOT spuriously fire
// during the no-fresh-command gap, and the next host poll must
// re-engage the held latch instead of launching a tiny phantom
// trajectory.  Symptom of the bug: move_to() never returns even
// though the motor has reached the target.
BOOST_AUTO_TEST_CASE(JerkLimitPostCompletionStreamingSameTarget) {
  Context ctx;
  constexpr float kRateHz = 30000.0f;
  constexpr float kAccel = 5.0f;
  constexpr float kJerk = 2.0f;
  constexpr float kTarget = -0.25f;
  // 2 ms host polling against a 30 kHz ISR (matches the value
  // moteus.move_to() uses by default).
  constexpr int kCyclesPerPoll = 60;
  ctx.set_rate_hz(kRateHz);
  ctx.data.position = kTarget;
  ctx.data.velocity = 0.0f;
  ctx.data.accel_limit = kAccel;
  ctx.data.jerk_limit = kJerk;
  ctx.data.velocity_limit = NaN;
  ctx.set_position(0.0f);
  ctx.set_velocity(0.0f);
  ctx.status.control_position_raw = ctx.to_raw(0.0f);
  ctx.status.control_velocity = 0.0f;

  // Phase 1: poll until the trajectory completes.  Each "poll" is
  // kCyclesPerPoll ISR cycles: one cycle that simulates a fresh
  // command frame, then kCyclesPerPoll - 1 cycles with no host
  // command (the gap during which the FW formerly NaN'd
  // data.position and self-invalidated the commit latch).
  const int kMaxPolls = 5000;  // ~10 s real time at 2 ms polling
  int completion_poll = -1;
  int64_t completion_pos_raw = 0;
  for (int poll = 0; poll < kMaxPolls; poll++) {
    for (int cyc = 0; cyc < kCyclesPerPoll; cyc++) {
      if (cyc == 0) {
        ctx.data.position = kTarget;
        ctx.data.velocity = 0.0f;
        ctx.data.position_relative_raw.reset();
      }
      ctx.Call();
    }
    if (ctx.status.trajectory_done) {
      completion_poll = poll;
      completion_pos_raw = ctx.status.control_position_raw.value();
      break;
    }
  }
  BOOST_TEST(completion_poll >= 0);

  // Phase 2: 500 more polls.  trajectory_done must stay true; (v, a)
  // must stay at 0; control_position_raw must not drift.  Without
  // the fix, the post-termination NaN'ing of data.position causes
  // the latch tolerance check to invalidate on the very next ISR
  // cycle, and each host poll then launches a tiny phantom
  // trajectory against the float/fixed-point residual.
  for (int poll = 0; poll < 500; poll++) {
    for (int cyc = 0; cyc < kCyclesPerPoll; cyc++) {
      if (cyc == 0) {
        ctx.data.position = kTarget;
        ctx.data.velocity = 0.0f;
        ctx.data.position_relative_raw.reset();
      }
      ctx.Call();
    }
    BOOST_TEST(ctx.status.trajectory_done == true);
    BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
    BOOST_TEST(ctx.status.control_acceleration == 0.0f);
    BOOST_TEST(
        ctx.status.control_position_raw.value() == completion_pos_raw);
  }
}

// === Overspeed rest-curve tests ===
//
// Issue 1.4: when v_curr exceeds velocity_limit (because the host
// lowered the limit mid-motion or because a fresh trajectory
// started at a higher v_curr), CalculateJerkLimitedAcceleration
// used to unconditionally return -a_max.  The outer slew bounded
// |Δa| <= j*dt so the jerk limit was respected, but with no rest-
// curve pre-switch v continued to drop well below v_limit during
// the subsequent slew of `a` back toward 0 -- a smoothness
// regression relative to the symmetric approach-from-below path.
//
// The fix commits to target_a = 0 (the terminal slew) when
// `|a|^2 >= 2*j*(|v| - v_limit)` -- the kinematic condition for
// "slew a to 0 from now lands |v| at v_limit".  These tests
// exercise both the deep-overspeed case (seed control_velocity
// above the limit) and the mid-motion overspeed case (lower the
// velocity_limit while a trajectory is running).

namespace {

// Slop on the |Δa| <= j*dt bound, identical to the formula used by
// RunAndCheckJerkBound; the recovered Δa = a_curr - (a_curr - j*dt)
// loses a unit of least precision (ULP) at |a_curr| ~ a_max so we allow
// O(a_max * eps).  Empirically 0.25*accel*eps suffices, but 1x gives
// ~4x headroom for minor changes to the controller's float
// arithmetic.
float JerkBoundFor(float jerk, float accel, float period_s) {
  const float kFloatEps = std::numeric_limits<float>::epsilon();
  return jerk * period_s + accel * kFloatEps;
}

}

// The basic deep-overspeed completion + jerk-bound case is the
// `canonical` row of JerkLimitOverspeedRestCurveSweep below.  The
// test here adds the smoothness assertion via a focused trace:
// capture v over the first slice of the trajectory (before the
// final brake to 0) and verify it doesn't dip below v_limit by
// more than the discrete-step floor.  Run in both directions to
// catch sign-handling regressions in the rest-curve switch
// condition.
BOOST_AUTO_TEST_CASE(JerkLimitOverspeedRestCurveSmoothEntry) {
  const float dirs[] = { +1.0f, -1.0f };
  for (const float dir : dirs) {
    BOOST_TEST_CONTEXT("dir=" << dir) {
      Context ctx;
      const float rate_hz = 30000.0f;
      const float accel = 100.0f;
      const float jerk = 5000.0f;
      const float vel_limit = 2.0f;
      ctx.set_rate_hz(rate_hz);
      // Long trajectory so we have a clear cruise phase to look at.
      ctx.data.position = dir * 50.0f;
      ctx.data.velocity = 0.0f;
      ctx.data.accel_limit = accel;
      ctx.data.jerk_limit = jerk;
      ctx.data.velocity_limit = vel_limit;
      ctx.set_position(0.0f);
      ctx.set_velocity(0.0f);
      ctx.status.control_position_raw = ctx.to_raw(0.0f);
      ctx.status.control_acceleration = 0.0f;
      ctx.status.control_velocity = dir * 5.0f;

      const float dt = 1.0f / rate_hz;
      const float jerk_bound = JerkBoundFor(jerk, accel, dt);
      float prev_a = 0.0f;
      int violations = 0;

      // Stage 1: brake until v reaches v_limit-ish.  Track
      // jerk-bound violations across the brake to catch a
      // sign-handling regression that would let |Δa| spike at
      // the rest-curve switch.
      bool reached_limit = false;
      for (int i = 0; i < 100000; i++) {
        ctx.Call();
        const float a = ctx.status.control_acceleration;
        if (std::abs(a - prev_a) > jerk_bound) { violations++; }
        prev_a = a;
        if (dir * ctx.status.control_velocity.value() <= vel_limit) {
          reached_limit = true;
          break;
        }
      }
      BOOST_TEST(reached_limit);
      BOOST_TEST(violations == 0);

      // Stage 2: observe v over a window long enough to expose
      // any slew-back undershoot.  3 * (a_max / j) seconds is
      // enough.  Track the worst-direction extreme: for dir=+1
      // we look at the minimum v (closest to dipping below
      // v_limit); for dir=-1 we look at the maximum |v| (closest
      // to passing below -v_limit).
      const int observe_cycles =
          static_cast<int>(3.0f * accel / jerk * rate_hz);
      double worst_v = dir * ctx.status.control_velocity.value();
      for (int i = 0; i < observe_cycles; i++) {
        ctx.Call();
        const double v = dir * ctx.status.control_velocity.value();
        if (v < worst_v) { worst_v = v; }
      }

      // Legacy behavior: worst_v drops to vel_limit - a^2/(2j) =
      // 1.0 (with these params).  With the fix: worst_v stays
      // within one discrete-step of v_limit.  Empirically the dip
      // is ~accel*dt (one discrete acceleration step).  Allow 1.5x
      // for safety.
      const float allowed_dip = 1.5f * accel * dt;
      BOOST_TEST(worst_v >= vel_limit - allowed_dip);
    }
  }
}

// Mid-trajectory velocity-limit reduction: start with a normal
// trajectory inside the limit, then mid-flight lower the limit
// below the current control_velocity.  The controller must
// recognize the new overspeed condition and brake smoothly into
// the new cruise band, then continue toward the position target.
BOOST_AUTO_TEST_CASE(JerkLimitOverspeedRestCurveLowerLimitMidFlight) {
  Context ctx;
  const float rate_hz = 30000.0f;
  const float accel = 100.0f;
  const float jerk = 5000.0f;
  ctx.set_rate_hz(rate_hz);
  ctx.data.position = 50.0f;
  ctx.data.velocity = 0.0f;
  ctx.data.accel_limit = accel;
  ctx.data.jerk_limit = jerk;
  ctx.data.velocity_limit = 5.0f;  // generous initially
  ctx.set_position(0.0f);
  ctx.set_velocity(0.0f);

  // Run until v_curr has reached the original v_limit cruise (5.0).
  bool at_cruise = false;
  for (int i = 0; i < 100000; i++) {
    ctx.Call();
    if (ctx.status.control_acceleration == 0.0f &&
        ctx.status.control_velocity.value() >= 4.5f) {
      at_cruise = true;
      break;
    }
  }
  BOOST_TEST(at_cruise);

  // Now drop velocity_limit to 2.0; this puts the controller in
  // overspeed against the new limit.
  ctx.data.velocity_limit = 2.0f;

  const float dt = 1.0f / rate_hz;
  const float bound = JerkBoundFor(jerk, accel, dt);
  float prev_a = ctx.status.control_acceleration;
  int violations = 0;

  // The overspeed recovery should complete within roughly
  // 2*(a_max/j) plus the cruise-at-(-a_max) duration.  For these
  // params: 2 * 100/5000 + 1 sec of cruise = 0.04 + 0.20 = ~0.24
  // sec.  Use a 0.5 sec observation window -- long enough to cover
  // any reasonable recovery, short enough to not bleed into the
  // final position-target brake (cruise at v=2 for the remaining
  // ~48 rev takes ~24 sec).
  const int observe_cycles = static_cast<int>(0.5f * rate_hz);
  double min_v_during_recovery = ctx.status.control_velocity.value();
  for (int i = 0; i < observe_cycles; i++) {
    ctx.Call();
    const float a = ctx.status.control_acceleration;
    if (std::abs(a - prev_a) > bound) { violations++; }
    prev_a = a;
    const double v = ctx.status.control_velocity.value();
    if (v < min_v_during_recovery) { min_v_during_recovery = v; }
  }
  BOOST_TEST(violations == 0);
  // With the rest-curve pre-switch, v lands at v_limit (within a
  // small discrete-step floor).  Legacy behavior dips to
  // v_limit - a_max^2/(2j) = 2 - 1 = 1 -- well below this bound.
  // Empirically the dip is bounded by one discrete acceleration
  // step; allow 1.5x for safety.
  const float allowed_dip = 1.5f * accel * dt;
  BOOST_TEST(min_v_during_recovery >= 2.0f - allowed_dip);
}

// Parameter sweep on the overspeed scenario: a variety of accel /
// jerk ratios and overspeed magnitudes.  Each case must terminate
// and respect the jerk bound throughout.
BOOST_AUTO_TEST_CASE(JerkLimitOverspeedRestCurveSweep) {
  struct TestCase {
    float v_initial;
    float v_limit;
    float xf;
    float accel;
    float jerk;
    float rate_hz;
    const char* desc;
  };
  // Position targets are sized to keep cruise-at-v_limit duration
  // within the 20 sec test budget after the overspeed recovery.
  TestCase cases[] = {
    {  5.0f, 2.0f,  10.0f, 100.0f, 5000.0f,  30000.0f, "canonical" },
    { 20.0f, 2.0f,  10.0f, 100.0f, 5000.0f,  30000.0f, "10x overspeed" },
    {  2.5f, 2.0f,  10.0f, 100.0f, 5000.0f,  30000.0f, "small overspeed" },
    { 10.0f, 5.0f,  20.0f, 500.0f, 10000.0f, 30000.0f, "high accel/jerk" },
    {  5.0f, 1.0f,   5.0f,  50.0f, 1000.0f,  30000.0f, "low j" },
    { -5.0f, 2.0f, -10.0f, 100.0f, 5000.0f,  30000.0f, "negative" },
    {  3.0f, 2.0f,  10.0f, 100.0f, 5000.0f,  15000.0f, "low rate" },
  };

  for (const auto& tc : cases) {
    BOOST_TEST_CONTEXT(tc.desc) {
      Context ctx;
      ctx.set_rate_hz(tc.rate_hz);
      ctx.data.position = tc.xf;
      ctx.data.velocity = 0.0f;
      ctx.data.accel_limit = tc.accel;
      ctx.data.jerk_limit = tc.jerk;
      ctx.data.velocity_limit = tc.v_limit;
      ctx.set_position(0.0f);
      ctx.set_velocity(0.0f);
      ctx.status.control_position_raw = ctx.to_raw(0.0f);
      ctx.status.control_acceleration = 0.0f;
      ctx.status.control_velocity = tc.v_initial;

      const float dt = 1.0f / tc.rate_hz;
      const float bound = JerkBoundFor(tc.jerk, tc.accel, dt);
      float prev_a = ctx.status.control_acceleration;
      int violations = 0;
      const int max_steps = static_cast<int>(20.0f * tc.rate_hz);
      int steps = 0;
      for (; steps < max_steps; steps++) {
        ctx.Call();
        const float a = ctx.status.control_acceleration;
        if (std::abs(a - prev_a) > bound) { violations++; }
        prev_a = a;
        if (ctx.status.trajectory_done) { break; }
      }
      BOOST_TEST(steps < max_steps);
      BOOST_TEST(ctx.status.trajectory_done == true);
      BOOST_TEST(violations == 0);
    }
  }
}

// === Mid-motion accel_limit changes ===
//
// accel_limit is read fresh each cycle, so the trajectory
// generator adapts immediately to changes.  The two interesting
// transients are:
//   - lowering the limit while |a| is above the new ceiling
//     (the jerk bound wins over the new accel bound during the
//      slew back into range);
//   - clearing the limit to NaN, which discretely switches the
//     trajectory generator to DoVelocityOnlyLimit (a deliberate
//     mode switch -- the velocity-only path bypasses the
//     jerk-limit logic by design).
//
// These tests pin down the current behavior so a regression
// (intentional or otherwise) shows up.

// Sweep over the three "single-limit-change" mid-motion
// scenarios.  Each case runs until the controller has built up
// some `a`, mid-motion changes accel_limit, and then runs to
// completion while checking the per-cycle jerk bound through the
// transition cycle and onward.  The bound check straddling the
// change cycle catches any discrete jump in `a` -- the
// no-discrete-jump property the legacy code would have violated
// when clearing/lowering the cap.
//
// (The static "no accel cap" and "all limits cleared" cases live
// in their own tests below; each exercises a different code
// path -- the trajectory-generator-with-infinite-a_max path and
// the no-limits shortcut, respectively.)
BOOST_AUTO_TEST_CASE(JerkLimitAccelLimitMidMotionChanges) {
  struct TestCase {
    float initial_accel;          // NaN: start uncapped
    float vel_limit;              // NaN: no velocity cap
    int run_cycles;               // run this many cycles before the change
    float new_accel;              // NaN: clear the cap
    float pre_change_min_abs_a;   // sanity: |a| should be >= this
    const char* desc;
  };
  TestCase cases[] = {
    { 100.0f,  NaN,  800, 30.0f, 90.0f,
      "lowered: cap 100 -> 30 while |a| at old cap" },
    { 100.0f, 3.0f, 1000,   NaN, 50.0f,
      "cleared: cap removed mid-motion" },
    {   NaN, 10.0f,  500, 30.0f, 50.0f,
      "enabled: cap imposed (a was uncapped)" },
  };

  for (const auto& tc : cases) {
    BOOST_TEST_CONTEXT(tc.desc) {
      Context ctx;
      const float rate_hz = 30000.0f;
      const float jerk = 5000.0f;
      ctx.set_rate_hz(rate_hz);
      ctx.data.position = 30.0f;
      ctx.data.velocity = 0.0f;
      ctx.data.accel_limit = tc.initial_accel;
      ctx.data.jerk_limit = jerk;
      ctx.data.velocity_limit = tc.vel_limit;
      ctx.set_position(0.0f);
      ctx.set_velocity(0.0f);

      for (int i = 0; i < tc.run_cycles; i++) { ctx.Call(); }
      BOOST_TEST(std::abs(ctx.status.control_acceleration) >=
                 tc.pre_change_min_abs_a);

      const float a_pre_change = ctx.status.control_acceleration;
      ctx.data.accel_limit = tc.new_accel;

      // Bound slop: worst of (pre-change |a|, initial cap, new cap).
      const float dt = 1.0f / rate_hz;
      const float worst_a = std::max({
          std::abs(a_pre_change),
          std::isnan(tc.initial_accel) ? 0.0f : tc.initial_accel,
          std::isnan(tc.new_accel) ? 0.0f : tc.new_accel});
      const float bound = JerkBoundFor(jerk, worst_a, dt);
      float prev_a = a_pre_change;
      int violations = 0;
      const int max_steps = static_cast<int>(30.0f * rate_hz);
      int steps = 0;
      for (; steps < max_steps; steps++) {
        ctx.Call();
        const float a = ctx.status.control_acceleration;
        if (std::abs(a - prev_a) > bound) { violations++; }
        prev_a = a;
        if (ctx.status.trajectory_done) { break; }
      }
      BOOST_TEST(steps < max_steps);
      BOOST_TEST(ctx.status.trajectory_done == true);
      BOOST_TEST(violations == 0);
    }
  }
}

// NaN accel_limit with a finite jerk_limit means "no acceleration
// cap, jerk shaping still applies" -- the trajectory generator
// runs the jerk-limited path with a_max effectively infinity.
// Verifies the jerk bound holds and the trajectory terminates.
BOOST_AUTO_TEST_CASE(JerkLimitNoAccelLimit) {
  Context ctx;
  const float rate_hz = 30000.0f;
  const float jerk = 5000.0f;
  const float vel_limit = 3.0f;
  ctx.set_rate_hz(rate_hz);
  ctx.data.position = 10.0f;
  ctx.data.velocity = 0.0f;
  ctx.data.accel_limit = NaN;
  ctx.data.jerk_limit = jerk;
  ctx.data.velocity_limit = vel_limit;
  ctx.set_position(0.0f);
  ctx.set_velocity(0.0f);

  const float dt = 1.0f / rate_hz;
  // Peak |a| is bounded by sqrt(j*v_limit) = sqrt(15000) ~ 122 for
  // the v-limit triangle, plus headroom for unit of least precision
  // (ULP) slop.
  const float accel_for_bound = 2.0f * std::sqrt(jerk * vel_limit);
  const float bound = JerkBoundFor(jerk, accel_for_bound, dt);
  float prev_a = 0.0f;
  int violations = 0;
  const int max_steps = static_cast<int>(20.0f * rate_hz);
  int steps = 0;
  for (; steps < max_steps; steps++) {
    ctx.Call();
    const float a = ctx.status.control_acceleration;
    if (std::abs(a - prev_a) > bound) { violations++; }
    prev_a = a;
    if (ctx.status.trajectory_done) { break; }
  }
  BOOST_TEST(steps < max_steps);
  BOOST_TEST(ctx.status.trajectory_done == true);
  BOOST_TEST(violations == 0);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
}

// Mid-motion clearing of BOTH accel_limit and velocity_limit.
// Hits the top-of-UpdateCommand "no limits" shortcut that
// snaps (a, v) to (0, command_velocity) and immediately fires
// trajectory_done.
BOOST_AUTO_TEST_CASE(JerkLimitAllLimitsClearedMidMotion) {
  Context ctx;
  const float rate_hz = 30000.0f;
  const float accel = 100.0f;
  const float jerk = 5000.0f;
  ctx.set_rate_hz(rate_hz);
  ctx.data.position = 10.0f;
  ctx.data.velocity = 0.0f;
  ctx.data.accel_limit = accel;
  ctx.data.jerk_limit = jerk;
  ctx.data.velocity_limit = NaN;
  ctx.set_position(0.0f);
  ctx.set_velocity(0.0f);

  for (int i = 0; i < 1000; i++) {
    ctx.Call();
  }
  BOOST_TEST(std::abs(ctx.status.control_acceleration) >=
             0.5f * accel);

  // Clear both limits.
  ctx.data.accel_limit = NaN;
  ctx.data.velocity_limit = NaN;

  ctx.Call();
  // No-limits shortcut: trajectory immediately "done", state
  // snapped to (a=0, v=command_velocity=0).
  BOOST_TEST(ctx.status.trajectory_done == true);
  BOOST_TEST(ctx.status.control_acceleration == 0.0f);
  BOOST_TEST(ctx.status.control_velocity.value() == 0.0f);
}

// SlewAcceleration should respect the per-cycle da_limit bound.
BOOST_AUTO_TEST_CASE(SlewAccelerationBasics) {
  const float bound = 5.0f;

  // Change exceeding the limit gets clamped to +/-da_limit.
  BOOST_TEST(BldcServoPosition::SlewAcceleration(0.0f, 100.0f, bound) ==
             bound);
  BOOST_TEST(BldcServoPosition::SlewAcceleration(0.0f, -100.0f, bound) ==
             -bound);
  // Change within the limit is passed through exactly.
  BOOST_TEST(BldcServoPosition::SlewAcceleration(0.0f, bound / 2.0f, bound) ==
             bound / 2.0f);
  BOOST_TEST(BldcServoPosition::SlewAcceleration(
                 0.0f, -bound / 2.0f, bound) ==
             -bound / 2.0f);
  // Non-zero starting point is offset correctly.
  BOOST_TEST(BldcServoPosition::SlewAcceleration(10.0f, 100.0f, bound) ==
             15.0f);
  BOOST_TEST(BldcServoPosition::SlewAcceleration(10.0f, 12.0f, bound) ==
             12.0f);
}
