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

#include <fstream>

#include <fmt/format.h>

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace tt = boost::test_tools;

namespace {
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
    { 5.0,  0.0,    0.0, 0.0,   2.0, NaN, 40,   0.000, 3.205 },

    /////////////////////////////////
    // Accel and velocity limits
    { 0.0,  0.0,    3.0, 0.0,   1.0, 0.5, 40,   5.502, 6.500 },
    { 0.0,  0.0,    3.0, 0.0,   1.0, 0.7, 40,   3.588, 5.010 },
    { 0.0,  0.0,    3.0, 0.0,   2.0, 0.7, 40,   3.937, 4.645 },
    { 0.0,  0.3,    3.0, 0.0,   2.0, 0.7, 40,   3.969, 4.522 },
    { 0.3,  0.3,    3.0, 0.0,   2.0, 0.7, 40,   3.540, 4.094 },
    // overspeed
    { 0.3,  2.0,    3.0, 0.0,   2.0, 0.7, 40,   2.429, 3.436 },
    { -0.3, -2.0,  -3.0, 0.0,   2.0, 0.7, 40,   2.429, 3.436 },
    // overshoot
    { 0.3,  4.0,    3.0, 0.0,   2.0, 0.7, 40,   1.504, 4.207 },

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
