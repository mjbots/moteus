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

#include "fw/math.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace tt = boost::test_tools;

BOOST_AUTO_TEST_CASE(log2f_approx_test, * boost::unit_test::tolerance(1e-3f)) {
  const float test_values[] = {
    1.0,
    0.5,
    0.25,
    1.01,
    1.02,
    2.0,
    1.5,
    3.0,
    4.0,
    20.0,
    31.0,
    32.1,
  };
  for (const auto value : test_values) {
    BOOST_TEST_CONTEXT(value) {
      BOOST_TEST(std::abs(std::log2(value) - log2f_approx(value)) < 2e-3);
      // This has a relative tolerance instead of an absolute one.
      BOOST_TEST(pow2f_approx(log2f(value)) == value);
    }
  }

  BOOST_TEST(pow2f_approx(0.0024f) == 1.00166f);
  BOOST_TEST(pow2f_approx(log2f_approx(0.01)) == 0.01,
             boost::test_tools::tolerance(3e-2));
}

BOOST_AUTO_TEST_CASE(WrapZeroToTwoPiTest) {
  for (float v = -50.0f; v <= 50.0f; v += 0.001f) {
    BOOST_TEST_CONTEXT(v) {
      auto result = WrapZeroToTwoPi(v);
      const auto oracle_neg_pos = std::atan2(std::sin(v), std::cos(v));
      const auto corrected =
          (oracle_neg_pos > 0.0f) ?
          oracle_neg_pos :
          (2.0f * M_PI + oracle_neg_pos);
      BOOST_TEST(std::abs(corrected - result) < 0.001f);
    }
  }
}

BOOST_AUTO_TEST_CASE(FastAtan2Test) {
  BOOST_TEST(FastAtan2(1, 0) == 1.57079637f);
  BOOST_TEST(FastAtan2(1, 0.5) == 1.10713387f);
  BOOST_TEST(FastAtan2(-1, 0) == -1.57079637f);
  BOOST_TEST(FastAtan2(0, 1) == 0.0f);
  BOOST_TEST(FastAtan2(0, -1) == 3.14159274f);
}
