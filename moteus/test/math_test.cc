// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include "moteus/math.h"

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
