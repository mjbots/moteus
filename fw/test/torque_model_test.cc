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

#include "fw/torque_model.h"

#include <fmt/format.h>

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

BOOST_AUTO_TEST_CASE(BasicTorqueModel, * boost::unit_test::tolerance(1e-3f)) {
  TorqueModel dut(0.41f, 17.0f, 0.002f, 97.0f);

  struct Case {
    float current;
    float torque;
  };

  Case cases[] = {
    { 0.0, 0.0 },
    { 6.5, 2.665 },
    { 12.5, 5.125 },
    { 17.4, 7.208 },
    { 20.0, 7.919 },
    { 30.0, 10.627 },
    { 50.0, 15.912 },
  };

  for (const auto& test : cases) {
    BOOST_TEST_CONTEXT(fmt::format("torque={} current={}",
                                   test.torque, test.current)) {
      const float calculated_torque = dut.current_to_torque(test.current);
      BOOST_TEST(calculated_torque == test.torque);
      const float calculated_current = dut.torque_to_current(test.torque);
      BOOST_TEST(calculated_current == test.current,
                 boost::test_tools::tolerance(2.5e-2f));

      // And negative.
      BOOST_TEST(dut.current_to_torque(-test.current) == -test.torque);
      BOOST_TEST(dut.torque_to_current(-test.torque) == -test.current,
                 boost::test_tools::tolerance(2.5e-2f));
    }
  }
}
