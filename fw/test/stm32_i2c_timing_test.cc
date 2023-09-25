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

#include "fw/stm32_i2c_timing.h"

#include <boost/test/auto_unit_test.hpp>

using namespace moteus;

namespace tt = boost::test_tools;

// Just some basic sanity checks

BOOST_AUTO_TEST_CASE(TimingTest1) {
  TimingInput input;
  input.peripheral_hz = 64000000;
  input.i2c_hz = 100000;
  input.i2c_mode = I2cMode::kStandard;
  input.analog_filter = AnalogFilter::kOff;

  const auto result = CalculateI2cTiming(input);
  BOOST_TEST(result.error == 0);
  BOOST_TEST(result.prescaler == 1);
  BOOST_TEST(result.scldel == 8);
  BOOST_TEST(result.sclh == 147);
  BOOST_TEST(result.scll == 172);
  BOOST_TEST(result.timingr == 0x108993ac);
}

BOOST_AUTO_TEST_CASE(TimingTest2) {
  TimingInput input;
  input.peripheral_hz = 64000000;
  input.i2c_hz = 400000;
  input.i2c_mode = I2cMode::kFast;
  input.analog_filter = AnalogFilter::kOff;

  const auto result = CalculateI2cTiming(input);
  BOOST_TEST(result.error == 0);
  BOOST_TEST(result.prescaler == 0);
  BOOST_TEST(result.scldel == 6);
  BOOST_TEST(result.sclh == 51);
  BOOST_TEST(result.scll == 108);
  BOOST_TEST(result.timingr == 0x0060336c);
}

BOOST_AUTO_TEST_CASE(TimingTest3) {
  TimingInput input;
  input.peripheral_hz = 64000000;
  input.i2c_hz = 1000000;
  input.i2c_mode = I2cMode::kFastPlus;
  input.analog_filter = AnalogFilter::kOff;

  const auto result = CalculateI2cTiming(input);
  BOOST_TEST(result.error == 0);
  BOOST_TEST(result.prescaler == 0);
  BOOST_TEST(result.scldel == 3);
  BOOST_TEST(result.sclh == 21);
  BOOST_TEST(result.scll == 42);
  BOOST_TEST(result.timingr == 0x0030152a);
}

BOOST_AUTO_TEST_CASE(TimingTest4) {
  TimingInput input;
  input.peripheral_hz = 128000000;
  input.i2c_hz = 1000000;
  input.i2c_mode = I2cMode::kFastPlus;
  input.analog_filter = AnalogFilter::kOff;

  const auto result = CalculateI2cTiming(input);
  BOOST_TEST(result.error == 0);
  BOOST_TEST(result.prescaler == 0);
  BOOST_TEST(result.scldel == 6);
  BOOST_TEST(result.sclh == 42);
  BOOST_TEST(result.scll == 85);
  BOOST_TEST(result.timingr == 0x00602a55);
}
