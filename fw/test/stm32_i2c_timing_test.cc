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

#include <utility>

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
  // sdadel=10 (was 9 before the ceiling fix); the hold time at
  // sdadel=9 is 296.875 ns, below the 300 ns Standard-mode minimum.
  BOOST_TEST(result.sdadel == 10);
  BOOST_TEST(result.timingr == 0x108a93ac);
}

// Regression test: across the supported range of peripheral clocks
// and Standard mode, the realised hardware SDA hold time
// (tSDADEL = SDADEL * tPRESC + tI2CCLK, per RM0440) must meet the
// 300 ns NXP Standard-mode minimum referenced in stm32_i2c_timing.h.
BOOST_AUTO_TEST_CASE(StandardModeHoldTimeMeetsMinimum) {
  for (const int peripheral_hz : {64000000, 85000000, 128000000, 170000000}) {
    TimingInput input;
    input.peripheral_hz = peripheral_hz;
    input.i2c_hz = 100000;
    input.i2c_mode = I2cMode::kStandard;
    input.analog_filter = AnalogFilter::kOff;

    const auto result = CalculateI2cTiming(input);
    BOOST_TEST_REQUIRE(result.error == 0);

    const int64_t t_i2cclk_ps = 1000000000000ll / peripheral_hz;
    const int64_t t_presc_ps = t_i2cclk_ps * (result.prescaler + 1);
    const int64_t actual_hold_ps =
        static_cast<int64_t>(result.sdadel) * t_presc_ps + t_i2cclk_ps;
    BOOST_TEST(actual_hold_ps >= 300000,
               "peripheral_hz=" << peripheral_hz
               << " sdadel=" << result.sdadel
               << " prescaler=" << result.prescaler
               << " actual_hold_ps=" << actual_hold_ps);
  }
}

BOOST_AUTO_TEST_CASE(TimingTestAnalogFilterStandard) {
  TimingInput input;
  input.peripheral_hz = 64000000;
  input.i2c_hz = 100000;
  input.i2c_mode = I2cMode::kStandard;
  input.analog_filter = AnalogFilter::kOn;

  const auto result = CalculateI2cTiming(input);
  BOOST_TEST(result.error == 0);
  BOOST_TEST(result.prescaler == 1);
  BOOST_TEST(result.scldel == 8);
  BOOST_TEST(result.sclh == 147);
  BOOST_TEST(result.scll == 172);
  // The analog filter delays the internally observed SCL falling
  // edge by at least tAF(min)=50ns, which counts toward the hold
  // time, so a smaller SDADEL suffices than with the filter off.
  BOOST_TEST(result.sdadel == 8);
  BOOST_TEST(result.timingr == 0x108893ac);
}

// With the analog filter enabled, the realised hold time plus the
// tAF(min) credit must still meet the 300 ns Standard-mode minimum.
BOOST_AUTO_TEST_CASE(StandardModeHoldTimeMeetsMinimumAnalogFilter) {
  // STM32G474 datasheet section 5.3.27, Table 88.
  constexpr int64_t taf_min_ps = 50000;

  for (const int peripheral_hz : {64000000, 85000000, 128000000, 170000000}) {
    TimingInput input;
    input.peripheral_hz = peripheral_hz;
    input.i2c_hz = 100000;
    input.i2c_mode = I2cMode::kStandard;
    input.analog_filter = AnalogFilter::kOn;

    const auto result = CalculateI2cTiming(input);
    BOOST_TEST_REQUIRE(result.error == 0);

    const int64_t t_i2cclk_ps = 1000000000000ll / peripheral_hz;
    const int64_t t_presc_ps = t_i2cclk_ps * (result.prescaler + 1);
    const int64_t actual_hold_ps =
        static_cast<int64_t>(result.sdadel) * t_presc_ps + t_i2cclk_ps +
        taf_min_ps;
    BOOST_TEST(actual_hold_ps >= 300000,
               "peripheral_hz=" << peripheral_hz
               << " sdadel=" << result.sdadel
               << " prescaler=" << result.prescaler
               << " actual_hold_ps=" << actual_hold_ps);
  }
}

// The analog filter must not render any supported mode or peripheral
// clock infeasible.
BOOST_AUTO_TEST_CASE(AnalogFilterAllModesValid) {
  for (const int peripheral_hz : {64000000, 85000000, 128000000, 170000000}) {
    for (const auto [mode, i2c_hz] :
             {std::pair{I2cMode::kStandard, 100000},
              std::pair{I2cMode::kFast, 400000},
              std::pair{I2cMode::kFastPlus, 1000000}}) {
      TimingInput input;
      input.peripheral_hz = peripheral_hz;
      input.i2c_hz = i2c_hz;
      input.i2c_mode = mode;
      input.analog_filter = AnalogFilter::kOn;

      const auto result = CalculateI2cTiming(input);
      BOOST_TEST(result.error == 0,
                 "peripheral_hz=" << peripheral_hz
                 << " i2c_hz=" << i2c_hz);
    }
  }
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
  // With tI2CCLK = 7.8 ns, the implicit +tI2CCLK hold alone is below
  // the 10 ns minimum, so one SDADEL count is required.
  BOOST_TEST(result.sdadel == 1);
  BOOST_TEST(result.timingr == 0x00612a55);
}
