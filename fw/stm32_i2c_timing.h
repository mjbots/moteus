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

#pragma once

#include <algorithm>
#include <cstdint>

namespace moteus {

struct TimingResult {
  // Non-zero implies the remaining values are unusable.
  int error = 0;

  int prescaler = 0;
  int scldel = 0;
  int sdadel = 0;
  int sclh = 0;
  int scll = 0;

  int digital_noise_filter = 0;

  uint32_t timingr = 0;
};

enum class I2cMode {
  kStandard,
  kFast,
  kFastPlus,
};

enum class AnalogFilter {
  kOff,
  kOn,
};

struct TimingInput {
  int peripheral_hz = 64000000;
  int i2c_hz = 100000;
  I2cMode i2c_mode = I2cMode::kStandard;
  AnalogFilter analog_filter = AnalogFilter::kOff;
  int rise_time_ns = 100;
  int fall_time_ns = 10;
};

namespace detail {
inline TimingResult TryTimingWithPrescaler(const TimingInput& i, int prescaler) {
  TimingResult result;
  result.prescaler = prescaler;
  const int64_t t_i2cclk_ps =
      1000000000000ll / static_cast<int64_t>(i.peripheral_hz);
  const int64_t t_presc_ps = t_i2cclk_ps * (prescaler + 1);

  // constexpr int64_t max_input_filter_ps = 50;
  const int64_t scl_min_low_ps =
      [&]() {
        switch (i.i2c_mode) {
          case I2cMode::kStandard: return 4700000;
          case I2cMode::kFast: return 1300000;
          case I2cMode::kFastPlus: return 500000;
        }
        return 0;
      }();
  const int64_t scl_min_high_ps =
      [&]() {
        switch (i.i2c_mode) {
          case I2cMode::kStandard: return 4000000;
          case I2cMode::kFast: return 600000;
          case I2cMode::kFastPlus: return 260000;
        }
        return 0;
      }();
  const int64_t data_min_hold_ps =
      [&]() {
        switch (i.i2c_mode) {
          // The NXP I2C doc says 0, but then in the footnote says it
          // needs to be at least 300ns.
          case I2cMode::kStandard: return 300000;
          // The NXP I2C doc lists 0 for both fast and fast+.  One
          // device, an AS5048, lists 10ns.  We'll go with 10.
          case I2cMode::kFast: return 10;
          case I2cMode::kFastPlus: return 10;
        }
        return 0;
      }();
  const int64_t data_min_setup_ps =
      [&]() {
        switch (i.i2c_mode) {
          case I2cMode::kStandard: return 250000;
          case I2cMode::kFast: return 100000;
          case I2cMode::kFastPlus: return 50000;
        }
        return 0;
      }();

  const int64_t i2c_max_hz =
      [&]() {
        switch (i.i2c_mode) {
          case I2cMode::kStandard: return 100000;
          case I2cMode::kFast: return 400000;
          case I2cMode::kFastPlus: return 1000000;
        }
        return 1000000;
      }();
  const int64_t actual_i2c_hz =
      std::max<int64_t>(1ll, std::min<int64_t>(i.i2c_hz, i2c_max_hz));
  const int64_t total_cycle_ps = 1000000000000ll / actual_i2c_hz;

  const int64_t desired_scl_low_ps =
      total_cycle_ps *
      [&]() {
        switch (i.i2c_mode) {
          case I2cMode::kStandard: return 54ll;
          case I2cMode::kFast: return 68ll;
          case I2cMode::kFastPlus: return 67ll;
        }
        return 0ll;
      }() / 100ll;

  result.scll = std::max(scl_min_low_ps, desired_scl_low_ps) / t_presc_ps;
  if (result.scll > 255) {
    // We can't achieve our rate with this prescaler.
    result.error = 2;
    return result;
  }

  const int64_t actual_scl_low_ps = (result.scll + 1) * t_presc_ps;

  const int64_t desired_scl_high_ps = total_cycle_ps - actual_scl_low_ps;
  result.sclh = std::max(scl_min_high_ps, desired_scl_high_ps) / t_presc_ps;
  if (result.sclh > 255) {
    result.error = 3;
    return result;
  }

  // const int64_t actual_scl_high_ps = (result.sclh + 1) * t_presc_ps;

  result.sdadel = data_min_hold_ps / t_presc_ps;
  if (result.sdadel > 15) {
    result.error = 4;
    return result;
  }

  result.scldel = data_min_setup_ps / t_presc_ps;
  if (result.scldel > 15) {
    result.error = 5;
    return result;
  }

  result.timingr =
      (result.scll << 0) |
      (result.sclh << 8) |
      (result.sdadel << 16) |
      (result.scldel << 20) |
      (result.prescaler << 28) |
      0;

  return result;
}
}

inline TimingResult CalculateI2cTiming(const TimingInput& i) {
  // We try to calculate things with progressively higher prescalers
  // until we find one that works.
  for (int prescaler = 0; prescaler < 16; prescaler++) {
    const auto result = detail::TryTimingWithPrescaler(i, prescaler);
    if (result.error) { continue; }

    return result;
  }

  // No solution was possible with any prescaler.
  TimingResult result;
  result.error = 1;
  return result;
}

}
