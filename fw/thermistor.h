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

namespace moteus {

/// Pre-calculate a thermistor table, then use it online to
/// efficiently determine the temperature given a thermistor ADC
/// reading.
class Thermistor {
 public:
  Thermistor() {}

  void Reset(float resistance_ohm) {
    for (size_t i = 0; i < table_.size(); i++) {
      table_[i] = MakeTable(i * 4096 / table_.size(), resistance_ohm);
    }
  }

  float Calculate(uint16_t adc_raw) MOTEUS_CCM_ATTRIBUTE {
    constexpr int adc_max = 4096;
    const size_t offset = std::max<size_t>(
        1, std::min<size_t>(
            table_.size() - 2,
            adc_raw * table_.size() / adc_max));
    const int16_t this_value = offset * adc_max / table_.size();
    const int16_t next_value = (offset + 1) * adc_max / table_.size();
    const float temp1 = table_[offset];
    const float temp2 = table_[offset + 1];
    return temp1 + (temp2 - temp1) *
        static_cast<float>(adc_raw - this_value) /
        static_cast<float>(next_value - this_value);
  }

 private:
  float MakeTable(size_t count, float thermistor_value) {
    const float v = 3.3f * count / 4096.0f;
    constexpr float B = 4050.0f;
    constexpr float R = 10000.0f;  // The resistor divider pair.

    float r_t = 3.3f * R / v - R;
    return (1.0f / (1.0f / (273.15f + 25.0f) +
                    (1.0f / B) *  std::log(r_t / thermistor_value)) -
            273.15f);
  }

  std::array<float, 32> table_ = {};
};
}
