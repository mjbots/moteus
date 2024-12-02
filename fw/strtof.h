// Copyright 2024 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include <cstdlib>
#include <string_view>

namespace moteus {

inline std::optional<float> Strtof(const char* data) {
  if (data == nullptr) {
    return {};
  }
  char* end = nullptr;
  const float result = std::strtof(&data[0], &end);
  if (end == nullptr ||
      (*end != 0 && !std::isspace(*end))) {
    return {};
  }
  return result;
}

inline std::optional<float> Strtof(const std::string_view& view) {
  return Strtof(view.data());
}

}
