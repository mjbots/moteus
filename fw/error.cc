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

#include "fw/error.h"

namespace moteus {

namespace {
struct MoteusErrorCategory : mjlib::micro::error_category {
  const char* name() const noexcept override { return "moteus"; }
  std::string_view message(int condition) const override {
    return "unknown";
  }
};

const mjlib::micro::error_category& moteus_error_category() {
  static MoteusErrorCategory result;
  return result;
}
}

mjlib::micro::error_code make_error_code(errc err) {
  return mjlib::micro::error_code(
      static_cast<int>(err), moteus_error_category());
}

}
