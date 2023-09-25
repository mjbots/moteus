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

#include <array>
#include <cstdint>

#include "mjlib/base/visitor.h"
#include "mjlib/micro/persistent_config.h"

namespace moteus {

class Uuid {
 public:
  Uuid(mjlib::micro::PersistentConfig& config);

  struct Data {
    std::array<uint8_t, 16> uuid = {};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(uuid));
    }
  };

 private:
  void Update();

  Data data_;
};

}
