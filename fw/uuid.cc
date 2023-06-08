// Copyright 2021-2023 Josh Pieper, jjp@pobox.com.
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

#include "fw/uuid.h"

namespace moteus {

const uint8_t* const g_otp_uuid = reinterpret_cast<const uint8_t*>(0x1fff7000);

Uuid::Uuid(mjlib::micro::PersistentConfig& config) {
  config.Register("uuid", &data_, [this]() { this->Update(); });

  Update();
}

void Uuid::Update() {
  // If any of the UUID bytes are non- 0xff, then just force our UUID
  // to be that from OTP.
  const bool otp_uuid_valid = [&]() {
    for (size_t i = 0; i < data_.uuid.size(); i++) {
      if (g_otp_uuid[i] != 0xff) { return true; }
    }
    return false;
  }();

  if (!otp_uuid_valid) { return; }

  for (size_t i = 0; i < data_.uuid.size(); i++) {
    data_.uuid[i] = g_otp_uuid[i];
  }
}



}
