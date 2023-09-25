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

#include "fw/firmware_info.h"

#include "fw/measured_hw_rev.h"
#include "fw/moteus_hw.h"

namespace moteus {

namespace {
struct Info {
  uint32_t version = 0;
  std::array<uint32_t, 3> serial_number = {};
  uint32_t model = 0;
  uint8_t family = g_measured_hw_family;
  uint8_t hwrev = g_measured_hw_rev;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(version));
    a->Visit(MJ_NVP(serial_number));
    a->Visit(MJ_NVP(model));
    a->Visit(MJ_NVP(family));
    a->Visit(MJ_NVP(hwrev));
  }
};
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-overread"

class FirmwareInfo::Impl {
 public:
  Impl(mjlib::micro::TelemetryManager& telemetry,
       uint32_t version,
       uint32_t model) {
    info_.version = version;
    info_.model = model;

    const int32_t* const device_signature =
        reinterpret_cast<const int32_t*>(
#if defined(TARGET_STM32G4)
            0x1fff7590
#else
#error "Unknown target"
#endif
                                         );

    std::memcpy(&info_.serial_number[0], device_signature,
                sizeof(uint32_t) * 3);
    telemetry.Register("firmware", &info_);
  }

  Info info_;
};

FirmwareInfo::FirmwareInfo(mjlib::micro::Pool& pool,
                           mjlib::micro::TelemetryManager& telemetry,
                           uint32_t version,
                           uint32_t model)
    : impl_(&pool, telemetry, version, model) {}

FirmwareInfo::~FirmwareInfo() {}

uint32_t FirmwareInfo::firmware_version() const {
  return impl_->info_.version;
}

FirmwareInfo::SerialNumber FirmwareInfo::serial_number() const {
  SerialNumber result;
  for (int i = 0; i < 3; i++) {
    result.number[i] = impl_->info_.serial_number[i];
  }
  return result;
}

#pragma GCC diagnostic pop

}
