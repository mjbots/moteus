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

#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

namespace moteus {

/// Holds information about the firmare.
class FirmwareInfo {
 public:
  FirmwareInfo(mjlib::micro::Pool&, mjlib::micro::TelemetryManager&,
               uint32_t version,
               uint32_t model);
  ~FirmwareInfo();

  uint32_t model_number() const;

  /// Return the firmware version encoded as 0x010203 major.minor.micro
  uint32_t firmware_version() const;

  struct SerialNumber {
    std::array<uint32_t, 3> number = {};
  };

  /// Return the unique serial number for this device.
  SerialNumber serial_number() const;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
