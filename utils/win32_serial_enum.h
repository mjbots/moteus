// Copyright 2020 Josh Pieper, jjp@pobox.com.
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

#include <cstdint>
#include <string>
#include <vector>

namespace moteus {
namespace tool {

struct Win32SerialInfo {
    // This will be "COMN"
    std::string device_name;

    // Whatever Windows reports as the "friendly" name.
    std::string friendly_name;

    // For USB devices, the device name.
    std::string device_description;

    // For USB devices, the VID and PID.
    uint16_t usb_vid = 0;
    uint16_t usb_pid = 0;
};

struct SerialEnumOptions {
    bool verbose = false;

    SerialEnumOptions() {}
};

// Enumerate serial ports on Windows.
std::vector<Win32SerialInfo> Win32SerialEnum(const SerialEnumOptions& = {});

}
}
