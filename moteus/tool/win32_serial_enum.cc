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

#pragma comment (lib, "Setupapi.lib")
#pragma comment (lib, "Advapi32.lib")

#include "moteus/tool/win32_serial_enum.h"

#include <initguid.h>
#include <windows.h>
#include <setupapi.h>

namespace moteus {
namespace tool {

namespace {
std::pair<uint16_t, uint16_t> ParseVidPid(const std::string& name) {
    auto get = [&](const std::string& prefix) -> uint16_t {
        const auto start = name.find(prefix);
        if (start == std::string::npos) { return 0; }
        const auto end = name.find_first_of(" &", start);
        if (end == std::string::npos) { return 0; }
        return std::stol(name.substr(start + prefix.size(), 4), nullptr, 16);
    };
    return std::make_pair(get("VID_"), get("PID_"));
}
}

std::vector<Win32SerialInfo> Win32SerialEnum() {
    const auto device_info_set =
        SetupDiGetClassDevs(nullptr, "USB", nullptr,
                            DIGCF_ALLCLASSES | DIGCF_PRESENT);

    if (device_info_set == INVALID_HANDLE_VALUE) {
        return {};
    }

    std::vector<Win32SerialInfo> result;

    SP_DEVINFO_DATA device_info_data = {};
    device_info_data.cbSize = sizeof(device_info_data);
    DWORD device_index = 0;

    while (SetupDiEnumDeviceInfo(device_info_set,
                                 device_index,
                                 &device_info_data)) {
        device_index++;

        auto get_property = [&](auto type) {
            char buffer[1024] = {};
            DEVPROPTYPE property_type = {};
            DWORD size = 0;
            const auto success = SetupDiGetDeviceRegistryProperty(
                device_info_set, &device_info_data, type,
                &property_type, reinterpret_cast<unsigned char*>(buffer),
                sizeof(buffer), &size);
            if (!success) { return std::string(); }
            return std::string(buffer, size);
        };

        const auto hardwareid = get_property(SPDRP_HARDWAREID);
        const auto friendly_name = get_property(SPDRP_FRIENDLYNAME);
        const auto device_description = get_property(SPDRP_DEVICEDESC);


        HKEY device_registry_key = SetupDiOpenDevRegKey(
            device_info_set, &device_info_data,
            DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (device_registry_key == INVALID_HANDLE_VALUE) {
            continue;
        }

        const auto port_name = [&]() -> std::string {
            char buffer[1024] = {};
            DWORD type = 0;
            DWORD size = sizeof(buffer);
            const auto success = RegQueryValueEx(
                device_registry_key, "PortName", nullptr, &type,
                reinterpret_cast<unsigned char*>(buffer), &size);
            return buffer;
        }();

        Win32SerialInfo info;
        info.device_name = port_name;
        info.friendly_name = friendly_name;
        info.device_description = device_description;
        auto vid_pid = ParseVidPid(hardwareid);
        info.usb_vid = vid_pid.first;
        info.usb_pid = vid_pid.second;

        result.push_back(info);

        RegCloseKey(device_registry_key);
    }

    SetupDiDestroyDeviceInfoList(device_info_set);

    return result;
}

}
}
