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

#include "fw/git_info.h"

#include <cstring>

namespace moteus {

namespace {

uint8_t ParseHexNibble(uint8_t c) {
  if (c >= '0' && c <= '9') { return c - '0'; }
  if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
  if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
  return 0;
}

uint8_t ParseHexByte(const char* data) {
  return (ParseHexNibble(data[0]) << 4) | ParseHexNibble(data[1]);
}

}

GitInfo::GitInfo() {
  if (std::strlen(kGitHash) != 40) {
    dirty = true;
  } else {
    for (size_t i = 0; i <= 20; i++) {
      hash[i] = ParseHexByte(&kGitHash[i * 2]);
    }

    dirty = kGitDirty[0] != '0';
  }
  timestamp = kGitTimestamp;
}

char kGitHash[41] __attribute__((weak)) = {};
char kGitDirty[10] __attribute__((weak)) = {};
uint64_t kGitTimestamp __attribute__((weak)) = 0;
}
