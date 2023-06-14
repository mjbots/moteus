// Copyright 2018-2023 Josh Pieper, jjp@pobox.com.
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

class MotorDriver {
 public:
  /// Turn on or off the driver.  Return true on success.
  virtual bool Enable(bool) = 0;

  /// Enable power to the output stage.
  virtual void Power(bool) = 0;

  /// Return true if the driver is currently reporting a fault.
  virtual bool fault() = 0;
};

}
