// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

#include <type_traits>

#include "mjlib/micro/error_code.h"

namespace moteus {

enum class errc {
  kSuccess = 0,

  kDmaStreamTransferError = 1,
  kDmaStreamFifoError = 2,
  kUartOverrunError = 3,
  kUartFramingError = 4,
  kUartNoiseError = 5,
  kUartBufferOverrunError = 6,
  kUartParityError = 7,

  kCalibrationFault = 32,
  kMotorDriverFault = 33,
  kOverVoltage = 34,
  kEncoderFault = 35,
  kMotorNotConfigured = 36,
  kPwmCycleOverrun = 37,
  kOverTemperature = 38,
  kStartOutsideLimit = 39,
};

mjlib::micro::error_code make_error_code(errc);
}

namespace mjlib {
namespace micro {

template <>
struct is_error_code_enum<moteus::errc> : std::true_type {};

}
}
