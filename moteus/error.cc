// Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "moteus/error.h"

namespace moteus {

namespace {
struct MoteusErrorCategory : mjlib::base::error_category {
  const char* name() const noexcept override { return "moteus"; }
  std::string_view message(int condition) const override {
    switch (static_cast<errc>(condition)) {
      case errc::kSuccess: return "success";
      case errc::kDmaStreamTransferError: return "dma stream transfer error";
      case errc::kDmaStreamFifoError: return "dma stream fifo error";
      case errc::kUartOverrunError: return "uart overrun error";
      case errc::kUartFramingError: return "uart framing error";
      case errc::kUartNoiseError: return "uart noise error";
      case errc::kUartBufferOverrunError: return "uart buffer overrun";
      case errc::kCalibrationFault: return "calibration fault";
      case errc::kMotorDriverFault: return "motor driver fault";
      case errc::kOverVoltage: return "over voltage";
      case errc::kEncoderFault: return "encoder fault";
      case errc::kCurrentControlFault: return "current control fault";
    }
    return "unknown";
  }
};

const mjlib::base::error_category& moteus_error_category() {
  static MoteusErrorCategory result;
  return result;
}
}

mjlib::base::error_code make_error_code(errc err) {
  return mjlib::base::error_code(
      static_cast<int>(err), moteus_error_category());
}

}
