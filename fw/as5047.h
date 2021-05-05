// Copyright 2018-2020 Josh Pieper, jjp@pobox.com.
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

#include "mbed.h"

#include "hal/spi_api.h"

#include "mjlib/micro/persistent_config.h"

#include "fw/moteus_hw.h"
#include "fw/stm32_spi.h"

namespace moteus {

class AS5047 {
 public:
  struct Options : Stm32Spi::Options {
    PinName external_cs = NC;
  };

  enum Mode {
    kIntegrated = 0,
    kExternalSpi = 1,
    kNumModes = 2,
  };

  struct Config {
    Mode mode = kIntegrated;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(mode));
    }
  };

  AS5047(mjlib::micro::PersistentConfig* persistent_config,
         const Options& options)
      : options_(options),
        spi_([options]() {
          // The next frequency down is only 6MHz, so we run a bit out
          // of tolerance to save a fair amount of time.
          auto copy = options;
          copy.frequency = 12000000;
          return copy;
        }()) {
    persistent_config->Register(
        "encoder", &config_,
        std::bind(&AS5047::HandleConfigUpdate, this));
  }

  uint16_t Sample() MOTEUS_CCM_ATTRIBUTE {
    return (spi_.write(0xffff) & 0x3fff) << 2;
  }

  void StartSample() MOTEUS_CCM_ATTRIBUTE {
    return spi_.start_write(0xffff);
  }

  uint16_t FinishSample() MOTEUS_CCM_ATTRIBUTE {
    return (spi_.finish_write() & 0x3fff) << 2;
  }

 private:
  void HandleConfigUpdate() {
    switch (config_.mode) {
      case kIntegrated: {
        spi_.set_cs(options_.cs);
        return;
      }
      case kExternalSpi: {
        spi_.set_cs(options_.external_cs);
        return;
      }
      case kNumModes:{
        break;
      }
    }

    // Hmmm, guess we'll stick with integrated.
    spi_.set_cs(options_.cs);
  }

  const Options options_;
  Config config_;
  Stm32Spi spi_;
};

}


namespace mjlib {
namespace base {

template <>
struct IsEnum<moteus::AS5047::Mode> {
  static constexpr bool value = true;

  using M = moteus::AS5047::Mode;
  static std::array<std::pair<M, const char*>,
                    static_cast<int>(M::kNumModes)> map() {
    return { {
        { M::kIntegrated, "integrated" },
        { M::kExternalSpi, "external_spi" },
      } };
  }
};

}
}
