// Copyright 2022 Josh Pieper, jjp@pobox.com.
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

#include "fw/aux_common.h"
#include "fw/ccm.h"
#include "fw/stm32_gpio_interrupt_in.h"

namespace moteus {
namespace aux {

enum I2cCapability {
  kSda,
  kScl,
  kNoI2c,
};


struct AuxPinConfig {
  PinName mbed = NC;
  int adc_num = -1;
  int adc_sqr = -1;
  I2cCapability i2c = kNoI2c;
};

struct SpiPinOption {
  SPI_TypeDef* spi = nullptr;
  PinName sck = NC;
  PinName miso = NC;
  PinName mosi = NC;
};

struct UartPinOption {
  USART_TypeDef* uart = nullptr;
  PinName rx = NC;
  PinName tx = NC;
};

struct AuxHardwareConfig {
  std::array<AuxPinConfig, 5> pins;
  std::array<SpiPinOption, 2> spi_options = { {} };
  std::array<UartPinOption, 2> uart_options = { {} };
};

inline PinMode MbedMapPull(aux::Pin::Pull pull) {
  switch (pull) {
    case aux::Pin::kNone: { return PullNone; }
    case aux::Pin::kPullUp: { return PullUp; }
    case aux::Pin::kPullDown: { return PullDown; }
    case aux::Pin::kOpenDrain: { return OpenDrain; }
  }
  return PullNone;
}

class Stm32Quadrature {
 public:
  template <typename PinArray>
  Stm32Quadrature(const Quadrature::Config& config,
                  aux::Quadrature::Status* status,
                  const PinArray& array,
                  const AuxHardwareConfig& hw_config)
      : config_(config),
        status_(status) {
    // TODO: Try to use hardware eventually.

    int count = 0;
    for (const auto& pin : array) {
      if (pin.mode == aux::Pin::Mode::kQuadratureSoftware) {
        count++;
      }
      if (pin.mode == aux::Pin::Mode::kQuadratureHardware) {
        error_ = aux::AuxError::kQuadPinError;
        return;
      }
    }
    if (count != 2) {
      error_ = aux::AuxError::kQuadPinError;
      return;
    }

    for (size_t i = 0; i < array.size(); i++) {
      const auto pin = array[i];
      if (pin.mode == aux::Pin::Mode::kQuadratureSoftware) {
        if (!a_) {
          a_ = Stm32GpioInterruptIn::Make(
              hw_config.pins[i].mbed,
              &Stm32Quadrature::ISR_CallbackDelegate,
              reinterpret_cast<uint32_t>(this));
          if (!a_) {
            error_ = aux::AuxError::kQuadPinError;
            return;
          }
        } else if (!b_) {
          b_ = Stm32GpioInterruptIn::Make(
              hw_config.pins[i].mbed,
              &Stm32Quadrature::ISR_CallbackDelegate,
              reinterpret_cast<uint32_t>(this));
          if (!b_) {
            error_ = aux::AuxError::kQuadPinError;
            return;
          }
        }
      }
    }

    status_->active = true;
  }

  aux::AuxError error() { return error_; }

  void ISR_Update(aux::Quadrature::Status* status) MOTEUS_CCM_ATTRIBUTE {
    return;
  }

  static void ISR_CallbackDelegate(uint32_t my_this) MOTEUS_CCM_ATTRIBUTE {
    reinterpret_cast<Stm32Quadrature*>(my_this)->ISR_Callback();
  }

  void ISR_Callback() MOTEUS_CCM_ATTRIBUTE {
    static constexpr int8_t kQuadUpdate[] = {
      0, // 00 00 => 0
      1, // 00 01 => 1
      -1, // 00 10 => -1
      0, // 00 11 => ?
      -1, // 01 00 => -1
      0, // 01 01 => 0
      0, // 01 10 => ?
      1, // 01 11 => 1
      1, // 10 00 => 1
      0, // 10 01 => ?
      0, // 10 10 => 0
      -1, // 10 11 => -1
      0, // 11 00 => ?
      -1, // 11 01 => -1
      1, // 11 10 => 1
      0, // 11 11 => 0
    };
    static constexpr int8_t kQuadError[] = {
      0, // 00 00 => 0
      0, // 00 01 => 1
      0, // 00 10 => -1
      1, // 00 11 => ?
      0, // 01 00 => -1
      0, // 01 01 => 0
      1, // 01 10 => ?
      0, // 01 11 => 1
      0, // 10 00 => 1
      1, // 10 01 => ?
      0, // 10 10 => 0
      0, // 10 11 => -1
      1, // 11 00 => ?
      0, // 11 01 => -1
      0, // 11 10 => 1
      0, // 11 11 => 0
    };
    const auto old_pins = status_->pins;
    status_->pins = (
        (a_->read() ? 1 : 0) |
        (b_->read() ? 2 : 0));
    const uint8_t update = (old_pins << 2) | status_->pins;
    const uint32_t new_value =
        static_cast<uint32_t>(status_->value) +
        kQuadUpdate[update] +
        config_.cpr;
    status_->value =
        static_cast<uint16_t>(
            new_value % static_cast<uint32_t>(config_.cpr));
    status_->error += kQuadError[update];
  }

 private:
  const Quadrature::Config config_;
  Quadrature::Status* const status_;
  aux::AuxError error_ = aux::AuxError::kNone;
  std::optional<Stm32GpioInterruptIn> a_;
  std::optional<Stm32GpioInterruptIn> b_;
};

class Stm32Index {
 public:
  template <typename PinArray>
  Stm32Index(const Index::Config& config,
             const PinArray& array,
             const AuxHardwareConfig& hw_config) {
    for (size_t i = 0; i < array.size(); i++) {
      const auto& cfg = array[i];
      if (cfg.mode == Pin::Mode::kIndex) {
        if (index_) {
          error_ = aux::AuxError::kIndexPinError;
          return;
        }
        index_.emplace(hw_config.pins[i].mbed, MbedMapPull(cfg.pull));
      }
    }
    if (!index_) {
      error_ = aux::AuxError::kIndexPinError;
      return;
    }
  }

  void ISR_Update(aux::Index::Status* status) MOTEUS_CCM_ATTRIBUTE {
    if (error_ != aux::AuxError::kNone) { return; }

    const bool old_raw = status->raw;
    status->raw = index_->read() ? true : false;
    status->value = status->raw && !old_raw;
    status->active = true;
  }

  aux::AuxError error() { return error_; }

 private:
  aux::AuxError error_ = aux::AuxError::kNone;
  std::optional<DigitalIn> index_;
};

struct SpiResult {
  SpiPinOption option;
  PinName cs = NC;
};

enum RequireCs {
  kRequireCs,
  kDoNotRequireCs,
};

template <typename PinArray>
std::optional<SpiResult> FindSpiOption(const PinArray& pin_array,
                                       const AuxHardwareConfig& hw_config,
                                       RequireCs require_cs) {
  SpiResult result;

  // We need one CS pin, and exactly 3 SPI pins that match one of our
  // available configs.
  int cs_count = 0;
  int spi_count = 0;
  for (size_t i = 0; i < pin_array.size(); i++) {
    const auto& cfg = pin_array[i];
    if (cfg.mode == Pin::Mode::kSpiCs) {
      cs_count++;
      result.cs = hw_config.pins[i].mbed;
    } else if (cfg.mode == Pin::Mode::kSpi ||
               (require_cs == kDoNotRequireCs && cfg.mode == Pin::Mode::kNC)) {
      const auto mbed_pin = hw_config.pins[i].mbed;
      if (result.option.spi == nullptr) {
        // We don't have a SPI port decided yet.  See if we can find one.
        for (const auto& spi_option : hw_config.spi_options) {
          if (spi_option.spi == nullptr) { continue; }
          if ((spi_option.sck == mbed_pin) ||
              (spi_option.miso == mbed_pin) ||
              (spi_option.mosi == mbed_pin)) {
            result.option = spi_option;
            break;
          }
        }
        if (result.option.spi == nullptr) {
          if (cfg.mode == Pin::Mode::kSpi) {
            // We couldn't find a SPI port, return an error.
            return {};
          }
        } else {
          spi_count++;
        }
      } else {
        if (!(result.option.sck == mbed_pin ||
              result.option.miso == mbed_pin ||
              result.option.mosi == mbed_pin)) {
          if (cfg.mode == Pin::Mode::kSpi) {
            return {};
          }
        } else {
          spi_count++;
        }
      }
    }
  }
  const int required_cs_count = (require_cs == kRequireCs) ? 1 : 0;

  if (cs_count != required_cs_count ||
      spi_count != 3) {
    return {};
  }

  return result;
}

template <typename PinArray>
std::optional<UartPinOption> FindUartOption(const PinArray& pin_array,
                                            const AuxHardwareConfig& hw_config) {
  UartPinOption result;

  int uart_count = 0;
  for (size_t i = 0; i < pin_array.size(); i++) {
    const auto& cfg = pin_array[i];
    if (cfg.mode == Pin::Mode::kUart) {
      const auto mbed_pin = hw_config.pins[i].mbed;
      if (result.uart == nullptr) {
        // We don't have a UART decided yet.  See if we can find one.
        for (const auto& uart_option : hw_config.uart_options) {
          if (uart_option.uart == nullptr) { continue; }
          if ((uart_option.rx == mbed_pin) ||
              (uart_option.tx == mbed_pin)) {
            result = uart_option;
            uart_count++;
            break;
          }
        }
        if (result.uart == nullptr) {
          // We couldn't find a UART, return an error.
          return {};
        }
      } else {
        if (!(result.rx == mbed_pin ||
              result.tx == mbed_pin)) {
          return {};
        } else {
          uart_count++;
        }
      }
    }
  }

  if (uart_count != 2) {
    return {};
  }

  return result;
}

}
}
