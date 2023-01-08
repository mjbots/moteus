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

struct AuxPinConfig {
  int number = -1;
  PinName mbed = NC;
  int adc_num = -1;
  int adc_sqr = -1;
  I2C_TypeDef* i2c = nullptr;
  SPI_TypeDef* spi = nullptr;
  USART_TypeDef* uart = nullptr;
  TIM_TypeDef* timer = nullptr;
};

struct AuxExtraOptions {
  PinName i2c_pullup = NC;
  PinName rs422_re = NC;
  PinName rs422_de = NC;
};

struct AuxHardwareConfig {
  std::array<AuxPinConfig, 8> pins;
  AuxExtraOptions options = {};
};

struct SpiPinOption {
  SPI_TypeDef* spi = nullptr;
  PinName sck = NC;
  PinName miso = NC;
  PinName mosi = NC;
  PinName cs = NC;
};

struct UartPinOption {
  USART_TypeDef* uart = nullptr;
  PinName tx = NC;
  PinName rx = NC;
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
      const auto mbed = [&]() {
          for (const auto& pin : hw_config.pins) {
            if (pin.number == static_cast<int>(i)) {
              return pin.mbed;
            }
          }
          return NC;
      }();
      if (mbed == NC) { continue; }

      const auto pin = array[i];
      if (pin.mode == aux::Pin::Mode::kQuadratureSoftware) {
        if (!a_) {
          a_ = Stm32GpioInterruptIn::Make(
              mbed,
              &Stm32Quadrature::ISR_CallbackDelegate,
              reinterpret_cast<uint32_t>(this));
          if (!a_) {
            error_ = aux::AuxError::kQuadPinError;
            return;
          }
        } else if (!b_) {
          b_ = Stm32GpioInterruptIn::Make(
              mbed,
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
    status_->value = new_value % config_.cpr;
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
        const auto mbed = [&]() {
            for (const auto& pin : hw_config.pins) {
              if (pin.number == static_cast<int>(i)) { return pin.mbed; }
            }
            return NC;
        }();
        if (mbed == NC) { continue; }
        index_.emplace(mbed, MbedMapPull(cfg.pull));
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

enum RequireCs {
  kRequireCs,
  kDoNotRequireCs,
};

template <typename PinArray>
std::optional<SpiPinOption> FindSpiOption(const PinArray& pin_array,
                                          const AuxHardwareConfig& hw_config,
                                          RequireCs require_cs) {
  SpiPinOption result;

  // Figure out if appropriate pins are configured.
  int cs_count = 0;
  for (size_t i = 0; i < pin_array.size(); i++) {
    const auto& cfg = pin_array[i];
    if (cfg.mode == Pin::Mode::kSpiCs) {
      cs_count++;
      result.cs = [&]() {
                    for (const auto& pin : hw_config.pins) {
                      if (pin.number == static_cast<int>(i)) {
                        return pin.mbed;
                      }
                    }
                    return NC;
                  }();
    } else if (cfg.mode == Pin::Mode::kSpi ||
               (require_cs == kDoNotRequireCs && cfg.mode == Pin::Mode::kNC)) {
      bool found = false;
      for (const auto& pin : hw_config.pins) {
        if (pin.number != static_cast<int>(i)) { continue; }
        if (pin.spi == nullptr) { continue; }

        // This SPI peripheral isn't the one we're already aiming to
        // use.
        if (result.spi &&
            result.spi != pin.spi) { return {}; }
        result.spi = pin.spi;

        const auto int_spi = reinterpret_cast<uint32_t>(result.spi);

        for (uint32_t alt : {0, 0x100, 0x200, 0x300, 0x400}) {
          const PinName mbed_pin = static_cast<PinName>(pin.mbed | alt);
          // Figure out which thing it should be for.
          if (pinmap_find_peripheral(mbed_pin, PinMap_SPI_MOSI) == int_spi) {
            result.mosi = mbed_pin;
            found = true;
            break;
          } else if (pinmap_find_peripheral(mbed_pin, PinMap_SPI_MISO) == int_spi) {
            result.miso = mbed_pin;
            found = true;
            break;
          } else if (pinmap_find_peripheral(pin.mbed, PinMap_SPI_SCLK) == int_spi) {
            result.sck = mbed_pin;
            found = true;
            break;
          } else {
            // Just keep looking.
          }
        }
        if (found) { break; }
      }
      if (cfg.mode == Pin::Mode::kSpi && !found) {
        return {};
      }
    }
  }

  if (require_cs == kRequireCs &&
      result.cs == NC) {
    return {};
  }
  if (result.miso != NC &&
      result.mosi != NC &&
      result.sck != NC) {
    return result;
  }
  return {};
}

template <typename PinArray>
std::optional<UartPinOption> FindUartOption(const PinArray& pin_array,
                                            const AuxHardwareConfig& hw_config) {
  UartPinOption result;

  for (size_t i = 0; i < pin_array.size(); i++) {
    const auto& cfg = pin_array[i];
    if (cfg.mode == Pin::Mode::kUart) {
      const auto* pin = [&]() {
          for (const auto& pin : hw_config.pins) {
            if (pin.number != static_cast<int>(i)) { continue; }
            if (pin.uart) { return &pin; }
          }
          return static_cast<const AuxPinConfig*>(nullptr);
      }();
      if (pin == nullptr) { return {}; }

      if (result.uart && result.uart != pin->uart) {
        return {};
      }

      if (result.uart == nullptr) {
        result.uart = pin->uart;
      }

      const auto int_uart = reinterpret_cast<uint32_t>(result.uart);

      for (const auto alt : { 0, 0x100, 0x200, 0x300, 0x400 }) {
        const auto mbed_pin = static_cast<PinName>(pin->mbed | alt);
        if (pinmap_find_peripheral(mbed_pin, PinMap_UART_TX) == int_uart) {
          if (result.tx != NC) { return {}; }
          result.tx = mbed_pin;
          break;
        } else if (pinmap_find_peripheral(mbed_pin, PinMap_UART_RX) == int_uart) {
          if (result.rx != NC) { return {}; }

          result.rx = pin->mbed;
          break;
        }
      }
    }
  }

  if (result.uart &&
      result.tx != NC &&
      result.rx != NC) {
    return result;
  }

  return {};
}

}
}
