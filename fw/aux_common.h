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
#include <cstdint>

#include "mjlib/base/visitor.h"

namespace moteus {
namespace aux {

struct Spi {
  struct Config {
    enum Mode {
      kOnboardAs5047,
      kDisabled,
      kAs5047,
      kIcPz,
      kMa732,

      kNumModes,
    };
    Mode mode = kOnboardAs5047;
    uint32_t rate_hz = 12000000;

    // For now, only the MA732 uses these.
    uint16_t filter_us = 64;
    uint8_t bct = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(rate_hz));
      a->Visit(MJ_NVP(filter_us));
      a->Visit(MJ_NVP(bct));
    }
  };
  struct Status {
    bool active = false;
    uint32_t value = 0;
    uint8_t nonce = 0;

    uint8_t ic_pz_bits = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(active));
      a->Visit(MJ_NVP(value));
      a->Visit(MJ_NVP(nonce));
      a->Visit(MJ_NVP(ic_pz_bits));
    }
  };
};

struct UartEncoder {
  // Used for anything that communicates with the UART, either via
  // logic level, or via converters like BiSS-C, etc.
  struct Config {
    enum Mode {
      kDisabled,
      kAksim2,
      kTunnel,
      kDebug,
      kCuiAmt21,

      kNumModes,
    };
    Mode mode = kDisabled;

    int32_t baud_rate = 115200;
    int32_t poll_rate_us = 100;
    bool rs422 = false;
    uint8_t cui_amt21_address = 0x54;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(mode));
      a->Visit(MJ_NVP(baud_rate));
      a->Visit(MJ_NVP(poll_rate_us));
      a->Visit(MJ_NVP(rs422));
      a->Visit(MJ_NVP(cui_amt21_address));
    }
  };

  struct Status {
    bool active = false;
    uint32_t value = 0;
    uint8_t nonce = 0;

    bool aksim2_err = false;
    bool aksim2_warn = false;
    uint16_t aksim2_status = 0;
    uint16_t checksum_errors = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(active));
      a->Visit(MJ_NVP(value));
      a->Visit(MJ_NVP(nonce));

      a->Visit(MJ_NVP(aksim2_err));
      a->Visit(MJ_NVP(aksim2_warn));
      a->Visit(MJ_NVP(aksim2_status));
      a->Visit(MJ_NVP(checksum_errors));
    }
  };
};

struct Quadrature {
  struct Config {
    bool enabled = false;
    uint32_t cpr = 16384;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(enabled));
      a->Visit(MJ_NVP(cpr));
    }
  };

  struct Status {
    bool active = false;
    uint8_t pins = 0;
    uint32_t value = 0;
    uint16_t error = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(active));
      a->Visit(MJ_NVP(pins));
      a->Visit(MJ_NVP(value));
      a->Visit(MJ_NVP(error));
    }
  };
};

struct Hall {
  struct Config {
    bool enabled = false;
    uint8_t polarity = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(enabled));
      a->Visit(MJ_NVP(polarity));
    }
  };

  struct Status {
    bool active = false;
    uint8_t bits = 0;
    uint8_t count = 0;
    uint16_t error = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(active));
      a->Visit(MJ_NVP(bits));
      a->Visit(MJ_NVP(count));
      a->Visit(MJ_NVP(error));
    }
  };
};

struct Index {
  struct Config {
    bool enabled = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(enabled));
    }
  };

  struct Status {
    bool active = false;
    bool raw = false;
    bool value = false;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(active));
      a->Visit(MJ_NVP(raw));
      a->Visit(MJ_NVP(value));
    }
  };
};

struct SineCosine {
  struct Config {
    bool enabled = false;
    uint16_t common = 1700;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(enabled));
      a->Visit(MJ_NVP(common));
    }
  };

  struct Status {
    bool active = false;
    uint16_t sine_raw = 0;
    uint16_t cosine_raw = 0;
    uint16_t value = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(active));
      a->Visit(MJ_NVP(sine_raw));
      a->Visit(MJ_NVP(cosine_raw));
      a->Visit(MJ_NVP(value));
    }
  };
};

struct I2C {
  struct DeviceConfig {
    enum Type {
      kNone,
      kAs5048,
      kAs5600,

      kNumTypes,
    };
    Type type = kNone;
    uint8_t address = 0x40;
    int32_t poll_ms = 10;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(type));
      a->Visit(MJ_NVP(address));
      a->Visit(MJ_NVP(poll_ms));
    }
  };

  struct Config {
    int32_t i2c_hz = 400000;
    // 0 = standard, 1 = fast, 2 = fast+
    int32_t i2c_mode = 1;
    bool pullup = false;

    std::array<DeviceConfig, 3> devices = { {} };

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(i2c_hz));
      a->Visit(MJ_NVP(i2c_mode));
      a->Visit(MJ_NVP(pullup));
      a->Visit(MJ_NVP(devices));
    }
  };

  struct DeviceStatus {
    bool active = false;
    uint16_t value = 0;
    uint8_t nonce = 0;
    uint32_t error_count = 0;

    uint8_t ams_agc = 0;
    uint8_t ams_diag = 0;
    uint16_t ams_mag = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(active));
      a->Visit(MJ_NVP(value));
      a->Visit(MJ_NVP(nonce));
      a->Visit(MJ_NVP(error_count));

      a->Visit(MJ_NVP(ams_agc));
      a->Visit(MJ_NVP(ams_diag));
      a->Visit(MJ_NVP(ams_mag));
    }
  };

  struct Status {
    std::array<DeviceStatus, 3> devices = { {} };

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(devices));
    }
  };
};

struct Pin {
  enum Mode {
    kNC,
    kSpi,
    kSpiCs,
    kUart,
    kQuadratureSoftware,
    kQuadratureHardware,
    kHall,
    kIndex,
    kSine,
    kCosine,
    kStep,
    kDir,
    kRcPwm,
    kI2C,
    kDigitalInput,
    kDigitalOutput,
    kAnalogInput,

    kLength,
  };
  Mode mode = kNC;

  // Not every mode supports pullup or pulldown.
  enum Pull {
    kNone,
    kPullUp,
    kPullDown,
    kOpenDrain,
  };
  Pull pull = kNone;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(mode));
    a->Visit(MJ_NVP(pull));
  }
};

struct AuxConfig {
  aux::I2C::Config i2c;
  aux::Spi::Config spi;
  aux::UartEncoder::Config uart;
  aux::Quadrature::Config quadrature;
  aux::Hall::Config hall;
  aux::Index::Config index;
  aux::SineCosine::Config sine_cosine;
  int32_t i2c_startup_delay_ms = 30;

  static constexpr size_t kNumPins = 5;
  std::array<Pin, kNumPins> pins = { {} };

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(i2c));
    a->Visit(MJ_NVP(spi));
    a->Visit(MJ_NVP(uart));
    a->Visit(MJ_NVP(quadrature));
    a->Visit(MJ_NVP(hall));
    a->Visit(MJ_NVP(index));
    a->Visit(MJ_NVP(sine_cosine));
    a->Visit(MJ_NVP(i2c_startup_delay_ms));
    a->Visit(MJ_NVP(pins));
  }
};

enum class AuxError {
  kNone,
  kSpiPinError,
  kNotConfigured,
  kUnsupported,
  kI2cPinError,
  kHallPinError,
  kQuadPinError,
  kIndexPinError,
  kAdcPinError,
  kSineCosinePinError,
  kUartPinError,

  kLength,
};

struct AuxStatus {
  AuxError error = AuxError::kNone;

  I2C::Status i2c;
  Spi::Status spi;
  UartEncoder::Status uart;
  Quadrature::Status quadrature;
  Hall::Status hall;
  Index::Status index;
  SineCosine::Status sine_cosine;

  uint8_t gpio_bit_active = 0;
  std::array<bool, 5> pins = { {} };

  uint8_t analog_bit_active = 0;
  std::array<float, 5> analog_inputs = { {} };

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(i2c));
    a->Visit(MJ_NVP(error));
    a->Visit(MJ_NVP(spi));
    a->Visit(MJ_NVP(uart));
    a->Visit(MJ_NVP(quadrature));
    a->Visit(MJ_NVP(hall));
    a->Visit(MJ_NVP(index));
    a->Visit(MJ_NVP(sine_cosine));
    a->Visit(MJ_NVP(gpio_bit_active));
    a->Visit(MJ_NVP(pins));
    a->Visit(MJ_NVP(analog_bit_active));
    a->Visit(MJ_NVP(analog_inputs));
  }
};

}
}

namespace mjlib {
namespace base {

template <>
struct IsEnum<moteus::aux::Spi::Config::Mode> {
  static constexpr bool value = true;

  using M = moteus::aux::Spi::Config::Mode;
  static std::array<std::pair<M, const char*>, M::kNumModes> map() {
    return {{
        { M::kOnboardAs5047, "onboard" },
        { M::kDisabled, "disabled" },
        { M::kAs5047, "ext_as5047" },
        { M::kIcPz, "ic_pz" },
        { M::kMa732, "ma732" },
      }};
  }
};

template <>
struct IsEnum<moteus::aux::UartEncoder::Config::Mode> {
  static constexpr bool value = true;

  using M = moteus::aux::UartEncoder::Config::Mode;

  static std::array<std::pair<M, const char*>, M::kNumModes> map() {
    return {{
        { M::kDisabled, "disabled" },
        { M::kAksim2, "aksim2" },
        { M::kTunnel, "tunnel" },
        { M::kDebug, "debug" },
        { M::kCuiAmt21, "cui_amt21" },
      }};
  }
};

template <>
struct IsEnum<moteus::aux::I2C::DeviceConfig::Type> {
  static constexpr bool value = true;

  using T = moteus::aux::I2C::DeviceConfig::Type;

  static std::array<std::pair<T, const char*>, T::kNumTypes> map() {
    return {{
        { T::kNone, "none" },
        { T::kAs5048, "as5048" },
        { T::kAs5600, "as5600" },
      }};
  }
};

template <>
struct IsEnum<moteus::aux::Pin::Mode> {
  static constexpr bool value = true;

  using P = moteus::aux::Pin::Mode;

  static std::array<std::pair<P, const char*>,
                    static_cast<int>(P::kLength)> map() {
    return {{
        { P::kNC, "nc" },
        { P::kSpi, "spi" },
        { P::kSpiCs, "spi_cs" },
        { P::kUart, "uart" },
        { P::kQuadratureSoftware, "quad_sw" },
        { P::kQuadratureHardware, "quad_hw" },
        { P::kHall, "hall" },
        { P::kIndex, "index" },
        { P::kSine, "sine" },
        { P::kCosine, "cosine" },
        { P::kStep, "step" },
        { P::kDir, "dir" },
        { P::kRcPwm, "rc_pwm" },
        { P::kI2C, "i2c" },
        { P::kDigitalInput, "digital_in" },
        { P::kDigitalOutput, "digital_out" },
        { P::kAnalogInput, "analog_in" },
      }};
  }
};

template <>
struct IsEnum<moteus::aux::Pin::Pull> {
  static constexpr bool value = true;

  using P = moteus::aux::Pin::Pull;
  static std::array<std::pair<P, const char*>, 4> map() {
    return {{
        { P::kNone, "none" },
        { P::kPullUp, "pull_up" },
        { P::kPullDown, "pull_down" },
        { P::kOpenDrain, "open_drain" },
      }};
  }
};

template <>
struct IsEnum<moteus::aux::AuxError> {
  static constexpr bool value = true;

  using A = moteus::aux::AuxError;

  static std::array<std::pair<A, const char*>,
                    static_cast<int>(A::kLength)> map() {
    return {{
        { A::kNone, "none" },
        { A::kSpiPinError, "spi_pin_error" },
        { A::kNotConfigured, "not_configured" },
        { A::kUnsupported, "unsupported" },
        { A::kI2cPinError, "i2c_pin_error" },
        { A::kHallPinError, "hall_pin_error" },
        { A::kQuadPinError, "quad_pin_error" },
        { A::kIndexPinError, "index_pin_error" },
        { A::kAdcPinError, "adc_pin_error" },
        { A::kSineCosinePinError, "sine_cosine_pin_error" },
        { A::kUartPinError, "uart_pin_error" },
      }};
  }
};


}
}
