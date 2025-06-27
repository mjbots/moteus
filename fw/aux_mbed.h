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

#include <atomic>

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

/// Figure out which mbed alt pin is associated with the given STM32
/// timer.
inline PinName FindTimerAlt(PinName pin, TIM_TypeDef* timer) {
  const auto int_timer = reinterpret_cast<uint32_t>(timer);

  for (uint32_t alt : {0, 0x100, 0x200, 0x300, 0x400}) {
    const PinName mbed_pin = static_cast<PinName>(pin | alt);
    if (pinmap_find_peripheral(mbed_pin, PinMap_PWM) == int_timer) {
      return mbed_pin;
    }
  }
  return NC;
}

class Stm32Quadrature {
 public:
  template <typename PinArray>
  Stm32Quadrature(const Quadrature::Config& config,
                  aux::Quadrature::Status* status,
                  const PinArray& array,
                  size_t array_size,
                  const AuxHardwareConfig& hw_config)
      : config_(config),
        status_(status) {
    aux::AuxPinConfig pina = {};
    aux::Pin::Mode pina_mode = {};
    aux::AuxPinConfig pinb = {};
    aux::Pin::Mode pinb_mode = {};

    for (size_t i = 0; i < array_size; i++) {
      const auto& pin = array[i];

      if (pin.mode != aux::Pin::Mode::kQuadratureSoftware &&
          pin.mode != aux::Pin::Mode::kQuadratureHardware) {
        continue;
      }

      const auto this_pin = [&]() {
          for (const auto& pin : hw_config.pins) {
            if (pin.number == static_cast<int>(i)) {
              return pin;
            }
          }
          return aux::AuxPinConfig();
      }();
      if (this_pin.mbed == NC) { continue; }

      if (pina.mbed == NC) {
        pina = this_pin;
        pina_mode = pin.mode;
      } else if (pinb.mbed == NC) {
        pinb = this_pin;
        pinb_mode = pin.mode;
      } else {
        error_ = aux::AuxError::kQuadPinError;
        return;
      }
    }
    // If we have a mix of hardware and software, trigger an error.
    if (pinb.mbed == NC ||
        pina_mode != pinb_mode) {
      error_ = aux::AuxError::kQuadPinError;
      return;
    }

    if (pina_mode == aux::Pin::Mode::kQuadratureSoftware) {
      a_ = Stm32GpioInterruptIn::Make(
          pina.mbed,
          &Stm32Quadrature::ISR_CallbackDelegate,
          reinterpret_cast<uint32_t>(this));
      if (!a_) {
        error_ = aux::AuxError::kQuadPinError;
        return;
      }
      b_ = Stm32GpioInterruptIn::Make(
          pinb.mbed,
          &Stm32Quadrature::ISR_CallbackDelegate,
          reinterpret_cast<uint32_t>(this));
      if (!b_) {
        error_ = aux::AuxError::kQuadPinError;
        return;
      }
    } else if (pina_mode == aux::Pin::Mode::kQuadratureHardware) {
      // Check to see if the two pins are on the same timer.
      if (pina.timer != pinb.timer ||
          pina.timer == nullptr) {
        // Either they aren't on the same timer, or they aren't
        // capable of hardware quadrature at all.
        error_ = aux::AuxError::kQuadPinError;
        return;
      }

      a_in_.emplace(pina.mbed);
      b_in_.emplace(pinb.mbed);

      const auto pina_alt = FindTimerAlt(pina.mbed, pina.timer);
      const auto pinb_alt = FindTimerAlt(pinb.mbed, pinb.timer);

      // Set the alternate function for each pin.
      pinmap_pinout(pina_alt, PinMap_PWM);
      pinmap_pinout(pinb_alt, PinMap_PWM);

      // Configure our timer.
      hwtimer_ = pina.timer;

      hwtimer_->CR1 = 0;

      hwtimer_->PSC = 0;  // no prescaler
      hwtimer_->ARR = 0xffff;
      hwtimer_->SMCR = (0x03 << TIM_SMCR_SMS_Pos);
      hwtimer_->CCMR1 =
          (1 << TIM_CCMR1_CC1S_Pos) | // CC1 is from tim_ic1
          (1 << TIM_CCMR1_CC2S_Pos);  // CC2 is from tim_ic2
      hwtimer_->CCER =
          TIM_CCER_CC1E | // Enable capture channel 1
          TIM_CCER_CC2E;  // Enable capture channel 2

      hwtimer_->EGR = 1;
      hwtimer_->CR1 = (TIM_CR1_CEN);
      old_timer_cnt_ = hwtimer_->CNT;
    }

    status_->active = true;
  }

  aux::AuxError error() { return error_; }

  void ISR_Update(aux::Quadrature::Status* status) MOTEUS_CCM_ATTRIBUTE {
    if (!hwtimer_) { return; }

    status_->pins =
        (a_in_->read() ? 1 : 0) |
        (b_in_->read() ? 2 : 0);

    const uint32_t new_cnt = hwtimer_->CNT;
    const int16_t delta =
        static_cast<int16_t>(
            static_cast<uint16_t>((new_cnt - old_timer_cnt_) & 0xffff));

    old_timer_cnt_ = new_cnt;

    const uint32_t new_value =
        static_cast<uint32_t>(status_->value) + delta + config_.cpr;
    status->value = new_value % config_.cpr;
    status->error = 0;
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

  TIM_TypeDef* hwtimer() const {
    return hwtimer_;
  }

 private:
  const Quadrature::Config config_;
  Quadrature::Status* const status_;
  aux::AuxError error_ = aux::AuxError::kNone;
  std::optional<Stm32GpioInterruptIn> a_;
  std::optional<Stm32GpioInterruptIn> b_;

  std::optional<DigitalIn> a_in_;
  std::optional<DigitalIn> b_in_;

  uint32_t old_timer_cnt_ = 0;

  TIM_TypeDef* hwtimer_ = nullptr;
};

class Stm32Index {
 public:
  template <typename PinArray>
  Stm32Index(const Index::Config& config,
             const PinArray& array,
             size_t array_size,
             const AuxHardwareConfig& hw_config)
      : invert_(config.invert) {
    for (size_t i = 0; i < array_size; i++) {
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
        index_isr_ = Stm32GpioInterruptIn::Make(
            mbed,
            &Stm32Index::ISR_CallbackDelegate,
            reinterpret_cast<uint32_t>(this));
        if (!index_isr_) {
          error_ = aux::AuxError::kIndexPinError;
          return;
        }
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
    const bool observed = observed_.exchange(false);
    status->raw = observed || (invert_ ^ index_isr_->read());
    status->value = status->raw && !old_raw;
    status->active = true;
  }

  aux::AuxError error() { return error_; }

  void ISR_Callback() MOTEUS_CCM_ATTRIBUTE {
    // The principle here is that we capture any high readings in the
    // ISR so that the minimum pulse width we can read is determined
    // by the ISR latency, not by the control period.
    if (invert_ ^ index_isr_->read()) { observed_.store(true); }
  }

  static void ISR_CallbackDelegate(uint32_t my_this) MOTEUS_CCM_ATTRIBUTE {
    reinterpret_cast<Stm32Index*>(my_this)->ISR_Callback();
  }

 private:
  bool invert_ = false;
  aux::AuxError error_ = aux::AuxError::kNone;
  std::atomic<bool> observed_{false};
  std::optional<Stm32GpioInterruptIn> index_isr_;
  std::optional<DigitalIn> index_;
};

/// This manages a single STM32 timer for PWM purposes, which may have
/// one or more pins operating on it at a time.
class Stm32PwmTimer {
 public:
  Stm32PwmTimer(TIM_TypeDef* timer,
                int period_us)
      : hwtimer_(timer) {
    // Configure the prescaler to get maximum resolution within the
    // desired period.
    const auto ticks_per_period =
        static_cast<int64_t>(HAL_RCC_GetPCLK1Freq() * 2) *
        period_us / 1000000;
    const auto prescale_ratio = ticks_per_period / 65536 + 1;

    hwtimer_->CR1 = 0;

    hwtimer_->PSC = prescale_ratio - 1;
    hwtimer_->ARR = ticks_per_period / prescale_ratio;
    counts_per_cycle_ = hwtimer_->ARR;

    hwtimer_->SMCR = 0;
    hwtimer_->CCMR1 = 0;
    hwtimer_->CCMR2 = 0;
    hwtimer_->CCER = 0;
    hwtimer_->EGR = 0;
  }

  void Start() {
    hwtimer_->CR1 = (TIM_CR1_CEN);
  }

  void Stop() {
    hwtimer_->CR1 = 0;
  }

  TIM_TypeDef* hwtimer() const {
    return hwtimer_;
  }

  uint32_t counts_per_cycle() const { return counts_per_cycle_; }

 private:
  TIM_TypeDef* const hwtimer_;
  uint32_t counts_per_cycle_ = 0;
};

/// This keeps track of all timers currently active on a given aux
/// port that are used for PWM purposes.
class Stm32PwmTimerSet {
 public:
  void reset() {
    for (auto& item : timers_) {
      if (!!item) {
        item->Stop();
      }
      item.reset();
    }
  }

  Stm32PwmTimer* get(TIM_TypeDef* timer, int period_us) {
    for (auto& item : timers_) {
      if (!!item && item->hwtimer() == timer) {
        return &*item;
      } else if (!item) {
        item.emplace(timer, period_us);
        return &*item;
      }
    }
    MJ_ASSERT(false);
    return nullptr;
  }

  void Start() {
    for (auto& item : timers_) {
      if (!!item) {
        item->Start();
      }
    }
  }

 private:
  std::array<std::optional<Stm32PwmTimer>, 2> timers_;
};

/// This manages a single PWM pin.  Before construction, it assumes
/// that the only thing that has happened since the Stm32PwmTimer was
/// constructed is that other instances of non-conflicting Stm32Pwm
/// pins have been created.
class Stm32Pwm {
 public:
  Stm32Pwm(PinName pin,
           const Stm32PwmTimer* timer)
      : counts_per_cycle_(timer->counts_per_cycle()) {

    const auto pin_alt = FindTimerAlt(pin, timer->hwtimer());

    pinmap_pinout(pin_alt, PinMap_PWM);

    const auto hwtimer = timer->hwtimer();

    // Figure out which TIM_CHx we are
    const auto function = pinmap_function(pin_alt, PinMap_PWM);
    const auto channel = STM_PIN_CHANNEL(function);

    const auto output_compare_mode = 6; // PWM mode 1

    // Channels 2 and 4 are shifted in their ccmr register.
    const auto ccmr = (output_compare_mode << TIM_CCMR1_OC1M_Pos)
        << (((channel % 2) == 0) ? 8 : 0);

    if (channel == 1 || channel == 2) {
      hwtimer->CCMR1 |= ccmr;
    } else if (channel == 3 || channel == 4) {
      hwtimer->CCMR2 |= ccmr;
    }

    // Configure the output enable for the output comparator.
    hwtimer->CCER |= (1 << ((channel - 1) * 4));

    ocr_ = [&]() {
      switch (channel) {
        case 1: { return &hwtimer->CCR1; }
        case 2: { return &hwtimer->CCR2; }
        case 3: { return &hwtimer->CCR3; }
        case 4: { return &hwtimer->CCR4; }
      }
      MJ_ASSERT(false);
      return &hwtimer->CCR1;
    }();
  }

  void write(float value) {
    // For now, this is formulated as "truncate", so you have to have
    // exactly 1.0 as an output to get continuously on, whereas there
    // is a small range of values that give completely off.
    const uint16_t int_value =
        std::max(
            0, std::min(
                65535,
                static_cast<int>(value * (counts_per_cycle_ + 1))));
    *ocr_ = int_value;
  }

  const uint32_t counts_per_cycle_;
  volatile uint32_t* ocr_ = nullptr;
};

enum RequireCs {
  kRequireCs,
  kDoNotRequireCs,
};

template <typename PinArray>
std::optional<SpiPinOption> FindSpiOption(const PinArray& pin_array,
                                          size_t array_size,
                                          const AuxHardwareConfig& hw_config,
                                          RequireCs require_cs) {
  SpiPinOption result;

  // Figure out if appropriate pins are configured.
  int cs_count = 0;
  for (size_t i = 0; i < array_size; i++) {
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
                                            size_t array_size,
                                            const AuxHardwareConfig& hw_config) {
  UartPinOption result;

  for (size_t i = 0; i < array_size; i++) {
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
