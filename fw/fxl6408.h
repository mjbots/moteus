// Copyright 2026 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#include <algorithm>
#include <cstdint>

#include "mbed.h"

#include "fw/millisecond_timer.h"

namespace moteus {

// Synchronous bit-banged I2C driver for the ONsemi FXL6408 8-bit I/O
// expander.  All transactions block the caller; this is only intended
// to be invoked from main-loop context (e.g. AuxPort::HandleConfigUpdate),
// not from any ISR.
//
// Usage: callers stage their desired direction/state changes with
// SetAsOutput()/SetAsInput(), then call Apply() once to flush all
// pending changes to the device in a single I2C burst.
class Fxl6408 {
 public:
  struct Options {
    PinName sda = NC;
    PinName scl = NC;
    // 7-bit I2C address.  FXL6408 ADDR tied low -> 0x43, tied high -> 0x44.
    uint8_t i2c_address = 0x43;
    int frequency_hz = 100000;
  };

  Fxl6408(MillisecondTimer* timer, const Options& options)
      : timer_(timer),
        options_(options) {
    InitPin(options.sda, &sda_);
    InitPin(options.scl, &scl_);

    // Release both lines (open-drain high) so the bus idles correctly
    // even before any transaction is issued.
    SdaRelease();
    SclRelease();

    // Ceiling division so we never run faster than frequency_hz.
    us_delay_ = std::max<uint32_t>(
        1u, (500000u + options.frequency_hz - 1) / options.frequency_hz);
  }

  // Stage bit `bit` (0..7) as a driven output with the given value.
  // No I2C activity occurs until Apply() is called.
  void SetAsOutput(int bit, bool value) {
    const uint8_t mask = static_cast<uint8_t>(1u << bit);
    staged_dir_ |= mask;
    if (value) {
      staged_state_ |= mask;
    } else {
      staged_state_ &= ~mask;
    }
  }

  // Stage bit `bit` (0..7) as a high-impedance input.  No I2C
  // activity occurs until Apply() is called.
  void SetAsInput(int bit) {
    const uint8_t mask = static_cast<uint8_t>(1u << bit);
    staged_dir_ &= ~mask;
    staged_state_ &= ~mask;
  }

  // Flush any pending direction or state changes to the device.  On
  // the first call, also performs one-time initialization (sets all
  // outputs to "driven" rather than high-Z).  No-op if nothing has
  // changed since the last Apply().
  void Apply() {
    // FXL6408 register map:
    //   0x03  I/O Direction      (1 = output, 0 = input)
    //   0x05  Output State
    //   0x07  Output High-Z      (1 = high-Z, 0 = driven)
    if (!initialized_) {
      WriteReg(0x07, 0x00);
      WriteReg(0x03, staged_dir_);
      WriteReg(0x05, staged_state_);
      device_dir_ = staged_dir_;
      device_state_ = staged_state_;
      initialized_ = true;
      return;
    }
    if (staged_dir_ != device_dir_) {
      WriteReg(0x03, staged_dir_);
      device_dir_ = staged_dir_;
    }
    if (staged_state_ != device_state_) {
      WriteReg(0x05, staged_state_);
      device_state_ = staged_state_;
    }
  }

 private:
  struct Pin {
    volatile uint32_t* bsrr = nullptr;
    volatile uint32_t* brr = nullptr;
    volatile uint32_t* idr = nullptr;
    uint32_t mask = 0;
  };

  static void InitPin(PinName pin, Pin* out) {
    const uint32_t port_index1 = STM_PORT(pin);
    GPIO_TypeDef* gpio = reinterpret_cast<GPIO_TypeDef*>([&]() {
      switch (port_index1) {
        case PortA: return GPIOA_BASE;
        case PortB: return GPIOB_BASE;
        case PortC: return GPIOC_BASE;
        case PortD: return GPIOD_BASE;
        case PortE: return GPIOE_BASE;
        case PortF: return GPIOF_BASE;
      }
      MJ_ASSERT(false);
      return GPIOA_BASE;
    }());
    const uint32_t pin_offset = (static_cast<uint32_t>(pin) & 0xf);

    // We program the GPIO registers directly below.

    out->bsrr = &gpio->BSRR;
    out->brr = &gpio->BRR;
    out->idr = &gpio->IDR;
    out->mask = 1u << pin_offset;

    // Drive the pin high before switching to output mode so we don't
    // glitch the bus low.
    gpio->BSRR = out->mask;

    // Open-drain output: OTYPER=1 (open drain), MODER=01 (output),
    // PUPDR=01 (internal pull-up).  The moteus-r5 has no external
    // pull-ups on the expander I2C bus, so the STM32's internal
    // pull-ups supply the bus bias.
    gpio->OTYPER |= out->mask;
    const uint32_t pupdr_mask = 0x3u << (pin_offset * 2);
    gpio->PUPDR = (gpio->PUPDR & ~pupdr_mask) | (0x1u << (pin_offset * 2));
    const uint32_t mode_mask = 0x3u << (pin_offset * 2);
    gpio->MODER = (gpio->MODER & ~mode_mask) | (0x1u << (pin_offset * 2));
  }

  void SdaRelease() { *sda_.bsrr = sda_.mask; }
  void SdaDrive()   { *sda_.brr  = sda_.mask; }
  void SclRelease() { *scl_.bsrr = scl_.mask; }
  void SclDrive()   { *scl_.brr  = scl_.mask; }
  bool SdaRead()    { return (*sda_.idr & sda_.mask) != 0; }

  void HalfBit() { timer_->wait_us(us_delay_); }

  void Start() {
    SdaRelease();
    SclRelease();
    HalfBit();
    SdaDrive();
    HalfBit();
    SclDrive();
    HalfBit();
  }

  void Stop() {
    SdaDrive();
    HalfBit();
    SclRelease();
    HalfBit();
    SdaRelease();
    HalfBit();
  }

  bool WriteByte(uint8_t value) {
    for (int i = 7; i >= 0; i--) {
      if ((value >> i) & 1) {
        SdaRelease();
      } else {
        SdaDrive();
      }
      HalfBit();
      SclRelease();
      HalfBit();
      SclDrive();
      HalfBit();
    }
    // 9th clock: release SDA, sample for ACK.
    SdaRelease();
    HalfBit();
    SclRelease();
    HalfBit();
    const bool ack = !SdaRead();
    SclDrive();
    HalfBit();
    return ack;
  }

  void WriteReg(uint8_t reg, uint8_t value) {
    Start();
    const bool a1 = WriteByte(static_cast<uint8_t>(options_.i2c_address << 1));
    const bool a2 = WriteByte(reg);
    const bool a3 = WriteByte(value);
    Stop();
    // The expander is on-board; a missing ACK is a hardware fault.
    MJ_ASSERT(a1 && a2 && a3);
  }

  MillisecondTimer* const timer_;
  const Options options_;
  uint32_t us_delay_ = 1;

  Pin sda_;
  Pin scl_;

  // Caller-visible staged state (modified by SetAs*).
  uint8_t staged_dir_ = 0;
  uint8_t staged_state_ = 0;

  // Last values written to the device.
  uint8_t device_dir_ = 0;
  uint8_t device_state_ = 0;
  bool initialized_ = false;
};

}  // namespace moteus
