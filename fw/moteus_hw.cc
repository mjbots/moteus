// Copyright 2018-2022 Josh Pieper, jjp@pobox.com.
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

#include "fw/moteus_hw.h"

#include "mjlib/base/assert.h"

#include "fw/stm32_bitbang_spi.h"

namespace moteus {

namespace {
//////////////////////////////////////////
// moteus - family 0

// The following silks correspond with the following hardware
// revisions:
//
//  "r1"           - 0
//  "r2"           - 1
//  "r3"           - 2
//  "r4.1"         - 3
//  "r4.2", "r4.3" - 4
//  "r4.4"         - 5
//  "r4.5"         - 6
//  "r4.5b"-"r4.8" - 7
//  "r4.11"        - 8

// The mapping between  and the version pins on the
// board.
constexpr int kFamily0HardwareInterlock[] = {
  -1,  // r1 (never printed for g4)
  -1,  // r2 (never printed for g4)
  -1,  // r3 (never printed for g4)
  0,   // r4.1
  0,   // r4.2/r4.3 (unfortunately, indistinguishable from the interlock)
  1,   // r4.4
  2,   // r4.5
  3,   // r4.5b-r4.8
  4,   // r4.10
};

}

FamilyAndVersion DetectMoteusFamily(MillisecondTimer* timer) {
  timer->wait_ms(2);

  FamilyAndVersion result;
  result.family = 0;

  // We check for family 1, "moteus hp", by seeing if we can find a
  // DRV8323 on a chip select that is different from that used on all
  // family 0 boards.
  {
    // Ensure that on family 0 boards, the drv8323 will be not
    // selected.
    DigitalOut family0_drv8323_cs(PC_4, 1);
    DigitalOut drv8323_family1_enable(PC_14, 1);

    // Wait 1ms after enabling.
    timer->wait_us(1000);

    Stm32BitbangSpi maybe_drv8323(
        timer,
        [&]() {
          Stm32BitbangSpi::Options out;
          out.mosi = PC_13;
          out.miso = PC_11;
          out.sck = PC_10;
          out.cs = PB_0;

          // We can use a slow speed since this is just a one-time
          // test.
          out.frequency = 500000;
          return out;
        }());
    pin_mode(PC_11, PullUp);
    auto read_reg =
        [&](int reg) {
          timer->wait_us(1);
          return maybe_drv8323.write(0x8000 | (reg << 11)) & 0x7ff;
        };
    bool found = false;
    for (int reg = 2; reg < 6; reg++) {
      const auto value = read_reg(reg);
      if (value != 0x7ff) {
        found = true;
        break;
      }
    }

    if (found) {
      result.family = 1;
    }
  }


  if (result.family == 0) {
    DigitalIn hwrev0(PC_6, PullUp);
    DigitalIn hwrev1(PA_15, PullUp);
    // Previously this was documented as PC_13, however we never
    // pulled it down, and decided to use PC_13 for something else.
    DigitalIn hwrev2(PA_10, PullUp);

    const uint8_t this_hw_pins =
        0x07 & (~(hwrev0.read() |
                  (hwrev1.read() << 1) |
                  (hwrev2.read() << 2)));
    const uint8_t measured_hw_rev =
        [&]() {
          int i = 0;
          for (auto rev_pins : kFamily0HardwareInterlock) {
            if (rev_pins == this_hw_pins) { return i; }
            i++;
          }
          return -1;
        }();
    result.hw_version = measured_hw_rev;
  } else if (result.family == 1) {
    // TODO: Verify this actually reads other values.  I think
    // AnalogIn may not work on the G4 in my hacked version.
    AnalogIn board_rev(PA_4);
    const uint16_t this_reading = board_rev.read_u16();
    if (this_reading < 0x1000) {
      result.hw_version = 0;
    } else {
      mbed_die();
    }
  } else {
    MJ_ASSERT(false);
  }

  return result;
}

namespace {
PinName unsupported() {
  mbed_die();
  return NC;
}
}

MoteusHwPins FindHardwarePins(FamilyAndVersion fv) {
  MoteusHwPins result;

  const auto hv = fv.hw_version;

  if (fv.family == 0) {
    result.vsense =
        (hv <= 4 ? PA_8 :
         hv >= 5 ? PB_12_ALT0 :
         unsupported());

    result.msense =
        (hv <= 3 ? NC :
         hv == 4 ? PB_12 :
         hv >= 5 ? PA_8 :
         unsupported());

    result.vsense_adc_scale =
        (hv <= 5 ? 0.00884f : 0.017947f);

    result.drv8323_enable = PA_3;
    result.drv8323_hiz = PB_7;
    result.drv8323_cs = PC_4;
    result.drv8323_fault = PB_6;

    // We've picked these particular pins so that all 3 channels are
    // one of the "slow" channels so they will have similar analog
    // performance characteristics.
    result.current1 = PB_0_ALT0;
    result.current2 = PB_1;
    result.current3 = PB_2;

    result.as5047_cs = PB_11;

    result.debug1 = PC_14;
    result.debug2 = PC_15;

    result.debug_uart_out = PB_3;
  } else {
    result.drv8323_enable = PC_14;
    result.drv8323_hiz = PC_15;
    result.drv8323_cs = PB_0;
    result.drv8323_fault = PB_13;

    // Family 1 devices should have all current sense inputs on "fast"
    // channels.
    result.current1 = PA_3;       // ADC1
    result.current2 = PA_6;       // ADC2  // current 2
    result.current3 = PB_1_ALT0;  // ADC3  // current 3


    result.vsense = PA_9;         // ADC5_IN2
    result.tsense = PB_12_ALT0;   // ADC4
    result.msense = PA_8;         // ADC5_IN1

    result.vsense_adc_scale = 0.017947f;

    result.uart_tx = NC;
    result.uart_rx = NC;

    result.as5047_cs = PB_4;

    result.debug1 = NC;
    result.debug2 = NC;
  }

  return result;
}

}
