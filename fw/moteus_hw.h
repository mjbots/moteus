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

namespace moteus {

// The measured version of MOTEUS_HW_REV
extern volatile uint8_t g_measured_hw_rev;

// r1 silk
// #define MOTEUS_HW_REV 0

// r2 silk
// #define MOTEUS_HW_REV 1

// r3 silk
// #define MOTEUS_HW_REV 2

// r4.1 silk
// #define MOTEUS_HW_REV 3

// r4.2 and r4.3 silk
// #define MOTEUS_HW_REV 4

// r4.4 silk
// #define MOTEUS_HW_REV 5

// r4.5 silk
// #define MOTEUS_HW_REV 6

// The most recent version of the HW.
#ifndef MOTEUS_HW_REV
// r4.5b-r4.8 silk
#define MOTEUS_HW_REV 7
#endif

// The mapping between MOTEUS_HW_REV and the version pins on the
// board.
#if defined(TARGET_STM32G4)
constexpr int kHardwareInterlock[] = {
  -1,  // r1 (never printed for g4)
  -1,  // r2 (never printed for g4)
  -1,  // r3 (never printed for g4)
  0,   // r4.1
  0,   // r4.2/r4.3 (unfortunately, indistinguishable from the interlock)
  1,   // r4.4
  2,   // r4.5
  3,   // r4.5b
};
#else
constexpr int kHardwareInterlock[] = {
  0,   // r1
  1,   // r2
  2,   // r3 & r3.1
  -1,  // never printed for f4
  -1,  // never printed for f4
  -1,  // never printed for f4
  -1,  // never printed for f4
  -1,  // never printed for f4
};
#endif

// This firmware is compatible with the following hardware revisions.
constexpr int kCompatibleHwRev[] = {
  // 3 isn't compatible, but we forgot to rev the version pins
  3,
  4, 5,
  6, 7
};

#define DRV8323_ENABLE PA_3

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#else
#define DRV8323_HIZ PB_7
#endif

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#elif MOTEUS_HW_REV >= 3
#define DRV8323_CS PC_4
#endif

#define DRV8323_MOSI PA_7
#define DRV8323_MISO PA_6
#define DRV8323_SCK PA_5

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#elif MOTEUS_HW_REV >= 3
#define DRV8323_FAULT PB_6
#endif

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#elif MOTEUS_HW_REV >= 3
#define DEBUG_LED1 PF_0
#define POWER_LED PF_1
#endif

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#elif MOTEUS_HW_REV >= 3
#define MOTEUS_DEBUG_UART_OUT PB_3
#endif

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#elif MOTEUS_HW_REV >= 3
#define HWREV_PIN0 PC_6
#define HWREV_PIN1 PA_15
// Previously this was documented as PC_13, however we never pulled it
// down, and decided to use PC_13 for something else.
#define HWREV_PIN2 PA_10
#endif

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#elif MOTEUS_HW_REV >= 3
// We've picked these particular pins so that all 3 channels are one
// of the "slow" channels so they will have similar analog performance
// characteristics.

// ADC3
#define MOTEUS_CURRENT1 PB_0_ALT0
// ADC1
#define MOTEUS_CURRENT2 PB_1
// ADC2
#define MOTEUS_CURRENT3 PB_2
#endif

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#elif MOTEUS_HW_REV == 3
#define MOTEUS_VSENSE PA_8
#elif MOTEUS_HW_REV >= 4
// Here, the vsense does depend on the hardware version, but it is
// detected at runtime between PA_8 (r4) and PB_12 (r5+)
#define MOTEUS_VSENSE PA_8
#define MOTEUS_VSENSE_5_AND_LATER PB_12_ALT0
#endif

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#elif MOTEUS_HW_REV >= 3
#define MOTEUS_TSENSE PA_9
#endif

#if MOTEUS_HW_REV <= 3
#define MOTEUS_MSENSE NC
#elif MOTEUS_HW_REV >= 4
// Here, msense does depend on the hardware version, but it is
// detected at runtime between PA_8 and PB_12.
#define MOTEUS_MSENSE PB_12
#define MOTEUS_MSENSE_5_AND_LATER PA_8
#endif

#ifndef MOTEUS_CURRENT_SENSE_OHM
#if MOTEUS_HW_REV <= 1
#error "Not supported"
#elif MOTEUS_HW_REV >= 2
#define MOTEUS_CURRENT_SENSE_OHM 0.0005f
#endif
#endif

#ifndef MOTEUS_VSENSE_ADC_SCALE
#define MOTEUS_VSENSE_ADC_SCALE_PRE6 0.00884f
#define MOTEUS_VSENSE_ADC_SCALE_POST6 0.017947f
#endif

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#else
#define MOTEUS_UART_TX PC_10
#define MOTEUS_UART_RX PC_11
#define MOTEUS_UART_DIR NC
#endif

#define MOTEUS_AS5047_MOSI PB_15
#define MOTEUS_AS5047_MISO PB_14
#define MOTEUS_AS5047_SCK PB_13

#if MOTEUS_HW_REV <= 2
#error "Not suppported"
#elif MOTEUS_HW_REV >= 3
#define MOTEUS_AS5047_CS PB_11
#endif

#define MOTEUS_EXTERNAL_ENCODER_CS PC_13

#if MOTEUS_HW_REV >= 3
#define MOTEUS_CAN_TD PA_12
#define MOTEUS_CAN_RD PA_11
#endif

#if MOTEUS_HW_REV <= 2
#error "Not supported"
#elif MOTEUS_HW_REV >= 3
#define MOTEUS_DEBUG1 PC_14
#define MOTEUS_DEBUG2 PC_15
#endif

#define MOTEUS_DEBUG_DAC PA_4

#define MOTEUS_ABS_SCL PB_8
#define MOTEUS_ABS_SDA PB_9

#if defined(TARGET_STM32G4)
#define MOTEUS_CCM_ATTRIBUTE __attribute__ ((section (".ccmram")))
#else
#error "Unknown target"
#endif


#define MOTEUS_MODEL_NUMBER ((MOTEUS_HW_REV) << 8 | 0x00)
#define MOTEUS_FIRMWARE_VERSION 0x000103

// Version history:

// # 0x0101 #
//
// * Fixed the calculation of feedforward voltage to have the correct
//   sign for the velocity component.  Previous firmwares,
//   inappropriately applied a negative feedforward term, which
//   counteracted rotation instead of being an actual feedforward.

// # 0x0102 #
//
// * Removed servo.feedforward_scale entirely

// # 0x0103 #
//
// * Added servo.pwm_scale, and for r4.8 boards changed the default
//   value of pwm_comp_off / pwm_comp_mag.

}
