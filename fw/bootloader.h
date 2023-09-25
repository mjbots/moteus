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

#include "fw/stm32.h"

extern "C" {
/// This function initiates the multiplex server bootloader.  It takes
/// over control of the communications hardware, and communicates over
/// it using the multiplex server async stream protocol.  It
/// implements a minimal command set to support re-flashing of the
/// application portion of the onboard flash.
///
/// This function will never return, as it leaves the hardware in an
/// undefined state.  It may internally cause the device to hard-reset
/// in order to restart the primary application.
///
/// It is intended to be placed into an alternate section of flash
/// from the main application, so that it can continue to execute
/// while the primary application is re-programmed.
///
/// Assumed hardware configuration:
///   The UART is already powered and configured to the proper baud rate.
///   TIM5 is configured to be a microsecond timer.
///
/// NOTE: All interrupts are disabled shortly after starting this
/// function.  Any actions required to safe a moving system should be
/// taken before calling this.
void MultiplexBootloader(uint8_t source_id,
                         USART_TypeDef* uart,
                         GPIO_TypeDef* direction_port,
                         int direction_pin) __attribute__ ((noreturn));
}
