/*

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



Our memory layout looks like the following:

0x8000000/16kb - ISR
  * This has to be located at the beginning address.  Currently we
    store nothing in the remainder of the beginning 16kb, although it
    might be possible in the future to store the bootloader here given
    that our application never changes the ISR vectors.

0x8004000/16kb - currently unused
0x8008000/16kb - currently unused

0x800c000/16kb - bootloader
  * This is minimal software which relies on the main application to
    get everything set up, but once there, it can receive a new
    firmware image over the RS485 protocol.

0x8010000 - application

0x807f000 - persistent settings

*/

MEMORY
{
  FLASH_ISR (rx) : ORIGIN = 0x8000000, LENGTH = 0x004000
}

MultiplexBootloader = 0x800c000;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH_ISR
}

/* And now delegate everything else to the regular mbed linker script */
INCLUDE external/com_github_ARMmbed_mbed-g4/linker_script.ld
