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

#if defined(TARGET_STM32G4)
#define MOTEUS_CCM_ATTRIBUTE __attribute__ ((section (".ccmram")))
// For CCM functions which should remain out of line regardless of
// the inlining budget GCC has left in a given translation unit, so
// that CCM usage does not depend upon unrelated code.
#define MOTEUS_CCM_NOINLINE_ATTRIBUTE \
  __attribute__ ((section (".ccmram"), noinline))
#else
#define MOTEUS_CCM_ATTRIBUTE
#define MOTEUS_CCM_NOINLINE_ATTRIBUTE
#endif
