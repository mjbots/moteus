// Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

// r1 silk
// #define MOTEUS_HW_REV 0

// The most recent version of the HW.
#ifndef MOTEUS_HW_REV
// r2 silk
#define MOTEUS_HW_REV 1
#endif

#define DRV8323_ENABLE PA_3
#define DRV8323_HIZ PC_3

#if MOTEUS_HW_REV == 0
#define DRV8323_CS PA_4
#elif MOTEUS_HW_REV >= 1
#define DRV8323_CS PC_8
#endif

#define DRV8323_MOSI PA_7
#define DRV8323_MISO PA_6
#define DRV8323_SCK PA_5
#define DRV8323_FAULT PC_4

#define DEBUG_LED1 PA_11
#define DEBUG_LED2 PA_12


#define HWREV_PIN0 PC_13
#define HWREV_PIN1 PC_14
#define HWREV_PIN2 PC_15

#define MOTEUS_VSENSE PC_1_ALT1

#if MOTEUS_HW_REV == 0
#define MOTEUS_TSENSE NC
#elif MOTEUS_HW_REV >= 1
#define MOTEUS_TSENSE PC_2_ALT1
#endif
