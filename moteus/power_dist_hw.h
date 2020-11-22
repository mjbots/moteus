// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

// The most recent version of the HW.
#ifndef POWER_DIST_HW_REV
// r2 silk
#define POWER_DIST_HW_REV 0
#endif

#define DEBUG_LED1 PA_4
#define DEBUG_LED2 PA_5

#define FET_PRECHARGE PA_0
#define FET_MAIN PA_1
#define PWR_LED PA_2
#define PWR_SW PA_3

#define VSAMP_IN PA_9
#define VSAMP_OUT PA_8

#define CAN_TX PA_12
#define CAN_RX PA_11
#define CAN_SHDN PA_10

// Only reworked boards r2/r3/r3.1 boards have this divider, but
// non-reworked versions have an older firmware installed.
#define VSAMP_DIVIDE (4.7f / (10.0f + 100.0f))
