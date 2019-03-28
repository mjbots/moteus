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
#ifndef IMU_JUNCTION_HW_REV
// r1 silk
#define IMU_JUNCTION_HW_REV 0
#endif

#define DEBUG_LED1 PB_12
#define DEBUG_LED2 PB_13


#define IMU_MOSI PB_5
#define IMU_MISO PB_4
#define IMU_SCK PB_3

#define IMU_ACCEL_CS PB_6
#define IMU_GYRO_CS PB_7
#define IMU_ACCEL_INT PC_14
#define IMU_GYRO_INT PC_13

#define FAN1 PA_6
#define FAN2 PA_7
