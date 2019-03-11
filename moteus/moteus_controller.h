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

#include "mjlib/micro/pool_ptr.h"

#include "moteus/as5047.h"
#include "moteus/bldc_servo.h"
#include "moteus/drv8323.h"
#include "moteus/millisecond_timer.h"

namespace moteus {

/// Glues together the various pieces of hardware that make a moteus
/// controller board.
class MoteusController {
 public:
  MoteusController(mjlib::micro::Pool*,
                   mjlib::micro::PersistentConfig* config,
                   mjlib::micro::TelemetryManager* telemetry_manager,
                   MillisecondTimer*);
  ~MoteusController();

  void Start();
  void PollMillisecond();

  AS5047* as5047();
  Drv8323* drv8323();
  BldcServo* bldc_servo();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};
}
