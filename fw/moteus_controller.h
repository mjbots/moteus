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

#include "mjlib/micro/pool_ptr.h"
#include "mjlib/multiplex/micro_server.h"

#include "fw/as5047.h"
#include "fw/bldc_servo.h"
#include "fw/drv8323.h"
#include "fw/firmware_info.h"
#include "fw/millisecond_timer.h"

namespace moteus {

/// Glues together the various pieces of hardware that make a moteus
/// controller board.
class MoteusController {
 public:
  MoteusController(mjlib::micro::Pool*,
                   mjlib::micro::PersistentConfig* config,
                   mjlib::micro::TelemetryManager* telemetry_manager,
                   MillisecondTimer*,
                   FirmwareInfo*);
  ~MoteusController();

  void Start();
  void Poll();
  void PollMillisecond();

  AS5047* as5047();
  Drv8323* drv8323();
  BldcServo* bldc_servo();

  mjlib::multiplex::MicroServer::Server* multiplex_server();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};
}
