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

#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/multiplex/micro_server.h"

#include "fw/bldc_servo.h"
#include "fw/clock_manager.h"
#include "fw/firmware_info.h"
#include "fw/millisecond_timer.h"
#include "fw/system_info.h"

namespace moteus {

/// Glues together the various pieces of hardware that make a moteus
/// controller board.
class MoteusController {
 public:
  MoteusController(mjlib::micro::Pool*,
                   mjlib::micro::PersistentConfig* config,
                   mjlib::micro::CommandManager* command_manager,
                   mjlib::micro::TelemetryManager* telemetry_manager,
                   mjlib::multiplex::MicroServer* multiplex_protocol,
                   ClockManager*,
                   SystemInfo*,
                   MillisecondTimer*,
                   FirmwareInfo*);
  ~MoteusController();

  void Start();
  void Poll();
  void PollMillisecond();

  BldcServo* bldc_servo();

  mjlib::multiplex::MicroServer::Server* multiplex_server();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};
}
