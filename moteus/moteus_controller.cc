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

#include "moteus/moteus_controller.h"

#include "moteus/moteus_hw.h"

namespace micro = mjlib::micro;

namespace moteus {

class MoteusController::Impl {
 public:
  Impl(micro::Pool* pool,
       micro::PersistentConfig* persistent_config,
       micro::TelemetryManager* telemetry_manager,
       MillisecondTimer* timer)
      : as5047_([]() {
          AS5047::Options options;
          options.mosi = PB_15;
          options.miso = PB_14;
          options.sck = PB_13;
          options.cs = PB_12;
          return options;
        }()),
        drv8323_(pool, persistent_config, telemetry_manager, timer, []() {
            Drv8323::Options options;
            options.mosi = DRV8323_MOSI;
            options.miso = DRV8323_MISO;
            options.sck = DRV8323_SCK;
            options.cs = DRV8323_CS;
            options.enable = DRV8323_ENABLE;
            options.fault = DRV8323_FAULT;
            options.hiz = DRV8323_HIZ;
            return options;
          }()),
        bldc_(pool, persistent_config, telemetry_manager, &as5047_, &drv8323_, []() {
            BldcServo::Options options;
            options.pwm1 = PA_0;
            options.pwm2 = PA_1;
            options.pwm3 = PA_2;

            options.current1 = PC_5;
            options.current2 = PB_0_ALT0;
            options.vsense = MOTEUS_VSENSE;
            options.tsense = MOTEUS_TSENSE;

            options.debug_out = PB_8;
            options.debug_uart_out = PC_10;

            return options;
          }()) {}

  void Start() {
    bldc_.Start();
  }

  void PollMillisecond() {
    drv8323_.PollMillisecond();
    bldc_.PollMillisecond();
  }

  AS5047 as5047_;
  Drv8323 drv8323_;
  BldcServo bldc_;
};

MoteusController::MoteusController(micro::Pool* pool,
                                   micro::PersistentConfig* persistent_config,
                                   micro::TelemetryManager* telemetry_manager,
                                   MillisecondTimer* timer)
    : impl_(pool, pool, persistent_config, telemetry_manager, timer) {}

MoteusController::~MoteusController() {}

void MoteusController::Start() {
  impl_->Start();
}

void MoteusController::PollMillisecond() {
  impl_->PollMillisecond();
}

AS5047* MoteusController::as5047() {
  return &impl_->as5047_;
}

Drv8323* MoteusController::drv8323() {
  return &impl_->drv8323_;
}

BldcServo* MoteusController::bldc_servo() {
  return &impl_->bldc_;
}

}
