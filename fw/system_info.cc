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

#include "fw/system_info.h"

#include "mbed.h"

#include "mjlib/base/inplace_function.h"
#include "mjlib/base/visitor.h"

#include "mjlib/micro/telemetry_manager.h"

namespace moteus {

volatile uint32_t SystemInfo::idle_count = 0;

namespace {
struct SystemInfoData {
  uint32_t pool_size = 0;
  uint32_t pool_available = 0;

  uint32_t idle_rate = 0;
  uint32_t can_reset_count = 0;

  // We deliberately start this counter near to int32 overflow so that
  // any applications that use it will likely have to handle it
  // properly.
  uint32_t ms_count = (1ull<<31) - 300000;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(pool_size));
    a->Visit(MJ_NVP(pool_available));
    a->Visit(MJ_NVP(idle_rate));
    a->Visit(MJ_NVP(can_reset_count));
    a->Visit(MJ_NVP(ms_count));
  }
};
}

class SystemInfo::Impl {
 public:
  Impl(mjlib::micro::Pool& pool, mjlib::micro::TelemetryManager& telemetry)
      : pool_(pool) {
    data_updater_ = telemetry.Register("system_info", &data_);
  }

  void PollMillsecond() {
    data_.ms_count++;
    ms_count_++;
    if (ms_count_ >= 10) {
      ms_count_ = 0;
    } else {
      return;
    }

    data_.pool_size = pool_.size();
    data_.pool_available = pool_.available();

    const auto this_idle_count = idle_count;
    data_.idle_rate = this_idle_count - last_idle_count_;
    last_idle_count_ = this_idle_count;

    data_updater_();
  }

  void SetCanResetCount(uint32_t value) {
    data_.can_reset_count = value;
  }

  mjlib::micro::Pool& pool_;

  uint8_t ms_count_ = 0;
  uint32_t last_idle_count_ = 0;
  SystemInfoData data_;
  mjlib::base::inplace_function<void ()> data_updater_;
};

SystemInfo::SystemInfo(mjlib::micro::Pool& pool,
                       mjlib::micro::TelemetryManager& telemetry)
    : impl_(&pool, pool, telemetry) {}

SystemInfo::~SystemInfo() {}

void SystemInfo::PollMillisecond() {
  impl_->PollMillsecond();
}

void SystemInfo::SetCanResetCount(uint32_t value) {
  impl_->SetCanResetCount(value);
}

uint32_t SystemInfo::millisecond_counter() const {
  return impl_->data_.ms_count;
}

}
