// Copyright 2021 Josh Pieper, jjp@pobox.com.
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

#include "fw/abs_port.h"

#include "fw/stm32_i2c.h"

namespace micro = mjlib::micro;

namespace {
const uint8_t AS5048_REG_AGC = 0xFA;
const uint8_t AS5048_REG_DIAG = 0xFB;
const uint8_t AS5048_REG_MAG_HIGH = 0xFC;
const uint8_t AS5048_REG_MAG_LOW = 0xFD;
const uint8_t AS5048_REG_ANGLE_HIGH = 0xFE;
const uint8_t AS5048_REG_ANGLE_LOW = 0xFF;
}

namespace moteus {

class AbsPort::Impl {
 public:
  Impl(micro::PersistentConfig* config,
       micro::TelemetryManager* telemetry_manager,
       MillisecondTimer* timer,
       const Options& options)
      : timer_(timer),
        options_(options) {
    config->Register("abs_port", &config_,
                     std::bind(&Impl::HandleConfigUpdate, this));
    telemetry_manager->Register("abs_port", &status_);
  }

  void PollMillisecond() {
    switch (config_.mode) {
      case kDisabled: {
        status_.encoder_valid = false;
        break;
      }
      case kAs5048: {
        // We don't read for the first 10ms of operation, since the
        // AS5048 documents that long of a turn on time before valid
        // results are produced.
        if (!status_.encoder_valid && timer_->read_ms() < 10) {
          break;
        }

        encoder_count_++;
        if (encoder_count_ >= std::max<int32_t>(5, config_.encoder_poll_ms)) {
          encoder_count_ = 0;

          StartAs5048Read();
        }
        break;
      }
      case kNumModes: {
        break;
      }
    }
  }

  void Poll() {
    if (i2c_) {
      i2c_->Poll();
      const auto read_status = i2c_->CheckRead();

      if (read_status == Stm32I2c::ReadStatus::kComplete) {
        // We got new data, publish it.
        status_.encoder_raw =
            (encoder_raw_data_[4] << 8) |
            (encoder_raw_data_[5]);
        status_.encoder_valid = true;

        status_.as5048_agc = encoder_raw_data_[0];
        status_.as5048_diag = encoder_raw_data_[1];
        status_.as5048_mag =
            (encoder_raw_data_[2] << 8) |
            (encoder_raw_data_[3]);

        status_.position =
            static_cast<float>(
                static_cast<int16_t>(
                    status_.encoder_raw +
                    config_.position_offset)) /
            65536.0f *
            config_.position_scale;
      } else if (read_status == Stm32I2c::ReadStatus::kError) {
        status_.as5048_error_count++;
      }
    }
  }

  void StartAs5048Read() {
    MJ_ASSERT(!!i2c_);
    i2c_->StartReadMemory(
        config_.encoder_i2c_address,
        AS5048_REG_AGC,
        mjlib::base::string_span(
            reinterpret_cast<char*>(&encoder_raw_data_[0]),
            sizeof(encoder_raw_data_)));
  }

  void HandleConfigUpdate() {
    switch (config_.mode) {
      case kDisabled: {
        i2c_.reset();
        break;
      }
      case kAs5048: {
        i2c_.emplace(
            [&]() {
              Stm32I2c::Options options;
              options.sda = options_.sda;
              options.scl = options_.scl;
              options.frequency = config_.i2c_hz;
              options.i2c_mode = static_cast<I2cMode>(config_.i2c_mode);
              return options;
            }());
        break;
      }
      case kNumModes: {
        break;
      }
    }
  }

  Config config_;
  Status status_;

  MillisecondTimer* const timer_;
  const Options options_;

  std::optional<Stm32I2c> i2c_;
  int32_t encoder_count_ = 0;
  uint8_t encoder_raw_data_[6] = {};
};

AbsPort::AbsPort(micro::Pool* pool,
                 micro::PersistentConfig* persistent_config,
                 micro::TelemetryManager* telemetry_manager,
                 MillisecondTimer* timer,
                 const Options& options)
    : impl_(pool, persistent_config, telemetry_manager, timer, options) {}

AbsPort::~AbsPort() {}

void AbsPort::PollMillisecond() { impl_->PollMillisecond(); }
void AbsPort::Poll() { impl_->Poll(); }

const AbsPort::Status& AbsPort::status() const {
  return impl_->status_;
}

}  // namespace moteus
