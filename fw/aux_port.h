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

#include <array>

#include "mbed.h"
#include "PeripheralPins.h"

#include "mjlib/base/tokenizer.h"
#include "mjlib/base/visitor.h"
#include "mjlib/micro/async_stream.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/aksim2.h"
#include "fw/as5047.h"
#include "fw/aux_adc.h"
#include "fw/aux_common.h"
#include "fw/aux_mbed.h"
#include "fw/ccm.h"
#include "fw/cui_amt21.h"
#include "fw/cui_amt22.h"
#include "fw/ic_pz.h"
#include "fw/math.h"
#include "fw/ma732.h"
#include "fw/millisecond_timer.h"
#include "fw/moteus_hw.h"
#include "fw/stm32_i2c.h"
#include "fw/strtof.h"

namespace moteus {

class AuxPort {
 public:
  using Config = aux::AuxConfig;
  using Status = aux::AuxStatus;

  enum SpiDefault {
    kNoDefaultSpi,
    kDefaultOnboardSpi,
    kDefaultOnboardMa600,
  };

  AuxPort(const char* aux_name,
          const char* icpz_name,
          const aux::AuxHardwareConfig& hw_config,
          AuxADC::AuxInfo* adc_info,
          mjlib::micro::PersistentConfig* persistent_config,
          mjlib::micro::CommandManager* command_manager,
          mjlib::micro::TelemetryManager* telemetry_manager,
          mjlib::micro::AsyncStream* tunnel_stream,
          MillisecondTimer* timer,
          SpiDefault spi_default,
          std::array<DMA_Channel_TypeDef*, 4> dma_channels)
      : pin_count_(std::max_element(
                       hw_config.pins.begin(), hw_config.pins.end(),
                       [](const auto& a, const auto& b) {
                         return a.number < b.number; })->number + 1),
        tunnel_stream_(tunnel_stream),
        timer_(timer),
        adc_info_(*adc_info),
        hw_config_(hw_config),
        dma_channels_(dma_channels),
        spi_default_(spi_default) {
    persistent_config->Register(aux_name, &config_,
                                std::bind(&AuxPort::HandleConfigUpdate, this));
    telemetry_manager->Register(aux_name, &status_);

    // The ic_pz_ optional may not always be set, but space for it is
    // allocated regardless, so it should be harmless for
    // TelemetryManager to read bogus values in the event it is not
    // populated.
    telemetry_manager->Register(icpz_name, &ic_pz_->status_);

    command_manager->Register(
        aux_name, std::bind(&AuxPort::HandleCommand, this,
                            std::placeholders::_1, std::placeholders::_2));

    MJ_ASSERT(i2c_state_.size() == config_.i2c.devices.size());
    for (const auto& pin : hw_config_.pins) {
      if (pin.adc_num >= AuxADC::kMaxAdcs) {
        mbed_die();
      }
    }

    if (hw_config_.options.rs422_re != NC) {
      rs422_re_.emplace(hw_config_.options.rs422_re, 1);
    }
    if (hw_config_.options.rs422_de != NC) {
      rs422_de_.emplace(hw_config_.options.rs422_de, 0);
    }

    HandleConfigUpdate();

    StartTunnelRead();
  }

  void ISR_MaybeStartSample() MOTEUS_CCM_ATTRIBUTE {
    for (size_t i = 0;; i++) {
      const auto this_type = start_sample_types_[i];
      switch (this_type) {
        case SampleType::kAs5047: {
          as5047_->StartSample();
          break;
        }
        case SampleType::kMa732: {
          ma732_->StartSample();
          break;
        }
        case SampleType::kIcPz: {
          ic_pz_->ISR_StartSample();
          break;
        }
        case SampleType::kNone: {
          return;
        }
        default: {
          break;
        }
      }
    }
  }

  void ISR_MaybeFinishSample() MOTEUS_CCM_ATTRIBUTE {
    if (!any_isr_enabled_) { return; }

    for (size_t i = 0;; i++) {
      const auto this_type = finish_sample_types_[i];
      switch (this_type) {
        case SampleType::kAs5047: {
          status_.spi.active = true;
          status_.spi.value = as5047_->FinishSample();
          status_.spi.nonce += 1;
          break;
        }
        case SampleType::kMa732: {
          status_.spi.active = true;
          status_.spi.value = ma732_->FinishSample();
          status_.spi.nonce += 1;
          break;
        }
        case SampleType::kIcPz: {
          ic_pz_->ISR_MaybeFinishSample();
          const auto& status = ic_pz_->status();
          if (status.active) {
            status_.spi.active = true;
            status_.spi.value = status.value;
            status_.spi.nonce = status.nonce;
            status_.spi.ic_pz_bits =
                (status.warn ? 1 : 0) |
                (status.err ? 2 : 0);
          }
          break;
        }
        case SampleType::kGpio: {
          for (size_t i = 0; i < pin_count_; i++) {
            if (config_.pins[i].mode == aux::Pin::Mode::kDigitalInput) {
              status_.pins[i] = digital_inputs_[i]->read() ? true : false;
            } else if (config_.pins[i].mode == aux::Pin::Mode::kDigitalOutput) {
              digital_outputs_[i]->write(status_.pins[i]);
            } else if (config_.pins[i].mode == aux::Pin::Mode::kPwmOut) {
              pwm_[i]->write(status_.pwm[i]);
            }
          }
          break;
        }
        case SampleType::kHall: {
          status_.hall.active = true;
          const auto old_bits = status_.hall.bits;
          status_.hall.bits =
              ((halla_->read() ? 1 : 0) << 0) |
              ((hallb_->read() ? 1 : 0) << 1) |
              ((hallc_->read() ? 1 : 0) << 2);
          const auto delta = status_.hall.bits ^ old_bits;
          const auto numbits_changed =
              (delta & 0x01) ? 1 : 0 +
              (delta & 0x02) ? 1 : 0 +
              (delta & 0x04) ? 1 : 0;

          if (numbits_changed > 1) {
            status_.hall.error++;
          }

          static constexpr uint8_t kHallMapping[] = {
            0,  // invalid
            0,  // 0b001 => 0
            2,  // 0b010 => 2
            1,  // 0b011 => 1
            4,  // 0b100 => 4
            5,  // 0b101 => 5
            3,  // 0b110 => 3
            0,  // invalid
          };
          status_.hall.count =
              kHallMapping[status_.hall.bits ^ config_.hall.polarity];
          break;
        }
        case SampleType::kQuad: {
          quad_->ISR_Update(&status_.quadrature);
          break;
        }
        case SampleType::kIndex: {
          index_->ISR_Update(&status_.index);
          break;
        }
        case SampleType::kAksim2: {
          aksim2_->ISR_Update(&status_.uart);
          break;
        }
        case SampleType::kCuiAmt21: {
          cui_amt21_->ISR_Update(&status_.uart);
          break;
        }
        case SampleType::kCuiAmt22: {
          cui_amt22_->ISR_Update(&status_.spi);
          break;
        }
        case SampleType::kI2c: {
          ISR_I2C_Update();
          break;
        }
        case SampleType::kNone: {
          return;
        }
        default: {
          break;
        }
      }
    }
  }

  void ISR_I2C_Update() {
    if (!i2c_startup_complete_) {
      if (timer_->read_ms() > config_.i2c_startup_delay_ms) {
        i2c_startup_complete_ = true;
      }
    }

    ISR_PollI2c();

    i2c_->Poll();

    const auto read_status = i2c_->CheckRead();

    if (read_status != Stm32I2c::ReadStatus::kNoStatus) {
      for (size_t i = 0; i < status_.i2c.devices.size(); i++) {
        auto& state = i2c_state_[i];
        auto& status = status_.i2c.devices[i];

        if (!state.pending) { continue; }

        state.pending = false;
        if (read_status == Stm32I2c::ReadStatus::kError) {
          status.error_count++;
          break;
        }

        ISR_ParseI2c(i);
        break;
      }
    }
  }

  // Call this after AuxADC::ISR_EndSample() has completed.
  void ISR_EndAnalogSample() MOTEUS_CCM_ATTRIBUTE {
    if (!any_adc_) { return; }

    for (size_t i = 0; i < pin_count_; i++) {
      if (analog_input_active_[i]) {
        status_.analog_inputs[i] =
            static_cast<float>(adc_info_.value[i]) / 4096.0f;
      }
    }

    if (config_.sine_cosine.enabled) {
      status_.sine_cosine.active = true;
      const auto& config = config_.sine_cosine;
      const auto s = adc_info_.value[sine_pin_];
      const auto c = adc_info_.value[cosine_pin_];
      status_.sine_cosine.sine_raw = s;
      status_.sine_cosine.cosine_raw = c;
      const auto value = std::min<int32_t>(
          65535,
          std::max<int32_t>(
              0,
              static_cast<int32_t>(
                  65536.0f / k2Pi *
                  (FastAtan2(
                      static_cast<float>(
                          static_cast<int32_t>(s) - config.common) / 2048.0f,
                      static_cast<float>(
                          static_cast<int32_t>(c) - config.common) / 2048.0f) +
                   kPi))));
      status_.sine_cosine.value = value;
    }
  }

  void PollMillisecond() {
    if (!as5047_ && as5047_options_) {
      // We can only start sampling the as5047 after 10ms have passed
      // since boot.
      if (timer_->read_ms() > 10) {
        // We could be operating from an ISR context, so we disable
        // interrupts before updating it.
        __disable_irq();
        status_.error = aux::AuxError::kNone;

        as5047_.emplace(*as5047_options_);
        AddSampleType(SampleType::kAs5047, true, true);

        // The very first SPI reading after power on returns bogus
        // results.  So we do one here to ensure that all those that
        // our ISR performs are good.
        as5047_->Sample();

        __enable_irq();
      }
    }

    if (!cui_amt22_ && cui_amt22_options_) {
      if (timer_->ms_since_boot() > 200) {
        __disable_irq();
        status_.error = aux::AuxError::kNone;

        cui_amt22_.emplace(*cui_amt22_options_);
        AddSampleType(SampleType::kCuiAmt22, true, true);

        __enable_irq();
      }
    }

    if (!ma732_ && ma732_options_) {
      // The worst case startup time for the MA732 is 260ms, however
      // we can't current measure that long from startup.  So we'll
      // just check for 10ms like the AS5047 and let the application
      // deal with it if it has configured a longer filter period.
      if (timer_->read_ms() > 10) {
        __disable_irq();
        status_.error = aux::AuxError::kNone;
        ma732_.emplace(timer_, *ma732_options_);
        if (ma732_->error()) {
          status_.error = aux::AuxError::kMaXXXConfigError;
        } else {
          AddSampleType(SampleType::kMa732, true, true);
          // Ensure we have at least one sample under our belt.
          ma732_->Sample();
        }

        __enable_irq();
      }
    }

    if (ic_pz_) {
      ic_pz_->PollMillisecond();
    }
  }

  void Poll() {
    if (tunnel_polling_enabled_) {
      if (tunnel_write_outstanding_) {
        if (uart_->is_dma_write_finished()) {
          uart_->finish_dma_write();
          tunnel_write_outstanding_ = false;
          StartTunnelRead();
        }
      }
      if (uart_->read_bytes_remaining() != sizeof(tunnel_write_buf_1_) &&
          !stream_write_outstanding_) {
        uart_->finish_dma_read();
        const auto bytes_read =
            sizeof(tunnel_write_buf_1_) - uart_->read_bytes_remaining();
        stream_write_outstanding_ = true;
        mjlib::micro::AsyncWrite(
            *tunnel_stream_,
            {current_tunnel_write_buf_.data(), bytes_read},
            [this](const auto&) {
              stream_write_outstanding_ = false;
            });
        std::swap(current_tunnel_write_buf_, next_tunnel_write_buf_);
        uart_->start_dma_read(current_tunnel_write_buf_);
      }
    }
  }


  void WriteDigitalOut(uint32_t value) {
    for (size_t i = 0; i < pin_count_; i++) {
      if (config_.pins[i].mode == aux::Pin::Mode::kDigitalOutput) {
        status_.pins[i] = (value & (1 << i)) ? true : false;
      }
    }
  }

  void WritePwmOut(int pin, float value) {
    pin = std::max<int>(0, std::min<int>(pin_count_ - 1, pin));
    status_.pwm[pin] = std::max(0.0f, std::min(1.0f, value));
  }

  USART_TypeDef* debug_uart() const {
    if (config_.uart.mode == aux::UartEncoder::Config::kDebug) {
      return uart_->uart();
    }
    return nullptr;
  }

  Status* status() { return &status_; }
  const Config* config() const { return &config_; }

 private:
  enum class SampleType {
    kNone = 0,

    // Sort these so that those that have start sampling functions are first.
    kAs5047 = 1,
    kMa732 = 2,
    kIcPz = 3,

    kGpio = 4,
    kHall = 5,
    kQuad = 6,
    kIndex = 7,
    kAksim2 = 8,
    kCuiAmt21 = 9,
    kI2c = 10,
    kCuiAmt22 = 11,

    kLastEntry,
  };

  void HandleCommand(const std::string_view& message,
                     const mjlib::micro::CommandManager::Response& response) {
    mjlib::base::Tokenizer tokenizer(message, " ");
    const auto cmd_text = tokenizer.next();

    if (cmd_text == "out") {
      const auto value_str = tokenizer.next();
      if (value_str.empty()) {
        WriteMessage(response, "ERR missing gpio value\r\n");
        return;
      }

      const auto value = std::strtol(value_str.data(), nullptr, 0);

      WriteDigitalOut(value);
      WriteOk(response);
      return;
    }

    if (cmd_text == "pwm") {
      const auto pin_str = tokenizer.next();
      const auto maybe_value = Strtof(tokenizer.next());
      if (pin_str.empty() ||
          !maybe_value) {
        WriteMessage(response, "ERR missing pin or value\r\n");
        return;
      }

      const auto pin = std::strtol(pin_str.data(), nullptr, 0);
      WritePwmOut(pin, *maybe_value);

      WriteOk(response);
      return;
    }

    if (cmd_text == "ic-extra") {
      if (!ic_pz_) {
        WriteMessage(response, "ERR no iC-PZ setup\r\n");
        return;
      }

      const auto cmd_str = tokenizer.next();
      if (cmd_str.empty()) {
        WriteMessage(response, "ERR missing bitfield\r\n");
        return;
      }

      ic_pz_->SetExtraRead(std::strtol(cmd_str.data(), nullptr, 0));

      WriteOk(response);
      return;
    }
    if (cmd_text == "ic-cmd") {
      if (!ic_pz_) {
        WriteMessage(response, "ERR no iC-PZ setup\r\n");
        return;
      }

      const auto cmd_str = tokenizer.next();
      if (cmd_str.empty()) {
        WriteMessage(response, "ERR missing cmd\r\n");
        return;
      }

      const uint8_t cmd = ParseHexByte(cmd_str.data());

      ic_pz_->StartCommand(cmd);

      WriteOk(response);
      return;
    }

    if (cmd_text == "ic-wr") {
      if (!ic_pz_) {
        WriteMessage(response, "ERR no iC-PZ setup\r\n");
        return;
      }

      const auto reg_str = tokenizer.next();
      const auto data_str = tokenizer.next();
      if (reg_str.empty() || data_str.empty()) {
        WriteMessage(response, "ERR missing reg/data\r\n");
        return;
      }

      const auto reg = ParseHexByte(reg_str.data());
      if (reg < 0) {
        WriteMessage(response, "ERR invalid reg\r\n");
        return;
      }
      uint8_t data[16] = {};
      int bytes = 0;
      for (size_t i = 0; i < data_str.size(); i += 2) {
        const int value = ParseHexByte(&data_str[i]);
        if (value < 0) {
          WriteMessage(response, "ERR invalid data\r\n");
          return;
        }
        data[bytes++] = value;
        if (bytes >= static_cast<int>(sizeof(data))) {
          WriteMessage(response, "ERR too much data\r\n");
          return;
        }
      }

      ic_pz_->WriteRegisters(
          reg, std::string_view(reinterpret_cast<const char*>(data), bytes));

      WriteOk(response);
      return;
    }

    if (cmd_text == "ic-rd") {
      if (!ic_pz_) {
        WriteMessage(response, "ERR no iC-PZ setup\r\n");
        return;
      }

      const auto reg_str = tokenizer.next();
      const auto size_str = tokenizer.next();
      if (reg_str.empty()) {
        WriteMessage(response, "ERR missing reg\r\n");
        return;
      }

      const auto reg = ParseHexByte(reg_str.data());
      if (reg < 0) {
        WriteMessage(response, "ERR invalid reg\r\n");
        return;
      }

      const auto size =
          size_str.empty() ? 1 : std::strtol(size_str.data(), nullptr, 0);
      uint8_t data[16] = {};
      if (size >= static_cast<int>(sizeof(data))) {
        WriteMessage(response, "ERR too much data\r\n");
        return;
      }

      ic_pz_->ReadRegisters(reg, mjlib::base::string_span(
                                reinterpret_cast<char*>(&data[0]), size));

      ssize_t pos = 0;
      auto fmt =
          [&](auto ...args) {
            pos += snprintf(&emit_line_[pos], sizeof(emit_line_) - pos, args...);
          };
      for (int i = 0; i < size; i++) {
        fmt("%02X", data[i]);
      }
      fmt("\r\nOK\r\n");
      WriteMessage(response, std::string_view(emit_line_, pos));
      return;
    }

    WriteMessage(response, "ERR unknown aux cmd\r\n");
  }

  void WriteOk(const mjlib::micro::CommandManager::Response& response) {
    WriteMessage(response, "OK\r\n");
  }

  void WriteMessage(const mjlib::micro::CommandManager::Response& response,
                    const std::string_view& message) {
    AsyncWrite(*response.stream, message, response.callback);
  }


  static constexpr uint8_t AS5048_REG_AGC = 0xFA;
  static constexpr uint8_t AS5048_REG_DIAG = 0xFB;
  static constexpr uint8_t AS5048_REG_MAG_HIGH = 0xFC;
  static constexpr uint8_t AS5048_REG_MAG_LOW = 0xFD;
  static constexpr uint8_t AS5048_REG_ANGLE_HIGH = 0xFE;
  static constexpr uint8_t AS5048_REG_ANGLE_LOW = 0xFF;

  static constexpr uint8_t AS5600_REG_CONF = 0x07;
  static constexpr uint8_t AS5600_REG_STATUS = 0x0B;
  static constexpr uint8_t AS5600_REG_RAW_ANGLE_HIGH = 0x0C;
  static constexpr uint8_t AS5600_REG_RAW_ANGLE_LOW = 0x0D;
  static constexpr uint8_t AS5600_REG_ANGLE_HIGH = 0x0E;
  static constexpr uint8_t AS5600_REG_ANGLE_LOW = 0x0F;
  static constexpr uint8_t AS5600_REG_AGC = 0x1A;
  static constexpr uint8_t AS5600_REG_MAG_HIGH = 0x1B;
  static constexpr uint8_t AS5600_REG_MAG_LOW = 0x1C;

  void ISR_ParseI2c(size_t index) {
    const auto& config = config_.i2c.devices[index];
    auto& status = status_.i2c.devices[index];

    using DC = aux::I2C::DeviceConfig;
    switch (config.type) {
      case DC::kAs5048: {
        ISR_ParseAs5048(&status);
        break;
      }
      case DC::kAs5600: {
        ISR_ParseAs5600(&status);
        break;
      }
      case DC::kNone:
      case DC::kNumTypes: {
        // Ignore.
        break;
      }
    }
  }

  void ISR_ParseAs5048(aux::I2C::DeviceStatus* status) {
    status->active = i2c_startup_complete_;

    status->value =
        (encoder_raw_data_[4] << 8) |
        (encoder_raw_data_[5] << 2);
    status->nonce += 1;

    status->ams_agc = encoder_raw_data_[0];
    status->ams_diag = encoder_raw_data_[1];
    status->ams_mag =
        (encoder_raw_data_[2] << 8) |
        (encoder_raw_data_[3] << 2);
  }

  void ISR_ParseAs5600(aux::I2C::DeviceStatus* status) {
    status->active = i2c_startup_complete_;

    status->value =
        ((encoder_raw_data_[1] << 8) |
         (encoder_raw_data_[2])) & 0x0fff;
    status->nonce += 1;

    status->ams_agc = 0;
    status->ams_diag = encoder_raw_data_[0];
    status->ams_mag = 0;
  }

  void ISR_PollI2c() {
    using DC = aux::I2C::DeviceConfig;

    // If our periperhal is currently busy, nothing we can do.
    if (i2c_->busy()) { return; }

    const auto now_us = timer_->read_us();

    for (size_t i = 0; i < i2c_state_.size(); i++) {
      const auto& config = config_.i2c.devices[i];

      if (config.type == DC::kNone) { continue; }

      auto& state = i2c_state_[i];

      if (!state.initialized) {
        switch (config.type) {
          case DC::kAs5048: {
            break;
          }
          case DC::kAs5600: {
            encoder_raw_data_[0] = 0
                | (0 << 5) // watchdog = off
                | (1 << 2) // fast filter threshold = 6/1
                | (3 << 0) // slow filter = 2x
                | 0;
            encoder_raw_data_[1] = 0
                | (0 << 6) // PWM frequency = 115Hz
                | (0 << 4) // Output stage = 0-100%
                | (0 << 2) // Hysteresis = OFF
                | (0 << 0) // Power Mode = NOM
                | 0;
            i2c_->StartWriteMemory(
                config.address, AS5600_REG_CONF,
                std::string_view(reinterpret_cast<const char*>(
                                     encoder_raw_data_), 2));
            break;
          }
          case DC::kNone:
          case DC::kNumTypes: {
            MJ_ASSERT(false);
            break;
          }
        }
        // We can initialize at most one device per poll cycle.
        state.initialized = true;
        return;
      }

      const auto delta_us =
          timer_->subtract_us(now_us, state.last_poll_us);

      if (delta_us >= config.poll_rate_us) {
        state.last_poll_us = now_us;
        state.pending = true;

        switch (config.type) {
          case DC::kAs5048: {
            StartI2cRead<6>(config.address, AS5048_REG_AGC);
            break;
          }
          case DC::kAs5600: {
            StartI2cRead<3>(config.address, AS5600_REG_STATUS);
            break;
          }
          case DC::kNone:
          case DC::kNumTypes: {
            MJ_ASSERT(false);
            break;
          }
        }
        // We can start at most 1 device per polling cycle, so bail
        // out here.
        return;
      }
    }
  }

  template <size_t size>
  void StartI2cRead(uint8_t address, uint8_t reg) {
    static_assert(sizeof(encoder_raw_data_) >= size);
    i2c_->StartReadMemory(address, reg,
                          mjlib::base::string_span(
                              reinterpret_cast<char*>(&encoder_raw_data_[0]),
                              size));
  }

  void StartTunnelRead() {
    tunnel_stream_->AsyncReadSome(
        tunnel_read_buf_,
        [this](const auto& error, const auto size) {
          HandleTunnelRead(error, size);
        });
  }

  void HandleTunnelRead(const mjlib::micro::error_code& ec,
                        std::ptrdiff_t size) {
    if (!ec &&
        uart_ &&
        config_.uart.mode == aux::UartEncoder::Config::kTunnel) {
      tunnel_write_outstanding_ = true;

      // We have a valid UART and can start a write.
      uart_->start_dma_write(std::string_view(tunnel_read_buf_, size));
    } else {
      // We are just going to discard this and start another read
      // again immediately.
      StartTunnelRead();
    }
  }


  void HandleConfigUpdate() {
    // Disable our ISR before changing any config.
    any_isr_enabled_ = false;

    const auto old_epoch = status_.epoch;

    status_ = {};

    status_.epoch = old_epoch + 1;

    ////////////////////////////////////////////
    // Reset everything to a known default.
    i2c_.reset();
    i2c_pullup_dout_.reset();
    if (hw_config_.options.i2c_pullup != NC) {
      // Construct, then destroy a DigitalInput to get the pin to be
      // tri-stated again.
      DigitalIn din(hw_config_.options.i2c_pullup);
    }
    i2c_state_ = {};

    as5047_.reset();
    as5047_options_.reset();
    ma732_.reset();
    ma732_options_.reset();
    onboard_cs_.reset();
    cui_amt22_.reset();
    cui_amt22_options_.reset();

    bool updated_any_isr = false;

    for (auto& in : digital_inputs_) { in.reset(); }
    for (auto& out : digital_outputs_) { out.reset(); }
    pwm_timer_set_.reset();
    for (auto& pwm : pwm_) { pwm.reset(); }
    halla_.reset();
    hallb_.reset();
    hallc_.reset();

    quad_.reset();
    index_.reset();
    ic_pz_.reset();

    uart_.reset();

    start_sample_types_ = {};
    finish_sample_types_ = {};

    tunnel_write_outstanding_ = false;
    tunnel_polling_enabled_ = false;

    // We purposefully don't reset the stream write flag, since just
    // because our config was updated doesn't mean the call to
    // AsyncWrite has completed.
    //
    // stream_write_outstanding_ = false;

    if (rs422_de_) { rs422_de_->write(0); }
    if (rs422_re_) { rs422_re_->write(1); }
    aksim2_.reset();
    cui_amt21_.reset();

    for (auto& cfg : adc_info_.config) {
      cfg.adc_num = -1;
      cfg.channel = -1;
    }

    any_adc_ = false;
    for (auto& value : analog_input_active_) {
      value = false;
    }

    sine_pin_ = -1;
    cosine_pin_ = -1;

    for (auto& device : config_.i2c.devices) {
      device.poll_rate_us = std::max<int32_t>(100, device.poll_rate_us);
    }

    for (auto& pin : hw_config_.pins) {
      if (pin.mbed == NC) { continue; }
      pin_function(pin.mbed, STM_MODE_ANALOG);
    }

    ////////////////////////////////////////////
    // Now that everything is reset, reset any board defaults to the
    // correct value.

    if (config_.spi.mode == aux::Spi::Config::kBoardDefault) {
      switch (spi_default_) {
        case kNoDefaultSpi: {
          config_.spi.mode = aux::Spi::Config::Mode::kDisabled;
          break;
        }
        case kDefaultOnboardSpi: {
          onboard_spi_available_ = true;
          config_.spi.mode = aux::Spi::Config::Mode::kOnboardAs5047;
          break;
        }
        case kDefaultOnboardMa600: {
          onboard_spi_available_ = true;
          config_.spi.mode = aux::Spi::Config::Mode::kOnboardMa600;
          break;
        }
      }
    }

    ////////////////////////////////////////////
    // Validate our config, one option at a time.

    const bool any_i2c =
        [&]() {
          for (const auto& device : config_.i2c.devices) {
            if (device.type != aux::I2C::DeviceConfig::kNone) {
              return true;
            }
          }
          return false;
        }();

    if (any_i2c) {
      PinName sda = NC;
      PinName scl = NC;
      for (size_t i = 0; i < pin_count_; i++) {
        if (config_.pins[i].mode != aux::Pin::Mode::kI2C) { continue; }

        for (const auto& pin : hw_config_.pins) {
          if (pin.number != static_cast<int>(i)) { continue; }
          if (!pin.i2c) { continue; }

          const auto int_i2c = reinterpret_cast<uint32_t>(pin.i2c);

          for (uint32_t alt : {0, 0x100, 0x200, 0x300, 0x400}) {
            const PinName mbed_pin = static_cast<PinName>(pin.mbed | alt);
            if (pinmap_find_peripheral(mbed_pin, PinMap_I2C_SCL) == int_i2c) {
              scl = mbed_pin;
              break;
            } else if (pinmap_find_peripheral(mbed_pin, PinMap_I2C_SDA) == int_i2c) {
              sda = mbed_pin;
              break;
            }
          }
        }
      }
      if (sda == NC ||
          scl == NC) {
        // We must have two pins configured as I2C that match our hw
        // ability.
        status_.error = aux::AuxError::kI2cPinError;
        return;
      }

      if (hw_config_.options.i2c_pullup != NC) {
        if (config_.i2c.pullup) {
          i2c_pullup_dout_.emplace(hw_config_.options.i2c_pullup, 1);
          i2c_pullup_dout_->write(1);
        }
      } else if (config_.i2c.pullup) {
        status_.error = aux::AuxError::kI2cPinError;
        return;
      }

      i2c_.emplace(
          [&]() {
            Stm32I2c::Options options;
            options.sda = sda;
            options.scl = scl;
            options.frequency = config_.i2c.i2c_hz;
            options.i2c_mode = static_cast<I2cMode>(config_.i2c.i2c_mode);
            return options;
          }());
      updated_any_isr = true;
    }

    if ((config_.spi.mode == aux::Spi::Config::kOnboardAs5047 ||
         config_.spi.mode == aux::Spi::Config::kOnboardMa600) &&
        !onboard_spi_available_) {
      status_.error = aux::AuxError::kSpiPinError;
      return;
    }

    if ((config_.spi.mode != aux::Spi::Config::kOnboardAs5047 &&
         config_.spi.mode != aux::Spi::Config::kOnboardMa600) &&
        onboard_spi_available_) {
      // If we're not using the onboard encoder, ensure that its CS
      // line is not asserted.
      onboard_cs_.emplace(g_hw_pins.as5047_cs, 1);
    }

    // Default to the onboard encoder for SPI.
    if (config_.spi.mode != aux::Spi::Config::kDisabled) {
      const auto maybe_spi = aux::FindSpiOption(
          config_.pins, pin_count_, hw_config_,
          ((config_.spi.mode == aux::Spi::Config::kOnboardAs5047 ||
            config_.spi.mode == aux::Spi::Config::kOnboardMa600) ?
           aux::kDoNotRequireCs : aux::kRequireCs));
      if (!maybe_spi) {
        status_.error = aux::AuxError::kSpiPinError;
        return;
      }
      const PinName spi_cs_pin =
          (config_.spi.mode == aux::Spi::Config::kOnboardAs5047 ||
           config_.spi.mode == aux::Spi::Config::kOnboardMa600) ?
          g_hw_pins.as5047_cs : maybe_spi->cs;

      Stm32Spi::Options spi_options;
      spi_options.frequency = config_.spi.rate_hz;
      spi_options.cs = spi_cs_pin;
      spi_options.mosi = maybe_spi->mosi;
      spi_options.miso = maybe_spi->miso;
      spi_options.sck = maybe_spi->sck;

      switch (config_.spi.mode) {
        case aux::Spi::Config::kAs5047:
        case aux::Spi::Config::kOnboardAs5047: {
          AS5047::Options options = spi_options;
          options.timeout = 200;
          as5047_options_ = options;

          break;
        }
        case aux::Spi::Config::kCuiAmt22: {
          CuiAmt22::Options options = spi_options;
          // The max limit is 2Mbps per the datasheet.  The minimum limit is set to ensure
          // there is sufficient setup time between each of:
          // Tclk: CS_low -> SPI: 2.5us
          // Tb: between bytes: 2.5us
          // Tr: SPI -> CS_high: 3us
          // Tcs: CS_low -> CS_low: 40us
          // assuming that each step is taken once per ISR and the maximum ISR rate is 30kHz.
          if (options.frequency > 2000000) { options.frequency = 2000000; }
          if (options.frequency < 600000) { options.frequency = 600000; }
          options.timeout = 2000;
          cui_amt22_options_ = options;

          break;
        }
        case aux::Spi::Config::kIcPz: {
          IcPz::Options options{spi_options};
          options.timeout = 2000;
          options.rx_dma = dma_channels_[0];
          options.tx_dma = dma_channels_[1];
          ic_pz_.emplace(options, timer_);
          break;
        }
        case aux::Spi::Config::kMa732: {
          MA732::Options options = spi_options;
          options.filter_us = config_.spi.filter_us;
          options.bct = config_.spi.bct;
          options.model = MA732::kMa732;
          using T = aux::Spi::Config::Trim;
          options.enable_trim =
              (config_.spi.trim == T::kNone) ? 0 :
              (config_.spi.trim == T::kTrimX) ? 1 :
              (config_.spi.trim == T::kTrimY) ? 2 :
              0;
          ma732_options_ = options;
          break;
        }
        case aux::Spi::Config::kMa600:
        case aux::Spi::Config::kOnboardMa600: {
          MA732::Options options = spi_options;
          options.filter_us = config_.spi.filter_us;
          options.bct = config_.spi.bct;
          options.model = MA732::kMa600;
          using T = aux::Spi::Config::Trim;
          options.enable_trim =
              (config_.spi.trim == T::kNone) ? 0 :
              (config_.spi.trim == T::kTrimX) ? 1 :
              (config_.spi.trim == T::kTrimY) ? 2 :
              0;
          ma732_options_ = options;
          break;
        }
        case aux::Spi::Config::kDisabled:
        case aux::Spi::Config::kBoardDefault:
        case aux::Spi::Config::kNumModes: {
          MJ_ASSERT(false);
          break;
        }
      }

      updated_any_isr = true;
    }

    if (config_.hall.enabled) {
      // We need exactly 3 hall sensors.
      int count = 0;
      for (size_t i = 0; i < pin_count_; i++) {
        if (config_.pins[i].mode == aux::Pin::Mode::kHall) { count++; }
      }
      if (count != 3) {
        status_.error = aux::AuxError::kHallPinError;
      }
      for (size_t i = 0; i < pin_count_; i++) {
        const auto cfg = config_.pins[i];
        if (cfg.mode != aux::Pin::Mode::kHall) { continue; }
        const auto mbed = [&]() {
            for (const auto& pin : hw_config_.pins) {
              if (pin.number == static_cast<int>(i)) { return pin.mbed; }
            }
            return NC;
        }();
        if (!halla_) {
          halla_.emplace(mbed, aux::MbedMapPull(cfg.pull));
        } else if (!hallb_) {
          hallb_.emplace(mbed, aux::MbedMapPull(cfg.pull));
        } else if (!hallc_) {
          hallc_.emplace(mbed, aux::MbedMapPull(cfg.pull));
        }
      }
      updated_any_isr = true;
    }

    if (config_.quadrature.enabled) {
      quad_.emplace(config_.quadrature, &status_.quadrature,
                    config_.pins, pin_count_, hw_config_);
      if (quad_->error() != aux::AuxError::kNone) {
        status_.error = quad_->error();
        quad_.reset();
      } else {
        updated_any_isr = true;
      }
    }

    if (config_.index.enabled) {
      index_.emplace(config_.index, config_.pins, pin_count_, hw_config_);
      if (index_->error() != aux::AuxError::kNone) {
        status_.error = index_->error();
        index_.reset();
      } else {
        updated_any_isr = true;
      }
    }

    if (config_.sine_cosine.enabled) {
      // We need exactly 1 each of a kSine and a kCosine.
      for (size_t i = 0; i < pin_count_; i++) {
        const auto cfg = config_.pins[i];
        if (cfg.mode == aux::Pin::Mode::kSine) {
          if (sine_pin_ != -1) {
            status_.error = aux::AuxError::kSineCosinePinError;
            return;
          }
          sine_pin_ = i;
        } else if (cfg.mode == aux::Pin::Mode::kCosine) {
          if (cosine_pin_ != -1) {
            status_.error = aux::AuxError::kSineCosinePinError;
            return;
          }
          cosine_pin_ = i;
        }
      }
      if (sine_pin_ == -1 || cosine_pin_ == -1) {
        status_.error = aux::AuxError::kSineCosinePinError;
        return;
      }
      // We leave the pin configuration to the analog section below.
      updated_any_isr = true;
    }

    if (config_.uart.mode != aux::UartEncoder::Config::kDisabled) {
      const auto maybe_uart = aux::FindUartOption(
          config_.pins, pin_count_, hw_config_);
      if (!maybe_uart) {
        status_.error = aux::AuxError::kUartPinError;
        return;
      }

      if (config_.uart.rs422 && (!rs422_de_ || !rs422_re_)) {
        status_.error = aux::AuxError::kUartPinError;
        return;
      }

      if (rs422_de_) { rs422_de_->write(config_.uart.rs422); }
      if (rs422_re_) { rs422_re_->write(!config_.uart.rs422); }

      uart_.emplace(
          [&]() {
            Stm32G4DmaUart::Options options;
            options.rx = maybe_uart->rx;
            options.tx = maybe_uart->tx;
            options.baud_rate = config_.uart.baud_rate;
            options.rx_dma = dma_channels_[2];
            options.tx_dma = dma_channels_[3];
            return options;
          }());

      using C = aux::UartEncoder::Config;
      switch (config_.uart.mode) {
        case C::kDisabled: {
          MJ_ASSERT(false);
          return;
        }
        case C::kAksim2: {
          aksim2_.emplace(config_.uart, &*uart_, timer_);
          break;
        }
        case C::kTunnel: {
          uart_->start_dma_read(current_tunnel_write_buf_);
          tunnel_polling_enabled_ = true;
          break;
        }
        case C::kDebug: {
          // Nothing special to do here.
          break;
        }
        case C::kCuiAmt21: {
          cui_amt21_.emplace(config_.uart, &*uart_, timer_);
          break;
        }
        default: {
          status_.error = aux::AuxError::kUartPinError;
          return;
        }
      }

      updated_any_isr = true;
    }

    for (size_t i = 0; i < pin_count_; i++) {
      const auto cfg = config_.pins[i];
      const auto first_mbed = [&]() {
          for (const auto& pin : hw_config_.pins) {
            if (pin.number == static_cast<int>(i)) {
              return pin.mbed;
            }
          }
          return NC;
      }();
      if (cfg.mode == aux::Pin::Mode::kDigitalInput) {
        status_.gpio_bit_active |= (1 << i);
        digital_inputs_[i].emplace(first_mbed,
                                   aux::MbedMapPull(cfg.pull));
        updated_any_isr = true;
      } else if (cfg.mode == aux::Pin::Mode::kDigitalOutput) {
        status_.gpio_bit_active |= (1 << i);
        digital_outputs_[i].emplace(first_mbed,
                                    aux::MbedMapPull(cfg.pull));
        updated_any_isr = true;
      } else if (cfg.mode == aux::Pin::Mode::kPwmOut) {
        const auto timer = [&]() -> TIM_TypeDef* {
          for (const auto& pin: hw_config_.pins) {
            if (pin.number == static_cast<int>(i) &&
                pin.timer) {
              if (quad_ && quad_->hwtimer() == pin.timer) {
                // If a hardware quadrature input is already using
                // this timer, then we can't.
                continue;
              }
              return pin.timer;
            }
          }
          // There were no available hardware timers for this pin.
          return nullptr;
        }();

        if (timer == nullptr) {
          status_.error = aux::AuxError::kPwmPinError;
          return;
        }
        auto* pwm_timer = pwm_timer_set_.get(timer, config_.pwm_period_us);

        status_.gpio_bit_active |= (1 << i);
        pwm_[i].emplace(first_mbed, pwm_timer);
        updated_any_isr = true;
      } else if (cfg.mode == aux::Pin::Mode::kAnalogInput ||
                 cfg.mode == aux::Pin::Mode::kSine ||
                 cfg.mode == aux::Pin::Mode::kCosine) {

        any_adc_ = true;
        analog_input_active_[i] = true;
        status_.analog_bit_active |= (1 << i);

        // Find the mbed pin, the adc num and adc channel for this
        // connector pin.
        const auto* pin = [&]() {
            for (const auto& pin : hw_config_.pins) {
              if (pin.number != static_cast<int>(i)) { continue; }
              if (pin.adc_num < 0) { continue; }
              return &pin;
            }
            return static_cast<const aux::AuxPinConfig*>(nullptr);
        }();
        if (pin == nullptr) {
          status_.error = aux::AuxError::kAdcPinError;
          return;
        }

        adc_info_.config[i].adc_num = pin->adc_num;
        adc_info_.config[i].channel = pin->adc_sqr;

        pinmap_pinout(pin->mbed, PinMap_ADC);
        switch (cfg.pull) {
          case aux::Pin::Pull::kPullUp:
          case aux::Pin::Pull::kOpenDrain: {
            // Analog inputs only support no pull, or a pull-down.
            status_.error = aux::AuxError::kAdcPinError;
            return;
          }
          case aux::Pin::Pull::kNone:
          case aux::Pin::Pull::kPullDown: {
            break;
          }
        }
        pin_mode(pin->mbed, aux::MbedMapPull(cfg.pull));
        updated_any_isr = true;
      }
    }

    if (ic_pz_) { AddSampleType(SampleType::kIcPz, true, true); }
    if (status_.gpio_bit_active != 0) {
      AddSampleType(SampleType::kGpio, false, true);
    }
    if (config_.hall.enabled) { AddSampleType(SampleType::kHall, false, true); }
    if (quad_) { AddSampleType(SampleType::kQuad, false, true); }
    if (index_) { AddSampleType(SampleType::kIndex, false, true); }
    if (aksim2_) { AddSampleType(SampleType::kAksim2, false, true); }
    if (cui_amt21_) { AddSampleType(SampleType::kCuiAmt21, false, true); }
    if (i2c_) { AddSampleType(SampleType::kI2c, false, true); }

    pwm_timer_set_.Start();

    adc_info_.config_update();
    any_isr_enabled_ = updated_any_isr;
  }

  void AddSampleType(SampleType stype, bool start, bool finish) {
    if (start) {
      for (size_t i = 0; ; i++) {
        if (start_sample_types_[i] == SampleType::kNone) {
          start_sample_types_[i] = stype;
          break;
        }
      }
    }
    if (finish) {
      for (size_t i = 0; ; i++) {
        if (finish_sample_types_[i] == SampleType::kNone) {
          finish_sample_types_[i] = stype;
          break;
        }
      }
    }
  }

  static int ParseHexNybble(char c) {
    if (c >= '0' && c <= '9') { return c - '0'; }
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    return -1;
  }

  static int ParseHexByte(const char* value) {
    int high = ParseHexNybble(value[0]);
    if (high < 0) { return high; }
    int low = ParseHexNybble(value[1]);
    if (low < 0) { return low; }
    return (high << 4) | low;
  }

  Config config_;
  Status status_;

  const size_t pin_count_;
  mjlib::micro::AsyncStream* const tunnel_stream_;
  MillisecondTimer* const timer_;

  bool any_isr_enabled_ = false;

  std::optional<AS5047> as5047_;
  std::optional<AS5047::Options> as5047_options_;

  std::optional<MA732> ma732_;
  std::optional<MA732::Options> ma732_options_;

  std::optional<CuiAmt22> cui_amt22_;
  std::optional<CuiAmt22::Options> cui_amt22_options_;

  std::optional<IcPz> ic_pz_;
  std::optional<DigitalOut> onboard_cs_;

  std::array<std::optional<DigitalIn>,
             aux::AuxConfig::kNumPins> digital_inputs_;
  std::array<std::optional<DigitalOut>,
             aux::AuxConfig::kNumPins> digital_outputs_;
  aux::Stm32PwmTimerSet pwm_timer_set_;
  std::array<std::optional<aux::Stm32Pwm>,
             aux::AuxConfig::kNumPins> pwm_;

  std::optional<DigitalIn> halla_;
  std::optional<DigitalIn> hallb_;
  std::optional<DigitalIn> hallc_;

  std::optional<aux::Stm32Quadrature> quad_;
  std::optional<aux::Stm32Index> index_;
  std::optional<Stm32G4DmaUart> uart_;
  std::optional<Aksim2> aksim2_;
  std::optional<CuiAmt21> cui_amt21_;
  std::optional<DigitalOut> rs422_re_;
  std::optional<DigitalOut> rs422_de_;

  AuxADC::AuxInfo& adc_info_;
  bool any_adc_ = false;
  std::array<bool, aux::AuxConfig::kNumPins> analog_input_active_ = {};
  int sine_pin_ = -1;
  int cosine_pin_ = -1;

  char emit_line_[48] = {};

  std::optional<Stm32I2c> i2c_;

  struct I2cState {
    bool initialized = false;
    int32_t last_poll_us = 0;
    bool pending = false;
  };

  std::array<I2cState, 3> i2c_state_;
  uint8_t encoder_raw_data_[6] = {};
  bool i2c_startup_complete_ = false;

  static constexpr size_t kTunnelBufSize = 64;

  char tunnel_read_buf_[kTunnelBufSize] = {};
  bool tunnel_write_outstanding_ = false;

  char tunnel_write_buf_1_[kTunnelBufSize] = {};
  char tunnel_write_buf_2_[kTunnelBufSize] = {};
  mjlib::base::string_span current_tunnel_write_buf_{tunnel_write_buf_1_};
  mjlib::base::string_span next_tunnel_write_buf_{tunnel_write_buf_2_};
  bool stream_write_outstanding_ = false;

  bool tunnel_polling_enabled_ = false;
  bool onboard_spi_available_ = false;

  std::optional<DigitalOut> i2c_pullup_dout_;
  const aux::AuxHardwareConfig hw_config_;
  const std::array<DMA_Channel_TypeDef*, 4> dma_channels_;
  const SpiDefault spi_default_;

  std::array<SampleType, static_cast<int>(SampleType::kLastEntry)> start_sample_types_ = {};
  std::array<SampleType, static_cast<int>(SampleType::kLastEntry)> finish_sample_types_ = {};
};

}
