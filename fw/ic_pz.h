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

#include <atomic>

#include "mjlib/base/inplace_function.h"

#include "fw/millisecond_timer.h"
#include "fw/stm32_spi.h"

namespace moteus {
class IcPz {
 public:
  struct Options : Stm32Spi::Options {
    uint8_t resolution_bits = 24;
  };

  IcPz(const Options& options,
       MillisecondTimer* timer)
      : spi_([&]() {
               auto options_copy = options;
               options_copy.width = 8;
               options_copy.mode = 0;
               return options_copy;
             }()),
        timer_(timer) {
  }

  void SetExtraRead(uint8_t bitfield) {
    extra_read_ = bitfield;
  }

  // Issue the given command.
  void StartCommand(uint8_t cmd) {
    const auto mode = mode_.load();
    // We can't do anything here.
    if (mode == kWarmup) { return; }

    ExitOperating();

    tx_buffer_[0] = kSpiWriteCommand;
    tx_buffer_[1] = cmd;
    DmaTransfer(2);

    command_active_ = true;

    EnterOperating();
  }

  void ReadRegisters(uint8_t reg, mjlib::base::string_span output) {
    const auto mode = mode_.load();
    if (mode == kWarmup) { return; }

    ExitOperating();

    tx_buffer_[0] = kSpiReadRegisters;
    tx_buffer_[1] = reg;
    tx_buffer_[2] = 0;
    MJ_ASSERT((output.size() + 3) < static_cast<int>(sizeof(tx_buffer_)));
    std::memset(&tx_buffer_[3], 0, output.size());

    DmaTransfer(3 + output.size());

    std::memcpy(&output[0], &rx_buffer_[3], output.size());

    EnterOperating();
  }

  void WriteRegisters(uint8_t reg, std::string_view data) {
    const auto mode = mode_.load();
    if (mode == kWarmup) { return; }

    ExitOperating();

    tx_buffer_[0] = kSpiWriteRegisters;
    tx_buffer_[1] = reg;
    MJ_ASSERT((data.size() + 2) < static_cast<int>(sizeof(tx_buffer_)));
    std::memcpy(&tx_buffer_[2], &data[0], data.size());
    DmaTransfer(2 + data.size());

    EnterOperating();
  }

  void PollMillisecond() {
    const auto mode = mode_.load();
    switch (mode) {
      case kWarmup: {
        if (timer_->read_ms() < 15) { return; }
        Initialize();
        mode_.store(kOperating);
        break;
      }
      case kOperating: {
        DoOperating();
        break;
      }
      case kCommand: {
        break;
      }
    }
  }

  void DoOperating() {
    if ((timer_->read_ms() % 10) != 0) { return; }

    if (command_active_) {
      ExitOperating();

      tx_buffer_[0] = kSpiReadRegisters;
      tx_buffer_[1] = 0x76;
      tx_buffer_[2] = 0x00;
      tx_buffer_[3] = 0x00;
      tx_buffer_[4] = 0x00;

      DmaTransfer(5);

      status_.cmd_stat = rx_buffer_[3];
      status_.cmd = rx_buffer_[4];

      if (status_.cmd == 0x00) {
        command_active_ = false;
      }
      EnterOperating();
    }

    if (extra_read_ & 0x01) {
      ExitOperating();

      tx_buffer_[0] = kSpiReadDiagnosis;
      tx_buffer_[1] = 0x00;
      tx_buffer_[2] = 0x00;
      tx_buffer_[3] = 0x00;
      tx_buffer_[4] = 0x00;

      DmaTransfer(5);

      status_.dig_sat = (rx_buffer_[1] & 0x01) ? true : false;
      status_.led_cur_low = (rx_buffer_[1] & 0x02) ? true : false;
      status_.temp_not_steady = (rx_buffer_[1] & 0x04) ? true : false;
      status_.vddio_low = (rx_buffer_[1] & 0x08) ? true : false;
      status_.interp_err = (rx_buffer_[1] & 0x10) ? true : false;
      status_.abz_not_ready = (rx_buffer_[1] & 0x40) ? true : false;
      status_.uvw_not_ready = (rx_buffer_[1] & 0x80) ? true : false;

      status_.pos_alpha = (rx_buffer_[2] & 0x01) ? true : false;
      status_.pos_omega = (rx_buffer_[2] & 0x02) ? true : false;
      status_.dig_not_steady = (rx_buffer_[2] & 0x04) ? true : false;
      status_.prc_sync_failed = (rx_buffer_[2] & 0x08) ? true : false;
      status_.ana_bound = (rx_buffer_[2] & 0x10) ? true : false;
      status_.dig_bound = (rx_buffer_[2] & 0x20) ? true : false;
      status_.temp1 = (rx_buffer_[2] & 0x40) ? true : false;
      status_.temp2 = (rx_buffer_[2] & 0x80) ? true : false;

      status_.adi = rx_buffer_[3];
      status_.adi = rx_buffer_[4];

      EnterOperating();
    }

    if (extra_read_ & 0x02) {
      ExitOperating();

      WriteRegister(0x40, 0x01);

      tx_buffer_[0] = kSpiReadRegisters;
      tx_buffer_[1] = kAiPhases;
      tx_buffer_[2] = 0;
      memset(&tx_buffer_[3], 0, 4);

      DmaTransfer(3 + 4);

      const int16_t ai_phases_int =
          static_cast<int16_t>((rx_buffer_[4] << 8) |
                               (rx_buffer_[3] << 0)) >> 6;

      status_.ai_phases = static_cast<float>(ai_phases_int) / 512.0f * 180.0f;

      const int16_t ai_scales_int =
          static_cast<int16_t>((rx_buffer_[6] << 8) |
                               (rx_buffer_[5] << 0)) >> 6;

      status_.ai_scales = static_cast<float>(ai_scales_int) / 1820.0f + 1.0f;

      EnterOperating();
    }
  }

  struct Status {
    bool active = false;
    bool err = false;
    bool warn = false;
    uint32_t value = 0;
    uint8_t nonce = 0;
    uint8_t cmd = 0;
    uint8_t cmd_stat = 0;

    float ai_phases = 0.0;
    float ai_scales = 0.0;

    bool dig_sat = false;
    bool led_cur_low = false;
    bool temp_not_steady = false;
    bool vddio_low = false;
    bool interp_err = false;
    bool abz_not_ready = false;
    bool uvw_not_ready = false;

    bool pos_alpha = false;
    bool pos_omega = false;
    bool dig_not_steady = false;
    bool prc_sync_failed = false;
    bool ana_bound = false;
    bool dig_bound = false;
    bool temp1 = false;
    bool temp2 = false;

    uint8_t adi = 0;
    uint8_t dig = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(active));
      a->Visit(MJ_NVP(err));
      a->Visit(MJ_NVP(warn));
      a->Visit(MJ_NVP(value));
      a->Visit(MJ_NVP(nonce));
      a->Visit(MJ_NVP(cmd));
      a->Visit(MJ_NVP(cmd_stat));
      a->Visit(MJ_NVP(ai_phases));
      a->Visit(MJ_NVP(ai_scales));
      a->Visit(MJ_NVP(dig_sat));
      a->Visit(MJ_NVP(led_cur_low));
      a->Visit(MJ_NVP(temp_not_steady));
      a->Visit(MJ_NVP(vddio_low));
      a->Visit(MJ_NVP(interp_err));
      a->Visit(MJ_NVP(abz_not_ready));
      a->Visit(MJ_NVP(uvw_not_ready));
      a->Visit(MJ_NVP(pos_alpha));
      a->Visit(MJ_NVP(pos_omega));
      a->Visit(MJ_NVP(dig_not_steady));
      a->Visit(MJ_NVP(prc_sync_failed));
      a->Visit(MJ_NVP(ana_bound));
      a->Visit(MJ_NVP(dig_bound));
      a->Visit(MJ_NVP(temp1));
      a->Visit(MJ_NVP(temp2));
      a->Visit(MJ_NVP(adi));
      a->Visit(MJ_NVP(dig));
    }
  };

  void ISR_StartSample() MOTEUS_CCM_ATTRIBUTE {
    const auto mode = mode_.load();
    if (mode != kOperating) { return; }
    if (active_.load()) { return; }

    StartDma(6);
    active_.store(true);
  }

  void ISR_MaybeFinishSample() MOTEUS_CCM_ATTRIBUTE {
    if (!active_.load()) { return; }
    if (!spi_.is_dma_finished()) { return; }

    spi_.finish_dma_transfer();

    status_.active = true;
    status_.value =
        (static_cast<uint32_t>(rx_buffer_[1]) << 16) |
        (static_cast<uint32_t>(rx_buffer_[2]) << 8) |
        (static_cast<uint32_t>(rx_buffer_[3]) << 0);
    status_.err = (rx_buffer_[4] & 0x80) ? true : false;
    status_.warn = (rx_buffer_[4] & 0x40) ? true : false;
    status_.nonce += 1;
    active_.store(false);

    // TODO: verify CRC.
  }

  const Status& status() { return status_; }
  Status status_;

 private:
  struct ConfigValue {
    uint8_t bank = 0;
    uint8_t address = 0;
    uint8_t value = 0;
  };

  // Bank, address, desired value
  static constexpr ConfigValue kDesiredRegisters[] = {
    { 0x00, 0x01, 0x01, },  // PORTX_DIS=0/0/0  VDDIOSEL=01 (3.3V)
    { 0x07, 0x08, 0x18, },  // SPI_ST_PDL=24 (24 single turn position bits)
    { 0x07, 0x09, 0x00, },  // SPI_MT_PDL=0 (0 multiturn position bits)
  };

  template <typename T>
  static uint32_t u32(T value) {
    return reinterpret_cast<uint32_t>(value);
  }

  void ExitOperating() {
    mode_.store(kCommand);
    while (active_.load());
  }

  void Initialize() {
    for (const auto& config_value : kDesiredRegisters) {
      WriteConfigValue(config_value);
    }

    PopulateOperatingBuffer();
  }

  void EnterOperating() {
    PopulateOperatingBuffer();
    mode_.store(kOperating);
  }

  void PopulateOperatingBuffer() {
    // All done.  From here on out, our transmit frame will always be
    // the same.
    tx_buffer_[0] = kSpiReadPosition;
    tx_buffer_[1] = 0x00;
    tx_buffer_[2] = 0x00;
    tx_buffer_[3] = 0x00;
    tx_buffer_[4] = 0x00;
    tx_buffer_[5] = 0x00;
  }

  void WriteConfigValue(const ConfigValue& config_value) {
    WriteRegister(0x40, config_value.bank);
    WriteRegister(config_value.address, config_value.value);
  }

  void WriteRegister(uint8_t address, uint8_t value) {
    tx_buffer_[0] = 0xCF;
    tx_buffer_[1] = address;
    tx_buffer_[2] = value;

    DmaTransfer(3);
  }

  void DmaTransfer(int size) {
    StartDma(size);
    spi_.finish_dma_transfer();
  }

  void StartDma(int size) MOTEUS_CCM_ATTRIBUTE {
    spi_.start_dma_transfer(
        std::string_view(
            reinterpret_cast<const char*>(&tx_buffer_[0]), size),
        mjlib::base::string_span(
            reinterpret_cast<char *>(&rx_buffer_[0]), size));
  }

  uint8_t tx_buffer_[16] = {};
  uint8_t rx_buffer_[16] = {};

  Stm32Spi spi_;
  MillisecondTimer* const timer_;

  bool command_active_ = false;
  uint8_t extra_read_ = 0;

  enum Mode {
    kWarmup,
    kOperating,
    kCommand,
  };
  // Only written from the program context.
  std::atomic<Mode> mode_{kWarmup};

  ////////////////////////////////////
  /// To be accessed from ISR context.

  // True if a position read is currently outstanding.  Only written
  // from an ISR context.
  std::atomic<bool> active_{false};

  ////////////////////////////////////
  /// Constants
  static constexpr uint8_t kSpiReadRegisters = 0x81;
  static constexpr uint8_t kSpiWriteRegisters = 0xCF;
  static constexpr uint8_t kSpiReadPosition = 0xA6;
  static constexpr uint8_t kSpiWriteCommand = 0xD9;
  static constexpr uint8_t kSpiReadDiagnosis = 0x9C;

  static constexpr uint8_t kAiPhases = 0x28;  // bank 1
};
}
