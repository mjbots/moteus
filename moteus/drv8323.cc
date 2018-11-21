// Copyright 2018 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "moteus/drv8323.h"

#include <functional>

#include "mbed.h"
#include "pinmap.h"

namespace micro = mjlib::micro;

namespace moteus {

namespace {
constexpr int kPollRate = 10;
}

class Drv8323::Impl {
 public:
  Impl(micro::PersistentConfig* config,
       micro::TelemetryManager* telemetry_manager,
       const Options& options)
      : spi_(options.mosi, options.miso, options.sck),
        cs_(options.cs),
        enable_(options.enable, 0),
        hiz_(options.hiz, 0),
        fault_(options.fault, PullUp) {
    // We need to enable the built-in pullup on the MISO pin, but
    // mbed's SPI class provides no mechanism to do that. :( Thus we
    // manage it manually here.
    pin_mode(options.miso, PullUp);

    spi_.format(16, 1);
    // I observed 2MHz being unreliable with the built-in pullup on
    // MISO, but 1MHz seemed OK.  Thus, lets just do a bit lower for
    // safety.
    spi_.frequency(750000);

    config->Register("drv8323_conf", &config_,
                     std::bind(&Impl::HandleConfigUpdate, this));
    status_update_ = telemetry_manager->Register("drv8323", &status_);
  }

  void Enable(bool value) {
    const bool old_enable = enable_.read() != 0;
    enable_ = value ? 1 : 0;

    if (value) {
      if (!old_enable) {
        // We need to wait 1ms before the chip will be ready to
        // operate when first enabled.
        wait_us(1000);
      }
      WriteConfig();
      Calibrate();
    }
  }

  void Power(bool value) {
    hiz_.write(value ? 1 : 0);
  }

  uint16_t Read(int reg) {
    cs_ = 0;
    const uint16_t result = spi_.write(0x8000 | (reg << 11)) & 0x7ff;
    cs_ = 1;
    wait_us(1);
    return result;
  }

  void Write(int reg, uint16_t value) {
    cs_ = 0;
    spi_.write((reg << 11) | (value & 0x7ff));
    cs_ = 1;
    wait_us(1);
  }

  void PollMillisecond() {
    loop_count_++;
    if (loop_count_ < kPollRate) { return; }

    loop_count_ = 0;

    if (enable_.read() == 0) {
      // If we are not enabled, then we can not communicate over SPI.
      return;
    }

    const uint16_t status[2] = {
      Read(0),
      Read(1),
    };

    const auto bit = [&](int reg, int b) {
      return (status[reg] & (1 << b)) != 0;
    };

    auto& s = status_;
    s.fault = bit(0, 10);
    s.vds_ocp = bit(0, 9);
    s.gdf = bit(0, 8);
    s.uvlo = bit(0, 7);
    s.otsd = bit(0, 6);
    s.vds_ha = bit(0, 5);
    s.vds_la = bit(0, 4);
    s.vds_hb = bit(0, 3);
    s.vds_lb = bit(0, 2);
    s.vds_hc = bit(0, 1);
    s.vds_lc = bit(0, 0);

    s.sa_oc = bit(1, 10);
    s.sb_oc = bit(1, 9);
    s.sc_oc = bit(1, 8);
    s.otw = bit(1, 7);
    s.cpuv = bit(1, 6);
    s.vgs_ha = bit(1, 5);
    s.vgs_la = bit(1, 4);
    s.vgs_hb = bit(1, 3);
    s.vgs_lb = bit(1, 2);
    s.vgs_hc = bit(1, 1);
    s.vgs_lc = bit(1, 0);

    s.fault_line = fault_.read() == 0;
    s.power = (hiz_.read() != 0);

    s.status_count++;

    status_update_();

    // TODO: At a lower rate, verify that our config still matches
    // what we commanded.
  }

  void HandleConfigUpdate() {
    WriteConfig();
  }

  void Calibrate() {
    // The offset calibration is done by temporarily setting the CAL
    // bits to one.
    const uint16_t old_reg6 = Read(6);

    // None of the cal bits should be set already.
    MJ_ASSERT((old_reg6 & 0x1c) == 0);

    spi_.write((6 << 11) | (old_reg6 | 0x1c));

    wait_us(200);

    // Now unset the cal bits.
    spi_.write((6 << 11) | old_reg6);
  }

  void WriteConfig() {
    const auto bit = [](bool val, int pos) -> uint16_t {
      return (val ? 1 : 0) << pos;
    };

    constexpr uint16_t idrivep_table[] = {
      10, 30, 60, 80, 120, 140, 170, 190,
      260, 330, 370, 440, 570, 680, 820, 1000,
    };

    constexpr uint16_t idriven_table[] = {
      20, 60, 120, 160, 240, 280, 340, 380,
      520, 660, 740, 880, 1140, 1360, 1640, 2000,
    };

    constexpr uint16_t tdrive_ns_table[] = {
      500, 1000, 2000, 4000,
    };

    constexpr uint16_t dead_time_table[] = {
      50, 100, 200, 400,
    };

    constexpr uint16_t deglitch_table[] = {
      2, 4, 6, 8,
    };

    constexpr uint16_t vds_lvl_table[] = {
      60, 130, 200, 260, 210, 450, 530, 600,
      680, 750, 940, 1130, 1300, 1500, 1700, 1880,
    };

    constexpr uint16_t csa_gain_table[] = {
      5, 10, 20, 40,
    };

    constexpr uint16_t sen_lvl_table[] = {
      250, 500, 750, 1000,
    };

    const auto map_choice = [](const auto& table, int ma) -> uint16_t {
      for (uint16_t i = 0; i < sizeof(table) / sizeof(*table); i++) {
        if (ma <= table[i]) { return i; }
      }
      return (sizeof(table) / sizeof(*table)) - 1;
    };


    // Drive Control Register
    const uint16_t reg2 =
        bit(config_.dis_cpuv, 9) |
        bit(config_.dis_gdf, 8) |
        bit(config_.otw_rep, 7) |
        (static_cast<uint16_t>(config_.pwm_mode) << 5) |
        bit(config_.pwm_1x_asynchronous, 4) |
        bit(config_.pwm_1x_dir, 3);

    const uint16_t reg3 =
        (3 << 8) |
        (map_choice(idrivep_table, config_.idrivep_hs_ma) << 4) |
        (map_choice(idriven_table, config_.idriven_hs_ma) << 0);

    const uint16_t reg4 =
        bit(config_.cbc, 10) |
        (map_choice(tdrive_ns_table, config_.tdrive_ns) << 8) |
        (map_choice(idrivep_table, config_.idrivep_ls_ma) << 4) |
        (map_choice(idriven_table, config_.idriven_ls_ma) << 0);

    const uint16_t reg5 =
        bit(config_.tretry, 10) |
        (map_choice(dead_time_table, config_.dead_time_ns) << 8) |
        (static_cast<uint16_t>(config_.ocp_mode) << 6) |
        (map_choice(deglitch_table, config_.ocp_deg_us) << 4) |
        (map_choice(vds_lvl_table, config_.vds_lvl_mv) << 0);

    const uint16_t reg6 =
        bit(config_.csa_fet, 10) |
        bit(config_.vref_div, 9) |
        bit(config_.ls_ref, 8) |
        (map_choice(csa_gain_table, config_.csa_gain) << 6) |
        bit(config_.dis_sen, 5) |
        (map_choice(sen_lvl_table, config_.sen_lvl_mv) << 0);

    const uint16_t regs[] = { 0, 0, reg2, reg3, reg4, reg5, reg6 };

    // First set all the registers.
    for (int i = 2; i <= 6; i++) {
      Write(i, regs[i]);
    }

    // Then verify that all registers got the value we want.
    uint8_t fault_config = 0;
    for (int i = 2; i <= 6; i++) {
      const auto result = Read(i);

      if (result != regs[i]) {
        fault_config |= (1 << i);
      }
    }

    status_.fault_config = fault_config;
    status_.config_count++;
  }

  Status status_;
  Config config_;

  SPI spi_;
  DigitalOut cs_;
  DigitalOut enable_;
  DigitalOut hiz_;
  DigitalIn fault_;

  uint16_t loop_count_ = 0;

  micro::StaticFunction<void()> status_update_;
};

Drv8323::Drv8323(micro::Pool* pool,
                 micro::PersistentConfig* persistent_config,
                 micro::TelemetryManager* telemetry_manager,
                 const Options& options)
    : impl_(pool, persistent_config, telemetry_manager, options) {}

Drv8323::~Drv8323() {}

void Drv8323::Enable(bool value) { impl_->Enable(value); }
void Drv8323::Power(bool value) { impl_->Power(value); }
void Drv8323::PollMillisecond() { impl_->PollMillisecond(); }
const Drv8323::Status* Drv8323::status() const { return &impl_->status_; }

}
