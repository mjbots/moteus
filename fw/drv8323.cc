// Copyright 2018-2022 Josh Pieper, jjp@pobox.com.
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

#include "fw/drv8323.h"

#include <functional>

#include "mbed.h"
#include "pinmap.h"

#include "mjlib/base/inplace_function.h"

#include "fw/ccm.h"
#include "fw/moteus_hw.h"
#include "fw/stm32_bitbang_spi.h"

namespace micro = mjlib::micro;

namespace moteus {

namespace {
constexpr int kPollRate = 10;
}

class Drv8323::Impl {
 public:
  Impl(micro::PersistentConfig* config,
       micro::TelemetryManager* telemetry_manager,
       MillisecondTimer* timer,
       const Options& options)
      : timer_(timer),
        spi_(
            timer,
            [&]() {
              Stm32BitbangSpi::Options out;
              out.mosi = options.mosi;
              out.miso = options.miso;
              out.sck = options.sck;
              out.cs = options.cs;
              // I observed 2MHz being unreliable with the built-in
              // pullup on MISO, but 1MHz seemed OK.  Thus, lets just do
              // a bit lower for safety.

              // Silk 4.1 and below lacked a pullup resistor on MISO.
              // Newer versions have a pullup resistor and can go
              // faster.
              out.frequency = (g_measured_hw_family == 0 &&
                               g_measured_hw_rev <= 3) ? 500000 : 1000000;

              return out;
            }()),
        enable_(options.enable, 0),
        hiz_(options.hiz, 0),
        fault_(options.fault, PullUp) {
    // We need to enable the built-in pullup on the MISO pin, but
    // mbed's SPI class provides no mechanism to do that. :( Thus we
    // manage it manually here.
    pin_mode(options.miso, PullUp);

    config->Register("drv8323_conf", &config_,
                     std::bind(&Impl::HandleConfigUpdate, this));
    status_update_ = telemetry_manager->Register("drv8323", &status_);
  }

  bool Enable(bool value) {
    const bool old_enable = enable_cache_ != 0;
    enable_ = enable_cache_ = value ? 1 : 0;

    if (value) {
      if (!old_enable) {
        // We need to wait 1ms before the chip will be ready to
        // operate when first enabled.
        timer_->wait_us(1000);
      }
      WriteConfig();
      if (!Calibrate()) { return false; }
    } else {
      status_.fault_config = 0;
    }

    return true;
  }

  void Power(bool value) {
    hiz_.write(value ? 1 : 0);
  }

  uint16_t Read(int reg) {
    const uint16_t result = spi_.write(0x8000 | (reg << 11)) & 0x7ff;
    timer_->wait_us(1);
    return result;
  }

  void Write(int reg, uint16_t value) {
    spi_.write((reg << 11) | (value & 0x7ff));
    timer_->wait_us(1);
  }

  void PollMillisecond() {
    loop_count_++;
    if (loop_count_ < kPollRate) { return; }

    loop_count_ = 0;

    auto& s = status_;

    s.fault_line = fault_.read() == 0;
    s.power = (hiz_.read() != 0);
    s.enabled = (enable_cache_ != 0);

    if (enable_cache_ == 0) {
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

    s.fsr1 = status[0];

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

    s.fsr2 = status[1];

    s.status_count++;

    status_update_();

    // TODO: At a lower rate, verify that our config still matches
    // what we commanded.
  }

  void HandleConfigUpdate() {
    if (g_measured_hw_family == 0 &&
        g_measured_hw_rev == 7) {
      // hw rev 7 (silk r4.8) can be damaged with higher gate drive
      // strength than this.
      config_.idrivep_hs_ma = std::min<uint16_t>(config_.idrivep_hs_ma, 50);
      config_.idriven_hs_ma = std::min<uint16_t>(config_.idriven_hs_ma, 100);
      config_.idrivep_ls_ma = std::min<uint16_t>(config_.idrivep_ls_ma, 50);
      config_.idriven_ls_ma = std::min<uint16_t>(config_.idriven_ls_ma, 100);
    } else {
      // hw rev 8 (silk 4.10) has improved layout and an additional
      // gate drive resistor, it will likely not be damaged at up to
      // 100/200, and higher may be possible in some situations.
      // Thus, the above limitation is now removed.
    }

    WriteConfig();
  }

  bool Calibrate() {
    // The offset calibration is done by temporarily setting the CAL
    // bits to one.
    const uint16_t old_reg6 = Read(6);

    // None of the cal bits should have been set by us already,
    // however they may briefly be valid during the power-on sequence.
    //
    // Thus we used to assert this, but no longer do.
    //
    // MJ_ASSERT((old_reg6 & 0x1c) == 0);

    spi_.write((6 << 11) | (old_reg6 | 0x1c));

    timer_->wait_us(200);

    // Now unset the cal bits.
    spi_.write((6 << 11) | (old_reg6 & ~0x1c));

    timer_->wait_us(100);

    // Verify that they are now 0.
    const uint16_t new_reg6 = Read(6);
    if ((new_reg6 & 0x1c) != 0) {
      // Error!
      return false;
    }

    return true;
  }

  void WriteConfig() {
    const auto bit = [](bool val, int pos) -> uint16_t {
      return (val ? 1 : 0) << pos;
    };

    const bool drv8323 =
        (g_measured_hw_family == 0 && g_measured_hw_rev <= 6);

    constexpr uint16_t idrivep_table_drv8323[] = {
      10, 30, 60, 80, 120, 140, 170, 190,
      260, 330, 370, 440, 570, 680, 820, 1000,
    };

    constexpr uint16_t idrivep_table_drv8353[] = {
      50, 50, 100, 150, 300, 350, 400, 450,
      550, 600, 650, 700, 850, 900, 950, 1000,
    };

    constexpr uint16_t idriven_table_drv8323[] = {
      20, 60, 120, 160, 240, 280, 340, 380,
      520, 660, 740, 880, 1140, 1360, 1640, 2000,
    };

    constexpr uint16_t idriven_table_drv8353[] = {
      100, 100, 200, 300, 600, 700, 800, 900,
      1100, 1200, 1300, 1400, 1700, 1800, 1900, 2000,
    };

    constexpr uint16_t tdrive_ns_table[] = {
      500, 1000, 2000, 4000,
    };

    constexpr uint16_t dead_time_table[] = {
      50, 100, 200, 400,
    };

    constexpr uint16_t deglitch_table_drv8323[] = {
      2, 4, 6, 8,
    };

    constexpr uint16_t deglitch_table_drv8353[] = {
      1, 2, 4, 8,
    };

    constexpr uint16_t vds_lvl_table_drv8323[] = {
      60, 130, 200, 260, 310, 450, 530, 600,
      680, 750, 940, 1130, 1300, 1500, 1700, 1880,
    };

    constexpr uint16_t vds_lvl_table_drv8353[] = {
      60, 70, 80, 90, 100, 200, 300, 400,
      500, 600, 700, 800, 900, 1000, 1500, 2000,
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
        // OCP_ACT, which needs to be set on the drv8353 so that all 3
        // half-bridges are shut down in response to a fault.  That is
        // the only possible behavior on the drv8323.
        (drv8323 ? 0 : bit(true, 10)) |

        bit(config_.dis_cpuv, 9) |
        bit(config_.dis_gdf, 8) |
        bit(config_.otw_rep, 7) |
        (static_cast<uint16_t>(config_.pwm_mode) << 5) |
        bit(config_.pwm_1x_asynchronous, 4) |
        bit(config_.pwm_1x_dir, 3);

    const uint16_t reg3 =
        (3 << 8) |
        (map_choice(drv8323 ? idrivep_table_drv8323 : idrivep_table_drv8353,
                    config_.idrivep_hs_ma) << 4) |
        (map_choice(drv8323 ? idriven_table_drv8323 : idriven_table_drv8353,
                    config_.idriven_hs_ma) << 0);

    const uint16_t reg4 =
        bit(config_.cbc, 10) |
        (map_choice(tdrive_ns_table, config_.tdrive_ns) << 8) |
        (map_choice(drv8323 ? idrivep_table_drv8323 : idrivep_table_drv8353,
                    config_.idrivep_ls_ma) << 4) |
        (map_choice(drv8323 ? idriven_table_drv8323 : idrivep_table_drv8353,
                    config_.idriven_ls_ma) << 0);

    const uint16_t reg5 =
        bit(config_.tretry, 10) |
        (map_choice(dead_time_table, config_.dead_time_ns) << 8) |
        (static_cast<uint16_t>(config_.ocp_mode) << 6) |
        (map_choice(drv8323 ? deglitch_table_drv8323 : deglitch_table_drv8353,
                    config_.ocp_deg_us) << 4) |
        (map_choice(drv8323 ? vds_lvl_table_drv8323 : vds_lvl_table_drv8353,
                    config_.vds_lvl_mv) << 0);

    const uint16_t reg6 =
        bit(config_.csa_fet, 10) |
        bit(config_.vref_div, 9) |
        bit(config_.ls_ref, 8) |
        (map_choice(csa_gain_table, config_.csa_gain) << 6) |
        bit(config_.dis_sen, 5) |
        (map_choice(sen_lvl_table, config_.sen_lvl_mv) << 0);

    const uint16_t reg7 =
        // This is the CAL_MODE bit, that must be 1 on the drv8353 to
        // have equivalent behavior to the drv8323.
        (drv8323 ? 0 : bit(true, 0));

    const uint16_t regs[] = { 0, 0, reg2, reg3, reg4, reg5, reg6, reg7 };

    // First set all the registers.
    for (int i = 2; i <= 7; i++) {
      Write(i, regs[i]);
    }

    // Then verify that all registers got the value we want.
    uint8_t fault_config = 0;
    for (int i = 2; i <= 7; i++) {
      const auto result = Read(i);

      if (result != regs[i]) {
        fault_config |= (1 << i);
      }
    }

    status_.fault_config = fault_config;
    status_.config_count++;
  }

  MillisecondTimer* const timer_;

  Status status_;
  Config config_;

  Stm32BitbangSpi spi_;
  DigitalOut enable_;
  int32_t enable_cache_ = false;
  DigitalOut hiz_;
  DigitalIn fault_;

  uint16_t loop_count_ = 0;

  mjlib::base::inplace_function<void()> status_update_;
};

Drv8323::Drv8323(micro::Pool* pool,
                 micro::PersistentConfig* persistent_config,
                 micro::TelemetryManager* telemetry_manager,
                 MillisecondTimer* timer,
                 const Options& options)
    : impl_(pool, persistent_config, telemetry_manager, timer, options) {}

Drv8323::~Drv8323() {}

bool Drv8323::Enable(bool value) { return impl_->Enable(value); }
void Drv8323::Power(bool value) { impl_->Power(value); }

bool Drv8323::fault() {
  return impl_->status_.fault_config ||
      ((impl_->enable_.read() != 0) &&
       ((g_measured_hw_family == 0 && g_measured_hw_rev == 3) ?
        // This revision seems to be unable to read the fault line
        // properly.  Thus we get a laggier version over SPI.
        (impl_->status_.fault == 1) :
        (impl_->fault_.read() == 0))
       );
}

void Drv8323::PollMillisecond() { impl_->PollMillisecond(); }
const Drv8323::Status* Drv8323::status() const { return &impl_->status_; }

}
