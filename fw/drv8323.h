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

#include "PinNames.h"

#include "mjlib/base/visitor.h"

#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/measured_hw_rev.h"
#include "fw/millisecond_timer.h"
#include "fw/motor_driver.h"
#include "fw/moteus_hw.h"

namespace moteus {

class Drv8323 : public MotorDriver {
 public:
  struct Options {
    PinName mosi = NC;
    PinName miso = NC;
    PinName sck = NC;
    PinName cs = NC;

    PinName enable = NC;
    PinName fault = NC;
    PinName hiz = NC;
  };

  Drv8323(mjlib::micro::Pool*,
          mjlib::micro::PersistentConfig* persistent_config,
          mjlib::micro::TelemetryManager* telemetry_manager,
          MillisecondTimer* timer,
          const Options&);
  ~Drv8323();

  // Turn on or off the driver.  When turning it on, all SPI
  // parameters are set from configuration and in this case it may not
  // be invoked from an interrupt context.
  //
  // Return true for success, false for failure.
  bool Enable(bool) override;

  void Power(bool) override;
  bool fault() override;

  void PollMillisecond();

  struct Status {
    // Fault Status Register 1
    bool fault = false;
    bool vds_ocp = false;  // VDS monitor overcurrent fault
    bool gdf = false;      // gate drive fault
    bool uvlo = false;     // undervoltage lockout fault
    bool otsd = false;     // overtemperature shutdown
    bool vds_ha = false;   // vds overcurrent fault on A high
    bool vds_la = false;   // vds overcurrent fault on A low
    bool vds_hb = false;   // vds overcurrent fault on B high
    bool vds_lb = false;   // vds overcurrent fault on B low
    bool vds_hc = false;   // vds overcurrent fault on C high
    bool vds_lc = false;   // vds overcurrent fault on C low

    uint16_t fsr1 = 0;

    // Fault Status Register 2
    bool sa_oc = false;   // overcurrent on phase A sense amp
    bool sb_oc = false;   // overcurrent on phase B sense amp
    bool sc_oc = false;   // overcurrent on phase C sense amp
    bool otw = false;     // overtemperature warning
    bool cpuv = false;    // charge pump undervoltage fault
    bool vgs_ha = false;  // gate drive fault on A high
    bool vgs_la = false;  // gate drive fault on A low
    bool vgs_hb = false;  // gate drive fault on B high
    bool vgs_lb = false;  // gate drive fault on B low
    bool vgs_hc = false;  // gate drive fault on C high
    bool vgs_lc = false;  // gate drive fault on C low

    uint16_t fsr2 = 0;

    // Whether a fault was signaled over the hard-line.
    bool fault_line = false;

    // Whether the motors are powered.
    bool power = false;

    // Whether the enable line is powered.
    bool enabled = false;

    // Bitmask of configuration registers which could not be set.
    uint8_t fault_config = 0;

    uint16_t config_count = 0;
    uint16_t status_count = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(fault));
      a->Visit(MJ_NVP(vds_ocp));
      a->Visit(MJ_NVP(gdf));
      a->Visit(MJ_NVP(uvlo));
      a->Visit(MJ_NVP(otsd));
      a->Visit(MJ_NVP(vds_ha));
      a->Visit(MJ_NVP(vds_la));
      a->Visit(MJ_NVP(vds_hb));
      a->Visit(MJ_NVP(vds_lb));
      a->Visit(MJ_NVP(vds_hc));
      a->Visit(MJ_NVP(vds_lc));

      a->Visit(MJ_NVP(fsr1));

      a->Visit(MJ_NVP(sa_oc));
      a->Visit(MJ_NVP(sb_oc));
      a->Visit(MJ_NVP(sc_oc));
      a->Visit(MJ_NVP(otw));
      a->Visit(MJ_NVP(cpuv));
      a->Visit(MJ_NVP(vgs_ha));
      a->Visit(MJ_NVP(vgs_la));
      a->Visit(MJ_NVP(vgs_hb));
      a->Visit(MJ_NVP(vgs_lb));
      a->Visit(MJ_NVP(vgs_hc));
      a->Visit(MJ_NVP(vgs_lc));

      a->Visit(MJ_NVP(fsr2));

      a->Visit(MJ_NVP(fault_line));
      a->Visit(MJ_NVP(power));
      a->Visit(MJ_NVP(enabled));

      a->Visit(MJ_NVP(fault_config));
      a->Visit(MJ_NVP(config_count));
      a->Visit(MJ_NVP(status_count));
    }
  };

  const Status* status() const;

  enum PwmMode {
    k6x = 0x0,
    k3x = 0x1,
    k1x = 0x2,
    kIndependent = 0x3,
    kNumPwmModes = 4
  };

  enum OcpMode {
    kLatchedFault,
    kAutomaticRetry,
    kReportNoAction,
    kIgnored,
    kNumOcpModes,
  };

  // The default values here will be assigned with PersistentConfig is
  // instructed to reset to defaults, so they should be appropriate
  // for the application.
  struct Config {
    // Driver Control Register
    bool dis_cpuv = false;  // Charge pump UVLO fault is disabled
    bool dis_gdf = false;   // Gate drive fault is disabled
    bool otw_rep = false;   // OTW is reported on nFAULT

    PwmMode pwm_mode = PwmMode::k3x;

    bool pwm_1x_asynchronous = false;
    bool pwm_1x_dir = false;


    // Gate Drive HS Register

    // hw rev 7 boards use a drv8353, which is sensitive to damage
    // ringing on the gate drives.  This version requires lower gate
    // drive strength to avoid damage.  hw rev 8 boards have a better
    // layout and an additional gate drive resistor which allows them
    // to go higher.
    uint16_t idrivep_hs_ma =
        g_measured_hw_family == 0 ?
         ((g_measured_hw_rev <= 6) ? 370 :
          (g_measured_hw_rev <= 7) ? 50 :
          100) :
        g_measured_hw_family == 1 ?
         150 :
        invalid_int();
    uint16_t idriven_hs_ma =
        g_measured_hw_family == 0 ?
         ((g_measured_hw_rev <= 6) ? 740 :
          (g_measured_hw_rev <= 7) ? 100 :
          200) :
        g_measured_hw_family == 1 ?
         300 :
        invalid_int();


    // Gate Drive LS Register
    bool cbc = true;  // Cycle-by cycle operation.
    uint16_t tdrive_ns = 1000;  // peak gate-current drive time
    uint16_t idrivep_ls_ma =
        g_measured_hw_family == 0 ?
         ((g_measured_hw_rev <= 6) ? 370 :
          (g_measured_hw_rev <= 7) ? 50 :
          100) :
        g_measured_hw_family == 1 ?
         150 :
        invalid_int();
    uint16_t idriven_ls_ma =
        g_measured_hw_family == 0 ?
         ((g_measured_hw_rev <= 6) ? 740 :
          (g_measured_hw_rev <= 7) ? 100 :
          200) :
        g_measured_hw_family == 1 ?
         300 :
        invalid_int();


    // OCP Control Register
    bool tretry = false;  // false = 4ms, true = 50us
    uint16_t dead_time_ns =
        g_measured_hw_family == 0 ?
         ((g_measured_hw_rev <= 6) ? 50 :
          (g_measured_hw_rev <= 7) ? 200 :
          50) :
        g_measured_hw_family == 1 ?
         50 :
        invalid_int();
    OcpMode ocp_mode = OcpMode::kLatchedFault;
    uint8_t ocp_deg_us = 4;  // valid options of 2, 4, 6, 8

    // hw rev 6 boards and later use a FET with roughly double the
    // Rdson.  We set a threshold that will trip only if we get well
    // over the rated 100A limit.
    uint16_t vds_lvl_mv =
        g_measured_hw_family == 0 ?
         ((g_measured_hw_rev <= 5) ? 260 :
          (g_measured_hw_rev <= 7) ? 450 :
          700) :
        g_measured_hw_family == 1 ?
         700 :
        invalid_int();


    // CSA Control Register
    bool csa_fet = false;
    bool vref_div = true;  // reference voltage is VREF / 2 (bi-directional)
    bool ls_ref = false;  // false = OCP for low side is measured SHx to SPx
                          // true = OCP is measured SHx to SNx
    uint8_t csa_gain = 20;  // 5, 10, 20, 40 are options
    bool dis_sen = false;  // if true, disable sense overcurrent fault

    uint16_t sen_lvl_mv = 500;  // OCP level, (250, 500, 750, 1000)


    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(dis_cpuv));
      a->Visit(MJ_NVP(dis_gdf));
      a->Visit(MJ_NVP(otw_rep));
      a->Visit(MJ_NVP(pwm_mode));
      a->Visit(MJ_NVP(pwm_1x_asynchronous));
      a->Visit(MJ_NVP(pwm_1x_dir));

      a->Visit(MJ_NVP(idrivep_hs_ma));
      a->Visit(MJ_NVP(idriven_hs_ma));

      a->Visit(MJ_NVP(cbc));
      a->Visit(MJ_NVP(tdrive_ns));
      a->Visit(MJ_NVP(idrivep_ls_ma));
      a->Visit(MJ_NVP(idriven_ls_ma));

      a->Visit(MJ_NVP(tretry));
      a->Visit(MJ_NVP(dead_time_ns));
      a->Visit(MJ_NVP(ocp_mode));
      a->Visit(MJ_NVP(ocp_deg_us));
      a->Visit(MJ_NVP(vds_lvl_mv));

      a->Visit(MJ_NVP(csa_fet));
      a->Visit(MJ_NVP(vref_div));
      a->Visit(MJ_NVP(ls_ref));
      a->Visit(MJ_NVP(csa_gain));
      a->Visit(MJ_NVP(dis_sen));
      a->Visit(MJ_NVP(sen_lvl_mv));
    }

    static int invalid_int() {
      MJ_ASSERT(false);
      return 0;
    }
  };

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}

namespace mjlib {
namespace base {

template <>
struct IsEnum<moteus::Drv8323::PwmMode> {
  static constexpr bool value = true;

  using P = moteus::Drv8323::PwmMode;
  static std::array<std::pair<P, const char*>,
                    static_cast<int>(P::kNumPwmModes)> map() {
    return { {
        { P::k6x, "6x" },
        { P::k3x, "3x" },
        { P::k1x, "1x" },
        { P::kIndependent, "independent" },
            } };
  }
};

template <>
struct IsEnum<moteus::Drv8323::OcpMode> {
  static constexpr bool value = true;

  using O = moteus::Drv8323::OcpMode;
  static std::array<std::pair<O, const char*>,
                    static_cast<int>(O::kNumOcpModes)> map() {
    return { {
        { O::kLatchedFault, "latched_fault" },
        { O::kAutomaticRetry, "automatic_retry" },
        { O::kReportNoAction, "report_no_action" },
        { O::kIgnored, "ignored" },
    } };
  }
};

}
}
