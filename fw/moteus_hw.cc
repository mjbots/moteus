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

#include "fw/moteus_hw.h"

#include "mjlib/base/assert.h"

#include "fw/stm32_bitbang_spi.h"
#include "fw/stm32g4_adc.h"

namespace moteus {

namespace {
//////////////////////////////////////////
// moteus - family 0

// The following silks correspond with the following hardware
// revisions:
//
//  "r1"           - 0
//  "r2"           - 1
//  "r3"           - 2
//  "r4.1"         - 3
//  "r4.2", "r4.3" - 4
//  "r4.4"         - 5
//  "r4.5"         - 6
//  "r4.5b"-"r4.8" - 7
//  "r4.11"        - 8

// The mapping between  and the version pins on the
// board.
constexpr int kFamily0HardwareInterlock[] = {
  -1,  // r1 (never printed for g4)
  -1,  // r2 (never printed for g4)
  -1,  // r3 (never printed for g4)
  0,   // r4.1
  0,   // r4.2/r4.3 (unfortunately, indistinguishable from the interlock)
  1,   // r4.4
  2,   // r4.5
  3,   // r4.5b-r4.8
  4,   // r4.10
};

bool DetectGateDriver(
    MillisecondTimer* timer,
    PinName drv8353_cs,
    PinName drv8353_enable,
    PinName drv8353_mosi,
    PinName drv8353_miso,
    PinName drv8353_sck) {
  DigitalOut enable(drv8353_enable, 1);

  timer->wait_us(1000);

  Stm32BitbangSpi maybe_drv8323(
      timer,
      [&]() {
        Stm32BitbangSpi::Options out;
        out.mosi = drv8353_mosi;
        out.miso = drv8353_miso;
        out.sck = drv8353_sck;
        out.cs = drv8353_cs;

        // We can use a slow speed since this is just a one-time
        // test.
        out.frequency = 500000;
        return out;
      }());
  pin_mode(drv8353_miso, PullUp);
  auto read_reg =
      [&](int reg) {
        timer->wait_us(1);
        return maybe_drv8323.write(0x8000 | (reg << 11)) & 0x7ff;
      };
  bool found = false;
  for (int reg = 2; reg < 6; reg++) {
    const auto value = read_reg(reg);
    if (value != 0x7ff && value != 0) {
      found = true;
      break;
    }
  }


  return found;
}

}

FamilyAndVersion DetectMoteusFamily(MillisecondTimer* timer) {
  FamilyAndVersion result;
  result.family = 0;

  uint8_t family_pullup = 0;
  {
    DigitalIn family_detect_a(PB_10, PullUp);
    DigitalIn family_detect_b(PB_11, PullUp);
    timer->wait_us(10);
    family_pullup =
        (family_detect_a.read() ? 1 : 0) |
        (family_detect_b.read() ? 2 : 0);
  }
  uint8_t family_pulldown = 0;
  {
    DigitalIn family_detect_a(PB_10, PullDown);
    DigitalIn family_detect_b(PB_11, PullDown);
    timer->wait_us(10);
    family_pulldown =
        (family_detect_a.read() ? 1 : 0) |
        (family_detect_b.read() ? 2 : 0);
  }
  const uint8_t family_code = (family_pullup | family_pulldown << 2);

  {
    // Family 2 boards can be identified by the external pull down on
    // PB10 and PB11 being floating.
    if (family_code == 0x02) {
      result.family = 2;
    } else if (family_code == 0x01) {
      // Family 3 boards can be identified by the external pull down
      // on PB11 and PB10 being floating.
      result.family = 3;
    } else if (family_code == 0x03) {
      // If both are floating, then we are family 0 or 1.

      // We check for family 1, "moteus n1", by seeing if we can find a
      // DRV8323 on a chip select that is different from that used on all
      // family 0 boards.
      while (true) {
        timer->wait_ms(2);
        {
          // Ensure that on family 0 boards, the drv8323 will be not
          // selected.
          DigitalOut family0_drv8323_cs(PC_4, 1);

          if (DetectGateDriver(timer,
                               PB_0, // CS
                               PC_14, // enable
                               PC_13, // MOSI,
                               PC_11, // MISO,
                               PC_10) // SCK
              ) {
            result.family = 1;
            break;
          }
        }
        if (result.family == 0) {
          // Verify we are actually on a moteus-r4 by looking for its gate
          // driver.
          if (!DetectGateDriver(timer,
                                PC_4, // CS
                                PA_3, // enable
                                PA_7, // MOSI,
                                PA_6, // MISO,
                                PA_5) // SCK
              ) {
            // If we can detect a gate driver in neither place, loop back
            // and try again after a while.  Maybe the input voltage is
            // just slewing up *really* slowly and our gate driver is not
            // at a voltage level where it will work yet.
            continue;
          }
          // Yes, we are on a r4.
          break;
        }
      }
    } else {
      // Unknown family.  Just die as we don't support whatever it is.
      MJ_ASSERT(false);
    }
  }

  if (result.family == 0) {
    DigitalIn hwrev0(PC_6, PullUp);
    DigitalIn hwrev1(PA_15, PullUp);
    // Previously this was documented as PC_13, however we never
    // pulled it down, and decided to use PC_13 for something else.
    DigitalIn hwrev2(PA_10, PullUp);

    timer->wait_ms(1);

    const uint8_t this_hw_pins =
        0x07 & (~(hwrev0.read() |
                  (hwrev1.read() << 1) |
                  (hwrev2.read() << 2)));
    result.hw_pins = this_hw_pins;
    // result.hw_version is signed so that an unmatched encoding can
    // travel as -1 to the compatibility check in moteus.cc; assign
    // the lambda result directly rather than going through a uint8_t
    // local.
    result.hw_version =
        [&]() -> int {
          int i = 0;
          for (auto rev_pins : kFamily0HardwareInterlock) {
            if (rev_pins == this_hw_pins) { return i; }
            i++;
          }
          return -1;
        }();
  } else if (result.family == 1 || result.family == 2 || result.family == 3) {
    __HAL_RCC_ADC12_CLK_ENABLE();

    DisableAdc(ADC2);

    // Our board version is programmed with a high impedance voltage
    // divider, so we need to custom program the ADC to get a large
    // sample time and to ensure the prescaler is set to a usable value.

    ADC12_COMMON->CCR = (7 << ADC_CCR_PRESC_Pos);  // 16x prescaler

    ADC2->SMPR2 = (0x7 << ADC_SMPR2_SMP17_Pos);  // 640.5 ADC clock cycles
    ADC2->SQR1 =
        (17 << ADC_SQR1_SQ1_Pos) |  // IN17
        (0 << ADC_SQR1_L_Pos);  // length 1

    // Use software trigger for one-time measurement during detection
    EnableAdc(timer, ADC2, 16, 0, AdcTriggerMode::kSoftware);

    // Trigger ADC2 using software mode
    ADC2->CR |= ADC_CR_ADSTART;
    while ((ADC2->ISR & ADC_ISR_EOC) == 0);

    const uint16_t this_reading = ADC2->DR << 4;

    if (result.family == 1) {
      if (this_reading < 0x0200) {
        // silk moteus r1.2
        result.hw_version = 0;
      } else if (this_reading > 0xfe00) {
        // silk moteus r1.3
        result.hw_version = 1;
      } else {
        // Unknown version.
        result.hw_version = -1;
      }
    } else if (result.family == 2) {
      // We don't have any uniformly specified versions yet.
      result.hw_version = 0;
    } else if (result.family == 3) {
      result.hw_version = 0;
    }
  } else {
    MJ_ASSERT(false);
  }

  return result;
}

namespace {
PinName unsupported() {
  mbed_die();
  return NC;
}
}

MoteusHwPins FindHardwarePins(FamilyAndVersion fv) {
  MoteusHwPins result;

  const auto hv = fv.hw_version;

  if (fv.family == 0) {
    result.vsense =
        (hv <= 4 ? PA_8 :
         hv >= 5 ? PB_12_ALT0 :
         unsupported());

    result.msense =
        // Note, the hv <=3 versions don't actually have a motor sense
        // ADC at all.  So we just pick it the same as the other
        // temperature sense so that things don't get broken.
        (hv <= 3 ? PA_9 :
         hv == 4 ? PB_12 :
         hv >= 5 ? PA_8 :
         unsupported());

    result.vsense_adc_scale =
        (hv <= 5 ? 0.00884f : 0.017947f);

    result.drv8323_enable = PA_3;
    result.drv8323_hiz = PB_7;
    result.drv8323_cs = PC_4;

    result.drv8323_mosi = PA_7;
    result.drv8323_miso = PA_6;
    result.drv8323_sck = PA_5;
    result.drv8323_fault = PB_6;

    result.debug_led1 = PF_0;
    result.power_led = PF_1;

    // We've picked these particular pins so that all 3 channels are
    // one of the "slow" channels so they will have similar analog
    // performance characteristics.
    result.current1 = PB_0_ALT0;
    result.current2 = PB_1;
    result.current3 = PB_2;

    result.as5047_cs = PB_11;

    result.can_td = PA_12;
    result.can_rd = PA_11;

    result.debug1 = PC_14;
    result.debug2 = PC_15;

    result.power_P_l_W = 900.0f;
    result.power_V_l = 30.0f;

    result.power_P_h_W = 400.0f;
    result.power_V_h = 38.0f;

    // moteus-r4
    //
    // TPH2R506PL, absolute maximum junction temperature 175 C.
    // Rds(on) (1.9 mOhm typical at Vgs = 10 V, 4.25 mOhm at 175 C)
    // and Qsw (19 nC) are from the datasheet.
    //
    // The rest were measured on the automated thermal fixture, the
    // constants from one complete dataset of 2026-10-14/15.  The losses
    // (switching scale, series resistance, switching energies) and the
    // junction's fast stage were fitted together, from fixed current
    // runs, locked rotor pulses to 57 A at 36 V, 15 and 60 kHz (24 and
    // 44 V, 30 and 45 kHz held out), and still air calorimetry, which
    // also fixes the camera's lag; still air resistance and the bridge's
    // overhead from switching only runs.  The fitted voltage-current and
    // dead time diode energies are negligible and left at zero.  The
    // copper of each FET is coupled to its neighbours in the column.
    // The network is the worst combination of the best-fit board
    // model's coolings, with the copper made 10% worse than fitted for
    // unit variation and the junction stages 10% worse; it is run bare
    // and with a heat spreader on the pads.  Chosen with the mfg-moteus
    // fixture tools so that the hotter reads at or above every measured
    // cooling and simulated fans and heat spreaders on the FETs (made
    // 10% worse), in steady state and in the response to a step of
    // loss, for any mix of the losses the phases produce.  Of the
    // networks that do, this one reads lowest for a rotating load and
    // the worst locked rotor load together.  It is checked in closed
    // loop with the default derating (fault 165 C) from any starting
    // temperature and pulse: the true junction stays below 165 C.  The
    // FETs are in one column, 0, 1, 3, 2, 4, 5, with the thermistor
    // nearest FET 4.
    result.fet_thermal.rds_on_25C_ohm = 1.9e-3f;
    result.fet_thermal.rds_on_tempco_per_C = 0.0085f;
    result.fet_thermal.gate_switch_charge_coulomb = 19e-9f;
    result.fet_thermal.r_junction_pad_C_per_W =
        { 7.556f, 6.748f, 7.556f, 6.748f, 7.556f, 6.748f };
    result.fet_thermal.tau_junction_pad_s =
        { 0.1706f, 0.1524f, 0.1706f, 0.1524f, 0.1706f, 0.1524f };
    result.fet_thermal.g_pad_thermistor_W_per_K =
        { 9e-06f, 9e-06f, 0.0362f, 0.004103f, 0.06861f, 9e-06f };
    result.fet_thermal.g_pad_ambient_W_per_K =
        { 0.02212f, 0.01284f, 0.01402f, 0.01083f, 0.004174f, 0.02586f };
    result.fet_thermal.c_pad_J_per_K = 0.2675f;
    result.fet_thermal.c_thermistor_J_per_K = 0.3749f;
    result.fet_thermal.c_board_J_per_K = 3.088f;
    result.fet_thermal.g_thermistor_board_W_per_K = 0.007327f;
    result.fet_thermal.g_thermistor_ambient_W_per_K = 0.03116f;
    result.fet_thermal.g_board_ambient_W_per_K = 0.1968f;
    result.fet_thermal.thermistor_lag_s = 0.02f;
    result.fet_thermal.g_pad_spreader_W_per_K = 0.01f;
    result.fet_thermal.g_spreader_ambient_W_per_K = 0.02f;
    result.fet_thermal.c_spreader_J_per_K = 10.0f;
    result.fet_thermal.switching_loss_scale = 0.3666f;
    result.fet_thermal.r_extra_ohm = 0.001668f;
    result.fet_thermal.switching_energy_J_per_V = 2.75e-07f;
    result.fet_thermal.switching_energy_J_per_V_A = 0.0f;
    result.fet_thermal.switching_energy_J_per_A2 = 1.262e-08f;
    result.fet_thermal.deadtime_diode_V_s = 0.0f;
    result.fet_thermal.network_couplings = 5;
    result.fet_thermal.coupling_a = { 0, 1, 2, 2, 4 };
    result.fet_thermal.coupling_b = { 1, 3, 3, 4, 5 };
    result.fet_thermal.coupling_W_per_K =
        { 0.1022f, 0.06614f, 0.1878f, 0.04367f, 0.08841f };
  } else if (fv.family == 1 || fv.family == 2 || fv.family == 3) {
    result.drv8323_enable = PC_14;
    result.drv8323_hiz = PC_15;
    result.drv8323_cs = PB_0;

    result.drv8323_mosi = PC_13;
    result.drv8323_miso = PC_11;
    result.drv8323_sck = PC_10;
    result.drv8323_fault = PB_13;

    result.debug_led1 = PB_15;
    result.power_led = PC_6;

    // Family 1 devices should have all current sense inputs on "fast"
    // channels.
    result.current1 = PA_3;       // ADC1
    result.current2 = PA_6;       // ADC2  // current 2
    result.current3 = PB_1_ALT0;  // ADC3  // current 3


    result.vsense = PA_9;         // ADC5_IN2
    result.tsense = PB_12_ALT0;   // ADC4
    result.msense = PA_8;         // ADC5_IN1

    result.vsense_adc_scale = 0.017947f;

    result.as5047_cs = PB_2;

    result.can_td = PB_6;
    result.can_rd = PB_5;

    result.debug1 = NC;
    result.debug2 = NC;

    if (fv.family == 1) {
      // moteus-n1

      result.power_P_l_W = 2000.0f;
      result.power_V_l = 36.0f;

      result.power_P_h_W = 1000.0f;
      result.power_V_h = 44.0f;

      // TPH2R408QM, absolute maximum junction temperature 175 C.
      // Rds(on) and Qsw are from the datasheet.
      //
      // The rest were measured on the automated thermal fixture, the
      // constants from one complete dataset of 2026-10-15/16.  The
      // losses (switching scale, series resistance, switching and dead
      // time diode energies) and the junction's fast stage were fitted
      // together, from fixed current runs, locked rotor pulses to 57 A
      // at 36 V, 15 and 60 kHz (24 and 51 V, 30 and 45 kHz held out), and
      // still air calorimetry, which also fixes the camera's lag; still
      // air resistance and the bridge's overhead from switching only
      // runs.  The copper of each FET is coupled to its neighbours in
      // the column.  The network is the least cooled best-fit board
      // model, with the copper made 10% worse than fitted for unit
      // variation and each FET's junction stage made as much worse as
      // it needs (29 to 63%); FET 0's tie to the thermistor is capped
      // at 0.1 W/K, so that it does not follow the thermistor's lag.
      // It is run bare and with a heat spreader on the pads.  Chosen
      // with the mfg-moteus fixture tools so that the hotter reads at
      // or above every measured cooling and simulated fans and heat
      // spreaders on the FETs (made 10% worse), in steady state and in
      // the response to a step of loss, for any mix of the losses the
      // phases produce.  Of the networks that do, this one reads lowest
      // for a rotating load.  It is checked in closed loop with the
      // default derating (fault 165 C) from any starting temperature
      // and pulse: the true junction stays below 165 C.  The FETs are
      // in one column, 0, 1, 3, 2, 4, 5, with the thermistor next to
      // FETs 0 and 1.
      result.fet_thermal.rds_on_25C_ohm = 0.0019f;
      result.fet_thermal.rds_on_tempco_per_C = 0.0053f;
      result.fet_thermal.gate_switch_charge_coulomb = 2.8e-08f;
      result.fet_thermal.r_junction_pad_C_per_W =
          { 6.884f, 9.315f, 7.252f, 10.25f, 8.565f, 11.41f };
      result.fet_thermal.tau_junction_pad_s =
          { 0.2298f, 0.3109f, 0.242f, 0.3423f, 0.2859f, 0.3808f };
      result.fet_thermal.g_pad_thermistor_W_per_K =
          { 0.1f, 4.536e-05f, 0.001953f, 0.02589f, 9e-06f, 9e-06f };
      result.fet_thermal.g_pad_ambient_W_per_K =
          { 0.001897f, 0.02614f, 0.01089f, 0.0005683f, 0.006963f, 0.02242f };
      result.fet_thermal.c_pad_J_per_K = 0.4548f;
      result.fet_thermal.c_thermistor_J_per_K = 0.8431f;
      result.fet_thermal.c_board_J_per_K = 0.5f;
      result.fet_thermal.g_thermistor_board_W_per_K = 0.004178f;
      result.fet_thermal.g_thermistor_ambient_W_per_K = 9e-06f;
      result.fet_thermal.g_board_ambient_W_per_K = 9e-06f;
      result.fet_thermal.thermistor_lag_s = 0.02001f;
      result.fet_thermal.g_pad_spreader_W_per_K = 0.1f;
      result.fet_thermal.g_spreader_ambient_W_per_K = 0.2f;
      result.fet_thermal.c_spreader_J_per_K = 10.0f;
      result.fet_thermal.switching_loss_scale = 0.5247f;
      result.fet_thermal.r_extra_ohm = 0.001761f;
      result.fet_thermal.switching_energy_J_per_V = 3.79e-07f;
      result.fet_thermal.switching_energy_J_per_V_A = 2.669e-09f;
      result.fet_thermal.switching_energy_J_per_A2 = 3.929e-08f;
      result.fet_thermal.deadtime_diode_V_s = 9.453e-08f;
      result.fet_thermal.network_couplings = 5;
      result.fet_thermal.coupling_a = { 0, 1, 2, 2, 4 };
      result.fet_thermal.coupling_b = { 1, 3, 3, 4, 5 };
      result.fet_thermal.coupling_W_per_K =
          { 0.2077f, 0.04999f, 0.1738f, 0.1091f, 0.09596f };
    } else if (fv.family == 2) {
      // moteus-c1

      result.power_P_l_W = 250.0f;
      result.power_V_l = 28.0f;

      result.power_P_h_W = 150.0f;
      result.power_V_h = 41.0f;

      // TPN19008QM, absolute maximum junction temperature 175 C.
      // Rds(on) (14.7 mOhm typical at Vgs = 10 V, about 2.1 times that
      // at 175 C) and Qsw (5.5 nC) are from the datasheet.
      //
      // The rest were measured on the automated thermal fixture, the
      // constants from one complete dataset of 2026-10-13/14.  The
      // losses (switching scale, series resistance, switching and dead
      // time diode energies) and the junction's fast stage were fitted
      // together, from fixed current runs, locked rotor pulses to 20 A
      // at 36 V, 15 and 60 kHz (24 and 51 V, 30 and 45 kHz held out), and
      // still air calorimetry, which also fixes the camera's lag; still
      // air resistance and the bridge's overhead from switching only
      // runs.  The fitted voltage-current energy is negligible and left
      // at zero.  The copper of each FET is coupled to its neighbours
      // in the row.  The network is the least cooled best-fit board
      // model, with the copper made 10% worse than fitted for unit
      // variation and each FET's junction stage made as much worse as
      // it needs (17 to 25%).  It is run bare and with a heat spreader
      // on the pads.  The fit pins FET 0's pad to the thermistor, which it
      // sits next to, but the sensor lags the pad in a pulse; that
      // conductance is cut to 0.1 W/K, which only raises the estimate,
      // so FET 0 reads at or above the camera in 2 s pulses.  Chosen
      // with the mfg-moteus fixture tools so that the hotter reads at
      // or above every measured cooling and simulated fans and heat
      // spreaders on the FETs (made 10% worse), in steady state and in
      // the response to a step of loss, for any mix of the losses the
      // phases produce.  Of the networks that do, this one reads lowest
      // for a rotating load.  It is checked in closed loop with the
      // default derating (fault 165 C) from any starting temperature
      // and pulse: the true junction stays below 165 C.  The FETs are
      // in one row, 5, 4, 2, 3, 1, 0, with the thermistor next to
      // FET 0.
      result.fet_thermal.rds_on_25C_ohm = 0.0147f;
      result.fet_thermal.rds_on_tempco_per_C = 0.0073f;
      result.fet_thermal.gate_switch_charge_coulomb = 5.5e-09f;
      result.fet_thermal.r_junction_pad_C_per_W =
          { 19.15f, 20.89f, 20.12f, 20.89f, 20.12f, 21.97f };
      result.fet_thermal.tau_junction_pad_s =
          { 0.2055f, 0.2242f, 0.2159f, 0.2242f, 0.2159f, 0.2357f };
      result.fet_thermal.g_pad_thermistor_W_per_K =
          { 0.1f, 9.003e-06f, 0.01598f, 0.004717f, 0.003312f, 9e-06f };
      result.fet_thermal.g_pad_ambient_W_per_K =
          { 0.005384f, 0.0118f, 0.008028f, 0.003324f, 0.01115f, 0.01109f };
      result.fet_thermal.c_pad_J_per_K = 0.2281f;
      result.fet_thermal.c_thermistor_J_per_K = 0.4839f;
      result.fet_thermal.c_board_J_per_K = 0.5f;
      result.fet_thermal.g_thermistor_board_W_per_K = 0.0009f;
      result.fet_thermal.g_thermistor_ambient_W_per_K = 0.0002098f;
      result.fet_thermal.g_board_ambient_W_per_K = 0.001165f;
      result.fet_thermal.thermistor_lag_s = 0.02f;
      result.fet_thermal.g_pad_spreader_W_per_K = 0.1f;
      result.fet_thermal.g_spreader_ambient_W_per_K = 0.2f;
      result.fet_thermal.c_spreader_J_per_K = 10.0f;
      result.fet_thermal.switching_loss_scale = 0.4353f;
      result.fet_thermal.r_extra_ohm = 0.001598f;
      result.fet_thermal.switching_energy_J_per_V = 1.1e-07f;
      result.fet_thermal.switching_energy_J_per_V_A = 0.0f;
      result.fet_thermal.switching_energy_J_per_A2 = 6.989e-08f;
      result.fet_thermal.deadtime_diode_V_s = 1.849e-07f;
      result.fet_thermal.network_couplings = 5;
      result.fet_thermal.coupling_a = { 0, 1, 2, 2, 4 };
      result.fet_thermal.coupling_b = { 1, 3, 3, 4, 5 };
      result.fet_thermal.coupling_W_per_K =
          { 0.0321f, 0.03891f, 0.07851f, 0.08665f, 0.07238f };
    } else if (fv.family == 3) {
      // moteus-x1

      result.power_P_l_W = 2000.0f;
      result.power_V_l = 36.0f;

      result.power_P_h_W = 1000.0f;
      result.power_V_h = 44.0f;

      // FDMT80080DC, a Dual Cool 88 package with its source pad exposed
      // on top for a heatsink, absolute maximum junction temperature
      // 150 C.  Rds(on) is the datasheet typical at Vgs = 10 V,
      // 1.06 mOhm, rising to 1.74 mOhm at 125 C.  The datasheet gives
      // no switching charge; Qgd (36 nC) plus about a third of Qgs
      // (69 nC) is the starting point, which the fitted switching loss
      // scale corrects.
      //
      // The rest were measured on the automated thermal fixture on
      // 2026-09-29 through 2026-10-13, the constants from one complete
      // dataset of 2026-10-12/13.  The losses (switching scale, series
      // resistance, switching and dead time diode energies) and the
      // junction's fast stage were fitted together, from fixed current
      // runs, locked rotor pulses to 111 A at 15 and 60 kHz (24 and
      // 54 V, 30 and 45 kHz held out), and still air calorimetry,
      // which also fixes the camera's lag; still air resistance and
      // the bridge's overhead from switching only runs.  The hard
      // switching losses grow faster than the PWM rate, 4e-6 per Hz.
      // The dead time diode term is raised from the fitted 0.33 to
      // 0.7 uV s so the low side pulses' high side partners read at or
      // above the camera at 60 kHz.  The network is the worst
      // combination of the best-fit board model's coolings, with the
      // copper made 10% worse than fitted for unit variation and each
      // FET's junction stage made as much worse as it needs (22 to
      // 44%).  It is run bare and with a heat spreader on the pads.
      // Chosen with the mfg-moteus fixture tools so that the hotter
      // reads at or above every measured cooling and simulated fans
      // and heat spreaders on the FETs (made 10% worse), in steady
      // state and in the response to a step of loss, for any mix of
      // the losses the phases produce.  Of the networks that do, this
      // one reads lowest for a rotating load.  It is checked in closed
      // loop with the default derating (fault 140 C) from any starting
      // temperature and pulse: the true junction stays below 140 C.
      // The FETs are in one row, 5, 4, 2, 3, 1, 0, with the thermistor
      // on FET 0's pad.
      //
      // The fit pins FET 0's pad to the thermistor (45 W/K), as it did
      // on the moteus-n1 and moteus-c1, where that tie is capped at
      // 0.1 W/K because FET 0 otherwise read up to 3 C below the camera
      // in pulses.  Here the thermistor really does sit on that pad,
      // the fitted thermistor lag stays at 0.83 s, and no FET of any
      // pulse reads more than 0.3 C below the camera.  The same cap
      // changes no check result, but lowers the locked rotor current
      // held with a fan from 36 A to 32 A and derates 40-86 A steps up
      // to 2.6 times sooner, so the fitted tie is kept.
      result.fet_thermal.rds_on_25C_ohm = 1.06e-3f;
      result.fet_thermal.rds_on_tempco_per_C = 0.0064f;
      result.fet_thermal.gate_switch_charge_coulomb = 60e-9f;
      result.fet_thermal.r_junction_pad_C_per_W =
          { 4.284f, 3.956f, 4.117f, 3.472f, 4.15f, 4.098f };
      result.fet_thermal.tau_junction_pad_s =
          { 0.1286f, 0.1188f, 0.1236f, 0.1043f, 0.1246f, 0.1231f };
      result.fet_thermal.g_pad_thermistor_W_per_K =
          { 45.0f, 9e-06f, 0.03655f, 0.006089f, 0.001248f, 0.005491f };
      result.fet_thermal.g_pad_ambient_W_per_K =
          { 0.001791f, 0.02806f, 0.01595f, 0.006952f, 0.01484f, 0.01827f };
      result.fet_thermal.c_pad_J_per_K = 0.646f;
      result.fet_thermal.c_thermistor_J_per_K = 0.2996f;
      result.fet_thermal.c_board_J_per_K = 0.7029f;
      result.fet_thermal.g_thermistor_board_W_per_K = 0.1705f;
      result.fet_thermal.g_thermistor_ambient_W_per_K = 0.005227f;
      result.fet_thermal.g_board_ambient_W_per_K = 9.651e-05f;
      result.fet_thermal.thermistor_lag_s = 0.8277f;
      result.fet_thermal.g_pad_spreader_W_per_K = 0.3f;
      result.fet_thermal.g_spreader_ambient_W_per_K = 2.0f;
      result.fet_thermal.c_spreader_J_per_K = 10.0f;
      result.fet_thermal.switching_loss_scale = 0.4194f;
      result.fet_thermal.r_extra_ohm = 0.000605f;
      result.fet_thermal.switching_energy_J_per_V = 4.51e-07f;
      result.fet_thermal.switching_energy_J_per_V_A = 1.63e-08f;
      result.fet_thermal.switching_energy_J_per_A2 = 1.834e-08f;
      result.fet_thermal.deadtime_diode_V_s = 7e-07f;
      result.fet_thermal.hard_switching_per_Hz = 4e-06f;
      result.fet_thermal.network_couplings = 5;
      result.fet_thermal.coupling_a = { 0, 1, 2, 2, 4 };
      result.fet_thermal.coupling_b = { 1, 3, 3, 4, 5 };
      result.fet_thermal.coupling_W_per_K =
          { 0.08442f, 0.04818f, 0.06422f, 0.1397f, 0.1203f };
    } else {
      MJ_ASSERT(false);
    }
  } else {
    MJ_ASSERT(false);
  }

  return result;
}

// NOTE: This must remain safe to call from a HardFault context.  That
// means: no dynamic allocation, no locks, no recursion, no reliance on
// C++ static constructors that may not yet have run, and no calls into
// mbed helpers that assert on `NC` pins.  If `g_hw_pins` has not been
// populated yet, all DRV8323 control pins are `NC` -- but in that
// window the GPIOs are still in MCU reset state and the on-board
// pull-downs hold the driver off, so there is nothing to disable.
void MoteusEnsureOff() {
  if (moteus::g_hw_pins.drv8323_enable != NC) {
    gpio_t power;
    gpio_init_out(&power, moteus::g_hw_pins.drv8323_hiz);
    gpio_write(&power, 0);

    // Also, disable the DRV8323 entirely, because, hey, why not.
    gpio_t enable;
    gpio_init_out(&enable, moteus::g_hw_pins.drv8323_enable);
    gpio_write(&enable, 0);
  }

  // We want to ensure that our primary interrupt is not running.
  // Which one it is could vary, so just turn them all off.
  NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
  NVIC_DisableIRQ(TIM2_IRQn);
  NVIC_DisableIRQ(TIM3_IRQn);
  NVIC_DisableIRQ(TIM4_IRQn);
  NVIC_DisableIRQ(TIM5_IRQn);
}

}
