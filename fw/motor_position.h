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

// TODO: Fault for absolute encoders which change too quickly.

#pragma once

#include <array>
#include <atomic>
#include <cmath>

#include "mjlib/base/inplace_function.h"
#include "mjlib/base/visitor.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/aux_common.h"
#include "fw/bldc_servo_structs.h"
#include "fw/ccm.h"
#include "fw/math.h"

namespace moteus {

class MotorPosition {
 public:
  static constexpr int kNumSources = 3;
  static constexpr int kHallCounts = 6;
  static constexpr int kCompensationSize = 32;

  struct SourceConfig {
    uint8_t aux_number = 1;
    enum Type {
      kNone,
      kSpi,
      kUart,
      kQuadrature,
      kHall,
      kIndex,
      kSineCosine,
      kI2C,
      kSensorless,

      kNumTypes,
    };
    Type type = kNone;
    uint8_t i2c_device = 0;

    // If 1 or 2, which aux port an index should be sourced from.
    // Only valid for incremental sources.
    int8_t incremental_index = -1;

    uint32_t cpr = 16384;
    float offset = 0.0f;
    int8_t sign = 1;
    int32_t debug_override = -1;

    enum Reference {
      kRotor,
      kOutput,
    };
    Reference reference = kRotor;

    float pll_filter_hz = 400.0;

    // The CPR for this source is subdivided into N equal segments.
    // This table specifies a fraction of CPR that should be applied
    // when at the *center* of that offset region.  Other counts will
    // be interpolated.
    //
    // For example, with a CPR of 256, the bins are:
    //
    //  Bin   Count Range    Center
    //  0     0-7            4
    //  1     8-15           12
    //  2     16-23          20
    //  3     24-31          28
    //  ... etc ...
    //  31    248-255        252
    std::array<float, kCompensationSize> compensation_table = {};

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(aux_number));
      a->Visit(MJ_NVP(type));
      a->Visit(MJ_NVP(i2c_device));
      a->Visit(MJ_NVP(incremental_index));
      a->Visit(MJ_NVP(cpr));
      a->Visit(MJ_NVP(offset));
      a->Visit(MJ_NVP(sign));
      a->Visit(MJ_NVP(debug_override));
      a->Visit(MJ_NVP(reference));
      a->Visit(MJ_NVP(pll_filter_hz));
      a->Visit(MJ_NVP(compensation_table));
    }
  };

  struct Config {
    // We store the sources in an array here so that the resulting
    // schema only has the enumeration names replicated once.
    std::array<SourceConfig, kNumSources> sources;

    // This source is used to determine what electrical phase the
    // motor is in.
    int8_t commutation_source = 0;

    struct Output {
      // This source is used to determine the velocity and position of
      // the output.
      int8_t source = 0;

      // An offset and sign applied to the selected source when
      // determining position and velocity.
      float offset = 0.0f;
      int8_t sign = 1;

      // This can be used to provide a reference for the output
      // position.  It may be any source that can generate an absolute
      // position, or an index source, in which case the offset is used
      // with the cpr to determine the position when the index is first
      // crossed.
      int8_t reference_source = -1;

      template <typename Archive>
      void Serialize(Archive* a) {
        a->Visit(MJ_NVP(source));
        a->Visit(MJ_NVP(offset));
        a->Visit(MJ_NVP(sign));
        a->Visit(MJ_NVP(reference_source));
      }
    };
    Output output;

    // If a gear reducer is between the rotor and the output, this
    // should be less than 1.0f, otherwise greater.
    float rotor_to_output_ratio = 1.0f;

    Config() {
      // The factory default is to get rotor position directly from
      // the onboard SPI encoder (attached to aux1).
      sources[0].aux_number = 1;
      sources[0].type = SourceConfig::kSpi;
      sources[0].reference = SourceConfig::kRotor;
    }

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(sources));
      a->Visit(MJ_NVP(commutation_source));
      a->Visit(MJ_NVP(output));
      a->Visit(MJ_NVP(rotor_to_output_ratio));
    }
  };

  struct SourceStatus {
    bool active_velocity = false;
    bool active_theta = false;
    bool active_absolute = false;
    uint32_t raw = 0;
    float time_since_update = 0.0f;

    // This will increment every time a new value is provided.
    uint8_t nonce = 0;

    // This value is the raw value plus any internal offset or index.
    uint32_t offset_value = 0.0;

    // This value is offset_value + compensation
    float compensated_value = 0.0f;

    // This value is compensated_value + pll_filter
    float filtered_value = 0.0f;

    float velocity = 0.0f;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(active_velocity));
      a->Visit(MJ_NVP(active_theta));
      a->Visit(MJ_NVP(active_absolute));
      a->Visit(MJ_NVP(raw));
      a->Visit(MJ_NVP(time_since_update));
      a->Visit(MJ_NVP(nonce));
      a->Visit(MJ_NVP(offset_value));
      a->Visit(MJ_NVP(compensated_value));
      a->Visit(MJ_NVP(filtered_value));
      a->Visit(MJ_NVP(velocity));
    }
  };

  struct Status {
    enum Error {
      kNone,
      kMotorNotConfigured,
      kInvalidConfig,
      kSourceError,

      kNumErrors,
    };
    Error error = kNone;

    std::array<SourceStatus, kNumSources> sources = { {} };

    // Increases anytime the configuration changes in a way that
    // causes the outputs to reset.
    uint8_t epoch = 0;

    ///////////////////////////////
    /// OUTPUT REFERENCED POSITIONS

    // The "relative" position is initialized to 0 at power on (or on
    // a 0 reset).
    bool position_relative_valid = false;
    int64_t position_relative_raw = 0;
    float position_relative = 0.0f;

    // This is the nearest point where the source encoder reads 0
    // exactly.
    int64_t position_relative_modulo = 0;

    // The "absolute" position is referenced to a global point, and
    // may not be known until after a homing procedure is complete or
    // an index pulse is found.  It is always the same scale as the
    // relative position, and thus has the same velocity.
    int64_t position_raw = 0;
    float position = 0.0f;

    // Has "homing" of some sort been applied to the absolute
    // position.
    enum Homed {
      // The available position is relative only
      kRelative,

      // The position has been referenced to an absolute rotor position
      kRotor,

      // The position has been referenced to an absolute output position
      kOutput,
    };
    Homed homed = kRelative;

    // The velocity is the same for both absolute and relative
    // position.
    float velocity = 0.0f;


    //////////////////////////////
    /// ROTOR REFERENCED POSITIONS

    // The current electrical phase for commutation.
    bool theta_valid = false;
    float electrical_theta = 0.0f;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(error));
      a->Visit(MJ_NVP(sources));
      a->Visit(MJ_NVP(position_relative_valid));
      a->Visit(MJ_NVP(position_relative_raw));
      a->Visit(MJ_NVP(position_relative));
      a->Visit(MJ_NVP(position_relative_modulo));
      a->Visit(MJ_NVP(position_raw));
      a->Visit(MJ_NVP(position));
      a->Visit(MJ_NVP(homed));
      a->Visit(MJ_NVP(velocity));
      a->Visit(MJ_NVP(theta_valid));
      a->Visit(MJ_NVP(electrical_theta));
    }
  };

  MotorPosition(mjlib::micro::PersistentConfig* persistent_config,
                mjlib::micro::TelemetryManager* telemetry_manager,
                const aux::AuxStatus* aux1_status,
                const aux::AuxStatus* aux2_status,
                const aux::AuxConfig* aux1_config,
                const aux::AuxConfig* aux2_config)
      : aux_status_{aux1_status, aux2_status},
        aux_config_{aux1_config, aux2_config} {
    persistent_config->Register(
        "motor_position", &config_,
        std::bind(&MotorPosition::HandleConfigUpdate, this));
    persistent_config->Register(
        "motor", &motor_,
        std::bind(&MotorPosition::HandleConfigUpdate, this));
    telemetry_manager->Register("motor_position", &status_);

    absolute_relative_delta.store(0);

    HandleConfigUpdate();
  }

  void ISR_Update(float dt) MOTEUS_CCM_ATTRIBUTE {
    // First, fill in each of our sources raw data and do source level
    // filtering.
    ISR_UpdateSources(dt);

    // Then update our output structures.
    ISR_UpdateState();
  }

  void RegisterConfigUpdated(mjlib::base::inplace_function<void ()> handler) {
    config_updated_ = handler;
  }

  // Force the output position to be the given value.
  void ISR_SetOutputPosition(float value) MOTEUS_CCM_ATTRIBUTE {
    status_.position = value;
    status_.position_raw = static_cast<int64_t>(static_cast<double>(value) * (1ll << 48));
    status_.homed = Status::kOutput;
  }

  static float IntToFloat(int64_t value) MOTEUS_CCM_ATTRIBUTE {
    return static_cast<float>(static_cast<int32_t>(value >> 32ll)) / 65536.0f;
  }

  static int64_t FloatToInt(float value) MOTEUS_CCM_ATTRIBUTE {
    return (1ll << 32) * static_cast<int32_t>((1l << 16) * value);
  }

  // Set the output position to be the nearest value consistent with
  // any absolute encoders or reference sources that may be enabled.
  //
  // An absolute encoder could be a actual absolute value, or an index
  // referenced incremental counter.
  void ISR_SetOutputPositionNearest(float value) MOTEUS_CCM_ATTRIBUTE {
    MJ_ASSERT(config_.output.source >= 0);
    if (!status_.sources[config_.output.source].active_absolute) {
      if (config_.output.reference_source < 0 ||
          !status_.sources[config_.output.reference_source].active_absolute) {
        // We have no absolute values to work with at all.  Thus just
        // ignore nearest requests.
        return;
      }
    }

    ISR_SetOutputPositionNearestHelper(value);
  }

  void ISR_RequireReindex() {
    status_.homed = Status::kRelative;
    for (auto& source : status_.sources) {
      source.active_theta = false;
    }
    status_.theta_valid = false;
  }

  const Status& status() const { return status_; }
  Config* config() { return &config_; }
  BldcServoMotor* motor() { return &motor_; }

  // The high 32 bits of (position - position_relative).
  std::atomic<int32_t> absolute_relative_delta;

  static float WrapBalancedCpr(float value, float cpr) MOTEUS_CCM_ATTRIBUTE {
    return WrapCpr(value + 1.5f * cpr, cpr) - 0.5f * cpr;
  }

  static float WrapCpr(float value, float cpr) MOTEUS_CCM_ATTRIBUTE {
    // We would use fmodf, but we're trying to be fast there.

    const int32_t divisor = static_cast<int>(value / cpr);
    const float mod = value - divisor * cpr;
    return (mod >= 0.0f) ? mod : (mod + cpr);
  }

  static uint32_t WrapIntCpr(int32_t value, uint32_t cpr) MOTEUS_CCM_ATTRIBUTE {
    return (value + cpr) % cpr;
  }

 private:
  bool IsThetaCapable(const SourceConfig& config) const MOTEUS_CCM_ATTRIBUTE {
      // TODO: eventually exclude spi options that are incremental

    return config.type == SourceConfig::kSpi ||
        config.type == SourceConfig::kI2C ||
        config.type == SourceConfig::kHall ||
        config.type == SourceConfig::kSineCosine ||
        config.type == SourceConfig::kUart;
  };

  void HandleConfigUpdate() {
    const auto old_epoch = status_.epoch;

    status_ = {};

    status_.epoch = old_epoch + 1;

    for (size_t i = 0; i < config_.sources.size(); i++) {
      auto& source_config = config_.sources[i];

      source_config.aux_number =
          std::max<uint8_t>(1, std::min<uint8_t>(2, source_config.aux_number));
      source_config.sign = (source_config.sign >= 0) ? 1 : -1;
      source_config.i2c_device =
          std::min<uint8_t>(source_config.i2c_device,
                            aux_status_[0]->i2c.devices.size());

      if (source_config.incremental_index > 2) {
        source_config.incremental_index = 2;
      }
      if (IsThetaCapable(source_config) && source_config.incremental_index >= 1) {
        status_.error = Status::kInvalidConfig;
        return;
      }

      const auto* aux_config = aux_config_[source_config.aux_number - 1];

      switch (source_config.type) {
        case SourceConfig::kSpi: {
          break;
        }
        case SourceConfig::kI2C: {
          // These can be really low rate, and we know in advance
          // approximately how low they will be.  Ensure that our PLL
          // filter bandwidth is no more than 1/10th of the expected
          // update rate.
          const float source_rate_hz =
              1000.0f /
              aux_config->i2c.devices[source_config.i2c_device].poll_ms;
          const float max_pll_hz = source_rate_hz / 10.0f;
          source_config.pll_filter_hz =
              std::min(source_config.pll_filter_hz, max_pll_hz);
          break;
        }
        case SourceConfig::kHall: {
          // Hall effect sources must be related to the rotor, and we
          // require the CPR to be directly related to the motor pole
          // count.
          source_config.cpr = kHallCounts * motor_.poles / 2;
          break;
        }
        case SourceConfig::kSineCosine: {
          source_config.cpr = 65536;
          break;
        }
        case SourceConfig::kIndex: {
          if (source_config.reference != SourceConfig::kOutput) {
            status_.error = Status::kInvalidConfig;
            return;
          }
          break;
        }
        default: {
          break;
        }
      }
    }

    if (config_.commutation_source < 0 ||
        (config_.commutation_source >=
         static_cast<int>(config_.sources.size()))) {
      status_.error = Status::kInvalidConfig;
      return;
    }

    // First, figure out what we need for commutation.
    commutation_config_ =
        &config_.sources[config_.commutation_source];
    commutation_status_ =
        &status_.sources[config_.commutation_source];

    if (IsThetaCapable(*commutation_config_)) {
      if (motor_.poles == 0 ||
          (motor_.poles % 2) != 0) {
        status_.error = Status::kMotorNotConfigured;
        return;
      }

      switch (commutation_config_->reference) {
        case SourceConfig::kRotor: {
          // this is always fine.
          break;
        }
        case SourceConfig::kOutput: {
          if (commutation_config_->type == SourceConfig::kHall) {
            // Hall effect sensors must be rotor relative.
            status_.error = Status::kInvalidConfig;
            return;
          }
          if (config_.rotor_to_output_ratio > 1.0f) {
            status_.error = Status::kInvalidConfig;
            return;
          }
          const float inverse = 1.0f / config_.rotor_to_output_ratio;
          const bool non_integral =
              std::abs(std::round(inverse) - inverse) > 0.001f;
          if (non_integral) {
            status_.error = Status::kInvalidConfig;
            return;
          }
          break;
        }
      }
    }
    commutation_pole_scale_ = motor_.poles * 0.5f * k2Pi;

    commutation_rotor_scale_ =
        (commutation_config_->reference == SourceConfig::kRotor ?
         1.0f :
         config_.rotor_to_output_ratio);


    // If we have a reference source, check it out.
    if (config_.output.reference_source >= 0) {
      config_.output.reference_source = std::min<int8_t>(
          config_.sources.size(), config_.output.reference_source);
      // It must be referenced to the output.
      const auto& output_reference_config =
          config_.sources[config_.output.reference_source];
      if (output_reference_config.reference != SourceConfig::kOutput &&
          config_.rotor_to_output_ratio != 1.0f) {
        status_.error = Status::kInvalidConfig;
        return;
      }
    }

    if (config_.output.source < 0 ||
        config_.output.source >=
        static_cast<int>(config_.sources.size())) {
      status_.error = Status::kInvalidConfig;
      return;
    }

    output_config_ = &config_.sources[config_.output.source];
    output_status_ = &status_.sources[config_.output.source];

    config_.output.sign = (config_.output.sign >= 0) ? 1 : -1;

    const float encoder_ratio =
        (output_config_->reference == SourceConfig::kRotor ?
         config_.rotor_to_output_ratio :
         1.0f);

    output_ambiguity_scale_ =
        (output_config_->reference == SourceConfig::kRotor) ?
        config_.rotor_to_output_ratio :
        1.0f;

    output_cpr_scale_ =
        config_.output.sign * encoder_ratio / output_config_->cpr;

    output_encoder_step_ =
        (1ll << 24) * static_cast<int32_t>((1l << 24) * std::abs(encoder_ratio));

    // We pre-calculate some thresholds for the high bits to save CPU
    // cycles later on.
    output_encoder_step_hb_1_4_ =
        (output_encoder_step_ / 4 * 1) >> 32;
    output_encoder_step_hb_3_4_ =
        (output_encoder_step_ / 4 * 3) >> 32;

    for (size_t i = 0; i < pll_filter_constants_.size(); i++) {
      const auto& config = config_.sources[i];
      auto& constants = pll_filter_constants_[i];

      const float w_3db = config.pll_filter_hz * k2Pi;
      constants.kp = 2.0f * w_3db;
      constants.ki = w_3db * w_3db;
    }

    if (config_updated_) {
      config_updated_();
    }
  }

  void ISR_UpdateState() MOTEUS_CCM_ATTRIBUTE {
    if (status_.error != Status::kNone) { return; }

    ISR_UpdateCommutation();
    ISR_UpdateOutput();
  }

  void ISR_UpdateCommutation() MOTEUS_CCM_ATTRIBUTE {
    const auto& commutation_config = *commutation_config_;
    const auto& commutation_status = *commutation_status_;

    if (commutation_status.active_theta) {
      const float ratio =
          commutation_status.filtered_value / commutation_config.cpr;
      const int offset_size = motor_.offset.size();
      const int offset_index =
          std::min<int>(offset_size - 1, ratio * offset_size);
      // MJ_ASSERT(offset_index >= 0 && offset_index < offset_size);

      status_.theta_valid = true;
      status_.electrical_theta = WrapZeroToTwoPi(
          ratio * commutation_pole_scale_ / commutation_rotor_scale_ +
          motor_.offset[offset_index]);
    }
  }

  void ISR_UpdateOutput() MOTEUS_CCM_ATTRIBUTE {
    const auto& output_status = *output_status_;

    if (output_status.active_velocity) {
      const float encoder_ratio =
          config_.output.sign == 1 ?
          (output_status.filtered_value / output_config_->cpr) :
          (1.0f - output_status.filtered_value / output_config_->cpr);
      const float scaled_encoder_ratio =
          config_.output.sign == 1 ?
          (output_status.filtered_value * output_cpr_scale_) :
          (output_config_->cpr - output_status.filtered_value) * std::abs(output_cpr_scale_);
      const int64_t scaled_int_encoder_ratio =
          (1ll << 24) * static_cast<int32_t>((1l << 24) * scaled_encoder_ratio);

      // If this is our very first relative position output, select
      // our modulo so that we start at position 0.
      if (!status_.position_relative_valid) {
        status_.position_relative_modulo = -scaled_int_encoder_ratio;
      }

      // We can update our relative position at least.
      status_.position_relative_valid = true;
      const auto old_position_relative_raw = status_.position_relative_raw;

      // We update our relative position by adding in the current
      // encoder offset to the "modulo" number of the encoder.  The
      // modulo number only increments in exact integral full encoder
      // cycles.  This ensures that our relative position remains
      // exactly in sync with the encoder value.
      const auto modulo_delta =
          status_.position_relative_raw - status_.position_relative_modulo;
      if ((modulo_delta >> 32) < output_encoder_step_hb_1_4_ &&
          encoder_ratio > 0.75f) {
        status_.position_relative_modulo -= output_encoder_step_;
      } else if ((modulo_delta >> 32) > output_encoder_step_hb_3_4_ &&
                 encoder_ratio < 0.25f) {
        status_.position_relative_modulo += output_encoder_step_;
      }

      status_.position_relative_raw =
          status_.position_relative_modulo +
          scaled_int_encoder_ratio;

      // Since position_relative is integral and exact, we can exactly
      // update our absolute position with no loss by applying its
      // incremental change to the absolute position.
      const auto relative_delta =
          status_.position_relative_raw - old_position_relative_raw;

      status_.position_raw += relative_delta;

      if (output_status.active_absolute &&
          status_.homed == Status::kRelative) {
        // Latch the current position
        const float float_first_output =
            WrapBalancedCpr(
                (scaled_encoder_ratio +
                 config_.output.offset * config_.output.sign),
                output_ambiguity_scale_);

        const int64_t int64_first_output =
            (1ll << 24) * static_cast<int32_t>((1l << 24) * float_first_output);

        status_.position_raw = int64_first_output;
        status_.homed = Status::kRotor;
      }

      status_.position_relative = IntToFloat(status_.position_relative_raw);
      status_.position = IntToFloat(status_.position_raw);
      status_.velocity = output_status.velocity * output_cpr_scale_;
    }

    if (!output_status.active_velocity &&
        !output_status.active_theta) {
      // We've got nothing, so just report all inactive.  If we
      // eventually support "output only" sensors, then we could do
      // something here.
      status_.position_relative_valid = false;
      status_.theta_valid = false;
      status_.homed = Status::kRelative;

      return;
    }

    absolute_relative_delta.store(
        (status_.position_raw - status_.position_relative_raw) >> 32);

    // If we have an output reference source, it is valid, the
    // position is valid, and we haven't had an index home yet, then
    // apply it.
    if (config_.output.reference_source >= 0 &&
        status_.sources[config_.output.reference_source].active_absolute &&
        status_.position_relative_valid &&
        status_.homed != Status::kOutput) {
      if (status_.homed == Status::kRotor) {
        ISR_SetOutputPositionNearestHelper(0.0);
      } else if (status_.homed == Status::kRelative) {
        ISR_SetOutputPosition(
            WrapBalancedCpr(
                ((status_.sources[config_.output.reference_source].filtered_value /
                  config_.sources[config_.output.reference_source].cpr +
                  config_.output.offset) *
                 config_.output.sign),
                1.0f));
      }
    }
  }

  void ISR_UpdateSources(float dt) MOTEUS_CCM_ATTRIBUTE {
    for (size_t i = 0; i < status_.sources.size(); i++) {
      const auto& config = config_.sources[i];

      if (config.type == SourceConfig::kNone) {
        continue;
      }

      auto& status = status_.sources[i];
      const auto& filter = pll_filter_constants_[i];

      bool updated = false;
      const bool old_active_theta = status.active_theta;
      const bool old_active_velocity = status.active_velocity;

      const auto* this_aux = aux_status_[config.aux_number - 1];
      if (this_aux->error != aux::AuxError::kNone) {
        status_.error = Status::kSourceError;
        status.active_theta = false;
        status.active_velocity = false;
        status.active_absolute = false;
        continue;
      }

      switch (config.type) {
        case SourceConfig::kSpi: {
          // We check this in HandleConfigUpdate
          // MJ_ASSERT(config.aux_number == 1);

          const auto* spi_data = &this_aux->spi;
          if (!spi_data->active) { break; }
          status.raw = spi_data->value;

          updated = ISR_UpdateAbsoluteSource(
              spi_data->nonce, spi_data->value,
              config.offset, config.sign, config.cpr,
              &status);
          status.active_absolute = true;
          break;
        }
        case SourceConfig::kUart: {
          const auto* uart_data = &this_aux->uart;
          if (!uart_data->active) { break; }
          status.raw = uart_data->value;

          updated = ISR_UpdateAbsoluteSource(
              uart_data->nonce, uart_data->value,
              config.offset, config.sign, config.cpr,
              &status);
          status.active_absolute = true;
          break;
        }
        case SourceConfig::kSineCosine: {
          const auto* sc_data = &this_aux->sine_cosine;

          if (!sc_data->active) { break; }
          status.raw = sc_data->value;
          updated = ISR_UpdateAbsoluteSource(
              status.nonce + 1, sc_data->value,
              config.offset, config.sign, config.cpr, &status);
          status.active_absolute = true;
          break;
        }
        case SourceConfig::kI2C: {
          const auto* i2c_data = &this_aux->i2c.devices[config.i2c_device];
          if (!i2c_data->active) { break; }
          status.raw = i2c_data->value;

          updated = ISR_UpdateAbsoluteSource(
              i2c_data->nonce, i2c_data->value,
              config.offset, config.sign, config.cpr, &status);
          status.active_absolute = true;
          break;
        }
        case SourceConfig::kHall: {
          const auto* hall_data = &this_aux->hall;
          if (!hall_data->active) { break; }
          status.raw = hall_data->bits;

          const uint32_t cpr = kHallCounts / 2 * motor_.poles;
          const int32_t current_with_offset =
              ((hall_data->count +
                static_cast<int32_t>(config.offset)) * config.sign +
               kHallCounts) % kHallCounts;
          const int32_t old_with_offset =
              status.offset_value % kHallCounts;
          const int32_t delta =
              (current_with_offset - old_with_offset +
               kHallCounts + kHallCounts / 2) %
              kHallCounts -
              (kHallCounts / 2);

          const uint32_t new_value = (status.offset_value + cpr + delta) % cpr;

          updated = ISR_UpdateAbsoluteSource(
              status.nonce + 1, new_value,
              0, 1, config.cpr,
              &status);
          status.active_absolute = false;
          break;
        }
        case SourceConfig::kQuadrature: {
          const auto* quad_status = &this_aux->quadrature;
          if (!quad_status->active) { break; }
          const auto old_raw = status.raw;
          const auto old_filtered_value = status.filtered_value;

          status.raw = quad_status->value;
          const auto delta = WrapIntCpr(status.raw - old_raw, config.cpr);
          status.offset_value = WrapIntCpr(
              static_cast<int32_t>(status.offset_value + delta * config.sign),
              config.cpr);
          status.nonce++;
          status.active_velocity = true;

          if (!status.active_theta &&
              config.incremental_index >= 1) {
            const auto* index_status =
                &aux_status_[config.incremental_index == 2 ? 1 : 0]->index;
            // TODO: Maybe optionally require a minimum velocity?
            if (index_status->value) {
              // This is our index time.
              status.offset_value = config.offset;
              status.active_theta = true;
              status.active_absolute = true;

              if (config_.output.source == static_cast<int8_t>(i)) {
                // This is an "absolute" encoder which can warp its
                // position.  In order for our "position_relative" to
                // remain continuous, the modulo must be updated
                // correspondingly.
                const float ratio = WrapBalancedCpr(
                    static_cast<float>(status.offset_value) -
                    static_cast<float>(old_filtered_value),
                    config.cpr) * output_cpr_scale_;
                const int64_t adjustment =
                    (1ll << 24) * static_cast<int32_t>((1l << 24) * ratio);
                status_.position_relative_modulo -= adjustment;
              }
            }
          }
          updated = true;
          break;
        }
        case SourceConfig::kIndex: {
          const auto& index_status = &this_aux->index;
          if (!index_status->active) { break; }
          if (index_status->value) {
            status.offset_value = config.offset;
            status.filtered_value = status.offset_value;
            status.active_theta = true;
            status.active_absolute = true;
          } else {
            status.active_theta = false;
            status.active_absolute = false;
          }
          break;
        }
        default: {
          MJ_ASSERT(false);
          break;
        }
      }

      if (config.debug_override >= 0) {
        status.active_theta = true;
        status.active_velocity = true;
        status.active_absolute = true;
        status.offset_value = config.debug_override;
        status.compensated_value = config.debug_override;
        status.filtered_value = config.debug_override;
        updated = true;
      } else {
        const int bin_size = std::max<int>(1, config.cpr / kCompensationSize);
        // Perform compensation.
        const int left_offset =
            ((status.offset_value - bin_size / 2) *
             kCompensationSize / config.cpr + kCompensationSize) %
            kCompensationSize;
        const int right_offset = (left_offset + 1) % kCompensationSize;
        const int delta =
            (status.offset_value + config.cpr -
             left_offset * bin_size -
             bin_size / 2) % bin_size;
        const float fraction =
            static_cast<float>(delta) / bin_size;
        const float left_comp = config.compensation_table[left_offset];
        const float right_comp = config.compensation_table[right_offset];
        const float comp_fraction =
            (right_comp - left_comp) * fraction + left_comp;

        status.compensated_value =
            WrapCpr(
                status.offset_value +
                comp_fraction * config.cpr, config.cpr);
      }

      if (!status.active_theta &&
          !status.active_velocity) {
        continue;
      }

      status.time_since_update += dt;

      status.filtered_value += dt * status.velocity;

      const float cpr = config.cpr;

      if (updated) {
        if (!old_active_velocity && status.active_velocity) {
          // This is our first update.  Just snap to the position.
          status.filtered_value = status.compensated_value;
          status.velocity = 0;
        } else if (!old_active_theta && status.active_theta) {
          // Our velocity was valid before, so leave it alone.
          status.filtered_value = status.compensated_value;
        } else if (config.pll_filter_hz != 0.0f) {
          // We check this in config.
          // MJ_ASSERT(config.pll_filter.enabled);

          const float unwrapped_error =
              -(status.filtered_value - status.compensated_value);
          const float error =
              WrapBalancedCpr(unwrapped_error, cpr);

          status.filtered_value +=
              status.time_since_update * filter.kp * error;

          status.velocity +=
              status.time_since_update * filter.ki * error;
        } else {
          status.filtered_value = status.compensated_value;
          status.velocity = 0.0f;
        }

        status.time_since_update = 0.0f;
      }

      status.filtered_value = WrapCpr(status.filtered_value, cpr);
    }
  }

  bool ISR_UpdateAbsoluteSource(
      uint8_t nonce, uint32_t value,
      int32_t offset,
      int32_t sign,
      uint32_t cpr,
      SourceStatus* status) MOTEUS_CCM_ATTRIBUTE {
    if (nonce == status->nonce) { return false; }

    status->offset_value = WrapIntCpr(
        (static_cast<int32_t>(value) + offset) * sign, cpr);
    status->nonce = nonce;
    status->active_velocity = true;
    status->active_theta = true;
    return true;
  }

  void ISR_SetOutputPositionNearestHelper(float value) MOTEUS_CCM_ATTRIBUTE {
    const auto& output_status =
        status_.sources[config_.output.source];
    const auto& output_config =
        config_.sources[config_.output.source];
    const float output_value =
        WrapBalancedCpr(
            ((output_status.filtered_value / output_config.cpr) *
             output_ambiguity_scale_ +
             config_.output.offset) * config_.output.sign,
            output_ambiguity_scale_);

    [&]() {
      if (config_.output.reference_source >= 0) {
        const auto& reference_source =
            status_.sources[config_.output.reference_source];
        const auto& reference_config =
            config_.sources[config_.output.reference_source];

        const float reference_value =
            WrapBalancedCpr(
                (reference_source.filtered_value / reference_config.cpr +
                 config_.output.offset) * config_.output.sign,
                1.0f);

        if (status_.sources[config_.output.source].active_absolute &&
            status_.sources[config_.output.reference_source].active_absolute &&
            output_ambiguity_scale_ < 1.0f) {
          // We have an active reference and a reduction.  Here, we need
          // to use the reference to resolve the reduction ambiguity, then
          // count integral multiples from the requested value.

          const float integral_offsets_lower =
              std::floor(reference_value / output_ambiguity_scale_);
          const float integral_offsets_upper =
              integral_offsets_lower + 1;

          const float maybe_lower_value =
              integral_offsets_lower * output_ambiguity_scale_ + output_value;
          const float maybe_upper_value =
              integral_offsets_upper * output_ambiguity_scale_ + output_value;

          float first_disambiguation = 0.0f;

          if (std::abs(maybe_lower_value - reference_value) <
              std::abs(maybe_upper_value - reference_value)) {
            first_disambiguation = maybe_lower_value;
          } else {
            first_disambiguation = maybe_upper_value;
          }

          // Now do the integral number of counts from the requested value.
          const float requested_offset = std::round(value - first_disambiguation);
          const float final_value = first_disambiguation + requested_offset;

          status_.position_raw = FloatToInt(final_value);

          return;
        } else if (!status_.sources[config_.output.source].active_absolute) {
          // Just position ourselves relative to the reference, ignoring
          // the output source, which is only relative.
          status_.position_raw = FloatToInt(reference_value);

          return;
        }
      }

      // We are going to assume we have no reference.  In this case,
      // we just disambiguate to the nearest output source ambiguity.

      const float integral_offsets =
          std::round((value - output_value) / output_ambiguity_scale_);
      const float final_value =
          output_value + integral_offsets * output_ambiguity_scale_;
      status_.position_raw = FloatToInt(final_value);
    }();

    // No matter what, make our floating point position correct and
    // update our homed status to output.
    status_.position = IntToFloat(status_.position_raw);

    status_.homed = Status::kOutput;
  }

  Config config_;
  BldcServoMotor motor_;
  Status status_;

  const aux::AuxStatus* const aux_status_[2];
  const aux::AuxConfig* const aux_config_[2];

  mjlib::base::inplace_function<void ()> config_updated_;

  // Values cached after config changes to make runtime computation
  // faster.
  const SourceConfig* commutation_config_ = nullptr;
  const SourceStatus* commutation_status_ = nullptr;
  float commutation_pole_scale_ = 1.0f;
  float commutation_rotor_scale_ = 1.0f;
  float output_ambiguity_scale_ = 1.0f;
  const SourceConfig* output_config_ = nullptr;
  const SourceStatus* output_status_ = nullptr;
  float output_cpr_scale_ = 1.0f;
  int64_t output_encoder_step_ = 0;
  int32_t output_encoder_step_hb_1_4_ = 0;
  int32_t output_encoder_step_hb_3_4_ = 0;

  struct PllFilterConstants {
    float kp = 0.0f;
    float ki = 0.0f;
  };
  std::array<PllFilterConstants, kNumSources> pll_filter_constants_;
};

}

namespace mjlib {
namespace base {

template<>
struct IsEnum<moteus::MotorPosition::SourceConfig::Type> {
  static constexpr bool value = true;

  using T = moteus::MotorPosition::SourceConfig::Type;
  static std::array<std::pair<T, const char*>, T::kNumTypes> map() {
    return { {
        { T::kNone, "none" },
        { T::kSpi, "spi" },
        { T::kUart, "uart" },
        { T::kQuadrature, "quadrature" },
        { T::kHall, "hall" },
        { T::kIndex, "index" },
        { T::kSineCosine, "sine_cosine" },
        { T::kI2C, "i2c" },
        { T::kSensorless, "sensorless" },
      }};
  }
};

template <>
struct IsEnum<moteus::MotorPosition::SourceConfig::Reference> {
  static constexpr bool value = true;

  using R = moteus::MotorPosition::SourceConfig::Reference;
  static std::array<std::pair<R, const char*>, 2> map() {
    return { {
        { R::kRotor, "rotor" },
        { R::kOutput, "output" },
      }};
  }
};

template <>
struct IsEnum<moteus::MotorPosition::Status::Error> {
  static constexpr bool value = true;

  using E = moteus::MotorPosition::Status::Error;
  static std::array<std::pair<E, const char*>, E::kNumErrors> map() {
    return { {
        { E::kNone, "none" },
        { E::kMotorNotConfigured, "motor_not_conf" },
        { E::kInvalidConfig, "invalid_config" },
        { E::kSourceError, "source_error" },
      }};
  }
};

template <>
struct IsEnum<moteus::MotorPosition::Status::Homed> {
  static constexpr bool value = true;

  using H = moteus::MotorPosition::Status::Homed;
  static std::array<std::pair<H, const char*>, 3> map() {
    return { {
        { H::kRelative, "relative" },
        { H::kRotor, "rotor" },
        { H::kOutput, "output" },
      }};
  }
};

}
}
