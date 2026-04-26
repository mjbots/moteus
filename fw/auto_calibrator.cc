#include "fw/auto_calibrator.h"

#include <algorithm>
#include <cstddef>
#include <cmath>

#include "fw/math.h"
#include "fw/motor_position.h"

namespace moteus {
namespace {

constexpr uint32_t kFaultServo = 1;
constexpr uint32_t kFaultTimeout = 2;
constexpr uint32_t kFaultNoCurrent = 3;
constexpr uint32_t kFaultBadResistance = 4;
constexpr uint32_t kFaultBadInductance = 5;
constexpr uint32_t kFaultNoEncoder = 6;
constexpr uint32_t kFaultBadEncoder = 7;
constexpr uint32_t kFaultBadKv = 8;

constexpr uint8_t kIndPeriods[] = {2, 3, 4, 6, 8, 12, 16, 24, 32};
constexpr std::size_t kIndPeriodCount = sizeof(kIndPeriods) / sizeof(kIndPeriods[0]);
constexpr std::size_t kKvPointCount = 5;

float WrapUnit(float value) {
  value -= static_cast<int32_t>(value);
  return value < 0.0f ? value + 1.0f : value;
}

float WrapBalanced(float value) {
  value = WrapUnit(value);
  return value > 0.5f ? value - 1.0f : value;
}

}

AutoCalibrator::AutoCalibrator(
    mjlib::micro::PersistentConfig* persistent_config,
    mjlib::micro::TelemetryManager* telemetry_manager,
    BldcServo* bldc)
    : persistent_config_(persistent_config), bldc_(bldc) {
  persistent_config_->Register("autocal", &config_, []() {});
  telemetry_manager->Register("autocal", &status_);
}

void AutoCalibrator::Begin() {
  status_ = {};
  cal_voltage_ = 0.01f;
  last_res_voltage_ = 0.0f;
  last_res_current_ = 0.0f;
  low_res_voltage_ = 0.0f;
  low_res_current_ = 0.0f;
  peak_res_voltage_ = 0.0f;
  peak_res_current_ = 0.0f;
  ind_index_ = 0;
  best_ind_di_dt_ = 0.0f;
  best_ind_voltage_ = 0.0f;
  ind_start_integrator_ = 0.0f;
  encoder_delta_ = 0;
  encoder_phase_delta_ = 0;
  encoder_phase_total_ = 0;
  encoder_old_ = 0;
  encoder_have_old_ = false;
  cal_phase_ = 0;
  encoder_up_count_ = 0;
  encoder_down_count_ = 0;
  kv_index_ = 0;
  kv_sum_x_ = 0.0f;
  kv_sum_y_ = 0.0f;
  kv_sum_xx_ = 0.0f;
  kv_sum_xy_ = 0.0f;
  kv_count_ = 0;
  SetState(kWaitCalibrationComplete);
}

void AutoCalibrator::PollMillisecond() {
  if (config_.start && status_.state == kIdle) {
    config_.start = false;
    Begin();
  }

  status_.state_ms = state_ms_;

  if (status_.state == kIdle ||
      status_.state == kComplete ||
      status_.state == kFault) {
    return;
  }

  state_ms_++;

  if (bldc_->status().mode == BldcServo::Mode::kFault) {
    Fault(kFaultServo);
    return;
  }

  switch (status_.state) {
    case kWaitCalibrationComplete: {
      if (bldc_->status().mode == BldcServo::Mode::kCalibrationComplete ||
          bldc_->status().mode == BldcServo::Mode::kStopped) {
        SetState(kResistanceMeasure);
      } else if (state_ms_ > 5000) {
        Fault(kFaultTimeout);
      }
      break;
    }

    case kResistanceMeasure: {
      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kVoltageFoc;
      command.theta = 0.0f;
      command.voltage = cal_voltage_;
      bldc_->Command(command);

      if ((state_ms_ % 150) != 149) { break; }

      const float current = std::hypot(bldc_->status().d_A,
                                       bldc_->status().q_A);
      if (last_res_current_ > 0.0f && current > last_res_current_) {
        const float step_res =
            (cal_voltage_ - last_res_voltage_) /
            (current - last_res_current_);
        if (std::isfinite(step_res) && step_res > 0.0f) {
          status_.resistance_ohm = step_res;
        }
      }

      if (current < 0.60f * peak_res_current_) {
        low_res_voltage_ = cal_voltage_;
        low_res_current_ = current;
      }
      peak_res_voltage_ = cal_voltage_;
      peak_res_current_ = current;
      last_res_voltage_ = cal_voltage_;
      last_res_current_ = current;
      status_.sample_count++;

      const float power = current * cal_voltage_ * 1.5f;
      if (power > config_.resistance_power_W ||
          cal_voltage_ > config_.resistance_max_voltage) {
        if (peak_res_current_ <= 0.05f) {
          Fault(kFaultNoCurrent);
          return;
        }
        if (low_res_current_ > 0.0f && peak_res_current_ > low_res_current_) {
          status_.resistance_ohm =
              (peak_res_voltage_ - low_res_voltage_) /
              (peak_res_current_ - low_res_current_);
        }
        if (!std::isfinite(status_.resistance_ohm) ||
            status_.resistance_ohm <= 0.0f) {
          Fault(kFaultBadResistance);
          return;
        }
        bldc_->motor()->resistance_ohm = status_.resistance_ohm;
        SetState(kInductanceMeasure);
      } else {
        cal_voltage_ *= 1.1f;
      }
      break;
    }

    case kInductanceMeasure: {
      if (ind_index_ >= kIndPeriodCount) {
        if (best_ind_di_dt_ <= 0.0f) {
          Fault(kFaultBadInductance);
          return;
        }
        status_.inductance_H = best_ind_voltage_ / best_ind_di_dt_;
        if (!std::isfinite(status_.inductance_H) ||
            status_.inductance_H < 1.0e-6f) {
          Fault(kFaultBadInductance);
          return;
        }
        SetState(kComputePid);
        break;
      }

      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kMeasureInductance;
      command.d_V = config_.inductance_voltage;
      command.meas_ind_period = kIndPeriods[ind_index_];
      bldc_->Command(command);

      if (state_ms_ == 1) {
        ind_start_integrator_ = bldc_->status().meas_ind_integrator;
      }
      if (state_ms_ < 1000) { break; }

      const float di_dt = std::abs(
          bldc_->status().meas_ind_integrator - ind_start_integrator_);
      if (di_dt > best_ind_di_dt_) {
        best_ind_di_dt_ = di_dt;
        best_ind_voltage_ = config_.inductance_voltage;
      }
      ind_index_++;
      SetState(kInductanceMeasure);
      break;
    }

    case kComputePid: {
      const float w_3db = config_.pid_bw_hz * 2.0f * kPi;
      bldc_->config()->pid_dq.kp = w_3db * status_.inductance_H;
      bldc_->config()->pid_dq.ki = w_3db * status_.resistance_ohm;
      SetState(kEncoderPhaseUp);
      break;
    }

    case kEncoderPhaseUp:
      if (EncoderTick(true)) {
        encoder_phase_total_ = std::abs(encoder_phase_delta_);
        encoder_delta_ = 0;
        encoder_phase_delta_ = 0;
        encoder_have_old_ = false;
        SetState(kEncoderPhaseDown);
      }
      break;

    case kEncoderPhaseDown:
      if (EncoderTick(false)) {
        const int32_t down_phase_total = std::abs(encoder_phase_delta_);
        if (encoder_phase_total_ == 0) {
          encoder_phase_total_ = down_phase_total;
        } else {
          encoder_phase_total_ = (encoder_phase_total_ + down_phase_total) / 2;
        }
        SetState(kEncoderCompute);
      }
      break;

    case kEncoderCompute:
      ComputeEncoder();
      if (status_.state != kFault) {
        SetState(kKvMeasure);
      }
      break;

    case kKvMeasure: {
      const float factors[] = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
      if (kv_index_ >= kKvPointCount) {
        const float denom = kv_count_ * kv_sum_xx_ - kv_sum_x_ * kv_sum_x_;
        if (kv_count_ < 2 || std::abs(denom) < 1.0e-6f) {
          Fault(kFaultBadKv);
          return;
        }
        const float slope =
            (kv_count_ * kv_sum_xy_ - kv_sum_x_ * kv_sum_y_) / denom;
        const float v_per_hz = 1.0f / slope;
        status_.Kv = 1.14f * 0.5f * 60.0f / v_per_hz;
        if (!std::isfinite(status_.Kv) || status_.Kv <= 0.0f) {
          Fault(kFaultBadKv);
          return;
        }
        bldc_->motor()->Kv = status_.Kv;
        SetState(kWriteConfig);
        break;
      }

      const float voltage = factors[kv_index_] * config_.kv_voltage;
      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kVoltageDq;
      command.d_V = 0.0f;
      command.q_V = voltage;
      bldc_->Command(command);

      if (state_ms_ < 2000) { break; }
      const float speed_hz = bldc_->status().velocity;
      if (kv_index_ == 4 || std::abs(speed_hz) > 0.1f) {
        kv_sum_x_ += voltage;
        kv_sum_y_ += speed_hz;
        kv_sum_xx_ += voltage * voltage;
        kv_sum_xy_ += voltage * speed_hz;
        kv_count_++;
      }
      kv_index_++;
      SetState(kKvMeasure);
      break;
    }

    case kWriteConfig:
      CommandStop();
      if (config_.write_config) {
        persistent_config_->Write();
      }
      SetState(kComplete);
      break;

    case kIdle:
    case kComplete:
    case kFault:
      break;
  }
}

void AutoCalibrator::SetState(AutoCalState state) {
  status_.state = state;
  state_ms_ = 0;
  status_.state_ms = 0;
  if (state != kResistanceMeasure) {
    status_.sample_count = 0;
  }
}

void AutoCalibrator::Fault(uint32_t fault) {
  status_.fault = fault;
  CommandStop();
  SetState(kFault);
}

void AutoCalibrator::CommandStop() {
  BldcServo::CommandData command;
  command.mode = BldcServo::Mode::kStopped;
  bldc_->Command(command);
}

uint16_t AutoCalibrator::ReadCommutationEncoder() const {
  const auto* config = bldc_->motor_position_config();
  const auto& position = bldc_->motor_position();
  const int source = config->commutation_source;
  const uint64_t raw = static_cast<uint32_t>(
      position.sources[source].compensated_value);
  const uint64_t cpr = config->sources[source].cpr;
  if (cpr == 0) { return 0; }
  return static_cast<uint16_t>(std::min<uint64_t>(65535,
      (raw * 65536ull) / cpr));
}

bool AutoCalibrator::EncoderTick(bool up) {
  const uint16_t encoder = ReadCommutationEncoder();
  if (!encoder_have_old_) {
    encoder_have_old_ = true;
    encoder_old_ = encoder;
  } else {
    encoder_delta_ += static_cast<int16_t>(encoder - encoder_old_);
    encoder_old_ = encoder;
  }

  if ((state_ms_ % 10) == 0) {
    auto& samples = up ? encoder_up_ : encoder_down_;
    auto& count = up ? encoder_up_count_ : encoder_down_count_;
    if (count < samples.size()) {
      samples[count++] = EncoderSample{cal_phase_, encoder};
    }
  }

  const int step = static_cast<int>(
      config_.encoder_speed_hz * 65536.0f / 1000.0f);
  const int signed_step = up ? step : -step;
  cal_phase_ = static_cast<uint16_t>(cal_phase_ + signed_step);
  encoder_phase_delta_ += signed_step;

  BldcServo::CommandData command;
  command.mode = BldcServo::Mode::kVoltageFoc;
  command.theta = (cal_phase_ / 65536.0f) * k2Pi;
  command.voltage = config_.encoder_voltage;
  bldc_->Command(command);

  if (state_ms_ > 20000) {
    Fault(kFaultTimeout);
    return false;
  }
  return std::abs(encoder_delta_) > 65536;
}

void AutoCalibrator::ComputeEncoder() {
  if (encoder_up_count_ < 16 || encoder_down_count_ < 16) {
    Fault(kFaultNoEncoder);
    return;
  }

  const int32_t total_encoder = std::abs(encoder_delta_);
  if (total_encoder == 0) {
    Fault(kFaultNoEncoder);
    return;
  }

  const int poles = std::max(2, static_cast<int>(
      (2 * encoder_phase_total_ + total_encoder / 2) / total_encoder));
  status_.poles = static_cast<uint8_t>(std::min(254, poles));
  bldc_->motor()->poles = status_.poles;

  std::array<float, 64> accum = {};
  std::array<uint16_t, 64> count = {};

  auto accumulate = [&](const std::array<EncoderSample, 512>& samples,
                        uint16_t sample_count) {
    for (uint16_t i = 0; i < sample_count; i++) {
      const float encoder_ratio = samples[i].encoder / 65536.0f;
      const float measured = samples[i].phase / 65536.0f;
      const float nominal = WrapUnit(encoder_ratio * status_.poles / 2.0f);
      const float offset = WrapBalanced(measured - nominal) * k2Pi;
      const uint8_t bin = static_cast<uint8_t>(
          std::min(63, static_cast<int>(encoder_ratio * 64.0f)));
      accum[bin] += offset;
      count[bin]++;
    }
  };

  accumulate(encoder_up_, encoder_up_count_);
  accumulate(encoder_down_, encoder_down_count_);

  float last = 0.0f;
  for (std::size_t i = 0; i < bldc_->motor()->offset.size(); i++) {
    if (count[i] != 0) {
      last = accum[i] / count[i];
    }
    bldc_->motor()->offset[i] = last;
  }

  if (status_.poles < 2) {
    Fault(kFaultBadEncoder);
  } else {
    bldc_->ApplyConfig();
  }
}

}
