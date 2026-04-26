#pragma once

#include <array>
#include <cstdint>

#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/bldc_servo.h"

namespace moteus {

class AutoCalibrator {
 public:
  enum AutoCalState {
    kIdle,
    kWaitCalibrationComplete,
    kResistanceMeasure,
    kInductanceMeasure,
    kComputePid,
    kEncoderPhaseUp,
    kEncoderPhaseDown,
    kEncoderCompute,
    kKvMeasure,
    kWriteConfig,
    kComplete,
    kFault,
  };

  struct Config {
    bool start = false;
    bool write_config = true;
    float resistance_power_W = 5.0f;
    float resistance_max_voltage = 6.0f;
    float inductance_voltage = 1.0f;
    float pid_bw_hz = 100.0f;
    float encoder_voltage = 1.0f;
    float encoder_speed_hz = 1.0f;
    float kv_voltage = 1.0f;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(start));
      a->Visit(MJ_NVP(write_config));
      a->Visit(MJ_NVP(resistance_power_W));
      a->Visit(MJ_NVP(resistance_max_voltage));
      a->Visit(MJ_NVP(inductance_voltage));
      a->Visit(MJ_NVP(pid_bw_hz));
      a->Visit(MJ_NVP(encoder_voltage));
      a->Visit(MJ_NVP(encoder_speed_hz));
      a->Visit(MJ_NVP(kv_voltage));
    }
  };

  struct Status {
    AutoCalState state = kIdle;
    uint32_t state_ms = 0;
    uint16_t sample_count = 0;
    float resistance_ohm = 0.0f;
    float inductance_H = 0.0f;
    float Kv = 0.0f;
    uint8_t poles = 0;
    uint32_t fault = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(state));
      a->Visit(MJ_NVP(state_ms));
      a->Visit(MJ_NVP(sample_count));
      a->Visit(MJ_NVP(resistance_ohm));
      a->Visit(MJ_NVP(inductance_H));
      a->Visit(MJ_NVP(Kv));
      a->Visit(MJ_NVP(poles));
      a->Visit(MJ_NVP(fault));
    }
  };

  AutoCalibrator(mjlib::micro::PersistentConfig*,
                 mjlib::micro::TelemetryManager*,
                 BldcServo*);

  void PollMillisecond();
  const Status& status() const { return status_; }
  Config* config() { return &config_; }

 private:
  struct EncoderSample {
    uint16_t phase = 0;
    uint16_t encoder = 0;
  };

  void Begin();
  void SetState(AutoCalState);
  void Fault(uint32_t);
  bool EncoderTick(bool up);
  void CommandStop();
  uint16_t ReadCommutationEncoder() const;
  void ComputeEncoder();

  mjlib::micro::PersistentConfig* const persistent_config_;
  BldcServo* const bldc_;
  Config config_;
  Status status_;

  uint32_t state_ms_ = 0;
  float cal_voltage_ = 0.01f;
  float last_res_voltage_ = 0.0f;
  float last_res_current_ = 0.0f;
  float low_res_voltage_ = 0.0f;
  float low_res_current_ = 0.0f;
  float peak_res_voltage_ = 0.0f;
  float peak_res_current_ = 0.0f;

  uint8_t ind_index_ = 0;
  float best_ind_di_dt_ = 0.0f;
  float best_ind_voltage_ = 0.0f;
  float ind_start_integrator_ = 0.0f;

  int32_t encoder_delta_ = 0;
  int32_t encoder_phase_delta_ = 0;
  int32_t encoder_phase_total_ = 0;
  uint16_t encoder_old_ = 0;
  bool encoder_have_old_ = false;
  uint16_t cal_phase_ = 0;
  std::array<EncoderSample, 512> encoder_up_ = {};
  std::array<EncoderSample, 512> encoder_down_ = {};
  uint16_t encoder_up_count_ = 0;
  uint16_t encoder_down_count_ = 0;

  uint8_t kv_index_ = 0;
  float kv_sum_x_ = 0.0f;
  float kv_sum_y_ = 0.0f;
  float kv_sum_xx_ = 0.0f;
  float kv_sum_xy_ = 0.0f;
  uint8_t kv_count_ = 0;
};

}
