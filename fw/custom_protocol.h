#pragma once

#include <cstdint>
#include <limits>
#include <mbed.h>

#include "fw/bldc_servo.h"
#include "fw/error.h"
#include "fw/fdcan.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/multiplex/micro_server.h"

using mjlib::micro::CommandManager;

namespace moteus {

class CustomProtocol {
public:
  CustomProtocol(mjlib::multiplex::MicroServer *multiplex_protocol,
                 BldcServo *bldc_servo, FDCan *fdcan,
                 mjlib::micro::PersistentConfig *persistent_config)
      : multiplex_protocol_(multiplex_protocol), bldc_servo_(bldc_servo),
        fdcan_(fdcan), persistent_config_(persistent_config) {}

  enum Mask : uint8_t {
    dir_offset = 10,
    node_offset = 5,
    cmd_offset = 0,
  };

  enum Dir : uint8_t {
    kReceive = 0,
    kSend = 1,
  };

  enum CmdId : uint8_t {
    CAN_CMD_MOTOR_DISABLE = 0,
    CAN_CMD_MOTOR_ENABLE = 1,
    CAN_CMD_SET_TORQUE = 2,
    CAN_CMD_SET_VELOCITY = 3,
    CAN_CMD_SET_POSITION = 4,
    CAN_CMD_CALIB_START = 5,
    CAN_CMD_CALIB_REPORT = 6,
    CAN_CMD_CALIB_ABORT = 7,
    CAN_CMD_ANTICOGGING_START = 8,
    CAN_CMD_ANTICOGGING_REPORT = 9,
    CAN_CMD_ANTICOGGING_ABORT = 10,
    CAN_CMD_SET_HOME = 11,
    CAN_CMD_ERROR_RESET = 12,
    CAN_CMD_GET_STATUSWORD = 13,
    CAN_CMD_STATUSWORD_REPORT = 14,
    CAN_CMD_GET_VALUE_1 = 15,
    CAN_CMD_GET_VALUE_2 = 16,
    CAN_CMD_SET_CONFIG = 17,
    CAN_CMD_GET_CONFIG = 18,
    CAN_CMD_SAVE_ALL_CONFIG = 19,
    CAN_CMD_RESET_ALL_CONFIG = 20,
    CAN_CMD_SYNC = 21,
    CAN_CMD_HEARTBEAT = 22,
    CAN_CMD_START_AUTO = 27,
    CAN_CMD_GET_FW_VERSION = 28,
    CAN_CMD_DFU_START = 29,
    CAN_CMD_DFU_DATA = 30,
    CAN_CMD_DFU_END = 31,
    CAN_CMD_COUNT = 32,
  };

  enum ConfigId : uint8_t {
    CONFIG_INVERT_MOTOR_DIR = 1,
    CONFIG_INERTIA = 2,
    CONFIG_TORQUE_CONSTANT = 3,
    CONFIG_MOTOR_POLE_PAIRS = 4,
    CONFIG_MOTOR_PHASE_RESISTANCE = 5,
    CONFIG_MOTOR_PHASE_INDUCTANCE = 6,
    CONFIG_CURRENT_LIMIT = 7,
    CONFIG_VELOCITY_LIMIT = 8,
    CONFIG_CALIB_CURRENT = 9,
    CONFIG_CALIB_VOLTAGE = 10,
    CONFIG_CONTROL_MODE = 11,
    CONFIG_POS_GAIN = 12,
    CONFIG_VEL_GAIN = 13,
    CONFIG_VEL_INTEGRATOR_GAIN = 14,
    CONFIG_CURRENT_CTRL_BW = 15,
    CONFIG_ANTICOGGING_ENABLE = 16,
    CONFIG_SYNC_TARGET_ENABLE = 17,
    CONFIG_TARGET_VELOCITY_WINDOW = 18,
    CONFIG_TARGET_POSITION_WINDOW = 19,
    CONFIG_TORQUE_RAMP_RATE = 20,
    CONFIG_VELOCITY_RAMP_RATE = 21,
    CONFIG_POSITION_FILTER_BW = 22,
    CONFIG_PROFILE_VELOCITY = 23,
    CONFIG_PROFILE_ACCEL = 24,
    CONFIG_PROFILE_DECEL = 25,
    CONFIG_PROTECT_UNDER_VOLTAGE = 26,
    CONFIG_PROTECT_OVER_VOLTAGE = 27,
    CONFIG_PROTECT_OVER_CURRENT = 28,
    CONFIG_PROTECT_I_BUS_MAX = 29,
    CONFIG_NODE_ID = 30,
    CONFIG_CAN_BAUDRATE = 31,
    CONFIG_HEARTBEAT_CONSUMER_MS = 32,
    CONFIG_HEARTBEAT_PRODUCER_MS = 33,
    CONFIG_CALIB_VALID = 34,
    CONFIG_ENCODER_DIR = 35,
    CONFIG_ENCODER_OFFSET = 36,
    CONFIG_OFFSET_LUT = 37,
    CONFIG_COUNT = 38,
  };

  static constexpr int8_t kDlcAny = -1;
  static constexpr int8_t kDlcNotUsed = -2;

  struct CmdEntry {
    int8_t expected_dlc;
    bool (CustomProtocol::*handler)(int dlc, const char *data);
  };
  bool SendFrame(uint32_t can_id, int dlc, const char *data) {
    if (fdcan_ == nullptr || dlc < 0 || dlc > 64) {
      return false;
    }
    FDCan::SendOptions options;
    options.fdcan_frame = FDCan::Override::kRequire;
    options.bitrate_switch = FDCan::Override::kRequire;
    fdcan_->Send(can_id, std::string_view(data, dlc), options);
    return true;
  }
  bool HandleFrame(uint32_t can_id, int dlc, const char *data) {
    static DigitalOut debug_led_canfd(PB_15, 0);

    const uint8_t dir = (can_id >> dir_offset) & 0x01;
    if (dir != kReceive) {
      return false;
    }

    const uint8_t node_id = (can_id >> node_offset) & 0x1F;
    if (node_id != multiplex_protocol_->config()->id) {
      return false;
    }

    const uint8_t cmd_id = (can_id >> cmd_offset) & 0x1F;
    if (cmd_id >= CAN_CMD_COUNT) {
      return false;
    }

    const CmdEntry &entry = kCommandTable[cmd_id];
    if (entry.handler == nullptr) {
      return false;
    }

    if (entry.expected_dlc == kDlcNotUsed) {
      return false;
    }
    if (entry.expected_dlc >= 0 && dlc != entry.expected_dlc) {
      return false;
    }
    debug_led_canfd = 1;

    (this->*(entry.handler))(dlc, data);

    debug_led_canfd = 0;
    return true;
  }

  static bool CallbackTrampoline(uint32_t can_id, int dlc, const char *data,
                                 void *context) {
    return static_cast<CustomProtocol *>(context)->HandleFrame(can_id, dlc,
                                                               data);
  }

private:
  bool HandleMotorDisable(int dlc, const char *data) {
    if (bldc_servo_ == nullptr) {
      return false;
    }

    if (bldc_servo_->status().mode != BldcServo::Mode::kStopped) {
      static BldcServo::CommandData command;

      command.mode = BldcServo::Mode::kStopped;
      bldc_servo_->Command(command);
    }
    char reply[4] = {0};
    SendFrame(kSend << dir_offset |
                  (multiplex_protocol_->config()->id << node_offset) |
                  CAN_CMD_MOTOR_DISABLE,
              4, reply);
    return true;
  }

  bool HandleMotorEnable(int dlc, const char *data) {
    if (bldc_servo_ == nullptr) {
      return false;
    }
    if (bldc_servo_->status().mode == BldcServo::Mode::kFault) {
      return false;
    }
    static BldcServo::CommandData command;
    command.mode = BldcServo::Mode::kPosition;
    command.position = std::numeric_limits<float>::quiet_NaN();
    command.velocity = 0.0f;
    command.timeout_s = std::numeric_limits<float>::quiet_NaN();
    bldc_servo_->Command(command);

    if (bldc_servo_->status().mode != BldcServo::Mode::kPosition) {
      int32_t fault_value = static_cast<int32_t>(bldc_servo_->status().fault);
      // 防止出现错误码为kSuccess但模式为kFault时错误地返回成功状态
      char reply[4] = {0xFF};
      reply[0] = fault_value & 0xFF;
      reply[1] = (fault_value >> 8) & 0xFF;
      reply[2] = (fault_value >> 16) & 0xFF;
      reply[3] = (fault_value >> 24) & 0xFF;
      SendFrame(kSend << dir_offset |
                    (multiplex_protocol_->config()->id << node_offset) |
                    CAN_CMD_MOTOR_ENABLE,
                4, reply);
      return false;
    } else {
      char reply[4] = {0};
      SendFrame(kSend << dir_offset |
                    (multiplex_protocol_->config()->id << node_offset) |
                    CAN_CMD_MOTOR_ENABLE,
                4, reply);
      return true;
    }
    return false;
  }

  bool HandleSetTorque(int dlc, const char *data) {
    std::memcpy(&pending_.feedforward_Nm, data, sizeof(float));
    return true;
  }
  bool HandleSetVelocity(int dlc, const char *data) {
    std::memcpy(&pending_.velocity, data, sizeof(float));
    return true;
  }
  bool HandleSetPosition(int dlc, const char *data) {
    std::memcpy(&pending_.position, data, sizeof(float));
    return true;
  }
  bool HandleCalibStart(int dlc, const char *data) { return false; }
  bool HandleCalibAbort(int dlc, const char *data) { return false; }
  bool HandleAnticoggingStart(int dlc, const char *data) { return false; }
  bool HandleAnticoggingAbort(int dlc, const char *data) { return false; }
  bool HandleSetHome(int dlc, const char *data) {
    if (bldc_servo_ == nullptr) {
      return false;
    }
    if (bldc_servo_->motor_position().homed < MotorPosition::Status::kRotor) {
      char reply[4] = {0xFF};
      SendFrame(kSend << dir_offset |
                    (multiplex_protocol_->config()->id << node_offset) |
                    CAN_CMD_SET_HOME,
                4, reply);
      return false;
    }
    auto *config = bldc_servo_->motor_position_config();
    config->output.offset = 0.0f;
    bldc_servo_->SetOutputPositionNearest(0.0f);

    const float cur_output = bldc_servo_->motor_position().position;
    const float error = 0.0f - cur_output;
    config->output.offset += error * config->output.sign;

    bldc_servo_->SetOutputPositionNearest(0.0f);

    struct NullAsyncWriteStream : public mjlib::micro::AsyncWriteStream {
      void AsyncWriteSome(const std::string_view&, const mjlib::micro::SizeCallback& callback) override {
        callback(mjlib::micro::ErrorCode(), 0);
      }
    } null_stream;

    bool write_success = true;
    NullAsyncWriteStream stream;
    CommandManager::Response response(&stream, [&write_success](mjlib::micro::ErrorCode ec, std::ptrdiff_t) {
      if (ec) {
        write_success = false;
      }
    });
    persistent_config_->Command("write", response);
    if (!write_success) {
      return false;
    }

    char reply[4] = {0};
    SendFrame(kSend << dir_offset |
                  (multiplex_protocol_->config()->id << node_offset) |
                  CAN_CMD_SET_HOME,
              4, reply);
    return true;
  }
  bool HandleErrorReset(int dlc, const char *data) {
    if (bldc_servo_ == nullptr) {
      return false;
    }

    if (bldc_servo_->status().mode != BldcServo::Mode::kStopped) {
      static BldcServo::CommandData command;

      command.mode = BldcServo::Mode::kStopped;
      bldc_servo_->Command(command);
    }

    char reply[4] = {0};
    SendFrame(kSend << dir_offset |
                  (multiplex_protocol_->config()->id << node_offset) |
                  CAN_CMD_ERROR_RESET,
              4, reply);
    return true;
  }
  bool HandleGetStatusword(int dlc, const char *data) { 
    return false; }
  bool HandleGetValue1(int dlc, const char *data) { return false; }
  bool HandleGetValue2(int dlc, const char *data) { return false; }
  bool HandleSetConfig(int dlc, const char *data) { return false; }
  bool HandleGetConfig(int dlc, const char *data) { return false; }
  bool HandleSaveAllConfig(int dlc, const char *data) { return false; }
  bool HandleResetAllConfig(int dlc, const char *data) { return false; }
  bool HandleSync(int dlc, const char *data) {
    if (bldc_servo_ == nullptr)
      return false;
    if (bldc_servo_->status().mode == BldcServo::Mode::kFault)
      return false;
    bldc_servo_->Command(pending_);
    return true;
  }
  bool HandleHeartbeat(int dlc, const char *data) { return false; }
  bool HandleStartAuto(int dlc, const char *data) { return false; }
  bool HandleGetFwVersion(int dlc, const char *data) { return false; }
  bool HandleDfuStart(int dlc, const char *data) { return false; }
  bool HandleDfuData(int dlc, const char *data) {
    if (dlc < 1 || dlc > 8) {
      return false;
    }
    return false;
  }
  bool HandleDfuEnd(int dlc, const char *data) { return false; }

  // Dispatch table: index = cmd_id.
  //   expected_dlc:
  //     >= 0          : exact match required
  //     kDlcAny  (-1) : any length accepted (handler checks)
  //     kDlcNotUsed(-2): not accepted from host (device->host only or reserved)
  //   handler == nullptr: unknown/unused slot
  static constexpr CmdEntry kCommandTable[CAN_CMD_COUNT] = {
      /*  0 MOTOR_DISABLE      */ {0, &CustomProtocol::HandleMotorDisable},
      /*  1 MOTOR_ENABLE       */ {0, &CustomProtocol::HandleMotorEnable},
      /*  2 SET_TORQUE         */ {4, &CustomProtocol::HandleSetTorque},
      /*  3 SET_VELOCITY       */ {4, &CustomProtocol::HandleSetVelocity},
      /*  4 SET_POSITION       */ {4, &CustomProtocol::HandleSetPosition},
      /*  5 CALIB_START        */ {0, &CustomProtocol::HandleCalibStart},
      /*  6 CALIB_REPORT       */ {kDlcNotUsed, nullptr},
      /*  7 CALIB_ABORT        */ {0, &CustomProtocol::HandleCalibAbort},
      /*  8 ANTICOGGING_START  */ {0, &CustomProtocol::HandleAnticoggingStart},
      /*  9 ANTICOGGING_REPORT */ {kDlcNotUsed, nullptr},
      /* 10 ANTICOGGING_ABORT  */ {0, &CustomProtocol::HandleAnticoggingAbort},
      /* 11 SET_HOME           */ {0, &CustomProtocol::HandleSetHome},
      /* 12 ERROR_RESET        */ {0, &CustomProtocol::HandleErrorReset},
      /* 13 GET_STATUSWORD     */ {0, &CustomProtocol::HandleGetStatusword},
      /* 14 STATUSWORD_REPORT  */ {kDlcNotUsed, nullptr},
      /* 15 GET_VALUE_1        */ {0, &CustomProtocol::HandleGetValue1},
      /* 16 GET_VALUE_2        */ {0, &CustomProtocol::HandleGetValue2},
      /* 17 SET_CONFIG         */ {8, &CustomProtocol::HandleSetConfig},
      /* 18 GET_CONFIG         */ {4, &CustomProtocol::HandleGetConfig},
      /* 19 SAVE_ALL_CONFIG    */ {0, &CustomProtocol::HandleSaveAllConfig},
      /* 20 RESET_ALL_CONFIG   */ {0, &CustomProtocol::HandleResetAllConfig},
      /* 21 SYNC               */ {0, &CustomProtocol::HandleSync},
      /* 22 HEARTBEAT          */ {0, &CustomProtocol::HandleHeartbeat},
      /* 23 reserved           */ {kDlcNotUsed, nullptr},
      /* 24 reserved           */ {kDlcNotUsed, nullptr},
      /* 25 reserved           */ {kDlcNotUsed, nullptr},
      /* 26 reserved           */ {kDlcNotUsed, nullptr},
      /* 27 START_AUTO         */ {1, &CustomProtocol::HandleStartAuto},
      /* 28 GET_FW_VERSION     */ {0, &CustomProtocol::HandleGetFwVersion},
      /* 29 DFU_START          */ {0, &CustomProtocol::HandleDfuStart},
      /* 30 DFU_DATA           */ {kDlcAny, &CustomProtocol::HandleDfuData},
      /* 31 DFU_END            */ {8, &CustomProtocol::HandleDfuEnd},
  };

  mjlib::multiplex::MicroServer *const multiplex_protocol_;
  BldcServo *const bldc_servo_;
  FDCan *const fdcan_;
  mjlib::micro::PersistentConfig *const persistent_config_;
  BldcServo::CommandData pending_;

  // 速度                bldc_servo_->status().velocity
  // 位置                bldc_servo_->status().position
  // hall偏移量          bldc_servo_->motor_position().sources[0].offset_value
  // hall值              bldc_servo_->motor_position().sources[0].raw
  // status_code         bldc_servo_->status().mode
  // errors_code         bldc_servo_->status().fault
  // 板载NTC             bldc_servo_->status().fet_temp_C
  // Iq电流              bldc_servo_->status().iq_current

  uint32_t status_;
  uint32_t errors_;

  // config params
  int32_t invert_motor_dir_;
  float inertia_;
  float torque_constant_;
  int32_t motor_pole_pairs_;
  float motor_phase_resistance_;
  float motor_phase_inductance_;
  float current_limit_;
  float velocity_limit_;
  float calib_current_;
  float calib_voltage_;
  int32_t control_mode_;
  float pos_gain_;
  float vel_gain_;
  float vel_integrator_gain_;
  float current_ctrl_bw_;
  int32_t anticogging_enable_;
  int32_t sync_target_enable_;
  float target_velocity_window_;
  float target_position_window_;
  float torque_ramp_rate_;
  float velocity_ramp_rate_;
  float position_filter_bw_;
  float profile_velocity_;
  float profile_accel_;
  float profile_decel_;
  float protect_under_voltage_;
  float protect_over_voltage_;
  float protect_over_current_;
  float protect_i_bus_max_;
  int32_t node_id_;
  int32_t can_baudrate_;
  int32_t heartbeat_consumer_ms_;
  int32_t heartbeat_producer_ms_;
  int32_t calib_valid_;
  int32_t encoder_dir_;
  int32_t encoder_offset_;
  int32_t offset_lut_;
};

} // namespace moteus
