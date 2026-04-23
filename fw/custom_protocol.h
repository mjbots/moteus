#pragma once

#include <cstdint>
#include <limits>
#include <mbed.h>

#include "fw/bldc_servo.h"
#include "fw/error.h"
#include "fw/fdcan.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/multiplex/micro_server.h"

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
    float torque;
    std::memcpy(&torque, data, sizeof(float));

    static BldcServo::CommandData command;
    command_private_.mode = BldcServo::Mode::kTorque;
    command_private_.torque_Nm = torque;
    command_private_.timeout_s = std::numeric_limits<float>::quiet_NaN();
    // bldc_servo_->Command(command_private_);
    return true;
  }
  bool HandleSetVelocity(int dlc, const char *data) {
    float velocity;
    std::memcpy(&velocity, data, sizeof(float));

    static BldcServo::CommandData command;
    command_private_.mode = BldcServo::Mode::kPosition;
    command_private_.position = std::numeric_limits<float>::quiet_NaN();
    command_private_.velocity = velocity;
    command_private_.kp_scale = 0.0f;
    command_private_.timeout_s = std::numeric_limits<float>::quiet_NaN();
    // bldc_servo_->Command(command_private_);
    return true;
  }
  bool HandleSetPosition(int dlc, const char *data) {
    float position;
    std::memcpy(&position, data, sizeof(float));

    static BldcServo::CommandData command;
    command_private_.mode = BldcServo::Mode::kPosition;
    command_private_.position = position;
    command_private_.velocity = 0.0f;
    command_private_.timeout_s = std::numeric_limits<float>::quiet_NaN();
    // bldc_servo_->Command(command_private_);
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
    auto *config = bldc_servo_->motor_position_config();
    config->output.offset = 0.0f;
    bldc_servo_->SetOutputPositionNearest(0.0f);

    const float cur_output = bldc_servo_->motor_position().position;
    const float error = 0.0f - cur_output;
    config->output.offset += error * config->output.sign;

    bldc_servo_->SetOutputPositionNearest(0.0f);

    persistent_config_->Write();
    return true;
  }
  bool HandleErrorReset(int dlc, const char *data) { return false; }
  bool HandleGetStatusword(int dlc, const char *data) { return false; }
  bool HandleGetValue1(int dlc, const char *data) { return false; }
  bool HandleGetValue2(int dlc, const char *data) { return false; }
  bool HandleSetConfig(int dlc, const char *data) { return false; }
  bool HandleGetConfig(int dlc, const char *data) { return false; }
  bool HandleSaveAllConfig(int dlc, const char *data) { return false; }
  bool HandleResetAllConfig(int dlc, const char *data) { return false; }
  bool HandleSync(int dlc, const char *data) { return false; }
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
  BldcServo::CommandData custom_command_private_;
};

} // namespace moteus
