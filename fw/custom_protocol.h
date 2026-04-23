#pragma once

#include <cstdint>
#include <limits>
#include <mbed.h>

#include "fw/bldc_servo.h"
#include "mjlib/multiplex/micro_server.h"

namespace moteus {

class CustomProtocol {
public:
  CustomProtocol(mjlib::multiplex::MicroServer *multiplex_protocol,
                 BldcServo *bldc_servo)
      : multiplex_protocol_(multiplex_protocol), bldc_servo_(bldc_servo) {}

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
    bool (CustomProtocol::*handler)(int dlc, const uint8_t *data);
  };

  bool HandleFrame(uint32_t can_id, int dlc, const char *data) {
    static DigitalOut debug_led_canfd(PB_15, 0);

    const uint8_t dir = (can_id >> 10) & 0x01;
    if (dir != 0) {
      return false;
    }

    const uint8_t node_id = (can_id >> 5) & 0x1F;
    if (node_id != multiplex_protocol_->config()->id) {
      return false;
    }

    const uint8_t cmd_id = (can_id >> 0) & 0x1F;
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

    const bool ok = (this->*(entry.handler))(dlc, *data);

    debug_led_canfd = 0;
    return ok;
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
    BldcServo::CommandData command;
    command.mode = BldcServo::Mode::kStopped;
    bldc_servo_->Command(command);
    return true;
  }
  bool HandleMotorEnable(int dlc, const char *data) {
    if (bldc_servo_ == nullptr) {
      return false;
    }
    BldcServo::CommandData command;
    command.mode = BldcServo::Mode::kPosition;
    command.position = std::numeric_limits<float>::quiet_NaN();
    command.velocity = 0.0f;
    command.max_torque_Nm = 10.0f;
    command.timeout_s = std::numeric_limits<float>::quiet_NaN();
    bldc_servo_->Command(command);
    return true;
  }
  bool HandleSetTorque(int dlc, const char *data) { return true; }
  bool HandleSetVelocity(int dlc, const char *data) { return true; }
  bool HandleSetPosition(int dlc, const char *data) { return true; }
  bool HandleCalibStart(int dlc, const char *data) { return true; }
  bool HandleCalibAbort(int dlc, const char *data) { return true; }
  bool HandleAnticoggingStart(int dlc, const char *data) { return true; }
  bool HandleAnticoggingAbort(int dlc, const char *data) { return true; }
  bool HandleSetHome(int dlc, const char *data) { return true; }
  bool HandleErrorReset(int dlc, const char *data) { return true; }
  bool HandleGetStatusword(int dlc, const char *data) { return true; }
  bool HandleGetValue1(int dlc, const char *data) { return true; }
  bool HandleGetValue2(int dlc, const char *data) { return true; }
  bool HandleSetConfig(int dlc, const char *data) { return true; }
  bool HandleGetConfig(int dlc, const char *data) { return true; }
  bool HandleSaveAllConfig(int dlc, const char *data) { return true; }
  bool HandleResetAllConfig(int dlc, const char *data) { return true; }
  bool HandleSync(int dlc, const char *data) { return true; }
  bool HandleHeartbeat(int dlc, const char *data) { return true; }
  bool HandleStartAuto(int dlc, const char *data) { return true; }
  bool HandleGetFwVersion(int dlc, const char *data) { return true; }
  bool HandleDfuStart(int dlc, const char *data) { return true; }
  bool HandleDfuData(int dlc, const char *data) {
    if (dlc < 1 || dlc > 8) {
      return false;
    }
    return true;
  }
  bool HandleDfuEnd(int dlc, const char *data) { return true; }

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
};

} // namespace moteus
