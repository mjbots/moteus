#pragma once

#include <cstdint>
#include <mbed.h>

#include "multiplex_protocol.h"

namespace moteus {

class CustomProtocol {
public:
  bool HandleFrame(uint32_t can_id, int dlc, const char *data) {
    static DigitalOut debug_led_canfd(PB_15, 1);

    const uint8_t dir = (can_id >> 10) & 0x01;
    const uint8_t node_id = (can_id >> 5) & 0x0F;
    const uint8_t cmd_id = (can_id >> 0) & 0x0F;

    if (dir != 0) {
      return false;
    }
    if (node_id != multiplex_protocol.config()->id) {
      return false;
    }
    debug_led_canfd = 1;

    switch (cmd_id) {
    default:
      return false;
    }
    debug_led_canfd = 0;
    return true;
  }

  // Static trampoline matching FDCanMicroServer::CustomCanCallback.
  static bool CallbackTrampoline(uint32_t can_id, int dlc, const char *data,
                                 void *context) {
    return static_cast<CustomProtocol *>(context)->HandleFrame(can_id, dlc,
                                                               data);
  }

private:
};

} // namespace moteus
