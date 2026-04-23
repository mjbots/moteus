#pragma once

#include <cstdint>
#include <mbed.h>

namespace moteus {

class CustomProtocol {
 public:
  bool HandleFrame(uint32_t can_id, int dlc, const char* data) {
    DigitalOut debugled(PB_15, 0);
    return false;
  }

  // Static trampoline matching FDCanMicroServer::CustomCanCallback.
  static bool CallbackTrampoline(uint32_t can_id, int dlc,
                                 const char* data, void* context) {
    return static_cast<CustomProtocol*>(context)
        ->HandleFrame(can_id, dlc, data);
  }
};

}
