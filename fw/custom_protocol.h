#pragma once

#include <cstdint>

namespace moteus {

class CustomProtocol {
 public:
  // Called for every incoming CAN frame.
  //   can_id: raw CAN identifier (standard or extended)
  //   dlc:    payload length in bytes
  //   data:   pointer to payload bytes
  // Return true  -> frame consumed, skip official moteus protocol
  // Return false -> not recognized, fall through to official protocol
  bool HandleFrame(uint32_t can_id, int dlc, const char* data) {
    // TODO: Implement your custom protocol parsing here.
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
