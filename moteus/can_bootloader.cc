// Copyright 2019-2020 Josh Pieper, jjp@pobox.com.
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

#if defined(TARGET_STM32G4)
#include "stm32g4xx.h"
#else
#error "Unsupported target"
#endif

#include <string.h>

#include <cstdint>
#include <string_view>

#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/tokenizer.h"

namespace {
void uint8_hex(uint8_t value, char* buffer) {
  constexpr char digits[] = "0123456789ABCDEF";
  buffer[0] = digits[value >> 4];
  buffer[1] = digits[value & 0x0f];
  buffer[2] = 0;
}

void uint32_hex(uint32_t value, char* buffer) {
  for (size_t i = 0; i < 4; i++) {
    uint8_hex(value >> (3 - i) * 8, &buffer[i * 2]);
  }
}

// I was going to just use strtol, but for some reason linking it in
// caused boost::crc to stop working?  Oh well, this is easy enough
// for now.
uint32_t hex_to_i(const std::string_view& str) {
  uint32_t result = 0;
  for (char c : str) {
    result <<= 4;
    result |= [&]() {
      if (c >= '0' && c <= '9') { return c - '0'; }
      if (c >= 'a' && c <= 'f') { return c - 'a' + 0x0a; }
      if (c >= 'A' && c <= 'F') { return c - 'A' + 0x0a; }
      return 0;
    }();
  }
  return result;
}

class MillisecondTimer {
 public:
  uint32_t read_ms() {
    return TIM5->CNT / 1000;
  }

  uint32_t read_us() {
    return TIM5->CNT;
  }

  void wait_ms(uint32_t delay_ms) {
    wait_us(delay_ms * 1000);
  }

  void wait_us(uint32_t delay_us) {
    uint32_t current = TIM5->CNT;
    uint32_t elapsed = 0;
    while (true) {
      const uint32_t next = TIM5->CNT;
      elapsed += next - current;
      if (elapsed >= (delay_us + 1)) { return; }
      current = next;
    }
  }
};

template <typename T>
struct Buffer {
  T data[256] = {};
  size_t pos = 0;

  std::string_view view() const {
    return {reinterpret_cast<const char*>(data), pos};
  }

  size_t capacity() const {
    return sizeof(data) / sizeof(*data);
  }

  mjlib::base::BufferWriteStream writer() {
    return mjlib::base::BufferWriteStream(
        mjlib::base::string_span(&data[pos], capacity() - pos));
  }
};

class FlashWriter {
 public:
  void Unlock() {
    __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
    __HAL_FLASH_DATA_CACHE_DISABLE();
    // Given that we always do a system reset to exit the bootloader,
    // we just leave these disabled.

    if (!locked_) { return; }

    FLASH->SR |= (
        FLASH_SR_OPTVERR |
        FLASH_SR_RDERR |
        FLASH_SR_FASTERR |
        FLASH_SR_MISERR |
        FLASH_SR_PGSERR |
        FLASH_SR_SIZERR |
        FLASH_SR_PGAERR |
        FLASH_SR_WRPERR |
        FLASH_SR_PROGERR |
        FLASH_SR_OPERR |
        FLASH_SR_EOP);
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    locked_ = false;
  }

  bool Lock() {
    if (locked_) { return true; }

    if (shadow_bits_) {
      if (!FlushWord()) {
        return false;
      }
    }

    FLASH->CR |= FLASH_CR_LOCK;
    locked_ = true;
    return true;
  }

  bool ProgramByte(uint32_t intaddr, uint8_t value) {
    const uint32_t this_shadow = intaddr & ~(0x7);
    const uint32_t offset = intaddr & 0x7;

    if (this_shadow != shadow_start_ && shadow_start_ != 0) {
      if (!FlushWord()) {
        return false;
      }
      if (!MaybeEraseSector(this_shadow)) {
        return false;
      }
    }

    shadow_start_ = this_shadow;

    uint32_t mask = (0xffull << (offset * 8));
    shadow_ = (shadow_ & ~mask) |
        (static_cast<uint64_t>(value) << (offset * 8));
    shadow_bits_ |= mask;

    if (shadow_bits_ == 0xffffffffffffffffull) {
      if (!FlushWord()) {
        return false;
      }
    }

    return true;
  }

 private:
  bool FlushWord() {
    FLASH->CR |= FLASH_CR_PG;

    *reinterpret_cast<uint32_t*>(shadow_start_) =
        static_cast<uint32_t>(shadow_ & 0xffffffff);

    __ISB();

    *reinterpret_cast<uint32_t*>(shadow_start_ + 4u) =
        static_cast<uint32_t>(shadow_ >> 32u);

    Wait();

    shadow_start_ = 0;
    shadow_ = ~0;
    shadow_bits_ = 0;

    return true;
  }

  bool MaybeEraseSector(uint32_t address) {
    // Figure out which bank and sector we are in.
    const int bank = (address < 0x08040000) ? 1 : 2;
    const uint32_t bank_start = [&]() {
      switch (bank) {
        case 1: { return 0x08000000; }
        case 2: { return 0x08040000; }
      }
      return 0;
    }();
    const int sector = (address - bank_start) / 2048;

    const int sector_index = (bank - 1) * 128 + sector;
    if (!sectors_erased_[sector_index]) {
      if (!EraseSector(bank, sector)) {
        return false;
      }
      sectors_erased_[sector_index] = true;
    }

    return true;
  }

  bool EraseSector(int bank, int sector) {
    switch (bank) {
      case 1: {
        FLASH->CR &= ~FLASH_CR_BKER;
        break;
      }
      case 2: {
        FLASH->CR |= FLASH_CR_BKER;
        break;
      }
    }

    FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB_Msk) | (sector << FLASH_CR_PNB_Pos);

    FLASH->CR |= FLASH_CR_PER;
    FLASH->CR |= FLASH_CR_STRT;

    return Wait();
  }

  bool Wait() {
    while (FLASH->SR & FLASH_FLAG_BSY);

    const uint32_t error = (FLASH->SR & FLASH_FLAG_SR_ERRORS);
    if (error != 0u) {
      FLASH->SR |= error;
      return false;
    }

    if (FLASH->SR & FLASH_FLAG_EOP) {
      FLASH->SR |= FLASH_FLAG_EOP;
    }

    return true;
  }

  bool locked_ = true;
  uint32_t shadow_start_ = 0;
  uint64_t shadow_ = 0;
  uint64_t shadow_bits_ = 0;
  bool sectors_erased_[256] = {};
};

class BootloaderServer {
 public:
  BootloaderServer(uint8_t id, FDCAN_GlobalTypeDef* fdcan)
      : id_(id),
        fdcan_(fdcan) {
    auto writer = response_.writer();
    writer.write("multiplex bootloader protocol 1\r\n");
    response_.pos = writer.offset();
  }

  void Run() {
    while (true) {
      ReadCommand();

      RunCommand();
    }
  }

 private:
  void ReadCommand() {
    while (true) {
      ReadFrame();

      if (command_.view().find_first_of("\r\n") != std::string_view::npos) {
        return;
      }
    }
  }

  void ReadFrame() {
    // TODO
  }

  void RunCommand() {
    auto writer = response_.writer();

    auto command_end = command_.view().find_first_of("\r\n");

    mjlib::base::Tokenizer tokenizer(command_.view(), " \r\n");
    const auto next = tokenizer.next();
    if (next == std::string_view()) {
      // Empty line, just ignore.
    } else if (next == "echo") {
      // We will just echo back the remainder.
      writer.write(tokenizer.remaining());
    } else if (next == "unlock") {
      Unlock();
      writer.write("OK\r\n");
    } else if (next == "lock") {
      if (!Lock()) {
        writer.write("error locking\r\n");
      } else {
        writer.write("OK\r\n");
      }
    } else if (next == "w") {
      const auto address = tokenizer.next();
      const auto data = tokenizer.next();
      if (address.empty() || data.empty()) {
        writer.write("malformed write\r\n");
      } else {
        WriteFlash(address, data, writer);
      }
    } else if (next == "r") {
      const auto address = tokenizer.next();
      const auto size = tokenizer.next();
      if (address.empty() || size.empty()) {
        writer.write("malformed read\r\n");
      } else {
        ReadFlash(address, size, writer);
      }
    } else if (next == "reset") {
      // Make sure flash is back in the locked state before resetting.
      if (!Lock()) {
        writer.write("error locking\r\n");
      } else {
        NVIC_SystemReset();
      }
    } else if (next == "fault") {
      uint32_t* const value = reinterpret_cast<uint32_t*>(0x00200002);
      *value = 1;
    } else {
      writer.write("unknown command\r\n");
    }

    response_.pos += writer.offset();

    const auto to_consume = command_end + 1;
    std::memmove(command_.data, command_.data + to_consume,
                 command_.capacity() - to_consume);
    command_.pos -= to_consume;
  }

  void Unlock() {
    flash_.Unlock();
  }

  bool Lock() {
    return flash_.Lock();
  }

  void ReadFlash(const std::string_view& address_str,
                 const std::string_view& size_str,
                 mjlib::base::WriteStream& writer) {
    char buf[10] = {};

    const uint32_t start_address = hex_to_i(address_str);
    const uint32_t size = hex_to_i(size_str);
    if (size > 32) {
      writer.write("size too big\r\n");
      return;
    }
    uint32_hex(start_address, buf);
    writer.write(buf);
    writer.write(" ");

    for (uint32_t i = 0; i < size; i++) {
      const uint8_t* const value = reinterpret_cast<uint8_t*>(start_address + i);
      uint8_hex(*value, buf);
      writer.write(buf);
    }
    writer.write("\r\n");
  }

  void WriteFlash(const std::string_view& address_str,
                  const std::string_view& data_str,
                  mjlib::base::WriteStream& writer) {
    const auto start = timer_.read_us();
    if (data_str.size() % 2 != 0) {
      writer.write("odd data size\r\n");
      return;
    }
    const uint32_t start_address = hex_to_i(address_str);
    const uint32_t bytes = data_str.size() / 2;
    for (uint32_t i = 0; i < bytes; i++) {
      const uint8_t this_byte =
          hex_to_i(std::string_view(data_str.data() + i * 2, 2));
      if (!WriteByte(start_address + i, this_byte, writer)) {
        return;
      }
    }
    const auto end = timer_.read_us();
    writer.write("OK ");
    char buf[16] = {};
    uint32_hex((end - start), buf);
    writer.write(buf);
    writer.write("\r\n");
  }

  bool WriteByte(uint32_t address, uint8_t byte, mjlib::base::WriteStream& writer) {
    if (address < 0x08000000 ||
        address >= 0x08080000) {
      writer.write("address not in flash\r\n");
      return false;
    }

    if (address >= 0x0800c000 &&
        address < 0x08010000) {
      writer.write("address not writable\r\n");
      return false;
    }

    if (!flash_.ProgramByte(address, byte)) {
      writer.write("program error\r\n");
      return false;
    }
    return true;
  }

  const uint8_t id_;
  FDCAN_GlobalTypeDef* const fdcan_;

  MillisecondTimer timer_;

  // The current command line that is being received.
  Buffer<char> command_;

  // The thing we are going to send back.
  Buffer<char> response_;

  Buffer<char> out_frame_;

  FlashWriter flash_;
};

void BadInterrupt() {
  while (true);
}
}

extern "C" {

extern uint8_t __bss_start__;
extern uint8_t __bss_end__;

void __attribute__((__section__(".multiplex_bootloader")))
MultiplexBootloader(uint8_t source_id,
                    USART_TypeDef* uart,
                    GPIO_TypeDef* direction_port,
                    int direction_pin) {
  // While we are bootloading, we want no interrupts whatsoever.
  __disable_irq();

  // Manually zero out our BSS.
  ::memset(&__bss_start__, 0, &__bss_end__ - &__bss_start__);

  // We don't want any handlers to go into the original application
  // code, so point everything to a noop.
  for (int i = 0; i <= 113; i++) {
    const auto irq = static_cast<IRQn_Type>(i);

    if (irq == DebugMonitor_IRQn) { continue; }
    NVIC_SetVector(irq, reinterpret_cast<uint32_t>(&BadInterrupt));
  }

  BootloaderServer server(source_id, nullptr);
  server.Run();
}

void abort() {
  while (true) {}
}

}

namespace mjlib {
namespace base {

void assertion_failed(const char* expression, const char* filename, int line) {
  while (true);
}

}
}
