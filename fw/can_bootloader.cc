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
#include "mjlib/multiplex/format.h"
#include "mjlib/multiplex/stream.h"

#include "fw/git_info.h"
#include "fw/stm32g4xx_fdcan_typedefs.h"

namespace {
using mjlib::multiplex::Format;

template <typename T>
uint32_t u32(T value) {
  return static_cast<uint32_t>(value);
}

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
  bool locked() const {
    return locked_;
  }

  void Unlock() {
    // After calling unlock, assume we have to erase everything we
    // touch.
    for (auto& v : sectors_erased_) { v = false; }

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

  uint32_t Lock() {
    if (locked_) { return 0; }

    if (shadow_bits_) {
      const auto err = FlushWord();
      if (err) { return err; }
    }

    FLASH->CR |= FLASH_CR_LOCK;
    locked_ = true;
    return 0;
  }

  uint32_t ProgramByte(uint32_t intaddr, uint8_t value) {
    const uint32_t this_shadow = intaddr & ~(0x7);
    const uint32_t offset = intaddr & 0x7;

    if (this_shadow != shadow_start_ && shadow_start_ != 0) {
      const auto err = FlushWord();
      if (err) { return err; }
    }

    shadow_start_ = this_shadow;

    uint64_t mask = (0xffull << (offset * 8));
    shadow_ = (shadow_ & ~mask) |
        (static_cast<uint64_t>(value) << (offset * 8));
    shadow_bits_ |= mask;

    if (shadow_bits_ == 0xffffffffffffffffull) {
      const auto err = FlushWord();
      if (err) { return err; }
    }

    return 0;
  }

 private:
  uint32_t FlushWord() {
    const auto err = MaybeEraseSector(shadow_start_);
    if (err) { return err; }

    uint32_t result = 0;

    if (*reinterpret_cast<uint32_t*>(shadow_start_) ==
        static_cast<uint32_t>(shadow_ & 0xffffffff) &&
        *reinterpret_cast<uint32_t*>(shadow_start_ + 4u) ==
        static_cast<uint32_t>(shadow_ >> 32u)) {
      // nothing to do
    } else {
      FLASH->CR |= FLASH_CR_PG;

      *reinterpret_cast<uint32_t*>(shadow_start_) =
          static_cast<uint32_t>(shadow_ & 0xffffffff);

      __ISB();

      *reinterpret_cast<uint32_t*>(shadow_start_ + 4u) =
          static_cast<uint32_t>(shadow_ >> 32u);

      result = Wait();

      FLASH->CR &= ~FLASH_CR_PG;
    }

    shadow_start_ = 0;
    shadow_ = ~0;
    shadow_bits_ = 0;

    return result;
  }

  uint32_t MaybeEraseSector(uint32_t address) {
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
      const auto err = EraseSector(bank, sector);
      if (err) { return err; }
      sectors_erased_[sector_index] = true;
    }

    return 0;
  }

  uint32_t EraseSector(int bank, int sector) {
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

    const auto result = Wait();
    FLASH->CR &= ~FLASH_CR_PER;
    return result;
  }

  uint32_t Wait() {
    while (FLASH->SR & FLASH_FLAG_BSY);

    const uint32_t error = (FLASH->SR & FLASH_FLAG_SR_ERRORS);
    if (error != 0u) {
      FLASH->SR |= error;
      return error;
    }

    if (FLASH->SR & FLASH_FLAG_EOP) {
      FLASH->SR |= FLASH_FLAG_EOP;
    }

    return 0;
  }

  bool locked_ = true;
  uint32_t shadow_start_ = 0;
  uint64_t shadow_ = 0;
  uint64_t shadow_bits_ = 0;
  bool sectors_erased_[256] = {};

  FLASH_TypeDef* const flash_ = FLASH;
};

constexpr int kDlcToSize[] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8,
  12, 16, 20, 24, 32, 48, 64,
};

constexpr int RoundUpDlc(int size) {
  for (int dlc = 0; ; dlc++) {
    if (size <= kDlcToSize[dlc]) { return dlc; }
  }
  return 15;
}

class BootloaderServer {
 public:
  BootloaderServer(uint8_t id, FDCAN_GlobalTypeDef* fdcan)
      : id_(id),
        fdcan_(fdcan) {
    uint32_t SramCanInstanceBase = SRAMCAN_BASE;
    if (fdcan == FDCAN2) {
      SramCanInstanceBase += SRAMCAN_SIZE;
    }
    if (fdcan == FDCAN3) {
      SramCanInstanceBase += SRAMCAN_SIZE * 2U;
    }

    fdcan_RxFIFO0SA_ = SramCanInstanceBase + SRAMCAN_RF0SA;
    fdcan_TxFIFOQSA_ = SramCanInstanceBase + SRAMCAN_TFQSA;

    auto writer = response_.writer();
    writer.write("multiplex bootloader protocol 1 ");

    moteus::GitInfo git_info;
    for (auto v : git_info.hash) {
      char buf[3] = {};
      uint8_hex(v, buf);
      writer.write(buf);
    }
    writer.write(" ");
    writer.write(git_info.dirty ? "dirty" : "clean");
    writer.write("\r\n");

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

  struct CanFrame {
    int id_type = 0;
    uint32_t identifier = 0;
    uint32_t error_state_indicator = 0;
    uint32_t timestamp = 0;
    uint32_t dlc = 0;
    uint32_t bit_rate_switch = 0;
    uint32_t fdformat = 0;
    uint32_t filter_index = 0;

    uint32_t size = 0;
    uint8_t data[64] = {};
  };

  CanFrame ReadCanFrame() {
    CanFrame result;

    // Wait until there is a CAN frame available.
    while (true) {
      // Do we have a frame?
      if ((fdcan_->RXF0S & FDCAN_RXF0S_F0FL) != 0) { break; }

      if (fdcan_->PSR & FDCAN_PSR_BO) {
        // We went bus off.  Attempt to recover.
        fdcan_->CCCR &= ~FDCAN_CCCR_INIT;
        timer_.wait_us(10);
      }
    }

    const auto get_index = (fdcan_->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
    auto rx_address = reinterpret_cast<uint32_t*>(
        fdcan_RxFIFO0SA_ + get_index * SRAMCAN_RF0_SIZE);

    result.id_type = *rx_address & FDCAN_ELEMENT_MASK_XTD;
    result.identifier = [&]() {
      if (result.id_type == FDCAN_STANDARD_ID) {
        return (*rx_address & FDCAN_ELEMENT_MASK_STDID) >> 18u;
      }
      return *rx_address & FDCAN_ELEMENT_MASK_EXTID;
    }();

    result.error_state_indicator = *rx_address & FDCAN_ELEMENT_MASK_ESI;

    rx_address++;

    result.timestamp = *rx_address & FDCAN_ELEMENT_MASK_TS;
    result.dlc = (*rx_address & FDCAN_ELEMENT_MASK_DLC) >> 16u;
    result.bit_rate_switch = *rx_address & FDCAN_ELEMENT_MASK_BRS;
    result.fdformat = *rx_address & FDCAN_ELEMENT_MASK_FDF;
    result.filter_index = (*rx_address & FDCAN_ELEMENT_MASK_FIDX) >> 24u;

    rx_address++;

    result.size = kDlcToSize[result.dlc];
    auto pdata = reinterpret_cast<uint8_t*>(rx_address);
    for (size_t i = 0; i < result.size; i++) {
      result.data[i] = pdata[i];
    }

    // Acknowledge that we have read FIFO0.
    fdcan_->RXF0A = get_index;

    return result;
  }

  void ReadFrame() {
    const auto can_frame = ReadCanFrame();

    // Check if it is addressed to us.
    uint8_t source_id = (can_frame.identifier >> 8) & 0xff;
    uint8_t dest_id = (can_frame.identifier & 0xff);

    if (dest_id != id_) {
      return;
    }

    // Look for the subframe we know about.
    mjlib::base::BufferReadStream buffer_stream{
      std::string_view(reinterpret_cast<const char*>(can_frame.data),
                       can_frame.size)};
    mjlib::multiplex::ReadStream read_stream{buffer_stream};

    const auto maybe_subframe_id = read_stream.ReadVaruint();
    if (!maybe_subframe_id) { return; }

    if (*maybe_subframe_id != u32(Format::Subframe::kClientToServer) &&
        *maybe_subframe_id != u32(Format::Subframe::kClientPollServer)) {
      return;
    }

    const bool poll_only =
        (*maybe_subframe_id == u32(Format::Subframe::kClientPollServer));

    const bool query = (source_id & 0x80) != 0;

    const auto maybe_channel = read_stream.ReadVaruint();
    if (!maybe_channel) { return; }
    if (*maybe_channel != 1) { return; }

    const auto maybe_bytes = read_stream.ReadVaruint();
    if (!maybe_bytes) { return; }

    if (!poll_only) {
      if (*maybe_bytes > static_cast<size_t>(buffer_stream.remaining())) {
        return;
      }
      if (*maybe_bytes > 0) {
        if (command_.pos + *maybe_bytes > command_.capacity()) {
          // We would have overrun our command buffer.  Just empty it out
          // and discard this one too.
          command_.pos = 0;
          return;
        }

        // Great, we have some bytes, move the data into the command buffer.
        std::memcpy(&command_.data[command_.pos],
                    can_frame.data + buffer_stream.offset(),
                    *maybe_bytes);
        command_.pos += *maybe_bytes;
      }
    }

    if (query) {
      WriteResponse(source_id & 0x7f,
                    poll_only ? *maybe_bytes : -1,
                    can_frame);
    }
  }

  void WriteResponse(uint8_t id, int max_bytes,
                     const CanFrame& source_frame) {
    // Formulate our out frame.
    out_frame_.pos = 0;
    auto buffer_stream = out_frame_.writer();
    mjlib::multiplex::WriteStream write_stream(buffer_stream);

    constexpr size_t kMaxCanPayload = 64 - 3;

    const size_t bytes_to_write =
        std::min<size_t>(
            kMaxCanPayload,
            max_bytes >= 0 ?
            std::min<size_t>(max_bytes, response_.pos) :
            response_.pos);

    write_stream.WriteVaruint(u32(Format::Subframe::kServerToClient));
    write_stream.WriteVaruint(1);
    write_stream.WriteVaruint(bytes_to_write);
    buffer_stream.write(response_.view().substr(0, bytes_to_write));

    out_frame_.pos = buffer_stream.offset();

    // Mark that we've written out all we have.
    std::memmove(&response_.data[0], &response_.data[bytes_to_write],
                 response_.pos - bytes_to_write);
    response_.pos -= bytes_to_write;

    // Now queue up the transfer.

    WriteCanFrame(((id_ << 8) | id), out_frame_.view(), source_frame);
  }

  void WriteCanFrame(uint32_t identifier, std::string_view data,
                     const CanFrame& source_frame) {
    // If the queue is full, something is going seriously wrong.
    if (fdcan_->TXFQS & FDCAN_TXFQS_TFQF) {
      // Just drop the frame for now. :(
      return;
    }

    auto put_index = (fdcan_->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;

    // Copy the message.

    uint32_t dlc = RoundUpDlc(data.size());

    const uint32_t element_w1 =
        (identifier >= 2048) ?
        (FDCAN_ESI_ACTIVE |
         FDCAN_EXTENDED_ID |
         FDCAN_DATA_FRAME |
         identifier) :
        (FDCAN_ESI_ACTIVE |
         FDCAN_STANDARD_ID |
         FDCAN_DATA_FRAME |
         (identifier << 18u));
    const uint32_t message_marker = 0;
    const uint32_t element_w2 = (
        (message_marker << 24u) |
        FDCAN_NO_TX_EVENTS |
        FDCAN_FD_CAN |
        (source_frame.bit_rate_switch ? FDCAN_BRS_ON : 0) |
        (dlc << 16u));

    auto* const tx_address_base = reinterpret_cast<uint32_t*>(
        fdcan_TxFIFOQSA_ + put_index * SRAMCAN_TFQ_SIZE);

    auto tx_address = tx_address_base;

    *tx_address = element_w1;
    tx_address++;
    *tx_address = element_w2;
    tx_address++;

    const size_t rounded_up_size = kDlcToSize[dlc];

    for (size_t i = 0; i < rounded_up_size; i += 4) {
      auto get_byte = [&](int offset) {
        auto pos = i + offset;
        return pos < data.size() ? data[pos] : 0;
      };
      const uint32_t this_word =
          (get_byte(3) << 24) |
          (get_byte(2) << 16) |
          (get_byte(1) << 8) |
          (get_byte(0) << 0);

      *tx_address = this_word;
      tx_address++;
    }

    // Activate the request.
    fdcan_->TXBAR = (1 << put_index);

    // Ignore the buffer index.  We're never going to cancel anything.
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
      if (Lock()) {
        writer.write("ERR error locking\r\n");
      } else {
        writer.write("OK\r\n");
      }
    } else if (next == "w") {
      const auto address = tokenizer.next();
      const auto data = tokenizer.next();
      if (address.empty() || data.empty()) {
        writer.write("ERR malformed write\r\n");
      } else {
        WriteFlash(address, data, writer);
      }
    } else if (next == "r") {
      const auto address = tokenizer.next();
      const auto size = tokenizer.next();
      if (address.empty() || size.empty()) {
        writer.write("ERR malformed read\r\n");
      } else {
        ReadFlash(address, size, writer);
      }
    } else if (next == "reset") {
      // Make sure flash is back in the locked state before resetting.
      if (Lock()) {
        writer.write("ERR error locking\r\n");
      } else {
        NVIC_SystemReset();
      }
    } else if (next == "fault") {
      uint32_t* const value = reinterpret_cast<uint32_t*>(0x00200002);
      *value = 1;
    } else {
      writer.write("ERR unknown command\r\n");
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

  uint32_t Lock() {
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
    if (flash_.locked()) {
      writer.write("ERR flash is locked\r\n");
      return false;
    }

    if (address < 0x08000000 ||
        address >= 0x08080000) {
      writer.write("ERR address not in flash\r\n");
      return false;
    }

    if (address >= 0x0800c000 &&
        address < 0x08010000) {
      writer.write("ERR address not writable\r\n");
      return false;
    }

    const auto err = flash_.ProgramByte(address, byte);
    if (err) {
      writer.write("ERR program error ");
      char buf[5] = {};
      uint32_hex(err, buf);
      writer.write(buf);
      writer.write("\r\n");
      return false;
    }
    return true;
  }

  const uint8_t id_;
  FDCAN_GlobalTypeDef* const fdcan_;
  uint32_t fdcan_RxFIFO0SA_ = 0;
  uint32_t fdcan_TxFIFOQSA_ = 0;

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

extern char _sdata;
extern char _edata;
extern char _sidata;

void __attribute__((__section__(".multiplex_bootloader")))
MultiplexBootloader(uint8_t source_id,
                    USART_TypeDef* uart,
                    GPIO_TypeDef* direction_port,
                    int direction_pin) {
  // While we are bootloading, we want no interrupts whatsoever.
  __disable_irq();

  // Manually zero out our BSS.
  ::memset(&__bss_start__, 0, &__bss_end__ - &__bss_start__);

  // Copy our static data into place.
  char* dst = &_sdata;
  char* src = &_sidata;
  while (dst != &_edata) {
    *dst = *src;
    dst++;
    src++;
  }

  // We don't want any handlers to go into the original application
  // code, so point everything to a noop.
  for (int i = -14; i <= 102; i++) {
    const auto irq = static_cast<IRQn_Type>(i);

    if (irq == DebugMonitor_IRQn) { continue; }
    NVIC_SetVector(irq, reinterpret_cast<uint32_t>(&BadInterrupt));
  }

  // Try to figure out which FDCAN peripheral was in use.  We disable
  // protocol exception handling on the FDCAN peripheral we use, so
  // look for that.
  FDCAN_GlobalTypeDef* fdcan =
      [&]() {
        if (FDCAN1->CCCR & FDCAN_CCCR_PXHD) { return FDCAN1; }
        if (FDCAN2->CCCR & FDCAN_CCCR_PXHD) { return FDCAN2; }
        if (FDCAN3->CCCR & FDCAN_CCCR_PXHD) { return FDCAN3; }
        return FDCAN1;
      }();
  BootloaderServer server(source_id, fdcan);
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
