// Copyright 2019 Josh Pieper, jjp@pobox.com.
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

#if defined(TARGET_STM32F4)
#include "stm32f4xx.h"
#else
#error "Unsupported target"
#endif

#include <string_view>

#include <boost/crc.hpp>

#include "mjlib/base/buffer_stream.h"
#include "mjlib/base/tokenizer.h"
#include "mjlib/multiplex/format.h"
#include "mjlib/multiplex/stream.h"

namespace {
using mjlib::multiplex::Format;

template <typename T>
uint32_t u32(T value) {
  return static_cast<uint32_t>(value);
}

// Copied from stm32f446_async_uart
struct Dma {
  DMA_Stream_TypeDef* stream;
  uint32_t channel;
  volatile uint32_t* status_clear;
  volatile uint32_t* status_register;
  uint32_t status_tcif;
  uint32_t status_htif;
  uint32_t status_teif;
  uint32_t status_dmeif;
  uint32_t status_feif;
  IRQn_Type irq;

  uint32_t all_status() const {
    return status_tcif |
        status_htif |
        status_teif |
        status_dmeif |
        status_feif;
  }
};

struct DmaPair {
  Dma tx;
  Dma rx;
};

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

#define MAKE_UART(DmaNumber, StreamNumber, ChannelNumber, StatusRegister) \
  Dma {                                                                 \
    DmaNumber ## _Stream ## StreamNumber,                               \
        (ChannelNumber) << DMA_SxCR_CHSEL_Pos,                          \
        & ( DmaNumber -> StatusRegister ## FCR ),                       \
        & ( DmaNumber -> StatusRegister ## SR ),                        \
        DMA_ ## StatusRegister ## SR_TCIF ## StreamNumber,              \
        DMA_ ## StatusRegister ## SR_HTIF ## StreamNumber,              \
        DMA_ ## StatusRegister ## SR_TEIF ## StreamNumber,              \
        DMA_ ## StatusRegister ## SR_DMEIF ## StreamNumber,             \
        DMA_ ## StatusRegister ## SR_FEIF ## StreamNumber,              \
        DmaNumber ## _Stream ## StreamNumber ## _IRQn,                  \
        }

DmaPair MakeDma(USART_TypeDef* uart) {
  if (uart == USART1) {
      return { MAKE_UART(DMA2, 7, 4, HI), MAKE_UART(DMA2, 2, 4, LI), };
  } else if (uart == USART2) {
    return { MAKE_UART(DMA1, 6, 4, HI), MAKE_UART(DMA1, 5, 4, HI), };
  } else if (uart == USART3) {
      return { MAKE_UART(DMA1, 3, 4, LI), MAKE_UART(DMA1, 1, 4, LI), };
  } else if (uart == UART4) {
    return { MAKE_UART(DMA1, 4, 4, HI), MAKE_UART(DMA1, 2, 4, LI), };
  } else if (uart == UART5) {
    return { MAKE_UART(DMA1, 7, 4, HI), MAKE_UART(DMA1, 0, 4, LI), };
  } else if (uart == USART6) {
    return { MAKE_UART(DMA2, 6, 5, HI), MAKE_UART(DMA2, 1, 5, LI), };
  }
  return {};
}

class DigitalOut {
 public:
  DigitalOut(GPIO_TypeDef* port, int pin) : port_(port), pin_(pin) {}

  void write(int value) {
    if (value) {
      port_->BSRR |= (1 << pin_);
    } else {
      port_->BSRR |= (1 << (pin_ + 16));
    }
  }

 private:
  GPIO_TypeDef* const port_;
  const int pin_;
};

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

class BootloaderServer {
 public:
  BootloaderServer(uint8_t id,
                   USART_TypeDef* uart, GPIO_TypeDef* dir_port, int dir_pin)
      : id_(id),
        uart_(uart),
        dir_(dir_port, dir_pin) {
    // We reconfigure the DMA entirely.  We need to re-point it at our
    // buffers in any event, and setting everything else up from
    // scratch is easy enough.
    const auto dma_pair = MakeDma(uart);
    dma_tx_ = dma_pair.tx;
    dma_rx_ = dma_pair.rx;

    // Make the tx DMA write to the uart.
    dma_tx_.stream->PAR = reinterpret_cast<uint32_t>(&(uart_->DR));
    dma_tx_.stream->CR =
        dma_tx_.channel |
        DMA_SxCR_MINC |
        DMA_MEMORY_TO_PERIPH;

    // Configure the RX to go into our buffer.  Notably, we enable no
    // interrupts, because we are going to poll.
    for (auto& item : rx_.data) { item = 0xffff; }

    dma_rx_.stream->PAR = reinterpret_cast<uint32_t>(&(uart_->DR));
    dma_rx_.stream -> CR =
        dma_rx_.channel |
        DMA_SxCR_MINC |
        DMA_PERIPH_TO_MEMORY |
        (0x1 << DMA_SxCR_MSIZE_Pos) | // 16-bit memory
        (0x1 << DMA_SxCR_PSIZE_Pos) | // 16-bit peripheral
        DMA_SxCR_CIRC;

    // Start reading into the circular buffer.
    dma_rx_.stream->M0AR = reinterpret_cast<uint32_t>(rx_.data);
    *dma_rx_.status_clear |= dma_rx_.all_status();
    dma_rx_.stream->NDTR = rx_.capacity();
    dma_rx_.stream->CR |= DMA_SxCR_EN;

    // Finally, enable receiving.
    uart_->CR3 |= USART_CR3_DMAR;

    // Send out our startup message.
    auto writer = response_.writer();
    writer.write("multiplex bootloader protocol 1\r\n");
    response_.pos = writer.offset();
  }

  void Run() {
    while (true) {
      // Read command.
      ReadCommand();

      // Do action.
      RunCommand();
    }
  }

  void WriteResponse(uint8_t id, int max_bytes) {
    // Formulate our out frame.
    out_frame_.pos = 0;
    auto buffer_stream = out_frame_.writer();
    mjlib::multiplex::WriteStream write_stream(buffer_stream);

    const size_t bytes_to_write =
        max_bytes >= 0 ?
        std::min<size_t>(max_bytes, response_.pos) :
        response_.pos;

    write_stream.Write(Format::kHeader);
    write_stream.Write<uint8_t>(id_);
    write_stream.Write<uint8_t>(id);
    write_stream.WriteVaruint(
        mjlib::multiplex::GetVaruintSize(u32(Format::Subframe::kServerToClient)) +
        mjlib::multiplex::GetVaruintSize(1) + // channel number
        mjlib::multiplex::GetVaruintSize(bytes_to_write) +
        bytes_to_write);

    write_stream.WriteVaruint(u32(Format::Subframe::kServerToClient));
    write_stream.WriteVaruint(1);
    write_stream.WriteVaruint(bytes_to_write);
    buffer_stream.write(response_.view().substr(0, bytes_to_write));

    // Calculate the CRC and write it out.
    boost::crc_ccitt_type crc;
    crc.process_bytes(out_frame_.data, buffer_stream.offset());
    write_stream.Write<uint16_t>(crc.checksum());

    out_frame_.pos = buffer_stream.offset();

    // Mark that we've written out all we have.
    std::memmove(&response_.data[0], &response_.data[bytes_to_write],
                 response_.pos - bytes_to_write);
    response_.pos -= bytes_to_write;


    // Now queue up the DMA transfer and wait for it to finish.


    // Set direction bit.
    dir_.write(1);

    *dma_tx_.status_clear |= dma_tx_.all_status();
    uart_->SR &= ~USART_SR_TC;

    dma_tx_.stream->NDTR = out_frame_.pos;
    dma_tx_.stream->M0AR = reinterpret_cast<uint32_t>(out_frame_.data);
    dma_tx_.stream->CR |= DMA_SxCR_EN;

    uart_->CR3 |= USART_CR3_DMAT;

    // Wait for transmission to complete.
    while (((*dma_tx_.status_register) & dma_tx_.status_tcif) == 0);
    while ((uart_->SR & USART_SR_TC) == 0);

    // Delay a bit.
    timer_.wait_us(2);

    // Clear the direction bit.
    dir_.write(0);
  }

  void ReadCommand() {
    while (true) {
      ReadFrame();

      if (command_.view().find_first_of("\r\n") != std::string_view::npos) {
        return;
      }
    }
  }

  void ReadFrame() {
    // Read until we get a start of frame.
    while (true) {
      frame_.pos = 0;
      const uint8_t hdr1 = ReadFrameByte();
      if (hdr1 != 0x54) { continue; }

      const uint8_t hdr2 = ReadFrameByte();
      if (hdr2 != 0xab) { continue; }

      break;
    }

    ReadFrameByte(); // source id
    ReadFrameByte(); // dest id
    const uint32_t size = ReadFrameVaruint();
    if (size > (frame_.capacity() - frame_.pos)) {
      // We'll just ignore this, since it is larger than we can
      // handle.
      return;
    }

    // Read the whole frame plus the checksum.
    for (uint32_t i = 0; i < size + 2; i++) {
      ReadFrameByte();
    }

    // Verify the checksum.
    boost::crc_ccitt_type crc;
    crc.process_bytes(frame_.data, frame_.pos - 2);
    const uint16_t expected_crc = crc.checksum();

    {
      mjlib::base::BufferReadStream buffer_stream{
        std::string_view{frame_.data + frame_.pos - 2, 2}};
      mjlib::multiplex::ReadStream read_stream{buffer_stream};
      const auto maybe_actual_crc = read_stream.ReadScalar<uint16_t>();
      if (!maybe_actual_crc) { return; }
      if (*maybe_actual_crc != expected_crc) { return; }
    }

    mjlib::base::BufferReadStream buffer_stream{
      std::string_view{frame_.data, frame_.pos - 2}};
    mjlib::multiplex::ReadStream read_stream{buffer_stream};
    read_stream.ReadScalar<uint16_t>();  // ignore frame header
    const auto maybe_source_id = read_stream.ReadScalar<uint8_t>();
    const auto maybe_dest_id = read_stream.ReadScalar<uint8_t>();

    if (!maybe_source_id || !maybe_dest_id) { return; }

    // Verify it is addressed to us.
    if (*maybe_dest_id != id_) { return; }

    read_stream.ReadVaruint();  // ignore the payload size

    // Look for the subframe we know about.
    const auto maybe_subframe_id = read_stream.ReadVaruint();
    if (!maybe_subframe_id) { return; }

    if (*maybe_subframe_id != u32(Format::Subframe::kClientToServer) &&
        *maybe_subframe_id != u32(Format::Subframe::kClientPollServer)) {
      return;
    }

    const bool poll_only =
        (*maybe_subframe_id == u32(Format::Subframe::kClientPollServer));

    const bool query = (*maybe_source_id & 0x80) != 0;

    const auto maybe_channel = read_stream.ReadVaruint();
    if (!maybe_channel) { return; }
    if (*maybe_channel != 1) { return; }

    const auto maybe_bytes = read_stream.ReadVaruint();
    if (!maybe_bytes) { return; }

    if (!poll_only) {
      if (*maybe_bytes > static_cast<size_t>(buffer_stream.remaining())) {
        return;
      }
      if (command_.pos + *maybe_bytes > command_.capacity()) {
        // We would have overrun our command buffer.  Just empty it out
        // and discard this one too.
        command_.pos = 0;
        return;
      }

      // Great, we have some bytes, move the data into the command buffer.
      std::memcpy(&command_.data[command_.pos], frame_.data + buffer_stream.offset(),
                  *maybe_bytes);
      command_.pos += *maybe_bytes;
    }

    if (query) {
      // Write out anything we've got after a short delay to give the
      // master a chance to switch back to receive mode.
      constexpr int kResponseDelayUs = 100;
      timer_.wait_us(kResponseDelayUs);

      WriteResponse(*maybe_source_id & 0x7f, poll_only ? *maybe_bytes : -1);
    }
  }

  uint32_t ReadFrameVaruint() {
    uint32_t result = 0;
    int pos = 0;
    for (int i = 0; i < 5; i++) {
      const auto this_byte = ReadFrameByte();
      result |= (this_byte & 0x7f) << pos;
      pos += 7;
      if ((this_byte & 0x80) == 0) { return result; }
    }
    return std::numeric_limits<uint32_t>::max();
  }

  uint8_t ReadFrameByte() {
    const uint8_t result = GetNextByte();
    frame_.data[frame_.pos] = result;
    frame_.pos++;
    return result;
  }

  uint8_t GetNextByte() {
    volatile uint16_t* data = rx_.data;

    int reset_count = 0;

    // Block until the DMA has populated something.
    while (data[rx_.pos] == 0xffff) {
      // Check to see if we perhaps got out of sync with the DMA.
      const auto write_position = rx_.capacity() - dma_rx_.stream->NDTR;
      if (write_position != rx_.pos) {
        reset_count++;
        if (reset_count > 1000) {
          // Wipe out everything and start over.
          std::memset(rx_.data, 0xff, sizeof(rx_.data));
          rx_.pos = write_position;
        }
      } else {
        reset_count = 0;
      }
    }

    const uint8_t result = data[rx_.pos];
    data[rx_.pos] = 0xffff;
    rx_.pos = (rx_.pos + 1) % rx_.capacity();
    return result;
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
      Lock();
      writer.write("OK\r\n");
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
      Lock();
      NVIC_SystemReset();
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
    if (!locked_) { return; }

    FLASH->SR |= FLASH_SR_PGSERR;
    FLASH->SR |= FLASH_SR_PGPERR;
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    locked_ = false;
  }

  void Lock() {
    if (locked_) { return; }

    FLASH->CR |= FLASH_CR_LOCK;
    locked_ = true;
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
    // What sector are we in.
    Sector* sector = [&]() -> Sector* {
      for (auto& sector : sectors_) {
        if (address >= sector.start &&
            address < (sector.start + sector.size)) {
          return &sector;
        }
      }
      return nullptr;
    }();

    if (!sector) {
      writer.write("address not in flash\r\n");
      return false;
    }

    if (!sector->programmable) {
      writer.write("sector not writeable\r\n");
      return false;
    }

    if (!sector->erased) {
      // Before we can write to this, we need to erase it.
      const uint32_t error = EraseSector(sector->number);
      if (error) {
        writer.write("erase error ");
        char buf[10] = {};
        uint32_hex(error, buf);
        writer.write(buf);
        writer.write("\r\n");
        return false;
      }
      sector->erased = true;
    }

    const uint32_t error = ProgramByte(sector->number, address, byte);
    if (error) {
      writer.write("program error ");
      char buf[10] = {};
      uint32_hex(error, buf);
      writer.write(buf);
      writer.write("\r\n");
      return false;
    }
    return true;
  }

  uint32_t EraseSector(uint32_t number) {
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR =
        // PSIZE = 0, so single byte
        (number << FLASH_CR_SNB_Pos) |
        FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;

    while (FLASH->SR & FLASH_SR_BSY);

    return FlashError();
  }

  uint32_t ProgramByte(uint32_t sector, uint32_t address, uint8_t byte) {
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR =
        (sector << FLASH_CR_SNB_Pos) |
        FLASH_CR_PG;

    *reinterpret_cast<volatile uint8_t*>(address) = byte;

    while (FLASH->SR & FLASH_SR_BSY);

    return FlashError();
  }

  uint32_t FlashError() {
    const uint32_t result = FLASH->SR & (
        FLASH_SR_WRPERR |
        FLASH_SR_PGAERR |
        FLASH_SR_PGPERR |
        FLASH_SR_PGSERR |
        FLASH_SR_RDERR);
    FLASH->SR = ~0;
    return result;
  }

 private:
  const uint8_t id_;
  USART_TypeDef* const uart_;
  DigitalOut dir_;

  MillisecondTimer timer_;
  Dma dma_tx_;
  Dma dma_rx_;

  // Data that the DMA is populating.
  Buffer<uint16_t> rx_;

  // The contents of one multiplex frame.
  Buffer<char> frame_;

  // The current command line that is being received.
  Buffer<char> command_;

  // The thing we are going to send back.
  Buffer<char> response_;

  Buffer<char> out_frame_;

  bool locked_ = true;

  struct Sector {
    uint32_t number;
    uint32_t start;
    uint32_t size;
    bool programmable;
    bool erased;
  };

  Sector sectors_[8] = {
    { 0, 0x8000000, 0x4000, true, false }, // The ISRs
    { 1, 0x8004000, 0x4000, false, false }, // PersistentConfig
    { 2, 0x8008000, 0x4000, true, false }, // unused
    { 3, 0x800c000, 0x4000, false, false }, // where we are located!
    { 4, 0x8010000, 0x10000, true, false },
    { 5, 0x8020000, 0x20000, true, false },
    { 6, 0x8040000, 0x20000, true, false },
    { 7, 0x8060000, 0x20000, true, false },
  };
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
  for (int i = -14; i <= 96; i++) {
    const auto irq = static_cast<IRQn_Type>(i);

    if (irq == DebugMonitor_IRQn) { continue; }
    NVIC_SetVector(irq, reinterpret_cast<uint32_t>(&BadInterrupt));
  }

  BootloaderServer server(source_id, uart, direction_port, direction_pin);
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
