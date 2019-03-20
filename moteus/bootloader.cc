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

#include "stm32f4xx.h"

#include <string_view>

#include <boost/crc.hpp>

#include "mjlib/base/buffer_stream.h"
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
  }

  void Run() {
    while (true) {
      // Read command.
      ReadCommand();

      // Do action.
      RunCommand();
    }
  }

  void WriteResponse(uint8_t id) {
    // Formulate our out frame.
    mjlib::base::BufferWriteStream buffer_stream(
        mjlib::base::string_span(out_frame_.data, out_frame_.capacity()));
    mjlib::multiplex::WriteStream write_stream(buffer_stream);

    write_stream.Write(Format::kHeader);
    write_stream.Write<uint8_t>(id_);
    write_stream.Write<uint8_t>(id);
    write_stream.WriteVaruint(
        mjlib::multiplex::GetVaruintSize(u32(Format::Subframe::kServerToClient)) +
        mjlib::multiplex::GetVaruintSize(1) + // channel number
        mjlib::multiplex::GetVaruintSize(response_.pos) +
        response_.pos);

    write_stream.WriteVaruint(u32(Format::Subframe::kServerToClient));
    write_stream.WriteVaruint(1);
    write_stream.WriteVaruint(response_.pos);
    buffer_stream.write(response_.view());

    // Calculate the CRC and write it out.
    boost::crc_ccitt_type crc;
    crc.process_bytes(out_frame_.data, buffer_stream.offset());
    write_stream.Write<uint16_t>(crc.checksum());

    out_frame_.pos = buffer_stream.offset();

    // Mark that we've written out all we have.
    response_.pos = 0;


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

      // TODO: Look in the command buffer and see if we have a full
      // line yet.
      if (frame_.view().find('\n') != std::string_view::npos) {
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

    if (*maybe_subframe_id != u32(Format::Subframe::kClientToServer)) {
      return;
    }

    const bool query = (*maybe_source_id & 0x80) != 0;

    const auto maybe_channel = read_stream.ReadVaruint();
    if (!maybe_channel) { return; }
    if (*maybe_channel != 1) { return; }

    const auto maybe_bytes = read_stream.ReadVaruint();
    if (!maybe_bytes) { return; }
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

    if (query) {
      // Write out anything we've got.
      WriteResponse(*maybe_source_id & 0x7f);
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
    // Block until the DMA has populated something.
    while (rx_.data[rx_.pos] == 0xffff);

    const uint8_t result = rx_.data[rx_.pos];
    rx_.data[rx_.pos] = 0xffff;
    rx_.pos = (rx_.pos + 1) % rx_.capacity();
    return result;
  }

  void RunCommand() {

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
};

}

extern "C" {

void MultiplexBootloader() {
  // While we are bootloading, we want no interrupts whatsoever.
  __disable_irq();

  BootloaderServer server(1, USART3, GPIOA, 8);
  server.Run();
}

// We never expect to exit normally, so there is no need to include
// any of the exit handling machinery.
void __wrap_atexit(void (*func)()) {
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
