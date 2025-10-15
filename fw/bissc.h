// Copyright 2025 mjbots Robotic Systems, LLC.  info@mjbots.com
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

#pragma once

#include "fw/aux_common.h"
#include "fw/bissc_extract.h"
#include "fw/mbed_util.h"
#include "fw/millisecond_timer.h"
#include "fw/stm32_dma.h"

namespace moteus {

// This manages a BiSS-C connection.
class BissC {
 public:
  template <typename Array>
  BissC(const aux::BissC::Config& config,
        aux::BissC::Status* status,
        const Array& array,
        size_t array_size,
        const aux::AuxHardwareConfig& hw_config,
        DMA_Channel_TypeDef* dma_channel,
        MillisecondTimer* timer)
      : config_(config),
        status_(status),
        dma_channel_(dma_channel),
        timer_(timer) {
    // If we're not enabled, we really shouldn't be here.
    MJ_ASSERT(config_.enabled);

    aux::AuxPinConfig pin_tx = {};
    aux::AuxPinConfig pin_rx = {};
    aux::Pin::Pull pin_rx_pull = aux::Pin::Pull::kNone;

    for (size_t i = 0; i < array_size; i++) {
      const auto& pin = array[i];

      if (pin.mode != aux::Pin::Mode::kBissC) { continue; }

      // Check if any hardware pin for this aux port number has UART_TX
      // capability. By convention, we use the aux port pin with UART_TX
      // capability as the BiSS-C TX pin (for boards with RS422
      // transceivers, this will be connected to the TX on the
      // transceiver).
      const bool aux_port_has_uart_tx = [&]() {
        for (const auto& hwpin : hw_config.pins) {
          if (hwpin.number != static_cast<int>(i)) { continue; }
          for (const auto alt : {0, 0x100, 0x200, 0x300, 0x400}) {
            const auto mbed_pin = static_cast<PinName>(hwpin.mbed | alt);
            if (pinmap_find_peripheral(mbed_pin, PinMap_UART_TX) !=
                static_cast<uint32_t>(NC)) {
              return true;
            }
          }
        }
        return false;
      }();

      if (aux_port_has_uart_tx) {
        // This aux port pin should be used for TX. Find a hardware pin
        // with PWM (timer) capability.

        if (pin_tx.mbed != NC) {
          // TX already assigned, too many TX pins configured.
          error_ = aux::AuxError::kBisscPinError;
          return;
        }

        for (const auto& hwpin : hw_config.pins) {
          if (hwpin.number != static_cast<int>(i)) { continue; }
          if (hwpin.mbed == NC) { continue; }
          if (hwpin.timer != nullptr) {
            pin_tx = hwpin;
            break;
          }
        }

        if (pin_tx.mbed == NC) {
          // No timer-capable pin found for this aux port.
          error_ = aux::AuxError::kBisscPinError;
          return;
        }
      } else {
        if (pin_rx.mbed != NC) {
          // RX already assigned, too many RX pins configured.
          error_ = aux::AuxError::kBisscPinError;
          return;
        }

        // This aux port pin should be used for RX. No constraints on
        // the hardware pin.
        for (const auto& hwpin : hw_config.pins) {
          if (hwpin.number != static_cast<int>(i)) { continue; }
          if (hwpin.mbed == NC) { continue; }
          pin_rx = hwpin;
          pin_rx_pull = pin.pull;
          break;
        }

        if (pin_rx.mbed == NC) {
          // No hardware pin found for this aux port.
          error_ = aux::AuxError::kBisscPinError;
          return;
        }
      }
    }

    if (pin_tx.mbed == NC ||
        pin_rx.mbed == NC) {
      error_ = aux::AuxError::kBisscPinError;
      return;
    }

    // Validate configuration against limits For now, we both validate
    // the maximum CRC bits that we could ever be capable of, and
    // simultaneously restrict ourselves to the 6 bits that this
    // implementation can do efficiently.
    if (config_.data_bits == 0 || config_.data_bits > kMaxDataBits ||
        config_.crc_bits > kMaxCrcBits ||
        config_.crc_bits != 6) {
      error_ = aux::AuxError::kBisscPinError;
      return;
    }

    // Determine the GPIO port and pin bit for RX
    rx_gpio_port_ = [&]() {
      switch(STM_PORT(pin_rx.mbed)) {
        case PortA: return GPIOA;
        case PortB: return GPIOB;
        case PortC: return GPIOC;
        case PortD: return GPIOD;
        case PortE: return GPIOE;
        case PortF: return GPIOF;
      }
      mbed_die();
    }();

    // Determine which byte of GPIO IDR contains our pin and the bit
    // position within that byte. This allows us to use 8-bit DMA
    // transfers.
    const uint8_t pin_num = STM_PIN(pin_rx.mbed);
    if (pin_num < 8) {
      // Pin is in lower byte of GPIO IDR
      rx_pin_bit_pos_ = pin_num;
      rx_pin_in_high_byte_ = false;
    } else {
      // Pin is in upper byte of GPIO IDR
      rx_pin_bit_pos_ = pin_num - 8;
      rx_pin_in_high_byte_ = true;
    }

    // Now we will configure our timer.  The timer will be configured
    // to:
    // * emit a PWM signal at the desired bitrate
    // * request a DMA transfer on each falling edge
    // * we will leave it disabled, and enable it when we want to
    //   start a transaction
    hwtimer_ = pin_tx.timer;

    // Enable timer clock
    if (hwtimer_ == TIM2) { __HAL_RCC_TIM2_CLK_ENABLE(); }
    else if (hwtimer_ == TIM3) { __HAL_RCC_TIM3_CLK_ENABLE(); }
    else if (hwtimer_ == TIM4) { __HAL_RCC_TIM4_CLK_ENABLE(); }
    else {
      mbed_die();
    }

    // Configure timer for PWM at desired frequency.  Timer clock is
    // typically 170 MHz (PCLK1 * 2).  For now, we will just have
    // undefined behavior if a bitrate is selected that is too high or
    // too low.
    const uint32_t timer_clock = HAL_RCC_GetPCLK1Freq() * 2;
    const uint32_t period = (timer_clock / config_.rate_hz) - 1;

    hwtimer_->CR1 = 0;  // Disable timer
    hwtimer_->PSC = 0;  // No prescaler for maximum resolution
    hwtimer_->ARR = period;

    // Determine which timer channel we're using
    const auto pin_tx_alt = FindTimerAlt(pin_tx.mbed, pin_tx.timer);
    const auto function = pinmap_function(pin_tx_alt, PinMap_PWM);
    timer_channel_ = STM_PIN_CHANNEL(function);

    // Configure PWM mode for 50% duty cycle
    const uint32_t ccr_value = period / 2;
    const auto output_compare_mode = 6; // PWM mode 1

    // Configure CCMR for output compare (PWM mode)
    const auto ccmr_shift = ((timer_channel_ % 2) == 0) ? 8 : 0;
    const auto ccmr =
        (output_compare_mode << TIM_CCMR1_OC1M_Pos) << ccmr_shift;

    if (timer_channel_ == 1 || timer_channel_ == 2) {
      hwtimer_->CCMR1 = ccmr;
    } else if (timer_channel_ == 3 || timer_channel_ == 4) {
      hwtimer_->CCMR2 = ccmr;
    }

    // Set compare value
    volatile uint32_t* ccr = GetCCR(timer_channel_);
    *ccr = ccr_value;

    // Configure to trigger DMA on CC event (falling edge of PWM).
    //
    // We'll use the update event DMA request for simplicity.
    //
    // Enable output and configure for falling edge capture (for DMA
    // trigger).
    const auto ccer_bits = (1 << ((timer_channel_ - 1) * 4));  // CCxE
    hwtimer_->CCER = ccer_bits;

    // Enable DMA clocks
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // Configure DMAMUX for timer CC DMA request
    dmamux_ = Stm32Dma::SelectDmamux(dma_channel_);

    dmamux_->CCR =
        GetTimerCCDmaRequest(hwtimer_, timer_channel_) &
        DMAMUX_CxCR_DMAREQ_ID;

    // Configure DMA for memory increment, peripheral-to-memory.
    dma_channel_->CCR =
        DMA_PERIPH_TO_MEMORY |
        DMA_PINC_DISABLE |
        DMA_MINC_ENABLE |
        DMA_PDATAALIGN_BYTE |   // 8-bit peripheral access
        DMA_MDATAALIGN_BYTE |   // 8-bit memory access
        DMA_PRIORITY_HIGH;

    // Source is the appropriate byte of GPIO IDR register
    // For pins 0-7: read IDR[7:0] (offset 0)
    // For pins 8-15: read IDR[15:8] (offset 1 on little-endian)
    dma_channel_->CPAR = reinterpret_cast<uint32_t>(&rx_gpio_port_->IDR) +
                         (rx_pin_in_high_byte_ ? 1 : 0);

    // Configure our pins
    pinmap_pinout(pin_tx_alt, PinMap_PWM);

    // The receive pin will be configured as a digital input
    rx_.emplace(pin_rx.mbed, MbedMapPull(pin_rx_pull));

    status_->active = true;
  }

  aux::AuxError error() const { return error_; }

  void ISR_Update() MOTEUS_CCM_ATTRIBUTE {
    if (error_ != aux::AuxError::kNone || !config_.enabled) { return; }

    if (dma_channel_->CNDTR != 0) {
      // We are actively reading.
      return;
    }

    if (hwtimer_->CR1 != 0) {
      ISR_ProcessRead();
    } else {
      // We never start a read in the same cycle as we finished one.
      // That ensures that we maintain the ~20us BiSS-C minimum idle
      // time.
      const uint32_t now_us = timer_->read_us();
      const int16_t delta_us =
          static_cast<int16_t>(now_us - last_query_start_us_);

      if (delta_us >= config_.poll_rate_us) {
        ISR_StartRead();
      }
    }
  }

 private:
  void ISR_ProcessRead() MOTEUS_CCM_ATTRIBUTE {
    // Stop timer
    hwtimer_->CR1 = 0;

    // Disable DMA request
    const uint32_t dier_bit = TIM_DIER_CC1DE << (timer_channel_ - 1);
    hwtimer_->DIER &= ~dier_bit;

    // Disable DMA
    dma_channel_->CCR &= ~DMA_CCR_EN;

    // Configure extraction parameters
    BisscExtractConfig extract_config;
    extract_config.pin_bit_pos = rx_pin_bit_pos_;
    extract_config.data_bits = config_.data_bits;
    extract_config.crc_bits = config_.crc_bits;

    // The debug fields are only used for unit tests.
    extract_config.store_debug = false;

    const auto result = ExtractBisscFrame(
        buffer_.data(), buffer_.size(), extract_config);

    if (result.error != BisscExtractError::kNone) {
      // Any error (no START bit, buffer too small, CRC mismatch)
      // counts as CRC error.
      status_->crc_errors++;
    } else {
      status_->value = result.data_value;
      status_->error_flag = result.error_flag;
      status_->warning_flag = result.warning_flag;
      status_->nonce++;
    }
  }

  // Start a BiSS-C read transaction
  void ISR_StartRead() MOTEUS_CCM_ATTRIBUTE {
    if (error_ != aux::AuxError::kNone || !config_.enabled) { return; }

    // Calculate total bits that we need to transfer with DMA.
    //
    // BiSS-C frame: variable ACK + START + data + Error + Warning + CRC
    const uint32_t total_bits =
        kMaxAckBits + kStartBits + kCdsBits +
        config_.data_bits + kStatusBits + config_.crc_bits;

    if (total_bits > buffer_.size()) {
      error_ = aux::AuxError::kBisscPinError;
      return;
    }

    // Configure DMA to capture 'total_bits' samples
    dma_channel_->CNDTR = total_bits;
    dma_channel_->CMAR = reinterpret_cast<uint32_t>(buffer_.data());

    // Enable DMA
    dma_channel_->CCR |= DMA_CCR_EN;

    // Enable timer DMA request for the appropriate channel
    const uint32_t dier_bit = TIM_DIER_CC1DE << (timer_channel_ - 1);
    hwtimer_->DIER |= dier_bit;

    // Start timer
    hwtimer_->CNT = 0;
    hwtimer_->CR1 = TIM_CR1_CEN;

    last_query_start_us_ = timer_->read_us();
  }

  // BiSS-C protocol limits
  static constexpr uint32_t kMaxAckBits = 20;
  static constexpr uint32_t kMaxDataBits = 64;
  static constexpr uint32_t kMaxCrcBits = 6;
  static constexpr uint32_t kStartBits = 1;
  static constexpr uint32_t kCdsBits = 1;
  static constexpr uint32_t kStatusBits = 2;  // Error + Warning
  static constexpr uint32_t kBufferSize =
      kMaxAckBits + kStartBits + kCdsBits + kMaxDataBits +
      kStatusBits + kMaxCrcBits;

  static uint32_t GetTimerCCDmaRequest(TIM_TypeDef* timer, uint32_t channel) {
    uint32_t base_request = 0;
    if (timer == TIM2) {
      base_request = DMA_REQUEST_TIM2_CH1;
    } else if (timer == TIM3) {
      base_request = DMA_REQUEST_TIM3_CH1;
    } else if (timer == TIM4) {
      base_request = DMA_REQUEST_TIM4_CH1;
    } else {
      mbed_die();
    }
    return base_request + (channel - 1);
  }

  volatile uint32_t* GetCCR(uint32_t channel) {
    switch (channel) {
      case 1: { return &hwtimer_->CCR1; }
      case 2: { return &hwtimer_->CCR2; }
      case 3: { return &hwtimer_->CCR3; }
      case 4: { return &hwtimer_->CCR4; }
      default: { mbed_die(); }
    }
    return &hwtimer_->CCR1;
  }

  const aux::BissC::Config config_;
  aux::BissC::Status* const status_;
  aux::AuxError error_ = aux::AuxError::kNone;

  TIM_TypeDef* hwtimer_ = nullptr;
  uint32_t timer_channel_ = 0;

  DMA_Channel_TypeDef* dma_channel_ = nullptr;
  DMAMUX_Channel_TypeDef* dmamux_ = nullptr;
  MillisecondTimer* const timer_;

  GPIO_TypeDef* rx_gpio_port_ = nullptr;

  // The bit position within the sampled byte (0-7)
  uint8_t rx_pin_bit_pos_ = 0;

  // True if pin is in GPIO bits 8-15
  bool rx_pin_in_high_byte_ = false;

  std::optional<DigitalIn> rx_;

  // Aligned for 32-bit access.  This would likely already be properly
  // aligned, but it doesn't hurt to be extra careful.
  alignas(4) std::array<uint8_t, kBufferSize> buffer_ = {};

  uint32_t last_query_start_us_ = 0;
};

}
