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

#include "fw/stm32_serial.h"

#include "mjlib/base/assert.h"

namespace moteus {

namespace {
int32_t GetMax16OversamplingBaud(UARTName uart) {
  switch (uart) {
    case UART_1:
      return 5620000;
    case UART_2:
    case UART_3:
    case UART_4:
    case UART_5:
#if defined(TARGET_STM32G4)
    case LPUART_1:
#endif
      return 2810000;
  }
  MJ_ASSERT(false);
  return 0;
}

void EnableUart(USART_TypeDef* uart) {
#if defined (USART1_BASE)
  if (uart == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART2_BASE)
  if (uart == USART2) {
    __HAL_RCC_USART2_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART3_BASE)
  if (uart == USART3) {
    __HAL_RCC_USART3_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART4_BASE)
  if (uart == UART4) {
    __HAL_RCC_UART4_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART4_BASE)
  if (uart == USART4) {
    __HAL_RCC_USART4_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART5_BASE)
  if (uart == UART5) {
    __HAL_RCC_UART5_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART5_BASE)
  if (uart == USART5) {
    __HAL_RCC_USART5_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART6_BASE)
  if (uart == USART6) {
    __HAL_RCC_USART6_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART7_BASE)
  if (uart == UART7) {
    __HAL_RCC_UART7_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART7_BASE)
  if (uart == USART7) {
    __HAL_RCC_USART7_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART8_BASE)
  if (uart == UART8) {
    __HAL_RCC_UART8_CLK_ENABLE();
    return;
  }
#endif
#if defined (USART8_BASE)
  if (uart == USART8) {
    __HAL_RCC_USART8_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART9_BASE)
  if (uart == UART9) {
    __HAL_RCC_UART9_CLK_ENABLE();
    return;
  }
#endif
#if defined (UART10_BASE)
  if (uart == UART10) {
    __HAL_RCC_UART10_CLK_ENABLE();
    return;
  }
#endif
  mbed_die();
}

}

Stm32Serial::Stm32Serial(const Options& options) {
  uart_= [&]() {
    const auto uart_tx = static_cast<UARTName>(
        pinmap_peripheral(options.tx, PinMap_UART_TX));
    const auto uart_rx = static_cast<UARTName>(
        pinmap_peripheral(options.rx, PinMap_UART_RX));
    return reinterpret_cast<USART_TypeDef*>(pinmap_merge(uart_tx, uart_rx));
  }();
  MJ_ASSERT(uart_ != nullptr);

  // Configure pins for alternate functions.
  pinmap_pinout(options.tx, PinMap_UART_TX);
  pinmap_pinout(options.rx, PinMap_UART_RX);
  if (options.tx != NC) {
    pin_mode(options.tx, PullUp);
  }
  if (options.rx != NC) {
    pin_mode(options.rx, PullUp);
  }

  // Reset and enable clock.
  EnableUart(uart_);

  // Then configure the UART itself.
  huart_.Instance = uart_;
  huart_.Init.BaudRate = options.baud_rate;
  huart_.Init.WordLength = UART_WORDLENGTH_8B;
  huart_.Init.StopBits = UART_STOPBITS_1;
  huart_.Init.Parity = UART_PARITY_NONE;

  if (options.tx == NC) {
    huart_.Init.Mode = UART_MODE_RX;
  } else if (options.rx == NC) {
    huart_.Init.Mode = UART_MODE_TX;
  } else {
    huart_.Init.Mode = UART_MODE_TX_RX;
  }

  huart_.Init.HwFlowCtl = UART_HWCONTROL_NONE;

  const int32_t max_16oversampling_baud =
      GetMax16OversamplingBaud(uart_name());
  huart_.Init.OverSampling =
      (options.baud_rate > max_16oversampling_baud ?
       UART_OVERSAMPLING_8 :
       UART_OVERSAMPLING_16);

#if defined(TARGET_STM32G4)
  huart_.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart_.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart_.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
#endif

  if (HAL_UART_Init(&huart_) != HAL_OK) {
    mbed_die();
  }

#if defined(TARGET_STM32G4)
  if (HAL_UARTEx_SetTxFifoThreshold(
          &huart_, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
    mbed_die();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(
          &huart_, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
    mbed_die();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart_) != HAL_OK) {
    mbed_die();
  }
#endif
}

}
