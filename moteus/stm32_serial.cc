// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

#include "moteus/stm32_serial.h"

#include "mjlib/base/assert.h"

namespace moteus {

namespace {
int32_t GetMax16OversamplingBaud(UARTName uart) {
  switch (uart) {
    case UART_1:
#if defined(TARGET_STM32F4)
    case UART_6:
#endif
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

void EnableClock(UARTName uart) {
  if (uart == UART_1) {
    __HAL_RCC_USART1_CLK_ENABLE();
  } else if (uart == UART_2) {
    __HAL_RCC_USART2_CLK_ENABLE();
  } else if (uart == UART_3) {
    __HAL_RCC_USART3_CLK_ENABLE();
  } else if (uart == UART_4) {
    __HAL_RCC_UART4_CLK_ENABLE();
  } else if (uart == UART_5) {
    __HAL_RCC_UART5_CLK_ENABLE();
#if defined(STM32F4)
  } else if (uart == UART_6) {
    __HAL_RCC_USART6_CLK_ENABLE();
#endif
  } else {
    MJ_ASSERT(false);
  }
#if defined(UART7_BASE) || defined(USART7_BASE)
  #error "Need to handle more uarts"
#endif
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

  // Reset and enable clock.
  EnableClock(uart_name());

  // Configure pins for alternate functions.
  pinmap_pinout(options.tx, PinMap_UART_TX);
  pinmap_pinout(options.rx, PinMap_UART_RX);
  if (options.tx != NC) {
    pin_mode(options.tx, PullUp);
  }
  if (options.rx != NC) {
    pin_mode(options.rx, PullUp);
  }

  // Then configure the UART itself.
  UART_HandleTypeDef huart{};

  huart.Instance = uart_;
  huart.Init.BaudRate = options.baud_rate;
  huart.Init.WordLength = 8;
  huart.Init.StopBits = 1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;

  const int32_t max_16oversampling_baud =
      GetMax16OversamplingBaud(uart_name());
  huart.Init.OverSampling =
      (options.baud_rate > max_16oversampling_baud ?
       UART_OVERSAMPLING_8 :
       UART_OVERSAMPLING_16);

  if (options.tx == NC) {
    huart.Init.Mode = UART_MODE_RX;
  } else if (options.rx == NC) {
    huart.Init.Mode = UART_MODE_TX;
  } else {
    huart.Init.Mode = UART_MODE_TX_RX;
  }

  HAL_UART_Init(&huart);
}

}
