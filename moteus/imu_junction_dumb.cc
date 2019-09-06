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

#include "mbed.h"

#include "moteus/imu_junction_hw.h"
#include "moteus/millisecond_timer.h"
#include "moteus/stm32_serial.h"

using namespace moteus;

int main(void) {
  DigitalOut led1(DEBUG_LED1, 1);
  DigitalOut led2(DEBUG_LED2, 2);

  Stm32Serial master_serial{[]() {
      Stm32Serial::Options options;
      options.tx = PC_10;
      options.rx = PC_11;
      // options.dir = PC_12;
      options.baud_rate = 3000000;
      return options;
    }()};

  auto* const master_uart = master_serial.uart();

  master_uart->CR1 &= ~USART_CR1_UE;
  master_uart->CR1 |= USART_CR1_TE | USART_CR1_RE;
  master_uart->CR1 |= USART_CR1_UE;

  DigitalOut master_dir(PC_12, 0);

  Stm32Serial slave1{[]() {
      Stm32Serial::Options options;
      options.tx = PA_2;
      options.rx = PA_3;
      // options.dir = PA_1;
      options.baud_rate = 3000000;
      return options;
    }()};

  auto* slave1_uart = slave1.uart();

  slave1_uart->CR1 &= ~USART_CR1_UE;
  slave1_uart->CR1 |= USART_CR1_TE | USART_CR1_RE;
  slave1_uart->CR1 |= USART_CR1_UE;

  DigitalOut slave1_dir(PA_1, 0);

  Stm32Serial slave2{[]() {
      Stm32Serial::Options options;
      options.tx = PC_6;
      options.rx = PC_7;
      // options.dir = PC_8;
      options.baud_rate = 3000000;
      return options;
    }()};

  auto* slave2_uart = slave2.uart();

  slave2_uart->CR1 &= ~USART_CR1_UE;
  slave2_uart->CR1 |= USART_CR1_TE | USART_CR1_RE;
  slave2_uart->CR1 |= USART_CR1_UE;

  DigitalOut slave2_dir(PC_8, 0);

  MillisecondTimer timer;
  uint32_t flash_count = 0;

  // We don't want *any* interrupts while we are running.  Our inner
  // loop runs at about 1us per iteration, and any glitches can
  // cause a receiver overrun.
  __disable_irq();

  int32_t master_sleep_count = 0;
  int32_t slave1_sleep_count = 0;
  int32_t slave2_sleep_count = 0;

  auto sleep_1us = [&]() {
    const auto start = timer.read_us();
    while (timer.read_us() == start);
  };

  // Poll in a tight loop.
  while (true) {
    if (master_uart->SR & USART_SR_RXNE) {
      const uint8_t value = master_uart->DR;
      if (slave1_dir.read() == 0 ||
          slave2_dir.read() == 0) {
        slave1_dir.write(1);
        slave2_dir.write(1);
        sleep_1us();
      }
      while ((slave1_uart->SR & USART_SR_TXE) == 0);
      while ((slave2_uart->SR & USART_SR_TXE) == 0);
      slave1_uart->DR = value;
      slave2_uart->DR = value;

      slave1_sleep_count = 0;
      slave2_sleep_count = 0;
    }

    if (slave1_uart->SR & USART_SR_RXNE) {
      const int value = slave1_uart->DR;
      if (master_dir.read() == 0) {
        master_dir.write(1);
        sleep_1us();
      }
      while ((master_uart->SR & USART_SR_TXE) == 0);
      master_uart->DR = value;
      master_sleep_count = 0;
    }

    if (slave2_uart->SR & USART_SR_RXNE) {
      const int value = slave2_uart->DR;
      if (master_dir.read() == 0) {
        master_dir.write(1);
        sleep_1us();
      }
      while ((master_uart->SR & USART_SR_TXE) == 0);
      master_uart->DR = value;
      master_sleep_count = 0;
    }

    if ((master_uart->SR & USART_SR_TC) != 0) {
      master_sleep_count++;
      if (master_sleep_count > 10) {
        master_dir.write(0);
        master_uart->SR &= ~USART_SR_TC;
      }
    }

    if ((slave1_uart->SR & USART_SR_TC) != 0) {
      slave1_sleep_count++;
      if (slave1_sleep_count > 10) {
        slave1_dir.write(0);
        slave1_uart->SR &= ~USART_SR_TC;
        led2.write(!led2.read());
      }
    }

    if ((slave2_uart->SR & USART_SR_TC) != 0) {
      slave2_sleep_count++;
      if (slave2_sleep_count > 10) {
        slave2_dir.write(0);
        slave2_uart->SR &= ~USART_SR_TC;
      }
    }

    {
      const uint32_t time_s = timer.read_us() >> 20;
      if (time_s != flash_count) {
        flash_count = time_s;
        led1.write(!led1.read());
      }
    }
  }
}

extern "C" {
  void abort() {
    mbed_die();
  }
}
