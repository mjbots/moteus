moteus-n1 is intended as a higher power/voltage, more flexible, yet
smaller in footprint version of the moteus brushless controller.

# Features #

 * 51V max voltage (vs 44V for moteus)
 * More flexible aux1/aux2 connectors
   * both JST GH (aux1 is 8 pin, aux2 is 7 pin)
   * 200mA of 5V between both connectors (all I/O pins are still only 3.3V)
   * 200mA of 3V between both connectors
   * aux1 now has two pins that can be used even if onboard encoder is used
   * SPI on both connectors
   * UART on both connectors
   * ADC on both connectors
   * Sine/cosine on both connectors
   * Hardware quadrature (higher count rates)
 * No connectors required on bottom of board for smaller footprint
 * Optional solder pads for power in for higher current capability
 * Variants
   * all ceramic for low profile
   * polymer electrolytic for higher peak power
   * optional power/CAN connectors on bottom for daisy chaining
 * CAN transceiver with 58V bus fault voltage
 * 46x46mm


# Pin Allocation #

 * PA0  - MOTOR 1       (fast ADC1/2)
 * PA1  - MOTOR 2       (fast ADC1/2)
 * PA2  - MOTOR 3       (fast ADC1)
 * PA3  - CURRENT       (fast ADC1)
 * PA4  - DEBUG_DAC
 * PA5  - AUX1_A
 * PA6  - CURRENT       (fast ADC2)
 * PA7  - AUX1_C
 * PA8  - TSENSE2       (fast ADC5)
 * PA9  - VSENSE        (fast ADC5)
 * PA10 - AUX2_A
 * PA11 - AUX2_B
 * PA12 - AUX2_I2C_PULLUP
 * PA13 - SWDIO
 * PA14 - SWCLK
 * PA15 - AUX1_D
 * PB0  - DRV8353 CS
 * PB1  - CURRENT        (fast ADC3)
 * PB2  - AS5047P CS     (ADC2)
 * PB3  - AUX1_E
 * PB4  - AUX1_B
 * PB5  - FDCAN2_RX
 * PB6  - FDCAN2_TX
 * PB7  - AUX2_C
 * PB8  - AUX1_I2C_PULLUP
 * PB9  - AUX1_E
 * PB10 - RS422_RE
 * PB11 - RS422_DE      (ADC1/2)
 * PB12 - TSENSE1      (fast ADC4)
 * PB13 - MOTOR_FAULT  (fast ADC3)
 * PB14 - AUX1_A       (fast ADC1/4)
 * PB15 - LED1         (fast ADC4)
 * PC4  - AUX2_B       (fast ADC2)
 * PC6  - LED2
 * PC10 - DRV8323 SCK
 * PC11 - DRV8323 MISO
 * PC13 - DRV8323 MOSI
 * PC14 - MOTOR_ENABLE
 * PC15 - MOTOR_HIZ
 * PF0 - AUX2_A
 * PF1 - AUX2_D

# Pins Required #

     * LEDs - 2
     * as5047 CS - 1
     * drv8353 CS - 1
     * 2x temp sense - 2
     * debug uart - 2
     * swd - 2
     * DAC debug?
     * voltage sense - 1
     * aux ADC - 2
     * primary aux - 1
     * CAN - 2
     * current sense - 3
       * needs to be "fast" ADC channels
     * aux fixed conn - 2
     * primary fixed conn - 4 (SPI shared with as5047)
     * drv8353 SPI - 3
     * gate PWM - 3

# Connectors #

## Aux 1 ##

 * notional pins (8  pins)
  * 5V
  * 3.3v
  * A: SPI1_SCK  / ADC1 / TIM2_CH1                             - PA5 / PB14
  * B: SPI1_MISO /      / TIM3_CH1 /          / USART2_RX / 5V - PB4
  * C: SPI1_MOSI / ADC2 / TIM3_CH2                             - PA7
  * D:           /      / TIM2_CH1 / I2C1_SCL / USART2_RX / 5V - PA15
  * E:           /      / TIM2_CH2 / I2C1_SDA / USART2_TX / 5V - PB3 / PB9
  * gnd

## Aux 2 ##

 * 3V and 5V power
 * I2C
 * quadrature
 * index
 * uart
 * SPI
 * PA14/PA15 have I2C1 and USART2 and TIM8_CH1/2
 * PB8/PB9 have I2C1 and USART3 and TIM4_CH3/4
 * TIM1/2 options: PA0/1/2/3/5/6/7/12/15
                   PB2/3/4/5/6/7/8/9/14/15
 * notional pins (7 pins)
   * 5V
   * 3V
   * A: SPI2 SCK  /           /          / ADC2 /           - PF1
   * B: SPI2 MISO / USART1_RX / I2C2_SDA / ADC1             - PA10 / PF0
   * C: SPI2 MOSI / USART1_TX / I2C2_SCL / ADC2 / TIM4_CH1  - PA11 / PC4
   * D:             USART1_RX /                 / TIM4_CH2  - PB7
   * gnd

# Brainstorm for future features #

 * STO
 * Optional BSSI-C input module
 * Would be nice if gate driver didn't induce audible noise from aliasing
