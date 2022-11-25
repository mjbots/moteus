moteus hp is intended as a higher power/voltage, more flexible, yet
smaller in footprint version of the moteus brushless controller.

# Features #

 * 54V max voltage (vs 44V for moteus)
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
 * PA5  - PRIMARY SCK - AUX1.1
 * PA6  - CURRENT       (fast ADC2)
 * PA7  - PRIMARY MOSI - AUX1.2  (fast ADC2)
 * PA8  - TSENSE2       (fast ADC5)
 * PA9  - VSENSE        (fast ADC5)
 * PA10 - AUX2.1
 * PA11 - AUX2.2
 * PA12 -
 * PA13 - SWDIO
 * PA14 - SWCLK
 * PA15 - AUX1.3
 * PB0  - DRV8353 CS
 * PB1  - CURRENT        (fast ADC3)
 * PB2  - AS5047P CS     (ADC2)
 * PB3  - AUX1.4
 * PB4  - PRIMARY SCK
 * PB5  - FDCAN2_RX
 * PB6  - FDCAN2_TX
 * PB7  - AUX2.3
 * PB8  -
 * PB9  - AUX1.4
 * PB10
 * PB11 -              (ADC1/2)
 * PB12 - TSENSE1      (fast ADC4)
 * PB13 -              (fast ADC3)
 * PB14 - AUX1.1       (fast ADC1/4)
 * PB15 -              (fast ADC4)
 * PC4  - AUX2.2       (fast ADC2)
 * PC6
 * PC10 - DRV8323 SCK
 * PC11 - DRV8323 MISO
 * PC13 - DRV8323 MOSI
 * PC14
 * PC15
 * PF0 - AUX2.1
 * PF1 - AUX2.4

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
  * gnd
  * 3.3v
  * 5V
   * SPI1_SCK  / ADC1 / TIM2_CH1                             - PA5 / PB14
   * SPI1_MISO /      / TIM3_CH1 /          / USART2_RX / 5V - PB4
   * SPI1_MOSI / ADC2 / TIM3_CH2                             - PA7
   *           /      / TIM2_CH1 / I2C1_SCL / USART2_RX / 5V - PA15
   *           /      / TIM2_CH2 / I2C1_SDA / USART2_TX / 5V - PB3 / PB9

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
   * gnd
   * 3V
   * 5V
   * SPI2 SCK  /           /          / ADC2 /           - PF1
   * SPI2 MISO / USART1_RX / I2C2_SDA / ADC1             - PA10 / PF0
   * SPI2 MOSI / USART1_TX / I2C2_SCL / ADC2 / TIM4_CH1  - PA11 / PC4
   *             USART1_RX /                 / TIM4_CH2  - PB7

# Brainstorm for future features #

 * STO
 * Optional BSSI-C input module
 * Would be nice if gate driver didn't induce audible noise from aliasing
