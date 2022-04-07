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
 * PA5  - DRV8353 CLK
 * PA6  - DRV8353 MISO  (fast ADC2)
 * PA7  - DRV8353 MOSI  (fast ADC2)
 * PA8  - TSENSE2       (fast ADC5)
 * PA9  - VSENSE        (fast ADC5)
 * PA10
 * PA11 - CAN_RX
 * PA12 - CAN_TX
 * PA13 - SWDIO
 * PA14 - SWCLK
 * PA15
 * PB0  - DRV8353 CS
 * PB1  - CURRENT       (fast ADC3)
 * PB2  - PRIMARY SPI CS (ADC2)
 * PB3
 * PB4  - ONBOARD SPI CS
 * PB5  - PRIMARY SPI MOSI
 * PB6  - DBG_UART_TX
 * PB7  - DBG_UART_RX
 * PB8  - AUX SCL/BiSS
 * PB9  - AUX SDA/BiSS
 * PB10
 * PB11 -  PRIMARY S/C (ADC1/2)
 * PB12 - TSENSE1      (fast ADC4)
 * PB13 -              (fast ADC3)
 * PB14 - AUX S/C      (fast ADC1/4)
 * PB15 - AUX S/C      (fast ADC4)
 * PC4  - CURRENT     (fast ADC2)
 * PC6
 * PC10 - PRIMARY SPI CLK
 * PC11 - PRIMARY SPI MISO
 * PC13
 * PC14
 * PC15
 * PF0 - LED1
 * PF1 - LED2

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

 * 3V and 5V power
 * 3x hall inputs
 * SPI
 * BiSS-C interface?
 * PB3/4 have both SPI and USART2 and TIM3_CH1/2
 * PC10/11 have SPI and UART4
 * index pulse
 * notional pins (8  pins)
  * gnd
  * 3.3v
  * 5V
  * SPI MISO - BiSS Rx    - HALL/IND - UART - (PC11/SPI3_MISO/UART4_TX)
  * SPI MOSI - QUADRATURE - HALL/IND -      - (PB5/SPI3_MOSI/TIM5_CH2)
  * SPI CLK  - BiSS Tx    - HALL/IND - UART - (PC10/SPI3_CK/UART4_RX)
  * SPI CS   - QUADRATURE - HALL/IND - S/C  - (PB2/TIM5_CH1/ADC2_IN12)
  *                       - HALL/IND - S/C  - (PB11/ADC12, )

## Aux 2 ##

 * 3V and 5V power
 * I2C
 * quadrature
 * index
 * BSSI-C?
 * PA14/PA15 have I2C1 and USART2 and TIM8_CH1/2
 * PB8/PB9 have I2C1 and USART3 and TIM4_CH3/4
 * TIM1/2 options: PA0/1/2/3/5/6/7/12/15
                   PB2/3/4/5/6/7/8/9/14/15
 * notional pins (7 pins)
   * gnd
   * 3V
   * 5V
   * I2C_SCL  - BiSS RX  - QUAD - HALL/IND - (PB8/USART2_RX/TIM8_CH1)
   * I2C_SDA  - BiSS TX  - QUAD - HALL/IND - (PB9/UART2_TX/TIM8_CH2) (PB13/SPI2_SCK)
   * SIN/COS  - STEPDIR  - HALL/IND - (PB14/ADC4_1/SPI2)
   * SIN/COS  - STEPDIR  - HALL/IND - (PB15/ADC4_2/SPI2)

# Brainstorm for future features #

 * STO
 * Optional BSSI-C input module
 * Would be nice if gate driver didn't induce audible noise from aliasing
