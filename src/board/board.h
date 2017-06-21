/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _BOARD_H_
#define _BOARD_H_

/*
 * Setup for the Olimex STM32-P103 proto board.
 */

/*
 * Board identifier.
 */
#define BOARD_SIXAX_V1_0
#define BOARD_NAME              "naniBox SixAx v1.0"

/*
 * Board frequencies.
 */
#define STM32_LSECLK            32768
#define STM32_HSECLK            12000000

/*
 * MCU type, supported types are defined in ./os/hal/platforms/hal_lld.h.
 */
#define STM32F10X_MD

/*
 * IO pins assignments.
 */
#define GPIOA_BUTTON            0
#define GPIOA_SYNC_OUT          1
#define GPIOA_USART2_TX         2
#define GPIOA_USART2_RX         3
#define GPIOA_SPI1NSS           4
#define GPIOA_SPI1SCK           5
#define GPIOA_SPI1MISO          6
#define GPIOA_SPI1MOSI          7

#define GPIOA_LED               8
#define GPIOA_USART1_MCU_TX_GPS_RX     9
#define GPIOA_USART1_MCU_RX_GPS_TX     10
#define GPIOA_L1_50_CTRL        11
#define GPIOA_L1_33_CTRL        12
#define GPIOA_SWDIO             13
#define GPIOA_SWCLK             14
#define GPIOA_SYNC_IN           15

#define GPIOB_USART1_RE         0
#define GPIOB_USART1_DE         1
#define GPIOB_GPS_33_CTRL       2
#define GPIOB_TRACESWO          3
#define GPIOB_GPS_EXT_INT       4
#define GPIOB_GPS_TIMEPULSE     5
#define GPIOB_I2C1_SCL          6
#define GPIOB_I2C1_SDA          7

#define GPIOD_OSC_IN            0 
#define GPIOD_OSC_OUT           1

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 *
 * The digits have the following meaning:
 *   0 - Analog input.
 *   1 - Push Pull output 10MHz.
 *   2 - Push Pull output 2MHz.
 *   3 - Push Pull output 50MHz.
 *   4 - Digital input.
 *   5 - Open Drain output 10MHz.
 *   6 - Open Drain output 2MHz.
 *   7 - Open Drain output 50MHz.
 *   8 - Digital input with PullUp or PullDown resistor depending on ODR.
 *   9 - Alternate Push Pull output 10MHz.
 *   A - Alternate Push Pull output 2MHz.
 *   B - Alternate Push Pull output 50MHz.
 *   C - Reserved.
 *   D - Alternate Open Drain output 10MHz.
 *   E - Alternate Open Drain output 2MHz.
 *   F - Alternate Open Drain output 50MHz.
 * Please refer to the STM32 Reference Manual for details.
 */

/*
 * Port A setup.

 * GPIOA_BUTTON                     0   -  4 - Digital input.
 * GPIOA_SYNC_OUT                   1   -  1 - Push Pull output 2MHz.
 * GPIOA_USART2_TX                  2   -  B - Alternate Push Pull output 50MHz.
 * GPIOA_USART2_RX                  3   -  4 - Digital input.
 * GPIOA_SPI1NSS                    4   -  B - Alternate Push Pull output 50MHz.
 * GPIOA_SPI1SCK                    5   -  B - Alternate Push Pull output 50MHz.
 * GPIOA_SPI1MISO                   6   -  4 - Digital input.
 * GPIOA_SPI1MOSI                   7   -  B - Alternate Push Pull output 50MHz.

 * GPIOA_LED                        8   -  3 - Push Pull output 50MHz.
 * GPIOA_USART1_MCU_TX_GPS_RX       9   -  B - Alternate Push Pull output 50MHz.
 * GPIOA_USART1_MCU_RX_GPS_TX       10  -  ?4 - Digital input.
 * GPIOA_L1_50_CTRL                 11  -  3 - Push Pull output 50MHz.
 * GPIOA_L1_33_CTRL                 12  -  3 - Push Pull output 50MHz.
 * GPIOA_SWDIO                      13  -  N/A
 * GPIOA_SWCLK                      14  -  N/A
 * GPIOA_SYNC_IN                    15  -  ?4 - Digital input. 

 */
#define VAL_GPIOACRL            0xB4BB4B14      /*  PA7...PA0 */
#define VAL_GPIOACRH            0x488334B3      /* PA15...PA8 */
#define VAL_GPIOAODR            0xFFFFFFFF

/*
 * Port B setup.

 * GPIOB_USART1_RE                  0   -  3 - Push Pull output 50MHz.
 * GPIOB_USART1_DE                  1   -  3 - Push Pull output 50MHz.
 * GPIOB_GPS_33_CTRL                2   -  3 - Push Pull output 50MHz.
 * GPIOB_TRACESWO                   3   -  N/A
 * GPIOB_GPS_EXT_INT                4   -  3 - Push Pull output 50MHz.
 * GPIOB_GPS_TIMEPULSE              5   -  4 - Digital input.
 * GPIOB_I2C1_SCL                   6   -  F - Alternate Open Drain output 50MHz.
 * GPIOB_I2C1_SDA                   7   -  F - Alternate Open Drain output 50MHz.

 */
#define VAL_GPIOBCRL            0xFF438333      /*  PB7...PB0 */
#define VAL_GPIOBCRH            0x88888888      /* PB15...PB8 */
#define VAL_GPIOBODR            0xFFFFFFFF

/*
 * Port C setup.
 */
#define VAL_GPIOCCRL            0x88888888      /*  PC7...PC0 */
#define VAL_GPIOCCRH            0x88888888      /* PC15...PC8 */
#define VAL_GPIOCODR            0xFFFFFFFF

/*
 * Port D setup.

 * PD0  - Normal input (XTAL).
 * PD1  - Normal input (XTAL).

 */
#define VAL_GPIODCRL            0x88888844      /*  PD7...PD0 */
#define VAL_GPIODCRH            0x88888888      /* PD15...PD8 */
#define VAL_GPIODODR            0xFFFFFFFF

/*
 * Port E setup.
 */
#define VAL_GPIOECRL            0x88888888      /*  PE7...PE0 */
#define VAL_GPIOECRH            0x88888888      /* PE15...PE8 */
#define VAL_GPIOEODR            0xFFFFFFFF

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _BOARD_H_ */
