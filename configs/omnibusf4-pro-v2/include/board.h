/************************************************************************************
 * configs/omnibusf4-pro-v2/include/board.h
 *
 *   Copyright (C) 2012, 2014-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __CONFIG_OMNIBUSF4_PRO_V2_INCLUDE_BOARD_H
#define __CONFIG_OMNIBUSF4_PRO_V2_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* Omnibus board features a single 8MHz crystal.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 168000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                          : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)             : 5
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (8,000,000 / 8) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM3_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM4_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM5_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM6_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM7_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on board the
 * stm32f4blackbox.  The following definitions describe how NuttX controls the LEDs:
 */
/*
#define LED_STARTED       0  /* LED1 * /
#define LED_HEAPALLOCATE  1  /* LED2 * /
#define LED_IRQSENABLED   2  /* LED1 + LED2 * /
#define LED_STACKCREATED  3  /* LED3 * /
#define LED_INIRQ         4  /* LED1 + LED3 * /
#define LED_SIGNAL        5  /* LED2 + LED3 * /
#define LED_ASSERTION     6  /* LED1 + LED2 + LED3 * /
#define LED_PANIC         7  /* N/C  + N/C  + N/C + LED4 * /
*/

/* Alternate function pin selections ************************************************/
/* UART1:
 */

#define GPIO_USART1_RX GPIO_USART1_RX_1     // PA10
#define GPIO_USART1_TX GPIO_USART1_TX_1     // PA9


/* UART2: */

//#define GPIO_USART2_RX GPIO_USART2_RX_1
//#define GPIO_USART2_TX GPIO_USART2_TX_1

/* UART3: is disabled for I2C2 sake on same pins */

//#define GPIO_USART3_TX GPIO_USART3_TX_1     // PB10
//#define GPIO_USART3_RX GPIO_USART3_RX_1     // PB11

/* UART4: is not used */

//#define GPIO_USART4_TX GPIO_USART4_TX_1
//#define GPIO_USART4_RX GPIO_USART4_RX_1

/* UART5: is not used */

//#define GPIO_USART5_TX GPIO_USART5_TX_1
//#define GPIO_USART5_RX GPIO_USART5_RX_1

/* UART6: is available with J10 pin group*/

#define GPIO_USART6_RX GPIO_USART6_RX_1     // PC7
#define GPIO_USART6_TX GPIO_USART6_TX_1     // PC6

/* PWM
 *
 * The STM32F4 Discovery has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using TIM4 CH2 on PD13.
 */

//#define GPIO_TIM4_CH2OUT GPIO_TIM4_CH2OUT_2

/* SPI1 is used for MPU6000 with cs on PA4*/

#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1   // PA6
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1   // PA7
#define GPIO_SPI1_SCK    GPIO_SPI1_SCK_1    // PA5

/* SPI2 is used for sd card */

#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1   // PB14
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1   // PB15
#define GPIO_SPI2_SCK    GPIO_SPI2_SCK_2    // PB13

/* SPI3 */

#define GPIO_SPI3_MISO   GPIO_SPI3_MISO_2   // PC11
#define GPIO_SPI3_MOSI   GPIO_SPI3_MOSI_2   // PC12
#define GPIO_SPI3_SCK    GPIO_SPI3_SCK_2    // PC10


/* I2C1 is not used */

// #define GPIO_I2C1_SCL    GPIO_I2C1_SCL_1
// #define GPIO_I2C1_SDA    GPIO_I2C1_SDA_2

/* I2C2 is avalible on pingroup J10 */
#define GPIO_I2C2_SCL   GPIO_I2C2_SCL_1 // PB10
#define GPIO_I2C2_SDA   GPIO_I2C2_SDA_1 // PB11

/* Timer Inputs/Outputs is not used yet */

//#define GPIO_TIM2_CH1IN  GPIO_TIM2_CH1IN_2
//#define GPIO_TIM2_CH2IN  GPIO_TIM2_CH2IN_1

//#define GPIO_TIM8_CH1IN  GPIO_TIM8_CH1IN_1
//#define GPIO_TIM8_CH2IN  GPIO_TIM8_CH2IN_1

#endif  /* __CONFIG_OMNIBUSF4_PRO_V2_INCLUDE_BOARD_H */

