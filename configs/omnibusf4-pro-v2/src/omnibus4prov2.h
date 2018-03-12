/****************************************************************************
 * configs/stm32f4discovery/src/stm32f4discovery.h
 *
 *   Copyright (C) 2011-2012, 2015-2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __CONFIGS_OMNIBUSF4PROV2_SRC_OMNIBUSF4PROV2_H
#define __CONFIGS_OMNIBUSF4PROV2_SRC_OMNIBUSF4PROV2_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration *************************************************************/

/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif


/* Assume that we have everything */

#define HAVE_USBDEV     1
#define HAVE_USBHOST    1
#define HAVE_USBMONITOR 1
#define HAVE_ELF        1

/* Can't support USB host or device features if USB OTG FS is not enabled */

#ifndef CONFIG_STM32_OTGFS
#  undef HAVE_USBDEV
#  undef HAVE_USBHOST
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB device monitor if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#  undef HAVE_USBMONITOR
#endif

/* Can't support USB host is USB host is not enabled */

#ifndef CONFIG_USBHOST
#  undef HAVE_USBHOST
#endif

/* Check if we should enable the USB monitor before starting NSH */

#if !defined(CONFIG_USBDEV_TRACE) || !defined(CONFIG_USBMONITOR)
#  undef HAVE_USBMONITOR
#endif


/* ELF */

#if defined(CONFIG_BINFMT_DISABLE) || !defined(CONFIG_ELF)
#  undef HAVE_ELF
#endif

/* NSH Network monitor  */

#if !defined(CONFIG_NET) || !defined(CONFIG_STM32_EMACMAC)
#  undef HAVE_NETMONITOR
#endif

#if !defined(CONFIG_NSH_NETINIT_THREAD) || !defined(CONFIG_ARCH_PHY_INTERRUPT) || \
    !defined(CONFIG_NETDEV_PHY_IOCTL) || !defined(CONFIG_NET_UDP) || \
     defined(CONFIG_DISABLE_SIGNALS)
#  undef HAVE_NETMONITOR
#endif

/* The NSH Network Monitor cannot be used with the STM32F4DIS-BB base board.
 * That is because the LAN8720 is configured in REF_CLK OUT mode.  In that
 * mode, the PHY interrupt is not supported.  The NINT pin serves instead as
 * REFLCK0.
 */

/* procfs File System */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* omnibusf4prov2 GPIOs **************************************************/
/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN5)

/* SPI chip selects */

// TODO
//#define GPIO_NRF24L01_CS

// sd card cs pin
#define GPIO_SDCARD_CS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN12)

// bmp280 cs pin
#define GPIO_BMP280_CS	  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN3)

// sd card detect pin
#define GPIO_SDCARD_DETECT  (GPIO_INPUT|GPIO_SPEED_50MHz|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN7)


/* omnibus-pro-v2 MicroSD
 *
 * ---------- ------------- ------------------------------
 * PIO        SIGNAL        Comments
 * ---------- ------------- ------------------------------
 * PB7        NCD           Pulled up externally(?)
 * PB12       CS            Configured by SPI driver
 * PB13       SCK           Configured by SPI driver
 * PB14       MISO          Configured by SPI driver
 * PB15       MOSI          Configured by SPI driver
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f4discovery
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

 /****************************************************************************
  * Name: stm32_i2sdev_initialize
  *
  * Description:
  *   Called to configure I2S chip select GPIO pins for the stm32f4discovery
  *   board.
  *
  ****************************************************************************/

FAR struct i2s_dev_s *stm32_i2sdev_initialize(int port);

// MMCD device initialization
int stm32_mmcsd_initialize(int minor);


// BMP280 device intialization
int stm32_bmp280_initialize(int minor);


#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_OMNIBUSF4PROV2_SRC_OMNIBUSF4PROV2_H */
