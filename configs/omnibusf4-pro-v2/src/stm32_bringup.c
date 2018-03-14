/****************************************************************************
 * config/stm32f4discovery/src/stm32_bringup.c
 *
 *   Copyright (C) 2012, 2014-2017 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include <nuttx/binfmt/elf.h>

#include "stm32.h"
#include "stm32_romfs.h"

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

/* Checking needed by MMC/SDCard */

#ifdef CONFIG_NSH_MMCSDMINOR
#  define MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#else
#  define MMCSD_MINOR 0
#endif

#define BMP280_MINOR 0

#include "omnibus4prov2.h"


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
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize() starts a thread
   * will monitor for USB connection and disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      uerr("ERROR: Failed to start USB monitor: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_ELF
  /* Initialize the ELF binary loader */

  ret = elf_initialize();
  if (ret < 0)
    {
      serr("ERROR: Initialization of the ELF loader failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n",
           STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_MMCSD
  ret = stm32_mmcsd_initialize(MMCSD_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot %d: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32_ROMFS
  ret = stm32_romfs_initialize();
  if (ret < 0)
    {
      serr("ERROR: Failed to mount romfs at %s: %d\n",
           STM32_ROMFS_MOUNTPOINT, ret);
    }
#endif

#if defined(CONFIG_RNDIS) && defined(CONFIG_NSH_MACADDR)
  uint8_t mac[6];
  mac[0] = 0xaa; /* TODO */
  mac[1] = (CONFIG_NSH_MACADDR >> (8 * 4)) & 0xff;
  mac[2] = (CONFIG_NSH_MACADDR >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NSH_MACADDR >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NSH_MACADDR >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NSH_MACADDR >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

#ifdef CONFIG_SENSORS_BMP280
	ret = stm32_sensors_bmp280_initialize(BMP280_MINOR);
	if (ret < 0) {
		syslog(LOG_ERR, "Failed to initialize BMP280 %d: %d\n", BMP280_MINOR, ret);
	}
#endif

  return ret;
}
