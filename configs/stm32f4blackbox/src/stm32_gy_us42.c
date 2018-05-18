/*
 * stm32_sensors.c
 *
 *  Created on: 12 мар. 2018 г.
 *      Author: developer
 */

/*****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/sensors/gy_us42.h>
#include <nuttx/i2c/i2c_master.h>

#include "stm32.h"
#include "stm32f4blackbox.h"
#include "stm32_i2c.h"

#ifdef CONFIG_SENSORS

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_GY_US42
#if !defined CONFIG_STM32_I2C1 && !defined CONFIG_STM32_I2C2 && !defined CONFIG_STM32_I2C3
#  error "GY_US42 driver requires CONFIG_STM32_I2Cx to be enabled"
#endif

/*****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define GY_US42_I2C_BUS 1 /* sonar is connected to i2c1 port */

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_sensors_gy_us42_initialize(int minor) {
	struct i2c_master_s *i2c;

	sninfo("INFO: Initializing GY_US42\n");

	i2c = stm32_i2cbus_initialize(GY_US42_I2C_BUS);
	if (i2c == NULL) {
		snerr("ERROR: Failed to initialize SPI port %d\n", BMP280_SPI_PORT);
		return -ENODEV;
	}

	char devpath[12] = {0};
	snprintf(devpath, sizeof(devpath), "/dev/range%d", minor);
	return gy_us42_register(devpath, i2c);
}

#endif /* CONFIG_SENSORS_GY_US42 */
#endif /* CONFIG_SENSORS */
