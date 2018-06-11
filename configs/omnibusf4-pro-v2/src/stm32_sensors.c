/*
 * stm32_sensors.c
 *
 *  Created on: 12 мар. 2018 г.
 *      Author: developer
 */

/*****************************************************************************
 * Included Files
 ****************************************************************************/


#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>

#include <nuttx/sensors/bmp280.h>
#include <nuttx/sensors/mpu6000.h>
#include <nuttx/sensors/gy_us42.h>
#include <nuttx/sensors/lsm303c.h>
#include <nuttx/sensors/tsl2561.h>

#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>

#include "stm32.h"
#include "omnibus4prov2.h"
#include "stm32_spi.h"
#include "stm32_i2c.h"

#ifdef CONFIG_SENSORS

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP280
#ifndef CONFIG_STM32_SPI3
#  error "BMP280 driver requires CONFIG_STM32_SPI3 to be enabled"
#endif
#endif

#ifdef CONFIG_SENSORS_MPU6000
#ifndef CONFIG_STM32_SPI1
#  error "MPU6000 driver requires CONFIG_STM32_SPI1 to be enabled"
#endif
#endif

#ifdef CONFIG_SENSORS_GY_US42
#ifndef CONFIG_STM32_I2C2
#  error "GY_US42 driver requires CONFIG_STM32_SPI1 to be enabled"
#endif
#endif

#ifdef CONFIG_SENSORS_LSM303C
#ifndef CONFIG_STM32_I2C2
#  error "LSM303C driver requires CONFIG_STM32_SPI1 to be enabled"
#endif
#endif

#ifdef CONFIG_SENSORS_TSL2561
#ifndef CONFIG_STM32_I2C2
#  error "TSL2561 driver requires CONFIG_STM32_SPI1 to be enabled"
#endif
#endif

/*****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define BMP280_SPI_PORT 3 /* BMP280 is connected to SPI3 port */
#define MPU6000_SPI_PORT 1 /* MPU6000 is connected to SPI1 port */
#define GY_US42_I2C_BUS 2 /* GY_US42 is connected to I2C2 port */
#define LSM303C_I2C_BUS 2 /* LSM303C is connected to I2C2 port */
#define TSL2561_I2C_BUS 2 /* TSL2561 is connected to I2C2 port */

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP280

int stm32_sensors_bmp280_initialize(int minor) {
	struct spi_dev_s *spi;

	sninfo("INFO: Initializing BMP280\n");

	spi = stm32_spibus_initialize(BMP280_SPI_PORT);
	if (spi == NULL) {
		snerr("ERROR: Failed to initialize SPI port %d\n", BMP280_SPI_PORT);
		return -ENODEV;
	}

	return bmp280_register(spi, minor);
}

#endif /* CONFIG_SENSORS_BMP280 */

#ifdef CONFIG_SENSORS_MPU6000

static int _mpu6000_irqbind(xcpt_t isr, FAR void *arg, bool state) {
	return stm32_gpiosetevent(GPIO_MPU6000_INT, state, false, false, isr, arg);
}

int stm32_sensors_mpu6000_initialize(int minor) {
	FAR struct spi_dev_s *spi;

	sninfo("INFO: Initializing MPU6000\n");

	spi = stm32_spibus_initialize(MPU6000_SPI_PORT);
	if (spi == NULL) {
		snerr("ERROR: Failed to initialize SPI port %d\n", MPU6000_SPI_PORT);
		return -ENODEV;
	}

	return mpu6000_register(spi, minor, NULL);
}

#endif /* CONFIG_SENSORS_MPU6000 */

#ifdef CONFIG_SENSORS_GY_US42

int stm32_sensors_gy_us42_initialize(int minor) {
	FAR struct i2c_master_s *i2c;

	sninfo("INFO: Initializing GY_US42\n");

	i2c = stm32_i2cbus_initialize(GY_US42_I2C_BUS);
	if(i2c == NULL) {
		snerr("ERROR: Failed to initialize I2C bus %d\n", GY_US42_I2C_BUS);
	}

	char devname[16];
	snprintf(devname, 16, "/dev/sonar%d", minor);
	return gy_us42_register(devname, i2c);
}

#endif /* CONFIG_SENSORS_GY_US42 */

#ifdef CONFIG_SENSORS_LSM303C

int stm32_sensors_lsm303c_initialize(int minor) {
	FAR struct i2c_master_s *i2c;

	sninfo("INFO: Initializing LSM303C\n");

	i2c = stm32_i2cbus_initialize(LSM303C_I2C_BUS);
	if(i2c == NULL) {
		snerr("ERROR: Failed to initialize I2C bus %d\n", GY_US42_I2C_BUS);
	}

	return lsm303c_register(i2c, minor);
}

#endif /* CONFIG_SENSORS_LSM303C */

#ifdef CONFIG_SENSORS_TSL2561

int stm32_sensors_tsl2561_initialize(int minor) {
	FAR struct i2c_master_s *i2c;

	sninfo("INFO: Initializing TSL2561\n");

	i2c = stm32_i2cbus_initialize(TSL2561_I2C_BUS);
	if (i2c == NULL) {
		snerr("ERROR: Failed to initialize I2C bus %d\n", TSL2561_I2C_BUS);
		return -ENODEV;
	}

	char devpath[12] = {0};
	snprintf(devpath, sizeof(devpath), "/dev/lumen%d", minor);
	return tsl2561_register(devpath, i2c);
}

#endif /* CONFIG_SENSORS_TSL2561 */

#endif /* CONFIG_SENSORS */
