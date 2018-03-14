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

#include <debug.h>

#include <nuttx/sensors/bmp280.h>
#include <nuttx/spi/spi.h>

#include "stm32.h"
#include "omnibus4prov2.h"
#include "stm32_spi.h"

#ifdef CONFIG_SENSORS

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SENSORS_BMP280
#ifndef CONFIG_STM32_SPI3
#  error "BMP280 driver requires CONFIG_STM32_SPI3 to be enabled"
#endif
#endif

/*****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define BMP280_SPI_PORT 3 /* BMP280 is connected to SPI3 port */

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

#endif /* CONFIG_SENSORS */
