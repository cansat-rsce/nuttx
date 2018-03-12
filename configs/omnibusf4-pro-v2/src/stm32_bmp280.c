/*
 * stm32_bmp280.c
 *
 *  Created on: 12 мар. 2018 г.
 *      Author: snork
 */

#include <nuttx/config.h>
#include <debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/bmp280.h>

#include "stm32.h"
#include "omnibus4prov2.h"
#include "stm32_spi.h"

#define BMP280_SPI (3)

#ifndef __ASSEMBLY__

int stm32_bmp280_initialize(int minor)
{
    FAR struct spi_dev_s * dev;
    dev = stm32_spibus_initialize(BMP280_SPI);
    if (!dev)
    {
        snerr("cant initialize bmp280 spi bus %d\n", BMP280_SPI);
        return -ENODEV;
    }

    int ret = bmp280_register(dev, minor);
    if (ret < 0)
    {
        snerr("cant register bmp280 device: %d\n", ret);
        return ret;
    }


    return 0;
}

#endif // #ifndef __ASSEMBLY__
