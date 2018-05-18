/*
 * tsl2561.h
 *
 *  Created on: 31 мар. 2018 г.
 *      Author: developer
 */

#ifndef NUTTX_INCLUDE_NUTTX_SENSORS_GY_US42_H_
#define NUTTX_INCLUDE_NUTTX_SENSORS_GY_US42_H_

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_GY_US42)

struct i2c_master_s;

typedef enum {
	TSL2561_GAIN_1X = 0,
	TSL2561_GAIN_16X = 1
} tsl2561_gain_t;

typedef enum {
	TSL2561_TYPE_T = 0,
	TSL2561_TYPE_CS = 1
} tsl2561_type_t;

typedef enum {
	TSL2561_INT_13MS = 0,
	TSL2561_INT_100MS = 1,
	TSL2561_INT_402MS = 2,
	TSL2561_INT_MANUAL = 3
} tsl2561_int_t;


typedef enum {
	TSL2561_IOCTL_CMD_MEASURE_CHANNEL_0,
	TSL2561_IOCTL_CMD_MEASURE_CHANNEL_1,
	TSL2561_IOCTL_CMD_GETLUX,
	TSL2561_IOCTL_CMD_SET_ADDR,
} tsl2561_ioctl_cmd_t;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int tsl2561_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

unsigned int tsl2561_get_lux(unsigned int ch0, unsigned int ch1);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_GY_US42 */

#endif /* NUTTX_INCLUDE_NUTTX_SENSORS_GY_US42_H_ */
