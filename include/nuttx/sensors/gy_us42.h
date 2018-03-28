/*
 * gy-us42.h
 *
 *  Created on: 17 мар. 2018 г.
 *      Author: developer
 */

#ifndef NUTTX_INCLUDE_NUTTX_SENSORS_GY_US42_H_
#define NUTTX_INCLUDE_NUTTX_SENSORS_GY_US42_H_

#include <nuttx/config.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_GY_US42)

struct i2c_master_s;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int gy_us42_register(FAR const char *devpath, FAR struct i2c_master_s *i2c);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_GY_US42 */

#endif /* NUTTX_INCLUDE_NUTTX_SENSORS_GY_US42_H_ */
