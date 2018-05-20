
#ifndef __INCLUDE_NUTTX_SENSORS_LSM303C_H
#define __INCLUDE_NUTTX_SENSORS_LSM303C_H

#include <nuttx/config.h>

#include <time.h>
#include <stdint.h>

#include <nuttx/irq.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM303C)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define LSM303C_I2C_FREQ         100000

#define LSM303C_DEV_FORMAT   "/dev/mag%d"
#define LSM303C_DEV_NAMELEN  16

/* Configuration ****************************************************************************/
/* Prerequisites:
 *
 * CONFIG_SENSORS_MPU6000
 *   Enables support for the MPU6000 driver
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef enum {
	LSM303C_SETTING_OPERATIVE_MODE_LOW_POWER = 0,
	LSM303C_SETTING_OPERATIVE_MODE_MEDIUM_PERFORMANCE,
	LSM303C_SETTING_OPERATIVE_MODE_HIGH_PERFORMANCE,
	LSM303C_SETTING_OPERATIVE_MODE_ULTRA_HIGH_PERFORMANCE,
} lsm303c_setting_operative_mode_t;

typedef enum {
	LSM303C_SETTING_ODR_0DOT625Hz = 0,
	LSM303C_SETTING_ODR_1DOT25Hz,
	LSM303C_SETTING_ODR_2DOT5Hz,
	LSM303C_SETTING_ODR_5Hz,
	LSM303C_SETTING_ODR_10Hz,
	LSM303C_SETTING_ODR_20Hz,
	LSM303C_SETTING_ODR_40Hz,
	LSM303C_SETTING_ODR_80Hz,
} lsm303c_setting_odr_t;

typedef enum {
	LSM303C_SETTING_MODE_CONTINUOUS = 0,
	LSM303C_SETTING_MODE_SINGLE,
	LSM303C_SETTING_MODE_POWERDOWN = 3,
} lsm303c_setting_mode_t;

//TODO дефолтные настройки
#pragma pack(push, 1)

typedef struct {
	int16_t x, y, z;
} lsm303c_sensordata_raw_t;

typedef struct {
	lsm303c_sensordata_raw_t field;
	int16_t temperature;
	struct timespec time;
} lsm303c_record_raw_t;

typedef struct {
	float x, y, z;
} lsm303c_sensordata_t;

typedef struct {
	lsm303c_sensordata_t field;
	float temperature;
	struct timespec time;
} lsm303c_record_t;

#pragma pack(pop)

typedef enum {
	LSM303C_CMD_SET_TEMPERATURE_ENABLED,
	LSM303C_CMD_SET_OPERATIVE_MODE_XY,
	LSM303C_CMD_SET_OPERATIVE_MODE_Z,
	LSM303C_CMD_SET_ODR,
	LSM303C_CMD_SET_MODE,
} lsm303c_ioctl_cmd_t;

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: lsm303c_register
 *
 * Description:
 *   Register the LSM303C character device as '/dev/magN'
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to use to communicate with LSM303C
 *	 minor   - Number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int lsm303c_register(FAR struct i2c_master_s *i2c, int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LSM303C */
#endif /* __INCLUDE_NUTTX_SENSORS_LSM303C_H */
