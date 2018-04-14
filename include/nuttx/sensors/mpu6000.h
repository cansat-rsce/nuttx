
#ifndef __INCLUDE_NUTTX_SENSORS_MPU6000_H
#define __INCLUDE_NUTTX_SENSORS_MPU6000_H

#include <nuttx/config.h>

#include <time.h>
#include <stdint.h>

#include <nuttx/irq.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MPU6000)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define MPU6000_SPI_FREQ         100000

#define MPU6000_DEV_FORMAT   "/dev/mpu%d"
#define MPU6000_DEV_NAMELEN  16

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
	MPU6000_SETTING_ACC_FULLSCALE_2G = 0,
	MPU6000_SETTING_ACC_FULLSCALE_4G,
	MPU6000_SETTING_ACC_FULLSCALE_8G,
	MPU6000_SETTING_ACC_FULLSCALE_16G
} mpu6000_setting_acc_fullscale_t;

typedef enum {
	MPU6000_SETTING_GYRO_FULLSCALE_250DPS = 0,
	MPU6000_SETTING_GYRO_FULLSCALE_500DPS,
	MPU6000_SETTING_GYRO_FULLSCALE_1000DPS,
	MPU6000_SETTING_GYRO_FULLSCALE_2000DPS
} mpu6000_setting_gyro_fullscale_t;

typedef enum {
	MPU6000_SETTING_CLKMODE_INTERNAL_8MHz = 0,
	MPU6000_SETTING_CLKMODE_PLL_XG,
	MPU6000_SETTING_CLKMODE_PLL_YG,
	MPU6000_SETTING_CLKMODE_PLL_ZG,
	MPU6000_SETTING_CLKMODE_EXTERNAL_32DOT768kHz,
	MPU6000_SETTING_CLKMODE_EXTERNAL_19DOT2MHz,
	MPU6000_SETTING_CLKMODE_STOPPED = 7
} mpu6000_setting_clkmode_t;

//TODO дефолтные настройки

typedef struct {
	int16_t x, y, z;
} mpu6000_sensordata_raw_t;

typedef struct {
	mpu6000_sensordata_raw_t acc;
	int16_t temperature;
	mpu6000_sensordata_raw_t gyro;
	struct timespec time;
} mpu6000_record_raw_t; //TODO dynamically turn off sensors

typedef struct {
	float x, y, z;
} mpu6000_sensordata_t;

typedef struct {
	mpu6000_sensordata_t acc;
	float temperature;
	mpu6000_sensordata_t gyro;
	struct timespec time;
} mpu6000_record_t;

typedef enum { //TODO
	MPU6000_CMD_SET_ACC_FULLSCALE,
	MPU6000_CMD_SET_GYRO_FULLSCALE,
	MPU6000_CMD_SET_FILTER,
	MPU6000_CMD_SET_SAMPLERATE_DIVIDER,
	MPU6000_CMD_SET_CLKMODE,
	MPU6000_CMD_SET_CONVERT,
	MPU6000_CMD_GET_CONVERT,
	MPU6000_CMD_FLUSHFIFO,
} mpu6000_ioctl_cmd_t;

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
 * Name: mpu6000_register
 *
 * Description:
 *   Register the mpu6000 character device as '/dev/mpuN'
 *
 * Input Parameters:
 *   spi     - An instance of the SPI interface to use to communicate with BMP280
 *	 minor   - Number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/
int mpu6000_register(FAR struct spi_dev_s *spi, int minor, int (_irqbind)(xcpt_t isr, FAR void *arg));

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_SENSORS_MPU6000 */
#endif /* __INCLUDE_NUTTX_SENSORS_MPU6000_H */
