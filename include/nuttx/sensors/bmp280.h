
#ifndef __INCLUDE_NUTTX_SENSORS_BMP280_H
#define __INCLUDE_NUTTX_SENSORS_BMP280_H

#include <stdint.h>
#include <nuttx/config.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_BMP280)

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

#define BMP280_FREQ         100000

#define BMP280_DEV_FORMAT   "/dev/baro%d"
#define BMP280_DEV_NAMELEN  16

/* Configuration ****************************************************************************/
/* Prerequisites:
 *
 * CONFIG_SENSORS_BMP280
 *   Enables support for the BMP280 driver
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

//Количество измерений на один результат. Выставляется отдельно для термометра и барометра
typedef enum {
	BMP280_OVERSAMPLING_OFF 	= 0,
	BMP280_OVERSAMPLING_X1 		= 1,
	BMP280_OVERSAMPLING_X2 		= 2,
	BMP280_OVERSAMPLING_X4 		= 3,
	BMP280_OVERSAMPLING_X8 		= 4,
	BMP280_OVERSAMPLING_X16 	= 5

} bmp280_oversampling_t;

//Время между двумя измерениями в непрерывном режиме
typedef enum {
	BMP280_STANDBYTIME_500US 		= 0,
	BMP280_STANDBYTIME_62DOT5MS 	= 1,
	BMP280_STANDBYTIME_125MS 		= 2,
	BMP280_STANDBYTIME_250MS 		= 3,
	BMP280_STANDBYTIME_500MS 		= 4,
	BMP280_STANDBYTIME_1S 			= 5,
	BMP280_STANDBYTIME_2S 			= 6,
	BMP280_STANDBYTIME_4S 			= 7,
} bmp280_standbytime_t;

/*Режимы работы.
 * SLEEP - сон, FORCED - одиночное измерение по команде, NORMAL - непрерывное измерение */
typedef enum {
	BMP280_MODE_SLEEP 	= 0,
	BMP280_MODE_FORCED 	= 1,
	BMP280_MODE_NORMAL	= 3,
} bmp280_mode_t;

//Режимы фильтрованных измерений (см. даташит)
typedef enum {
	BMP280_FILTER_OFF 	= 0,
	BMP280_FILTER_X2 	= 1,
	BMP280_FILTER_X4 	= 2,
	BMP280_FILTER_X8 	= 3,
	BMP280_FILTER_X16 	= 4,
} bmp280_filter_t;

typedef struct {
	// Режим измерений
	bmp280_mode_t mode;
	// Режим префильтрации измерений давления
	bmp280_oversampling_t pressure_oversampling;
	// Режим префильтрации измерений температуры
	bmp280_oversampling_t temperature_oversampling;
	// Период непрерывных измерений
	bmp280_standbytime_t standbytyme;
	// Режим фильтрации
	bmp280_filter_t filter;

} bmp280_parameters_t;

#define BMP280_OVERSAMPLING_DEFAULT_P 	BMP280_OVERSAMPLING_X8
#define BMP280_OVERSAMPLING_DEFAULT_T 	BMP280_OVERSAMPLING_X2
#define BMP280_STANDBYTIME_DEFAULT 		BMP280_STANDBYTIME_500US
#define BMP280_MODE_DEFAULT 			BMP280_MODE_NORMAL
#define BMP280_FILTER_DEFAULT 			BMP280_FILTER_OFF

typedef struct {
	int32_t pressure, temperature;
} bmp280_data_raw_t;

typedef struct {
	double pressure, temperature;
} bmp280_data_t;

typedef enum {
	BMP280_IOCTL_CMD_OVERSAMPLING_P,
	BMP280_IOCTL_CMD_OVERSAMPLING_T,
	BMP280_IOCTL_CMD_MODE,
	BMP280_IOCTL_CMD_STANDBYTIME,
	BMP280_IOCTL_CMD_FILTER,
} bmp280_ioctl_cmd_t;

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
 * Name: bmp280_register
 *
 * Description:
 *   Register the BMP280 character device as '/dev/baroN'
 *
 * Input Parameters:
 *   spi     - An instance of the SPI interface to use to communicate with BMP280
 *	 minor   - Number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp280_register(FAR struct spi_dev_s *spi, int minor);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_SPI && CONFIG_SENSORS_BMP280 */
#endif /* __INCLUDE_NUTTX_SENSORS_BMP280_H */
