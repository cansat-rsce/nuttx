/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_DEBUG_SENSORS

#define CONFIG_DEBUG_INFO
#define CONFIG_DEBUG_SENSORS_INFO

#endif

#include <stdlib.h>
#include <stdio.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/bmp280.h>
#include <nuttx/random.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_BMP280)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DEVID               0x58

#define BMP280_T1_MSB		0x88
#define BMP280_T1_LSB		0x89
#define BMP280_T2_MSB		0x8A
#define BMP280_T2_LSB		0x8B
#define BMP280_T3_MSB		0x8C
#define BMP280_T3_LSB		0x8D
#define BMP280_P1_MSB		0x8E
#define BMP280_P1_LSB		0x8F
#define BMP280_P2_MSB		0x90
#define BMP280_P2_LSB		0x91
#define BMP280_P3_MSB		0x92
#define BMP280_P3_LSB		0x93
#define BMP280_P4_MSB		0x94
#define BMP280_P4_LSB		0x95
#define BMP280_P5_MSB		0x96
#define BMP280_P5_LSB		0x97
#define BMP280_P6_MSB		0x98
#define BMP280_P6_LSB		0x99
#define BMP280_P7_MSB		0x9A
#define BMP280_P7_LSB		0x9B
#define BMP280_P8_MSB		0x9C
#define BMP280_P8_LSB		0x9D
#define BMP280_P9_MSB		0x9E
#define BMP280_P9_LSB		0x9F

#define BMP280_DEVID        0xD0
#define BMP280_SOFT_RESET   0xE0
#define BMP280_CTRL_MEAS    0xF4
#define BMP280_CONFIG		0XF5
#define BMP280_PRESS_MSB	0xF7
#define BMP280_PRESS_LSB	0xF8
#define BMP280_PRESS_XLSB	0xF9
#define BMP280_TEMP_MSB		0xFA
#define BMP280_TEMP_LSB		0xFB
#define BMP280_TEMP_XLSB	0xFC

#define READADDR(ADDR)		( ADDR | (1 << 7) )
#define WRITEADDR(ADDR)		( ADDR & ~(1 << 7))

#define FORM_CTRL_MEAS(MODE, OS_P, OS_T)	(uint8_t)( MODE | ( OS_P << 2 ) | ( OS_T << 5 ) )
#define FORM_CONFIG(STANDBYTIME, FILTER)	(uint8_t)( ( STANDBYTIME << 5 ) | ( FILTER << 2 ) )

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

//FIXME тут, наверное, нужно выравнивание структур отключить?
typedef struct {
	uint16_t T1;
	int16_t T2, T3;
	uint16_t P1;
	int16_t P2, P3, P4, P5, P6, P7, P8, P9;
} bmp280_calibration_values_t;

typedef struct bmp280_dev_s
{
	FAR struct spi_dev_s *spi; 		/* SPI interface */
	int devnum;						/* Number of /dev/baroN */
	bmp280_calibration_values_t calvals;
	/*bool calvals_correct;*/
	bmp280_parameters_t params;
} bmp280_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void _configspi(FAR struct spi_dev_s *spi);
static uint8_t _getreg8(FAR bmp280_t *priv, uint8_t regaddr);
static uint16_t _getreg16(FAR bmp280_t *priv, uint8_t regaddr);
static void _getregmany(FAR bmp280_t *priv, uint8_t regaddr, size_t count, FAR void * buffer);
static void _putreg8(FAR bmp280_t *priv, uint8_t regaddr, uint8_t regval);
static void _updatecaldata(FAR bmp280_t *priv);
static bmp280_data_raw_t _read_raw(FAR bmp280_t *priv);
static bmp280_data_t _recalc(FAR bmp280_calibration_values_t * calvals, bmp280_data_raw_t rawdata);

/* Character driver methods */

static int _open(FAR struct file *filep);
static int _close(FAR struct file *filep);
static ssize_t _read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t _write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int _ioctl(FAR struct file* filep, int cmd, unsigned long arg);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_bmp280fops =
{
  _open,                  		/* open */
  _close,                 		/* close */
  _read,                  		/* read */
  _write,                 		/* write */
  0,                            /* seek */
  _ioctl,                            /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                            /* poll */
#endif
  0                             /* unlink */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _configspi
 *
 * Description:
 *
 ****************************************************************************/

static inline void _configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the BMP280 */

  SPI_SETMODE(spi, SPIDEV_MODE3);
  SPI_SETBITS(spi, 8);
  (void)SPI_HWFEATURES(spi, 0);
  (void)SPI_SETFREQUENCY(spi, BMP280_FREQ);
}

/****************************************************************************
 * Name: _getreg8
 *
 * Description:
 *   Read from an 8-bit BMP280 register
 *
 ****************************************************************************/

static uint8_t _getreg8(FAR bmp280_t *priv, uint8_t regaddr)
{
  uint8_t regval;

  /* If SPI bus is shared then lock and configure it */

  (void)SPI_LOCK(priv->spi, true);
  _configspi(priv->spi);

  /* Select the BMP280 */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(priv->devnum), true);

  /* Send register to read and get the next byte */

  (void)SPI_SEND(priv->spi, READADDR( regaddr ));
  SPI_RECVBLOCK(priv->spi, &regval, 1);

  /* Deselect the BMP280 */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(priv->devnum), false);

  /* Unlock bus */

  (void)SPI_LOCK(priv->spi, false);

  return regval;
}

/****************************************************************************
 * Name: _getreg16
 *
 * Description:
 *   Read two 8-bit from a BMP280 register
 *
 ****************************************************************************/
static uint16_t _getreg16(FAR bmp280_t *priv, uint8_t regaddr)
{
  uint16_t regval;

  /* If SPI bus is shared then lock and configure it */

  (void)SPI_LOCK(priv->spi, true);
  _configspi(priv->spi);

  /* Select the BMP280 */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(priv->devnum), true);

  /* Send register to read and get the next 2 bytes */

  (void)SPI_SEND(priv->spi, READADDR( regaddr ));
  SPI_RECVBLOCK(priv->spi, &regval, 2);

  /* Deselect the BMP280 */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(priv->devnum), false);

  /* Unlock bus */

  (void)SPI_LOCK(priv->spi, false);

  return regval;
}

/****************************************************************************
 * Name: _getregmany
 *
 * Description:
 *   Read many 8-bit from a BMP280 register
 *
 ****************************************************************************/
static void _getregmany(FAR bmp280_t *priv, uint8_t regaddr, size_t count, FAR void * buffer)
{
  /* If SPI bus is shared then lock and configure it */

  (void)SPI_LOCK(priv->spi, true);
  _configspi(priv->spi);

  /* Select the BMP280 */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(priv->devnum), true);

  /* Send register to read and get the next 2 bytes */

  (void)SPI_SEND(priv->spi, READADDR( regaddr ));
  SPI_RECVBLOCK(priv->spi, buffer, count);

  /* Deselect the BMP280 */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(priv->devnum), false);

  /* Unlock bus */

  (void)SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: _putreg8
 *
 * Description:
 *   Write to an 8-bit BMP280 register
 *
 ****************************************************************************/
static void _putreg8(FAR bmp280_t *priv, uint8_t regaddr, uint8_t regval)
{
  /* If SPI bus is shared then lock and configure it */

  (void)SPI_LOCK(priv->spi, true);
  _configspi(priv->spi);

  /* Select the BMP280 */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(priv->devnum), true);

  /* Send register address and set the value */

  (void)SPI_SEND(priv->spi, WRITEADDR( regaddr ));
  (void)SPI_SEND(priv->spi, regval);

  /* Deselect the BMP280 */

  SPI_SELECT(priv->spi, SPIDEV_BAROMETER(priv->devnum), false);

  /* Unlock bus */

  (void)SPI_LOCK(priv->spi, false);
}

/****************************************************************************
 * Name: _checkid
 *
 * Description:
 *   Read and verify the BMP280 chip ID
 *
 ****************************************************************************/

static int _checkid(FAR bmp280_t *priv)
{
  uint8_t devid = 0;

  /* Read device ID */

  devid = _getreg8(priv, BMP280_DEVID);
  sninfo("devid: 0x%02x\n", devid);

  if (devid != (uint16_t)DEVID)
    {
      /* ID is not Correct */

      snerr("ERROR: Wrong Device ID!\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: _updatecaldata
 *
 * Description:
 *   Update Calibration Coefficient Data
 *
 ****************************************************************************/

static inline void _updatecaldata(FAR bmp280_t *priv)
{
	bmp280_calibration_values_t * calvals = &priv->calvals;
	_getregmany(priv, BMP280_T1_MSB, sizeof(bmp280_calibration_values_t), calvals);
}

/****************************************************************************
 * Name: _updateparams
 *
 * Description:
 *   Update Calibration Coefficient Data
 *
 ****************************************************************************/

static inline void _updateparams(FAR bmp280_t *priv) {
	FAR bmp280_parameters_t * params = &( priv->params );

	uint8_t tmp = FORM_CTRL_MEAS(	params->mode,
									params->pressure_oversampling,
									params->temperature_oversampling );
	_putreg8(priv, BMP280_CTRL_MEAS, tmp);

	tmp = FORM_CONFIG(	params->standbytyme,
						params->filter);
	_putreg8(priv, BMP280_CTRL_MEAS, tmp);
}

/****************************************************************************
 * Name: _read_raw
 *
 * Description:
 *   Read raw pressure and temperature from BMP280 and store it in the
 *   bmp280_data_t.
 *
 ****************************************************************************/

static bmp280_data_raw_t _read_raw(FAR bmp280_t *priv)
{
	bmp280_data_raw_t retval = {0, 0};

	/* Issue a read temperature command */
	if(priv->params.mode == BMP280_MODE_FORCED)	{
		_putreg8( priv, BMP280_CTRL_MEAS, FORM_CTRL_MEAS(	priv->params.mode,
															priv->params.pressure_oversampling,
															priv->params.temperature_oversampling) );
		/* Wait 5ms */
		nxsig_usleep(5000); //FIXME для forced режима лучше бы нормально ждать с таймаутом. Впрочем, кому он нужен?
	}

	uint8_t tmp[6];

	_getregmany(priv, BMP280_PRESS_MSB, 6, tmp);

	/* Read temperature and pressure*/
	retval.pressure = ((uint32_t)tmp[0] << 12) | ((uint32_t)tmp[1] << 4) | ((uint32_t)tmp[2] >> 4);
	retval.temperature = ((uint32_t)tmp[3] << 12) | ((uint32_t)tmp[4] << 4) | ((uint32_t)tmp[5] >> 4);

  	sninfo("Uncompensated temperature = %d\n", retval.temperature);
  	sninfo("Uncompensated pressure = %d\n", retval.pressure);
  	add_sensor_randomness(retval.pressure);
  	add_sensor_randomness(retval.temperature);

  	return retval;
}

/****************************************************************************
 * Name: _recalc
 *
 * Description:
 *   Recalcs pressure (in Pa) and temperature (in degC) from raw data.
 *
 ****************************************************************************/

static bmp280_data_t _recalc(FAR bmp280_calibration_values_t * calvals, bmp280_data_raw_t rawdata) {
	double t_fine;
	bmp280_data_t retval = {0, 0};

	{
		double var1, var2;
		var1 = (((double)rawdata.temperature)/16384.0 - ((double)calvals->T1)/1024.0) * ((double)calvals->T2);
		var2 = ((((double)rawdata.temperature)/131072.0 - ((double)calvals->T1)/8192.0) *
		(((double)rawdata.temperature)/131072.0 - ((double) calvals->T1)/8192.0)) * ((double)calvals->T3);
		t_fine = var1 + var2;
		retval.temperature = (var1 + var2) / 5120.0;
	}

	{
		double var1, var2;
		var1 = (t_fine/2.0) - 64000.0;
		var2 = var1 * var1 * ((double)calvals->P6) / 32768.0;
		var2 = var2 + var1 * ((double)calvals->P5) * 2.0;
		var2 = (var2/4.0)+(((double)calvals->P4) * 65536.0);
		var1 = (((double)calvals->P3) * var1 * var1 / 524288.0 + ((double)calvals->P2) * var1) / 524288.0;
		var1 = (1.0 + var1 / 32768.0)*((double)calvals->P1);
		if (var1 == 0.0)
		{
			return retval; // avoid exception caused by division by zero
		}
		retval.pressure = 1048576.0 - (double)rawdata.pressure;
		retval.pressure = (retval.pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)calvals->P9) * retval.pressure * retval.pressure / 2147483648.0;
		var2 = retval.pressure * ((double)calvals->P8) / 32768.0;
		retval.pressure = retval.pressure + (var1 + var2 + ((double)calvals->P7)) / 16.0;
	}

	return retval;
}

/****************************************************************************
 * Name: _open
 *
 * Description:
 *   This routine is called whenever the BMP280 device is opened.
 *
 ****************************************************************************/

static int _open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: _close
 *
 * Description:
 *   This routine is called when the BMP280 device is closed.
 *
 ****************************************************************************/

static int _close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: _read
 *
 * Description:
 *   This routine handles read from device.
 *
 ****************************************************************************/

static ssize_t _read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
	FAR struct inode        *inode = filep->f_inode;
	FAR bmp280_t 			*priv  = inode->i_private;

	if (!buffer) {
		snerr("ERROR: Buffer is null\n");
		return -EINVAL;
	}

	if( buflen != 8 && buflen != 16) {
		snerr("ERROR: You can't read something other than raw data (8 bytes) or converted data (16 bytes)\n");
		return -EINVAL;
	}

	// Get the raw pressure
	*( (bmp280_data_raw_t *) buffer )  = _read_raw(priv);

	if( buflen == 16 )
		/* Get the pressure compensated */
		*( (bmp280_data_t *) buffer ) = _recalc(&( priv->calvals ), *( (bmp280_data_raw_t *) buffer ));

	/* Return size of the buffer (which is equal to real data size)*/
	return buflen;
}

/****************************************************************************
 * Name: _write
 ****************************************************************************/

static ssize_t _write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

static int _ioctl(FAR struct file* filep, int cmd, unsigned long arg) {
	FAR struct inode        *inode = filep->f_inode;
	FAR bmp280_t 			*priv  = inode->i_private;
	FAR bmp280_parameters_t *params = &(priv->params);


	//TODO ассерты что ль поставить
	switch(cmd) {

	case BMP280_IOCTL_CMD_OVERSAMPLING_P:
		params->pressure_oversampling = arg;
		break;

	case BMP280_IOCTL_CMD_OVERSAMPLING_T:
		params->temperature_oversampling = arg;
		break;

	case BMP280_IOCTL_CMD_MODE:
		params->mode = arg;
		break;

	case BMP280_IOCTL_CMD_STANDBYTIME:
		params->standbytyme = arg;
		break;

	case BMP280_IOCTL_CMD_FILTER:
		params->filter = arg;
		break;

	default:
		return -EINVAL;
	}

	_updateparams(priv);
	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bmp280_register
 *
 * Description:
 *   Register the BMP280 character device as '/dev/baroN'
 *
 * Input Parameters:
 *   spi     - An instance of the I2C interface to use to communicate with BMP280
 *	 minor   - Number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int bmp280_register(FAR struct spi_dev_s *spi, int minor)
{
	FAR bmp280_t *priv;
	int ret;

	/* Initialize the BMP280 device structure */

	priv = (FAR bmp280_t *)kmm_malloc(sizeof(bmp280_t));
	if (!priv) {
		snerr("ERROR: Failed to allocate bmp280 instance\n");
		return -ENOMEM;
	}

	priv->spi = spi;
	priv->devnum = minor;

	//default config
	priv->params.mode = BMP280_MODE_DEFAULT;
	priv->params.pressure_oversampling = BMP280_OVERSAMPLING_DEFAULT_P;
	priv->params.temperature_oversampling = BMP280_OVERSAMPLING_DEFAULT_T;
	priv->params.standbytyme = BMP280_STANDBYTIME_DEFAULT;
	priv->params.filter = BMP280_FILTER_DEFAULT;

	/* Check Device ID */
	ret = _checkid(priv);
	if (ret < 0) {
		snerr("ERROR: Failed to register driver: %d\n", ret);
		kmm_free(priv);
		return ret;
	}

	/* Update config */
	_updateparams(priv);

	/* Read the coefficient value */
	_updatecaldata(priv);

	/* Fill format string */
	char devname[BMP280_DEV_NAMELEN];
	snprintf(devname, BMP280_DEV_NAMELEN, BMP280_DEV_FORMAT, minor);

	/* Register the character driver */
	ret = register_driver(devname, &g_bmp280fops, 0666, priv);
	if (ret < 0) {
		snerr("ERROR: Failed to register driver: %d\n", ret);
		kmm_free(priv);
	}

#ifdef CONFIG_DEBUG_SENSORS_INFO
	sninfo("BMP280 driver loaded successfully!\n");

	nxsig_usleep(1000);

	struct file thisIsNotFile;
	struct inode thisIsNotInode;
	bmp280_data_t result = {0, 0};

	thisIsNotFile.f_inode = &thisIsNotInode;
	thisIsNotInode.i_private = priv;

	while(true) {
		_read(&thisIsNotFile, &result, 16);
		sninfo("BMP280: pressure: %f pa, temperature: %f deg\n",
				result.pressure, result.temperature);
		nxsig_usleep(1000);
	}
#endif
	return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_BMP280 */
