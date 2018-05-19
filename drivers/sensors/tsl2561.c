/*
 * tsl2561.c
 *
 *  Created on: 31 мар. 2018 г.
 *      Author: developer
 */
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include "nuttx/sensors/tsl2561.h"

#define CONTROL_REG_ADDR 			 0x00
#define CONTROL_REG_VALUE_POWER_DOWN 0x00
#define CONTROL_REG_VALUE_POWER_UP   0x03

#define TSL2561_COMMAND_BIT          0x80    ///< Must be 1
#define TSL2561_CLEAR_BIT            0x40    ///< Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT             0x20    ///< 1 = read/write word (rather than byte)
#define TSL2561_BLOCK_BIT            0x10    ///< 1 = using block read/write

#define TSL2561_CONTROL_POWERON      0x03    ///< Control register setting to turn on
#define TSL2561_CONTROL_POWEROFF     0x00    ///< Control register setting to turn off

#define TSL2561_REGISTER_CONTROL     		0x00 // Control/power register
#define TSL2561_REGISTER_TIMING             0x01 // Set integration time register
#define TSL2561_REGISTER_THRESHHOLDL_LOW    0x02 // Interrupt low threshold low-byte
#define TSL2561_REGISTER_THRESHHOLDL_HIGH   0x03 // Interrupt low threshold high-byte
#define TSL2561_REGISTER_THRESHHOLDH_LOW    0x04 // Interrupt high threshold low-byte
#define TSL2561_REGISTER_THRESHHOLDH_HIGH   0x05 // Interrupt high threshold high-byte
#define TSL2561_REGISTER_INTERRUPT          0x06 // Interrupt settings
#define TSL2561_REGISTER_CRC                0x08 // Factory use only
#define TSL2561_REGISTER_ID                 0x0A // TSL2561 identification setting
#define TSL2561_REGISTER_CHAN0_LOW          0x0C // Light data channel 0, low byte
#define TSL2561_REGISTER_CHAN0_HIGH         0x0D // Light data channel 0, high byte
#define TSL2561_REGISTER_CHAN1_LOW          0x0E // Light data channel 1, low byte
#define TSL2561_REGISTER_CHAN1_HIGH         0x0F  // Light data channel 1, high byte

#define K1C  0x0043 // 0.130 * 2^RATIO_SCALE
#define B1C  0x0204 // 0.0315 * 2^LUX_SCALE
#define M1C  0x01ad // 0.0262 * 2^LUX_SCALE
#define K2C  0x0085 // 0.260 * 2^RATIO_SCALE
#define B2C  0x0228 // 0.0337 * 2^LUX_SCALE
#define M2C  0x02c1 // 0.0430 * 2^LUX_SCALE
#define K3C  0x00c8 // 0.390 * 2^RATIO_SCALE
#define B3C  0x0253 // 0.0363 * 2^LUX_SCALE
#define M3C  0x0363 // 0.0529 * 2^LUX_SCALE
#define K4C  0x010a // 0.520 * 2^RATIO_SCALE
#define B4C  0x0282 // 0.0392 * 2^LUX_SCALE
#define M4C  0x03df // 0.0605 * 2^LUX_SCALE
#define K5C  0x014d // 0.65 * 2^RATIO_SCALE
#define B5C  0x0177 // 0.0229 * 2^LUX_SCALE
#define M5C  0x01dd // 0.0291 * 2^LUX_SCALE
#define K6C  0x019a // 0.80 * 2^RATIO_SCALE
#define B6C  0x0101 // 0.0157 * 2^LUX_SCALE
#define M6C  0x0127 // 0.0180 * 2^LUX_SCALE
#define K7C  0x029a // 1.3 * 2^RATIO_SCALE
#define B7C  0x0037 // 0.00338 * 2^LUX_SCALE
#define M7C  0x002b // 0.00260 * 2^LUX_SCALE
#define K8C  0x029a // 1.3 * 2^RATIO_SCALE
#define B8C  0x0000 // 0.000 * 2^LUX_SCALE
#define M8C  0x0000 // 0.000 * 2^LUX_SCALE
#define K1T  0x0040 // 0.125 * 2^RATIO_SCALE
#define B1T  0x01f2 // 0.0304 * 2^LUX_SCALE
#define M1T  0x01be // 0.0272 * 2^LUX_SCALE
#define K2T  0x0080 // 0.250 * 2^RATIO_SCALE
#define B2T  0x0214 // 0.0325 * 2^LUX_SCALE
#define M2T  0x02d1 // 0.0440 * 2^LUX_SCALE
#define K3T  0x00c0 // 0.375 * 2^RATIO_SCALE
#define B3T  0x023f // 0.0351 * 2^LUX_SCALE
#define M3T  0x037b // 0.0544 * 2^LUX_SCALE
#define K4T  0x0100 // 0.50 * 2^RATIO_SCALE
#define B4T  0x0270 // 0.0381 * 2^LUX_SCALE
#define M4T  0x03fe // 0.0624 * 2^LUX_SCALE
#define K5T  0x0138 // 0.61 * 2^RATIO_SCALE
#define B5T  0x016f // 0.0224 * 2^LUX_SCALE
#define M5T  0x01fc // 0.0310 * 2^LUX_SCALE
#define K6T  0x019a // 0.80 * 2^RATIO_SCALE
#define B6T  0x00d2 // 0.0128 * 2^LUX_SCALE
#define M6T  0x00fb // 0.0153 * 2^LUX_SCALE
#define K7T  0x029a // 1.3 * 2^RATIO_SCALE
#define B7T  0x0018 // 0.00146 * 2^LUX_SCALE
#define M7T  0x0012 // 0.00112 * 2^LUX_SCALE
#define K8T  0x029a // 1.3 * 2^RATIO_SCALE
#define B8T  0x0000 // 0.000 * 2^LUX_SCALE
#define M8T  0x0000 // 0.000 * 2^LUX_SCALE

#define LUX_SCALE   14     // scale by 2^14
#define RATIO_SCALE 9      // scale ratio by 2^9
#define CH_SCALE           10     // scale channel values by 2^10
#define CHSCALE_TINT0      0x7517 // 322/11 * 2^CH_SCALE
#define CHSCALE_TINT1      0x0fe7 // 322/81 * 2^CH_SCALE

#define TSL2561_FREQ 100000

// I2C address options
#define TSL2561_ADDR_LOW          (0x29)    ///< Default address (pin pulled low)
#define TSL2561_ADDR_FLOAT        (0x39)    ///< Default address (pin left floating)
#define TSL2561_ADDR_HIGH         (0x49)    ///< Default address (pin pulled high)

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_TSL2561)

struct tsl2561_dev_s
{
	FAR struct i2c_master_s *i2c;      /* I2C interface */
	struct {
		uint32_t frequency;          /* I2C frequency */
		uint8_t address;             /* I2C address (7- or 10-bit) */
	} bus_config;
	struct {
		uint16_t tsl2561_data16_channel_0;
		uint16_t tsl2561_data16_channel_1;
		int tsl2561_data_lux;
	} data;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Character driver methods */

static int tsl2561_open(FAR struct file *filep);
static int tsl2561_close(FAR struct file *filep);
static int tsl2561_read(FAR struct file *filep, FAR char *buffer, size_t buflen);
static int tsl2561_ioctl(FAR struct file *filep, int cmd, unsigned long arg);

static uint8_t tsl2561_getreg8(FAR struct tsl2561_dev_s *priv, uint8_t regaddr);
static uint16_t tsl2561_getreg16(FAR struct tsl2561_dev_s *priv, uint8_t regaddr);
static void tsl2561_putreg8(FAR struct tsl2561_dev_s *priv, uint8_t regaddr, uint8_t regval);

void tsl2561_enable(FAR struct tsl2561_dev_s *priv);
void tsl2561_disable(FAR struct tsl2561_dev_s *priv);
int tsl2561_calculate(uint16_t ch0, uint16_t ch1);
static int tsl2561_getlux(FAR struct tsl2561_dev_s *priv);

static const struct file_operations g_tsl2561fops =
{
  tsl2561_open,                  /* open   */
  tsl2561_close,                 /* close  */
  tsl2561_read,                  /* read   */
  NULL,                 		 /* write  */
  NULL,						     /* seek   */
  tsl2561_ioctl,			     /* ioctl  */
#ifndef CONFIG_DISABLE_POLL
  0,                             /* poll   */
#endif
  0                              /* unlink */
};

/****************************************************************************
 * Name: tsl2561_getreg8
 *
 * Description:
 *   Read from an 8-bit TSL2561 register
 *
 ****************************************************************************/

static uint8_t tsl2561_getreg8(FAR struct tsl2561_dev_s *priv, uint8_t regaddr)
{
  struct i2c_config_s config;
  uint8_t regval = 0;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->bus_config.frequency;
  config.address   = priv->bus_config.address;
  config.addrlen   = 7;

  /* Write the register address */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
  {
	  snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
  }

  /* Read the register value */

  ret = i2c_read(priv->i2c, &config, &regval, 1);
  if (ret < 0)
  {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
  }

  return regval;
}


/****************************************************************************
 * Name: tsl2561_getreg16
 *
 * Description:
 *   Read two 8-bit from a TSL2561 register
 *
 ****************************************************************************/

static uint16_t tsl2561_getreg16(FAR struct tsl2561_dev_s *priv, uint8_t regaddr)
{
  struct i2c_config_s config;
  uint16_t msb, lsb;
  uint16_t regval = 0;
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->bus_config.frequency;
  config.address   = priv->bus_config.address;
  config.addrlen   = 7;

  /* Register to read */

  ret = i2c_write(priv->i2c, &config, &regaddr, 1);
  if (ret < 0)
    {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return ret;
    }

  /* Read register */

  ret = i2c_read(priv->i2c, &config, (uint8_t *)&regval, 2);
  if (ret < 0)
    {
      snerr("ERROR: i2c_read failed: %d\n", ret);
      return ret;
    }

  /* MSB and LSB are inverted */

  lsb = (regval & 0xFF);
  msb = (regval & 0xFF00) >> 8;

  regval = (msb << 8) | lsb;

  return regval;
}

/****************************************************************************
 * Name: tsl2561_putreg8
 *
 * Description:
 *   Write to an 8-bit TSL2561 register
 *
 ****************************************************************************/


static void tsl2561_putreg8(FAR struct tsl2561_dev_s *priv, uint8_t regaddr,
                           uint8_t regval)
{
  struct i2c_config_s config;
  uint8_t data[2];
  int ret;

  /* Set up the I2C configuration */

  config.frequency = priv->bus_config.frequency;
  config.address   = priv->bus_config.address;
  config.addrlen   = 7;

  data[0] = regaddr;
  data[1] = regval;

  /* Write the register address and value */

  ret = i2c_write(priv->i2c, &config, (uint8_t *) &data, 2);
  if (ret < 0)
  {
      snerr("ERROR: i2c_write failed: %d\n", ret);
      return;
  }

  return;
}

void tsl2561_enable(FAR struct tsl2561_dev_s *priv) {
	uint8_t regaddr = (uint8_t)TSL2561_COMMAND_BIT | (uint8_t)TSL2561_REGISTER_CONTROL;
	uint8_t regval  = (uint8_t)TSL2561_CONTROL_POWERON;
	tsl2561_putreg8(priv, regaddr, regval);
	regaddr = (uint8_t)TSL2561_COMMAND_BIT | (uint8_t)TSL2561_REGISTER_TIMING;
	regval = 0x02;
	tsl2561_putreg8(priv, regaddr, regval);
	regaddr = (uint8_t)TSL2561_COMMAND_BIT | (uint8_t)TSL2561_REGISTER_INTERRUPT;
	regval = 0x00;
	tsl2561_putreg8(priv, regaddr, regval);
}

void tsl2561_disable(FAR struct tsl2561_dev_s *priv)
{
	uint8_t regaddr = (uint8_t)TSL2561_COMMAND_BIT | (uint8_t)TSL2561_REGISTER_CONTROL;
	uint8_t regval  = (uint8_t)TSL2561_CONTROL_POWEROFF;
	tsl2561_putreg8(priv, regaddr, regval);
	free(priv);
}

int tsl2561_calculateLUX(uint16_t ch0, uint16_t ch1)
{

	// iGain - масштаб (0 : 1X, 1 : 16X)
	// tInt - время интеграции, где 0 : 13мс, 1 : 100мс, 2 : 402мс, 3 : ручной
	// iType - тип корпуса (0 : T, 1 : CS)
	// масштабировать, если время интеграции НЕ 402мс

	unsigned int iGain = 0;
	int iType =  1;
	int tInt = 2;

	unsigned long chScale;
	unsigned long channel1;
	unsigned long channel0;
	switch (tInt)
	{
	case 0:    // 13.7мс
		chScale = CHSCALE_TINT0;
		break;
	case 1:    // 101мс
		chScale = CHSCALE_TINT1;
		break;
	default:   // без масштабирования
		chScale = (1 << CH_SCALE);
		break;
	}
	// масштабировать, если gain НЕ 16X
	if (!iGain) chScale = chScale << 4;   // scale 1X to 16X
	// scale the channel values
	channel0 = (ch0 * chScale) >> CH_SCALE;
	channel1 = (ch1 * chScale) >> CH_SCALE;
	//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−
	// find the ratio of the channel values (Channel1/Channel0)
	// protect against divide by zero
	unsigned long ratio1 = 0;
	if (channel0 != 0) ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;
	// round the ratio value
	unsigned long ratio = (ratio1 + 1) >> 1;
	// is ratio <= eachBreak ?
	unsigned int b = 0, m = 0;
	switch (iType)
	{
	case 0: // T, FN and CL package
		if ((ratio >= 0) && (ratio <= K1T))
		{b=B1T; m=M1T;}
		else if (ratio <= K2T)
		{b=B2T; m=M2T;}
		else if (ratio <= K3T)
		{b=B3T; m=M3T;}
		else if (ratio <= K4T)
		{b=B4T; m=M4T;}
		else if (ratio <= K5T)
		{b=B5T; m=M5T;}
		else if (ratio <= K6T)
		{b=B6T; m=M6T;}
		else if (ratio <= K7T)
		{b=B7T; m=M7T;}
		else if (ratio > K8T)
		{b=B8T; m=M8T;}
		break;
	case 1:// CS package
		if ((ratio >= 0) && (ratio <= K1C))
		{b=B1C; m=M1C;}
		else if (ratio <= K2C)
		{b=B2C; m=M2C;}
		else if (ratio <= K3C)
		{b=B3C; m=M3C;}
		else if (ratio <= K4C)
		{b=B4C; m=M4C;}
		else if (ratio <= K5C)
		{b=B5C; m=M5C;}
		else if (ratio <= K6C)
		{b=B6C; m=M6C;}
		else if (ratio <= K7C)
		{b=B7C; m=M7C;}
		else if (ratio > K8C)
		{b=B8C; m=M8C;}
		break;
	}
	unsigned long temp;
	temp = ((channel0 * b) - (channel1 * m));
	// do not allow negative lux value
	if (temp < 0) temp = 0;
	// round lsb (2^(LUX_SCALE−1))
	temp += (1 << (LUX_SCALE - 1));
	// strip off fractional portion
	unsigned long lux = temp >> LUX_SCALE;

	return lux;
}

static int tsl2561_getlux(FAR struct tsl2561_dev_s *priv)
{
	uint16_t ch1 = tsl2561_getreg16(priv, (0x80 | 0x20 | 0x0E));
	printf("ch1 from getlux(): %d\n", ch1);
	uint16_t ch0 = tsl2561_getreg16(priv, (0x80 | 0x20 | 0x0C));
	printf("ch0 from getlux(): %d\n", ch0);

	int lux = tsl2561_calculateLUX(ch0, ch1);
	printf("lux from getlux(): %d\n", lux);
	priv->data.tsl2561_data_lux = lux;
	return lux;
}

/****************************************************************************
 * Name: tsl2561_open
 *
 * Description:
 *   This function is called whenever the TSL2561 device is opened.
 *
 ****************************************************************************/

static int tsl2561_open(FAR struct file *filep)
{
	return OK;
}

/****************************************************************************
 * Name: tsl2561_close
 *
 * Description:
 *   This routine is called when the TSL2561 device is closed.
 *
 ****************************************************************************/

static int tsl2561_close(FAR struct file *filep)
{
	return OK;
}

static int tsl2561_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
	FAR struct inode *inode = filep->f_inode;
	FAR struct tsl2561_dev_s *priv  = inode->i_private;
	FAR uint16_t *lux = (FAR uint16_t *) buffer;

	if (!buffer) {
		snerr("ERROR: Buffer is null\n");
		return -EINVAL;
	}

	if (buflen != 2) {
		snerr("ERROR: Buffer size is not 2\n");
		return -EINVAL;
	}

	*lux = tsl2561_getlux(priv);
	return 2;
}

static int tsl2561_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	FAR struct inode *inode = filep->f_inode;
	FAR struct tsl2561_dev_s *priv = inode->i_private;

	int ret;

	switch(cmd) {

	case TSL2561_IOCTL_CMD_SETUP:
		tsl2561_enable(priv);
		break;
	default:
		ret = -1;
		break;

	}

	return ret;
}

/****************************************************************************
 * Name: tsl2561_register
 *
 * Description:
 *   Register the TSL2561 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             TSL2561
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int tsl2561_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
	FAR struct tsl2561_dev_s *priv;
	int ret;

	/* Initialize the TSL2561 device structure */

	priv = (FAR struct tsl2561_dev_s *)kmm_malloc(sizeof(struct tsl2561_dev_s));
	if (!priv)
    {
		snerr("ERROR: Failed to allocate instance\n");
		return -1;
	}

	priv->i2c = i2c;
	priv->bus_config.address = TSL2561_ADDR_FLOAT;
	priv->bus_config.frequency = TSL2561_FREQ;

	/* Register the character driver */

	ret = register_driver(devpath, &g_tsl2561fops, 0666, priv);
	if (ret < 0)
	{
		snerr("ERROR: Failed to register driver: %d\n", ret);
		kmm_free(priv);
	}

	sninfo("TSL2561 driver loaded successfully!\n");
	return ret;

}


#endif /* CONFIG_I2C && CONFIG_SENSORS_TSL2561 */


