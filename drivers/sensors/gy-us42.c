/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>

#define CONFIG_I2C
#define CONFIG_SENSORS_GY_US42

#define GY-US42_ADDR         0x77
#define GY-US42_FREQ         100000
#define ADDR_UNLOCK_1	     0xAA
#define ADDR_UNLOCK_2		 0xA5
#define ADDR_DEFAULT 		 0xE0

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_GY-US42)

struct gy-us42_dev_s
{
	FAR struct i2c_master_s *i2c; /* I2C interface */
	uint8_t addr;                 /* GY-US42 I2C address */
	int freq;                     /* GY-US42 Frequency */
};

/****************************************************************************
 * Name: gy-us42_change_addr
 *
 * Description:
 *   Change GY_US42 sensor address from default value of 224.
 *
 * Input Parameters:
 *   priv     - A descriptor
 *   new_addr - The new address. The sensor will only accept even address values,
 *   			except for 0, 80, 64, 170
 *
 ****************************************************************************/

static void gy-us42_change_addr(FAR struct gy-us42_dev_s *priv, struct i2c_config_s config, uint8_t new_addr)
{
	int ret;
	uint8_t regaddr = config.addr;
	uint8_t unlock_1 = ADDR_UNLOCK_1;
	uint8_t unlock_2 = ADDR_UNLOCK_2;

	ret = i2c_write(priv->i2c, &config, &regaddr, 1);
	ret = i2c_write(priv->i2c, &config, &unlock_1, 1);
	ret = i2c_write(priv->i2c, &config, &unlock_2, 1);
	ret = i2c_write(priv->i2c, &config, &new_addr, 1);
}

/****************************************************************************
 * Name: gy-us42_getreg16
 *
 * Description:
 *   Read two 8-bit from a GY-US42 register
 *
 ****************************************************************************/

static uint16_t gy-us42_getreg16(FAR struct gy-us42_dev_s *priv, uint8_t regaddr)
{
	struct i2c_config_s config;
	uint16_t msb, lsb;
	uint16_t regval = 0;
	int ret;

	/* Set up the I2C configuration */

	config.frequency = priv->freq;
	config.address   = priv->addr;
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

	msb = (regval & 0xFF);
	lsb = (regval & 0xFF00) >> 8;

	regval = (msb << 8) | lsb;

	return regval;
}

/****************************************************************************
 * Name: gy-us42_open
 *
 * Description:
 *   This function is called whenever the GY-US42 device is opened.
 *
 ****************************************************************************/

static int gy-us42_open(FAR struct file *filep)
{
	return OK;
}

/****************************************************************************
 * Name: gy-us42_close
 *
 * Description:
 *   This routine is called when the GY-US42 device is closed.
 *
 ****************************************************************************/

static int gy-us42_close(FAR struct file *filep)
{
	return OK;
}

/****************************************************************************
 * Name: gy-us42_register
 *
 * Description:
 *   Register the GY-US42 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             GY-US42
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gy-us42_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
	priv = (FAR struct gy-us42_dev_s *)kmm_malloc(sizeof(struct gy-us42_dev_s));
	if (!priv) {
	      snerr("ERROR: Failed to allocate instance\n");
	      return -ENOMEM;
	}

	priv->i2c = i2c;
	priv->addr = GY-US42_ADDR;
    priv->freq = GY-US42_FREQ;

	/* Check Device ID */

	ret = gy-us42_checkid(priv);
	if (ret < 0)
	{
	  snerr("ERROR: Failed to register driver: %d\n", ret);
	  kmm_free(priv);
	  return ret;
	}

	/* Register the character driver */

	ret = register_driver(devpath, &g_gy-us42fops, 0666, priv);
	if (ret < 0)
	{
	  snerr("ERROR: Failed to register driver: %d\n", ret);
	  kmm_free(priv);
	}

	sninfo("GY-US42 driver loaded successfully!\n");
	return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_GY-US42 */

