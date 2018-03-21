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

#define GY_US42_ADDR 		 0x77
#define GY_US42_FREQ         100000
#define ADDR_UNLOCK_1	     0xAA
#define ADDR_UNLOCK_2		 0xA5
#define CONFIG_SENSORS_GY_US42   // перенести в конфиг потом

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_GY_US42)

struct gy_us42_dev_s
{
	FAR struct i2c_master_s *i2c;  /* I2C interface */
	struct i2c_config_s *config;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static uint16_t gy_us42_getreg8(FAR struct gy_us42_dev_s *priv, uint8_t regaddr);
static uint8_t gy_us42_getreg16(FAR struct gy_us42_dev_s *priv, uint8_t regaddr);

/* Character driver methods */

static int gy_us42_open(FAR struct file *filep);
static int gy_us42_close(FAR struct file *filep);

static const struct file_operations g_gy_us42fops =
{
  gy_us42_open,                  /* open */
  gy_us42_close,                 /* close */
  0,                            /* seek */
  0,                            /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                            /* poll */
#endif
  0                             /* unlink */
};

/****************************************************************************
 * Name: gy_us42_change_addr
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

/*static void gy_us42_change_addr(FAR struct gy_us42_dev_s *priv, uint8_t new_addr)
{
	struct i2c_msg_s msg;
	uint8_t unlock1 = ADDR_UNLOCK_1;
	uint8_t unlock2 = ADDR_UNLOCK_2;
	uint8_t commands[3] = {unlock1, unlock2, new_addr};
	int ret;

	// Setup 8-bit GY_US42 address write message

	msg.frequency = priv->config->frequency;  // I2C frequency
	msg.addr      = priv->config->address;    // 7-bit address
	msg.flags     = 0;                        // Write transaction, beginning with START
	msg.buffer    = commands;                 // Transfer from this address
	msg.length    = 3;                        // Send one byte following the address
												// (no STOP)
	// Perform the transfer

	ret = I2C_TRANSFER(priv->i2c, msg, 2);
	if (ret < 0)
	{
		snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
	}

}*/

/****************************************************************************
 * Name: gy_us42_getreg16
 *
 * Description:
 *   Read two 8-bit from a GY_US42 register
 *
 ****************************************************************************/

uint8_t gy_us42_getreg16(FAR struct gy_us42_dev_s *priv, uint8_t regaddr)
{
    struct i2c_msg_s msg[2];
    uint8_t regval;
    int ret;

    /* Setup 8-bit GY_US42 address write message */

    msg[0].frequency = priv->config->frequency;  /* I2C frequency */
    msg[0].addr      = priv->config->address;    /* 7-bit address */
    msg[0].flags     = 0;                        /* Write transaction, beginning with START */
    msg[0].buffer    = &regaddr;                 /* Transfer from this address */
    msg[0].length    = 1;                        /* Send one byte following the address
                                                * (no STOP) */

    /* Set up the 8-bit GY_US42 data read message */

    msg[1].frequency = priv->config->frequency;  /* I2C frequency */
    msg[1].addr      = priv->config->address;    /* 7-bit address */
    msg[1].flags     = I2C_M_READ;               /* Read transaction, beginning with Re-START */
    msg[1].buffer    = &regval;                  /* Transfer to this address */
    msg[1].length    = 2;                        /* Receive two bytes following the address
                                                  * (then STOP) */

    /* Perform the transfer */

    ret = I2C_TRANSFER(priv->i2c, msg, 2);
    if (ret < 0)
    {
        snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
        return 0;
    }
    return regval;
}

/****************************************************************************
 * Name: gy_us42_getreg8
 *
 * Description:
 *   Read 8-bit from a GY_US42 register
 *
 ****************************************************************************/

uint8_t gy_us42_getreg8(FAR struct gy_us42_dev_s *priv, uint8_t regaddr) {
	struct i2c_msg_s msg[2];
	uint8_t regval;
	int ret;

	/* Setup 8-bit GY_US42 address write message */

	msg[0].frequency = priv->config->frequency;  /* I2C frequency */
	msg[0].addr      = priv->config->address;    /* 7-bit address */
	msg[0].flags     = 0;                        /* Write transaction, beginning with START */
	msg[0].buffer    = &regaddr;                 /* Transfer from this address */
	msg[0].length    = 1;                        /* Send one byte following the address
	                                                * (no STOP) */

	/* Set up the 8-bit GY_US42 data read message */

	msg[1].frequency = priv->config->frequency;  /* I2C frequency */
	msg[1].addr      = priv->config->address;    /* 7-bit address */
	msg[1].flags     = I2C_M_READ;               /* Read transaction, beginning with Re-START */
	msg[1].buffer    = &regval;                  /* Transfer to this address */
	msg[1].length    = 2;                        /* Receive two bytes following the address
	                                                  * (then STOP) */

	/* Perform the transfer */

	ret = I2C_TRANSFER(priv->i2c, msg, 2);
	if (ret < 0)
	{
	    snerr("ERROR: I2C_TRANSFER failed: %d\n", ret);
	    return 0;
	}
	return regval;
}

/****************************************************************************
 * Name: gy_us42_open
 *
 * Description:
 *   This function is called whenever the GY_US42 device is opened.
 *
 ****************************************************************************/

static int gy_us42_open(FAR struct file *filep)
{
	return OK;
}

/****************************************************************************
 * Name: gy_us42_close
 *
 * Description:
 *   This routine is called when the GY_US42 device is closed.
 *
 ****************************************************************************/

static int gy_us42_close(FAR struct file *filep)
{
	return OK;
}

/****************************************************************************
 * Name: gy_us42_register
 *
 * Description:
 *   Register the GY_US42 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/press0"
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             GY_US42
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gy_us42_register(FAR const char *devpath, FAR struct i2c_master_s *i2c)
{
	  FAR struct gy_us42_dev_s *priv;
	  int ret;

	  /* Initialize the GY_US42 device structure */

	  priv = (FAR struct gy_us42_dev_s *)kmm_malloc(sizeof(struct gy_us42_dev_s));
	  if (!priv)
	    {
	      snerr("ERROR: Failed to allocate instance\n");
	      return -1;
	    }

	  priv->i2c = i2c;
	  priv->config->address = GY_US42_ADDR;
	  priv->config->frequency = GY_US42_FREQ;

	  /* Register the character driver */

	  ret = register_driver(devpath, &g_gy_us42fops, 0666, priv);
	  if (ret < 0)
	  {
	      snerr("ERROR: Failed to register driver: %d\n", ret);
	      kmm_free(priv);
	  }

	  sninfo("GY_US42 driver loaded successfully!\n");
	  return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_GY_US42 */

