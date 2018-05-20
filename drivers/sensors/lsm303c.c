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
#include <time.h>
#include <errno.h>
#include <debug.h>
#include <string.h>
#include <nuttx/wqueue.h>

#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lsm303c.h>
#include <nuttx/random.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LSM303C)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LSM303C_DEVID			0x3D

#define LSM303C_I2C_ADDR		0x1E

#define LSM303C_WHO_AM_I_M		0x0F
#define LSM303C_CTRL_REG_1_M	0x20
#define LSM303C_CTRL_REG_2_M	0x21
#define LSM303C_CTRL_REG_3_M	0x22
#define LSM303C_CTRL_REG_4_M	0x23
#define LSM303C_CTRL_REG_5_M	0x24
#define LSM303C_STATUS_REG_M	0x27
#define LSM303C_OUT_X_L_M		0x28
#define LSM303C_OUT_X_H_M		0x29
#define LSM303C_OUT_Y_L_M		0x2A
#define LSM303C_OUT_Y_H_M		0x2B
#define LSM303C_OUT_Z_L_M		0x2C
#define LSM303C_OUT_Z_H_M		0x2D
#define LSM303C_TEMP_L_M		0x2E
#define LSM303C_TEMP_H_M		0x2F
#define LSM303C_INT_CFG_M		0x30
#define LSM303C_INT_SRC_M		0x31
#define LSM303C_INT_THS_L_M		0x32
#define LSM303C_INT_THS_H_M		0x33

#define BYTESWAP(WORD)	( (WORD << 8 | WORD >> 8) & 0xFFFF )

#define TEMP_EN 	7
#define OM_XY 		5
#define OM_Z		2
#define DO			2
#define MD			0

/****************************************************************************
 * devate Type Definitions
 ****************************************************************************/
//TODO дескриптор
typedef struct {
	FAR struct i2c_master_s *i2c; 		/* I2C interface */
	int devnum;						/* Number of /dev/mpuN */
	sem_t sem;
} lsm303c_t;

/****************************************************************************
 * devate Function Prototypes
 ****************************************************************************/

static int16_t _getreg8(FAR lsm303c_t *dev, uint8_t regaddr);
static int _getregmany(FAR lsm303c_t *dev, uint8_t regaddr, size_t count, FAR void * buffer);
static uint16_t _putreg8(FAR lsm303c_t *dev, uint8_t regaddr, uint8_t regval);

/* Character driver methods */

static int _open(FAR struct file *filep);
static int _close(FAR struct file *filep);
static ssize_t _read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t _write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int _ioctl(FAR struct file* filep, int cmd, unsigned long arg);


/****************************************************************************
 * devate Data
 ****************************************************************************/

static const struct file_operations g_lsm303cfops =
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
 * devate Functions
 ****************************************************************************/

/****************************************************************************
 * Name: _getreg8
 *
 * Description:
 *   Read from an 8-bit MPU6000 register
 *
 ****************************************************************************/

static int16_t _getreg8(FAR lsm303c_t *dev, uint8_t regaddr)
{
	struct i2c_msg_s msg[2];
	uint8_t buffer;

	msg[0].addr = LSM303C_I2C_ADDR;
	msg[0].buffer = &regaddr;
	msg[0].flags = 0;
	msg[0].frequency = LSM303C_I2C_FREQ;
	msg[0].length = 1;

	msg[1].addr = LSM303C_I2C_ADDR;
	msg[1].buffer = &buffer;
	msg[1].flags = I2C_M_READ;
	msg[1].frequency = LSM303C_I2C_FREQ;
	msg[1].length = 1;

	int retval = I2C_TRANSFER(dev->i2c, msg, 2);

	return retval >= 0? buffer:retval;
}

/****************************************************************************
 * Name: _getregmany
 *
 * Description:
 *   Read many 8-bit from a MPU6000 register
 *
 ****************************************************************************/
static int _getregmany(FAR lsm303c_t *dev, uint8_t regaddr, size_t count, FAR void * buffer)
{
	struct i2c_msg_s msg[2];

	msg[0].addr = LSM303C_I2C_ADDR;
	msg[0].buffer = &regaddr;
	msg[0].flags = 0;
	msg[0].frequency = LSM303C_I2C_FREQ;
	msg[0].length = 1;

	msg[1].addr = LSM303C_I2C_ADDR;
	msg[1].buffer = (uint8_t *)buffer;
	msg[1].flags = I2C_M_READ;
	msg[1].frequency = LSM303C_I2C_FREQ;
	msg[1].length = count;

	return I2C_TRANSFER(dev->i2c, msg, 2);
}

/****************************************************************************
 * Name: _putreg8
 *
 * Description:
 *   Write to an 8-bit MPU6000 register
 *
 ****************************************************************************/
static uint16_t _putreg8(FAR lsm303c_t *dev, uint8_t regaddr, uint8_t regval)
{
	struct i2c_msg_s msg[2];

	msg[0].addr = LSM303C_I2C_ADDR;
	msg[0].buffer = &regaddr;
	msg[0].flags = 0;
	msg[0].frequency = LSM303C_I2C_FREQ;
	msg[0].length = 1;

	msg[1].addr = LSM303C_I2C_ADDR;
	msg[1].buffer = &regval;
	msg[1].flags = I2C_M_NORESTART;
	msg[1].frequency = LSM303C_I2C_FREQ;
	msg[1].length = 1;

	return I2C_TRANSFER(dev->i2c, msg, 2);
}

static void _convert(lsm303c_t * dev, lsm303c_record_raw_t raw, lsm303c_record_t * record) {
	record->field.x = raw.field.x * 0.58 / 1000.0;
	record->field.y = raw.field.y * 0.58 / 1000.0;
	record->field.z = raw.field.z * 0.58 / 1000.0;
	record->temperature = raw.temperature / 8.0;
	record->time = raw.time;
}

/****************************************************************************
 * Name: _checkid
 *
 * Description:
 *   Read and verify the MPU6000 DEVID
 *
 ****************************************************************************/

static int _checkid(FAR lsm303c_t *dev)
{
  uint8_t devid = 0;

  /* Read device ID */

  devid = _getreg8(dev, LSM303C_WHO_AM_I_M);
  sninfo("devid: 0x%02x\n", devid);

  if (devid != (uint16_t)LSM303C_DEVID)
    {
      /* ID is not Correct */

      snerr("ERROR: Wrong Device ID!\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: _set_temperature_enabled
 *
 * Description:
 *   Enables/disables temperature sensor
 *
 ****************************************************************************/
static int _set_temperature_enabled(lsm303c_t * dev, bool enable) {
	int16_t reg = _getreg8(dev, LSM303C_CTRL_REG_1_M);
	if(reg < 0) return reg;

	_putreg8(dev, LSM303C_CTRL_REG_1_M, reg | (enable << TEMP_EN));
	return OK;
}

/****************************************************************************
 * Name: _set_operative_mode_XY
 *
 * Description:
 *   Sets operative mode of X and Y axes
 *
 ****************************************************************************/
static int _set_operative_mode_XY(lsm303c_t * dev, lsm303c_setting_operative_mode_t mode) {
	int16_t reg = _getreg8(dev, LSM303C_CTRL_REG_1_M);
	if(reg < 0) return reg;

	_putreg8(dev, LSM303C_CTRL_REG_1_M, reg | (mode << OM_XY));
	return OK;
}

/****************************************************************************
 * Name: _set_operative_mode_Z
 *
 * Description:
 *   Sets operative mode of Z axis
 *
 ****************************************************************************/
static int _set_operative_mode_Z(lsm303c_t * dev, lsm303c_setting_operative_mode_t mode) {
	int16_t reg = _getreg8(dev, LSM303C_CTRL_REG_4_M);
	if(reg < 0) return reg;

	_putreg8(dev, LSM303C_CTRL_REG_4_M, reg | (mode << OM_Z));
	return OK;
}

/****************************************************************************
 * Name: _set_odr
 *
 * Description:
 *   Sets output datarate
 *
 ****************************************************************************/
static int _set_odr(lsm303c_t * dev, lsm303c_setting_odr_t datarate) {
	int16_t reg = _getreg8(dev, LSM303C_CTRL_REG_1_M);
	if(reg < 0) return reg;

	_putreg8(dev, LSM303C_CTRL_REG_1_M, reg | (datarate << DO));
	return OK;
}

/****************************************************************************
 * Name: _set_mode
 *
 * Description:
 *   Sets measurement mode
 *
 ****************************************************************************/
static int _set_mode(lsm303c_t * dev, lsm303c_setting_mode_t mode) {
	int16_t reg = _getreg8(dev, LSM303C_CTRL_REG_3_M);
	if(reg < 0) return reg;

	_putreg8(dev, LSM303C_CTRL_REG_3_M, reg | (mode << MD));
	return OK;
}


/****************************************************************************
 * Name: _open
 *
 * Description:
 *   This routine is called whenever the MPU6000 device is opened.
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
 *   This routine is called when the MPU6000 device is closed.
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
 *   This routine handles read from MPU6000 device.
 *
 ****************************************************************************/

static ssize_t _read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
	FAR volatile struct inode        *inode = filep->f_inode;
	FAR lsm303c_t *dev  = inode->i_private;

	(void)nxsem_wait(&dev->sem);

	int ret = OK;

	if (!buffer) {
		snerr("ERROR: Buffer is null\n");
		ret = -EINVAL;
	}

	else if( buflen != sizeof(lsm303c_record_raw_t) && buflen != sizeof(lsm303c_record_t) ) {
		snerr("ERROR: You can't read something other than raw data (16 bytes) or converted data (24 bytes)\n");
		ret = -EINVAL;
	}

	else {
		lsm303c_record_raw_t * record_raw = (lsm303c_record_raw_t *)buffer;

		_getregmany(dev, LSM303C_OUT_X_L_M, sizeof(int16_t) * 4, record_raw);
		clock_gettime(CLOCK_MONOTONIC, &(record_raw->time) );

		if(buflen == sizeof(lsm303c_record_t)) {
			_convert(dev, *record_raw, buffer);
		}
	}

	nxsem_post(&dev->sem);
	return ret;
}

/****************************************************************************
 * Name: _write
 ****************************************************************************/

static ssize_t _write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: _ioctl
 *
 * Description:
 *   This routine handles ioctl commands.
 *
 ****************************************************************************/
static int _ioctl(FAR struct file* filep, int cmd, unsigned long arg) {

	FAR struct inode        *inode = filep->f_inode;
	FAR lsm303c_t 			*dev  = inode->i_private;

	(void)nxsem_wait(&dev->sem);

	//TODO ассерты что ль поставить
	switch(cmd) {

	case LSM303C_CMD_SET_TEMPERATURE_ENABLED:
			_set_temperature_enabled(dev, *(bool *)arg);
			break;

	case LSM303C_CMD_SET_OPERATIVE_MODE_XY:
			_set_operative_mode_XY(dev, *(lsm303c_setting_operative_mode_t *)arg);
			break;

	case LSM303C_CMD_SET_OPERATIVE_MODE_Z:
			_set_operative_mode_Z(dev, *(lsm303c_setting_operative_mode_t *)arg);
			break;

	case LSM303C_CMD_SET_ODR:
			_set_odr(dev, *(lsm303c_setting_odr_t *)arg);
			break;

	case LSM303C_CMD_SET_MODE:
			_set_mode(dev, *(lsm303c_setting_mode_t *)arg);
			break;

	default:
		nxsem_post(&dev->sem);
		return -EINVAL;
	}

	nxsem_post(&dev->sem);
	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
int lsm303c_register(FAR struct i2c_master_s *i2c, int minor)
{
	FAR lsm303c_t *dev;
	int ret;

	/* Initialize the MPU6000 device structure */

	dev = (FAR lsm303c_t *)kmm_malloc(sizeof(lsm303c_t));
	if (!dev) {
		snerr("ERROR: Failed to allocate mpu6000 instance\n");
		return -ENOMEM;
	}

	dev->i2c = i2c;
	dev->devnum = minor;
	nxsem_init(&(dev->sem), 0, 0);

	/* Check Device ID */
	ret = _checkid(dev);
	if (ret < 0) {
		snerr("ERROR: Failed to register driver: %d\n", ret);
		kmm_free(dev);
		return ret;
	}

	_putreg8(dev, LSM303C_CTRL_REG_1_M, 3); //reset sensor

	/* Fill format string */
	char devname[LSM303C_DEV_NAMELEN];
	snprintf(devname, LSM303C_DEV_NAMELEN, LSM303C_DEV_FORMAT, minor);

	/* Register the character driver */
	ret = register_driver(devname, &g_lsm303cfops, 0666, dev);
	if (ret < 0) {
		snerr("ERROR: Failed to register driver: %d\n", ret);
		kmm_free(dev);
		return ret;
	}

	nxsem_post(&dev->sem);
	return ret;
}

#endif /* CONFIG_I2C && CONFIG_SENSORS_LSM303C */
