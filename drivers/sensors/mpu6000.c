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
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/mpu6000.h>
#include <nuttx/random.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MPU6000)

#ifndef MPU6000_FIFO_LEN
#define MPU6000_FIFO_LEN (sizeof(mpu6000_record_t))
#endif

#define MPU6000_RECORD_LEN ( dev->convert? sizeof(mpu6000_record_t) : sizeof(mpu6000_record_raw_t) )

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MPU6000_DEVID               0x68

#define MPU6000_SELF_TEST_X			0x0D
#define MPU6000_SELF_TEST_Y			0x0E
#define MPU6000_SELF_TEST_Z			0x0F
#define MPU6000_SELF_TEST_A			0x10
#define MPU6000_SMPLRT_DIV			0x19
#define MPU6000_CONFIG				0x1A
#define MPU6000_GYRO_CONFIG			0x1B
#define MPU6000_ACCEL_CONFIG		0x1C
#define MPU6000_FIFO_EN				0x23
/*#define MPU6000_I2C_MST_CTRL		0x24
#define MPU6000_I2C_SLV0_ADDR		0x25
#define MPU6000_I2C_SLV0_REG		0x26
#define MPU6000_I2C_SLV0_CTRL		0x27
#define MPU6000_I2C_SLV1_ADDR		0x28
#define MPU6000_I2C_SLV1_REG		0x29
#define MPU6000_I2C_SLV1_CTRL		0x2A
#define MPU6000_I2C_SLV2_ADDR		0x2B
#define MPU6000_I2C_SLV2_REG		0x2C
#define MPU6000_I2C_SLV2_CTRL		0x2D
#define MPU6000_I2C_SLV3_ADDR		0x2E
#define MPU6000_I2C_SLV3_REG		0x2F
#define MPU6000_I2C_SLV3_CTRL		0x30
#define MPU6000_I2C_SLV4_ADDR		0x31
#define MPU6000_I2C_SLV4_REG		0x32
#define MPU6000_I2C_SLV4_DO			0x33
#define MPU6000_I2C_SLV4_CTRL		0x34
#define MPU6000_I2C_SLV4_DI			0x35
#define MPU6000_I2C_MST_STATUS		0x36*/
#define MPU6000_INT_PIN_CFG			0x37
#define MPU6000_INT_ENABLE			0x38
#define MPU6000_INT_STATUS			0x3A
#define MPU6000_ACCEL_XOUT_H		0x3B
#define MPU6000_ACCEL_XOUT_L		0x3C
#define MPU6000_ACCEL_YOUT_H		0x3D
#define MPU6000_ACCEL_YOUT_L		0x3E
#define MPU6000_ACCEL_ZOUT_H		0x3F
#define MPU6000_ACCEL_ZOUT_L		0x40
#define MPU6000_TEMP_OUT_H			0x41
#define MPU6000_TEMP_OUT_L			0x42
#define MPU6000_GYRO_XOUT_H			0x43
#define MPU6000_GYRO_XOUT_L			0x44
#define MPU6000_GYRO_YOUT_H			0x45
#define MPU6000_GYRO_YOUT_L			0x46
#define MPU6000_GYRO_ZOUT_H			0x47
#define MPU6000_GYRO_ZOUT_L			0x48
/*#define MPU6000_EXT_SENS_DATA_00	0x49
#define MPU6000_EXT_SENS_DATA_01	0x4A
#define MPU6000_EXT_SENS_DATA_02	0x4B
#define MPU6000_EXT_SENS_DATA_03	0x4C
#define MPU6000_EXT_SENS_DATA_04	0x4D
#define MPU6000_EXT_SENS_DATA_05	0x4E
#define MPU6000_EXT_SENS_DATA_06	0x4F
#define MPU6000_EXT_SENS_DATA_07	0x50
#define MPU6000_EXT_SENS_DATA_08	0x51
#define MPU6000_EXT_SENS_DATA_09	0x52
#define MPU6000_EXT_SENS_DATA_10	0x53
#define MPU6000_EXT_SENS_DATA_11	0x54
#define MPU6000_EXT_SENS_DATA_12	0x55
#define MPU6000_EXT_SENS_DATA_13	0x56
#define MPU6000_EXT_SENS_DATA_14	0x57
#define MPU6000_EXT_SENS_DATA_15	0x58
#define MPU6000_EXT_SENS_DATA_16	0x59
#define MPU6000_EXT_SENS_DATA_17	0x5A
#define MPU6000_EXT_SENS_DATA_18	0x5B
#define MPU6000_EXT_SENS_DATA_19	0x5C
#define MPU6000_EXT_SENS_DATA_20	0x5D
#define MPU6000_EXT_SENS_DATA_21	0x5E
#define MPU6000_EXT_SENS_DATA_22	0x5F
#define MPU6000_EXT_SENS_DATA_23	0x60
#define MPU6000_I2C_SLV0_DO			0x63
#define MPU6000_I2C_SLV1_DO			0x64
#define MPU6000_I2C_SLV2_DO			0x65
#define MPU6000_I2C_SLV3_DO			0x66
#define MPU6000_I2C_MST_DELAY_CTRL	0x67*/
#define MPU6000_SIGNAL_PATH_RESET	0x68
#define MPU6000_USER_CTRL			0x6A
#define MPU6000_PWR_MGMT_1			0x6B
#define MPU6000_PWR_MGMT_2			0x6C
/*#define MPU6000_FIFO_COUNTH			0x72
#define MPU6000_FIFO_COUNTL			0x73
#define MPU6000_FIFO_R_W			0x74*/
#define MPU6000_WHO_AM_I			0x75


#define READADDR(ADDR)		( ADDR | (1 << 7) )
#define WRITEADDR(ADDR)		( ADDR & ~(1 << 7))

#define BYTESWAP(WORD)	( (WORD << 8 | WORD >> 8) & 0xFFFF )

#define FORM_USER_CTRL(I2C_IF_DIS, SIG_COND_RESET) ( (I2C_IF_DIS << 4) | (SIG_COND_RESET & 0x01) )
#define FORM_INT_ENABLE(DATA_RDY_EN) ( (DATA_RDY_EN & 0x01) )
#define FORM_INT_PIN_CFG(LATCH_INT_EN,INT_RD_CLEAR) ( (LATCH_INT_EN << 5) | (INT_RD_CLEAR << 4) )
#define FORM_GYRO_CONFIG(FS_SEL) ( (FS_SEL << 3) & 0x18 )
#define FORM_ACCEL_CONFIG(AFS_SEL) ( (AFS_SEL << 3) & 0x18 )
#define FORM_CONFIG(DLPF) ( (DLPF) & 0x07 )
#define FORM_PWR_MGMT_1(CLKSEL) (CLKSEL & 0x07)

/****************************************************************************
 * devate Type Definitions
 ****************************************************************************/
//TODO дескриптор
typedef struct {
	FAR struct spi_dev_s *spi; 		/* SPI interface */
	int devnum;						/* Number of /dev/mpuN */
	sem_t sem;

	mpu6000_setting_acc_fullscale_t acc_fullscale;
	mpu6000_setting_gyro_fullscale_t gyro_fullscale;
	bool convert;

	struct mpu6000_fifo_s {
		FAR uint8_t * addr;
		uint16_t head, tail, length;
		sem_t sem;
	} fifo;
	struct work_s irq_work;
} mpu6000_t;

/****************************************************************************
 * devate Function Prototypes
 ****************************************************************************/

static inline void _configspi(FAR struct spi_dev_s *spi);
static uint8_t _getreg8(FAR mpu6000_t *dev, uint8_t regaddr);
static uint16_t _getreg16(FAR mpu6000_t *dev, uint8_t regaddr);
static void _getregmany(FAR mpu6000_t *dev, uint8_t regaddr, size_t count, FAR void * buffer);
static void _putreg8(FAR mpu6000_t *dev, uint8_t regaddr, uint8_t regval);

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

static const struct file_operations g_mpu6000fops =
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
 * Name: _configspi
 *
 * Description: configures SPI settings to accomply with MPU6000 requirements
 *
 ****************************************************************************/

static inline void _configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the MPU6000 */

  SPI_SETMODE(spi, SPIDEV_MODE3);
  SPI_SETBITS(spi, 8);
  (void)SPI_HWFEATURES(spi, 0);
  (void)SPI_SETFREQUENCY(spi, MPU6000_SPI_FREQ);
}

/****************************************************************************
 * Name: _getreg8
 *
 * Description:
 *   Read from an 8-bit MPU6000 register
 *
 ****************************************************************************/

static uint8_t _getreg8(FAR mpu6000_t *dev, uint8_t regaddr)
{
  uint8_t regval;

  /* If SPI bus is shared then lock and configure it */

  (void)SPI_LOCK(dev->spi, true);
  _configspi(dev->spi);

  /* Select the MPU6000 */

  SPI_SELECT(dev->spi, SPIDEV_ACCELEROMETER(dev->devnum), true);

  /* Send register to read and get the next byte */

  (void)SPI_SEND(dev->spi, READADDR( regaddr ));
  SPI_RECVBLOCK(dev->spi, &regval, 1);

  /* Deselect the MPU6000 */

  SPI_SELECT(dev->spi, SPIDEV_ACCELEROMETER(dev->devnum), false);

  /* Unlock bus */

  (void)SPI_LOCK(dev->spi, false);

  return regval;
}

/****************************************************************************
 * Name: _getreg16
 *
 * Description:
 *   Read two 8-bit from a MPU6000 register
 *
 ****************************************************************************/
static uint16_t _getreg16(FAR mpu6000_t *dev, uint8_t regaddr)
{
  uint16_t regval;

  /* If SPI bus is shared then lock and configure it */

  (void)SPI_LOCK(dev->spi, true);
  _configspi(dev->spi);

  /* Select the MPU6000 */

  SPI_SELECT(dev->spi, SPIDEV_ACCELEROMETER(dev->devnum), true);

  /* Send register to read and get the next 2 bytes */

  (void)SPI_SEND(dev->spi, READADDR( regaddr ));
  SPI_RECVBLOCK(dev->spi, &regval, 2);

  /* Deselect the MPU6000 */

  SPI_SELECT(dev->spi, SPIDEV_ACCELEROMETER(dev->devnum), false);

  /* Unlock bus */

  (void)SPI_LOCK(dev->spi, false);

  return regval;
}

/****************************************************************************
 * Name: _getregmany
 *
 * Description:
 *   Read many 8-bit from a MPU6000 register
 *
 ****************************************************************************/
static void _getregmany(FAR mpu6000_t *dev, uint8_t regaddr, size_t count, FAR void * buffer)
{
  /* If SPI bus is shared then lock and configure it */

  (void)SPI_LOCK(dev->spi, true);
  _configspi(dev->spi);

  /* Select the MPU6000 */

  SPI_SELECT(dev->spi, SPIDEV_ACCELEROMETER(dev->devnum), true);

  /* Send register to read and get bytes */

  (void)SPI_SEND(dev->spi, READADDR( regaddr ));
  SPI_RECVBLOCK(dev->spi, buffer, count);

  /* Deselect the MPU6000 */

  SPI_SELECT(dev->spi, SPIDEV_ACCELEROMETER(dev->devnum), false);

  /* Unlock bus */

  (void)SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: _putreg8
 *
 * Description:
 *   Write to an 8-bit MPU6000 register
 *
 ****************************************************************************/
static void _putreg8(FAR mpu6000_t *dev, uint8_t regaddr, uint8_t regval)
{
  /* If SPI bus is shared then lock and configure it */

  (void)SPI_LOCK(dev->spi, true);
  _configspi(dev->spi);

  /* Select the MPU6000 */

  SPI_SELECT(dev->spi, SPIDEV_ACCELEROMETER(dev->devnum), true);

  /* Send register address and set the value */

  (void)SPI_SEND(dev->spi, WRITEADDR( regaddr ));
  (void)SPI_SEND(dev->spi, regval);

  /* Deselect the MPU6000 */

  SPI_SELECT(dev->spi, SPIDEV_ACCELEROMETER(dev->devnum), false);

  /* Unlock bus */

  (void)SPI_LOCK(dev->spi, false);
}

/****************************************************************************
 * Name: _getfifo
 *
 * Description:
 *   Get size bytes from fifo
 *
 ****************************************************************************/
static ssize_t _getfifo(mpu6000_t * dev, FAR void * buffer, size_t size) {
	struct mpu6000_fifo_s * fifo = &( dev->fifo );
	uint8_t * ubuffer = (uint8_t *) buffer;

	(void)nxsem_wait(&dev->fifo.sem);

	size_t barrier = (fifo->head > fifo->tail) || (fifo->tail == 0) ? fifo->length :(MPU6000_FIFO_LEN - fifo->tail);
	size_t firstread = size < barrier? size : barrier;

	memcpy(ubuffer, fifo->addr + fifo->tail, firstread);

	fifo->tail = ( fifo->tail + firstread ) % MPU6000_FIFO_LEN;
	fifo->length -= firstread;

	size_t secondread = (size - firstread) <= fifo->length ? (size - firstread):fifo->length;

	memcpy(ubuffer + firstread, fifo->addr + fifo->tail, secondread);
	fifo->tail += secondread;
	fifo->length -= secondread;

	size_t readed = firstread + secondread;

	size_t todiscard = ( MPU6000_RECORD_LEN - (readed % MPU6000_RECORD_LEN) ) %  MPU6000_RECORD_LEN;
	fifo->tail = (fifo->tail +  todiscard) % MPU6000_FIFO_LEN;
	fifo->length -= todiscard;

	nxsem_post(&dev->fifo.sem);

	return readed;
}

/****************************************************************************
 * Name: _putfifo
 *
 * Description:
 *   Put size bytes into fifo
 *
 ****************************************************************************/
static ssize_t _putfifo(mpu6000_t * dev, FAR void * buffer, size_t size) {
	struct mpu6000_fifo_s * fifo = &( dev->fifo );
	uint8_t * ubuffer = (uint8_t *) buffer;

	if(size > MPU6000_FIFO_LEN) return -ENOSPC;

	(void)nxsem_wait(&dev->fifo.sem);

	size = size - (size % MPU6000_RECORD_LEN); //cut to record size

	while(size > (MPU6000_FIFO_LEN - fifo->length)) {
		fifo->tail += MPU6000_RECORD_LEN;
		fifo->tail %= MPU6000_FIFO_LEN;
		fifo->length -= MPU6000_RECORD_LEN;
	}

	size_t firstwrite = size < (MPU6000_FIFO_LEN - fifo->head) ? size : (MPU6000_FIFO_LEN - fifo->head);

	memcpy(fifo->addr + fifo->head, ubuffer, firstwrite);

	fifo->head = ( fifo->head + firstwrite ) % MPU6000_FIFO_LEN;
	fifo->length += firstwrite;

	size_t secondwrite = size - firstwrite;

	memcpy(fifo->addr + fifo->head, ubuffer + firstwrite, secondwrite);

	fifo->head += secondwrite;
	fifo->length += secondwrite;

	nxsem_post(&dev->fifo.sem);

	return size;
}

static void _flushfifo(mpu6000_t * dev) {
	struct mpu6000_fifo_s * fifo = &(dev->fifo);

	(void)nxsem_wait(&fifo->sem);

	fifo->head = 0;
	fifo->tail = 0;
	fifo->length = 0;

	nxsem_post(&fifo->sem);
}

static void _convert(mpu6000_t * dev, mpu6000_record_raw_t * raw,mpu6000_record_t * record) {
	float acc_coeff, gyro_coeff;

	switch(dev->acc_fullscale) {
	default:
	case MPU6000_SETTING_ACC_FULLSCALE_2G:
		acc_coeff = 16384.0f;
		break;

	case MPU6000_SETTING_ACC_FULLSCALE_4G:
		acc_coeff = 8192.0f;
		break;

	case MPU6000_SETTING_ACC_FULLSCALE_8G:
		acc_coeff = 4096.0f;
		break;

	case MPU6000_SETTING_ACC_FULLSCALE_16G:
		acc_coeff = 2048.0f;
		break;
	}

	switch(dev->gyro_fullscale) {
	default:
	case MPU6000_SETTING_GYRO_FULLSCALE_250DPS:
		gyro_coeff = 131.0f;
		break;

	case MPU6000_SETTING_GYRO_FULLSCALE_500DPS:
		gyro_coeff = 65.5f;
		break;

	case MPU6000_SETTING_GYRO_FULLSCALE_1000DPS:
		gyro_coeff = 32.8f;
		break;

	case MPU6000_SETTING_GYRO_FULLSCALE_2000DPS:
		gyro_coeff = 16.4f;
		break;
	}

	record->acc.x = (float)raw->acc.x / acc_coeff;
	record->acc.y = (float)raw->acc.y / acc_coeff;
	record->acc.z = (float)raw->acc.z / acc_coeff;
	record->temperature = (float)raw->temperature / 340.0f + 36.53f;
	record->gyro.x = (float)raw->gyro.x / gyro_coeff;
	record->gyro.y = (float)raw->gyro.y / gyro_coeff;
	record->gyro.z = (float)raw->gyro.z / gyro_coeff;
	record->time = raw->time;
}

/****************************************************************************
 * Name: _worker
 *
 * Description:
 *  Routine for reading data from MPU6000
 *
 ****************************************************************************/
static void _worker(void * arg) {
	FAR mpu6000_t * dev = (FAR mpu6000_t *) arg;

	uint8_t status = _getreg8(dev, MPU6000_INT_STATUS);
	if(!(status | 0x01)) //check DATA_RDY bit
		return;

	mpu6000_record_raw_t record;

	_getregmany(dev, MPU6000_ACCEL_XOUT_H, sizeof(mpu6000_record_raw_t) - sizeof(struct timespec), &record);

	uint8_t * to_revert = (uint8_t *)&record;
	//Now, it's time to revert byte order!!! FUN!!!
	for(size_t i = 0; i < sizeof(mpu6000_record_raw_t) - sizeof (struct timespec); i+=2) {
		*( (uint16_t *)(to_revert + i) ) = BYTESWAP( *( (uint16_t *)(to_revert + i) ) );
	}

	clock_gettime(CLOCK_REALTIME, &record.time);
	//if(record.gyro.x > 50) printf("%d\n", record.gyro.x);

	if(dev->convert) {
		mpu6000_record_t converted;
		_convert(dev, &record, &converted);
		_putfifo(dev, &converted, MPU6000_RECORD_LEN);
	}

	else _putfifo(dev, &record, MPU6000_RECORD_LEN);
}

static int _irqhandler(int irq, FAR void *context, FAR void *arg) {
	FAR mpu6000_t * dev = (FAR mpu6000_t *)arg;

	work_queue(HPWORK, &dev->irq_work, _worker, dev, 0);

	return OK;
}

/****************************************************************************
 * Name: _checkid
 *
 * Description:
 *   Read and verify the MPU6000 DEVID
 *
 ****************************************************************************/

static int _checkid(FAR mpu6000_t *dev)
{
  uint8_t devid = 0;

  /* Read device ID */

  devid = _getreg8(dev, MPU6000_WHO_AM_I);
  sninfo("devid: 0x%02x\n", devid);

  if (devid != (uint16_t)MPU6000_DEVID)
    {
      /* ID is not Correct */

      snerr("ERROR: Wrong Device ID!\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: _set_acc_fullscale
 *
 * Description:
 *   Set AFS in ACCEL_CONFIG register
 *
 ****************************************************************************/
static int _set_acc_fullscale(mpu6000_t * dev, mpu6000_setting_acc_fullscale_t fullscale) {
	_putreg8(dev, MPU6000_ACCEL_CONFIG, FORM_ACCEL_CONFIG(fullscale) );
	dev->acc_fullscale = fullscale;

	return OK;
}

/****************************************************************************
 * Name: _set_gyro_fullscale
 *
 * Description:
 *   Set GFS in GYRO_CONFIG register
 *
 ****************************************************************************/
static int _set_gyro_fullscale(mpu6000_t * dev, mpu6000_setting_gyro_fullscale_t fullscale) {
	_putreg8(dev, MPU6000_GYRO_CONFIG, FORM_GYRO_CONFIG(fullscale) );
	dev->gyro_fullscale = fullscale;

	return OK;
}

/****************************************************************************
 * Name: _set_filter
 *
 * Description:
 *   Set DLPF_CFG in CONFIG register
 *
 ****************************************************************************/
static int _set_filter(mpu6000_t * dev, uint8_t level) {
	_putreg8(dev, MPU6000_CONFIG, FORM_CONFIG(level) );

	return OK;
}

/****************************************************************************
 * Name: _set_samplerate_divider
 *
 * Description:
 *   Set SMPLRT_DIV register
 *
 ****************************************************************************/
static int _set_samplerate_divider(mpu6000_t * dev, uint8_t divider) {
	_putreg8(dev, MPU6000_SMPLRT_DIV, divider);

	return OK;
}

/****************************************************************************
 * Name: _set_clkmode
 *
 * Description:
 *   Set CLKSEL in PWR_MGMT_1 register
 *
 ****************************************************************************/
static int _set_clkmode(mpu6000_t * dev, mpu6000_setting_clkmode_t clkmode) {
	_putreg8(dev, MPU6000_PWR_MGMT_1, FORM_PWR_MGMT_1(clkmode) );

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
	FAR mpu6000_t *dev  = inode->i_private;

	(void)nxsem_wait(&dev->sem);

	ssize_t ret = _getfifo(dev, buffer, buflen);

	nxsem_post(&dev->sem);
	printf("MPU6000 _read() pointer %p\n", inode->u.i_ops->read);
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
	FAR mpu6000_t 			*dev  = inode->i_private;

	(void)nxsem_wait(&dev->sem);

	//TODO ассерты что ль поставить
	switch(cmd) {

	case MPU6000_CMD_SET_ACC_FULLSCALE:
		_set_acc_fullscale(dev, (mpu6000_setting_acc_fullscale_t) arg); //FIXME передавать как указатели
		break;

	case MPU6000_CMD_SET_GYRO_FULLSCALE:
		_set_gyro_fullscale(dev, (mpu6000_setting_gyro_fullscale_t) arg);
		break;

	case MPU6000_CMD_SET_FILTER:
		_set_filter(dev, (uint8_t) arg);
		break;

	case MPU6000_CMD_SET_SAMPLERATE_DIVIDER:
		_set_samplerate_divider(dev, (uint8_t) arg);
		break;

	case MPU6000_CMD_SET_CLKMODE:
		_set_clkmode(dev, (mpu6000_setting_clkmode_t) arg);
		break;

	case MPU6000_CMD_SET_CONVERT:
		_flushfifo(dev);
		dev->convert = (bool) arg;
		break;

	case MPU6000_CMD_GET_CONVERT:
		*(bool *)arg = dev->convert;
		break;

	case MPU6000_CMD_FLUSHFIFO:
		_flushfifo(dev);
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
int mpu6000_register(FAR struct spi_dev_s *spi, int minor, int (_irqbind)(xcpt_t isr, FAR void *arg))
{
	FAR mpu6000_t *dev;
	int ret;

	/* Initialize the MPU6000 device structure */

	dev = (FAR mpu6000_t *)kmm_malloc(sizeof(mpu6000_t));
	if (!dev) {
		snerr("ERROR: Failed to allocate mpu6000 instance\n");
		return -ENOMEM;
	}

	dev->fifo.addr = (FAR uint8_t *)kmm_malloc(MPU6000_FIFO_LEN);
	if (!dev) {
		snerr("ERROR: Failed to allocate mpu6000 instance\n");
		kmm_free(dev);
		return -ENOMEM;
	}

	dev->spi = spi;
	dev->devnum = minor;
	dev->convert = false;
	memset(&( dev->irq_work ), 0, sizeof( struct work_s) );
	nxsem_init(&(dev->fifo.sem), 0, 1);
	nxsem_init(&(dev->sem), 0, 0);
	_flushfifo(dev);


	/* Check Device ID */
	ret = _checkid(dev);
	if (ret < 0) {
		snerr("ERROR: Failed to register driver: %d\n", ret);
		kmm_free(dev->fifo.addr);
		kmm_free(dev);
		return ret;
	}

	_putreg8(dev, MPU6000_USER_CTRL, FORM_USER_CTRL(false, true)); //reset sensor

	ret = _irqbind(_irqhandler, dev);
	if (ret < 0) {
		snerr("ERROR: Failed to bind irq: %d\n", ret);
		kmm_free(dev->fifo.addr);
		kmm_free(dev);
		nxsem_post(&dev->sem);
		return ret;
	}

	_putreg8(dev, MPU6000_USER_CTRL, FORM_USER_CTRL(true, false)); 	//enabling only SPI

	_set_acc_fullscale(dev, MPU6000_SETTING_ACC_FULLSCALE_16G);
	_set_gyro_fullscale(dev, MPU6000_SETTING_GYRO_FULLSCALE_2000DPS); //maximum scale
	_set_filter(dev, 0); //no filter
	_set_samplerate_divider(dev, 79); //100 Hz sample rate
	_set_clkmode(dev, MPU6000_SETTING_CLKMODE_INTERNAL_8MHz); //internal clock

	_putreg8(dev, MPU6000_INT_ENABLE, FORM_INT_ENABLE(true));	//enabling data ready interrupt source
	_putreg8(dev, MPU6000_INT_PIN_CFG, FORM_INT_PIN_CFG(true, true) );

	/* Fill format string */
	char devname[MPU6000_DEV_NAMELEN];
	snprintf(devname, MPU6000_DEV_NAMELEN, MPU6000_DEV_FORMAT, minor);

	/* Register the character driver */
	ret = register_driver(devname, &g_mpu6000fops, 0666, dev);
	if (ret < 0) {
		snerr("ERROR: Failed to register driver: %d\n", ret);
		kmm_free(dev->fifo.addr);
		kmm_free(dev);
		return ret;
	}

#ifdef CONFIG_DEBUG_SENSORS_INFO
	sninfo("BMP280 driver loaded successfully!\n");

	nxsig_usleep(1000);

	struct file thisIsNotFile;
	struct inode thisIsNotInode;
	bmp280_data_t result = {0, 0};

	thisIsNotFile.f_inode = &thisIsNotInode;
	thisIsNotInode.i_private = dev;

	_read(&thisIsNotFile, &result, 16);
	sninfo("BMP280: pressure: %f pa, temperature: %f deg\n",
				result.pressure, result.temperature);
#endif

	//_worker(dev);

	nxsem_post(&dev->sem);
	return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_MPU6000 */
