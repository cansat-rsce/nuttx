/*****************************************************************************
 * Included Files
 ****************************************************************************/


#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/wireless/nrf24l01.h>
#include <nuttx/spi/spi.h>

#include "stm32.h"
#include "omnibus4prov2.h"
#include "stm32_spi.h"

#ifdef CONFIG_WL_NRF24L01

/*****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_SPI3
#  error "NRF24L01+ driver requires CONFIG_STM32_SPI3 to be enabled"
#endif

/*****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define NRF_SPI_PORT 3 /* nsf24 is connected to SPI3 port */

/*****************************************************************************
 * Private Definitions
 ****************************************************************************/

static int irqattach(xcpt_t isr, FAR void *arg) {
	return stm32_gpiosetevent(GPIO_NRF24L01_INT, true, false, false, isr, arg); //FIXME check args
}

static void chipenable(bool enable) {
	stm32_gpiowrite(GPIO_NRF24L01_CE, enable);
}

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

struct nrf24l01_config_s config = {
	.irqattach = irqattach,
	.chipenable = chipenable
};
 
int stm32_nrf24l01_initialize(void) {
	struct spi_dev_s *spi;
	stm32_gpiosetevent
	sninfo("INFO: Initializing NRF24L01+\n");

	spi = stm32_spibus_initialize(NRF_SPI_PORT);
	if (spi == NULL) {
		snerr("ERROR: Failed to initialize SPI port %d\n", NRF_SPI_PORT);
		return -ENODEV;
	}

	return nrf24l01_register(spi, &config);
}

#endif /* CONFIG_WL_NRF24L01 */
