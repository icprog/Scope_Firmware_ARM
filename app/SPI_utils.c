#include "SPI_utils.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "boards.h"
#include <string.h>

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
static uint8_t       m_tx_buf[2];
static uint8_t       m_rx_buf[2];
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
}

void spi_init(void)
{
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
		spi_config.ss_pin = SPI_CS_PIN;
		spi_config.mode = NRF_DRV_SPI_MODE_3;
		APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
		
		nrf_gpio_cfg_output(SPI_CS_ACC);
		nrf_gpio_cfg_output(SPI_CS_GYRO);
	
	  NRF_GPIO->OUTSET = (1<<SPI_CS_ACC);
	  NRF_GPIO->OUTSET = (1<<SPI_CS_GYRO);
		
		printf("\n\rSPI Master Configuration:");
		printf("\n\r  SCK pin: %x", spi_config.sck_pin);
		printf("\n\r  MOSI pin: %x", spi_config.mosi_pin);
		printf("\n\r  MISO pin: %x", spi_config.miso_pin);
		printf("\n\r  Chip Select pin:w %x", spi_config.ss_pin);
		printf("\n\r  SPI Mode: %x", spi_config.mode);
}

uint8_t SPIReadByte(uint8_t address, SPI_DEVICE device)
{
	uint8_t rx_buf[2], tx_buf[2], CS_pin;
	
	if (device == LSM_DEVICE)
		CS_pin = SPI_CS_ACC;
	else if (device == L3G_DEVICE)
		CS_pin = SPI_CS_GYRO;
	
	spi_xfer_done = false;
	tx_buf[0] = address | 0x80;	// set read bit
	
	NRF_GPIO->OUTCLR = (1<<CS_pin);
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buf, 2, rx_buf, 2));
	NRF_GPIO->OUTSET = (1<<CS_pin);

	while (!spi_xfer_done)
	{
			__WFE();
	}
	return(rx_buf[1]);
}

void SPIReadMultipleBytes(uint8_t address, uint8_t * tx_buf, uint8_t * rx_buf, uint8_t length)
{
	
	spi_xfer_done = false;
	tx_buf[0] = address | 0x80;	// set read bit
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buf, length, rx_buf, length));

	while (!spi_xfer_done)
	{
			__WFE();
	}
}

void SPIWriteReg(uint8_t address, uint8_t regVal, SPI_DEVICE device)
{
		uint8_t rx_buf[2], tx_buf[2], CS_pin;
	
		if (device == LSM_DEVICE)
				CS_pin = SPI_CS_ACC;
		else if (device == L3G_DEVICE)
			CS_pin = SPI_CS_GYRO;
	
		spi_xfer_done = false;
		tx_buf[0] = address;
		tx_buf[1] = regVal;
		
		NRF_GPIO->OUTCLR = (1<<CS_pin);
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buf, 2, rx_buf, 2));
		NRF_GPIO->OUTSET = (1<<CS_pin);

		while (!spi_xfer_done)
		{
				__WFE();
		}
}


