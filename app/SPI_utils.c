/*******************************************************************************
  SPI_utils: SPI Master and Slave utilities
  
  Company:
    AvaTech
  
  File Name:
    SPI_utils.c
		
  Summary:
    SPI Master and Slave utilities.
		
  Description:
    This file contains the initialization, configuration and handlers for the SPI master and slave module.
     
  Processor:       
    nRF51822
		
  Toolchaing:
		MDK-Lite version 5.20
 
  Author(s): 
    Richard Kirby
 
  Created on:
    June 1, 2016
  
  Revision History:
    Development version      June 1, 2016
 *******************************************************************************/
 
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
#include "nrf_drv_spis.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "boards.h"
#include <string.h>
#include "SEGGER_RTT.h"


static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG(SPIS_INSTANCE);
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
volatile bool spis_xfer_done; /**< Flag used to indicate that SPIS instance completed the transfer. */
uint8_t       m_tx_buf_s[4] = {0xb0, 0x0b, 0x55, 0x55};
uint8_t       m_rx_buf_s[5];
static const uint8_t m_length = sizeof(m_tx_buf_s);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    SEGGER_RTT_printf(0, "SPI event handler");
    spi_xfer_done = true;
}

/**
 * @brief SPIS user event handler.
 *
 * @param event
 */
void spis_event_handler(nrf_drv_spis_event_t event)
{
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {
        spis_xfer_done = true;
		if (nrf_drv_spis_buffers_set(&spis, m_tx_buf_s, m_length, m_rx_buf_s, m_length) != NRF_SUCCESS)
			LEDS_ON(BSP_LED_3_MASK);
		LEDS_INVERT(BSP_LED_2_MASK);
        SEGGER_RTT_printf(0, "\nreceived ");
        for(int i = 0; i < m_length; i++)
        {
             SEGGER_RTT_printf(0, "0x%x  ", m_rx_buf_s[i]);
        }

    }
}

/**
 * @brief SPI Master Initialization
 *
 */
void spi_init(void)
{
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
		spi_config.ss_pin = SPI_CS_PIN;
	  /* TODO(rk): turn off CS pin in master SPI driver */
		spi_config.mode = NRF_DRV_SPI_MODE_3;
		APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
		
		nrf_gpio_cfg_output(SPI_CS_ACC);
		nrf_gpio_cfg_output(SPI_CS_GYRO);
	
	  NRF_GPIO->OUTSET = (1<<SPI_CS_ACC);
	  NRF_GPIO->OUTSET = (1<<SPI_CS_GYRO);
		
//		printf("\n\r\n\rSPI Master Configuration:");
//		printf("\n\r  SCK pin: %d", spi_config.sck_pin);
//		printf("\n\r  MOSI pin: %d", spi_config.mosi_pin);
//		printf("\n\r  MISO pin: %d", spi_config.miso_pin);
//		printf("\n\r  Chip Select pin: %d", spi_config.ss_pin);
//		printf("\n\r  SPI Mode: %d", spi_config.mode);
}


/**
 * @brief SPI Slave Initialization
 *
 */

void spis_init(void)
{
    spis_config.csn_pin         = SPIS_CS_PIN;
    spis_config.mode 			= NRF_DRV_SPIS_MODE_1; //NRF_DRV_SPIS_MODE_3;
	
	/*printf*/SEGGER_RTT_printf(0,"\n\r\n\rSPI Slave Configuration:");
	/*printf*/SEGGER_RTT_printf(0,"\n\r  SCK pin: %d", spis_config.sck_pin);
    /*printf*/SEGGER_RTT_printf(0,"\n\r  MOSI pin: %d", spis_config.mosi_pin);
    /*printf*/SEGGER_RTT_printf(0,"\n\r  MISO pin: %d", spis_config.miso_pin);
    /*printf*/SEGGER_RTT_printf(0,"\n\r  Chip Select pin: %d", spis_config.csn_pin);
    /*printf*/SEGGER_RTT_printf(0,"\n\r  SPI Mode: %d", spis_config.mode);

    if (nrf_drv_spis_init(&spis, &spis_config, spis_event_handler) == NRF_SUCCESS)
            /*printf*/SEGGER_RTT_printf(0,"\nSPI Slave Initialization Succeeded");
    else
        /*printf*/SEGGER_RTT_printf(0,"\nSPI Slave Initialization Failed");
            
    if (nrf_drv_spis_buffers_set(&spis, m_tx_buf_s, m_length, m_rx_buf_s, m_length) == NRF_SUCCESS)
          /*printf*/SEGGER_RTT_printf(0,"\nSPI Slave Buffer Set Succeeded");
    else
        /*printf*/SEGGER_RTT_printf(0,"\nSPI Slave Buffer Set Failed");
    
    /******* initialize RDY pin  ********/
    nrf_gpio_cfg_output(SPIS_RDY_PIN);
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

void set_RDY(void)
{
    nrf_gpio_pin_set(SPIS_RDY_PIN);
}
void clear_RDY(void)
{
    nrf_gpio_pin_clear(SPIS_RDY_PIN);
}

