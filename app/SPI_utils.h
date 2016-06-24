/*******************************************************************************
  SPI_utils: SPI Master and Slave utilities header
  
  Company:
    AvaTech
  
  File Name:
    SPI_utils.h
		
  Summary:
    SPI Master and Slave utilities header.
		
  Description:
    This file contains the function prototypes and definitions for teh SPI master and slave
		initialization, configuration and handlers.
     
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
 
#ifndef SPI_UTILS_H
#define	SPI_UTILS_H

#include <stdint.h>
#include "nrf_drv_spis.h"

#define SPI_CS_PIN   7 //chips select for the SPI module for the IMU
#define SPIS_CS_PIN 10 //REQN on ARM, RDYN on PIC
#define SPI_CS_ACC	 2
#define SPI_CS_GYRO  15 // TODO: change for our ble pcb
#define SPI_INSTANCE  0 /**< SPI instance index. */
#define SPIS_INSTANCE 1 /**< SPIS instance index. */

typedef enum
{
    LSM_DEVICE = 0,
    L3G_DEVICE = 1
} SPI_DEVICE;

void spi_init(void);
uint8_t getLSMID(void);
uint8_t SPIReadByte(uint8_t address, SPI_DEVICE device);
void SPIWriteReg(uint8_t address, uint8_t regVal, SPI_DEVICE device);
void SPIReadMultipleBytes(uint8_t address, uint8_t * tx_buf, uint8_t * rx_buf, uint8_t length);
void spis_init(void);
void spis_event_handler(nrf_drv_spis_event_t event);

#endif	/* SPI_UTILS_H */

