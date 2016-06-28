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
#define SPIS_CS_PIN 12 //CSN generated on PIC (active low)
#define SPIS_RDY_PIN 10 //RDY pin to signal to PIC that data is ready to be sent (active high)
#define SPI_CS_ACC	 2
#define SPI_CS_GYRO  15 //TODO: change for our ble pcb
#define SPI_INSTANCE  0 /**< SPI instance index. */
#define SPIS_INSTANCE 1 /**< SPIS instance index. */

#define SPIS_BUFFER_MAX 256
#define SPIS_DATA_LENGTH_MAX 255

#define PIC_ARM_START_BYTE 0x5C
#define PIC_ARM_STOP_BYTE 0xC5
#define PIC_ARM_HEADER_SIZE 5 //in bytes --- start byte + cmd + length(2) + stop byte

//#define PIC_ARM_DATA_START_BYTE 0xDA
//#define PIC_ARM_DATA_STOP_BYTE 0xAD




typedef enum
{
    LSM_DEVICE = 0,
    L3G_DEVICE = 1
} SPI_DEVICE;

typedef enum
{
    PA_DEVICE_STATUS,
    PA_PROFILE
} pic_arm_code_t;

typedef struct
{
    uint8_t start_byte;
    pic_arm_code_t code;
    uint16_t length;
    uint8_t stop_byte;
} header_packet_t;

void spi_init(void);
uint8_t getLSMID(void);
uint8_t SPIReadByte(uint8_t address, SPI_DEVICE device);
void SPIWriteReg(uint8_t address, uint8_t regVal, SPI_DEVICE device);
void SPIReadMultipleBytes(uint8_t address, uint8_t * tx_buf, uint8_t * rx_buf, uint8_t length);
void spis_init(void);
void spis_event_handler(nrf_drv_spis_event_t event);
uint8_t  prep_packet_for_PIC(pic_arm_code_t pa_code, uint16_t pa_tx_length, uint8_t * pa_data, uint8_t * tx_buffer)
uint8_t parse_packet_from_PIC(uint8_t * rx_buffer);
void set_RDY(void);
void clear_RDY(void);
bool isRDY(void);
uint8_t buffer_size_calc(uint16_t spis_transfer_length);


#endif	/* SPI_UTILS_H */

