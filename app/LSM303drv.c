/*******************************************************************************
 LSM303D accelerometer Driver for nRF51
  
  Company:
    AvaTech
  
  File Name:
    LSM303.c
		
  Summary:
    Driver for the LSM303D Accelerometer.
		
  Description:
    This file contains the definitions and function prototypes for the LSM303D 
    accelerometer.
     
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

// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "SPI_utils.h"
#include <stdint.h>
#include "LSM303drv.h"
#include "nrf51.h"
#include "nrf_delay.h"


// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void uint8_t getLSM303ID(void)
		
  Remarks:
    See prototype in LSM303drv.h.
 */
 
uint8_t get_LSM303_ID(void)
{
		return(SPIReadByte(LSM_WHO_AM_I|LSM303_READ_BIT, LSM_DEVICE));
}


/*******************************************************************************
  Function:
    void init_SPI_IMU(void)

  Remarks:
    See prototype in LSM303drv.h.
 */

void init_LSM303(void)
{
		
		uint8_t address, SPIData;
//		nrf_gpio_cfg_output(ACC_CS_PIN);
		nrf_gpio_cfg_input(4,GPIO_PIN_CNF_PULL_Disabled);

		address = LSM_CTRL1;
		SPIData = LSM_DATA_RATE_800HZ | LSM_AZEN | LSM_AYEN | LSM_AXEN;
		
		SPIWriteReg(address, SPIData, LSM_DEVICE);
    
        address = LSM_CTRL2;
        SPIData = LSM_RANGE_16G;
		
		SPIWriteReg(address, SPIData, LSM_DEVICE); 
	
		//nrf_gpio_pin_clear(ACC_CS_PIN);
}

void sleep_LSM303(void)
{
		
		uint8_t address, SPIData;


		address = LSM_CTRL1;
		SPIData = 0x00;
		
		SPIWriteReg(address, SPIData, LSM_DEVICE);

}
/*******************************************************************************
  Function:
    LSM303_DATA getLSM303data(void)

  Remarks:
    See prototype in LSM303drv.h.
 */

LSM303_DATA getLSM303data(void)
{
    LSM303_DATA data;
    uint8_t address, tx_buf[7], rx_buf[7];
		
    nrf_gpio_cfg_output(ACC_CS_PIN);
    nrf_gpio_pin_set(ACC_CS_PIN);
    nrf_delay_us(1); //TODO is this necessary?
    
    address = LSM_OUT_X_L_A | LSM303_READ_BIT | LSM303_CONTINUOUS_BIT;  
	
	NRF_GPIO->OUTCLR = (1<<SPI_CS_ACC);
    SPIReadMultipleBytes(address, tx_buf, rx_buf, 7);//read in 7 bytes 2*3 axes + 1 bytes of header info
	NRF_GPIO->OUTSET = (1<<SPI_CS_ACC);
	
	data.X = rx_buf[1] + ((((uint16_t)rx_buf[2])<<8)&0xff00);
	data.Y = rx_buf[3] + ((((uint16_t)rx_buf[4])<<8)&0xff00);
	data.Z = rx_buf[5] + ((((uint16_t)rx_buf[6])<<8)&0xff00);
	
	nrf_gpio_pin_clear(ACC_CS_PIN);
    
    return data;
}

