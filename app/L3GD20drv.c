/*******************************************************************************
  L3GD20 Gyroscope driver
  
  Company:
    AvaTech
  
  File Name:
    L3GD20drv.c
		
  Summary:
    Driver for the L3GD20 Gyro.
		
  Description:
    Driver for the L3GD20 Gyro.
     
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

#include "L3GD20drv.h"
#include "SPI_utils.h"
#include <stdint.h>
#include "nrf51.h"


// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    uint8_t getL3GD_ID(void)
		
  Remarks:
    See prototype in L3GDdrv.h.
 */

uint8_t get_L3GD_ID(void)
{
	
    return(SPIReadByte(L3GD_WHO_AM_I|L3GD_READ_BIT, L3G_DEVICE));  
}


/*******************************************************************************
  Function:
    void init_L3GD(void)
  Remarks:
    See prototype in L3GD20drv.h.
 */

void init_L3GD(void)
{
    uint8_t address, SPIData;
    
    address = L3GD_CTRL1;  
    SPIData = L3GD_DATA_RATE_800HZ | L3GD_CUTOFF_100HZ | L3GD_POWER | L3GD_ZEN | L3GD_YEN | L3GD_XEN;
	
		SPIWriteReg(address, SPIData, L3G_DEVICE);

}


/*******************************************************************************
  Function:
    L3GD_DATA getL3GDdata(void)
  Remarks:
    See prototype in L3GDdrv.h.
 */

L3GD_DATA getL3GDdata(void)
{
    L3GD_DATA data;
	
    uint8_t address, tx_buf[7], rx_buf[7];
    
    address = L3GD_OUT_X_L | L3GD_READ_BIT | L3GD_CONTINUOUS_BIT; 
	
	 NRF_GPIO->OUTCLR = (1<<SPI_CS_GYRO);
	 SPIReadMultipleBytes(address, tx_buf, rx_buf, 7);
	 NRF_GPIO->OUTSET = (1<<SPI_CS_GYRO);

    data.X = rx_buf[1] + ((((uint16_t)rx_buf[2])<<8)&0xff00);
    data.Y = rx_buf[3] + ((((uint16_t)rx_buf[4])<<8)&0xff00);
    data.Z = rx_buf[5] + ((((uint16_t)rx_buf[6])<<8)&0xff00);
    
    return(data);
}
