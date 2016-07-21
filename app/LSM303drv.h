/*******************************************************************************
 LSM303D accelerometer Driver for nRF51
  
  Company:
    AvaTech
  
  File Name:
    LSM303.h
		
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

#ifndef LSM303DRV_H
#define	LSM303DRV_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>


// *****************************************************************************
// *****************************************************************************
// Section: Definitions 
// *****************************************************************************
// *****************************************************************************

#define LSM303_READ_BIT         0x80
#define LSM303_CONTINUOUS_BIT   0x40

#define ACC_CS_PIN              2


// *****************************************************************************
/* LSM303D Registers

  Summary:
    Control Registers used by the LSM303D.
		
  Description:
    This enumeration defines the allowed accelerometer control registers used by 
    LSM303D accelerometer 
*/

typedef enum
{
    LSM_TEMP_OUT_L = 0x05,
    LSM_TEMP_OUT_H = 0x06,        
    LSM_WHO_AM_I = 0x0f,
    LSM_INT_CTRL_M = 0x12, //interrupt control
    LSM_INT_SRC_M = 0x13,  //interrupt source
    LSM_INT_THS_L = 0x14,  // interrupt threshold (low byte)
    LSM_INT_THS_H = 0x15,  // interrupt threshold (high byte)
    LSM_CTRL1 = 0x20,
    LSM_CTRL2 = 0x21, //contains settings for accel range +-2g -> +-16g
    LSM_CTRL5 = 0x24,
    LSM_CTRL6 = 0x25,
    LSM_CTRL7 = 0x26,
    LSM_OUT_X_L_A = 0x28,
    LSM_OUT_X_H_A = 0x29,
    LSM_OUT_Y_L_A = 0x2a,
    LSM_OUT_Y_H_A = 0x2b,
    LSM_OUT_Z_L_A = 0x2c,
    LSM_OUT_Z_H_A = 0x2d,
    LSM_IG_CFG1 = 0x30    //interrupt config
} LSM_CONTROL;


// *****************************************************************************
/* LSM303D CTRL1 configuration Bits

  Summary:
    CTRL1 configuration Bits

  Description:
    CTRL1 configuration Bits.  This is a partial list. 
*/

typedef enum
{
		LSM_AXEN = 0x01,
		LSM_AYEN = 0x02,
		LSM_AZEN = 0x04,
		LSM_DATA_RATE_200HZ = 0x70,
		LSM_DATA_RATE_400HZ = 0x80,
		LSM_DATA_RATE_800HZ = 0x90,
		LSM_DATA_RATE_1600HZ = 0xa0
} LSM_CTRL1_BITS;


// *****************************************************************************
/* LSM303D CTRL2 configuration Bits

  Summary:
    LSM303D CTRL2 configuration Bits

  Description:
    LSM303D CTRL2 configuration Bits.  This is a partial list. 
*/

typedef enum
{
    LSM_RANGE_2G = 0x00,
    LSM_RANGE_4G = 0x08,
    LSM_RANGE_6G = 0x10,
    LSM_RANGE_8G = 0x18,
    LSM_RANGE_16G = 0x20
} LSM_CTRL2_BITS;


// *****************************************************************************
/* LSM303D Accelerometer Data

  Summary:
    LSM303D Accelerometer Data

  Description:
    This structure holds 3 axes of LSM303D accelerometer data. 
*/

typedef struct LSM303_DATA
{
    int16_t    X;
    int16_t    Y;
    int16_t    Z;
} LSM303_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Public Application Prototypes
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************
  Function:
    uint8_t getLSM303ID(void);

  Summary:
    Returns the "Who Am I?" value for the LSM303D.

  Description:
    Returns the "Who Am I?" value for the LSM303D.  
		
  Precondition:
    None.  
		
  Parameters:
    None.
		
  Returns:
    Who Am I register value.
		
  Example:
    <code>
	  printf("\n\rLSM303D Who Am I register: %x," getLSM303ID());
    </code>
		
  Remarks:
    None.
*/

uint8_t get_LSM303_ID(void);


/*******************************************************************************
  Function:
    void init_LSM303(void);

  Summary:
    Configures the IMU.

  Description:
    Configures the LSM303 accelerometer.  
		
  Precondition:
    SPI0 must be initialized  
		
  Parameters:
    None.
		
  Returns:
    None.
		
  Example:
    <code>
	  init_LSM303()
    </code>
		
  Remarks:
    None.
*/

void init_LSM303(void);


/*******************************************************************************
  Function:
    LSM303_DATA getLSM303data(void);
		
  Summary:
    Get acceleration data.
		
  Description:
    Gets 3 axes of acceleration data. 
		
  Precondition:
    SPI0 must be initialized and the LSM must be initialized 
		
  Parameters:
    None.
		
  Returns:
    LSM303_DATA structure.
		
  Example:
    <code>
    LAM303_DATA data;
    data = getLSM303data();
    </code>
		
  Remarks:
    None.
*/

LSM303_DATA getLSM303data(void);

#endif	/* LSM303DRV_H */
