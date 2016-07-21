/*******************************************************************************
  L3GD20 Gyroscope driver
  
  Company:
    AvaTech
  
  File Name:
    L3GD20drv.c
		
  Summary:
    Driver for the L3GD20 Gyro.
		
  Description:
    DThis file contains the definitions and function prototypes for the L3GD20 gyro.
     
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
#ifndef L3GD20DRV_H
#define	L3GD20DRV_H


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

#define L3GD_READ_BIT               0x80
#define L3GD_CONTINUOUS_BIT         0x40

// *****************************************************************************
/* Gyro Registers

  Summary:
    Control Registers used by IMU.c.
		
  Description:
    This enumeration defines the allowed gyro control registers used by 
    IMU.c for the Pololu AltIMU-10 v4 with an L3GD20H.
*/

typedef enum
{
    L3GD_WHO_AM_I = 0x0f,
    L3GD_CTRL1 = 0x20,
    L3GD_CTRL2 = 0x21,
    L3GD_OUT_X_L = 0x28,
    L3GD_OUT_X_H = 0x29,
    L3GD_OUT_Y_L = 0x2a,
    L3GD_OUT_Y_H = 0x2b,
    L3GD_OUT_Z_L = 0x2c,
    L3GD_OUT_Z_H = 0x2d      
} L3GD_COMMANDS;

//*****************************************************************************
/* GYRO CTRL1 configuration Bits

  Summary:
    GYRO CTRL1 configuration Bits

  Description:
    GYRO CTRL2 configuration Bits.  This is a partial list. 
*/

typedef enum
{
    L3GD_DATA_RATE_800HZ = 0xc0,
    L3GD_CUTOFF_100HZ = 0x30,
    L3GD_POWER = 0x08,
    L3GD_ZEN = 0x04,
    L3GD_YEN = 0x02,
    L3GD_XEN = 0x01
} L3GD_CTRL1_BITS;


typedef struct
{
    int16_t    X;
    int16_t    Y;
    int16_t    Z;
} L3GD_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Public Prototypes
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************
  Function:
    uint8_t getL3GD_ID(void)

  Summary:
   Returns Who Am I.

  Description:
   This function returns the Who Am I byte.

  Precondition:
    The SPI pins and the SPI driver must be initialized.  The LSM303D must be
    initialized.

  Parameters:
    None.

  Returns:
    uint8_t

  Example:
    <code>
    uint8_t IDdata;
    L3GD_init();
    IDdata = getL2GD_ID();
    </code>

  Remarks:
    None.
*/

uint8_t get_L3GD_ID(void);


/*******************************************************************************
  Function:
    void L3GD_init(void)

  Summary:
   Initializes the L3GD20.

  Description:
   This function initializes the pins used by the L3GD20.

  Precondition:
    The SPI pins and the SPI driver must be initialized.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    L3GD_init();
    </code>

  Remarks:
    None.
*/

void init_L3GD(void);


/*******************************************************************************
  Function:
    L3GD_DATA getL3GDdata(void)

  Summary:
   Retrieves gyro data.

  Description:
   This function returns the gyro data.

  Precondition:
    The SPI pins and the SPI driver must be initialized.  The L3GD20 must be
    initialized.

  Parameters:
    None.

  Returns:
    L3GD_DATA structure

  Example:
    <code>
    L3GD_DATA data;
    L3GD_init();
    data = getL3GDdata();
    </code>

  Remarks:
    None.
*/

L3GD_DATA getL3GDdata(void);

#endif	/* L3GD20DRV_H */


