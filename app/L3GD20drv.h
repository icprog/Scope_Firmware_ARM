#include <stdint.h>

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

uint8_t getL3GD_ID(void);
void init_L3GD(void);
L3GD_DATA getL3GDdata(void);
