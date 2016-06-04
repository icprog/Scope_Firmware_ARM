#ifndef SPI_UTILS_H
#define	SPI_UTILS_H

#include <stdint.h>

#define SPI_CS_PIN   7
#define SPI_CS_ACC	 5
#define SPI_CS_GYRO  6
#define SPI_INSTANCE  0 /**< SPI instance index. */

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

#endif	/* SPI_UTILS_H */

