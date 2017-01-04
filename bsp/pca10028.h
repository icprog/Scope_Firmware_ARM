/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#ifndef PCA10028_H
#define PCA10028_H

/************* Leftover pin mappings **************/
#define RX_PIN_NUMBER 0
#define TX_PIN_NUMBER 0
#define RTS_PIN_NUMBER 0
#define CTS_PIN_NUMBER 0

/**************** Scope Pin Mappings  *************/

/****** shutdown logic pins   *******/
#define SCOPE_HALL_PIN              7
#define SCOPE_3V3_ENABLE_PIN        30
#define SCOPE_SPIS_READY            1

/****** SPI Slave for PIC comms  *******/
#define SPIS_SCK_PIN 8
#define SPIS_MOSI_PIN 9
#define SPIS_MISO_PIN 11
#define SPIS_CS_PIN 12 //CSN generated on PIC (active low)
#define SPIS_ARM_REQ_PIN 10 //RDY pin to signal to PIC that data is ready to be sent (active high)
#define SPIS_ARM_RDY_PIN  1 //pin to signal that ARM is ready for a transaction. 

/******* SPI port for IMU  ******/
#define IMU_SPI_SCK_PIN     6
#define IMU_SPI_MOSI_PIN    5
#define IMU_SPI_MISO_PIN    4
#define IMU_SPI_CS_GYRO_PIN  3 
#define IMU_SPI_CS_ACC_PIN	 2


// Low frequency clock source to be used by the SoftDevice
#ifdef S210
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#else
#define NRF_CLOCK_LFCLKSRC      {.source        = /*NRF_CLOCK_LF_SRC_SYNTH*/ NRF_CLOCK_LF_SRC_XTAL,      \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
#endif

#endif // PCA10028_H
