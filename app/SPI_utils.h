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
    Joe Trovato, Richard Kirby
 
  Created on:
    June 1, 2016
  
 *******************************************************************************/
 
#ifndef SPI_UTILS_H
#define	SPI_UTILS_H

#include <stdint.h>
#include "nrf_drv_spis.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
#define SPIS_INSTANCE 1 /**< SPIS instance index. */

#define SPIS_BUFFER_MAX 256
#define SPIS_DATA_LENGTH_MAX 255

#define PIC_ARM_START_BYTE 0x5C
#define PIC_ARM_STOP_BYTE 0xC5

#define  BYTE_SWAP(X) (X>>8)|(X<<8)


//#define PIC_ARM_DATA_START_BYTE 0xDA
//#define PIC_ARM_DATA_STOP_BYTE 0xAD

extern uint16_t force_cal_consts[5];


typedef enum
{
    LSM_DEVICE = 0,
    L3G_DEVICE = 1
} SPI_DEVICE;

typedef enum
{
    PA_ACK,
    PA_RESTART,
    PA_DEVICE_STATUS,
    PA_PROFILE,
    PA_FORCE_CAL_INIT,
    PA_RAW_DATA, //ARM->PIC = ACK. PIC->ARM = DATA
	PA_RAW_SUB_DATA, //subsampled raw data
    PA_ACCEL_START,
    PA_ACCEL_STOP, 
    PA_IMU_DATA,
    PA_PROBE_ERROR, //PIC->ARM
    PA_ARM_DONE, //ARM ->PIC
    PA_PROFILE_ID,
    PA_LOCATION_TIME,
    PA_TIMEOUT,
	PA_SERIAL_SET,  //for setting serial number on PIC
    PA_XMODEM,
    PA_START_TEST,
    PA_NEW_ID,
    
    PA_PCB_TEST,
    PA_PCB_TEST_DATA,
    PA_DEVICE_INFO,
    PA_FWU_START,
    PA_FWU_DATA,
    PA_FWU_ACK,
     /******************  calibration codes  ****************/
    PA_CAL_START, //dummy code used to mark where cal codes start
            
    PA_FORCE_CAL_WEIGHT, //ARM -> PIC
    PA_FORCE_CAL_DATA, //PIC -> ARM
    
    PA_OPTICAL_CAL_LENGTH, //ARM -> PIC
    PA_OPTICAL_CAL_DATA, //PIC -> ARM
    
    PA_VIB_CAL_RDY,  //ARM -> PIC
            
    PA_CAL_COMPLETE, //PIC -> ARM
    PA_OPTICAL_CAL_RESULT, //PIC-> ARM            
    //NOTE: no hall effect codes because the hall effect happen completely on the ARM
    
} pic_arm_code_t;

typedef struct
{
    pic_arm_code_t code;
    uint8_t * data;
    uint16_t data_size;
}pic_arm_pack_t;

extern pic_arm_pack_t test_code_pack;
extern pic_arm_pack_t force_cal_init_pack;
extern pic_arm_pack_t force_cal_data_pack;
extern pic_arm_pack_t force_cal_weight_pack;
extern pic_arm_pack_t vib_cal_rdy_pack;
extern pic_arm_pack_t optical_cal_length_pack;
extern pic_arm_pack_t hall_effect_test_pack;
extern pic_arm_pack_t send_sn_pack;
extern pic_arm_pack_t get_profile_pack;
extern pic_arm_pack_t arm_done_pack;
extern pic_arm_pack_t raw_data_ack_pack;
extern pic_arm_pack_t profile_id_pack;
extern pic_arm_pack_t stop_accel_pack;
extern pic_arm_pack_t location_time_pack;
extern pic_arm_pack_t spis_fail_pack;
extern pic_arm_pack_t send_device_info_pack;
extern pic_arm_pack_t serial_set_pack;
extern pic_arm_pack_t xmodem_pack;
extern pic_arm_pack_t start_test_pack;
extern pic_arm_pack_t fwu_start_pack;
extern pic_arm_pack_t fwu_data_pack;

extern volatile bool raw_data_transfer_in_progress;


typedef struct
{
    uint8_t start_byte;
    uint8_t code;
    uint16_t length;
    uint8_t stop_byte;
} header_packet_t;
#define PIC_ARM_HEADER_SIZE sizeof(header_packet_t)

void spi_init(void);
uint8_t getLSMID(void);
uint8_t SPIReadByte(uint8_t address, SPI_DEVICE device);
void SPIWriteReg(uint8_t address, uint8_t regVal, SPI_DEVICE device);
void SPIReadMultipleBytes(uint8_t address, uint8_t * tx_buf, uint8_t * rx_buf, uint8_t length);
void spis_init(void);
void spis_event_handler(nrf_drv_spis_event_t event);
uint8_t send_data_to_PIC(pic_arm_pack_t pa_pack);
uint8_t parse_packet_from_PIC(uint8_t * rx_buffer, uint8_t rx_buffer_length);
void set_ARM_RDY(void);
void clear_ARM_RDY(void);
void set_ARM_REQ(void);
void clear_ARM_REQ(void);
bool get_ARM_REQ(void);
uint8_t buffer_size_calc(uint16_t spis_transfer_length);


#endif	/* SPI_UTILS_H */

