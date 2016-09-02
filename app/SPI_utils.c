/*******************************************************************************
  SPI_utils: SPI Master and Slave utilities
  
  Company:
    AvaTech
  
  File Name:
    SPI_utils.c
		
  Summary:
    SPI Master and Slave utilities.
		
  Description:
    This file contains the initialization, configuration and handlers for the SPI master and slave module.
     
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
 
#include "SPI_utils.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_spis.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "boards.h"
#include <string.h>
#include "SEGGER_RTT.h"
#include "app.h"
#include "calibration.h"
#include "LSM303drv.h"
#include "debug.h"

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */

static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG(SPIS_INSTANCE);
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
uint8_t       m_tx_buf_s[SPIS_BUFFER_MAX];
uint8_t       m_rx_buf_s[SPIS_BUFFER_MAX];
uint8_t       dummy_buf[32];
LSM303_DATA accel_data; //acelerometer data to pass to PIC
uint8_t sending_data_to_phone =0;
volatile bool device_info_received = false;


/********  global variable for building a tx packet for PIC   **********/
volatile bool raw_data_transfer_in_progress = false;
uint16_t spis_rx_transfer_length = 0;
uint16_t spis_tx_transfer_length = 0;
void * rx_data_ptr; //where to put the data received from the PIC
void * tx_data_ptr; //where to pull data from to send to PIC

/********  pic arm pack varaibles  *******/
pic_arm_pack_t test_code_pack = {TEST_CODE, dummy_buf, 0};
pic_arm_pack_t force_cal_init_pack = {PA_FORCE_CAL_INIT, dummy_buf, 0};
pic_arm_pack_t force_cal_weight_pack = {PA_FORCE_CAL_WEIGHT, &(cal_data.current_weight), 1};
pic_arm_pack_t vib_cal_rdy_pack = {PA_VIB_CAL_RDY, dummy_buf, 0};
pic_arm_pack_t optical_cal_length_pack = {PA_OPTICAL_CAL_LENGTH, cal_data.optical_parameters, 3};
pic_arm_pack_t get_profile_pack = {PA_PROFILE, dummy_buf, 4};
pic_arm_pack_t accelerometer_pack = {PA_ACCELEROMETER, (uint8_t *)&accel_data, 6}; //will it blend?
pic_arm_pack_t arm_done_pack = {PA_ARM_DONE, dummy_buf, 0};
pic_arm_pack_t raw_data_ack_pack = {PA_RAW_DATA, dummy_buf, 0};
pic_arm_pack_t profile_id_pack = {PA_PROFILE_ID, (uint8_t *)&(appData.profile_id), sizeof(profile_id_t)};
pic_arm_pack_t location_time_pack = {PA_LOCATION_TIME, (uint8_t *)metadata.location, 12};
pic_arm_pack_t spis_fail_pack = {PA_TIMEOUT, dummy_buf, 0};

extern device_info_t device_info;
extern subsampled_raw_data_t raw_sub_data;
extern profile_data_t profile_data;
extern ble_dbs_t						m_ds;
extern uint8_t					raw_sub_buff[BYTES_RAW_SUB_DATA];

/*
 * build the header packet, enable the RDY line and wait for the PIC to clock in the packet. 
 * Then handle the subsequent data packets in the spis_event_handler.
 */
uint8_t send_data_to_PIC(pic_arm_pack_t pa_pack)
{
    //TODO while(spis_tx_transfer_length) 
    if(spis_tx_transfer_length == 0) /* Header packet */
    {
        uint32_t err_code;
        header_packet_t packet;
        packet.start_byte = PIC_ARM_START_BYTE;
        packet.stop_byte = PIC_ARM_STOP_BYTE;
        packet.code = pa_pack.code;
        spis_tx_transfer_length = pa_pack.data_size; //TODO make this a regular variable
        packet.length = pa_pack.data_size;
        tx_data_ptr = pa_pack.data;
        
        memcpy(m_tx_buf_s, &packet, PIC_ARM_HEADER_SIZE); //TODO: look into why this memcoy is necessary as well.
        //TODO: something weird with timing here. need the print statement to get correct values
//        SEGGER_RTT_printf(0, "sending:");
//        for(int i = 0; i < PIC_ARM_HEADER_SIZE; i++)
//        {
//            SEGGER_RTT_printf(0, "  0x%x", ((uint8_t *)&packet)[i]);
//        }
        err_code = nrf_drv_spis_buffers_set(&spis, m_tx_buf_s, PIC_ARM_HEADER_SIZE, m_rx_buf_s, 0);
        if (err_code != NRF_SUCCESS)
        {
            SEGGER_RTT_printf(0, "SPIS error %d in send_data_to_PIC\n", err_code);
            return err_code;
        }
        //check that the SPIS semaphore is free before telling the PIC we are ready
        NRF_SPIS_Type * p_spis = spis.p_reg;
        if(nrf_spis_semaphore_status_get(p_spis) == NRF_SPIS_SEMSTAT_FREE)
        {
            set_RDY(); 
        }
        else
        {
            while(nrf_spis_semaphore_status_get(p_spis) != NRF_SPIS_SEMSTAT_FREE)
            {
                SEGGER_RTT_printf(0, "sem stat = %d\n", nrf_spis_semaphore_status_get(p_spis));
            }
            set_RDY(); 
        }
    }
    else
    {
        return 1;//ERROR
    }
    return 0;
}


/*
 * function to parse packets coming from PIC 
 */
uint8_t parse_packet_from_PIC(uint8_t * rx_buffer, uint8_t rx_buffer_length)
{
    
    header_packet_t * packet = (header_packet_t *)rx_buffer;
    static APP_STATES next_state = APP_STATE_POLLING;
		char debug_out_string[20];
    
    if(packet->start_byte == PIC_ARM_START_BYTE  &&  packet->stop_byte == PIC_ARM_STOP_BYTE && appData.transfer_in_progress == false)
    {
        //SEGGER_RTT_printf(0, "parsing header packet\n");
        spis_rx_transfer_length = packet->length;
        if(spis_rx_transfer_length > 0)
        {
            appData.SPIS_timeout_flag = 1;
            appData.transfer_in_progress = true;
        }
        //SEGGER_RTT_printf(0, "setting spis_rx_trasnfer_length to %d", spis_rx_transfer_length);
        
        //TODO sort out where to put the data from the buffer
        //SEGGER_RTT_printf(0, "code = %d\n", packet->code);
        switch(packet->code)
        {
            case TEST_CODE:
            {
               next_state = APP_STATE_FORCE_CAL_WEIGHT;
               break;
            }
            case PA_FORCE_CAL_DATA:
            {
                next_state = APP_STATE_FORCE_CAL_DATA;
                rx_data_ptr = cal_data.force_data;
                break;
            }
            case PA_OPTICAL_CAL_DATA:
            {
                //SEGGER_RTT_printf(0, "PA_OPTICAL_CAL_DATA\n");
                //set up a place to store data and change state
                next_state = APP_STATE_OPTICAL_CAL_DATA;
                rx_data_ptr = &(cal_data.optical_data);
                break;
            }
//            case PA_DEVICE_STATUS:
//            {
//                //SEGGER_RTT_printf(0, "DEV STATUS\n");
//                
//                break;
//            }
            case PA_PROFILE:
            {
                SEGGER_RTT_printf(0, "PA_PROFILE\n");
								rx_data_ptr = &profile_data;
								next_state = APP_STATE_TRANSFER_PROFILE_IDS;
                break;
            }
            case PA_ACCELEROMETER:
            {
                //SEGGER_RTT_printf(0, "PA_ACCELEROMETER\n");
								
//								sprintf(debug_out_string,"* DEBUG TEST STRING*");
//								ble_debug_update(&m_ds,debug_out_string, 20);  //send debug to phone
//								//char debug_out_string[20];
//								sprintf(debug_out_string,"*DEBUG TEST STRING 2");
//								ble_debug_update(&m_ds,debug_out_string, 20);  //send debug to phone
							
														next_state = APP_STATE_ACCELEROMETER;
                break;
            }
            case PA_OPTICAL_CAL_RESULT:
            {
                //SEGGER_RTT_printf(0, "PA_OPTICAL_CAL_RESULT\n");
                next_state = APP_STATE_OPTICAL_CAL_RESULT;
                rx_data_ptr = &(cal_data.optical_result);
                break;
            }
            case PA_PCB_TEST:
            {
                SEGGER_RTT_printf(0, "PA_PCB_TEST\n");
                next_state = APP_STATE_PCB_TEST;
                break;
            }
            case PA_DEVICE_INFO:
            {
                SEGGER_RTT_printf(0, "PA_DEVICE_INFO\n");
                rx_data_ptr = &device_info;
                next_state = APP_STATE_DEVICE_INFO;
                break;
            }
            case PA_RAW_DATA:
            {
                SEGGER_RTT_printf(0, "PA_RAW_DATA\n");
                next_state = APP_STATE_RAW_DATA_RECEIVE;
                rx_data_ptr = &raw_data_buff;
                break;
            }
            case PA_RAW_SUB_DATA:
            {
				SEGGER_RTT_printf(0, "PA_RAW_SUB_DATA\n");
                next_state = APP_STATE_RAW_SUB_DATA_RECEIVE;
                rx_data_ptr = &raw_sub_data;
                break;
			}
            case PA_PROBE_ERROR:
            {
                rx_data_ptr = &(metadata.error_code);
                next_state = APP_STATE_PROBE_ERROR;
                break;
            }
            case PA_RESTART:
            {
                SEGGER_RTT_printf(0, "PA_RESTART\n");
                break;
            }
            default:
            {
                SEGGER_RTT_printf(0, "SPIS ERROR: code not recognized\n");
                break;
            }
        }
        if(!appData.transfer_in_progress)
        {
            appData.state = next_state;
        }
    }
    else if(appData.transfer_in_progress) //Data packet
    {
        //SEGGER_RTT_printf(0, "parsing data packet\n");
        //length = buffer_size_calc(spis_rx_transfer_length);
        memcpy(rx_data_ptr, (void *)rx_buffer, rx_buffer_length);
				rx_data_ptr = (uint8_t *)rx_data_ptr + rx_buffer_length;
        //SEGGER_RTT_printf(0, "transfer length: %d \n",spis_rx_transfer_length);
        //TODO check the checksum
        if(spis_rx_transfer_length == 0) //finished transferring
        {
            appData.SPIS_timeout_flag = 0;
            SEGGER_RTT_printf(0, "finished transferring\n");
            appData.transfer_in_progress = false;
            appData.state = next_state;
        }
    }
    else
    {
		//SEGGER_RTT_printf(0, " ***  ERROR  *** \n");
        return 1; //ERROR
    }
    return 0;
}

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    //SEGGER_RTT_printf(0, "SPI event handler");
    spi_xfer_done = true;
}

/**
 * @brief SPIS user event handler. This function can basically only be used for reading from 
 * PIC as this is triggered on the SS line going high after being low. 
 *
 * @param event
 */
void spis_event_handler(nrf_drv_spis_event_t event)
{
    uint8_t rx_length, tx_length;
    uint8_t error_code = 0;
   
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE &&  sending_data_to_phone == 0)
    {
        /********* determine length of packet received  *********/
        rx_length = buffer_size_calc(spis_rx_transfer_length);
        if(spis_rx_transfer_length != 0)
        {
            spis_rx_transfer_length -= rx_length;
        }
        //SEGGER_RTT_printf(0, "\nreceived %d bytes: ", rx_length);
//        for(int i = 0; i < rx_length; i++)
//        {
//             SEGGER_RTT_printf(0, "0x%x  ", m_rx_buf_s[i]);
//        }
        //SEGGER_RTT_printf(0, "\n");

        /*** parse the received packet ****/
        parse_packet_from_PIC(m_rx_buf_s, rx_length); //sets spis_rx_transfer_length
		//SEGGER_RTT_printf(0, "\n %d \n", appData.transfer_in_progress);
        /******* determine if rdy needs to remain high after sending current data  ******/
        if(spis_tx_transfer_length == 0)
        {
            clear_RDY();
        }
        
        /**********  determine length of the packet to send and new one to receive *********/
        rx_length = buffer_size_calc(spis_rx_transfer_length);
        tx_length = buffer_size_calc(spis_tx_transfer_length);
        
        
        memcpy(m_tx_buf_s, tx_data_ptr, tx_length); //copy data to send into tx buffer
        if(spis_tx_transfer_length != 0)
        {
            //TODO: assign pointer
            //TODO: increment pointer
            
//            SEGGER_RTT_printf(0, "sending:");
//            for(int i = 0; i < tx_length; i++)
//            {
//                SEGGER_RTT_printf(0, "  0x%x", ((uint8_t *)&m_tx_buf_s)[i]);
//            }
            spis_tx_transfer_length -= tx_length;
        }
        
        //SEGGER_RTT_printf(0, "\nplanning on receiving %d bytes: ", rx_length);
        if (nrf_drv_spis_buffers_set(&spis, m_tx_buf_s, tx_length, m_rx_buf_s, rx_length) != NRF_SUCCESS)
        {
            SEGGER_RTT_printf(0, "SPIS error: failed to set buffers in interrupt");
        }  
    }
}

/**
 * @brief SPI Master Initialization
 *
 */
void spi_init(void)
{
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);
		spi_config.ss_pin = SPI_CS_PIN;
		
	  /* TODO(rk): turn off CS pin in master SPI driver */
		spi_config.mode = NRF_DRV_SPI_MODE_3;
		APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
		
		nrf_gpio_cfg_output(SPI_CS_ACC);
		nrf_gpio_cfg_output(SPI_CS_GYRO);
	
        NRF_GPIO->OUTSET = (1<<SPI_CS_ACC);
        NRF_GPIO->OUTSET = (1<<SPI_CS_GYRO);
		
//		printf("\n\r\n\rSPI Master Configuration:");
//		printf("\n\r  SCK pin: %d", spi_config.sck_pin);
//		printf("\n\r  MOSI pin: %d", spi_config.mosi_pin);
//		printf("\n\r  MISO pin: %d", spi_config.miso_pin);
//		printf("\n\r  Chip Select pin: %d", spi_config.ss_pin);
//		printf("\n\r  SPI Mode: %d", spi_config.mode);
}


/**
 * @brief SPI Slave Initialization
 *
 */

void spis_init(void)
{
    spis_config.csn_pin         = SPIS_CS_PIN;
    spis_config.mode 			= NRF_DRV_SPIS_MODE_0; //NRF_DRV_SPIS_MODE_3;
//	/*printf*/SEGGER_RTT_printf(0,"\n\r\n\rSPI Slave Configuration:");
//	/*printf*/SEGGER_RTT_printf(0,"\n\r  SCK pin: %d", spis_config.sck_pin);
//    /*printf*/SEGGER_RTT_printf(0,"\n\r  MOSI pin: %d", spis_config.mosi_pin);
//    /*printf*/SEGGER_RTT_printf(0,"\n\r  MISO pin: %d", spis_config.miso_pin);
//    /*printf*/SEGGER_RTT_printf(0,"\n\r  Chip Select pin: %d", spis_config.csn_pin);
//    /*printf*/SEGGER_RTT_printf(0,"\n\r  SPI Mode: %d", spis_config.mode);

    if (nrf_drv_spis_init(&spis, &spis_config, spis_event_handler) == NRF_SUCCESS)
    {
           // /*printf*/SEGGER_RTT_printf(0,"\nSPI Slave Initialization Succeeded");
    }
    else
    {
        /*printf*/SEGGER_RTT_printf(0,"\nSPI Slave Initialization Failed");
    }      
    if (nrf_drv_spis_buffers_set(&spis, m_tx_buf_s, PIC_ARM_HEADER_SIZE, m_rx_buf_s, PIC_ARM_HEADER_SIZE) == NRF_SUCCESS) //dummy lengths
    {
          ///*printf*/SEGGER_RTT_printf(0,"\nSPI Slave Buffer Set Succeeded");
    }
    else
    {
        /*printf*/SEGGER_RTT_printf(0,"\nSPI Slave Buffer Set Failed");
    }
    
    /******* initialize RDY pin  ********/
    nrf_gpio_cfg_output(SPIS_RDY_PIN);
   // SEGGER_RTT_printf(0,"\nSPI RDY pin: %d", SPIS_RDY_PIN);
}


uint8_t SPIReadByte(uint8_t address, SPI_DEVICE device)
{
	uint8_t rx_buf[2], tx_buf[2], CS_pin;
	uint16_t spi_timeout = 0;
	if (device == LSM_DEVICE)
		CS_pin = SPI_CS_ACC;
	else if (device == L3G_DEVICE)
		CS_pin = SPI_CS_GYRO;
	
	spi_xfer_done = false;
	tx_buf[0] = address | 0x80;	// set read bit
	
	NRF_GPIO->OUTCLR = (1<<CS_pin);
	APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buf, 2, rx_buf, 2));
	NRF_GPIO->OUTSET = (1<<CS_pin);

	while (!spi_xfer_done)
	{
			//__WFE();
		spi_timeout++;
		if(spi_timeout > 15000)
		{
			/*printf*/SEGGER_RTT_printf(0,"\n\r  timout!!!");
			spi_xfer_done = true;
			break;
			
		}
	}
	
	return(rx_buf[1]);
}

void SPIReadMultipleBytes(uint8_t address, uint8_t * tx_buf, uint8_t * rx_buf, uint8_t length)
{
	uint16_t spi_timeout = 0;
	spi_xfer_done = false;
	tx_buf[0] = address | 0x80;	// set read bit	 
    uint32_t err_code = nrf_drv_spi_transfer(&spi, tx_buf, length, rx_buf, length);

	while (!spi_xfer_done)
	{
		//__WFE();
		spi_timeout++;
		if(spi_timeout > 15000)
		{
			/*printf*/SEGGER_RTT_printf(0,"\n\r  timout!!!");
            nrf_delay_ms(50);
			spi_xfer_done = true;
			spi_init();
			break;
		}
	}
	
}

void SPIWriteReg(uint8_t address, uint8_t regVal, SPI_DEVICE device)
{
		uint8_t rx_buf[2], tx_buf[2], CS_pin;
		uint16_t spi_timeout = 0;
		if (device == LSM_DEVICE)
				CS_pin = SPI_CS_ACC;
		else if (device == L3G_DEVICE)
			CS_pin = SPI_CS_GYRO;
	
		spi_xfer_done = false;
		tx_buf[0] = address;
		tx_buf[1] = regVal;
		
		NRF_GPIO->OUTCLR = (1<<CS_pin);
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_buf, 2, rx_buf, 2));
		NRF_GPIO->OUTSET = (1<<CS_pin);

		while (!spi_xfer_done)
        {
			//__WFE();
		spi_timeout++;
		if(spi_timeout > 15000)
		{
			/*printf*/SEGGER_RTT_printf(0,"\n\r  timout!!!");
			spi_xfer_done = true;
			break;
			
		}
	}
}

void set_RDY(void)
{
    nrf_gpio_pin_set(SPIS_RDY_PIN);
    //SEGGER_RTT_printf(0, "\n setting RDY ");
}
void clear_RDY(void)
{
    nrf_gpio_pin_clear(SPIS_RDY_PIN);
    //SEGGER_RTT_printf(0, "\n clearing RDY ");
}
bool isRDY(void)
{
    return (bool)(NRF_GPIO->OUT & (1 << SPIS_RDY_PIN));
}

uint8_t buffer_size_calc(uint16_t spis_transfer_length)
{
    uint8_t buffer_size;
    if(spis_transfer_length == 0)
    {
        buffer_size = PIC_ARM_HEADER_SIZE;
    }
    else if(spis_transfer_length > SPIS_DATA_LENGTH_MAX)
    {
        buffer_size = SPIS_DATA_LENGTH_MAX;
    }
    else
    {
        buffer_size = spis_transfer_length;
    }
    return buffer_size;
}
