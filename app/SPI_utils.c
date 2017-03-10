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
 
  Author(s): 
    Richard Kirby, Joe Trovato
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
#include "L3GD20drv.h"
#include "debug.h"
#include "timers.h"
#include "pcb_test.h"

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG(SPIS_INSTANCE);
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */
uint8_t       m_tx_buf_s[SPIS_BUFFER_MAX];
uint8_t       m_rx_buf_s[SPIS_BUFFER_MAX];
uint8_t       dummy_buf[32];
LSM303_DATA accel_data; //acelerometer data to pass to PIC
L3GD_DATA gyro_data; //gyro data to pass to PIC
imu_data_t imu_data;
uint8_t sending_data_to_phone = 0;
volatile bool device_info_received = false;
volatile bool debug_file_received = false;
extern uint8_t pcb_test_results[NUM_ARM_PCB_TESTS];

/********  global variable for building a tx packet for PIC   **********/
volatile bool raw_data_transfer_in_progress = false;
uint16_t spis_rx_transfer_length = 0;
uint16_t spis_tx_transfer_length = 0;
void * rx_data_ptr; //where to put the data received from the PIC
void * tx_data_ptr; //where to pull data from to send to PIC
char serial_num[6];

/********  pic arm pack varaibles  *******/
pic_arm_pack_t force_cal_init_pack = {PA_FORCE_CAL_INIT, dummy_buf, 0};
pic_arm_pack_t force_cal_weight_pack = {PA_FORCE_CAL_WEIGHT, &(cal_data.current_weight), 1};
pic_arm_pack_t vib_cal_rdy_pack = {PA_VIB_CAL_RDY, dummy_buf, 0};
pic_arm_pack_t optical_cal_length_pack = {PA_OPTICAL_CAL_LENGTH, cal_data.optical_parameters, 3};
pic_arm_pack_t get_profile_pack = {PA_PROFILE, dummy_buf, 4};
pic_arm_pack_t send_device_info_pack = {PA_DEVICE_INFO, dummy_buf, 0};
pic_arm_pack_t accelerometer_pack = {PA_IMU_DATA, (uint8_t *)&imu_data, sizeof(imu_data_t)};
pic_arm_pack_t arm_done_pack = {PA_ARM_DONE, dummy_buf, 0};
pic_arm_pack_t raw_data_ack_pack = {PA_RAW_DATA, dummy_buf, 0};
pic_arm_pack_t profile_id_pack = {PA_PROFILE_ID, (uint8_t *)&(appData.profile_id), sizeof(profile_id_t)};
pic_arm_pack_t location_time_pack = {PA_LOCATION_TIME, (uint8_t *)metadata.location, 12};
pic_arm_pack_t spis_fail_pack = {PA_TIMEOUT, dummy_buf, 0};
pic_arm_pack_t serial_set_pack = {PA_SERIAL_SET, (uint8_t *)&device_info.serial_number, 6};
pic_arm_pack_t xmodem_pack = {PA_XMODEM, dummy_buf, 0};
pic_arm_pack_t start_test_pack = {PA_START_TEST, dummy_buf, 0};
pic_arm_pack_t fwu_start_pack={PA_FWU_START, dummy_buf, sizeof(uint32_t)};
pic_arm_pack_t fwu_data_pack={PA_FWU_DATA, appData.fwu_data_buf, 0}; //will need to update length dynamically
pic_arm_pack_t pcb_test_data_pack = {PA_PCB_TEST_DATA, pcb_test_results, NUM_ARM_PCB_TESTS};
pic_arm_pack_t squal_cal_start_pack = {PA_SQUAL_CAL, dummy_buf, 0};
pic_arm_pack_t set_pic_to_cal_pack = {PA_CAL_MODE, dummy_buf, 0}; // for setting cal mode on PIC
pic_arm_pack_t get_debug_pack = {PA_DEBUG_FILE, dummy_buf, 0}; // for setting cal mode on PIC

extern device_info_t device_info;
extern subsampled_raw_data_t raw_sub_data;
extern profile_data_t profile_data;
extern ble_dbs_t						m_ds;
extern uint8_t * debug_file;

/*
 * build the header packet, enable the RDY line and wait for the PIC to clock in the packet. 
 * Then handle the subsequent data packets in the spis_event_handler.
 */
uint8_t send_data_to_PIC(pic_arm_pack_t pa_pack)
{
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
            set_ARM_REQ(); 
        }
        else if(nrf_spis_semaphore_status_get(p_spis) == NRF_SPIS_SEMSTAT_CPU)
        {
            return 1;
        }
        else
        {
            while(nrf_spis_semaphore_status_get(p_spis) != NRF_SPIS_SEMSTAT_FREE)
            {
                //SEGGER_RTT_printf(0, "sem stat = %d\n", nrf_spis_semaphore_status_get(p_spis));
            }
            set_ARM_REQ(); 
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
    nrf_gpio_pin_clear(SCOPE_SPIS_READY); //clear ready pin
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
            case PA_FORCE_CAL_DATA:
            {
                next_state = APP_STATE_FORCE_CAL_DATA;
                rx_data_ptr = cal_data.force_data;
                break;
            }
            case PA_OPTICAL_CAL_DATA:
            {
                SEGGER_RTT_printf(0, "PA_OPTICAL_CAL_DATA\n");
                next_state = APP_STATE_OPTICAL_CAL_DATA;
                rx_data_ptr = cal_data.optical_data;
                appData.ble_disconnect_flag = false;
                break;
            }
            case PA_DEVICE_STATUS:
            {
                SEGGER_RTT_printf(0, "DEV STATUS\n");
                rx_data_ptr = &(appData.status);
                next_state = appData.state;
                break;
            }
            case PA_DEBUG_FILE:
            {
                SEGGER_RTT_printf(0, "DEBUG FILE\n");
                rx_data_ptr = &debug_file;
                next_state = APP_STATE_DEBUG_REC_TEST;
                break;
            }
            case PA_NEW_ID:
            {
                SEGGER_RTT_printf(0, "PA_NEW_ID\n");
				rx_data_ptr = &(appData.new_profile_num);
				next_state = APP_STATE_NEW_ID;
                break;
            }
            case PA_PROFILE:
            {
                appData.ble_disconnect_flag = false;
                SEGGER_RTT_printf(0, "PA_PROFILE\n");
				rx_data_ptr = &profile_data;
				next_state = APP_STATE_PROFILE_TRANSFER; //TODO
                break;
            }
            case PA_ACCEL_START:
            {
                SEGGER_RTT_printf(0, "PA_ACCEL_START\n");
                enable_imu();
                next_state = appData.state;
                break;
            }
            case PA_ACCEL_STOP:
            {   
                SEGGER_RTT_printf(0, "PA_ACCEL_STOP\n");
                disable_imu();
                next_state = appData.state;
                break;
            }
            case PA_OPTICAL_CAL_RESULT:
            {
                SEGGER_RTT_printf(0, "PA_OPTICAL_CAL_RESULT\n");
                rx_data_ptr = &(cal_data.optical_result);
                next_state = APP_STATE_OPTICAL_CAL_RESULT;
                break;
            }
            case PA_OPTICAL_CAL_CONST:
            {
                SEGGER_RTT_printf(0, "PA_OPTICAL_CAL_CONST\n");
                rx_data_ptr = &(cal_data.optical_const);
                next_state = appData.state;
                break;
            }
            case PA_SQUAL_CAL:
            {
                rx_data_ptr = cal_data.squal_and_pic;
                next_state = APP_STATE_SQUAL_CAL_RESULT;
                appData.ble_disconnect_flag = false;
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
				appData.ble_disconnect_flag = false;
                break;
			}
            case PA_PROBE_ERROR:
            {
                SEGGER_RTT_printf(0, "PA_PROBE_ERROR\n");
                rx_data_ptr = &(metadata.error_code);
                next_state = APP_STATE_PROBE_ERROR;
                break;
            }
            case PA_RESTART:
            {
                SEGGER_RTT_printf(0, "PA_RESTART\n");
                next_state = appData.state;
                break;
            }
            case PA_ACK:
            {
                SEGGER_RTT_printf(0, "PA_ACK\n");
                appData.ack = 1;
                next_state = appData.state;
                break;
            }
            case PA_FWU_ACK:
            {
                //SEGGER_RTT_printf(0, "PA_FWU_ACK\n");
                appData.ack = 1;
                next_state = APP_STATE_FWU_ACK;
                break;
            }
            case PA_FWU_DONE:
            {
                next_state = APP_STATE_FWU_DONE;
                break;
            }
            default:
            {
                SEGGER_RTT_printf(0, "SPIS ERROR: code not recognized\n");
                next_state = appData.state;
                break;
            }
            nrf_gpio_pin_set(SCOPE_SPIS_READY); //set ready pin
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
        appData.SPIS_timeout_flag = 0;
        if(spis_rx_transfer_length == 0) //finished transferring
        {
            nrf_gpio_pin_set(SCOPE_SPIS_READY); //set ready pin
            SEGGER_RTT_printf(0, "finished transferring, cur state %d, next state %d\n", appData.state, next_state);
            appData.transfer_in_progress = false;
            appData.state = next_state;
        }
    }
    else
    {
        /* This case gets triggered whenever we send a header packet to the PIC becuase we 
        also clock in a garbage packet on our end and try to parse it */
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
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {
        /********* determine length of packet received  *********/
        rx_length = buffer_size_calc(spis_rx_transfer_length);
        if(spis_rx_transfer_length != 0)
        {
            appData.SPIS_timeout_flag = 1;
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
            clear_ARM_REQ(); ///TODO: REQ not RDY
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
		spi_config.ss_pin = NRF_DRV_SPI_PIN_NOT_USED;
		
	  /* TODO(rk): turn off CS pin in master SPI driver */
		spi_config.mode = NRF_DRV_SPI_MODE_3;
		APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
		
		nrf_gpio_cfg_output(IMU_SPI_CS_ACC_PIN);
		nrf_gpio_cfg_output(IMU_SPI_CS_GYRO_PIN);
	
        NRF_GPIO->OUTSET = (1<<IMU_SPI_CS_ACC_PIN);
        NRF_GPIO->OUTSET = (1<<IMU_SPI_CS_GYRO_PIN);
		
//		printf("\n\r\n\rSPI Master Configuration:");
//		printf("\n\r  SCK pin: %d", spi_config.sck_pin);
//		printf("\n\r  MOSI pin: %d", spi_config.mosi_pin);
//		printf("\n\r  MISO pin: %d", spi_config.miso_pin);
//		printf("\n\r  Chip Select pin: %d", spi_config.ss_pin);
//		printf("\n\r  SPI Mode: %d", spi_config.mode);
}

void spi_un_init(void)
{
    nrf_drv_spi_uninit(&spi);
    //nrf_drv_spis_uninit(&spis);
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
    
    /******* initialize extra pin  ********/
    nrf_gpio_cfg_output(SPIS_ARM_REQ_PIN); //TODO change to REQ 
    nrf_gpio_cfg_output(SPIS_ARM_RDY_PIN);
    start_ARM_RDY_timer();
    
   // SEGGER_RTT_printf(0,"\nSPI RDY pin: %d", SPIS_ARM_REQ_PIN);
}


uint8_t SPIReadByte(uint8_t address, SPI_DEVICE device)
{
	uint8_t rx_buf[2], tx_buf[2], CS_pin;
	uint16_t spi_timeout = 0;
	if (device == LSM_DEVICE)
		CS_pin = IMU_SPI_CS_ACC_PIN;
	else if (device == L3G_DEVICE)
		CS_pin = IMU_SPI_CS_GYRO_PIN;
	
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
			CS_pin = IMU_SPI_CS_ACC_PIN;   
		else if (device == L3G_DEVICE)
			CS_pin = IMU_SPI_CS_GYRO_PIN;
	
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

/* wait for the semaphore to be free, then give the PIC the OK! */
void set_ARM_RDY(void)
{
    NRF_SPIS_Type * p_spis = spis.p_reg;
    while(nrf_spis_semaphore_status_get(p_spis) != NRF_SPIS_SEMSTAT_FREE);
    nrf_gpio_pin_set(SPIS_ARM_RDY_PIN);
}

void clear_ARM_RDY(void)
{
    nrf_gpio_pin_clear(SPIS_ARM_RDY_PIN);
}

void set_ARM_REQ(void)
{
    nrf_gpio_pin_set(SPIS_ARM_REQ_PIN);
    //SEGGER_RTT_printf(0, "\n setting RDY ");
}
void clear_ARM_REQ(void)
{
    nrf_gpio_pin_clear(SPIS_ARM_REQ_PIN);
    //SEGGER_RTT_printf(0, "\n clearing RDY ");
}
bool get_ARM_REQ(void)
{
    return (bool)(NRF_GPIO->OUT & (1 << SPIS_ARM_REQ_PIN));
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
