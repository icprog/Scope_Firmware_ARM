/*******************************************************************************
  Application Source File
  
  Company:
    AvaTech
  
  File Name:
    app.c
		
  Summary:
    This file contains the source code for the Servo Controller
    application.
		
  Description:
    This file contains the source code for the Servo Controller 
    application.  It implements the logic of the application's state machine and 
    it may call API routines of other modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
     
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

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "boards.h"
#include <string.h>
#include "SPI_utils.h"
#include "LSM303drv.h"
#include "L3GD20drv.h"
#include "nrf_drv_spis.h"
#include "SEGGER_RTT.h" 
#include "app.h"
#include "SPI_utils.h"
#include "nrf_drv_config.h"
#include "calibration.h"
#include "cal_force.h"
#include "nrf_drv_gpiote.h" //for the hall effect test
#include "cal_optical.h"
#include "cal_hall_effect.h"
#include "pcb_test.h"
#include "profile_service.h"
#include "nrf_nvic.h"
#include "ble_err.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
extern volatile bool spis_xfer_done; 													/**< Flag used to indicate that SPIS instance completed the transfer. */
extern nrf_drv_spis_config_t spis_config;
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);	            /**< SPIS instance. */
extern uint8_t       m_tx_buf_s[4];           											/**< TX buffer. */
extern uint8_t       m_rx_buf_s[5];    													/**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf_s);        								/**< Transfer length. */
extern cal_force_t                             m_force;
extern cal_optical_t													 m_optical;
extern cal_hall_effect_t											 m_hall_effect;
extern ble_ps_t                            		 m_ps;
extern uint8_t			profile_data_in[1500]; // holder for profile data from PIC
extern uint16_t     profile_block_counter; //keeps track of current block of 250 bytes
extern uint8_t sending_data_to_phone;
//extern nrf_drv_spis_t spis;/**< SPIS instance. */

extern LSM303_DATA accel_data; //acelerometer data to pass to PIC
uint8_t pcb_test_results[NUM_ARM_PCB_TESTS];
extern pic_arm_pack_t accelerometer_pack;
extern void * tx_data_ptr; //where to pull data from to send to PIC

subsampled_raw_data_t raw_data;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;


// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************


/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void)
{
		SEGGER_RTT_WriteString(0, "Init Start \n");
        /* Place the App state machine in its initial state. */
        appData.state = APP_STATE_INIT;		
	
		spi_init();
		spis_init();
		spis_xfer_done = false;
		profile_block_counter = 0;
		init_LSM303();
		SEGGER_RTT_WriteString(0, "Init End \n");
}


/******************************************************************************
  Function:
    void APP_Tasks(void)

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void)
{   
	uint16_t kk;
	uint8_t * p_is_nested_critical_region;
	
	//SEGGER_RTT_WriteString(0, "App tasks start \n");
    switch (appData.state)
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            SEGGER_RTT_WriteString(0, "init state \n");
            //prompt();
            appData.state = APP_STATE_POLLING;
            break;
        }
        case APP_STATE_POLLING:
        {
            //monitor();
            break;
        }
				case APP_STATE_PROFILE_TRANSFER:
				{
					//handle incoming profile data from PIC here
					//profile_data_in[];
					//profile_block_counter++; //keeps track of current block of 250 bytes
//				SEGGER_RTT_printf(0, "app data counter %d \n",profile_block_counter);
//				if(profile_block_counter >= 5)
//				{
//						profile_block_counter = 0;
//						SEGGER_RTT_WriteString(0, "Finished receiving profile from PIC \n");
//						for(kk=0;kk<75;kk+=20)profile_data_update(&m_ps, &profile_data_in[kk],20);
//				}
					SEGGER_RTT_WriteString(0, "Finished receiving profile \n");
					for(kk=0;kk<1500;kk+=20) profile_data_update(&m_ps, &profile_data_in[kk],20);  //notify phone with profile data
					SEGGER_RTT_WriteString(0, "Finished sending profile \n");
					appData.state = APP_STATE_POLLING;
					break;
				}
				case APP_STATE_ACCELEROMETER:
				{

						//sd_nvic_critical_region_enter(p_is_nested_critical_region);
						//accel_data = getLSM303data();
						//sd_nvic_critical_region_exit(0);
						//SEGGER_RTT_printf(0, "PA_ACCELEROMETER_DATA\n");
						//SEGGER_RTT_printf(0, "PA_ACCELEROMETER_DATA: %d \n",accel_data.Y); //TODO: remove
						//tx_data_ptr = &accel_data;

						//nrf_delay_us(90); //TODO: remove
						send_data_to_PIC(accelerometer_pack);
						//SEGGER_RTT_printf(0, "sent accelerometer pack\n");
						appData.state = APP_STATE_POLLING;
					break;
				}
//        case APP_STATE_VIB_CAL_RDY:
//        {
//            SEGGER_RTT_printf(0, "VIB_CAL_RDY\n");
//            send_data_to_PIC(vib_cal_rdy_pack);
//            appData.state = APP_STATE_POLLING;
//            break;
//        }
//        case APP_STATE_FORCE_CAL_WEIGHT:
//        {
//            SEGGER_RTT_printf(0, "FORCE_CAL_WEIGHT\n");
//            send_data_to_PIC(force_cal_weight_pack);
//            appData.state = APP_STATE_POLLING;
//            break;
//        }
//        case APP_STATE_OPTICAL_CAL_LENGTH:
//        {
//            SEGGER_RTT_printf(0, "OPTICAL_CAL_LENGTH\n");
//            send_data_to_PIC(optical_cal_length_pack);
//            appData.state = APP_STATE_POLLING;
//            break;
//        }
        case APP_STATE_FORCE_CAL_DATA:
        {
						uint16_t test[7];
            SEGGER_RTT_printf(0, "FORCE_CAL_DATA\n");
            SEGGER_RTT_printf(0, "received force calibration data: ");
            for(int i = 0; i < 7; i++)
            {
                SEGGER_RTT_printf(0, "  %d", cal_data.force_data[i]);
								test[i] = i;
            }

						cal_points_update(&m_force, cal_data.force_data);
            appData.state = APP_STATE_POLLING;
            break;
        }
        case APP_STATE_HALL_EFFECT_TEST:
        {
            if(nrf_drv_gpiote_in_is_set(SCOPE_HALL_PIN)) //if HALL is set then the bullet is out of the pole
            {
                //communicate that the test failed!
								cal_result_update(&m_hall_effect, 0);
                appData.state = APP_STATE_POLLING;
            }
            if(cal_data.hall_status == 0) //test complete
            {
                //communicate that the test passed
								cal_result_update(&m_hall_effect, 1);
                appData.state = APP_STATE_POLLING;
            }
            break;
        }
        case APP_STATE_OPTICAL_CAL_DATA:
        {
            SEGGER_RTT_printf(0, "OPTICAL_CAL_DATA\n");
            optical_cal_update(&m_optical, cal_data.optical_data);
            appData.state = APP_STATE_POLLING;
            break;
        }
        case APP_STATE_OPTICAL_CAL_RESULT:
        {
            SEGGER_RTT_printf(0, "OPTICAL_CAL_RESULT\n");
            optical_cal_result_update(&m_optical, cal_data.optical_result);
            appData.state = APP_STATE_POLLING;
            break;
        }
        case APP_STATE_HALL_EFFECT_RESULT:
        {
            cal_result_update(&m_hall_effect, cal_data.hall_result);
            break;
        }
        case APP_STATE_PCB_TEST:
        {
            run_pcb_tests(pcb_test_results);
            pic_arm_pack_t pcb_test_data_pack = {PA_PCB_TEST_DATA, pcb_test_results, NUM_ARM_PCB_TESTS};
            send_data_to_PIC(pcb_test_data_pack); //send pcb test data back to PIC
            appData.state = APP_STATE_POLLING;
            break;
        }
        case APP_STATE_DEVICE_INFO:
        {
            SEGGER_RTT_printf(0, "\nserial number = ");
            for(int i = 0; i < 5; i++)
            {
                SEGGER_RTT_printf(0, "%c", device_info.serial_number[i]);
            }
            SEGGER_RTT_printf(0, "\ndevice name = ");
            for(int i=0; i < 32; i++)
            {
                SEGGER_RTT_printf(0, "%c", device_info.device_name[i]);
            }
            
            appData.state = APP_STATE_POLLING;
            break;
        }
        case APP_STATE_RAW_DATA_RECEIVE:
        {
            uint8_t bytes_sent = 0;
            static int data_counts = 0;
            uint8_t counter = 0;
            uint32_t err_code;
            uint8_t done_flag = 0;
             sending_data_to_phone = 1;
            uint8_t * raw_data_ptr = (uint8_t *)&raw_data;
            //nrf_drv_spis_uninit(&spis);
            
            while(data_counts<BYTES_RAW_DATA)
            {      
                err_code = raw_data_update(&m_ps, (uint8_t *)(&raw_data)+data_counts, 20, &bytes_sent);  //notify phone with raw data
				data_counts += bytes_sent;			
                if(data_counts >= BYTES_RAW_DATA)
                {
                    done_flag = 1;
                    appData.state = APP_STATE_POLLING;
                    sending_data_to_phone = 0;
                    SEGGER_RTT_printf(0, "data_counts = %d\n", data_counts);

                }
                if(err_code == BLE_ERROR_NO_TX_PACKETS || counter == 3)
                {
                    SEGGER_RTT_printf(0, "data_counts = %d\n", data_counts);
                    break;
                    
                }
                counter++;
                //SEGGER_RTT_printf(0, "\n data: %d",data_counts);
            }		
            if((err_code == BLE_ERROR_NO_TX_PACKETS  || counter == 3) && !done_flag)
            {
                counter = 0;
                appData.state = APP_STATE_POLLING;
                break;
            }
            break;
        }

        default:
        {
            break;
        }
    }
}


/*******************************************************************************
  Function:
    void prompt(void)

  Remarks:
    See prototype in app.h.
 */

void prompt(void)
{
	printf("\n\r\n\rScope nRF>>");
}


/*******************************************************************************
  Function:
    void monitor(void)

  Remarks:
    See prototype in app.h.
 */

void monitor(void)
{
		uint8_t cr;
    if(app_uart_get(&cr) == NRF_SUCCESS)
		{
				while(app_uart_put(cr) != NRF_SUCCESS) {};
					
				switch (cr)
				{		
						case '?':
						{
								printf("\n\r\n\r?  Help");
								printf("\n\rb  Get button state.");
								printf("\n\rc  Configure LSM303D.");
								printf("\n\rg  Get IMU Data.");
								printf("\n\rw  IMU who am I.");
								prompt();
								break;
						}
						
						// get button state
						case 'b':
						{
								if(~NRF_GPIO->IN & 1<<17)
										printf("\n\rButton 1 pressed");
								if(~NRF_GPIO->IN & 1<<18)
										printf("\n\rButton 2 pressed");
								if(~NRF_GPIO->IN & 1<<19)
										printf("\n\rButton 3 pressed");
								if(~NRF_GPIO->IN & 1<<20)
										printf("\n\rButton 4 pressed");
								SEGGER_RTT_WriteString(0, "Button pressed\n");
								break;
						}
						
						// configure LSM303w
						case 'c':
						{
								init_LSM303();					
								printf("\r\nLSM Ctrl post init: %x",SPIReadByte(LSM_CTRL1, LSM_DEVICE));
								init_L3GD();
								printf("\r\nL3GD Ctrl post init: %x",SPIReadByte(L3GD_CTRL1, L3G_DEVICE));
						}
						
						case 'g':
						{
								LSM303_DATA data;
								data = getLSM303data();
								printf("\r\nAccelerometer X axis: %d",data.X);
								printf("\r\nAccelerometer Y axis: %d",data.Y);
								printf("\r\nAccelerometer Z axis: %d",data.Z);
							
								L3GD_DATA data2;
								data2 = getL3GDdata();
								printf("\r\nGyro X axis: %d",data2.X);
								printf("\r\nGyro Y axis: %d",data2.Y);
								printf("\r\nGyro Z axis: %d",data2.Z);
								prompt();
						}
						
						case 'w':
						{
								printf("\r\nLSM Who Am I: %x",get_LSM303_ID());
								printf("\r\nL3G Who Am I: %x",get_L3GD_ID());
								prompt();											
						}
				}
		}
}
