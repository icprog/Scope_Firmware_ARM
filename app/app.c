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
extern cal_force_t                           m_force;
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
            if(~NRF_GPIO->IN & 1<<17)
            {
                    //printf("\n\rButton 1 pressed.  Sending LSM303 Initialization SPI package.");
                SEGGER_RTT_WriteString(0, "\n\rButton 1 pressed.  Sending LSM303 Initialization SPI package. \n");
                  init_LSM303();
                  LEDS_INVERT(BSP_LED_0_MASK);
                    while(~NRF_GPIO->IN & 1<<17);
            }
            
            //monitor();
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
            SEGGER_RTT_printf(0, "FORCE_CAL_DATA\n");
            SEGGER_RTT_printf(0, "received force calibration data: ");
            for(int i = 0; i < 7; i++)
            {
                SEGGER_RTT_printf(0, "  %d", cal_data.force_data[i]);
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
                appData.state = APP_STATE_POLLING;
            }
            if(cal_data.hall_status == COMPLETE)
            {
                //communicate that the test passed
                appData.state = APP_STATE_POLLING;
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
								printf("\r\nLSM Who Am I: %x",getLSM303ID());
								printf("\r\nL3G Who Am I: %x",getL3GD_ID());
								prompt();											
						}
				}
		}
}
