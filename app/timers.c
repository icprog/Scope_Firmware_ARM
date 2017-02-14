
#include <stdint.h>
#include "app_timer.h"
#include "timers.h"
#include "LSM303drv.h"
#include "L3GD20drv.h"
#include "ble_status.h"
#include "ble_bas.h"
#include "ble_slope.h"
#include "app.h"
#include "SEGGER_RTT.h"
#include "profile_service.h"
#include "nrf_drv_spis.h"
#include "spi_utils.h"
#include "pca10028.h"

static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
extern LSM303_DATA accel_data;
extern L3GD_DATA gyro_data;
extern imu_data_t imu_data;
extern ble_status_t  m_status; //status service
extern ble_bas_t m_bas; //battery service
extern ble_slope_t m_slope; //slope service
extern ble_ps_t m_ps;
extern uint8_t sending_data_to_phone;
extern uint16_t spis_rx_transfer_length;
extern uint8_t status_disable_flag;

/**@brief Function for the Timer initialization.
 * @details Initializes the timer module. This creates and starts application timers.
 */
void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_acc_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                acc_timeout_handler);
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_timeout_handler);
	err_code = app_timer_create(&m_slope_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                slope_timeout_handler);
	err_code = app_timer_create(&m_status_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                status_timeout_handler);
	
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting application timers.
 */
void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, battery_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    
	
	err_code = app_timer_start(m_slope_timer_id, slope_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_start(m_status_timer_id, status_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    

    //err_code = app_timer_start(m_acc_timer_id, acc_LEVEL_MEAS_INTERVAL, NULL);
    //APP_ERROR_CHECK(err_code);
}

void application_timers_stop(void)
{
    uint32_t err_code;

    // Stop application timers.
    err_code = app_timer_stop(m_battery_timer_id);
    APP_ERROR_CHECK(err_code);
    
	
	err_code = app_timer_stop(m_slope_timer_id);
    APP_ERROR_CHECK(err_code);
    
	
	err_code = app_timer_stop(m_status_timer_id);
    APP_ERROR_CHECK(err_code);
    

    err_code = app_timer_stop(m_acc_timer_id);
    APP_ERROR_CHECK(err_code);
}

/********** app timer handlers  ***********/
void acc_timeout_handler(void *p_context)
{
    //NRF_SPIS_Type * p_spis = spis.p_reg;
    
    UNUSED_PARAMETER(p_context);
    accel_data = getLSM303data();
    imu_data.ax = accel_data.X;
    imu_data.ay = accel_data.Y;
    imu_data.az = accel_data.Z;
//    gyro_data = getL3GDdata();
//    imu_data.gx = gyro_data.X;
//    imu_data.gy = gyro_data.Y;
//    imu_data.gz = gyro_data.Z;
    appData.send_imu_flag = true;
    //nrf_gpio_pin_toggle(SPIS_ARM_REQ_PIN); //JUST A TEST
    
}
void enable_imu(void)
{
    uint32_t err_code = app_timer_start(m_acc_timer_id, acc_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    appData.imu_enabled = true;
}

void disable_imu(void)
{
    uint32_t err_code = app_timer_stop(m_acc_timer_id);
    APP_ERROR_CHECK(err_code);
    appData.imu_enabled = false;
}

//This actually handles slope:
void battery_timeout_handler(void *p_context)
{
//    uint8_t slope_local;
//    slope_local = slope_calc(imu_data.ax,imu_data.ay,imu_data.az);
    //ble_slope_level_update(&m_slope, slope_local);
    //ble_bas_battery_level_update(&m_bas, 77);
}

void slope_timeout_handler(void *p_context)
{
    //bunch of stuff that is not related to slope:
    
    static uint8_t ack_count = 0;
    if(appData.ack == 0)
    {
        appData.ack_retry = 1;
    }
    UNUSED_PARAMETER(p_context);
}

void status_timeout_handler(void *p_context)
{

    //stuff that probably should have its own timer but is here instead:
    UNUSED_PARAMETER(p_context);
    if(appData.SPIS_timeout_flag == 1)
    {
        SEGGER_RTT_printf(0, "transfer timed out :(\n");
        appData.transfer_in_progress = false;
        spis_rx_transfer_length = 0;
        appData.state = APP_STATE_SPIS_FAIL;
    }
    appData.SPIS_timeout_flag = 0;
    UNUSED_PARAMETER(p_context);
    
    //actual status update:
    ble_status_status_level_update(&m_status, appData.status);
}
    
void start_ARM_RDY_timer(void)
{		
    NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;  // Set the timer in Counter Mode
    NRF_TIMER2->TASKS_CLEAR = 1;               // clear the task first to be usable for later
	NRF_TIMER2->PRESCALER = TIMER_PRESCALER;                             //Set prescaler. Higher number gives slower timer. Prescaler = 0 gives 16MHz timer
	NRF_TIMER2->BITMODE = TIMER_BITMODE_BITMODE_16Bit;		 //Set counter to 16 bit resolution
	NRF_TIMER2->CC[0] = CC_DELAY;                              //Set value for TIMER2 compare register 0
		
  // Enable interrupt on Timer 2, both for CC[0] and CC[1] compare match events
	NRF_TIMER2->INTENSET = (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);
    NVIC_EnableIRQ(TIMER2_IRQn);
		
    NRF_TIMER2->TASKS_START = 1;               // Start TIMER2
}



/** TIMTER2 peripheral interrupt handler. This interrupt handler is called whenever there it a TIMER2 interrupt
 */
void TIMER2_IRQHandler(void)
{
    static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
    NRF_SPIS_Type * p_spis = spis.p_reg;
	if (NRF_TIMER2->EVENTS_COMPARE[0] != 0)
    {
		NRF_TIMER2->EVENTS_COMPARE[0] = 0;           //Clear compare register 0 event	
        if(nrf_spis_semaphore_status_get(p_spis) != NRF_SPIS_SEMSTAT_FREE)
        {
            if(appData.ack == 0)
            {
                SEGGER_RTT_printf(0, "clearing = %d     ", nrf_spis_semaphore_status_get(p_spis));
            }
            nrf_gpio_pin_clear(SPIS_ARM_RDY_PIN);       
		}
        else
        {
            nrf_gpio_pin_set(SPIS_ARM_RDY_PIN);
        }
        
        NRF_TIMER2->CC[0] += CC_DELAY;
    }
}

