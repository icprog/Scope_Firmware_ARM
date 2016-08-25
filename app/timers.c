
#include <stdint.h>
#include "app_timer.h"
#include "timers.h"
#include "LSM303drv.h"
#include "ble_status.h"
#include "ble_bas.h"
#include "ble_slope.h"
#include "app.h"
#include "SEGGER_RTT.h"

extern LSM303_DATA accel_data;
extern ble_status_t  m_status; //status service
extern ble_bas_t m_bas; //battery service
extern ble_slope_t m_slope; //slope service
extern uint16_t spis_rx_transfer_length;

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
    

    err_code = app_timer_start(m_acc_timer_id, acc_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
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
    UNUSED_PARAMETER(p_context);
    accel_data = getLSM303data();
    //nrf_gpio_pin_toggle(SPIS_RDY_PIN); //JUST A TEST
}
void battery_timeout_handler(void *p_context)
{
    //SEGGER_RTT_printf(0,"batt\n");
    UNUSED_PARAMETER(p_context);
    if(appData.SPIS_timeout_flag == 1)
    {
        SEGGER_RTT_printf(0, "transfer timed out :(\n");
        appData.transfer_in_progress = false;
        spis_rx_transfer_length = 0;
        appData.state = APP_STATE_SPIS_FAIL;
    }
    appData.SPIS_timeout_flag = 0;
    ble_bas_battery_level_update(&m_bas, 77);
}
void slope_timeout_handler(void *p_context)
{
    //SEGGER_RTT_printf(0,"slope\n");
    ble_slope_level_update(&m_slope, 38);
    UNUSED_PARAMETER(p_context);
}
void status_timeout_handler(void *p_context)
{
    //SEGGER_RTT_printf(0,"status\n");
    UNUSED_PARAMETER(p_context);
    ble_status_status_level_update(&m_status, appData.ble_status);
}