/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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
/**
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_slope.h"
#include "ble_status.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
//#include "ble_dev_status.h"
#include "SEGGER_RTT.h"
#include "LSM303drv.h"
#include "nrf_delay.h"
#include "spi_utils.h"
#include "app.h"
#include "calibration.h"

//services
#include "probe_error.h"
#include "profile_service.h"
#include "cal_optical.h"
#include "cal_force.h"
#include "cal_vib.h"
#include "cal_hall_effect.h"
#include "probe_error.h"
#include "profile_service.h"

/*Addition to do beacon non connectable advertising at all time*/
#include "advertiser_beacon.h"
#define IS_SRVC_CHANGED_CHARACT_PRESENT      0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/
#define CENTRAL_LINK_COUNT                   0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT                1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/
#define APP_COMPANY_IDENTIFIER               0x0059                                     /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define BEACON_UUID 0x00, 0x00, 0x29, 0x02, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb			//put in place to match the app							
										
#define BEACON_ADV_INTERVAL                  400                                        /**< The Beacon's advertising interval, in milliseconds*/
#define BEACON_MAJOR                         0x1234                                     /**< The Beacon's Major*/
#define BEACON_MINOR                         0x5678                                     /**< The Beacon's Minor*/
#define BEACON_RSSI                          0xC3                                       /**< The Beacon's measured RSSI at 1 meter distance in dBm. */


static ble_beacon_init_t beacon_init;
/*end addition for beacon*/


/****** parameters for application timers  ******/
#define APP_ADV_INTERVAL                     480                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 300 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           180                                         /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER                  3                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE              4                                          /**< Size of timer operation queues. */

#define battery_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */                         
#define slope_LEVEL_MEAS_INTERVAL          	 APP_TIMER_TICKS(500, APP_TIMER_PRESCALER) /**< slope level measurement interval (ticks). */
#define status_LEVEL_MEAS_INTERVAL           APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< status level measurement interval (ticks). */
#define acc_LEVEL_MEAS_INTERVAL              APP_TIMER_TICKS(2, APP_TIMER_PRESCALER)

/*********  BLE connection params  ******/
#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(50, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(50, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                        0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                     MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY        APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT         3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                       1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                       0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                       0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                   0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES            BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                        0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE               7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE               16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                            0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


extern LSM303_DATA accel_data;
device_info_t device_info;
extern uint8_t dummy_buf[32];
extern uint8_t sending_data_to_phone;
pic_arm_pack_t send_device_info_pack = {PA_DEVICE_INFO, dummy_buf, 0};

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
static ble_pes_t 	 						 m_pes; //probing error service
ble_ps_t                             		 m_ps; //profile service
static ble_slope_t                           m_slope; //slope service
static ble_status_t							 m_status;
cal_optical_t								 m_optical;
cal_force_t			    					 m_force;
cal_hall_effect_t						     m_hall_effect;
static cal_vib_t							 m_vib;    //vibration motor cal struct

//defines variables to be used for app timers
APP_TIMER_DEF(m_acc_timer_id);
APP_TIMER_DEF(m_slope_timer_id);
APP_TIMER_DEF(m_battery_timer_id);                                                      /**< Battery timer. */
APP_TIMER_DEF(m_status_timer_id);   
/**< Status timer. */

static dm_application_instance_t             m_app_handle;                              /**< Application identifier allocated by device manager. */

static ble_uuid_t m_adv_uuids[] =                                                       /**< Universally unique service identifiers. */
{
	{SCOPE_UUID_SLOPE,                    BLE_UUID_TYPE_BLE},
    {SCOPE_UUID_BATTERY,                  BLE_UUID_TYPE_BLE},
    {SCOPE_UUID_DEVICE_INFO, 			  BLE_UUID_TYPE_BLE},
	{SCOPE_UUID_STATUS, 				  BLE_UUID_TYPE_BLE},
	{PROBE_ERROR_SERVICE_UUID,			  BLE_UUID_TYPE_BLE},
    {PROFILE_SERVICE_UUID,                BLE_UUID_TYPE_BLE},
    //{BLE_UUID_CAL_OPTICAL_SERVICE,        BLE_UUID_TYPE_BLE},
		
};

uint8_t SLOPE_GLOBAL = 0;

/**@brief Callback function for asserts in the SoftDevice.
 * @details This function will be called in case of an assert in the SoftDevice.
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}
/********** app timer handlers  ***********/
static void acc_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
    accel_data = getLSM303data();
    //nrf_gpio_pin_toggle(SPIS_RDY_PIN); //JUST A TEST
}
static void battery_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
}
static void slope_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
}
static void status_timeout_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);
}

/**@brief Function for the Timer initialization.
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
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

static void device_name_update(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_advdata_t advdata;
    
    //change name
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)device_info.device_name,
                                          strlen(device_info.device_name));
    APP_ERROR_CHECK(err_code);
    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_advdata_set(&advdata, NULL); //scan response data is NULL
    SEGGER_RTT_printf(0, "device name = ");
    for(int i = 0; i < 10; i++)
    {
        SEGGER_RTT_printf(0, "%c", device_info.device_name[i]);
    }
}
/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

//    SEGGER_RTT_printf(0, "serial number in gap init = ");
//    for(int i = 0; i < 5; i++)
//    {
//        SEGGER_RTT_printf(0, "%c", device_info.serial_number[i]);
//    }
//    SEGGER_RTT_printf(0, "\n");
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)device_info.device_name,
                                          strlen(device_info.device_name));
                                          
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

//void force_write_handler(cal_force_t * p_force, uint8_t data_in)
//{
//		SEGGER_RTT_printf(0,"input: %d",data_in);
//}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery, Slope and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
	
    // Initialize Device Information Service.
    ble_dis_init_t dis_init;
    memset(&dis_init, 0, sizeof(dis_init));
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
    
    //battery service init:
    ble_bas_init_t bas_init;
    memset(&bas_init, 0, sizeof(bas_init));
    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
    
    if(CALIBRATION)
    {
        // Initialize Optical Cal. service
        cal_optical_init_t optical_init;
        memset(&optical_init, 0, sizeof(optical_init));
        err_code = cal_optical_init(&m_optical, &optical_init);
        APP_ERROR_CHECK(err_code);


        // Initialize Force Cal. service
        cal_force_init_t force_init;
        memset(&force_init, 0, sizeof(force_init));
        err_code = cal_force_init(&m_force, &force_init);
        APP_ERROR_CHECK(err_code);
                
        // Initialize vib Cal. service
        cal_vib_init_t vib_init;
        memset(&vib_init, 0, sizeof(vib_init));
        err_code = cal_vib_init(&m_vib, &vib_init);
        APP_ERROR_CHECK(err_code);

        // Initialize cal hall effect service.
        cal_hall_effect_init_t hall_effect_init;
        memset(&hall_effect_init, 0, sizeof(hall_effect_init));
        err_code = cal_hall_effect_init(&m_hall_effect, &hall_effect_init);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
	
    // Initialize Slope Service.
    ble_slope_init_t slope_init;
    memset(&slope_init, 0, sizeof(slope_init));
    err_code = ble_slope_init(&m_slope, &slope_init);
    APP_ERROR_CHECK(err_code);
        
    // Initialize Device Status Service.
    ble_status_init_t status_init;
    memset(&status_init, 0, sizeof(status_init));
    err_code = ble_status_init(&m_status, &status_init);
    APP_ERROR_CHECK(err_code);
	
	//initialize probe error service
    ble_probe_error_service_init(&m_pes);
    
    //initialize profile service
    ble_profile_service_init(&m_ps);
    }

}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
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


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    //cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;
    //SEGGER_RTT_WriteString(0, "BLE evt (no write fxn) \n");
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            app_beacon_start();
            
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);

            app_beacon_stop();
        
            // when not using the timeslot implementation, it is necessary to initialize the advertizing data again.
            advertising_init();
            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        
            break;
				case BLE_EVT_TX_COMPLETE:
					if(sending_data_to_phone) appData.state = APP_STATE_RAW_DATA_RECEIVE; //TODO make this better
        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    
    dm_ble_evt_handler(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);

    if(CALIBRATION)
    {
        cal_vib_on_ble_evt(&m_vib,p_ble_evt);
        cal_hall_effect_on_ble_evt(&m_hall_effect, p_ble_evt);
        cal_optical_on_ble_evt(&m_optical,p_ble_evt);
        cal_force_on_ble_evt(&m_force,p_ble_evt);
    }
    else
    {
        ble_bas_on_ble_evt(&m_bas, p_ble_evt);
        ble_slope_on_ble_evt(&m_slope, p_ble_evt);
        ble_status_on_ble_evt(&m_status, p_ble_evt);
        ble_probe_error_service_on_ble_evt(&m_pes, p_ble_evt);
        ble_profile_service_on_ble_evt(&m_ps, p_ble_evt);
    }
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    app_beacon_on_sys_evt(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in]   p_evt   Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const    * p_handle,
                                           dm_event_t const     * p_event,
                                           ret_code_t           event_result)
{
    APP_ERROR_CHECK(event_result);
    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t  register_param;
    
    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling a BeaconAdvertiser error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void beacon_advertiser_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing Beacon advertiser.
 */
static void beacon_adv_init(void)
{
    static uint8_t beacon_uuid[] = {BEACON_UUID};
    
    memcpy(beacon_init.uuid.uuid128, beacon_uuid, sizeof(beacon_uuid));
    beacon_init.adv_interval  = BEACON_ADV_INTERVAL;
    beacon_init.major         = BEACON_MAJOR;
    beacon_init.minor         = BEACON_MINOR;
    beacon_init.manuf_id      = APP_COMPANY_IDENTIFIER;
    beacon_init.rssi          = BEACON_RSSI;
    beacon_init.error_handler = beacon_advertiser_error_handler;
    
    uint32_t err_code = sd_ble_gap_address_get(&beacon_init.beacon_addr);
    APP_ERROR_CHECK(err_code);
    
    app_beacon_init(&beacon_init);
}

void device_info_update(void)
{
    device_name_update();
}

/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    nrf_drv_gpiote_out_toggle(SCOPE_3V3_ENABLE_PIN);
}
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output, 
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void shutdown_gpio_init(void)
{
    ret_code_t err_code;

    //init gpiote module if not already initialized
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }
    
    uint32_t out_pin_initial_state = 0; //for the 3v3 enable
    
    /********** HALL INPUT PIN  *******/
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

    err_code = nrf_drv_gpiote_in_init(SCOPE_HALL_PIN, &in_config, in_pin_handler); //set pin and event handler
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(SCOPE_HALL_PIN, true); //enable event handling
    
    if(nrf_drv_gpiote_in_is_set(SCOPE_HALL_PIN))
    {
        out_pin_initial_state = 1; //if HALL is high then the bullet is out of the pole so enable the supply
    }
    
    /********** 3V3 ENABLE OUTPUT PIN   **********/
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(out_pin_initial_state); //config output with with correct initial state

    err_code = nrf_drv_gpiote_out_init(SCOPE_3V3_ENABLE_PIN, &out_config);
    APP_ERROR_CHECK(err_code);
}


void init_device_info(void)
{
    strcpy(device_info.serial_number, "NO SN");
    strcpy(device_info.device_name, "SCOPE NO SN");
    send_data_to_PIC(send_device_info_pack);
    //wait for PIC to respond with device info
    if(!transfer_in_progress)
    {
        while(!transfer_in_progress);
    }
    while(transfer_in_progress);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;

 // Initialize.
    timers_init();
    shutdown_gpio_init();
    ble_stack_init();
    beacon_adv_init();
    device_manager_init(erase_bonds);

    strcpy(device_info.serial_number, "NO SN");
    strcpy(device_info.device_name, "FUCK YOU");
    //init_device_info();
    
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
    
    APP_Initialize();
	
    // Start execution.
    application_timers_start();
    //err_code = ble_advertising_start(BLE_ADV_MODE_FAST); //TODO: advertize
    APP_ERROR_CHECK(err_code);

	SEGGER_RTT_WriteString(0, "main loop:\n");

    while(true)
    {
        if(accel_flag)
        {
            accel_flag = 0;
        }
        APP_Tasks();
        //power_manage();
	}
}

