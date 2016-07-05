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
#include "probe_error.h"
#include "profile_service.h"
#include "LSM303drv.h"
#include "nrf_delay.h"
#include "spi_utils.h"
#include "app.h"
//#include "SEGGER_RTT_printf.h"

//services
#include "probe_error.h"
#include "profile_service.h"

#include "cal_optical.h"
#include "cal_force.h"

/*Addition to do beacon non connectable advertising at all time*/
#include "advertiser_beacon.h"


#define IS_SRVC_CHANGED_CHARACT_PRESENT      0                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT                   0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT                1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define APP_COMPANY_IDENTIFIER               0x0059                                     /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */

//#define BEACON_UUID 0xff, 0xfe, 0x2d, 0x12, 0x1e, 0x4b, 0x0f, 0xa4,\
                    0x99, 0x4e, 0xce, 0xb5, 0x31, 0xf4, 0x05, 0x45 
#define BEACON_UUID 0x00, 0x00, 0x29, 0x02, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5f, 0x9b, 0x34, 0xfb			//put in place to match the app							
										
#define BEACON_ADV_INTERVAL                  400                                        /**< The Beacon's advertising interval, in milliseconds*/
#define BEACON_MAJOR                         0x1234                                     /**< The Beacon's Major*/
#define BEACON_MINOR                         0x5678                                     /**< The Beacon's Minor*/
#define BEACON_RSSI                          0xC3                                       /**< The Beacon's measured RSSI at 1 meter distance in dBm. */


static ble_beacon_init_t beacon_init;


/*end addition for beacon*/



#define APP_ADV_INTERVAL                     480                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 300 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS           180                                         /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER                  0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE              4                                          /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL          APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL                    81                                         /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                    100                                        /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT              1                                          /**< Increment between each simulated battery level measurement. */

#define slope_LEVEL_MEAS_INTERVAL          	 APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< slope level measurement interval (ticks). */
#define MIN_slope_LEVEL                   	 81                                         /**< Minimum simulated slope level. */
#define MAX_slope_LEVEL                      100                                        /**< Maximum simulated slope level. */
#define slope_LEVEL_INCREMENT                1                                          /**< Increment between each simulated slope level measurement. */

#define status_LEVEL_MEAS_INTERVAL          	 APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER) /**< status level measurement interval (ticks). */
#define MIN_status_LEVEL                   	 81                                         /**< Minimum simulated status level. */
#define MAX_status_LEVEL                      100                                        /**< Maximum simulated status level. */
#define status_LEVEL_INCREMENT                1                                          /**< Increment between each simulated status level measurement. */

#define HEART_RATE_MEAS_INTERVAL             APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER) /**< Heart rate measurement interval (ticks). */
#define MIN_HEART_RATE                       140                                        /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                       300                                        /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                 10                                         /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL                 APP_TIMER_TICKS(300, APP_TIMER_PRESCALER)  /**< RR interval interval (ticks). */
#define MIN_RR_INTERVAL                      100                                        /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                      500                                        /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT                1                                          /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL     APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                    MSEC_TO_UNITS(500, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL                    MSEC_TO_UNITS(1000, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (1 second). */
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

static uint16_t                              m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */
static ble_bas_t                             m_bas;                                     /**< Structure used to identify the battery service. */
static ble_pes_t 						     						 m_pes; //probing error service
static ble_ps_t                              m_ps; //profile service
static ble_hrs_t                             m_hrs;                                     /**< Structure used to identify the heart rate service. */
static ble_slope_t                           m_slope;
static ble_status_t													 m_status;
static cal_optical_t												 m_optical;
static cal_force_t													 m_force;
static bool                                  m_rr_interval_enabled = true;              /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static sensorsim_cfg_t                       m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t                     m_battery_sim_state;                       /**< Battery Level sensor simulator state. */

static sensorsim_cfg_t                       m_slope_sim_cfg;                         /**< Slope sensor simulator configuration. */
static sensorsim_state_t                     m_slope_sim_state;                       /**< Slope sensor simulator state. */

static sensorsim_cfg_t                       m_status_sim_cfg;                         /**< status sensor simulator configuration. */
static sensorsim_state_t                     m_status_sim_state;                       /**< status sensor simulator state. */

static sensorsim_cfg_t                       m_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t                     m_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t                       m_rr_interval_sim_cfg;                     /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t                     m_rr_interval_sim_state;                   /**< RR Interval sensor simulator state. */

APP_TIMER_DEF(m_battery_timer_id);                                                      /**< Battery timer. */

APP_TIMER_DEF(m_slope_timer_id);                                                        /**< Slope timer. */
APP_TIMER_DEF(m_status_timer_id);                                                        /**< Status timer. */
APP_TIMER_DEF(m_rr_interval_timer_id);                                                  /**< RR interval timer. */
APP_TIMER_DEF(m_sensor_contact_timer_id);                                               /**< Sensor contact detected timer. */

static dm_application_instance_t             m_app_handle;                              /**< Application identifier allocated by device manager. */

static ble_uuid_t m_adv_uuids[] =                                                       /**< Universally unique service identifiers. */
{
		{SCOPE_UUID_SLOPE,                BLE_UUID_TYPE_BLE},
    {SCOPE_UUID_BATTERY,                  BLE_UUID_TYPE_BLE},
    {SCOPE_UUID_DEVICE_INFO, 			  BLE_UUID_TYPE_BLE},
	{SCOPE_UUID_STATUS, 				  BLE_UUID_TYPE_BLE},
	{PROBE_ERROR_SERVICE_UUID,			  BLE_UUID_TYPE_BLE},
    {PROFILE_SERVICE_UUID,                BLE_UUID_TYPE_BLE}
		
};

uint8_t SLOPE_GLOBAL = 0;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    uint32_t err_code;
    uint8_t  battery_level;

    //battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
		battery_level = SLOPE_GLOBAL;
    err_code = ble_bas_battery_level_update(&m_bas, battery_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
    
    //TESTING THE PROBE ERROR UPDATE
    uint8_t probe_error_code = battery_level;
    err_code = ble_probe_error_update(&m_pes, probe_error_code);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for performing status measurement and updating the status Level characteristic
 *        in status Service.
 */
static void status_level_update(void)
{
    uint32_t err_code;
    uint8_t  status_level;

    status_level = (uint8_t)sensorsim_measure(&m_status_sim_state, &m_status_sim_cfg);

    err_code = ble_status_status_level_update(&m_status, status_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for performing slope measurement and updating the slope Level characteristic
 *        in slope Service.
 */
static void slope_level_update(void)
{
    uint32_t err_code;
    uint8_t  slope_level;
	//LSM303_DATA test_data_303;
		
    slope_level = (uint8_t)sensorsim_measure(&m_slope_sim_state, &m_slope_sim_cfg);
	//set slope based on lsm303 data:
//		test_data_303 = getLSM303data();
//		slope_level = 0;
//		slope_level = (uint8_t)test_data_303.X;
	slope_level = SLOPE_GLOBAL;
    err_code = ble_slope_slope_level_update(&m_slope, slope_level);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING) 
    )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires. 
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}

/**@brief Function for handling the slope measurement timer timeout.
 *
 * @details This function will be called each time the slope level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void slope_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    slope_level_update();
}


/**@brief Function for handling the status measurement timer timeout.
 *
 * @details This function will be called each time the status level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void status_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    status_level_update();
}



/**@brief Function for handling the RR interval timer timeout.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void rr_interval_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;

        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                      &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}


/**@brief Function for handling the Sensor Contact Detected timer timeout.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void sensor_contact_detected_timeout_handler(void * p_context)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(p_context);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    uint32_t err_code;

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
	  err_code = app_timer_create(&m_slope_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                slope_level_meas_timeout_handler);
		err_code = app_timer_create(&m_status_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                status_level_meas_timeout_handler);
	
    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_create(&m_heart_rate_timer_id,
//                                APP_TIMER_MODE_REPEATED,
//                                heart_rate_meas_timeout_handler);
//    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_rr_interval_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                rr_interval_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_sensor_contact_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                sensor_contact_detected_timeout_handler);
    APP_ERROR_CHECK(err_code);
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

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
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


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery, Slope and Device Information services.
 */
static void services_init(void)
{
    uint32_t       err_code;
	
		//battery service init:
		ble_bas_init_t bas_init;
		memset(&bas_init, 0, sizeof(bas_init));
		err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
	
	
    // Initialize Slope Service.
//		ble_slope_init_t slope_init;
//		memset(&slope_init, 0, sizeof(slope_init));
//    err_code = ble_slope_init(&m_slope, &slope_init);
//    APP_ERROR_CHECK(err_code);
		
		// Initialize Device Information Service.
		ble_dis_init_t dis_init;
    memset(&dis_init, 0, sizeof(dis_init));
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
		
//		// Initialize Device Status Service.
//		ble_status_init_t status_init;
//    memset(&status_init, 0, sizeof(status_init));
//    err_code = ble_status_init(&m_status, &status_init);
//    APP_ERROR_CHECK(err_code);
//	
//	//initialize probe error service
//	ble_probe_error_service_init(&m_pes);
//    
//    //initialize profile service
//   ble_profile_service_init(&m_ps);

    // Initialize Optical Cal.
    cal_optical_init_t optical_init;
    memset(&optical_init, 0, sizeof(optical_init));
    err_code = cal_optical_init(&m_optical, &optical_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Force Cal.
    cal_force_init_t force_init;
    memset(&force_init, 0, sizeof(force_init));
    err_code = cal_force_init(&m_force, &force_init);
    APP_ERROR_CHECK(err_code);

}



///**@brief Function for initializing the sensor simulators.
// */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
	
		m_slope_sim_cfg.min          = MIN_slope_LEVEL;
    m_slope_sim_cfg.max          = MAX_slope_LEVEL;
    m_slope_sim_cfg.incr         = slope_LEVEL_INCREMENT;
    m_slope_sim_cfg.start_at_max = true;

    sensorsim_init(&m_slope_sim_state, &m_slope_sim_cfg);
	
		m_status_sim_cfg.min          = MIN_status_LEVEL;
    m_status_sim_cfg.max          = MAX_status_LEVEL;
    m_status_sim_cfg.incr         = status_LEVEL_INCREMENT;
    m_status_sim_cfg.start_at_max = true;

    sensorsim_init(&m_status_sim_state, &m_status_sim_cfg);

    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}


/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_slope_timer_id, slope_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_status_timer_id, status_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

//    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_rr_interval_timer_id, RR_INTERVAL_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_sensor_contact_timer_id, SENSOR_CONTACT_DETECTED_INTERVAL, NULL);
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
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
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

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
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
SEGGER_RTT_WriteString(0, "BLE evt (no write fxn) \n");
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
	SEGGER_RTT_WriteString(0, "ble dispatch \n");
    dm_ble_evt_handler(p_ble_evt);
    //ble_hrs_on_ble_evt(&m_hrs, p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
	ble_slope_on_ble_evt(&m_slope, p_ble_evt);
	ble_status_on_ble_evt(&m_status, p_ble_evt);
	ble_probe_error_service_on_ble_evt(&m_pes, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
   // bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
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


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
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

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
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

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
//    uint8_t slope_level;
//    LSM303_DATA test_data_303;
	
    // Initialize.
    timers_init();
    shutdown_gpio_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    beacon_adv_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
	
    // Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
	SEGGER_RTT_WriteString(0, "Hello World!\n");
    SEGGER_RTT_printf(0, "HELLO WORLD!\n");

	APP_Initialize();

    while(true)
    {
        
        APP_Tasks();
        power_manage();
        
        /********   Dave's test:   ********/
//		test_data_303 = getLSM303data();
//		slope_level = 0;
//		slope_level = (uint8_t)((test_data_303.X & 0xFF00)>>8);
//	
//		SLOPE_GLOBAL = slope_level;
//		SEGGER_RTT_printf(0,"slope: %d",test_data_303.X);



    }

}

