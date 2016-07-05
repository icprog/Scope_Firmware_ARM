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

/** @file
 *
 * @defgroup ble_sdk_srv_optical Battery Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Battery Service module.
 *
 * @details This module implements the Battery Service with the Battery Level characteristic.
 *          During initialization it adds the Battery Service and Battery Level characteristic
 *          to the BLE stack dataopticale. Optionally it can also add a Report Reference descriptor
 *          to the Battery Level characteristic (used when including the Battery Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the Battery Level characteristic
 *          through the cal_optical_cal_result_update() function.
 *          If an event handler is supplied by the application, the Battery Service will
 *          generate Battery Service events to the application.
 *
 * @note The application must propagate BLE stack events to the Battery Service module by calling
 *       cal_optical_on_ble_evt() from the @ref softdevice_handler callback.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef CAL_OPTICAL_H__
#define CAL_OPTICAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UUID_CAL_OPTICAL_SERVICE  0xa67e  // optical cal service UUID


#define  BLE_UUID_OPTICAL_CAL_OPTICAL_CAL_CHAR 0x02a7
#define BLE_UUID_OPTICAL_CAL_TEST_VARS_CHAR 0x26f3
#define BLE_UUID_OPTICAL_CAL_OPTICAL_DATA_CHAR 0x8d21
#define BLE_UUID_OPTICAL_CAL_RESULT_CHAR 0xb5b8   //optical cal result characteristic UUID

/**@brief Battery Service event type. */
typedef enum
{
    CAL_OPTICAL_EVT_NOTIFICATION_ENABLED,                             /**< Battery value notification enabled event. */
    CAL_OPTICAL_EVT_NOTIFICATION_DISABLED                             /**< Battery value notification disabled event. */
} cal_optical_evt_type_t;

/**@brief Battery Service event. */
typedef struct
{
    cal_optical_evt_type_t evt_type;                                  /**< Type of event. */
} cal_optical_evt_t;

// Forward declaration of the cal_optical_t type. 
typedef struct cal_optical_s cal_optical_t;

/**@brief Battery Service event handler type. */
typedef void (*cal_optical_evt_handler_t) (cal_optical_t * p_optical, cal_optical_evt_t * p_evt);

/**@brief Battery Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    cal_optical_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
		cal_optical_evt_handler_t			result_handler;
		cal_optical_evt_handler_t			data_handler;
		cal_optical_evt_handler_t			cal_handler;
		cal_optical_evt_handler_t			test_vars_handler;
    bool                          support_notification;           /**< TRUE if notification of Battery Level measurement is supported. */
    ble_srv_report_ref_t *        p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic */
    uint8_t                       initial_batt_level;             /**< Initial battery level */
    ble_srv_cccd_security_mode_t  cal_result_char_attr_md;     /**< Initial security level for battery characteristics attribute */
    ble_gap_conn_sec_mode_t       cal_result_report_read_perm; /**< Initial security level for battery report read attribute */
} cal_optical_init_t;

/**@brief Battery Service structure. This contains various status information for the service. */
struct cal_optical_s
{
    cal_optical_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
		cal_optical_evt_handler_t			result_handler;
		cal_optical_evt_handler_t			data_handler;
		cal_optical_evt_handler_t			cal_handler;
		cal_optical_evt_handler_t			test_vars_handler;
    uint16_t                      service_handle;                 /**< Handle of Battery Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      cal_result_handles;          /**< Handles related to the Battery Level characteristic. */
		ble_gatts_char_handles_t      cal_optical_cal_handles;          /**< Handles related to the Battery Level characteristic. */
		ble_gatts_char_handles_t      cal_optical_data_handles;          /**< Handles related to the Battery Level characteristic. */
		ble_gatts_char_handles_t      cal_test_vars_handles;          /**< Handles related to the Battery Level characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint8_t                       cal_result_last;             /**< Last Battery Level measurement passed to the Battery Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of Battery Level is supported. */
};

/**@brief Function for initializing the Battery Service.
 *
 * @param[out]  p_optical       Battery Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_optical_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t cal_optical_init(cal_optical_t * p_optical, const cal_optical_init_t * p_optical_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note For the requirements in the OPTICAL specification to be fulfilled,
 *       cal_optical_cal_result_update() must be called upon reconnection if the
 *       battery level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_optical      Battery Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void cal_optical_on_ble_evt(cal_optical_t * p_optical, ble_evt_t * p_ble_evt);

/**@brief Function for updating the battery level.
 *
 * @details The application calls this function after having performed a battery measurement. If
 *          notification has been enabled, the battery level characteristic is sent to the client.
 *
 * @note For the requirements in the OPTICAL specification to be fulfilled,
 *       this function must be called upon reconnection if the battery level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_optical          Battery Service structure.
 * @param[in]   cal_result  New battery measurement value (in percent of full capacity).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t optical_cal_result_update(cal_optical_t * p_optical, uint8_t cal_result);
uint32_t optical_cal_update(cal_optical_t * p_optical, uint8_t cal_result);
uint32_t optical_cal_data_update(cal_optical_t * p_optical, uint8_t cal_result);
uint32_t optical_cal_test_vars_update(cal_optical_t * p_optical, uint8_t cal_result);
#endif // CAL_OPTICAL_H__

/** @} */
