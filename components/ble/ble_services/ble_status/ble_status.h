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
 * @defgroup ble_sdk_srv_status status Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief status Service module.
 *
 * @details This module implements the status Service with the status Level characteristic.
 *          During initialization it adds the status Service and status Level characteristic
 *          to the BLE stack datastatuse. Optionally it can also add a Report Reference descriptor
 *          to the status Level characteristic (used when including the status Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the status Level characteristic
 *          through the ble_status_status_level_update() function.
 *          If an event handler is supplied by the application, the status Service will
 *          generate status Service events to the application.
 *
 * @note The application must propagate BLE stack events to the status Service module by calling
 *       ble_status_on_ble_evt() from the @ref softdevice_handler callback.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_STATUS_H__
#define BLE_STATUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define SCOPE_UUID_STATUS				0x119B	  // UUID for status service
#define status_CHAR_UUID         0x2222    //UUID for status characteristic

static uint8_t             initial_status = 0x13;  //initial status value for testing purposes

/**@brief status Service event type. */
typedef enum
{
    BLE_status_EVT_NOTIFICATION_ENABLED,                             /**< status value notification enabled event. */
    BLE_status_EVT_NOTIFICATION_DISABLED                             /**< status value notification disabled event. */
} ble_status_evt_type_t;

/**@brief status Service event. */
typedef struct
{
    ble_status_evt_type_t evt_type;                                  /**< Type of event. */
} ble_status_evt_t;

// Forward declaration of the ble_status_t type. 
typedef struct ble_status_s ble_status_t;

/**@brief status Service event handler type. */
typedef void (*ble_status_evt_handler_t) (ble_status_t * p_status, ble_status_evt_t * p_evt);

/**@brief status Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_status_evt_handler_t       evt_handler;                    /**< Event handler to be called for handling events in the status Service. */
    bool                          support_notification;           /**< TRUE if notification of status Level measurement is supported. */
    ble_srv_report_ref_t *        p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the status Level characteristic */
    uint8_t                       initial_batt_level;             /**< Initial status level */
    ble_srv_cccd_security_mode_t  status_level_char_attr_md;     /**< Initial security level for status characteristics attribute */
    ble_gap_conn_sec_mode_t       status_level_report_read_perm; /**< Initial security level for status report read attribute */
} ble_status_init_t;

/**@brief status Service structure. This contains various status information for the service. */
struct ble_status_s
{
    ble_status_evt_handler_t       evt_handler;                    /**< Event handler to be called for handling events in the status Service. */
    uint16_t                      service_handle;                 /**< Handle of status Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      status_level_handles;          /**< Handles related to the status Level characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint8_t                       status_level_last;             /**< Last status Level measurement passed to the status Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of status Level is supported. */
};

/**@brief Function for initializing the status Service.
 *
 * @param[out]  p_status       status Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_status_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_status_init(ble_status_t * p_status, const ble_status_init_t * p_status_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the status Service.
 *
 * @note For the requirements in the status specification to be fulfilled,
 *       ble_status_status_level_update() must be called upon reconnection if the
 *       status level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_status      status Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_status_on_ble_evt(ble_status_t * p_status, ble_evt_t * p_ble_evt);

/**@brief Function for updating the status level.
 *
 * @details The application calls this function after having performed a status measurement. If
 *          notification has been enabled, the status level characteristic is sent to the client.
 *
 * @note For the requirements in the status specification to be fulfilled,
 *       this function must be called upon reconnection if the status level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_status          status Service structure.
 * @param[in]   status_level  New status measurement value (in percent of full capacity).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_status_status_level_update(ble_status_t * p_status, uint8_t status_level);

#endif // BLE_status_H__

/** @} */
