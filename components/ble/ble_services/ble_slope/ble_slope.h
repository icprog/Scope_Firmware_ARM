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
 * @defgroup ble_sdk_srv_slope slope Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief slope Service module.
 *
 * @details This module implements the slope Service with the slope Level characteristic.
 *          During initialization it adds the slope Service and slope Level characteristic
 *          to the BLE stack dataslopee. Optionally it can also add a Report Reference descriptor
 *          to the slope Level characteristic (used when including the slope Service in
 *          the HID service).
 *
 *          If specified, the module will support notification of the slope Level characteristic
 *          through the ble_slope_slope_level_update() function.
 *          If an event handler is supplied by the application, the slope Service will
 *          generate slope Service events to the application.
 *
 * @note The application must propagate BLE stack events to the slope Service module by calling
 *       ble_slope_on_ble_evt() from the @ref softdevice_handler callback.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef BLE_SLOPE_H__
#define BLE_SLOPE_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define SCOPE_UUID_SLOPE				0x112C	  // UUID for slope service
#define SLOPE_CHAR_UUID         0x4322    //UUID for slope characteristic

static uint8_t             initial_slope = 0x13;  //initial slope value for testing purposes

/**@brief slope Service event type. */
typedef enum
{
    BLE_slope_EVT_NOTIFICATION_ENABLED,                             /**< slope value notification enabled event. */
    BLE_slope_EVT_NOTIFICATION_DISABLED                             /**< slope value notification disabled event. */
} ble_slope_evt_type_t;

/**@brief slope Service event. */
typedef struct
{
    ble_slope_evt_type_t evt_type;                                  /**< Type of event. */
} ble_slope_evt_t;

// Forward declaration of the ble_slope_t type. 
typedef struct ble_slope_s ble_slope_t;

/**@brief slope Service event handler type. */
typedef void (*ble_slope_evt_handler_t) (ble_slope_t * p_slope, ble_slope_evt_t * p_evt);

/**@brief slope Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_slope_evt_handler_t       evt_handler;                    /**< Event handler to be called for handling events in the slope Service. */
    bool                          support_notification;           /**< TRUE if notification of slope Level measurement is supported. */
    ble_srv_report_ref_t *        p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the slope Level characteristic */
    uint8_t                       initial_batt_level;             /**< Initial slope level */
    ble_srv_cccd_security_mode_t  slope_level_char_attr_md;     /**< Initial security level for slope characteristics attribute */
    ble_gap_conn_sec_mode_t       slope_level_report_read_perm; /**< Initial security level for slope report read attribute */
} ble_slope_init_t;

/**@brief slope Service structure. This contains various status information for the service. */
struct ble_slope_s
{
    ble_slope_evt_handler_t       evt_handler;                    /**< Event handler to be called for handling events in the slope Service. */
    uint16_t                      service_handle;                 /**< Handle of slope Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      slope_level_handles;          /**< Handles related to the slope Level characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint8_t                       slope_level_last;             /**< Last slope Level measurement passed to the slope Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of slope Level is supported. */
};

/**@brief Function for initializing the slope Service.
 *
 * @param[out]  p_slope       slope Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_slope_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_slope_init(ble_slope_t * p_slope, const ble_slope_init_t * p_slope_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the slope Service.
 *
 * @note For the requirements in the slope specification to be fulfilled,
 *       ble_slope_slope_level_update() must be called upon reconnection if the
 *       slope level has changed while the service has been disconnected from a bonded
 *       client.
 *
 * @param[in]   p_slope      slope Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_slope_on_ble_evt(ble_slope_t * p_slope, ble_evt_t * p_ble_evt);

/**@brief Function for updating the slope level.
 *
 * @details The application calls this function after having performed a slope measurement. If
 *          notification has been enabled, the slope level characteristic is sent to the client.
 *
 * @note For the requirements in the slope specification to be fulfilled,
 *       this function must be called upon reconnection if the slope level has changed
 *       while the service has been disconnected from a bonded client.
 *
 * @param[in]   p_slope          slope Service structure.
 * @param[in]   slope_level  New slope measurement value (in percent of full capacity).
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_slope_level_update(ble_slope_t * p_slope, uint8_t slope_level);

#endif // BLE_slope_H__

/** @} */
