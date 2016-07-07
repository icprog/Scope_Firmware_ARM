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
 * @defgroup ble_sdk_srv_force Device Information Service
 * @{
 * @ingroup ble_sdk_srv
 * @brief Device Information Service module.
 *
 * @details This module implements the Device Information Service.
 *          During initialization it adds the Device Information Service to the BLE stack database.
 *          It then encodes the supplied information, and adds the curresponding characteristics.
 *
 * @note Attention! 
 *  To maintain compliance with Nordic Semiconductor ASA Bluetooth profile 
 *  qualification listings, this section of source code must not be modified.
 */

#ifndef cal_force_H__
#define cal_force_H__

#include <stdint.h>
#include "ble_srv_common.h"

//Service UUID:
#define SCOPE_UUID_FORCE_CAL 0x3467
//Characteristic UUIDs:

//#define       SCOPE_CHAR_UUID_cal_result		0x2A27


#define		SCOPE_CHAR_UUID_WEIGHTS					0xae5a 
#define		SCOPE_CHAR_UUID_CAL_POINTS			0X70de
#define		SCOPE_CHAR_UUID_RESULT					0X654d
#define		SCOPE_CHAR_UUID_READY						0X1234

#define   WEIGHTS    	0x01
#define		CAL_POINTS  0x01
#define   RESULT 			0x01


/** @defgroup force_VENDOR_ID_SRC_VALUES Vendor ID Source values  
 * @{
 */
#define cal_force_VENDOR_ID_SRC_BLUETOOTH_SIG   1                 /**< Vendor ID assigned by Bluetooth SIG. */
#define cal_force_VENDOR_ID_SRC_USB_IMPL_FORUM  2                 /**< Vendor ID assigned by USB Implementer's Forum. */
/** @} */

/**@brief slope Service event type. */
typedef enum
{
    cal_force_EVT_NOTIFICATION_ENABLED,                             /**< slope value notification enabled event. */
    cal_force_EVT_NOTIFICATION_DISABLED                             /**< slope value notification disabled event. */
} cal_force_evt_type_t;

/**@brief force Service event. */
typedef struct
{
    cal_force_evt_type_t evt_type;                                  /**< Type of event. */
} cal_force_evt_t;

// Forward declaration of the cal_force_t type. 
typedef struct cal_force_s cal_force_t;

/**@brief force Service event handler type. */
typedef void (*cal_force_evt_handler_t) (cal_force_t * p_force, cal_force_evt_t * p_evt);
typedef void (*cal_force_write_handler_t) (cal_force_t * p_force, uint8_t data_in);

///**@brief PnP ID parameters */
//typedef struct
//{
//    uint8_t  vendor_id_source;                                  /**< Vendor ID Source. see @ref force_VENDOR_ID_SRC_VALUES. */
//    uint16_t vendor_id;                                         /**< Vendor ID. */
//    uint16_t product_id;                                        /**< Product ID. */
//    uint16_t product_version;                                   /**< Product Version. */
//} cal_force_pnp_id_t;

/**@brief Device Information Service init structure. This contains all possible characteristics 
 *        needed for initialization of the service.
 */
typedef struct
{
    ble_srv_utf8_str_t            test_data_str;           /**< Manufacturer Name String. */
		ble_srv_utf8_str_t            test_vars_str;           /**< Manufacturer Name String. */
    ble_srv_utf8_str_t            force_data_str;               /**< Model Number String. */
    ble_srv_utf8_str_t            force_cal_str;              /**< Serial Number String. */
    ble_srv_utf8_str_t            cal_result_str;                  /**< Hardware Revision String. */
		//ble_srv_security_mode_t     force_attr_md;                 /**< Initial Security Setting for Device Information Characteristics. */
		cal_force_evt_handler_t       evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
    bool                          support_notification;           /**< TRUE if notification of Battery Level measurement is supported. */
    ble_srv_report_ref_t *        p_report_ref;                   /**< If not NULL, a Report Reference descriptor with the specified value will be added to the Battery Level characteristic */
    uint8_t                       initial_batt_level;             /**< Initial battery level */
    ble_srv_cccd_security_mode_t  force_char_cccd_attr_md;     /**< Initial security level for battery characteristics attribute */
    ble_gap_conn_sec_mode_t       force_report_read_perm; 			/**< Initial security level for battery report read attribute */
		cal_force_write_handler_t 		force_write_handler; 
} cal_force_init_t;


/**@brief force Service structure. This contains various status information for the service. */
struct cal_force_s
{
    cal_force_evt_handler_t     	evt_handler;                    /**< Event handler to be called for handling events in the force Service. */
    uint16_t                      service_handle;                 /**< Handle of force Service (as provided by the BLE stack). */
		ble_gatts_char_handles_t      force_level_handles;          /**< Handles related to the force Level characteristic. */
    ble_gatts_char_handles_t      force_weight_handles;          /**< Handles related to the force Level characteristic. */
		ble_gatts_char_handles_t      force_cal_handles;          /**< Handles related to the force Level characteristic. */
		ble_gatts_char_handles_t      force_result_handles;          /**< Handles related to the force Level characteristic. */
		ble_gatts_char_handles_t      force_ready_handles;          /**< Handles related to the force Level characteristic. */
    uint16_t                      report_ref_handle;              /**< Handle of the Report Reference descriptor. */
    uint8_t                       force_level_last;             /**< Last force Level measurement passed to the force Service. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    bool                          is_notification_supported;      /**< TRUE if notification of force Level is supported. */
		cal_force_write_handler_t 		force_write_handler; 
};


/**@brief Function for initializing the Device Information Service.
 *
 * @details This call allows the application to initialize the device information service. 
 *          It adds the force service and force characteristics to the database, using the initial
 *          values supplied through the p_force_init parameter. Characteristics which are not to be
 *          added, shall be set to NULL in p_force_init.
 *
 * @param[in]   p_force_init   The structure containing the values of characteristics needed by the
 *                           service.
 *
 * @return      NRF_SUCCESS on successful initialization of service.
 */
uint32_t cal_force_init(cal_force_t * p_force, const cal_force_init_t * p_force_init);

void cal_force_on_ble_evt(cal_force_t * p_force, ble_evt_t * p_ble_evt);
//uint32_t cal_weights_update(cal_force_t * p_force, uint8_t weight);
uint32_t cal_points_update(cal_force_t * p_force, uint16_t weight[7]);
void cal_force_on_ble_evt(cal_force_t * p_force, ble_evt_t * p_ble_evt);
#endif // cal_force_H__

/** @} */
