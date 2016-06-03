/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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
 * @defgroup ble_sdk_srv_dev_status LED Button Service Server
 * @{
 * @ingroup ble_sdk_srv
 *
 * @brief LED Button Service Server module.
 *
 * @details This module implements a custom LED Button Service with an LED and Button Characteristics.
 *          During initialization, the module adds the LED Button Service and Characteristics
 *          to the BLE stack database.
 *
 *          The application must supply an event handler for receiving LED Button Service
 *          events. Using this handler, the service notifies the application when the
 *          LED value changes.
 *
 *          The service also provides a function for letting the application notify
 *          the state of the Button Characteristic to connected peers.
 *
 * @note The application must propagate BLE stack events to the LED Button Service
 *       module by calling ble_dev_status_on_ble_evt() from the @ref softdevice_handler callback.
*/

#ifndef ble_dev_status_H__
#define ble_dev_status_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define DEV_STATUS_UUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}


#define SCOPE_UUID_DEVICE_STATUS    0x119B

#define CHAR_UUID_HARDWARE_REV      0x2A27															
															
															
//#define SCOPE_UUID_DEVICE_STATUS    0x119B
#define SCOPE_UUID_BATTERY					0x120B //0x180F  DH TODO: temp value to avoiod conflict with phone battery service
#define SCOPE_UUID_DEVICE_INFO			0x180A												
#define SCOPE_UUID_PROBING_ERRORS		0x123A
#define SCOPE_UUID_SNOW_PROFILE			0x145C
#define SCOPE_UUID_SLOPE						0x112C									 
															
static uint8_t             initial_slope = 0x13;
															
// Forward declaration of the ble_dev_status_t type. 
typedef struct ble_dev_status_s ble_dev_status_t;

typedef void (*ble_dev_status_led_write_handler_t) (ble_dev_status_t * p_dev_status, uint8_t new_state);

/** @brief LED Button Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_dev_status_led_write_handler_t led_write_handler; /**< Event handler to be called when the LED Characteristic is written. */
} ble_dev_status_init_t;

/**@brief LED Button Service structure. This structure contains various status information for the service. */
struct ble_dev_status_s
{
    uint16_t                    service_handle;      /**< Handle of LED Button Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    led_char_handles;    /**< Handles related to the LED Characteristic. */
    ble_gatts_char_handles_t    button_char_handles; /**< Handles related to the Button Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the LED Button Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_dev_status_led_write_handler_t led_write_handler;   /**< Event handler to be called when the LED Characteristic is written. */
};

void ble_slope_on_ble_evt(ble_dev_status_t * p_bas, ble_evt_t * p_ble_evt);
/**@brief Function for initializing the LED Button Service.
 *
 * @param[out] p_dev_status      LED Button Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_dev_status_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_device_status_init(ble_dev_status_t * p_dev_status, const ble_dev_status_init_t * p_dev_status_init);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the LED Button Service.
 *
 * @param[in] p_dev_status      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
void ble_device_status_on_ble_evt(ble_dev_status_t * p_dev_status, ble_evt_t * p_ble_evt);

/**@brief Function for sending a button state notification.
 *
 * @param[in] p_dev_status      LED Button Service structure.
 * @param[in] button_state  New button state.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_dev_status_on_button_change(ble_dev_status_t * p_dev_status, uint8_t button_state);

uint32_t ble_dev_status_on_other_button_change(ble_dev_status_t * p_dev_status, uint8_t button_state); //added to test data_send on button press DH

#endif // ble_dev_status_H__

/** @} */
