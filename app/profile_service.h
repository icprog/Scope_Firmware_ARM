#ifndef PROFILE_SERVICE_H__
#define PROFILE_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "app.h" //only for the raw data type. //TODO move its definition

/********* UUIDS ************/
#define PROFILE_BASE_UUID  {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x88, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define PROFILE_SERVICE_UUID 0x145C
#define PROFILE_CHAR_UUID 0x321A
#define PROFILE_IDS_CHAR_UUID 0x321B
#define TRANSFER_IDS_CHAR_UUID 0x321C
#define DELETE_IDS_CHAR_UUID 0x321D
#define PROFILE_ERROR_CHAR_UUID 0x321E
#define PROFILE_LENGTH_CHAR_UUID 0x321F
#define RAW_DATA_CHAR_UUID 0x3219




/**@brief Battery Service event type. */
typedef enum
{
    PROFILE_EVT_NOTIFICATION_ENABLED,                             /**< Battery value notification enabled event. */
    PROFILE_EVT_NOTIFICATION_DISABLED                             /**< Battery value notification disabled event. */
} profile_evt_type_t;

/**@brief Battery Service event. */
typedef struct
{
    profile_evt_type_t evt_type;                                  /**< Type of event. */
} profile_evt_t;

// Forward declaration of the cal_optical_t type. 
typedef struct profile_s profile_t;

/**@brief Battery Service event handler type. */
typedef void (*profile_evt_handler_t) (profile_t * p_profile, profile_evt_t * p_evt);

//service structure
//TODO typedef might mess things up
typedef struct {
	profile_evt_handler_t       evt_handler;    /**< Event handler to be called for handling events in the Battery Service. */
	uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    profile_char_handles;   //handles for the profile characteristic attributes to our struct
    ble_gatts_char_handles_t    profile_ids_char_handles;   //handles for the profile ids characteristic attributes to our struct
    ble_gatts_char_handles_t    transfer_ids_char_handles;   //handles for the transfer ids characteristic attributes to our struct
    ble_gatts_char_handles_t    delete_ids_char_handles;   //handles for the delete ids characteristic attributes to our struct
    ble_gatts_char_handles_t    profile_error_char_handles;   //handles for the profile error characteristic attributes to our struct
    ble_gatts_char_handles_t    raw_data_char_handles;   //handles for the profile error characteristic attributes to our struct
    ble_gatts_char_handles_t    profile_length_char_handles;   //handles for the profile error characteristic attributes to our struct

}ble_ps_t;

/********* FUNCTIONS  ***********/
void ble_profile_service_init(ble_ps_t * p_profile_service);
void ble_profile_service_on_ble_evt(ble_ps_t * p_ps_service, ble_evt_t * p_ble_evt);
uint32_t ble_profile_update(ble_ps_t * p_ps, uint8_t probe_error_code);
void profile_data_update(ble_ps_t * p_ps, uint8_t * send_data, uint8_t size); // send data to phone
uint32_t update_profile_length(ble_ps_t * p_ps, uint16_t length);
uint32_t raw_data_update(ble_ps_t * p_ps, uint8_t * raw_data, uint8_t size, uint8_t * bytes_sent);
void profile_ids_update(ble_ps_t * p_ps, uint16_t max_profile_num);

#endif //PROFILE_SERVICE_H__
