#ifndef PROFILE_SERVICE_H__
#define PROFILE_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

/********* UUIDS ************/
#define PROFILE_BASE_UUID  {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x88, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define PROFILE_SERVICE_UUID 0x145C
#define PROFILE_CHAR_UUID 0x321A
#define PROFILE_IDS_CHAR_UUID 0x321B
#define TRANSFER_IDS_CHAR_UUID 0x321C
#define DELETE_IDS_CHAR_UUID 0x321D
#define PROFILE_ERROR_CHAR_UUID 0x321E



//service structure
//TODO typedef might mess things up
typedef struct {
	uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    char_handles;   //handles for the characteristic attributes to our struct
}ble_ps_t;

/********* FUNCTIONS  ***********/
void ble_profile_service_init(ble_ps_t * p_profile_service);
void ble_profile_service_on_ble_evt(ble_ps_t * p_ps_service, ble_evt_t * p_ble_evt);
uint32_t ble_profile_update(ble_ps_t * p_ps, uint8_t probe_error_code);

#endif //PROFILE_SERVICE_H__
