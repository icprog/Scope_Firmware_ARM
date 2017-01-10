#ifndef PROBE_ERROR_H__
#define PROBE_ERROR_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

/********* UUIDS ************/
#define PROBE_ERROR_BASE_UUID  {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define PROBE_ERROR_SERVICE_UUID 0x123A
#define PROBE_ERROR_CHAR_UUID 0x1234

/**@brief Probe Error Service event type. */
typedef enum
{
    BLE_PES_EVT_NOTIFICATION_ENABLED,                             /**< Battery value notification enabled event. */
    BLE_PES_EVT_NOTIFICATION_DISABLED                             /**< Battery value notification disabled event. */
} ble_pes_evt_type_t;

/**@brief Probe Error Service event. */
typedef struct
{
    ble_pes_evt_type_t evt_type;                                  /**< Type of event. */
} ble_pes_evt_t;

//Forward declartion
typedef struct ble_pes_s ble_pes_t;

//function pointer type for event handler
//typedef void (*ble_pes_evt_handler_t) (ble_pes_t * p_pes, ble_pes_evt_t * p_evt);

struct ble_pes_s{
	uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    char_handles;   //handles for the characteristic attributes to our struct
    uint8_t                     probe_error_code;
    
    //ble_pes_evt_handler_t       evt_handler;
};

void ble_probe_error_service_init(ble_pes_t * p_probe_error_service);
void ble_probe_error_service_on_ble_evt(ble_pes_t * p_pes_service, ble_evt_t * p_ble_evt);
uint32_t ble_probe_error_update(ble_pes_t * p_pes, uint8_t probe_error_code);


#endif // PROBE_ERROR_H__
