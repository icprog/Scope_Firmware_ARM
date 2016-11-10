#ifndef FWU_SERVICE_H__
#define FWU_SERVICE_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

/********* UUIDS ************/
#define FWU_SERVICE_BASE_UUID  {{0x23, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define FWU_SERVICE_UUID 0x1123
#define FWU_CMD_CHAR_UUID 0x5812
#define FWU_DATA_CHAR_UUID 0x5813

typedef enum
{
    PLACE_HOLDER,
    START_ARM_FWU,
    START_PIC_FWU,
    FWU_ACK,
    FWU_NACK,
    DONE_PIC_FWU,
    RESTART,
} fwu_code_t;

/**@brief Firmware Update Service event type. */
typedef enum
{
    BLE_FWU_EVT_NOTIFICATION_ENABLED,
    BLE_FWU_EVT_NOTIFICATION_DISABLED 
} ble_fwu_evt_type_t;

/**@brief Firmware Update Service event. */
typedef struct
{
    ble_fwu_evt_type_t evt_type;                                  /**< Type of event. */
} ble_fwu_evt_t;

//Forward declartion
typedef struct ble_fwu_s ble_fwu_t;

//function pointer type for event handler
//typedef void (*ble_pes_evt_handler_t) (ble_pes_t * p_pes, ble_pes_evt_t * p_evt);

struct ble_fwu_s{
	uint16_t                    conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    cmd_char_handles;   //handles for the characteristic attributes to our struct
    ble_gatts_char_handles_t    data_char_handles;   //handles for the characteristic attributes to our struct
};

void ble_fwu_service_init(ble_fwu_t * p_fwu_service);
void ble_fwu_service_on_ble_evt(ble_fwu_t * p_fwu_service, ble_evt_t * p_ble_evt);
uint32_t ble_fwu_update(ble_fwu_t * p_fwu, uint8_t fwu_code);


#endif // FWU_SERVICE_H__
