#ifndef DEBUG_H__
#define DEBUG_H__

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

/********* UUIDS ************/
#define DEBUG_BASE_UUID  {{0x33, 0xD1, 0x13, 0xEF, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}} // 128-bit base UUID
#define DEBUG_SERVICE_UUID 0x0011
#define DEBUG_CHAR_UUID 0x0012

/**@brief Probe Error Service event type. */
typedef enum
{
    BLE_dbs_EVT_NOTIFICATION_ENABLED,                             /**< Battery value notification enabled event. */
    BLE_dbs_EVT_NOTIFICATION_DISABLED                             /**< Battery value notification disabled event. */
} ble_dbs_evt_type_t;

/**@brief Probe Error Service event. */
typedef struct
{
    ble_dbs_evt_type_t evt_type;                                  /**< Type of event. */
} ble_dbs_evt_t;

//Forward declartion
typedef struct ble_dbs_s ble_dbs_t;

//function pointer type for event handler
typedef void (*ble_dbs_evt_handler_t) (ble_dbs_t * p_dbs, ble_dbs_evt_t * p_evt);

struct ble_dbs_s{
    uint8_t                     uuid_type; /**< vendor specific UUID type returned by sd_ble_uuid_vs_add() */
	uint16_t                    	conn_handle;    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection).*/
    uint16_t                    service_handle; /**< Handle of Our Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    char_handles;   //handles for the characteristic attributes to our struct
    char                     		*debug_message;
    ble_dbs_evt_handler_t       evt_handler;
	  bool                        is_notification_supported;      /**< TRUE if notification of slope Level is supported. */
};

static void on_write(ble_dbs_t * p_ds, ble_evt_t * p_ble_evt);
void ble_debug_service_init(ble_dbs_t * p_debug_service);
void ble_debug_service_on_ble_evt(ble_dbs_t * p_dbs_service, ble_evt_t * p_ble_evt);
uint32_t ble_debug_update(ble_dbs_t * p_dbs, char * debug_message, uint8_t length);


#endif // DEBUG_H__
