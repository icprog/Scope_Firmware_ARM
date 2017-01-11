

#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "debug.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"
#include "app.h"
#include "spi_utils.h"

void ble_debug_service_on_ble_evt(ble_dbs_t * p_dbs, ble_evt_t * p_ble_evt)
{
	//SEGGER_RTT_printf(0, "debug evt");
	if (p_dbs == NULL || p_ble_evt == NULL)
    {
        return;
    }
    // OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
    switch (p_ble_evt->header.evt_id)
    {        
        case BLE_GAP_EVT_CONNECTED:
        {
            p_dbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        }
        case BLE_GAP_EVT_DISCONNECTED:
        {
            p_dbs->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        }
		case BLE_GATTS_EVT_WRITE:
        {
            on_write(p_dbs, p_ble_evt);
            break;
        }
        default:
            // No implementation needed.
            break;
    }
}

void debug_char_add(ble_dbs_t * p_dbs)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUID and add it to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = DEBUG_CHAR_UUID;
    char_uuid.type = p_dbs->uuid_type;
   // BLE_UUID_BLE_ASSIGN(char_uuid, DEBUG_CHAR_UUID);
    
    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 1;
    char_md.char_props.read = 1;
    
    /******   Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure   ****/
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
		char_md.char_props.read   = 1;
    /*** Configure the attribute metadata ***/
    ble_gatts_attr_md_t attr_md;
    attr_md.vlen = 1;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    /****Set read/write security levels to our characteristic ***/
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    /**** Configure the characteristic value attribute  ****/
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));     
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    /***  Set characteristic length in number of bytes  ****/
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    char value            = 'w';
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_dbs->service_handle, &char_md, &attr_char_value, &p_dbs->char_handles);
    APP_ERROR_CHECK(err_code);
}


void ble_debug_service_init(ble_dbs_t * p_debug_service)
{
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare service UUID and add it to the BLE stack  *****/
    ble_uuid128_t base_uuid = DEBUG_BASE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_debug_service->uuid_type);
    APP_ERROR_CHECK(err_code);
    
    
    ble_uuid_t service_uuid;
    service_uuid.uuid = DEBUG_SERVICE_UUID;
    service_uuid.type = p_debug_service->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, DEBUG_SERVICE_UUID);
    
    p_debug_service->conn_handle = BLE_CONN_HANDLE_INVALID; //Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_debug_service->service_handle);
    APP_ERROR_CHECK(err_code);
    
    /******* add charateristic  *******/
    debug_char_add(p_debug_service);
}


static void on_write(ble_dbs_t * p_ds, ble_evt_t * p_ble_evt)
{
		//SEGGER_RTT_printf(0, "debug write");
        ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
        

        if(p_evt_write->handle == p_ds->char_handles.value_handle && (p_evt_write->len == 5))
        {
            memcpy(&(device_info.serial_number), p_evt_write->data, 5);
            appData.state = APP_STATE_NEW_SN;
        }
        else if(p_evt_write->handle == p_ds->char_handles.value_handle && (p_evt_write->len == 1))
        {
            if(*(p_evt_write->data) == 'x')
            {
                SEGGER_RTT_printf(0, "start x modem\n");
                appData.state = APP_STATE_X_MODEM;
            }
            else if(*(p_evt_write->data) == 't')
            {
                SEGGER_RTT_printf(0, "start test\n");
                appData.state = APP_STATE_START_TEST;
            }
            else if(*(p_evt_write->data) == 's')
            {
                SEGGER_RTT_printf(0, "s\n");
                appData.state = APP_STATE_PIC_FWU_START;
                SEGGER_RTT_printf(0, "state = %d", appData.state);
                
            }
        }
        if(p_evt_write->handle == p_ds->char_handles.cccd_handle && (p_evt_write->len == 2))
        {

            // CCCD written, call application event handler
            if (p_ds->evt_handler != NULL)
            {
                ble_dbs_evt_t evt;
				//evt.evt_type = BLE_dbs_EVT_NOTIFICATION_ENABLED;
                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
					SEGGER_RTT_printf(0, "debug notification");
                    evt.evt_type = BLE_dbs_EVT_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = BLE_dbs_EVT_NOTIFICATION_DISABLED  ;
                }

                p_ds->evt_handler(p_ds, &evt);
            }
        }
    
}

uint32_t ble_debug_update(ble_dbs_t * p_dbs, char * debug_message,uint8_t length)
{
		//SEGGER_RTT_printf(0, "debug update");
    if (p_dbs == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;


	// Initialize value struct.
	memset(&gatts_value, 0, sizeof(gatts_value));
	gatts_value.len     = length;
	gatts_value.offset  = 0;
	gatts_value.p_value = debug_message;

	// Update database.
	err_code = sd_ble_gatts_value_set(p_dbs->conn_handle, p_dbs->char_handles.value_handle, &gatts_value);
	if (err_code == NRF_SUCCESS)
	{
		//save probe error code
		p_dbs->debug_message = debug_message;
	}
	else
	{
		return err_code;
	}

	// Send value if connected and notifying.
	if ((p_dbs->conn_handle != BLE_CONN_HANDLE_INVALID))
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_dbs->char_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		
		err_code = sd_ble_gatts_hvx(p_dbs->conn_handle, &hvx_params);
		//SEGGER_RTT_printf(0, "notifying %d", debug_code);
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}


    return err_code;
}
