#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "profile_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"



void profile_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = PROFILE_CHAR_UUID;
    BLE_UUID_BLE_ASSIGN(char_uuid, PROFILE_CHAR_UUID); //TODO might be redundant witht he previous line

    
    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    //char_md.char_props.write = 1;
    char_md.char_props.read = 1;
	char_md.char_props.notify = 1;
    
    /******   Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure   ****/
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
    
    /*** Configure the attribute metadata ***/
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    /****Set read/write security levels to our characteristic ***/
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
    /**** Configure the characteristic value attribute  ****/
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));     
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    /***  Set characteristic length in number of bytes  ****/
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    uint8_t value               = 0x00;
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_ps->service_handle, &char_md, &attr_char_value, &p_ps->profile_char_handles);
    APP_ERROR_CHECK(err_code);
}

void profile_ids_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = PROFILE_IDS_CHAR_UUID;
    BLE_UUID_BLE_ASSIGN(char_uuid, PROFILE_IDS_CHAR_UUID); //TODO might be redundant witht he previous line

    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    //char_md.char_props.write = 1;
    char_md.char_props.read = 1;
    
    /******   Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure   ****/
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
    
    /*** Configure the attribute metadata ***/
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    /****Set read/write security levels to our characteristic ***/
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); //needed for write or notify
    
    /**** Configure the characteristic value attribute  ****/
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));     
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    /***  Set characteristic length in number of bytes  ****/
    attr_char_value.max_len     = 1;
    attr_char_value.init_len    = 1;
    uint8_t value               = 0x00;
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_ps->service_handle, &char_md, &attr_char_value, &p_ps->profile_ids_char_handles);
    APP_ERROR_CHECK(err_code);
}

void transfer_ids_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = TRANSFER_IDS_CHAR_UUID;
    BLE_UUID_BLE_ASSIGN(char_uuid, TRANSFER_IDS_CHAR_UUID); //TODO might be redundant witht he previous line

    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 1;
    //char_md.char_props.read = 1;
    
    /******   Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure   ****/
//    ble_gatts_attr_md_t cccd_md;
//    memset(&cccd_md, 0, sizeof(cccd_md));
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
//    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
//    char_md.p_cccd_md           = &cccd_md;
//    char_md.char_props.notify   = 1;
    
    /*** Configure the attribute metadata ***/
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    /****Set read/write security levels to our characteristic ***/
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); //needed for write or notify
    
    /**** Configure the characteristic value attribute  ****/
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));     
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    /***  Set characteristic length in number of bytes  ****/
    attr_char_value.max_len     = 1;
    attr_char_value.init_len    = 1;
    uint8_t value               = 0x00;
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_ps->service_handle, &char_md, &attr_char_value, &p_ps->transfer_ids_char_handles);
    APP_ERROR_CHECK(err_code);
}

void delete_ids_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = DELETE_IDS_CHAR_UUID;
    BLE_UUID_BLE_ASSIGN(char_uuid, DELETE_IDS_CHAR_UUID); //TODO might be redundant witht he previous line

    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 1;
    //char_md.char_props.read = 1;
    
    /******   Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure   ****/
//    ble_gatts_attr_md_t cccd_md;
//    memset(&cccd_md, 0, sizeof(cccd_md));
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
//    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
//    char_md.p_cccd_md           = &cccd_md;
//    char_md.char_props.notify   = 1;
    
    /*** Configure the attribute metadata ***/
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    /****Set read/write security levels to our characteristic ***/
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); //needed for write or notify
    
    /**** Configure the characteristic value attribute  ****/
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));     
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    /***  Set characteristic length in number of bytes  ****/
    attr_char_value.max_len     = 1;
    attr_char_value.init_len    = 1;
    uint8_t value               = 0x00;
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_ps->service_handle, &char_md, &attr_char_value, &p_ps->delete_ids_char_handles);
    APP_ERROR_CHECK(err_code);
}

void profile_error_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = PROFILE_ERROR_CHAR_UUID;
    BLE_UUID_BLE_ASSIGN(char_uuid, PROFILE_ERROR_CHAR_UUID); //TODO might be redundant witht he previous line

    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    //char_md.char_props.write = 1;
    //char_md.char_props.read = 1;
    
    /******   Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure   ****/
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
    char_md.p_cccd_md           = &cccd_md;
    char_md.char_props.notify   = 1;
    
    /*** Configure the attribute metadata ***/
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md)); 
    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
    
    /****Set read/write security levels to our characteristic ***/
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); //needed for write or notify
    
    /**** Configure the characteristic value attribute  ****/
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));     
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    /***  Set characteristic length in number of bytes  ****/
    attr_char_value.max_len     = 1;
    attr_char_value.init_len    = 1;
    uint8_t value               = 0x00;
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_ps->service_handle, &char_md, &attr_char_value, &p_ps->profile_error_char_handles);
    APP_ERROR_CHECK(err_code);
}

void ble_profile_service_init(ble_ps_t * p_profile_service)
{
        uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Decalre service UUIDs and add them to the BLE stack  *****/
    ble_uuid_t service_uuid;
    ble_uuid128_t base_uuid = PROFILE_BASE_UUID;
    service_uuid.uuid = PROFILE_SERVICE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    p_profile_service->conn_handle = BLE_CONN_HANDLE_INVALID; //Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_profile_service->service_handle);
    APP_ERROR_CHECK(err_code);
    
    /******* add charateristics  *******/
    profile_char_add(p_profile_service);
    profile_ids_char_add(p_profile_service);
    transfer_ids_char_add(p_profile_service);
    delete_ids_char_add(p_profile_service);
    profile_error_char_add(p_profile_service);
    
}

void profile_data_update(ble_ps_t * p_ps, uint8_t * send_data, uint8_t size)
{
	if (p_ps == NULL)
    {
        //return NRF_ERROR_NULL;
		SEGGER_RTT_printf(0, "error: null profile input \n");
		return;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = size*sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = send_data;

    // Update data.
    err_code = sd_ble_gatts_value_set(p_ps->conn_handle,
                                      p_ps->profile_char_handles.value_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
        SEGGER_RTT_printf(0, "data update success \n");
    }
    else
    {
        SEGGER_RTT_printf(0, "error in profile data update fxn\n");
    }

    // Send value if connected and notifying.
    if ((p_ps->conn_handle != BLE_CONN_HANDLE_INVALID) )//&& p_ps->is_notification_supported)
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ps->profile_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_ps->conn_handle, &hvx_params);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
}

void on_write_profile_service(ble_ps_t * p_ps, ble_evt_t * p_ble_evt)
{
	//need to get data from PIC

    SEGGER_RTT_printf(0, "profile data write fxn\n");
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    profile_t * p_profile;
    if ((p_evt_write->handle == p_ps->profile_char_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written, call application event handler
        if (p_ps->evt_handler != NULL)
        {
            profile_evt_t evt;
//                if (ble_srv_is_notification_enabled(p_evt_write->data))
//                {
                evt.evt_type = PROFILE_EVT_NOTIFICATION_ENABLED;
                SEGGER_RTT_printf(0, "profile data notification enabled *** *** *** \n");
//                }
//                else
//                {
                //evt.evt_type = PROFILE_EVT_NOTIFICATION_DISABLED;
//                }
<<<<<<< HEAD

            

=======
//								p_ps->evt_handler(p_ps, &evt); // need to fix type conflict
                //p_optical->result_handler(p_optical, 13);
            //}
             //p_ps->evt_handler(p_ps, &evt); // need to fix type conflict
            //p_optical->result_handler(p_optical, 13);
>>>>>>> 5c5d6bd39fffe5cee090d4252fc17eab4d3af6f9
        }
    }

	
}
void ble_profile_service_on_ble_evt(ble_ps_t * p_ps, ble_evt_t * p_ble_evt)
{
	if (p_ps == NULL || p_ble_evt == NULL)
    {
		SEGGER_RTT_WriteString(0, "profile null \n");
        return;
    }
    SEGGER_RTT_WriteString(0, "profile evt handler \n");
    switch (p_ble_evt->header.evt_id)
    {        
        case BLE_GATTS_EVT_WRITE:
			SEGGER_RTT_WriteString(0, "profile evt handler --write evt \n");
			on_write_profile_service(p_ps, p_ble_evt);
            break;
        case BLE_GAP_EVT_CONNECTED:
            p_ps->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_ps->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            // No implementation needed.
            break;
    }
}
