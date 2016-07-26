#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "probe_error.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"

//static void on_ble_write(ble_pes_t * p_pes, ble_evt_t * p_ble_evt)
//{
//    
//    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

//    if (
//        (p_evt_write->handle == p_pes->char_handles.cccd_handle)
//        &&
//        (p_evt_write->len == 2)
//       )
//    {
//        // CCCD written, call application event handler
//        if (p_pes->evt_handler != NULL)
//        {
//            ble_pes_evt_t evt;

//            if (ble_srv_is_notification_enabled(p_evt_write->data))
//            {
//                evt.evt_type = BLE_PES_EVT_NOTIFICATION_ENABLED;
//            }
//            else
//            {
//                evt.evt_type = BLE_PES_EVT_NOTIFICATION_DISABLED;
//            }

//            p_pes->evt_handler(p_pes, &evt);
//        }
//    }


//    // Decclare buffer variable to hold received data. The data can only be 32 bit long.
//    uint32_t data_buffer;
//    // Pupulate ble_gatts_value_t structure to hold received data and metadata.
//    ble_gatts_value_t rx_data;
//    rx_data.len = sizeof(uint32_t);
//    rx_data.offset = 0;
//    rx_data.p_value = (uint8_t*)&data_buffer;
//    
//    // Check if write event is performed on our characteristic or the CCCD
//    if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->char_handles.value_handle)
//    {
//        // Get data
//        sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->char_handles.value_handle, &rx_data);
//        // Print handle and value 
//        //printf("Value received on handle %#06x: %#010x\r\n", p_ble_evt->evt.gatts_evt.params.write.handle, data_buffer);
//    }
//    else if(p_ble_evt->evt.gatts_evt.params.write.handle == p_our_service->char_handles.cccd_handle)
//    {
//        // Get data
//        sd_ble_gatts_value_get(p_our_service->conn_handle, p_our_service->char_handles.cccd_handle, &rx_data);
//        // Print handle and value 
//        //printf("Value received on handle %#06x: %#06x\r\n", p_ble_evt->evt.gatts_evt.params.write.handle, data_buffer);
//        if(data_buffer == 0x0001)
//        {
//            //printf("Notification enabled\r\n");
//        }
//        else if(data_buffer == 0x0000)
//        {
//            //printf("Notification disabled\r\n");
//        }
//    }
//}


void ble_probe_error_service_on_ble_evt(ble_pes_t * p_pes, ble_evt_t * p_ble_evt)
{

    // OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
    switch (p_ble_evt->header.evt_id)
    {        
        case BLE_GATTS_EVT_WRITE:
            //on_ble_write(p_pes, p_ble_evt);
            break;
        case BLE_GAP_EVT_CONNECTED:
            p_pes->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_pes->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            // No implementation needed.
            break;
    }
}

void probe_error_char_add(ble_pes_t * p_pes)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = PROBE_ERROR_CHAR_UUID;
    BLE_UUID_BLE_ASSIGN(char_uuid, PROBE_ERROR_CHAR_UUID);
//    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
//    APP_ERROR_CHECK(err_code);
    
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
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
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
    err_code = sd_ble_gatts_characteristic_add(p_pes->service_handle, &char_md, &attr_char_value, &p_pes->char_handles);
    APP_ERROR_CHECK(err_code);
}

void ble_probe_error_service_init(ble_pes_t * p_probe_error_service)
{
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Decalre service UUIDs and add them to the BLE stack  *****/
    ble_uuid_t service_uuid;
    ble_uuid128_t base_uuid = PROBE_ERROR_BASE_UUID;
    service_uuid.uuid = PROBE_ERROR_SERVICE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
    APP_ERROR_CHECK(err_code);
    
    p_probe_error_service->conn_handle = BLE_CONN_HANDLE_INVALID; //Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_probe_error_service->service_handle);
    APP_ERROR_CHECK(err_code);
    
    /******* add charateristic  *******/
    probe_error_char_add(p_probe_error_service);
}


uint32_t ble_probe_error_update(ble_pes_t * p_pes, uint8_t probe_error_code)
{
    if (p_pes == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    if (probe_error_code != p_pes->probe_error_code) //notify anytime the value changes
    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));
        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = &probe_error_code;

        // Update database.
        err_code = sd_ble_gatts_value_set(p_pes->conn_handle, p_pes->char_handles.value_handle, &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            //save probe error code
            p_pes->probe_error_code = probe_error_code;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_pes->conn_handle != BLE_CONN_HANDLE_INVALID))
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_pes->char_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            
            err_code = sd_ble_gatts_hvx(p_pes->conn_handle, &hvx_params);
            SEGGER_RTT_printf(0, "notifying %d", probe_error_code);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}
