#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "fwu_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"
#include "app.h"
#include "spi_utils.h"

/*
* the on write is used to pull hex data from a file on to the phone to the PIC.
*/
void on_write_fwu_service(ble_fwu_t * p_fwu, ble_evt_t * p_ble_evt)
{
    uint8_t ignore_length = 0;
    static uint8_t data_len = 0;
    static uint8_t data_ind = 0;
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    if(p_evt_write->handle == p_fwu->data_char_handles.value_handle)
    {
        
        if(data_len == 0)/***** new line of data  ****/
        {
            ignore_length = 1;
            data_len = (p_evt_write->data)[0] - 1 ; 
            //SEGGER_RTT_printf(0, "expecting %d bytes of data", data_len);
        }
        //SEGGER_RTT_printf(0, "data ind = %d\n", data_ind);
        memcpy(appData.fwu_data_buf + data_ind , (p_evt_write->data) + ignore_length, p_evt_write->len - ignore_length); 
        data_ind += p_evt_write->len - ignore_length;
        if(data_ind == data_len) /***** finished reading line ****/
        {
            fwu_data_pack.data_size = data_ind;
            appData.state = APP_STATE_FWU_DATA_SEND;
            data_len = 0;
            data_ind = 0;

        }
        else if(data_ind > data_len)
        {
            SEGGER_RTT_printf(0, "problem with fwu data packet. got %d expecting %d\n", p_evt_write->len, data_len - data_ind - p_evt_write->len);
        }
        
        
    }
    if(p_evt_write->handle == p_fwu->cmd_char_handles.value_handle)
    {
            fwu_code_t fwu_code = *(p_evt_write->data);
            if(fwu_code == START_ARM_FWU)
            {
                SEGGER_RTT_printf(0, "start ARM _FWU\n");
                //appData.state = APP_STATE_START_ARM_FWU;
            }
            else if(fwu_code == START_PIC_FWU)
            {
                SEGGER_RTT_printf(0, "start PIC FWU\n");
                appData.state = APP_STATE_PIC_FWU_START;
            }
            else if(fwu_code == RESTART)
            {
                appData.state = APP_STATE_RESTART;
            }
    }

}

void ble_fwu_service_on_ble_evt(ble_fwu_t * p_fwu, ble_evt_t * p_ble_evt)
{

    // OUR_JOB: Step 3.D Implement switch case handling BLE events related to our service. 
    switch (p_ble_evt->header.evt_id)
    {                
        case BLE_GATTS_EVT_WRITE:
			on_write_fwu_service(p_fwu, p_ble_evt);
            break;
        case BLE_GAP_EVT_CONNECTED:
            p_fwu->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_fwu->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            // No implementation needed.
            break;
    }
}

void fwu_cmd_char_add(ble_fwu_t * p_fwu)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = FWU_CMD_CHAR_UUID;
    BLE_UUID_BLE_ASSIGN(char_uuid, FWU_CMD_CHAR_UUID);
//    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
//    APP_ERROR_CHECK(err_code);
    
    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 1;
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
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
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
    err_code = sd_ble_gatts_characteristic_add(p_fwu->service_handle, &char_md, &attr_char_value, &p_fwu->cmd_char_handles);
    APP_ERROR_CHECK(err_code);
}

void fwu_data_char_add(ble_fwu_t * p_fwu)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = FWU_DATA_CHAR_UUID;
    BLE_UUID_BLE_ASSIGN(char_uuid, FWU_DATA_CHAR_UUID);
//    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
//    APP_ERROR_CHECK(err_code);
    
    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    //char_md.char_props.write = 1;
    char_md.char_props.write_wo_resp = 1;
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
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    
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
    err_code = sd_ble_gatts_characteristic_add(p_fwu->service_handle, &char_md, &attr_char_value, &p_fwu->data_char_handles);
    APP_ERROR_CHECK(err_code);
}

void ble_fwu_service_init(ble_fwu_t * p_fwu_service)
{
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Decalre service UUIDs and add them to the BLE stack  *****/
    ble_uuid_t service_uuid;
//    ble_uuid128_t base_uuid = FWU_SERVICE_BASE_UUID;
//    service_uuid.uuid = FWU_SERVICE_UUID;
//    err_code = sd_ble_uuid_vs_add(&base_uuid, &service_uuid.type);
//    APP_ERROR_CHECK(err_code);
    
    BLE_UUID_BLE_ASSIGN(service_uuid, FWU_SERVICE_UUID);
    
    p_fwu_service->conn_handle = BLE_CONN_HANDLE_INVALID; //Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_fwu_service->service_handle);
    APP_ERROR_CHECK(err_code);
    
    /******* add charateristics  *******/
    fwu_cmd_char_add(p_fwu_service);
    fwu_data_char_add(p_fwu_service);
}


uint32_t ble_fwu_update(ble_fwu_t * p_fwu, uint8_t fwu_code)
{
    if (p_fwu == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));
    gatts_value.len     = sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = &fwu_code;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_fwu->conn_handle, p_fwu->cmd_char_handles.value_handle, &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
        //successful update
    }
    else
    {
        
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_fwu->conn_handle != BLE_CONN_HANDLE_INVALID))
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_fwu->cmd_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        
        err_code = sd_ble_gatts_hvx(p_fwu->conn_handle, &hvx_params);
        //SEGGER_RTT_printf(0, "FWU cmd notifying %d", fwu_code);
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }

    return err_code;
}
