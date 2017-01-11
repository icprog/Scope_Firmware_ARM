#include <stdint.h>
#include <string.h>
#include "nrf_gpio.h"
#include "profile_service.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"
#include "SPI_utils.h"
#include "timers.h"

void profile_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUID and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = PROFILE_CHAR_UUID;
    char_uuid.type = p_ps->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
    
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
    
    /***** Declare char UUID and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = PROFILE_IDS_CHAR_UUID;
    char_uuid.type = p_ps->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
    /****** add write properties ******/
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
    attr_char_value.max_len     = sizeof(uint16_t);
    attr_char_value.init_len    = sizeof(uint16_t);
    uint8_t value               = 0x0000;
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_ps->service_handle, &char_md, &attr_char_value, &p_ps->profile_ids_char_handles);
    APP_ERROR_CHECK(err_code);
}

void transfer_ids_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUID and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = TRANSFER_IDS_CHAR_UUID;
    char_uuid.type = p_ps->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
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
    attr_char_value.max_len     = 4;
    attr_char_value.init_len    = 4;
    uint8_t value               = 0x00000000;
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_ps->service_handle, &char_md, &attr_char_value, &p_ps->transfer_ids_char_handles);
    APP_ERROR_CHECK(err_code);
}

void location_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUID and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = LOCATION_CHAR_UUID;
    char_uuid.type = p_ps->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
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
    attr_char_value.max_len     = 8;
    attr_char_value.init_len    = 8;
    uint8_t value               = 0x0000000000000000;
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_ps->service_handle, &char_md, &attr_char_value, &p_ps->location_char_handles);
    APP_ERROR_CHECK(err_code);
}

void profile_error_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUID and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = PROFILE_ERROR_CHAR_UUID;
    char_uuid.type = p_ps->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
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

void raw_data_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUID and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = RAW_DATA_CHAR_UUID;
    char_uuid.type = p_ps->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
    /****** add read/write properties ******/
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
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); //needed for write or notify
    
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
    err_code = sd_ble_gatts_characteristic_add(p_ps->service_handle, &char_md, &attr_char_value, &p_ps->raw_data_char_handles);
    APP_ERROR_CHECK(err_code);
}

void profile_length_char_add(ble_ps_t * p_ps)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUIDs and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = PROFILE_LENGTH_CHAR_UUID;
    char_uuid.type = p_ps->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
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
    attr_char_value.max_len     = 2;
    attr_char_value.init_len    = 2;
    uint8_t value               = 0x00;
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_ps->service_handle, &char_md, &attr_char_value, &p_ps->profile_length_char_handles);
    APP_ERROR_CHECK(err_code);
}

void ble_profile_service_init(ble_ps_t * p_profile_service)
{
        uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Decalre service UUIDs and add them to the BLE stack  *****/
    
    ble_uuid128_t base_uuid = PROFILE_BASE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &(p_profile_service->uuid_type));
    APP_ERROR_CHECK(err_code);
    
    ble_uuid_t service_uuid;
    service_uuid.uuid = PROFILE_SERVICE_UUID;
    service_uuid.type = p_profile_service->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
    p_profile_service->conn_handle = BLE_CONN_HANDLE_INVALID; //Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_profile_service->service_handle);
    APP_ERROR_CHECK(err_code);
    
    /******* add charateristics  *******/
    profile_char_add(p_profile_service);
    profile_ids_char_add(p_profile_service);
    transfer_ids_char_add(p_profile_service);
    location_char_add(p_profile_service);
    profile_error_char_add(p_profile_service);
    profile_length_char_add(p_profile_service);
    raw_data_char_add(p_profile_service);
    
}

uint32_t update_profile_length(ble_ps_t * p_ps, uint16_t length)
{
    if (p_ps == NULL)
    {
        return NRF_ERROR_NULL;
    }
    SEGGER_RTT_printf(0, "updating length = %d \n", length);
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
        
    memset(&gatts_value, 0, sizeof(gatts_value)); // Initialize value struct.
    gatts_value.len     = sizeof(uint16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t *)&length;
    
    //update the value
    err_code = sd_ble_gatts_value_set(p_ps->conn_handle,
                                  p_ps->profile_length_char_handles.value_handle,
                                  &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
        //do something noe that length is set
    }
    else
    {
        return err_code;
    }
    
    // Send value if connected and notifying.
    if ((p_ps->conn_handle != BLE_CONN_HANDLE_INVALID))
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ps->profile_length_char_handles.value_handle;
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

uint32_t profile_data_update(ble_ps_t * p_ps, uint8_t * profile_data, uint8_t size, uint8_t * bytes_sent)
{
    static int count=0;
	if (p_ps == NULL)
    {
        //return NRF_ERROR_NULL;
		SEGGER_RTT_printf(0, "error: null profile input \n");
		return NRF_ERROR_INVALID_STATE;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = size*sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = profile_data;

    // Update data.
    err_code = sd_ble_gatts_value_set(p_ps->conn_handle,
                                      p_ps->profile_char_handles.value_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
        //SEGGER_RTT_printf(0, "data update success \n");
    }
    else
    {
        SEGGER_RTT_printf(0, "error %d in profile update\n", err_code);
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
        if (err_code == NRF_SUCCESS)
        {
            count+=gatts_value.len;
            *bytes_sent = gatts_value.len;
            //SEGGER_RTT_printf(0, "data to phone: %d\n", count);
        }
        else
        {
            *bytes_sent = 0;
            SEGGER_RTT_printf(0, "error %d in profile update\n", err_code);
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
    return err_code;
}

uint32_t raw_data_update(ble_ps_t * p_ps, uint8_t * raw_data, uint8_t size, uint8_t * bytes_sent)
{
    static int count=0;
	if (p_ps == NULL)
    {
        //return NRF_ERROR_NULL;
		SEGGER_RTT_printf(0, "error: null profile input \n");
		return NRF_ERROR_INVALID_STATE;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = size*sizeof(uint8_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = raw_data;

    // Update data.
    err_code = sd_ble_gatts_value_set(p_ps->conn_handle,
                                      p_ps->raw_data_char_handles.value_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
        //SEGGER_RTT_printf(0, "data update success \n");
    }
    else
    {
        SEGGER_RTT_printf(0, "error in raw data update fxn\n");
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_ps->conn_handle != BLE_CONN_HANDLE_INVALID) )//&& p_ps->is_notification_supported)
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ps->raw_data_char_handles.value_handle; //TODO: change to raw  data char when we are using it
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_ps->conn_handle, &hvx_params);
        if (err_code == NRF_SUCCESS)
        {
            count+=gatts_value.len;
            *bytes_sent = gatts_value.len;
            //SEGGER_RTT_printf(0, "data to phone: %d\n", count);
        }
        else
        {
            *bytes_sent = 0;
            SEGGER_RTT_printf(0, "error %d in raw data update\n", err_code);
        }
	}
    else
    {
        SEGGER_RTT_printf(0, "\n invalid conn handle \n");	
        err_code = NRF_ERROR_INVALID_STATE;
    }
		return err_code;
}

void on_write_profile_service(ble_ps_t * p_ps, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    profile_t * p_profile;
    if(p_evt_write->handle == p_ps->transfer_ids_char_handles.value_handle)
    {
        disable_imu();
        memcpy(&(appData.profile_id), (p_evt_write->data), sizeof(profile_id_t));
        SEGGER_RTT_printf(0,"profile_to_transfer:  type = %d num = %d \n", appData.profile_id.type, appData.profile_id.test_num);
        appData.state = APP_STATE_REQUEST_PROFILE;
    }
    else if(p_evt_write->handle == p_ps->location_char_handles.value_handle)
    {
        SEGGER_RTT_printf(0,"location received \n");
        memcpy(metadata.location, p_evt_write->data, 2*sizeof(float));
        memcpy(profile_data.metadata.location, p_evt_write->data, 2*sizeof(float));
    }
}
void ble_profile_service_on_ble_evt(ble_ps_t * p_ps, ble_evt_t * p_ble_evt)
{
	if (p_ps == NULL || p_ble_evt == NULL)
    {
		SEGGER_RTT_WriteString(0, "profile null \n");
        return;
    }
    //SEGGER_RTT_WriteString(0, "profile evt handler \n");
    switch (p_ble_evt->header.evt_id)
    {        
        case BLE_GATTS_EVT_WRITE:
			//SEGGER_RTT_WriteString(0, "profile evt handler --write evt \n");
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

void profile_ids_update(ble_ps_t * p_ps, uint16_t max_profile_num)
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

    gatts_value.len     = sizeof(uint16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t *)&max_profile_num;

    // Update data.
    err_code = sd_ble_gatts_value_set(p_ps->conn_handle,
                                      p_ps->profile_ids_char_handles.value_handle,
                                      &gatts_value);
    if (err_code == NRF_SUCCESS)
    {
        //SEGGER_RTT_printf(0, "data update success \n");
    }
    else
    {
        SEGGER_RTT_printf(0, "error %d in profile ids update\n", err_code);
    }

    // Send value if connected and notifying.
    if ((p_ps->conn_handle != BLE_CONN_HANDLE_INVALID) )//&& p_ps->is_notification_supported)
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_ps->profile_ids_char_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_ps->conn_handle, &hvx_params);
        if (err_code == NRF_SUCCESS)
        {
            //SEGGER_RTT_printf(0, "data update success \n");
        }
        else
        {
            SEGGER_RTT_printf(0, "error %d in profile ids update\n", err_code);
        }
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
    }
}
