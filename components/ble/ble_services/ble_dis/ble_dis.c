/*
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "ble_dis.h"

#include <stdlib.h>
#include <string.h>
#include "app_error.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "SEGGER_RTT.h"
#include "app.h"
#include "timers.h"
extern device_info_t device_info;

void device_name_char_add(ble_dis_t * p_dis, char * device_name)
{
        uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUIDs and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = DEVICE_NAME_CHAR_UUID;
    char_uuid.type = p_dis->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 1;
    char_md.char_props.read = 1;
    
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
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); //needed for write or notify
    
    /**** Configure the characteristic value attribute  ****/
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));     
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    /***  Set characteristic length in number of bytes  ****/
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    attr_char_value.p_value     = (uint8_t *)device_name;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_dis->service_handle, &char_md, &attr_char_value, &p_dis->device_name_handles);
    APP_ERROR_CHECK(err_code);
}

void serial_number_char_add(ble_dis_t * p_dis, char * serial_number)
{
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUIDs and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = SERIAL_NUM_CHAR_UUID;
    char_uuid.type = p_dis->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    //char_md.char_props.write = 1;
    char_md.char_props.read = 1;
    
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
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); //needed for write or notify
    
    /**** Configure the characteristic value attribute  ****/
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));     
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    /***  Set characteristic length in number of bytes  ****/
    attr_char_value.max_len     = 6;
    attr_char_value.init_len    = 6;
    attr_char_value.p_value     = (uint8_t *)serial_number;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_dis->service_handle, &char_md, &attr_char_value, &p_dis->serial_num_handles);
    APP_ERROR_CHECK(err_code);
}
void hw_rev_char_add(ble_dis_t * p_dis, char * hw_rev)
{
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUIDs and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = HW_REV_CHAR_UUID;
    char_uuid.type = p_dis->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    //char_md.char_props.write = 1;
    char_md.char_props.read = 1;
    
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
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); //needed for write or notify
    
    /**** Configure the characteristic value attribute  ****/
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));     
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    /***  Set characteristic length in number of bytes  ****/
    attr_char_value.max_len     = 3;
    attr_char_value.init_len    = 3;
    attr_char_value.p_value     = (uint8_t *)hw_rev;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_dis->service_handle, &char_md, &attr_char_value, &p_dis->hw_rev_handles);
    APP_ERROR_CHECK(err_code);
}
void fw_rev_char_add(ble_dis_t * p_dis, char * fw_rev)
{
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Declare char UUIDs and add them to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = FW_REV_CHAR_UUID;
    char_uuid.type = p_dis->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    //char_md.char_props.write = 1;
    char_md.char_props.read = 1;
    
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
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); //needed for write or notify
    
    /**** Configure the characteristic value attribute  ****/
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));     
    attr_char_value.p_uuid      = &char_uuid;
    attr_char_value.p_attr_md   = &attr_md;
    
    /***  Set characteristic length in number of bytes  ****/
    attr_char_value.max_len     = 20;
    attr_char_value.init_len    = 20;
    attr_char_value.p_value     = (uint8_t *)fw_rev;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_dis->service_handle, &char_md, &attr_char_value, &p_dis->fw_rev_handles);
    APP_ERROR_CHECK(err_code);
}

void ble_device_info_service_init(ble_dis_t * p_dis)
{
     uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /***** Decalre service UUIDs and add them to the BLE stack  *****/
    
    ble_uuid128_t base_uuid = DEVICE_INFO_BASE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &(p_dis->uuid_type));
    APP_ERROR_CHECK(err_code);
    
    ble_uuid_t service_uuid;
    service_uuid.uuid = SCOPE_UUID_DEVICE_INFO;
    service_uuid.type = p_dis->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, PROFILE_SERVICE_UUID);
    
    p_dis->conn_handle = BLE_CONN_HANDLE_INVALID; //Set our service connection handle to default value. I.e. an invalid handle since we are not yet in a connection.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_dis->service_handle);
    APP_ERROR_CHECK(err_code);
    
    //build fw and hw strings
    char fw_string[20];
    strcpy(fw_string, (char *)(device_info.PIC_firmware_version));
    strcat(fw_string, ",");
    strcat(fw_string, FW_VERSION);
    char hw_string[] = "4";
    
    /******* add charateristics  *******/
    device_name_char_add(p_dis, device_info.device_name);
    serial_number_char_add(p_dis, device_info.serial_number);
    hw_rev_char_add(p_dis, hw_string);
    fw_rev_char_add(p_dis, fw_string);
}

void on_write_device_info(ble_dis_t * p_dis, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    if(p_evt_write->handle == p_dis->device_name_handles.value_handle)
    {
        disable_imu();
//        memcpy(&(appData.profile_id), (p_evt_write->data), sizeof(profile_id_t));
//        SEGGER_RTT_printf(0,"profile_to_transfer:  type = %d num = %d \n", appData.profile_id.type, appData.profile_id.test_num);
//        appData.state = APP_STATE_REQUEST_PROFILE;
    }
}

void ble_device_info_service_on_ble_evt(ble_dis_t * p_dis, ble_evt_t * p_ble_evt)
{
	if (p_dis == NULL || p_ble_evt == NULL)
    {
		SEGGER_RTT_WriteString(0, "service null \n");
        return;
    }
    //SEGGER_RTT_WriteString(0, "profile evt handler \n");
    switch (p_ble_evt->header.evt_id)
    {        
        case BLE_GATTS_EVT_WRITE:
			//SEGGER_RTT_WriteString(0, "profile evt handler --write evt \n");
			on_write_device_info(p_dis, p_ble_evt);
            break;
        case BLE_GAP_EVT_CONNECTED:
            p_dis->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
        case BLE_GAP_EVT_DISCONNECTED:
            p_dis->conn_handle = BLE_CONN_HANDLE_INVALID;
            break;
        default:
            // No implementation needed.
            break;
    }
}
