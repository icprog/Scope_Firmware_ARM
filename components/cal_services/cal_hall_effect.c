/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

#include "cal_hall_effect.h"

#include <stdlib.h>
#include <string.h>
#include "app_error.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "SEGGER_RTT.h"
#include "calibration.h"
#include "app.h"


#define cal_hall_effect_SYS_ID_LEN 8  /**< Length of System ID Characteristic Value. */
#define cal_hall_effect_PNP_ID_LEN 7  /**< Length of Pnp ID Characteristic Value. */

static uint16_t                 service_handle;
static ble_gatts_char_handles_t test_vars_handles;
static ble_gatts_char_handles_t hall_effect_data_handles;
static ble_gatts_char_handles_t hall_effect_cal_handles;
static ble_gatts_char_handles_t cal_result_handles;
//static ble_gatts_char_handles_t pnp_id_handles;

extern  APP_DATA appData;

///**@brief Function for encoding a PnP ID.
// *
// * @param[out]  p_encoded_buffer   Buffer where the encoded data will be written.
// * @param[in]   p_pnp_id           PnP ID to be encoded.
// */
//static void pnp_id_encode(uint8_t * p_encoded_buffer, const cal_hall_effect_pnp_id_t * p_pnp_id)
//{
//    uint8_t len = 0;

//    APP_ERROR_CHECK_BOOL(p_pnp_id != NULL);
//    APP_ERROR_CHECK_BOOL(p_encoded_buffer != NULL);

//    p_encoded_buffer[len++] = p_pnp_id->vendor_id_source;

//    len += uint16_encode(p_pnp_id->vendor_id, &p_encoded_buffer[len]);
//    len += uint16_encode(p_pnp_id->product_id, &p_encoded_buffer[len]);
//    len += uint16_encode(p_pnp_id->product_version, &p_encoded_buffer[len]);

//    APP_ERROR_CHECK_BOOL(len == cal_hall_effect_PNP_ID_LEN);
//}


/**@brief Function for adding the Characteristic.
 *
 * @param[in]   uuid           UUID of characteristic to be added.
 * @param[in]   p_char_value   Initial value of characteristic to be added.
 * @param[in]   char_len       Length of initial value. This will also be the maximum value.
 * @param[in]   hall_effect_attr_md    Security settings of characteristic to be added.
 * @param[out]  p_handles      Handles of new characteristic.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
//static uint32_t cal_weights_char_add(cal_hall_effect_t * p_hall_effect, const cal_hall_effect_init_t * p_hall_effect_init)
//{
//    uint32_t            err_code;
//    ble_gatts_char_md_t char_md;
//    ble_gatts_attr_md_t cccd_md;
//    ble_gatts_attr_t    attr_char_value;
//    ble_uuid_t          ble_uuid;
//    ble_gatts_attr_md_t attr_md;
//    uint8_t             initial_cal_result = 3;
//    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
//    uint8_t             init_len;

//    // Add Battery Level characteristic
////    if (p_hall_effect->is_notification_supported)
////    {
////        memset(&cccd_md, 0, sizeof(cccd_md));

////        // According to hall_effect_SPEC_V10, the read operation on cccd should be possible without
////        // authentication.
////        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
////				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
////        cccd_md.write_perm = p_hall_effect_init->hall_effect_char_cccd_attr_md.cccd_write_perm;
////        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
////    }
//		
//		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
//		
//    memset(&char_md, 0, sizeof(char_md));

//    char_md.char_props.read   = 0;
//		char_md.char_props.write  = 1;
//    char_md.char_props.notify = 0;//(p_hall_effect->is_notification_supported) ? 1 : 0;
//    char_md.p_char_user_desc  = NULL;
//    char_md.p_char_pf         = NULL;
//    char_md.p_user_desc_md    = NULL;
//    char_md.p_cccd_md         = NULL;//(p_hall_effect->is_notification_supported) ? &cccd_md : NULL;
//    char_md.p_sccd_md         = NULL;

//    BLE_UUID_BLE_ASSIGN(ble_uuid, SCOPE_CHAR_UUID_WEIGHTS);

//    memset(&attr_md, 0, sizeof(attr_md));

//    attr_md.read_perm  = p_hall_effect_init->hall_effect_char_cccd_attr_md.read_perm;
//    attr_md.write_perm = p_hall_effect_init->hall_effect_char_cccd_attr_md.write_perm;
//    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//    attr_md.rd_auth    = 1;
//    attr_md.wr_auth    = 1;
//    attr_md.vlen       = 0;

//    initial_cal_result = p_hall_effect_init->initial_batt_level;

//    memset(&attr_char_value, 0, sizeof(attr_char_value));

//    attr_char_value.p_uuid    = &ble_uuid;
//    attr_char_value.p_attr_md = &attr_md;
//    attr_char_value.init_len  = sizeof(uint8_t);
//    attr_char_value.init_offs = 0;
//    attr_char_value.max_len   = sizeof(uint8_t);
//    attr_char_value.p_value   = &initial_cal_result;

//    err_code = sd_ble_gatts_characteristic_add(p_hall_effect->service_handle, &char_md,
//                                               &attr_char_value,
//                                               &p_hall_effect->hall_effect_weight_handles);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }

//    if (p_hall_effect_init->p_report_ref != NULL)
//    {
//        // Add Report Reference descriptor
//        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

//        memset(&attr_md, 0, sizeof(attr_md));

//        attr_md.read_perm = p_hall_effect_init->hall_effect_report_read_perm;
//        //BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

//        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
//        attr_md.rd_auth = 0;
//        attr_md.wr_auth = 1;
//        attr_md.vlen    = 0;
//        
//        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_hall_effect_init->p_report_ref);
//        
//        memset(&attr_char_value, 0, sizeof(attr_char_value));

//        attr_char_value.p_uuid    = &ble_uuid;
//        attr_char_value.p_attr_md = &attr_md;
//        attr_char_value.init_len  = init_len;
//        attr_char_value.init_offs = 0;
//        attr_char_value.max_len   = attr_char_value.init_len;
//        attr_char_value.p_value   = encoded_report_ref;

//        err_code = sd_ble_gatts_descriptor_add(p_hall_effect->hall_effect_weight_handles.value_handle,
//                                               &attr_char_value,
//                                               &p_hall_effect->report_ref_handle);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }
//    }
//    else
//    {
//        p_hall_effect->report_ref_handle = BLE_GATT_HANDLE_INVALID;
//    }

//    return err_code;
//}


//static uint32_t cal_weights_char_add(cal_hall_effect_t * p_hall_effect, const cal_hall_effect_init_t * p_hall_effect_init)
//void cal_weights_char_add(cal_hall_effect_t * p_hall_effect)
//{
//    
//    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
//    
//    /****** add char UUID ******/
//    ble_uuid_t          char_uuid;
//    char_uuid.uuid      = SCOPE_CHAR_UUID_WEIGHTS;
//    BLE_UUID_BLE_ASSIGN(char_uuid, SCOPE_CHAR_UUID_WEIGHTS); //TODO might be redundant witht he previous line

//    /****** add read write properties ******/
//    ble_gatts_char_md_t char_md;
//    memset(&char_md, 0, sizeof(char_md));
//    char_md.char_props.write = 1;
//    //char_md.char_props.read = 1;
//    
//    /******   Configuring Client Characteristic Configuration Descriptor metadata and add to char_md structure   ****/
////    ble_gatts_attr_md_t cccd_md;
////    memset(&cccd_md, 0, sizeof(cccd_md));
////    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
////    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
////    cccd_md.vloc                = BLE_GATTS_VLOC_STACK;    
////    char_md.p_cccd_md           = &cccd_md;
////    char_md.char_props.notify   = 1;
//    
//    /*** Configure the attribute metadata ***/
//    ble_gatts_attr_md_t attr_md;
//    memset(&attr_md, 0, sizeof(attr_md)); 
//    attr_md.vloc        = BLE_GATTS_VLOC_STACK;   
//    
//    /****Set read/write security levels to our characteristic ***/
//    //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm); //needed for write or notify
//    
//    /**** Configure the characteristic value attribute  ****/
//    ble_gatts_attr_t    attr_char_value;
//    memset(&attr_char_value, 0, sizeof(attr_char_value));     
//    attr_char_value.p_uuid      = &char_uuid;
//    attr_char_value.p_attr_md   = &attr_md;
//    
//    /***  Set characteristic length in number of bytes  ****/
//    attr_char_value.max_len     = 3;
//    attr_char_value.init_len    = 3;
//    uint8_t value               = 0x00;
//    attr_char_value.p_value     = &value;
//    
//    /**** add it too the softdevice  *****/
//    err_code = sd_ble_gatts_characteristic_add(p_hall_effect->service_handle, &char_md, &attr_char_value, &p_hall_effect->hall_effect_weight_handles);
//    APP_ERROR_CHECK(err_code);
//}

//static uint32_t cal_weights_char_add(cal_hall_effect_t * p_hall_effect, const cal_hall_effect_init_t * p_hall_effect_init)
void cal_test_add(cal_hall_effect_t * p_hall_effect)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = SCOPE_CHAR_UUID_HALL_EFFECT_TEST;
    BLE_UUID_BLE_ASSIGN(char_uuid, SCOPE_CHAR_UUID_HALL_EFFECT_TEST); //TODO might be redundant witht he previous line

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
    err_code = sd_ble_gatts_characteristic_add(p_hall_effect->service_handle, &char_md, &attr_char_value, &p_hall_effect->hall_effect_test_handles);
    APP_ERROR_CHECK(err_code);
}

void cal_result_char_add(cal_hall_effect_t * p_hall_effect)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = SCOPE_CHAR_UUID_HALL_EFFECT_RESULT;
    BLE_UUID_BLE_ASSIGN(char_uuid, SCOPE_CHAR_UUID_HALL_EFFECT_RESULT); //TODO might be redundant witht he previous line

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
    err_code = sd_ble_gatts_characteristic_add(p_hall_effect->service_handle, &char_md, &attr_char_value, &p_hall_effect->hall_effect_result_handles);
    APP_ERROR_CHECK(err_code);
}

//calibration points characteristic
//static uint32_t cal_points_char_add(cal_hall_effect_t * p_hall_effect, const cal_hall_effect_init_t * p_hall_effect_init)
//{
//    uint32_t            err_code;
//    ble_gatts_char_md_t char_md;
//    ble_gatts_attr_md_t cccd_md;
//    ble_gatts_attr_t    attr_char_value;
//    ble_uuid_t          ble_uuid;
//    ble_gatts_attr_md_t attr_md;
//    uint8_t             initial_cal_result;
//    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
//    uint8_t             init_len;

//    // Add Battery Level characteristic
//    if (p_hall_effect->is_notification_supported)
//    {
//        memset(&cccd_md, 0, sizeof(cccd_md));

//        // According to hall_effect_SPEC_V10, the read operation on cccd should be possible without
//        // authentication.
//        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//				//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
//        cccd_md.write_perm = p_hall_effect_init->hall_effect_char_cccd_attr_md.cccd_write_perm;
//        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
//				//BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
//    }

//    memset(&char_md, 0, sizeof(char_md));

//    char_md.char_props.read   = 1;
//		//char_md.char_props.write  = 0;
//    char_md.char_props.notify = (p_hall_effect->is_notification_supported) ? 1 : 0;
//    char_md.p_char_user_desc  = NULL;
//    char_md.p_char_pf         = NULL;
//    char_md.p_user_desc_md    = NULL;
//    char_md.p_cccd_md         = (p_hall_effect->is_notification_supported) ? &cccd_md : NULL;
//    char_md.p_sccd_md         = NULL;

//    BLE_UUID_BLE_ASSIGN(ble_uuid, SCOPE_CHAR_UUID_CAL_POINTS);

//    memset(&attr_md, 0, sizeof(attr_md));

//    attr_md.read_perm  = p_hall_effect_init->hall_effect_char_cccd_attr_md.read_perm;
//    attr_md.write_perm = p_hall_effect_init->hall_effect_char_cccd_attr_md.write_perm;
//    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//    attr_md.rd_auth    = 0;
//    attr_md.wr_auth    = 0;
//    attr_md.vlen       = 0;

//    initial_cal_result = p_hall_effect_init->initial_batt_level;

//    memset(&attr_char_value, 0, sizeof(attr_char_value));

//    attr_char_value.p_uuid    = &ble_uuid;
//    attr_char_value.p_attr_md = &attr_md;
//    attr_char_value.init_len  = 14;//sizeof(uint8_t);
//    attr_char_value.init_offs = 0;
//    attr_char_value.max_len   = 14;//sizeof(uint8_t);
//    attr_char_value.p_value   = &initial_cal_result;

//    err_code = sd_ble_gatts_characteristic_add(p_hall_effect->service_handle, &char_md,
//                                               &attr_char_value,
//                                               &p_hall_effect->hall_effect_cal_handles);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }

//    if (p_hall_effect_init->p_report_ref != NULL)
//    {
//        // Add Report Reference descriptor
//        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

//        memset(&attr_md, 0, sizeof(attr_md));

//        attr_md.read_perm = p_hall_effect_init->hall_effect_report_read_perm;
//        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

//        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
//        attr_md.rd_auth = 0;
//        attr_md.wr_auth = 0;
//        attr_md.vlen    = 0;
//        
//        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_hall_effect_init->p_report_ref);
//        
//        memset(&attr_char_value, 0, sizeof(attr_char_value));

//        attr_char_value.p_uuid    = &ble_uuid;
//        attr_char_value.p_attr_md = &attr_md;
//        attr_char_value.init_len  = 14;//init_len;
//        attr_char_value.init_offs = 0;
//        attr_char_value.max_len   = 14;//attr_char_value.init_len;
//        attr_char_value.p_value   = encoded_report_ref;

//        err_code = sd_ble_gatts_descriptor_add(p_hall_effect->hall_effect_cal_handles.value_handle,
//                                               &attr_char_value,
//                                               &p_hall_effect->report_ref_handle);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }
//    }
//    else
//    {
//        p_hall_effect->report_ref_handle = BLE_GATT_HANDLE_INVALID;
//    }

//    return err_code;
//}


//calibration result characteristic
//static uint32_t cal_result_char_add(cal_hall_effect_t * p_hall_effect, const cal_hall_effect_init_t * p_hall_effect_init)
//{
//    uint32_t            err_code;
//    ble_gatts_char_md_t char_md;
//    ble_gatts_attr_md_t cccd_md;
//    ble_gatts_attr_t    attr_char_value;
//    ble_uuid_t          ble_uuid;
//    ble_gatts_attr_md_t attr_md;
//    uint8_t             initial_cal_result;
//    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
//    uint8_t             init_len;

//    // Add Battery Level characteristic
//    if (p_hall_effect->is_notification_supported)
//    {
//        memset(&cccd_md, 0, sizeof(cccd_md));

//        // According to hall_effect_SPEC_V10, the read operation on cccd should be possible without
//        // authentication.
//        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
//				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
//        cccd_md.write_perm = p_hall_effect_init->hall_effect_char_cccd_attr_md.cccd_write_perm;
//        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
//    }

//    memset(&char_md, 0, sizeof(char_md));

//    char_md.char_props.read   = 1;
//		char_md.char_props.write  = 0;
//    char_md.char_props.notify = (p_hall_effect->is_notification_supported) ? 1 : 0;
//    char_md.p_char_user_desc  = NULL;
//    char_md.p_char_pf         = NULL;
//    char_md.p_user_desc_md    = NULL;
//    char_md.p_cccd_md         = (p_hall_effect->is_notification_supported) ? &cccd_md : NULL;
//    char_md.p_sccd_md         = NULL;

//    BLE_UUID_BLE_ASSIGN(ble_uuid, SCOPE_CHAR_UUID_RESULT);

//    memset(&attr_md, 0, sizeof(attr_md));

//    attr_md.read_perm  = p_hall_effect_init->hall_effect_char_cccd_attr_md.read_perm;
//    attr_md.write_perm = p_hall_effect_init->hall_effect_char_cccd_attr_md.write_perm;
//    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//    attr_md.rd_auth    = 0;
//    attr_md.wr_auth    = 1;
//    attr_md.vlen       = 0;

//    initial_cal_result = p_hall_effect_init->initial_batt_level;

//    memset(&attr_char_value, 0, sizeof(attr_char_value));

//    attr_char_value.p_uuid    = &ble_uuid;
//    attr_char_value.p_attr_md = &attr_md;
//    attr_char_value.init_len  = sizeof(uint8_t);
//    attr_char_value.init_offs = 0;
//    attr_char_value.max_len   = sizeof(uint8_t);
//    attr_char_value.p_value   = &initial_cal_result;

//    err_code = sd_ble_gatts_characteristic_add(p_hall_effect->service_handle, &char_md,
//                                               &attr_char_value,
//                                               &p_hall_effect->hall_effect_result_handles);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }

//    if (p_hall_effect_init->p_report_ref != NULL)
//    {
//        // Add Report Reference descriptor
//        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

//        memset(&attr_md, 0, sizeof(attr_md));

//        attr_md.read_perm = p_hall_effect_init->hall_effect_report_read_perm;
//        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

//        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
//        attr_md.rd_auth = 0;
//        attr_md.wr_auth = 0;
//        attr_md.vlen    = 0;
//        
//        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_hall_effect_init->p_report_ref);
//        
//        memset(&attr_char_value, 0, sizeof(attr_char_value));

//        attr_char_value.p_uuid    = &ble_uuid;
//        attr_char_value.p_attr_md = &attr_md;
//        attr_char_value.init_len  = init_len;
//        attr_char_value.init_offs = 0;
//        attr_char_value.max_len   = attr_char_value.init_len;
//        attr_char_value.p_value   = encoded_report_ref;

//        err_code = sd_ble_gatts_descriptor_add(p_hall_effect->hall_effect_result_handles.value_handle,
//                                               &attr_char_value,
//                                               &p_hall_effect->report_ref_handle);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }
//    }
//    else
//    {
//        p_hall_effect->report_ref_handle = BLE_GATT_HANDLE_INVALID;
//    }

//    return err_code;
//}


uint32_t cal_hall_effect_init(cal_hall_effect_t * p_hall_effect, const cal_hall_effect_init_t * p_hall_effect_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
		
	cal_hall_effect_init_t * hall_effect_init = (cal_hall_effect_init_t *)p_hall_effect_init;  //undo const declaration
	//cal_hall_effect_init_t hall_effect_init = *ptr;  //get back to cal_hall_effect_init_t
		

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hall_effect_init->hall_effect_char_cccd_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hall_effect_init->hall_effect_char_cccd_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hall_effect_init->hall_effect_char_cccd_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&hall_effect_init->hall_effect_report_read_perm);

    //hall_effect_init->evt_handler          = NULL;
    hall_effect_init->support_notification = true;
    hall_effect_init->p_report_ref         = NULL;
    hall_effect_init->initial_batt_level   = 100;
	//p_hall_effect_init->hall_effect_write_handler = hall_effect_write_handler;
    
    if (p_hall_effect == NULL || p_hall_effect_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Initialize service structure
    p_hall_effect->evt_handler                      = p_hall_effect_init->evt_handler;
    p_hall_effect->conn_handle                      = BLE_CONN_HANDLE_INVALID;
    p_hall_effect->is_notification_supported        = p_hall_effect_init->support_notification;
    p_hall_effect->hall_effect_level_last           = NULL;
	p_hall_effect->hall_effect_write_handler        = p_hall_effect_init->hall_effect_write_handler;
		
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, SCOPE_UUID_HALL_EFFECT_CAL);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_hall_effect->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add characteristics
    cal_test_add(p_hall_effect); //weights characteristic
	//cal_ready_add(p_hall_effect); //ready characteristic
	cal_result_char_add(p_hall_effect);
//		err_code =  cal_points_char_add(p_hall_effect, p_hall_effect_init); // calibration points service
//		if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
//		err_code =  cal_result_char_add(p_hall_effect, p_hall_effect_init); // calibration points service
//		if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }



    return err_code;
}

static void on_connect(cal_hall_effect_t * p_hall_effect, ble_evt_t * p_ble_evt)
{
    p_hall_effect->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_hall_effect       hall_effect Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(cal_hall_effect_t * p_hall_effect, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_hall_effect->conn_handle = BLE_CONN_HANDLE_INVALID;
}

void hall_effect_write_handler(cal_hall_effect_t * p_hall_effect, uint8_t data_in)
{
	
	cal_data.hall_status = data_in;
	if(data_in ==1) appData.state = APP_STATE_HALL_EFFECT_TEST;

}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_hall_effect       hall_effect Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(cal_hall_effect_t * p_hall_effect, ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	
	if ((p_evt_write->handle == p_hall_effect->hall_effect_test_handles.value_handle) &&
        (p_evt_write->len <= 2))// &&(p_hall_effect->hall_effect_write_handler != NULL))
    {
			SEGGER_RTT_printf(0, "hall_effect data write handler data[0] \n");
			SEGGER_RTT_printf(0,"input: %d",p_evt_write->data[0]);

			
			hall_effect_write_handler(p_hall_effect, p_evt_write->data[0]);
    }

    if (p_hall_effect->is_notification_supported)
    {
        //ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

			if (  
            (p_evt_write->handle == p_hall_effect->hall_effect_result_handles.cccd_handle)
            &&
            (p_evt_write->len == 2)
           )
        {
            // CCCD written, call application event handler
            if (p_hall_effect->evt_handler != NULL)
            {
                cal_hall_effect_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = cal_hall_effect_EVT_NOTIFICATION_ENABLED;
									SEGGER_RTT_printf(0, "notification enabled \n");
                }
                else
                {
                    evt.evt_type = cal_hall_effect_EVT_NOTIFICATION_DISABLED;
									SEGGER_RTT_printf(0, "notification disabled \n");
                }

                p_hall_effect->evt_handler(p_hall_effect, &evt);
            }
        }
    }
}


void cal_hall_effect_on_ble_evt(cal_hall_effect_t * p_hall_effect, ble_evt_t * p_ble_evt)
{
    
    if (p_hall_effect == NULL || p_ble_evt == NULL)
    {
				//SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT NULL \n");
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
					//SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT CONN \n");
            on_connect(p_hall_effect, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
					//SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT DISCONN \n");
            on_disconnect(p_hall_effect, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
						//SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT WRITE \n");
            on_write(p_hall_effect, p_ble_evt);
            break;

        default:
					//SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT DEFAULT \n");
					//on_write(p_hall_effect, p_ble_evt);
            // No implementation needed.
            break;
			}
}

//uint32_t cal_result_update(cal_hall_effect_t * p_hall_effect, uint8_t weight)
//{
//    if (p_hall_effect == NULL)
//    {
//        return NRF_ERROR_NULL;
//    }
//    
//    uint32_t err_code = NRF_SUCCESS;
//    ble_gatts_value_t gatts_value;

//    //if (cal_result != p_hall_effect->cal_result_last)
//    //{
//        // Initialize value struct.
//        memset(&gatts_value, 0, sizeof(gatts_value));

//        gatts_value.len     = sizeof(uint8_t);
//        gatts_value.offset  = 0;
//        gatts_value.p_value = &weight;

//        // Update datahall_effecte.
//        err_code = sd_ble_gatts_value_set(p_hall_effect->conn_handle,
//                                          p_hall_effect->hall_effect_weight_handles.value_handle,
//                                          &gatts_value);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }

//        // Send value if connected and notifying.
//        if ((p_hall_effect->conn_handle != BLE_CONN_HANDLE_INVALID) && p_hall_effect->is_notification_supported)
//        {
//            ble_gatts_hvx_params_t hvx_params;

//            memset(&hvx_params, 0, sizeof(hvx_params));

//            hvx_params.handle = p_hall_effect->hall_effect_weight_handles.value_handle;
//            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
//            hvx_params.offset = gatts_value.offset;
//            hvx_params.p_len  = &gatts_value.len;
//            hvx_params.p_data = gatts_value.p_value;

//            err_code = sd_ble_gatts_hvx(p_hall_effect->conn_handle, &hvx_params);
//        }
//        else
//        {
//            err_code = NRF_ERROR_INVALID_STATE;
//        }
//    //}

//    return err_code;
//}

//uint32_t cal_points_update(cal_hall_effect_t * p_hall_effect, uint8_t weight)//[7])
//{
//	uint8_t index = 0;
//	SEGGER_RTT_printf(0, "hall_effect write!\n");
//	
////	SEGGER_RTT_printf(0, "data: %d \n",weight[0]);
////	SEGGER_RTT_printf(0, "data: %d \n",weight[1]);
////	SEGGER_RTT_printf(0, "data: %d \n",weight[2]);
////	SEGGER_RTT_printf(0, "data: %d \n",weight[3]);
//    if (p_hall_effect == NULL)
//    {
//			SEGGER_RTT_printf(0, "**  hall_effect NULL!\n");
//        return NRF_ERROR_NULL;
//    }
//    
//    uint32_t err_code = NRF_SUCCESS;
//    ble_gatts_value_t gatts_value;

//    //if (cal_result != p_hall_effect->cal_result_last)
//    //{
//        // Initialize value struct.
//        memset(&gatts_value, 0, sizeof(gatts_value));

//        gatts_value.len     = 1;//sizeof(uint8_t);
//        gatts_value.offset  = 0;
////				for(index=0;index<6;index++)
////				{
////					gatts_value.p_value[index] = weight[index];
////				}
//				gatts_value.p_value = &weight;//(uint8_t *)weight;
//        // Update datahall_effecte.
//        err_code = sd_ble_gatts_value_set(p_hall_effect->conn_handle,
//                                          p_hall_effect->hall_effect_cal_handles.value_handle,
//                                          &gatts_value);
//        if (err_code != NRF_SUCCESS)
//        {
//            return err_code;
//        }

//        // Send value if connected and notifying.
//        if ((p_hall_effect->conn_handle != BLE_CONN_HANDLE_INVALID) && p_hall_effect->is_notification_supported)
//        {
//            ble_gatts_hvx_params_t hvx_params;

//            memset(&hvx_params, 0, sizeof(hvx_params));

//            hvx_params.handle = p_hall_effect->hall_effect_weight_handles.value_handle;
//            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
//            hvx_params.offset = gatts_value.offset;
//            hvx_params.p_len  = &gatts_value.len;
//            hvx_params.p_data = gatts_value.p_value;

//            err_code = sd_ble_gatts_hvx(p_hall_effect->conn_handle, &hvx_params);
//        }
//        else
//        {
//            err_code = NRF_ERROR_INVALID_STATE;
//        }
//    //}

//    return err_code;
//}

uint32_t cal_result_update(cal_hall_effect_t * p_hall_effect, uint8_t result)
{
	SEGGER_RTT_printf(0, "HALL_EFFECT_UPDATE\n");
    if (p_hall_effect == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

//    if (cal_result != p_hall_effect->cal_result_last)
//    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = 1;//sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = &result;

        // Update datahall_effecte.
        err_code = sd_ble_gatts_value_set(p_hall_effect->conn_handle,
                                          p_hall_effect->hall_effect_result_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new battery value.
            //p_hall_effect->cal_result_last = cal_result;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_hall_effect->conn_handle != BLE_CONN_HANDLE_INVALID) && p_hall_effect->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_hall_effect->hall_effect_result_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_hall_effect->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    //}

    return err_code;
}
//void ble_hall_effect_on_ble_evt(cal_hall_effect_t * p_hall_effect, ble_evt_t * p_ble_evt)
//{
//		SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT BLE \n");
//    if (p_hall_effect == NULL || p_ble_evt == NULL)
//    {
//				SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT NULL \n");
//        return;
//    }
//    
//    switch (p_ble_evt->header.evt_id)
//    {
//        case BLE_GAP_EVT_CONNECTED:
//					SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT CONN \n");
//            on_connect(p_hall_effect, p_ble_evt);
//            break;

//        case BLE_GAP_EVT_DISCONNECTED:
//					SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT DISCONN \n");
//            on_disconnect(p_hall_effect, p_ble_evt);
//            break;

//        case BLE_GATTS_EVT_WRITE:
//						SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT WRITE \n");
//            on_write(p_hall_effect, p_ble_evt);
//            break;

//        default:
//					SEGGER_RTT_WriteString(0, "CAL HALL_EFFECT DEFAULT \n");
//					//on_write(p_hall_effect, p_ble_evt);
//            // No implementation needed.
//            break;
//    }
//}


