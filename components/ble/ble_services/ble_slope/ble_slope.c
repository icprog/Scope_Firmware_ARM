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

#include "ble_slope.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "SEGGER_RTT.h"

   /* computes the principal value of the arc sine of x */
   /* a domain error occurs for arguments not in the range -1 to 1 */
   /* and -HUGE_VAL is returned. */
   /* Returns: the arc sine in the range -Pi/2 to Pi/2. */


#define INVALID_slope_LEVEL 255


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_slope       slope Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_slope_t * p_slope, ble_evt_t * p_ble_evt)
{
    p_slope->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_slope       slope Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_slope_t * p_slope, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_slope->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_slope       slope Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_slope_t * p_slope, ble_evt_t * p_ble_evt)
{
    if (p_slope->is_notification_supported)
    {
        ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

        if (
            (p_evt_write->handle == p_slope->slope_level_handles.cccd_handle)
            &&
            (p_evt_write->len == 2)
           )
        {
            // CCCD written, call application event handler
            if (p_slope->evt_handler != NULL)
            {
                ble_slope_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = BLE_slope_EVT_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = BLE_slope_EVT_NOTIFICATION_DISABLED;
                }

                p_slope->evt_handler(p_slope, &evt);
            }
        }
    }
}


void ble_slope_on_ble_evt(ble_slope_t * p_slope, ble_evt_t * p_ble_evt)
{
    if (p_slope == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_slope, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_slope, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_slope, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the slope Level characteristic.
 *
 * @param[in]   p_slope        slope Service structure.
 * @param[in]   p_slope_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t slope_level_char_add(ble_slope_t * p_slope, const ble_slope_init_t * p_slope_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_slope_level;
    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
    uint8_t             init_len;

    // Add slope Level characteristic
    if (p_slope->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to slope_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        cccd_md.write_perm = p_slope_init->slope_level_char_attr_md.cccd_write_perm;
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 0; // was 1, DH
    char_md.char_props.notify = (p_slope->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_slope->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid,  SLOPE_CHAR_UUID); //was slope-level char

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_slope_init->slope_level_char_attr_md.read_perm;
    attr_md.write_perm = p_slope_init->slope_level_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    initial_slope_level = p_slope_init->initial_batt_level;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = &initial_slope_level;

    err_code = sd_ble_gatts_characteristic_add(p_slope->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_slope->slope_level_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_slope_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

        memset(&attr_md, 0, sizeof(attr_md));

        attr_md.read_perm = p_slope_init->slope_level_report_read_perm;
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen    = 0;
        
        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_slope_init->p_report_ref);
        
        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid    = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len  = init_len;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len   = attr_char_value.init_len;
        attr_char_value.p_value   = encoded_report_ref;

        err_code = sd_ble_gatts_descriptor_add(p_slope->slope_level_handles.value_handle,
                                               &attr_char_value,
                                               &p_slope->report_ref_handle);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {
        p_slope->report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}


uint32_t ble_slope_init(ble_slope_t * p_slope, const ble_slope_init_t * p_slope_init)
{
		
		ble_slope_init_t * slope_init = (ble_slope_init_t *)p_slope_init;  //undo const declaration
	
		// Here the sec level for the slope Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&slope_init->slope_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&slope_init->slope_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&slope_init->slope_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&slope_init->slope_level_report_read_perm);

    slope_init->evt_handler          = NULL;
    slope_init->support_notification = true;
    slope_init->p_report_ref         = NULL;
    slope_init->initial_batt_level   = 100;
	
    if (p_slope == NULL || p_slope_init == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_slope->evt_handler               = p_slope_init->evt_handler;
    p_slope->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_slope->is_notification_supported = p_slope_init->support_notification;
    p_slope->slope_level_last        = INVALID_slope_LEVEL;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, SCOPE_UUID_SLOPE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_slope->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add slope level characteristic
    return slope_level_char_add(p_slope, p_slope_init);
}
//float ava_exp(float base, uint32_t ex)
//{
//    uint32_t ii;
//    float output = base;
//    if(ex == 0)
//    {
//        output = 1;
//    }
//    else if(ex == 1)
//    {
//        output = base;
//    }    
//    else
//    {
//        for(ii=2;ii<=ex;ii++)
//        {
//            output = output * base;
//        }
//    }
//    return output;
//}

//float factorial(uint32_t n)
//{
//  int c;
//  float result = 1;
// 
//  for (c = 1; c <= n; c++)
//    result = result * c;
// 
//  return result;
//}
//float num_asin(float x,int iterations)
//{
//    float out = 0;
//    uint16_t n = 0;
//    for(n=0;n<iterations;n++)
//    {
//        out = out + ((factorial(2*n))/((ava_exp(2,(2*n)))*(factorial(n))*factorial(n))*((ava_exp(x,(2*n+1)))/(2*n+1)));
//    }
//    
//    return out;
//}

//// calculate slope:

//uint8_t slope_calc(int16_t ax,int16_t ay,int16_t az)
//{
//    float slope = 0;
//    float inner = 0;
//    float lower = 0;
//    float roll = 0;
//    float pitch = 0;
//    float grav_full = 1310;
//    //get pitch
////    lower = (ax*ax) + (az*az);
////    inner = ay/(sqrt(lower));
////    pitch = atan(inner);
////    pitch = (pitch/(2*3.14159))*360;
//    
//    //get roll
////    lower = (ay*ay) + (az*az);
////    inner = ax/(sqrt(lower));
////    pitch = atan(inner);
////    pitch = (pitch/(2*3.14159))*360;
//    if(ay < 0) ay = 0-ay;
//    slope = num_asin((float)ay/grav_full,8);
//   // slope = (slope/(2*3.14159))*360;
//    slope = (slope*180)/3.14159;
//    if(slope<0)slope = 0-slope;
//    return (uint8_t)slope;
//    
//    
//}

uint32_t ble_slope_level_update(ble_slope_t * p_slope, uint8_t slope_level)
{
    if (p_slope == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    if (slope_level != p_slope->slope_level_last)
    {
        
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = &slope_level;

        // Update dataslopee.
        err_code = sd_ble_gatts_value_set(p_slope->conn_handle,
                                          p_slope->slope_level_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new slope value.
            p_slope->slope_level_last = slope_level;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_slope->conn_handle != BLE_CONN_HANDLE_INVALID) && p_slope->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_slope->slope_level_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_slope->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}
