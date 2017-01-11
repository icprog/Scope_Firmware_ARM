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

#include "ble_status.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_error.h"


#define INVALID_status_LEVEL 255


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_status       status Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_status_t * p_status, ble_evt_t * p_ble_evt)
{
    p_status->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_status       status Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_status_t * p_status, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_status->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_status       status Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_status_t * p_status, ble_evt_t * p_ble_evt)
{
    if (p_status->is_notification_supported)
    {
        ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

        if (
            (p_evt_write->handle == p_status->status_level_handles.cccd_handle)
            &&
            (p_evt_write->len == 2)
           )
        {
            // CCCD written, call application event handler
            if (p_status->evt_handler != NULL)
            {
                ble_status_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = BLE_status_EVT_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = BLE_status_EVT_NOTIFICATION_DISABLED;
                }

                p_status->evt_handler(p_status, &evt);
            }
        }
    }
}


void ble_status_on_ble_evt(ble_status_t * p_status, ble_evt_t * p_ble_evt)
{
    if (p_status == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_status, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_status, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_status, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the status Level characteristic.
 *
 * @param[in]   p_status        status Service structure.
 * @param[in]   p_status_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t status_level_char_add(ble_status_t * p_status, const ble_status_init_t * p_status_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_status_level;
    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
    uint8_t             init_len;

    /***** Declare char UUID and add it to the BLE stack  *****/
    ble_uuid_t char_uuid;
    char_uuid.uuid = STATUS_CHAR_UUID;
    char_uuid.type = p_status->uuid_type;
   // BLE_UUID_BLE_ASSIGN(char_uuid, STATUS_CHAR_UUID);
    
    // Add status Level characteristic
    if (p_status->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to status_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        cccd_md.write_perm = p_status_init->status_level_char_attr_md.cccd_write_perm;
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1; // was 1, DH
    char_md.char_props.notify = (p_status->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_status->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_status_init->status_level_char_attr_md.read_perm;
    attr_md.write_perm = p_status_init->status_level_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    initial_status_level = p_status_init->initial_batt_level;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = &initial_status_level;

    err_code = sd_ble_gatts_characteristic_add(p_status->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_status->status_level_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_status_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_REPORT_REF_DESCR);

        memset(&attr_md, 0, sizeof(attr_md));

        attr_md.read_perm = p_status_init->status_level_report_read_perm;
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen    = 0;
        
        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_status_init->p_report_ref);
        
        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid    = &char_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len  = init_len;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len   = attr_char_value.init_len;
        attr_char_value.p_value   = encoded_report_ref;

        err_code = sd_ble_gatts_descriptor_add(p_status->status_level_handles.value_handle,
                                               &attr_char_value,
                                               &p_status->report_ref_handle);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {
        p_status->report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}


uint32_t ble_status_init(ble_status_t * p_status, const ble_status_init_t * p_status_init)
{
	uint32_t   err_code;
	ble_status_init_t * status_init = (ble_status_init_t *)p_status_init;  //undo const declaration
	
    /***** Declare service UUID and add it to the BLE stack  *****/
    ble_uuid128_t base_uuid = STATUS_BASE_UUID;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &(p_status->uuid_type));
    APP_ERROR_CHECK(err_code);
    
    ble_uuid_t service_uuid;
    service_uuid.uuid = SCOPE_UUID_STATUS;
    service_uuid.type = p_status->uuid_type;
    //BLE_UUID_BLE_ASSIGN(service_uuid, SCOPE_UUID_STATUS);
    
	// Here the sec level for the status Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&status_init->status_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&status_init->status_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&status_init->status_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&status_init->status_level_report_read_perm);

    status_init->evt_handler          = NULL;
    status_init->support_notification = true;
    status_init->p_report_ref         = NULL;
    status_init->initial_batt_level   = 100;
	
    if (p_status == NULL || p_status_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    // Initialize service structure
    p_status->evt_handler               = p_status_init->evt_handler;
    p_status->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_status->is_notification_supported = p_status_init->support_notification;
    p_status->status_level_last        = INVALID_status_LEVEL;

    // Add service

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &p_status->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add status level characteristic
    return status_level_char_add(p_status, p_status_init);
}


uint32_t ble_status_status_level_update(ble_status_t * p_status, uint8_t status_level)
{
    if (p_status == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    if (status_level != p_status->status_level_last)
    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = &status_level;

        // Update datastatuse.
        err_code = sd_ble_gatts_value_set(p_status->conn_handle,
                                          p_status->status_level_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new status value.
            p_status->status_level_last = status_level;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_status->conn_handle != BLE_CONN_HANDLE_INVALID) && p_status->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_status->status_level_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_status->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    }

    return err_code;
}
