/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

#include "ble_dev_status.h"
#include "ble_lbs.h"
#include "ble_srv_common.h"
#include "sdk_common.h"



void ble_slope_on_ble_evt(ble_dev_status_t * p_dev, ble_evt_t * p_ble_evt)
{
		initial_slope = 0;
		ble_slope_update(p_dev, 7); //test value = 7
		initial_slope=0;
    if (p_dev == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            //on_connect(p_bas, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            //on_disconnect(p_bas, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            //on_write(p_bas, p_ble_evt);
						initial_slope=0;
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the Connect event.
 *
 * @param[in] p_dev_status      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(ble_dev_status_t * p_dev_status, ble_evt_t * p_ble_evt)
{
    p_dev_status->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_dev_status      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_dev_status_t * p_dev_status, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt); 
    p_dev_status->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in] p_dev_status      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_dev_status_t * p_dev_status, ble_evt_t * p_ble_evt)
{
	initial_slope=0;
		ble_slope_update(p_dev_status, 7); //test value = 7
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_dev_status->slope_char_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_dev_status-> dev_status_write_handler != NULL))
    {
        p_dev_status-> dev_status_write_handler(p_dev_status, p_evt_write->data[0]);
				ble_dev_status_on_other_button_change(p_dev_status, 5);
    }
		ble_slope_update(p_dev_status, 7); //test value = 7
}


void ble_device_status_on_ble_evt(ble_dev_status_t * p_dev_status, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_dev_status, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_dev_status, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write(p_dev_status, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Device Status Characteristic.
 *
 * @param[in] p_dev_status      LED Button Service structure.
 * @param[in] p_dev_status_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */


static uint32_t hardware_rev_char_add(ble_dev_status_t * p_dev_status, const ble_dev_status_init_t * p_dev_status_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_dev_status->uuid_type;
    ble_uuid.uuid = CHAR_UUID_HARDWARE_REV; 
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_dev_status->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_dev_status->slope_char_handles);
}

/**@brief Function for adding the Probing Errors Characteristic.
 *
 * @param[in] p_dev_status      LED Button Service structure.
 * @param[in] p_dev_status_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */


static uint32_t probing_errors_char_add(ble_dev_status_t * p_dev_status, const ble_dev_status_init_t * p_dev_status_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_dev_status->uuid_type;
    ble_uuid.uuid = SCOPE_UUID_PROBING_ERRORS; 
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_dev_status->service_handle,
                                           &char_md,
                                           &attr_char_value,
		&p_dev_status->slope_char_handles);  //TODO: need to change led_char_handles to something else
}


static uint32_t snow_profile_char_add(ble_dev_status_t * p_dev_status, const ble_dev_status_init_t * p_dev_status_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_dev_status->uuid_type;
    ble_uuid.uuid = SCOPE_UUID_SNOW_PROFILE; 
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_dev_status->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_dev_status->slope_char_handles);
}

/**@brief Function for adding the Probing Errors Characteristic.
 *
 * @param[in] p_dev_status      LED Button Service structure.
 * @param[in] p_dev_status_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */


static uint32_t slope_char_add(ble_dev_status_t * p_dev_status, const ble_dev_status_init_t * p_dev_status_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_dev_status->uuid_type;
    ble_uuid.uuid = SCOPE_UUID_SLOPE; 
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = &initial_slope;

    return sd_ble_gatts_characteristic_add(p_dev_status->service_handle,
                                           &char_md,
                                           &attr_char_value,
		&p_dev_status->slope_char_handles);  
}


/**@brief Function for adding the Button Characteristic.
 *
 * @param[in] p_dev_status      LED Button Service structure.
 * @param[in] p_dev_status_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t battery_char_add(ble_dev_status_t * p_dev_status, const ble_dev_status_init_t * p_dev_status_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    
    memset(&char_md, 0, sizeof(char_md));
    
    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_dev_status->uuid_type;
    ble_uuid.uuid = SCOPE_UUID_BATTERY;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t[3]); //DH was uint8_t
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_dev_status->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_dev_status->slope_char_handles);  //change to battery char handles?
}

uint32_t ble_device_status_init(ble_dev_status_t * p_dev_status, const ble_dev_status_init_t * p_dev_status_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_dev_status->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_dev_status-> dev_status_write_handler = p_dev_status_init->led_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {DEV_STATUS_UUID_BASE}; //was LBS UUID_BASE
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_dev_status->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_dev_status->uuid_type;
    ble_uuid.uuid = SCOPE_UUID_DEVICE_STATUS;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_dev_status->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
		
		err_code = snow_profile_char_add(p_dev_status, p_dev_status_init);
    VERIFY_SUCCESS(err_code);
		
		err_code = slope_char_add(p_dev_status, p_dev_status_init);
		VERIFY_SUCCESS(err_code);
		
		err_code = probing_errors_char_add(p_dev_status, p_dev_status_init);
    VERIFY_SUCCESS(err_code);
		
		err_code = hardware_rev_char_add(p_dev_status, p_dev_status_init);
		VERIFY_SUCCESS(err_code);
		
		err_code = battery_char_add(p_dev_status, p_dev_status_init);
		VERIFY_SUCCESS(err_code);
		
		
		
//		err_code = button_char_add(p_dev_status, p_dev_status_init);
//    VERIFY_SUCCESS(err_code);

    

//    err_code = led_char_add(p_dev_status, p_dev_status_init);
//    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t ble_dev_status_on_button_change(ble_dev_status_t * p_dev_status, uint8_t button_state)
{
	
		uint8_t button_state2[3] = {1,2,3}; //DH test
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(button_state2); //DH test
    
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
   // params.handle = p_dev_status->button_char_handles.value_handle;
    params.p_data = button_state2; //had & DH
    params.p_len = &len;
    
    return sd_ble_gatts_hvx(p_dev_status->conn_handle, &params);
}
uint32_t ble_dev_status_on_other_button_change(ble_dev_status_t * p_dev_status, uint8_t button_state)
{
	
		uint8_t button_state2[3] = {5,6,7}; //DH test
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(button_state2); //DH test
    
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    //params.handle = p_dev_status->button_char_handles.value_handle;
    params.p_data = button_state2; //had & DH
    params.p_len = &len;
    
    return sd_ble_gatts_hvx(p_dev_status->conn_handle, &params);
}
//function for updating slope values:
uint32_t ble_slope_update(ble_dev_status_t * p_dev, uint8_t slope)
{
	
		uint8_t button_state2[3] = {5,6,7}; //DH test
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(button_state2); //DH test
    
    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    //params.handle = p_dev_status->button_char_handles.value_handle;
    params.p_data = button_state2; //had & DH
    params.p_len = &len;
    
    return sd_ble_gatts_hvx(p_dev->conn_handle, &params);
	
	
	
	
//    if (p_dev == NULL)
//    {
//        return NRF_ERROR_NULL;
//    }
//    
//    uint32_t err_code = NRF_SUCCESS;
//    ble_gatts_value_t gatts_value;

//    if (slope != p_dev->slope)
//    {
//        // Initialize value struct.
//        memset(&gatts_value, 0, sizeof(gatts_value));

//        gatts_value.len     = sizeof(uint8_t);
//        gatts_value.offset  = 0;
//        gatts_value.p_value = &slope;

//        // Update database.
//        err_code = sd_ble_gatts_value_set(p_dev->conn_handle,
//                                          p_dev->slope_char_handles.value_handle,
//                                          &gatts_value);
//        if (err_code == NRF_SUCCESS)
//        {
//            // Save new battery value.
//            p_dev->slope = slope;
//        }
//        else
//        {
//            return err_code;
//        }

//        // Send value if connected and notifying.
//        if ((p_dev->conn_handle != BLE_CONN_HANDLE_INVALID))// && p_dev->is_notification_supported)
//        {
//            ble_gatts_hvx_params_t hvx_params;

//            memset(&hvx_params, 0, sizeof(hvx_params));

//            hvx_params.handle = p_dev->slope_char_handles.value_handle;
//            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
//            hvx_params.offset = gatts_value.offset;
//            hvx_params.p_len  = &gatts_value.len;
//            hvx_params.p_data = gatts_value.p_value;

//            err_code = sd_ble_gatts_hvx(p_dev->conn_handle, &hvx_params);
//        }
//        else
//        {
//            err_code = NRF_ERROR_INVALID_STATE;
//        }
//    }

//    return err_code;
}

