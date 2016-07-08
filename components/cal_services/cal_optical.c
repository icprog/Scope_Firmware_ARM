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

#include "cal_optical.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "SEGGER_RTT.h"
#include "SPI_utils.h"
#include "calibration.h"

#define INVALID_BATTERY_LEVEL 255


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_optical       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(cal_optical_t * p_optical, ble_evt_t * p_ble_evt)
{
    p_optical->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_optical       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(cal_optical_t * p_optical, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_optical->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_optical       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
//    ble_gatts_char_handles_t      cal_result_handles;          /**< Handles related to the Battery Level characteristic. */
//		ble_gatts_char_handles_t      cal_optical_cal_handles;          /**< Handles related to the Battery Level characteristic. */
//		ble_gatts_char_handles_t      cal_optical_data_handles;          /**< Handles related to the Battery Level characteristic. */
//		ble_gatts_char_handles_t      cal_test_vars_handles;          /**< Handles related to the Battery Level characteristic. */
static void on_write(cal_optical_t * p_optical, ble_evt_t * p_ble_evt)
{
	
	
		SEGGER_RTT_printf(0, "optical write fxn\n");
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	
	if ((p_evt_write->handle == p_optical->cal_test_vars_handles.value_handle) &&
        (p_evt_write->len <= 3))// &&(p_optical->optical_write_handler != NULL))
    {
			SEGGER_RTT_printf(0, "optical data write handler data[0] \n");
			SEGGER_RTT_printf(0,"input: %d ",p_evt_write->data[0]);
        //p_optical->optical_write_handler(p_optical, p_evt_write->data[0]); //null pointer crashes processor...
			cal_data.optical_parameters[0] = p_evt_write->data[0]; //length
			cal_data.optical_parameters[1] = p_evt_write->data[1]; //max speed
			cal_data.optical_parameters[2] = p_evt_write->data[2]; // tolerance
			send_data_to_PIC(optical_cal_length_pack);
			
    }
    if (p_optical->is_notification_supported)
    {
        //ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

        if (
            (p_evt_write->handle == p_optical->cal_result_handles.cccd_handle)
            &&
            (p_evt_write->len == 2)
           )
        {
            // CCCD written, call application event handler
            if (p_optical->evt_handler != NULL)
            {
                cal_optical_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CAL_OPTICAL_EVT_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = CAL_OPTICAL_EVT_NOTIFICATION_DISABLED;
                }
								p_optical->evt_handler(p_optical, &evt);
                //p_optical->result_handler(p_optical, 13);
            }
        }
				if (
            (p_evt_write->handle == p_optical->cal_optical_cal_handles.cccd_handle)
            &&
            (p_evt_write->len == 2)
           )
        {
            // CCCD written, call application event handler
            if (p_optical->evt_handler != NULL)
            {
                cal_optical_evt_t evt;

                if (ble_srv_is_notification_enabled(p_evt_write->data))
                {
                    evt.evt_type = CAL_OPTICAL_EVT_NOTIFICATION_ENABLED;
                }
                else
                {
                    evt.evt_type = CAL_OPTICAL_EVT_NOTIFICATION_DISABLED;
                }

                p_optical->cal_handler(p_optical, &evt);
            }
        }
    }
}


void cal_optical_on_ble_evt(cal_optical_t * p_optical, ble_evt_t * p_ble_evt)
{
		SEGGER_RTT_WriteString(0, "CAL OPTICAL BLE \n");
    if (p_optical == NULL || p_ble_evt == NULL)
    {
				SEGGER_RTT_WriteString(0, "optical NULL \n");
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
					SEGGER_RTT_WriteString(0, "optical conn \n");
            on_connect(p_optical, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
					SEGGER_RTT_WriteString(0, "optical discon\n");
            on_disconnect(p_optical, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
					SEGGER_RTT_WriteString(0, "optical write \n");
            on_write(p_optical, p_ble_evt);
            break;

        default:
            // No implementation needed.
				SEGGER_RTT_WriteString(0, "optical default \n");
            break;
    }
}


/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_optical        Battery Service structure.
 * @param[in]   p_optical_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cal_result_char_add(cal_optical_t * p_optical, const cal_optical_init_t * p_optical_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_cal_result;
    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
    uint8_t             init_len;

    // Add Battery Level characteristic
    if (p_optical->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to OPTICAL_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        cccd_md.write_perm = p_optical_init->cal_result_char_attr_md.cccd_write_perm;
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = (p_optical->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_optical->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_OPTICAL_CAL_RESULT_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_optical_init->cal_result_char_attr_md.read_perm;
    attr_md.write_perm = p_optical_init->cal_result_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    initial_cal_result = p_optical_init->initial_batt_level;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = &initial_cal_result;

    err_code = sd_ble_gatts_characteristic_add(p_optical->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_optical->cal_result_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_optical_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

        memset(&attr_md, 0, sizeof(attr_md));

        attr_md.read_perm = p_optical_init->cal_result_report_read_perm;
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen    = 0;
        
        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_optical_init->p_report_ref);
        
        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid    = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len  = init_len;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len   = attr_char_value.init_len;
        attr_char_value.p_value   = encoded_report_ref;

        err_code = sd_ble_gatts_descriptor_add(p_optical->cal_result_handles.value_handle,
                                               &attr_char_value,
                                               &p_optical->report_ref_handle);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {
        p_optical->report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_optical        Battery Service structure.
 * @param[in]   p_optical_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cal_optical_cal_char_add(cal_optical_t * p_optical, const cal_optical_init_t * p_optical_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_cal_result;
    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
    uint8_t             init_len;

    // Add Battery Level characteristic
    if (p_optical->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to OPTICAL_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
				BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
        //cccd_md.write_perm = p_optical_init->cal_result_char_attr_md.cccd_write_perm;
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = (p_optical->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_optical->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_OPTICAL_CAL_OPTICAL_CAL_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_optical_init->cal_result_char_attr_md.read_perm;
    attr_md.write_perm = p_optical_init->cal_result_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    initial_cal_result = p_optical_init->initial_batt_level;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(float);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(float);
    attr_char_value.p_value   = &initial_cal_result;

    err_code = sd_ble_gatts_characteristic_add(p_optical->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_optical->cal_optical_cal_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_optical_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

        memset(&attr_md, 0, sizeof(attr_md));

        attr_md.read_perm = p_optical_init->cal_result_report_read_perm;
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen    = 0;
        
        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_optical_init->p_report_ref);
        
        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid    = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len  = init_len;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len   = attr_char_value.init_len;
        attr_char_value.p_value   = encoded_report_ref;

        err_code = sd_ble_gatts_descriptor_add(p_optical->cal_result_handles.value_handle,
                                               &attr_char_value,
                                               &p_optical->report_ref_handle);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {
        p_optical->report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_optical        Battery Service structure.
 * @param[in]   p_optical_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cal_optical_data_add(cal_optical_t * p_optical, const cal_optical_init_t * p_optical_init)
{
    uint32_t            err_code;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;
    uint8_t             initial_cal_result = 13;
    uint8_t             encoded_report_ref[BLE_SRV_ENCODED_REPORT_REF_LEN];
    uint8_t             init_len;

    // Add Battery Level characteristic
    if (p_optical->is_notification_supported)
    {
        memset(&cccd_md, 0, sizeof(cccd_md));

        // According to OPTICAL_SPEC_V10, the read operation on cccd should be possible without
        // authentication.
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
        cccd_md.write_perm = p_optical_init->cal_result_char_attr_md.cccd_write_perm;
        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
    }

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = (p_optical->is_notification_supported) ? 1 : 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = (p_optical->is_notification_supported) ? &cccd_md : NULL;
    char_md.p_sccd_md         = NULL;

    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_OPTICAL_CAL_OPTICAL_DATA_CHAR);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.read_perm  = p_optical_init->cal_result_char_attr_md.read_perm;
    attr_md.write_perm = p_optical_init->cal_result_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    initial_cal_result = p_optical_init->initial_batt_level;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(uint8_t);
    attr_char_value.p_value   = &initial_cal_result;

    err_code = sd_ble_gatts_characteristic_add(p_optical->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_optical->cal_optical_data_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    if (p_optical_init->p_report_ref != NULL)
    {
        // Add Report Reference descriptor
        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

        memset(&attr_md, 0, sizeof(attr_md));

        attr_md.read_perm = p_optical_init->cal_result_report_read_perm;
        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
        attr_md.rd_auth = 0;
        attr_md.wr_auth = 0;
        attr_md.vlen    = 0;
        
        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_optical_init->p_report_ref);
        
        memset(&attr_char_value, 0, sizeof(attr_char_value));

        attr_char_value.p_uuid    = &ble_uuid;
        attr_char_value.p_attr_md = &attr_md;
        attr_char_value.init_len  = init_len;
        attr_char_value.init_offs = 0;
        attr_char_value.max_len   = attr_char_value.init_len;
        attr_char_value.p_value   = encoded_report_ref;

        err_code = sd_ble_gatts_descriptor_add(p_optical->cal_result_handles.value_handle,
                                               &attr_char_value,
                                               &p_optical->report_ref_handle);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else
    {
        p_optical->report_ref_handle = BLE_GATT_HANDLE_INVALID;
    }

    return NRF_SUCCESS;
}

/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_optical        Battery Service structure.
 * @param[in]   p_optical_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
//static uint32_t cal_test_vars_add(cal_optical_t * p_optical, const cal_optical_init_t * p_optical_init)
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
////    if (p_optical->is_notification_supported)
////    {
////        memset(&cccd_md, 0, sizeof(cccd_md));

////        // According to OPTICAL_SPEC_V10, the read operation on cccd should be possible without
////        // authentication.
////        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
////        cccd_md.write_perm = p_optical_init->cal_result_char_attr_md.cccd_write_perm;
////        cccd_md.vloc       = BLE_GATTS_VLOC_STACK;
////    }

//    memset(&char_md, 0, sizeof(char_md));

//    char_md.char_props.read   = 0;
//		char_md.char_props.write   = 1;
//    char_md.char_props.notify = 0;//(p_optical->is_notification_supported) ? 1 : 0;
//    char_md.p_char_user_desc  = NULL;
//    char_md.p_char_pf         = NULL;
//    char_md.p_user_desc_md    = NULL;
//    char_md.p_cccd_md         = NULL;//(p_optical->is_notification_supported) ? &cccd_md : NULL;
//    char_md.p_sccd_md         = NULL;

//    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_OPTICAL_CAL_TEST_VARS_CHAR);

//    memset(&attr_md, 0, sizeof(attr_md));

//    attr_md.read_perm  = p_optical_init->cal_result_char_attr_md.read_perm;
//    attr_md.write_perm = p_optical_init->cal_result_char_attr_md.write_perm;
//    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
//    attr_md.rd_auth    = 0;
//    attr_md.wr_auth    = 1;
//    attr_md.vlen       = 0;

//		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

//    initial_cal_result = p_optical_init->initial_batt_level;

//    memset(&attr_char_value, 0, sizeof(attr_char_value));

//    attr_char_value.p_uuid    = &ble_uuid;
//    attr_char_value.p_attr_md = &attr_md;
//    attr_char_value.init_len  = sizeof(uint8_t);
//    attr_char_value.init_offs = 0;
//    attr_char_value.max_len   = sizeof(uint8_t);
//    attr_char_value.p_value   = &initial_cal_result;

//    err_code = sd_ble_gatts_characteristic_add(p_optical->service_handle, &char_md,
//                                               &attr_char_value,
//                                               &p_optical->cal_test_vars_handles);
//    if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }

////    if (p_optical_init->p_report_ref != NULL)
////    {
////        // Add Report Reference descriptor
////        BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_REPORT_REF_DESCR);

////        memset(&attr_md, 0, sizeof(attr_md));

////        attr_md.read_perm = p_optical_init->cal_result_report_read_perm;
////        BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);

////        attr_md.vloc    = BLE_GATTS_VLOC_STACK;
////        attr_md.rd_auth = 0;
////        attr_md.wr_auth = 0;
////        attr_md.vlen    = 0;
////        
////        init_len = ble_srv_report_ref_encode(encoded_report_ref, p_optical_init->p_report_ref);
////        
////        memset(&attr_char_value, 0, sizeof(attr_char_value));

////        attr_char_value.p_uuid    = &ble_uuid;
////        attr_char_value.p_attr_md = &attr_md;
////        attr_char_value.init_len  = init_len;
////        attr_char_value.init_offs = 0;
////        attr_char_value.max_len   = attr_char_value.init_len;
////        attr_char_value.p_value   = encoded_report_ref;

////        err_code = sd_ble_gatts_descriptor_add(p_optical->cal_result_handles.value_handle,
////                                               &attr_char_value,
////                                               &p_optical->report_ref_handle);
////        if (err_code != NRF_SUCCESS)
////        {
////            return err_code;
////        }
////    }
////    else
////    {
////        p_optical->report_ref_handle = BLE_GATT_HANDLE_INVALID;
////    }

//    return NRF_SUCCESS;
//}


//static uint32_t cal_test_vars_char_add(cal_optical_t * p_optical, const cal_optical_init_t * p_optical_init)
void cal_test_vars_char_add(cal_optical_t * p_optical)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = BLE_UUID_OPTICAL_CAL_TEST_VARS_CHAR;
    BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_OPTICAL_CAL_TEST_VARS_CHAR); //TODO might be redundant witht he previous line

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
    attr_char_value.max_len     = 3;
    attr_char_value.init_len    = 3;  // expecting 3 x uint8_t
    uint8_t value               = 0x00;
    attr_char_value.p_value     = &value;
    
    /**** add it too the softdevice  *****/
    err_code = sd_ble_gatts_characteristic_add(p_optical->service_handle, &char_md, &attr_char_value, &p_optical->cal_test_vars_handles);
    //APP_ERROR_CHECK(err_code);
}




uint32_t cal_optical_init(cal_optical_t * p_optical, const cal_optical_init_t * p_optical_init)
{
		//    // Initialize Battery Service.
	
	//recover optical_init:
		cal_optical_init_t * optical_init = (cal_optical_init_t *)p_optical_init;  //undo const declaration
		//cal_optical_init_t optical_init = *ptr;  //get back to cal_optical_init_t
    

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&optical_init->cal_result_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&optical_init->cal_result_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&optical_init->cal_result_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&optical_init->cal_result_report_read_perm);

    optical_init->evt_handler          = NULL;
    optical_init->support_notification = true;
    optical_init->p_report_ref         = NULL;
    optical_init->initial_batt_level   = 100;

    
    if (p_optical == NULL || p_optical_init == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_optical->evt_handler               = p_optical_init->evt_handler;
    p_optical->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_optical->is_notification_supported = p_optical_init->support_notification;
    p_optical->cal_result_last        = INVALID_BATTERY_LEVEL;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_CAL_OPTICAL_SERVICE);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_optical->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add battery level characteristic
		
//    err_code = cal_test_vars_add(p_optical, p_optical_init);
//		if (err_code != NRF_SUCCESS)
//    {
//        return err_code;
//    }
		cal_test_vars_char_add(p_optical);
		
		err_code = cal_optical_data_add(p_optical, p_optical_init);
		if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		err_code = cal_optical_cal_char_add(p_optical, p_optical_init);
		if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		err_code = cal_result_char_add(p_optical, p_optical_init);
		if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		return err_code;
}





uint32_t optical_cal_result_update(cal_optical_t * p_optical, uint8_t cal_result)
{
    if (p_optical == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

//    if (cal_result != p_optical->cal_result_last)
//    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = &cal_result;

        // Update dataopticale.
        err_code = sd_ble_gatts_value_set(p_optical->conn_handle,
                                          p_optical->cal_result_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new battery value.
            p_optical->cal_result_last = cal_result;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_optical->conn_handle != BLE_CONN_HANDLE_INVALID) && p_optical->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_optical->cal_result_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_optical->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    //}

    return err_code;
}
uint32_t optical_cal_update(cal_optical_t * p_optical, float cal_result)
{
    if (p_optical == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    //if (cal_result != p_optical->cal_result_last)
    //{
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(float);
        gatts_value.offset  = 0;
        gatts_value.p_value = (uint8_t *)&cal_result; 

        // Update dataopticale.
        err_code = sd_ble_gatts_value_set(p_optical->conn_handle,
                                          p_optical->cal_optical_cal_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new battery value.
            p_optical->cal_result_last = cal_result;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_optical->conn_handle != BLE_CONN_HANDLE_INVALID) && p_optical->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_optical->cal_optical_cal_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_optical->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    //}

    return err_code;
}


uint32_t optical_cal_data_update(cal_optical_t * p_optical, uint8_t cal_result)
{
    if (p_optical == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

//    if (cal_result != p_optical->cal_result_last)
//    {
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = &cal_result;

        // Update dataopticale.
        err_code = sd_ble_gatts_value_set(p_optical->conn_handle,
                                          p_optical->cal_optical_data_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new battery value.
            p_optical->cal_result_last = cal_result;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_optical->conn_handle != BLE_CONN_HANDLE_INVALID) && p_optical->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_optical->cal_optical_data_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_optical->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    //}

    return err_code;
}

uint32_t optical_cal_test_vars_update(cal_optical_t * p_optical, uint8_t cal_result)
{
    if (p_optical == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    //if (cal_result != p_optical->cal_result_last)
    //{
        // Initialize value struct.
        memset(&gatts_value, 0, sizeof(gatts_value));

        gatts_value.len     = sizeof(uint8_t);
        gatts_value.offset  = 0;
        gatts_value.p_value = &cal_result;

        // Update dataopticale.
        err_code = sd_ble_gatts_value_set(p_optical->conn_handle,
                                          p_optical->cal_test_vars_handles.value_handle,
                                          &gatts_value);
        if (err_code == NRF_SUCCESS)
        {
            // Save new battery value.
            p_optical->cal_result_last = cal_result;
        }
        else
        {
            return err_code;
        }

        // Send value if connected and notifying.
        if ((p_optical->conn_handle != BLE_CONN_HANDLE_INVALID) && p_optical->is_notification_supported)
        {
            ble_gatts_hvx_params_t hvx_params;

            memset(&hvx_params, 0, sizeof(hvx_params));

            hvx_params.handle = p_optical->cal_test_vars_handles.value_handle;
            hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
            hvx_params.offset = gatts_value.offset;
            hvx_params.p_len  = &gatts_value.len;
            hvx_params.p_data = gatts_value.p_value;

            err_code = sd_ble_gatts_hvx(p_optical->conn_handle, &hvx_params);
        }
        else
        {
            err_code = NRF_ERROR_INVALID_STATE;
        }
    //}

    return err_code;
}

