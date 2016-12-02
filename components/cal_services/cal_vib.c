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

#include "cal_vib.h"

#include <stdlib.h>
#include <string.h>
#include "app_error.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "SEGGER_RTT.h"
#include "app.h"



#define cal_vib_SYS_ID_LEN 8  /**< Length of System ID Characteristic Value. */
#define cal_vib_PNP_ID_LEN 7  /**< Length of Pnp ID Characteristic Value. */

static uint16_t                 service_handle;
static ble_gatts_char_handles_t test_vars_handles;
static ble_gatts_char_handles_t vib_data_handles;
static ble_gatts_char_handles_t vib_cal_handles;
static ble_gatts_char_handles_t cal_result_handles;
//static ble_gatts_char_handles_t pnp_id_handles;





//static uint32_t cal_weights_char_add(cal_vib_t * p_vib, const cal_vib_init_t * p_vib_init)
void cal_vib_char_add(cal_vib_t * p_vib)
{
    
    uint32_t err_code; // Variable to hold return codes from library and softdevice functions
    
    /****** add char UUID ******/
    ble_uuid_t          char_uuid;
    char_uuid.uuid      = SCOPE_CHAR_UUID_VIB	;
    BLE_UUID_BLE_ASSIGN(char_uuid, SCOPE_CHAR_UUID_VIB	); //TODO might be redundant witht he previous line

    /****** add read write properties ******/
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.write = 1;
    //char_md.char_props.read = 1;
    

    
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
    err_code = sd_ble_gatts_characteristic_add(p_vib->service_handle, &char_md, &attr_char_value, &p_vib->vib_handles);
    APP_ERROR_CHECK(err_code);
}





uint32_t cal_vib_init(cal_vib_t * p_vib, const cal_vib_init_t * p_vib_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
		
		cal_vib_init_t * vib_init = (cal_vib_init_t *)p_vib_init;  //undo const declaration
		//cal_vib_init_t vib_init = *ptr;  //get back to cal_vib_init_t
		
		
    // Here the sec level for the Vibration Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&vib_init->vib_char_cccd_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&vib_init->vib_char_cccd_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&vib_init->vib_char_cccd_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&vib_init->vib_report_read_perm);

    vib_init->evt_handler          = NULL;
    vib_init->support_notification = true;
    vib_init->p_report_ref         = NULL;
    vib_init->initial_batt_level   = 100;
		//p_vib_init->vib_write_handler = vib_write_handler;
    
    if (p_vib == NULL || p_vib_init == NULL)
    {
        return NRF_ERROR_NULL;
    }
    


    // Initialize service structure
    p_vib->evt_handler               = p_vib_init->evt_handler;
    p_vib->conn_handle               = BLE_CONN_HANDLE_INVALID;
    p_vib->is_notification_supported = p_vib_init->support_notification;
    p_vib->vib_level_last          = NULL;
		p_vib->vib_write_handler		 = p_vib_init->vib_write_handler;
		
    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, SCOPE_UUID_VIB_CAL);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_vib->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add characteristics
    cal_vib_char_add(p_vib); //weights characteristic




    return err_code;
}

static void on_connect(cal_vib_t * p_vib, ble_evt_t * p_ble_evt)
{
    p_vib->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_vib       vib Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(cal_vib_t * p_vib, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_vib->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_vib       vib Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(cal_vib_t * p_vib, ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	
	if ((p_evt_write->handle == p_vib->vib_handles.value_handle) &&
        (p_evt_write->len <= 2))
    {
		SEGGER_RTT_printf(0, "vib data write handler data[0] \n");
		SEGGER_RTT_printf(0,"input: %d",p_evt_write->data[0]);
		appData.state = APP_STATE_START_VIB_CAL;	
    }
		
}


void cal_vib_on_ble_evt(cal_vib_t * p_vib, ble_evt_t * p_ble_evt)
{
    
    if (p_vib == NULL || p_ble_evt == NULL)
    {
				//SEGGER_RTT_WriteString(0, "CAL vib NULL \n");
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
					//SEGGER_RTT_WriteString(0, "CAL vib CONN \n");
            on_connect(p_vib, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
					//SEGGER_RTT_WriteString(0, "CAL vib DISCONN \n");
            on_disconnect(p_vib, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
						//SEGGER_RTT_WriteString(0, "CAL vib WRITE \n");
            on_write(p_vib, p_ble_evt);
            break;

        default:
					//SEGGER_RTT_WriteString(0, "CAL vib DEFAULT \n");
					//on_write(p_vib, p_ble_evt);
            // No implementation needed.
            break;
			}
}

