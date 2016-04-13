
#include "ble_mtn.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"


#define BLE_UUID_ACC_INFO_CHAR 0x2A56	
#define BLE_UUID_GYRO_INFO_CHAR	0x2A57

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_mtn       motion service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_mtn_t * p_mtn, ble_evt_t * p_ble_evt)
{
    p_mtn->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_mtn       motion service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_mtn_t * p_mtn, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_mtn->conn_handle = BLE_CONN_HANDLE_INVALID;
}


static void on_cccd_write(ble_mtn_t* p_mtn, ble_gatts_evt_write_t* p_evt_write)
{
	if(p_evt_write->len == 2){
		if(p_mtn->evt_handler != NULL){
			ble_mtn_evt_t evt;
			if(ble_srv_is_notification_enabled(p_evt_write->data)){
				evt.evt_type = BLE_MTN_EVT_NOTIFICATION_ENABLED;
			}else{
				evt.evt_type = BLE_MTN_EVT_NOTIFICATION_DISABLED;
			}
			p_mtn->evt_handler(p_mtn, &evt);
		}
	}
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_mtn       motion service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_mtn_t * p_mtn, ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if(p_evt_write->handle == p_mtn->acc_status_handles.cccd_handle){
		on_cccd_write(p_mtn, p_evt_write);
	}
	if(p_evt_write->handle == p_mtn->gyro_status_handles.cccd_handle){
		on_cccd_write(p_mtn, p_evt_write);
	}
}


void ble_mtn_on_ble_evt(ble_mtn_t * p_mtn, ble_evt_t * p_ble_evt)
{
	switch (p_ble_evt->header.evt_id)
	{
		case BLE_GAP_EVT_CONNECTED:
			on_connect(p_mtn, p_ble_evt);
			break;

		case BLE_GAP_EVT_DISCONNECTED:
			on_disconnect(p_mtn, p_ble_evt);
			break;

		case BLE_GATTS_EVT_WRITE:
			on_write(p_mtn, p_ble_evt);
			break;

		default:
			// No implementation needed.
			break;
	}
}

/**@brief Function for adding the gyro information  characteristic.
 *
 * @param[in]   p_mtn        motion service structure.
 * @param[in]   p_mtn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t gyro_status_char_add(ble_mtn_t * p_mtn, const ble_mtn_init_t * p_mtn_init)
{
	uint32_t            err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	uint8_t 			initial_value[7] = {0};
	memset(&cccd_md, 0, sizeof(cccd_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	cccd_md.write_perm = p_mtn_init->gyro_status_char_attr_md.cccd_write_perm;
	cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.notify = 1;
	char_md.char_props.read = 1;
	char_md.p_char_user_desc  = NULL;
	char_md.p_char_pf         = NULL;
	char_md.p_user_desc_md    = NULL;
	char_md.p_cccd_md         = &cccd_md;
	char_md.p_sccd_md         = NULL;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_GYRO_INFO_CHAR);

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm  = p_mtn_init->gyro_status_char_attr_md.read_perm;
	attr_md.write_perm = p_mtn_init->gyro_status_char_attr_md.write_perm;
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;


	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = 7;
	attr_char_value.p_value = initial_value; 

	err_code = sd_ble_gatts_characteristic_add(p_mtn->service_handle, &char_md,
			&attr_char_value,
			&p_mtn->gyro_status_handles);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}


	return NRF_SUCCESS;
}

/**@brief Function for adding the accelerometer information  characteristic.
 *
 * @param[in]   p_mtn        motion service structure.
 * @param[in]   p_mtn_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t acc_status_char_add(ble_mtn_t * p_mtn, const ble_mtn_init_t * p_mtn_init)
{
	uint32_t            err_code;
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t    attr_char_value;
	ble_uuid_t          ble_uuid;
	ble_gatts_attr_md_t attr_md;
	uint8_t 			initial_value[7] = {0};
	
	memset(&cccd_md, 0, sizeof(cccd_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	cccd_md.write_perm = p_mtn_init->acc_status_char_attr_md.cccd_write_perm;
	cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read = 1;
	char_md.char_props.notify = 1;
	char_md.p_char_user_desc  = NULL;
	char_md.p_char_pf         = NULL;
	char_md.p_user_desc_md    = NULL;
	char_md.p_cccd_md         = &cccd_md;
	char_md.p_sccd_md         = NULL;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ACC_INFO_CHAR);

	memset(&attr_md, 0, sizeof(attr_md));

	attr_md.read_perm  = p_mtn_init->acc_status_char_attr_md.read_perm;
	attr_md.write_perm = p_mtn_init->acc_status_char_attr_md.write_perm;
	attr_md.vloc       = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth    = 0;
	attr_md.wr_auth    = 0;
	attr_md.vlen       = 1;


	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid    = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len  = 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len   = 7;
	attr_char_value.p_value = initial_value; 

	err_code = sd_ble_gatts_characteristic_add(p_mtn->service_handle, &char_md,
			&attr_char_value,
			&p_mtn->acc_status_handles);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}


	return NRF_SUCCESS;
}


uint32_t ble_mtn_init(ble_mtn_t * p_mtn, const ble_mtn_init_t * p_mtn_init)
{
	uint32_t   err_code;
	ble_uuid_t ble_uuid;

	// Initialize service structure
	p_mtn->evt_handler               = p_mtn_init->evt_handler;
	p_mtn->conn_handle               = BLE_CONN_HANDLE_INVALID;

	// Add service
	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ALERT_NOTIFICATION_SERVICE);

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_mtn->service_handle);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	

	err_code = acc_status_char_add(p_mtn, p_mtn_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	err_code = gyro_status_char_add(p_mtn, p_mtn_init);
	if (err_code != NRF_SUCCESS)
	{
		return err_code;
	}
	
	return NRF_SUCCESS;
}


uint32_t ble_mtn_acc_status_update(ble_mtn_t * p_mtn, int16_t*  motion_status, uint8_t acc_range)
{
	uint32_t err_code = NRF_SUCCESS;

	uint8_t value[7] = {0};
	uint16_t len = sizeof(value);
	value[0] = acc_range;
	value[1] = (((uint16_t)motion_status[0] >> 8) & 0xff);
	value[2] = ((uint16_t)motion_status[0] & 0xff);

	value[3] = (((uint16_t)motion_status[1] >> 8) & 0xff);
	value[4] = ((uint16_t)motion_status[1] & 0xff);

	value[5] = (((uint16_t)motion_status[2] >> 8) & 0xff);
	value[6] = ((uint16_t)motion_status[2] & 0xff);


	if ((p_mtn->conn_handle != BLE_CONN_HANDLE_INVALID) )
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_mtn->acc_status_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len  = &len;
		hvx_params.p_data = value;

		err_code = sd_ble_gatts_hvx(p_mtn->conn_handle, &hvx_params);
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;
}

uint32_t ble_mtn_gyro_status_update(ble_mtn_t * p_mtn, int16_t*  motion_status, uint8_t gyro_range)
{
	uint32_t err_code = NRF_SUCCESS;

	uint8_t value[7] = {0};
	uint16_t len = sizeof(value);
	value[0] = gyro_range;
	value[1] = (((uint16_t)motion_status[0] >> 8) & 0xff);
	value[2] = ((uint16_t)motion_status[0] & 0xff);

	value[3] = (((uint16_t)motion_status[1] >> 8) & 0xff);
	value[4] = ((uint16_t)motion_status[1] & 0xff);

	value[5] = (((uint16_t)motion_status[2] >> 8) & 0xff);
	value[6] = ((uint16_t)motion_status[2] & 0xff);


	// Send value if connected and notifying
	if ((p_mtn->conn_handle != BLE_CONN_HANDLE_INVALID) )
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_mtn->gyro_status_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len  = &len;
		hvx_params.p_data = value;

		err_code = sd_ble_gatts_hvx(p_mtn->conn_handle, &hvx_params);
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}

	return err_code;
}
