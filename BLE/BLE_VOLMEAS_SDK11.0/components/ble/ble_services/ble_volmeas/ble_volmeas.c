#include "ble_volmeas.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

static uint8_t tmp;


static void on_connect_key(ble_key_t * p_lbs, ble_evt_t * p_ble_evt)
{
    p_lbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the Connect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(ble_volmeass_t * p_lbs, ble_evt_t * p_ble_evt)
{
    p_lbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_connect_charg(ble_charg_t * p_lbs, ble_evt_t * p_ble_evt)
{
    p_lbs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect_charg(ble_charg_t * p_lbs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lbs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_disconnect_key(ble_key_t * p_lbs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lbs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_volmeass_t * p_lbs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_lbs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

void ble_charg_on_ble_evt(ble_charg_t * p_lbs, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect_charg(p_lbs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect_charg(p_lbs, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            //on_write(p_lbs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

void ble_key_on_ble_evt(ble_key_t * p_lbs, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect_key(p_lbs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect_key(p_lbs, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            //on_write(p_lbs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

void ble_volmeass_on_ble_evt(ble_volmeass_t * p_lbs, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_lbs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_lbs, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            //on_write(p_lbs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t charg_char_add(ble_charg_t * p_lbs, const ble_charg_init_t * p_key_init)
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
    
    char_md.char_props.read = 1;
	  char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
	  char_md.p_cccd_md         = NULL;
    char_md.p_cccd_md         = &cccd_md;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = CHARG_FLAG;

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

		attr_char_value.init_len     = 1;
    attr_char_value.init_offs    = 0;
		attr_char_value.max_len      = 1;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_lbs->charg_char_handles);
	
}

static uint32_t key_char_add(ble_key_t * p_lbs, const ble_key_init_t * p_key_init)
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
    
    char_md.char_props.read = 1;
	  char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
	  char_md.p_cccd_md         = NULL;
    char_md.p_cccd_md         = &cccd_md;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = KEY_FLAG;

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

		attr_char_value.init_len     = 1;
    attr_char_value.init_offs    = 0;
		attr_char_value.max_len      = 1;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_lbs->key_char_handles);
	
}
/**********************************************************************************************
 * 描  述 : 添加THI特性(thi是温度、湿度、人体红外检测的缩写)
 * 参  数 : 
 *          
 * 返回值 : 成功返回NRF_SUCCESS，否则返回错误代码
 ***********************************************************************************************/ 
static uint32_t volmeas_char_add(ble_volmeass_t * p_lbs, const ble_volmeass_init_t * p_volmeass_init)
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
    
    char_md.char_props.read = 1;
	  char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
	  char_md.p_cccd_md         = NULL;
    char_md.p_cccd_md         = &cccd_md;

    ble_uuid.type = p_lbs->uuid_type;
    ble_uuid.uuid = VOLMEASS_UUID_VOL_CHAR;

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

		attr_char_value.init_len     = 1;
    attr_char_value.init_offs    = 0;
		attr_char_value.max_len      = 1;
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_lbs->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_lbs->mpu_char_handles);
}

uint32_t ble_charg_init(ble_charg_t * p_volmeass, const ble_charg_init_t * p_volmeass_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_volmeass->conn_handle       = BLE_CONN_HANDLE_INVALID;

    // Add service.
    ble_uuid128_t base_uuid = {VOLMEASUUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_volmeass->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_volmeass->uuid_type;
    ble_uuid.uuid = CHARG_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_volmeass->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = charg_char_add(p_volmeass, p_volmeass_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_key_init(ble_key_t * p_volmeass, const ble_key_init_t * p_volmeass_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_volmeass->conn_handle       = BLE_CONN_HANDLE_INVALID;

    // Add service.
    ble_uuid128_t base_uuid = {VOLMEASUUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_volmeass->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_volmeass->uuid_type;
    ble_uuid.uuid = KEY_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_volmeass->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = key_char_add(p_volmeass, p_volmeass_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint32_t ble_volmeass_init(ble_volmeass_t * p_volmeass, const ble_volmeass_init_t * p_volmeass_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_volmeass->conn_handle       = BLE_CONN_HANDLE_INVALID;

    // Add service.
    ble_uuid128_t base_uuid = {VOLMEASUUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_volmeass->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_volmeass->uuid_type;
    ble_uuid.uuid = VOLMEASS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_volmeass->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = volmeas_char_add(p_volmeass, p_volmeass_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}

uint16_t ble_send_charg_data(ble_charg_t * p_volmeass, uint8_t *pCharg)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = 1;
	  uint32_t err_code;
	  uint8_t  chargValue;//0x01 充电开始 0x02充电中断 0x03充电满
	
	//  keyValue = *pKey;
		chargValue = *pCharg;
	//	tmp++;

	  if (p_volmeass->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			memset(&params, 0, sizeof(params));
      params.type = BLE_GATT_HVX_NOTIFICATION;
      params.handle = p_volmeass->charg_char_handles.value_handle;
      params.p_data = &chargValue;
      params.p_len = &len;
    
      return sd_ble_gatts_hvx(p_volmeass->conn_handle, &params);
		}
		else{
			err_code = NRF_ERROR_INVALID_STATE;
		}
    return err_code;
    
}

uint16_t ble_send_key_data(ble_key_t * p_volmeass, uint8_t *pKey)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = 1;
	  uint32_t err_code;
	  uint8_t  keyValue;
	
	  keyValue = *pKey;

	  if (p_volmeass->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			memset(&params, 0, sizeof(params));
      params.type = BLE_GATT_HVX_NOTIFICATION;
      params.handle = p_volmeass->key_char_handles.value_handle;
      params.p_data = &keyValue;
      params.p_len = &len;
    
      return sd_ble_gatts_hvx(p_volmeass->conn_handle, &params);
		}
		else{
			err_code = NRF_ERROR_INVALID_STATE;
		}
    return err_code;
    
}

/**********************************************************************************************
 * 描  述 : 使用notification将温湿度值、人体红外状态发送给主机
 * 参  数 : p_lbs[in]:指向thi服务结构体
 *        : dht11val[in]:DTH11温湿度传感器检测的温湿度值
 *        : infraredval[in]:人体红外状态
 * 返回值 : 成功返回NRF_SUCCESS，否则返回错误代码
 ***********************************************************************************************/ 
uint16_t ble_send_vol_data(ble_volmeass_t * p_volmeass, int16_t *pVol_ch3)
{
    ble_gatts_hvx_params_t params;
    uint16_t len = 1;
	  uint32_t err_code;
	  uint8_t  volValue;
		int16_t volData;
		
		volData = *pVol_ch3;
	//	volValue = tmp;
//	tmp++;
		if(volData > 386 && volData <= 450)
		{
			volValue = 0x04;
		}
		else if(volData > 377 && volData <= 386)
		{
			volValue = 0x03;
		}
		else if(volData > 371 && volData <= 377)
		{
			volValue = 0x02;
		}
		else if(volData >= 368 && volData <= 371)
		{
			volValue = 0x01;
		}
		else if(volData < 368)
		{
			volValue = 0x00;
		}

	  if (p_volmeass->conn_handle != BLE_CONN_HANDLE_INVALID)
		{
			memset(&params, 0, sizeof(params));
      params.type = BLE_GATT_HVX_NOTIFICATION;
      params.handle = p_volmeass->mpu_char_handles.value_handle;
      params.p_data = &volValue;
      params.p_len = &len;
    
      return sd_ble_gatts_hvx(p_volmeass->conn_handle, &params);
		}
		else{
			err_code = NRF_ERROR_INVALID_STATE;
		}
    return err_code;
    
}

/********************************************END FILE*******************************************/

