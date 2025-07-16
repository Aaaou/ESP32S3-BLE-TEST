#include "ble_file_transfer.h"

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

// Service UUID
static uint8_t service_uuid[16] = {
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,    // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010,    // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t text_char_handle;
    uint16_t image_char_handle;
    esp_bt_uuid_t text_char_uuid;
    esp_bt_uuid_t image_char_uuid;
    bool is_connected;
};

static struct gatts_profile_inst gl_profile = {0};
static EventGroupHandle_t ble_event_group;

// 数据缓冲区
static uint8_t text_buffer[TEXT_MAX_LEN];
static size_t text_len = 0;
static uint8_t image_buffer[IMAGE_MAX_LEN];
static size_t image_len = 0;

// 回调函数指针
static text_received_cb_t text_callback = NULL;
static image_received_cb_t image_callback = NULL;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0) {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed");
        }
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        // 收到配对请求
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        // 配对完成
        if (param->ble_security.auth_cmpl.success) {
            ESP_LOGI(GATTS_TAG, "Pairing successful with device: %02x:%02x:%02x:%02x:%02x:%02x",
                param->ble_security.auth_cmpl.bd_addr[0],
                param->ble_security.auth_cmpl.bd_addr[1],
                param->ble_security.auth_cmpl.bd_addr[2],
                param->ble_security.auth_cmpl.bd_addr[3],
                param->ble_security.auth_cmpl.bd_addr[4],
                param->ble_security.auth_cmpl.bd_addr[5]);
        } else {
            ESP_LOGE(GATTS_TAG, "Pairing failed, reason = 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    case ESP_GAP_BLE_KEY_EVT:
        // 密钥交换事件
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_KEY_EVT, key type = %d", param->ble_security.ble_key.key_type);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:
        // 显示配对密钥
        ESP_LOGI(GATTS_TAG, "ESP_GAP_BLE_PASSKEY_NOTIF_EVT, passkey = %06" PRIu32, param->ble_security.key_notif.passkey);
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile.service_id.is_primary = true;
        gl_profile.service_id.id.inst_id = 0x00;
        gl_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile.service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret) {
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }

        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret) {
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret) {
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

        esp_ble_gatts_create_service(gatts_if, &gl_profile.service_id, GATTS_NUM_HANDLE);
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d, service_handle %d", param->create.status, param->create.service_handle);
        gl_profile.service_handle = param->create.service_handle;

        // Add Text Characteristic
        gl_profile.text_char_uuid.len = ESP_UUID_LEN_16;
        gl_profile.text_char_uuid.uuid.uuid16 = GATTS_TEXT_CHAR_UUID;
        esp_ble_gatts_start_service(gl_profile.service_handle);

        esp_ble_gatts_add_char(gl_profile.service_handle, &gl_profile.text_char_uuid,
                              ESP_GATT_PERM_WRITE_ENCRYPTED | ESP_GATT_PERM_READ_ENCRYPTED,
                              ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ,
                              NULL, NULL);

        // Add Image Characteristic
        gl_profile.image_char_uuid.len = ESP_UUID_LEN_16;
        gl_profile.image_char_uuid.uuid.uuid16 = GATTS_IMAGE_CHAR_UUID;
        esp_ble_gatts_add_char(gl_profile.service_handle, &gl_profile.image_char_uuid,
                              ESP_GATT_PERM_WRITE_ENCRYPTED | ESP_GATT_PERM_READ_ENCRYPTED,
                              ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ,
                              NULL, NULL);
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d, attr_handle %d, service_handle %d",
                param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        if (param->add_char.char_uuid.uuid.uuid16 == GATTS_TEXT_CHAR_UUID) {
            gl_profile.text_char_handle = param->add_char.attr_handle;
            ESP_LOGI(GATTS_TAG, "Text characteristic added, handle = %d", gl_profile.text_char_handle);
        } else if (param->add_char.char_uuid.uuid.uuid16 == GATTS_IMAGE_CHAR_UUID) {
            gl_profile.image_char_handle = param->add_char.attr_handle;
            ESP_LOGI(GATTS_TAG, "Image characteristic added, handle = %d", gl_profile.image_char_handle);
        }
        break;
    case ESP_GATTS_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "WRITE_EVT, handle = %d, value len = %d", param->write.handle, param->write.len);
        ESP_LOG_BUFFER_HEX(GATTS_TAG, param->write.value, param->write.len);

        if (param->write.handle == gl_profile.text_char_handle) {
            if (param->write.len <= TEXT_MAX_LEN) {
                memcpy(text_buffer, param->write.value, param->write.len);
                text_len = param->write.len;
                xEventGroupSetBits(ble_event_group, TEXT_RECEIVED_BIT);
                
                // 调用回调函数
                if (text_callback) {
                    text_callback(text_buffer, text_len);
                }
                
                // Send response
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            } else {
                ESP_LOGW(GATTS_TAG, "Text data too long (%d > %d)", param->write.len, TEXT_MAX_LEN);
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_INVALID_ATTR_LEN, NULL);
            }
        } else if (param->write.handle == gl_profile.image_char_handle) {
            if (param->write.len <= IMAGE_MAX_LEN) {
                memcpy(image_buffer, param->write.value, param->write.len);
                image_len = param->write.len;
                xEventGroupSetBits(ble_event_group, IMAGE_RECEIVED_BIT);
                
                // 调用回调函数
                if (image_callback) {
                    image_callback(image_buffer, image_len);
                }
                
                // Send response
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            } else {
                ESP_LOGW(GATTS_TAG, "Image data too long (%d > %d)", param->write.len, IMAGE_MAX_LEN);
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_INVALID_ATTR_LEN, NULL);
            }
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        ESP_LOGI(GATTS_TAG, "CONNECT from: %02x:%02x:%02x:%02x:%02x:%02x",
                param->connect.remote_bda[0], param->connect.remote_bda[1],
                param->connect.remote_bda[2], param->connect.remote_bda[3],
                param->connect.remote_bda[4], param->connect.remote_bda[5]);

        gl_profile.gatts_if = gatts_if;
        gl_profile.conn_id = param->connect.conn_id;
        gl_profile.is_connected = true;
        xEventGroupSetBits(ble_event_group, BLE_CONNECTED_BIT);
        
        // 启动加密 - 使用与HID示例相同的加密级别
        esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        gl_profile.conn_id = 0;
        gl_profile.is_connected = false;
        xEventGroupSetBits(ble_event_group, BLE_DISCONNECTED_BIT);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile.gatts_if = gatts_if;
        }
    }

    if (gatts_if == gl_profile.gatts_if) {
        gatts_profile_event_handler(event, gatts_if, param);
    }
}

void ble_file_transfer_init(void)
{
    esp_err_t ret;

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed", __func__);
        return;
    }

    ble_event_group = xEventGroupCreate();

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);

    // 设置安全参数 - 使用与HID示例相同的配置
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     // 只需要绑定，不需要MITM保护
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;          // 设置为无输入输出能力
    uint8_t key_size = 16;                              // 密钥大小保持16字节
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    esp_ble_gatts_app_register(0);
}

void ble_register_text_callback(text_received_cb_t callback)
{
    text_callback = callback;
}

void ble_register_image_callback(image_received_cb_t callback)
{
    image_callback = callback;
}

bool is_device_connected(void)
{
    return gl_profile.is_connected;
}

const uint8_t* get_last_text_data(size_t* len)
{
    if (len) {
        *len = text_len;
    }
    return text_buffer;
}

const uint8_t* get_last_image_data(size_t* len)
{
    if (len) {
        *len = image_len;
    }
    return image_buffer;
} 