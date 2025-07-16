#ifndef BLE_FILE_TRANSFER_H
#define BLE_FILE_TRANSFER_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#define GATTS_TAG "BLE_TRANSFER"

#define TEST_DEVICE_NAME            "ESP32S3_BLE_TRANSFER"
#define GATTS_SERVICE_UUID         0x00FF
#define GATTS_TEXT_CHAR_UUID       0xFF01
#define GATTS_IMAGE_CHAR_UUID      0xFF02
#define GATTS_NUM_HANDLE           6

#define TEXT_MAX_LEN               1024
#define IMAGE_MAX_LEN              20480  // 20KB for image data

// Event bit definitions
#define BLE_CONNECTED_BIT          BIT0
#define BLE_DISCONNECTED_BIT       BIT1
#define TEXT_RECEIVED_BIT          BIT2
#define IMAGE_RECEIVED_BIT         BIT3

// 数据接收回调函数类型定义
typedef void (*text_received_cb_t)(const uint8_t* text, size_t len);
typedef void (*image_received_cb_t)(const uint8_t* image_data, size_t len);

// Function declarations
void ble_file_transfer_init(void);
void ble_register_text_callback(text_received_cb_t callback);
void ble_register_image_callback(image_received_cb_t callback);
bool is_device_connected(void);

// 获取最后接收的数据
const uint8_t* get_last_text_data(size_t* len);
const uint8_t* get_last_image_data(size_t* len);

#endif /* BLE_FILE_TRANSFER_H */ 