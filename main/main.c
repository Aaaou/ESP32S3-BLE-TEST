#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "ble_file_transfer.h"

static const char* TAG = "MAIN";

// 文本数据接收回调函数
static void text_received(const uint8_t* text, size_t len)
{
    ESP_LOGI(TAG, "Received text data (%d bytes):", len);
    // 将接收到的数据作为字符串打印（如果是文本的话）
    if (len > 0) {
        char* text_str = malloc(len + 1);
        if (text_str) {
            memcpy(text_str, text, len);
            text_str[len] = '\0';
            ESP_LOGI(TAG, "Text content: %s", text_str);
            free(text_str);
        }
    }
}

// 图片数据接收回调函数
static void image_received(const uint8_t* image_data, size_t len)
{
    ESP_LOGI(TAG, "Received image data (%d bytes)", len);
    // 这里可以添加图片处理代码
    // 例如：保存到文件系统、显示到屏幕等
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing BLE File Transfer Service...");
    
    // 初始化BLE服务
    ble_file_transfer_init();
    
    // 注册数据接收回调函数
    ble_register_text_callback(text_received);
    ble_register_image_callback(image_received);
    
    ESP_LOGI(TAG, "BLE File Transfer Service is ready");
    ESP_LOGI(TAG, "Waiting for client connection...");
    
    // 主循环
    while (1) {
        // 检查连接状态
        if (is_device_connected()) {
            ESP_LOGI(TAG, "Device is connected");
        } else {
            ESP_LOGI(TAG, "Waiting for connection...");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));  // 5秒检查一次状态
    }
}
