#include "app_config.h"
#include "comm_jetson.h"
#include "motor_driver.h"
#include "arm_control.h"
#include "protocol.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "陇参卫士 除草执行端启动");

    ESP_ERROR_CHECK(comm_jetson_init());
    ESP_ERROR_CHECK(motor_driver_init());
    ESP_ERROR_CHECK(comm_jetson_start());
    ESP_ERROR_CHECK(arm_control_start());

    /* 自检完成回 READY */
    comm_jetson_send_status(CMD_READY, 0, 0);
    ESP_LOGI(TAG, "自检完成，已回报 READY");
}
