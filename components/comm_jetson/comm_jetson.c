#include "comm_jetson.h"
#include "app_config.h"
#include "protocol.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "comm";

#define RX_TASK_STACK   4096
#define RX_TASK_PRIO    6          /* 高于 arm_task，保活回 PONG 不被作业阻塞 */
#define RX_BUF_SIZE     512
#define LINE_BUF_SIZE   128
#define CMD_QUEUE_LEN   8

static QueueHandle_t s_cmd_queue;
static SemaphoreHandle_t s_tx_mutex;

esp_err_t comm_jetson_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = CFG_JETSON_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CFG_JETSON_UART_PORT, RX_BUF_SIZE, RX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CFG_JETSON_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CFG_JETSON_UART_PORT, CFG_JETSON_UART_TX_GPIO, CFG_JETSON_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    s_cmd_queue = xQueueCreate(CMD_QUEUE_LEN, sizeof(frame_t));
    s_tx_mutex = xSemaphoreCreateMutex();
    if (!s_cmd_queue || !s_tx_mutex) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

QueueHandle_t comm_jetson_cmd_queue(void)
{
    return s_cmd_queue;
}

esp_err_t comm_jetson_send(const frame_t *frame)
{
    char buf[PROTOCOL_ENCODE_BUFLEN];
    size_t n = protocol_encode(frame, buf);

    xSemaphoreTake(s_tx_mutex, portMAX_DELAY);
    int w = uart_write_bytes(CFG_JETSON_UART_PORT, buf, n);
    xSemaphoreGive(s_tx_mutex);
    return (w == (int)n) ? ESP_OK : ESP_FAIL;
}

esp_err_t comm_jetson_send_status(uint8_t cmd, uint16_t id, uint8_t err)
{
    frame_t f = { .cmd = cmd, .id = id, .x = 0, .y = 0, .z = 0, .err = err };
    return comm_jetson_send(&f);
}

/* 处理一行（不含 '\n'）：解码失败丢弃；PING 直接回 PONG；其余投递队列。 */
static void handle_line(const char *line, size_t len)
{
    frame_t f;
    if (!protocol_decode(line, len, &f)) {
        return;  /* 长度/帧头/SUM 不合法，丢弃 */
    }
    if (f.cmd == CMD_PING) {
        comm_jetson_send_status(CMD_PONG, f.id, 0);
        return;
    }
    xQueueSend(s_cmd_queue, &f, 0);  /* 队满则丢弃新命令，不阻塞收帧 */
}

static void comm_rx_task(void *arg)
{
    (void)arg;
    uint8_t rx[128];
    char line[LINE_BUF_SIZE];
    size_t line_len = 0;

    while (1) {
        int n = uart_read_bytes(CFG_JETSON_UART_PORT, rx, sizeof(rx), pdMS_TO_TICKS(50));
        for (int i = 0; i < n; i++) {
            char c = (char)rx[i];
            if (c == '\n') {
                handle_line(line, line_len);
                line_len = 0;
            } else if (c != '\r') {
                if (line_len < LINE_BUF_SIZE - 1) {
                    line[line_len++] = c;
                } else {
                    line_len = 0;  /* 超长行视为噪声，丢弃重新分帧 */
                }
            }
        }
    }
}

esp_err_t comm_jetson_start(void)
{
    if (xTaskCreate(comm_rx_task, "comm_rx", RX_TASK_STACK, NULL, RX_TASK_PRIO, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "rx task started");
    return ESP_OK;
}
