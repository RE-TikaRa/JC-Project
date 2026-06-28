#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "protocol.h"

/*
 * 与 Jetson 的 UART 链路（115200 8N1，ASCII 十六进制行协议）。
 * rx 任务做行分帧 + 解码：PING 直接回 PONG（保活最高优先级，不进队列）；
 * TARGET/HOME/STOP/RESET 投递命令队列给 arm_task。
 * 多任务共用 TX，comm_jetson_send 内部加锁。
 */

/* 配 UART、装 driver、建命令队列与 TX 互斥锁。 */
esp_err_t comm_jetson_init(void);

/* 启动收发任务。 */
esp_err_t comm_jetson_start(void);

/* arm_task 取命令的队列（元素为 frame_t）。 */
QueueHandle_t comm_jetson_cmd_queue(void);

/* 互斥保护的整帧回报。 */
esp_err_t comm_jetson_send(const frame_t *frame);

/* 便捷回报：XYZ 填 0，按 cmd/id/err 组帧发送。 */
esp_err_t comm_jetson_send_status(uint8_t cmd, uint16_t id, uint8_t err);
