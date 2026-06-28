#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 帧格式：AA55 CMD ID X Y Z ERR SUM\n（去行尾后固定 26 ASCII 字符） */
#define PROTOCOL_FRAME_LEN      26          /* 不含 '\n' 的字符数 */
#define PROTOCOL_ENCODE_BUFLEN  28          /* 26 + '\n' + '\0' */

/* Jetson → ESP32 命令码 */
#define CMD_TARGET  0x01
#define CMD_HOME    0x02
#define CMD_STOP    0x03
#define CMD_RESET   0x04
#define CMD_PING    0x05

/* ESP32 → Jetson 状态码 */
#define CMD_ACCEPTED 0x81
#define CMD_BUSY     0x82
#define CMD_DONE     0x83
#define CMD_ERROR    0x84
#define CMD_READY    0x85
#define CMD_PONG     0x86

typedef struct {
    uint8_t  cmd;
    uint16_t id;
    int16_t  x;
    int16_t  y;
    int16_t  z;
    uint8_t  err;
} frame_t;

/* CMD..ERR 各字节二进制累加取低 8 位（与 Jetson serial_bridge checksum 一致） */
uint8_t protocol_checksum(uint8_t cmd, uint16_t id, int16_t x, int16_t y, int16_t z, uint8_t err);

/* 编码为 26 字符 + '\n' + '\0'，写入 out（至少 PROTOCOL_ENCODE_BUFLEN 字节）。返回写入字符数（含 '\n'，不含 '\0'）。 */
size_t protocol_encode(const frame_t *frame, char *out);

/* 解码一行（不含 '\n'）。校验长度、帧头 AA55、SUM。成功返回 true 并填充 frame。 */
bool protocol_decode(const char *line, size_t len, frame_t *frame);
