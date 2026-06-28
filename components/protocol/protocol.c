#include "protocol.h"

#include <string.h>

static const char HEX_UPPER[] = "0123456789ABCDEF";

uint8_t protocol_checksum(uint8_t cmd, uint16_t id, int16_t x, int16_t y, int16_t z, uint8_t err)
{
    uint32_t sum = cmd;
    sum += (id >> 8) & 0xFF;
    sum += id & 0xFF;
    uint16_t ux = (uint16_t)x, uy = (uint16_t)y, uz = (uint16_t)z;
    sum += (ux >> 8) & 0xFF;
    sum += ux & 0xFF;
    sum += (uy >> 8) & 0xFF;
    sum += uy & 0xFF;
    sum += (uz >> 8) & 0xFF;
    sum += uz & 0xFF;
    sum += err;
    return (uint8_t)(sum & 0xFF);
}

/* 把 value 的低 width 个十六进制位（大写）写入 dst */
static void put_hex(char *dst, uint32_t value, int width)
{
    for (int i = width - 1; i >= 0; i--) {
        dst[i] = HEX_UPPER[value & 0xF];
        value >>= 4;
    }
}

/* 解析 count 个十六进制字符为无符号整数；非法字符返回 false */
static bool parse_hex(const char *src, int count, uint32_t *out)
{
    uint32_t v = 0;
    for (int i = 0; i < count; i++) {
        char c = src[i];
        uint32_t d;
        if (c >= '0' && c <= '9') {
            d = (uint32_t)(c - '0');
        } else if (c >= 'A' && c <= 'F') {
            d = (uint32_t)(c - 'A' + 10);
        } else if (c >= 'a' && c <= 'f') {
            d = (uint32_t)(c - 'a' + 10);
        } else {
            return false;
        }
        v = (v << 4) | d;
    }
    *out = v;
    return true;
}

size_t protocol_encode(const frame_t *frame, char *out)
{
    memcpy(out, "AA55", 4);
    put_hex(out + 4, frame->cmd, 2);
    put_hex(out + 6, frame->id, 4);
    put_hex(out + 10, (uint16_t)frame->x, 4);
    put_hex(out + 14, (uint16_t)frame->y, 4);
    put_hex(out + 18, (uint16_t)frame->z, 4);
    put_hex(out + 22, frame->err, 2);
    uint8_t sum = protocol_checksum(frame->cmd, frame->id, frame->x, frame->y, frame->z, frame->err);
    put_hex(out + 24, sum, 2);
    out[26] = '\n';
    out[27] = '\0';
    return PROTOCOL_FRAME_LEN + 1;
}

bool protocol_decode(const char *line, size_t len, frame_t *frame)
{
    if (len < PROTOCOL_FRAME_LEN || memcmp(line, "AA55", 4) != 0) {
        return false;
    }

    uint32_t cmd, id, x, y, z, err, sum;
    if (!parse_hex(line + 4, 2, &cmd) ||
        !parse_hex(line + 6, 4, &id) ||
        !parse_hex(line + 10, 4, &x) ||
        !parse_hex(line + 14, 4, &y) ||
        !parse_hex(line + 18, 4, &z) ||
        !parse_hex(line + 22, 2, &err) ||
        !parse_hex(line + 24, 2, &sum)) {
        return false;
    }

    frame->cmd = (uint8_t)cmd;
    frame->id = (uint16_t)id;
    frame->x = (int16_t)(uint16_t)x;
    frame->y = (int16_t)(uint16_t)y;
    frame->z = (int16_t)(uint16_t)z;
    frame->err = (uint8_t)err;

    uint8_t expect = protocol_checksum(frame->cmd, frame->id, frame->x, frame->y, frame->z, frame->err);
    return expect == (uint8_t)sum;
}
