#include "motor_driver.h"
#include "app_config.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <string.h>

static const char *TAG = "motor";

#define MB_BUF_MAX        32
#define MB_RX_TIMEOUT_MS  100
#define MB_POLL_GAP_MS    10

/* 标准 Modbus CRC16（初值 0xFFFF，低字节在前） */
static uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/* 收发一帧：发 tx_len 字节，读 expect 字节，校验 CRC。返回 ESP_OK。 */
static esp_err_t modbus_txn(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t expect)
{
    uart_flush_input(CFG_RS485_UART_PORT);
    int w = uart_write_bytes(CFG_RS485_UART_PORT, tx, tx_len);
    if (w != (int)tx_len) {
        return ESP_FAIL;
    }
    int r = uart_read_bytes(CFG_RS485_UART_PORT, rx, expect, pdMS_TO_TICKS(MB_RX_TIMEOUT_MS));
    if (r != (int)expect) {
        ESP_LOGW(TAG, "txn read %d/%u", r, (unsigned)expect);
        return ESP_ERR_TIMEOUT;
    }
    uint16_t crc = modbus_crc16(rx, expect - 2);
    if ((rx[expect - 2] != (crc & 0xFF)) || (rx[expect - 1] != ((crc >> 8) & 0xFF))) {
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
}

esp_err_t motor_driver_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = CFG_RS485_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CFG_RS485_UART_PORT, 256, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CFG_RS485_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CFG_RS485_UART_PORT, CFG_RS485_TX_GPIO, CFG_RS485_RX_GPIO,
                                 CFG_RS485_DE_GPIO, UART_PIN_NO_CHANGE));
    /* RTS 引脚作 DE，UART 硬件自动在发送时拉高、收完拉低 */
    ESP_ERROR_CHECK(uart_set_mode(CFG_RS485_UART_PORT, UART_MODE_RS485_HALF_DUPLEX));
    return ESP_OK;
}

esp_err_t motor_write_reg(uint8_t addr, uint16_t reg, uint16_t value)
{
    uint8_t tx[8];
    tx[0] = addr;
    tx[1] = 0x06;
    tx[2] = (reg >> 8) & 0xFF;
    tx[3] = reg & 0xFF;
    tx[4] = (value >> 8) & 0xFF;
    tx[5] = value & 0xFF;
    uint16_t crc = modbus_crc16(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = (crc >> 8) & 0xFF;

    uint8_t rx[8];
    return modbus_txn(tx, 8, rx, 8);  /* 0x06 回帧回显请求，长度 8 */
}

esp_err_t motor_read_regs(uint8_t addr, uint16_t reg, uint16_t count, uint16_t *out)
{
    if (count == 0 || count > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t tx[8];
    tx[0] = addr;
    tx[1] = 0x03;
    tx[2] = (reg >> 8) & 0xFF;
    tx[3] = reg & 0xFF;
    tx[4] = (count >> 8) & 0xFF;
    tx[5] = count & 0xFF;
    uint16_t crc = modbus_crc16(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = (crc >> 8) & 0xFF;

    size_t expect = 5 + (size_t)count * 2;  /* addr+fn+bytecount + data + crc(2) */
    uint8_t rx[MB_BUF_MAX];
    if (expect > MB_BUF_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = modbus_txn(tx, 8, rx, expect);
    if (err != ESP_OK) {
        return err;
    }
    if (rx[0] != addr || rx[1] != 0x03 || rx[2] != count * 2) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    for (uint16_t i = 0; i < count; i++) {
        out[i] = ((uint16_t)rx[3 + i * 2] << 8) | rx[4 + i * 2];
    }
    return ESP_OK;
}

/* 0x10 写多寄存器（用于绝对位置两寄存器原子写入） */
static esp_err_t modbus_write_multi(uint8_t addr, uint16_t reg, const uint16_t *vals, uint16_t count)
{
    uint8_t tx[MB_BUF_MAX];
    uint8_t bytes = (uint8_t)(count * 2);
    tx[0] = addr;
    tx[1] = 0x10;
    tx[2] = (reg >> 8) & 0xFF;
    tx[3] = reg & 0xFF;
    tx[4] = (count >> 8) & 0xFF;
    tx[5] = count & 0xFF;
    tx[6] = bytes;
    for (uint16_t i = 0; i < count; i++) {
        tx[7 + i * 2] = (vals[i] >> 8) & 0xFF;
        tx[8 + i * 2] = vals[i] & 0xFF;
    }
    size_t n = 7 + bytes;
    uint16_t crc = modbus_crc16(tx, n);
    tx[n] = crc & 0xFF;
    tx[n + 1] = (crc >> 8) & 0xFF;

    uint8_t rx[8];  /* 0x10 回帧：addr+fn+reg(2)+count(2)+crc(2) */
    esp_err_t err = modbus_txn(tx, n + 2, rx, 8);
    if (err != ESP_OK) {
        return err;
    }
    if (rx[0] != addr || rx[1] != 0x10) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t motor_enter_comm_mode(uint8_t addr, bool *need_reboot)
{
    *need_reboot = false;

    /* 驱动器输出使能（非保存类，每次上电须置 1 才出力） */
    esp_err_t err = motor_write_reg(addr, MB_REG_DRIVE_EN, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "addr %u 驱动器输出使能失败: %s", addr, esp_err_to_name(err));
        return err;
    }

    uint16_t gear = 0xFFFF;
    err = motor_read_regs(addr, MB_REG_GEAR_NUM, 1, &gear);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "addr %u 读电子齿轮失败: %s", addr, esp_err_to_name(err));
        return err;
    }
    if (gear == 0) {
        ESP_LOGI(TAG, "addr %u 已在通信模式", addr);
        return ESP_OK;  /* 已配置，跳过写 flash */
    }

    /* Modbus 使能 → 电子齿轮分子=0（进通信位置模式）→ 保存 */
    err = motor_write_reg(addr, MB_REG_MODBUS_EN, 1);
    if (err == ESP_OK) err = motor_write_reg(addr, MB_REG_GEAR_NUM, 0);
    if (err == ESP_OK) err = motor_write_reg(addr, MB_REG_SAVE, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "addr %u 通信模式配置失败: %s", addr, esp_err_to_name(err));
        return err;
    }

    *need_reboot = true;  /* 保存类参数需重新上电生效（手册） */
    ESP_LOGW(TAG, "addr %u 已写入通信模式，需重新上电生效", addr);
    return ESP_OK;
}

esp_err_t motor_set_abs_pulses(uint8_t addr, int32_t pulses)
{
    uint32_t u = (uint32_t)pulses;
    uint16_t regs[2] = {
        (uint16_t)(u & 0xFFFF),         /* 0x16 低 16 位 */
        (uint16_t)((u >> 16) & 0xFFFF), /* 0x17 高 16 位 */
    };
    return modbus_write_multi(addr, MB_REG_ABS_POS_LO, regs, 2);
}

esp_err_t motor_get_abs_pulses(uint8_t addr, int32_t *out)
{
    uint16_t regs[2] = {0, 0};
    esp_err_t err = motor_read_regs(addr, MB_REG_ABS_POS_LO, 2, regs);
    if (err != ESP_OK) {
        return err;
    }
    uint32_t u = ((uint32_t)regs[1] << 16) | regs[0];
    *out = (int32_t)u;
    return ESP_OK;
}

esp_err_t motor_get_alarm(uint8_t addr, uint16_t *out)
{
    return motor_read_regs(addr, MB_REG_ALARM, 1, out);
}

esp_err_t motor_wait_in_position(uint8_t addr, int32_t target, uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    while (elapsed < timeout_ms) {
        uint16_t alarm = 0;
        if (motor_get_alarm(addr, &alarm) == ESP_OK && alarm != 0) {
            ESP_LOGE(TAG, "addr %u 报警 0x%04X", addr, alarm);
            return ESP_FAIL;
        }
        int32_t cur = 0;
        if (motor_get_abs_pulses(addr, &cur) == ESP_OK) {
            int32_t diff = cur - target;
            if (diff <= 2 && diff >= -2) {
                return ESP_OK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MB_POLL_GAP_MS));
        elapsed += MB_POLL_GAP_MS;
    }
    ESP_LOGE(TAG, "addr %u 到位超时", addr);
    return ESP_ERR_TIMEOUT;
}
