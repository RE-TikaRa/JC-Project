#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/*
 * YZ-AIM 一体化伺服 Modbus RTU 驱动（RS485 半双工，19200 8N1）。
 * 两台电机挂同一总线，从站地址区分（CFG_MOTOR_ADDR_A / _B）。
 */

/* 关键寄存器地址（手册第五节寄存器表） */
#define MB_REG_MODBUS_EN     0x00   /* Modbus 使能 0/1 */
#define MB_REG_DRIVE_EN      0x01   /* 驱动器输出使能 0/1 */
#define MB_REG_TARGET_SPEED  0x02   /* 目标/最大速度 r/min */
#define MB_REG_GEAR_NUM      0x0A   /* 电子齿轮分子（=0 进通信模式） */
#define MB_REG_ALARM         0x0E   /* 报警代码（只读） */
#define MB_REG_SAVE          0x14   /* 参数保存标志 0/1/2 */
#define MB_REG_ABS_POS_LO    0x16   /* 绝对位置低 16 位，0x16/0x17 一次写两寄存器 */

/* 初始化 RS485 UART（半双工，RTS 作 DE）。 */
esp_err_t motor_driver_init(void);

/* Modbus 功能码 0x06：写单寄存器。 */
esp_err_t motor_write_reg(uint8_t addr, uint16_t reg, uint16_t value);

/* Modbus 功能码 0x03：读 n 个寄存器到 out。 */
esp_err_t motor_read_regs(uint8_t addr, uint16_t reg, uint16_t count, uint16_t *out);

/*
 * 进入通信位置控制模式：读 0x0A，若已为 0 视为已配置直接返回；
 * 否则使能 + 电子齿轮分子=0 + 保存，并通过 need_reboot 标记需重新上电。
 */
esp_err_t motor_enter_comm_mode(uint8_t addr, bool *need_reboot);

/* 写绝对位置（脉冲），寄存器 0x16/0x17，低字在前。 */
esp_err_t motor_set_abs_pulses(uint8_t addr, int32_t pulses);

/* 读绝对位置（脉冲）。 */
esp_err_t motor_get_abs_pulses(uint8_t addr, int32_t *out);

/* 读报警代码（0 表示正常）。 */
esp_err_t motor_get_alarm(uint8_t addr, uint16_t *out);

/* 轮询等待到位（容差 ±2 脉冲），超时或报警返回错误。 */
esp_err_t motor_wait_in_position(uint8_t addr, int32_t target, uint32_t timeout_ms);
