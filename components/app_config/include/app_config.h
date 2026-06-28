#pragma once

#include <stdint.h>

/*
 * 集中配置：引脚、Modbus 从站地址、动作参数、机构几何常量、保活时间。
 * 几何与标定值为占位，标 TODO，待现场标定后填入。
 */

/* ===== Jetson UART（ASCII 十六进制行协议，115200 8N1） ===== */
/* COM 口（CH343P）桥接 UART0/GPIO43,44；console 已改走 USB-Serial-JTAG */
#define CFG_JETSON_UART_PORT        UART_NUM_0
#define CFG_JETSON_UART_TX_GPIO     43
#define CFG_JETSON_UART_RX_GPIO     44
#define CFG_JETSON_UART_BAUD        115200

/* ===== 电机 RS485（Modbus RTU，19200 8N1，半双工） ===== */
#define CFG_RS485_UART_PORT         UART_NUM_2
#define CFG_RS485_TX_GPIO           4    /* TODO: 按实际接线确认 */
#define CFG_RS485_RX_GPIO           5    /* TODO: 按实际接线确认 */
#define CFG_RS485_DE_GPIO           6    /* RTS 作收发方向控制，TODO 确认 */
#define CFG_RS485_BAUD              19200

/* ===== Modbus 从站地址 ===== */
#define CFG_MOTOR_ADDR_A            1    /* A 电机：水平平面旋转 */
#define CFG_MOTOR_ADDR_B            2    /* B 电机：垂直平面运动（除草杆插拔） */

/* ===== 编码器与电子齿轮 ===== */
#define CFG_ENCODER_PULSES_PER_REV  32768  /* YZ-AIM 单圈脉冲数（手册） */

/* ===== 动作参数（plan 七，保存在本地） ===== */
#define CFG_APPROACH_HEIGHT_MM      50     /* 接近高度 TODO 标定 */
#define CFG_INSERT_DEPTH_MM         80     /* 插入深度 TODO 标定 */
#define CFG_HOLD_MS                 500    /* 保持时长 TODO 标定 */
#define CFG_RETRACT_HEIGHT_MM       60     /* 拔出高度 TODO 标定 */
#define CFG_MOVE_SPEED_RPM          800    /* 平移速度 TODO 标定 */
#define CFG_INSERT_SPEED_RPM        400    /* 插入速度 TODO 标定 */

/* ===== 机构几何（占位，TODO 待现场标定） ===== */
#define CFG_ARM_A_GEAR_RATIO        1.0    /* A 轴减速比 TODO */
#define CFG_ARM_B_SCREW_LEAD_MM     5.0    /* B 轴丝杆螺距 mm/圈 TODO */
#define CFG_ARM_B_GEAR_RATIO        1.0    /* B 轴减速比 TODO */
#define CFG_ARM_A_ZERO_DEG          0.0    /* A 轴关节零位 TODO */
#define CFG_ARM_B_ZERO_MM           0.0    /* B 轴关节零位 TODO */

/* ===== 工作空间（mm，对齐 Jetson arm_workspace.yaml，米→毫米） ===== */
#define CFG_WS_X_MIN_MM             100
#define CFG_WS_X_MAX_MM             450
#define CFG_WS_Y_MIN_MM            (-200)
#define CFG_WS_Y_MAX_MM            200
#define CFG_WS_Z_MIN_MM            0
#define CFG_WS_Z_MAX_MM            250

/* ===== 运动超时 ===== */
#define CFG_MOVE_TIMEOUT_MS         5000   /* 单段运动到位超时 */
