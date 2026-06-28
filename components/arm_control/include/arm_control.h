#pragma once

#include "esp_err.h"

/*
 * 作业状态机：从命令队列取 TARGET/HOME/STOP/RESET，
 * 解逆运动学，驱动 A/B 电机走 approach→insert→hold→retract，按状态回报。
 */

/* 错误码（回 ERROR 帧的 ERR 字段，记入 PLAN_MISMATCHES） */
#define ARM_ERR_OUT_OF_RANGE  0x01   /* 目标超工作空间 */
#define ARM_ERR_MOTOR_COMM    0x02   /* 电机通信失败 */
#define ARM_ERR_MOTOR_ALARM   0x03   /* 电机报警 */
#define ARM_ERR_MOVE_TIMEOUT  0x04   /* 运动到位超时 */
#define ARM_ERR_STOPPED       0x10   /* 收到 STOP 中止 */

/* 配置电机进通信模式、建作业任务。须在 comm_jetson_init 之后调用。 */
esp_err_t arm_control_start(void);
