#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * 逆运动学（参数化占位）。
 * A 电机控水平平面旋转，B 电机控垂直平面运动（除草杆插拔）。
 * 几何模型为占位：A = atan2(y, x) 对准目标方位；B 由插入深度决定。
 * 具体臂长、零位、减速比见 app_config，标 TODO 待现场标定。
 */

typedef enum {
    KIN_OK = 0,
    KIN_ERR_OUT_OF_RANGE = 1,   /* 目标超工作空间 */
} kin_result_t;

/* 工作空间检查：对照 app_config 的 CFG_WS_* */
bool kin_workspace_check(int16_t x_mm, int16_t y_mm, int16_t z_mm);

/* 逆解：目标点（mm）→ A 轴角度（度）、B 轴线位移（mm）。返回 KIN_OK 或错误码。 */
kin_result_t kin_solve(int16_t x_mm, int16_t y_mm, int16_t z_mm,
                       double *joint_a_deg, double *joint_b_mm);

/* 关节量 → 电机绝对脉冲（编码器一圈 CFG_ENCODER_PULSES_PER_REV） */
int32_t kin_a_deg_to_pulses(double deg);
int32_t kin_b_mm_to_pulses(double mm);
