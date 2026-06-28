#include "kinematics.h"
#include "app_config.h"

#include <math.h>

#define DEG_PER_RAD  (180.0 / M_PI)

bool kin_workspace_check(int16_t x_mm, int16_t y_mm, int16_t z_mm)
{
    return x_mm >= CFG_WS_X_MIN_MM && x_mm <= CFG_WS_X_MAX_MM &&
           y_mm >= CFG_WS_Y_MIN_MM && y_mm <= CFG_WS_Y_MAX_MM &&
           z_mm >= CFG_WS_Z_MIN_MM && z_mm <= CFG_WS_Z_MAX_MM;
}

kin_result_t kin_solve(int16_t x_mm, int16_t y_mm, int16_t z_mm,
                       double *joint_a_deg, double *joint_b_mm)
{
    if (!kin_workspace_check(x_mm, y_mm, z_mm)) {
        return KIN_ERR_OUT_OF_RANGE;
    }

    /*
     * 占位模型：
     * A 轴角度对准目标在水平面的方位角（x 前方、y 左方）。
     * B 轴线位移取目标深度 z（向下为正），由 arm_control 叠加 approach/insert/retract。
     * TODO: 接入真实机构几何（臂长、零位、连杆），现场标定后替换。
     */
    *joint_a_deg = atan2((double)y_mm, (double)x_mm) * DEG_PER_RAD;
    *joint_b_mm = (double)z_mm;
    return KIN_OK;
}

int32_t kin_a_deg_to_pulses(double deg)
{
    /* 关节角减零位 → 圈数 × 减速比 × 每圈脉冲 */
    double rev = ((deg - CFG_ARM_A_ZERO_DEG) / 360.0) * CFG_ARM_A_GEAR_RATIO;
    return (int32_t)lround(rev * CFG_ENCODER_PULSES_PER_REV);
}

int32_t kin_b_mm_to_pulses(double mm)
{
    /* 线位移减零位 → 圈数（丝杆螺距）× 减速比 × 每圈脉冲 */
    double rev = ((mm - CFG_ARM_B_ZERO_MM) / CFG_ARM_B_SCREW_LEAD_MM) * CFG_ARM_B_GEAR_RATIO;
    return (int32_t)lround(rev * CFG_ENCODER_PULSES_PER_REV);
}
