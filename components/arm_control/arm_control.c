#include "arm_control.h"
#include "app_config.h"
#include "protocol.h"
#include "kinematics.h"
#include "motor_driver.h"
#include "comm_jetson.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "arm";

#define ARM_TASK_STACK  4096
#define ARM_TASK_PRIO   4          /* 低于 comm_rx，保活优先 */

/* 把电机错误映射为回报错误码 */
static uint8_t motor_err_to_code(esp_err_t err)
{
    switch (err) {
        case ESP_ERR_TIMEOUT: return ARM_ERR_MOVE_TIMEOUT;
        case ESP_FAIL:        return ARM_ERR_MOTOR_ALARM;
        default:              return ARM_ERR_MOTOR_COMM;
    }
}

/*
 * 排空命令队列：BUSY 期间只响应 STOP（中止），新 TARGET 按规范不打断而丢弃，
 * 其余命令忽略。返回 true 表示收到 STOP。
 */
static bool drain_check_stop(void)
{
    frame_t f;
    bool stop = false;
    while (xQueueReceive(comm_jetson_cmd_queue(), &f, 0) == pdTRUE) {
        if (f.cmd == CMD_STOP) {
            stop = true;
        }
    }
    return stop;
}

/* 锁定当前位置急停：把两台电机目标设为各自当前位置 */
static void freeze_motors(void)
{
    int32_t a = 0, b = 0;
    if (motor_get_abs_pulses(CFG_MOTOR_ADDR_A, &a) == ESP_OK) {
        motor_set_abs_pulses(CFG_MOTOR_ADDR_A, a);
    }
    if (motor_get_abs_pulses(CFG_MOTOR_ADDR_B, &b) == ESP_OK) {
        motor_set_abs_pulses(CFG_MOTOR_ADDR_B, b);
    }
}

/*
 * 一段运动：设速度 → 写 A/B 绝对位置 → 等两台到位。
 * STOP 在段边界检查（到位后），不打断段内运动。
 * 成功返回 ESP_OK；否则返回电机错误。
 */
static esp_err_t seg_move(int32_t a_pulses, int32_t b_pulses, uint16_t speed_rpm)
{
    esp_err_t err;
    err = motor_write_reg(CFG_MOTOR_ADDR_A, MB_REG_TARGET_SPEED, speed_rpm);
    if (err == ESP_OK) err = motor_write_reg(CFG_MOTOR_ADDR_B, MB_REG_TARGET_SPEED, speed_rpm);
    if (err == ESP_OK) err = motor_set_abs_pulses(CFG_MOTOR_ADDR_A, a_pulses);
    if (err == ESP_OK) err = motor_set_abs_pulses(CFG_MOTOR_ADDR_B, b_pulses);
    if (err != ESP_OK) {
        return err;
    }
    err = motor_wait_in_position(CFG_MOTOR_ADDR_A, a_pulses, CFG_MOVE_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }
    return motor_wait_in_position(CFG_MOTOR_ADDR_B, b_pulses, CFG_MOVE_TIMEOUT_MS);
}

static void process_target(const frame_t *cmd)
{
    comm_jetson_send_status(CMD_ACCEPTED, cmd->id, 0);

    double a_deg, b_mm;
    if (kin_solve(cmd->x, cmd->y, cmd->z, &a_deg, &b_mm) != KIN_OK) {
        comm_jetson_send_status(CMD_ERROR, cmd->id, ARM_ERR_OUT_OF_RANGE);
        return;
    }

    int32_t a_pulses   = kin_a_deg_to_pulses(a_deg);
    int32_t b_approach = kin_b_mm_to_pulses(b_mm - CFG_APPROACH_HEIGHT_MM);
    int32_t b_insert   = kin_b_mm_to_pulses(b_mm + CFG_INSERT_DEPTH_MM);
    int32_t b_retract  = kin_b_mm_to_pulses(b_mm - CFG_RETRACT_HEIGHT_MM);

    comm_jetson_send_status(CMD_BUSY, cmd->id, 0);

    struct {
        int32_t  b;
        uint16_t speed;
    } steps[] = {
        { b_approach, CFG_MOVE_SPEED_RPM },
        { b_insert,   CFG_INSERT_SPEED_RPM },
    };

    for (size_t i = 0; i < sizeof(steps) / sizeof(steps[0]); i++) {
        esp_err_t err = seg_move(a_pulses, steps[i].b, steps[i].speed);
        if (err != ESP_OK) {
            freeze_motors();
            comm_jetson_send_status(CMD_ERROR, cmd->id, motor_err_to_code(err));
            return;
        }
        if (drain_check_stop()) {
            freeze_motors();
            comm_jetson_send_status(CMD_ERROR, cmd->id, ARM_ERR_STOPPED);
            return;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(CFG_HOLD_MS));
    if (drain_check_stop()) {
        freeze_motors();
        comm_jetson_send_status(CMD_ERROR, cmd->id, ARM_ERR_STOPPED);
        return;
    }

    esp_err_t err = seg_move(a_pulses, b_retract, CFG_MOVE_SPEED_RPM);
    if (err != ESP_OK) {
        freeze_motors();
        comm_jetson_send_status(CMD_ERROR, cmd->id, motor_err_to_code(err));
        return;
    }

    comm_jetson_send_status(CMD_DONE, cmd->id, 0);
}

static void process_home(const frame_t *cmd)
{
    comm_jetson_send_status(CMD_ACCEPTED, cmd->id, 0);
    esp_err_t err = seg_move(0, 0, CFG_MOVE_SPEED_RPM);
    if (err != ESP_OK) {
        comm_jetson_send_status(CMD_ERROR, cmd->id, motor_err_to_code(err));
        return;
    }
    comm_jetson_send_status(CMD_DONE, cmd->id, 0);
}

static void arm_task(void *arg)
{
    (void)arg;
    frame_t cmd;
    while (1) {
        if (xQueueReceive(comm_jetson_cmd_queue(), &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        switch (cmd.cmd) {
            case CMD_TARGET:
                process_target(&cmd);
                break;
            case CMD_HOME:
                process_home(&cmd);
                break;
            case CMD_STOP:
                freeze_motors();  /* 空闲态收到 STOP：锁定当前位置 */
                break;
            case CMD_RESET:
                /* 无持久错误状态，清错即就绪，回 READY 闭环 */
                comm_jetson_send_status(CMD_READY, cmd.id, 0);
                break;
            default:
                break;
        }
    }
}

esp_err_t arm_control_start(void)
{
    bool reboot_a = false, reboot_b = false;
    esp_err_t ea = motor_enter_comm_mode(CFG_MOTOR_ADDR_A, &reboot_a);
    esp_err_t eb = motor_enter_comm_mode(CFG_MOTOR_ADDR_B, &reboot_b);
    if (ea != ESP_OK || eb != ESP_OK) {
        ESP_LOGE(TAG, "电机通信模式配置失败 A=%s B=%s",
                 esp_err_to_name(ea), esp_err_to_name(eb));
        /* 不阻断启动：保活仍需运行，作业时再以 ERROR 回报 */
    }
    if (reboot_a || reboot_b) {
        ESP_LOGW(TAG, "电机参数已写入，需重新上电后通信位置模式才生效");
    }

    if (xTaskCreate(arm_task, "arm", ARM_TASK_STACK, NULL, ARM_TASK_PRIO, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "arm task started");
    return ESP_OK;
}
