#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include "MainInclude.h"
#include "encoder.h"
#include "MotorInclude.h"
#include "od.h"

typedef enum{
    INIT,
    MIT,
    ENCODER_CALIBRATE,
    ENCODER_ZERO,
    SOFT_STOP,
}motor_mode_t;

#define CW_CMD_OPERATION_ENABLE             0x01
#define CW_CMD_OPERATION_DISABLE            0x02
#define CW_CMD_RESET_HOME                   0x03
#define CW_CMD_ERROR_RESET                  0xFF
#define CW_CMD_DEV_ENCODER_CALIB            0xF1

typedef struct {
    motor_mode_t state;   // 控制模式状态

    int32_t degree_ref_q14;   // 输出端目标角度 rad
    int32_t velocity_ref_q14; // 输出端目标速度 rad/s
    int32_t current_ref_q14;  // mit前馈电流 A
    uint32_t Kp_q14;          // mit_kp 0.01
    uint32_t Kd_q14;          // mit_kd 0.01

    int32_t board_temp_q14;   // 驱动器温度
    int32_t motor_temp_q14;   // 电机温度
}motor_ctrl_t;

extern motor_ctrl_t motor_ctrl;

void servo_loop(void);
void info_collect_loop(void);

int MC_controlword_update(void);

#endif
