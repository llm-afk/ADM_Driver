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

    int32_t position_ref_q12; // 输出端目标角度 rad q12
    int32_t velocity_ref_q12; // 输出端目标速度 rad/s q12
    int32_t current_ref_q12;  // mit前馈电流 A q12
    uint32_t Kp_q12;          // mit_kp 0.01
    uint32_t Kd_q12;          // mit_kd 0.01

    int32_t board_temp_q12;   // 驱动器温度
    int32_t motor_temp_q12;   // 电机温度
}motor_ctrl_t;

extern motor_ctrl_t motor_ctrl;

void servo_loop(void);
void info_collect_loop(void);

#endif
