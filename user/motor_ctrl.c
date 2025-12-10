#include "motor_ctrl.h"
#include "utils.h"

motor_ctrl_t motor_ctrl = {
    .state = INIT,    
};

extern long Iq;
extern long Id;
extern int RunSignal;

/**
 * @brief 电机状态控制主循环
 * @note 放在stimer框架下2Khz循环执行
 */
#pragma CODE_SECTION(servo_loop, "ramfuncs");
void servo_loop(void)
{
    switch(ODObjs.control_word & 0x00FF)
    {
        case CW_CMD_OPERATION_ENABLE: // 使能电机
        {
            RunSignal = 1;
            break;
        }
        case CW_CMD_OPERATION_DISABLE: // 失能电机
        {
            RunSignal = 0;
            break;
        }
        case CW_CMD_RESET_HOME: // 复位原点
        {

            break;
        }
        case CW_CMD_ERROR_RESET: // 错误清除
        {
            ODObjs.error_code = 0;
            break;
        }
        case CW_CMD_DEV_ENCODER_CALIB: // 编码器校准
        {
            motor_ctrl.state = ENCODER_CALIBRATE;
            break;
        }
        default:
        {
            break;
        }
    }

    switch(motor_ctrl.state)
    {
        case INIT:
        {
            motor_ctrl.state = MIT;
            break;
        }
        case MIT:
        {
            encoder.position_q12_gear = (int32_t)((float)encoder.motor_turns * GEAR_RATIO_INV * TWO_PI * 4096.0f) + (int32_t)((float)encoder.motor_position * GEAR_RATIO_INV * TWO_PI * 4096.0f / 65536.0f + 0.5f);

            // encoder.volacity_q12_gear = (int32_t)(((float64)encoder.volacity_q12 / 4096.0) * TWO_PI / 65536.0 / GEAR_RATIO * 20000.0 * 4096.0 + 0.5);
            encoder.volacity_q12_gear = (int32_t)((float32)encoder.volacity_q12 * 1.917476038208008f * GEAR_RATIO_INV + 0.5f); 

            int32_t pos_err_q12 = motor_ctrl.position_ref_q12 - encoder.position_q12_gear; 
            int32_t vol_err_q12 = motor_ctrl.velocity_ref_q12 - encoder.volacity_q12_gear;

            pos_err_q12 = CLAMP(pos_err_q12, -514450,514450);
            int32_t torque_q12 = (((int64_t)motor_ctrl.Kp_q12 * pos_err_q12) >> 12) + (((int64_t)motor_ctrl.Kd_q12 * vol_err_q12) >> 12) + (motor_ctrl.current_ref_q12 * 40960 / MOTOR_RATED_CUR);
            torque_q12 = CLAMP(torque_q12, -33554432, 33554432);
            int16_t torque = (int16_t)((torque_q12 + 2048) >> 12); 
            torque = CLAMP(torque, -8192, 8192);

            Iq = torque; 
            Id = 0;
            break;
        }
        case ENCODER_CALIBRATE: 
        {
            

            break;
        }
        case ENCODER_ZERO: 
        {

            break;
        }
        case SOFT_STOP: 
        {

            break;
        }
        default :
        {
            break;
        }
    }
}

/**
 * @brief 采集电机控制相关数据
 * @note 放在stimer框架下1Khz循环执行
 */
#pragma CODE_SECTION(info_collect_loop, "ramfuncs");
void info_collect_loop(void)
{
    // 采集驱动板温度
    
}
