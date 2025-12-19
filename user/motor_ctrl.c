#include "motor_ctrl.h"
#include "utils.h"

motor_ctrl_t motor_ctrl = {
    .state = INIT,    
};

#pragma CODE_SECTION(MC_controlword_update, "ramfuncs");
int MC_controlword_update(void)
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
    ODObjs.control_word &= 0xFF00;
    return 0;
}

/**
 * @brief 电机状态控制主循环
 * @note 放在stimer框架下2Khz循环执行
 */
#pragma CODE_SECTION(servo_loop, "ramfuncs");
void servo_loop(void)
{
    // 更新减速端位置和速度
    // encoder.degree_q14 = (encoder.enc_turns / GEAR_RATIO * 2pi * 2^14) + (encoder.enc_degree_lined / ENCODER_CPR / GEAR_RATIO * 2pi * 2^14) ;
    encoder.degree_q14 = (encoder.enc_turns * 8580) + (((int32_t)encoder.enc_degree_lined * 8579) >> 14);
    encoder.velocity_q14 = (int32_t)(encoder.enc_velocity_q14 * GEAR_RATIO_INV);
    
    switch(motor_ctrl.state)
    {
        case INIT:
        {
            motor_ctrl.state = MIT;
            break;
        }
        case MIT:
        {
            int64_t degree_err_q14 = motor_ctrl.degree_ref_q14 - encoder.degree_q14;
            int64_t velocity_err_q14 = motor_ctrl.velocity_ref_q14 - encoder.velocity_q14;

            int32_t out_q14 = (int32_t)(((int64_t)motor_ctrl.Kp_q14 * degree_err_q14) >> 14) \
                            + (int32_t)(((int64_t)motor_ctrl.Kd_q14 * velocity_err_q14) >> 14) \
                            + (motor_ctrl.current_ref_q14 * 40960 / MOTOR_RATED_CUR);

            out_q14 = CLAMP(out_q14, -67108864, 67108864); // 4096 * 16384
            
            Iq = out_q14 >> 14; 
            Id = 0;
            break;
        }
        case ENCODER_CALIBRATE: 
        {
            if(encoder_calibrate() == 1)
            {
                motor_ctrl.state = MIT;
            }
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
