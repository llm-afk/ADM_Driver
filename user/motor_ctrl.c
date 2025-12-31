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
#pragma CODE_SECTION(MC_servo_loop, "ramfuncs");
void MC_servo_loop(void)
{
    // 更新减速端位置和速度
    // encoder.degree_q14 = (encoder.enc_turns / GEAR_RATIO * 2pi * 2^14) + ((encoder.enc_degree_lined - 上电时刻主编码器的角度值) / ENCODER_CPR / GEAR_RATIO * 2pi * 2^14) + (encoder.error * 0.75f * 2pi * 16384 / 8192) ;
    encoder.degree_q14 = (encoder.enc_turns * 8580) + (((int32_t)((int32_t)encoder.enc_degree_lined - (int32_t)encoder.in_enc_deg_zero) * 8580) >> 14) + (int32_t)((float)encoder.error * 9.245f);
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

            degree_err_q14   = CLAMP(degree_err_q14,  -102944,  102944); // 解决位置目标值和实际值过大导致计算中间过程溢出导致反向转动的问题
            velocity_err_q14 = CLAMP(velocity_err_q14,-1638400, 1638400);

            int32_t out_q14 = 0;


            out_q14 = (int32_t)(((int64_t)motor_ctrl.Kp_q14 * degree_err_q14) >> 14) \
                    + (int32_t)(((int64_t)motor_ctrl.Kd_q14 * velocity_err_q14) >> 14) \
                    + (int32_t)((int64_t)motor_ctrl.current_ref_q14 * 40960 / MOTOR_RATED_CUR);


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
            static int32_t Iq_init = 0;
            static int32_t Id_init = 0;
            static uint16_t ramp_count = 0;
            static uint16_t  ramp_flag = 0;
            
            #define RAMP_STEPS 4000 
            
            if (ramp_flag == 0)
            {
                ramp_flag = 1;
                ramp_count = 0;
                Iq_init = Iq;
                Id_init = Id;
            }
            if (ramp_count < RAMP_STEPS)
            {
                ramp_count++;
                
                // I = I_init × (STEPS - count) / STEPS
                Iq = (int16_t)((Iq_init * (int32_t)(RAMP_STEPS - ramp_count)) / RAMP_STEPS);
                Id = (int16_t)((Id_init * (int32_t)(RAMP_STEPS - ramp_count)) / RAMP_STEPS);
            }
            else
            {
                Iq = 0;
                Id = 0;
                ramp_flag = 0;
                RunSignal = 0; // 清除电机使能位
                motor_ctrl.state = STOPPED;
            }
            
            break;
        }
        case STOPPED:
        {
            if(RunSignal == 1) // 保留一个外部手动起的功能
            {
                motor_ctrl.state = INIT;
            }
            break;
        }
        default :
        {
            break;
        }
    }
}

/**
 * @brief 置位错误
 * @param err 错误类型
 */
void set_err(tErrorCode err)
{
    if(ODObjs.error_code & err) // 当前错误已存在
    {
        return; 
    }
    else
    {
        ODObjs.error_code |= err; // 置位该错误
    }
}

/**
 * @brief 清除错误
 * @param err 错误类型
 */
void clr_err(tErrorCode err)
{
    if(ODObjs.error_code & err) // 当前错误已存在
    {
        ODObjs.error_code &= ~err; // 清除该错误
    }
}

float board_temp;
float motor_temp;

#define ADC_MAX 4095.0f
#define R_PULLUP 4700.0f   // 4.7k
#define R25   10000.0f
#define BETA  3445.0f

/**
 * @brief 采集电机控制相关数据
 * @note 放在stimer框架下100hz循环执行
 */
void info_collect_loop(void)
{
    // static uint16_t cnt = 0;
    // if(cnt%2 == 0)
    // {
    //     Iq = 0;
    //     Id = 1000;
    // }
    // else
    // {
    //     Iq = 0;
    //     Id = 0;
    // }
    // cnt++;

    static float r_ntc;
    static float board_temp_filt = 0.0f;
    static float motor_temp_filt = 0.0f;
    const float alpha = 0.01f; // 滤波系数，越小越平滑，0~1

    // 采集驱动板温度
    r_ntc = 4700.0f * ADC_NTC_M / (4095.0f - ADC_NTC_M);
    float board_temp_raw = 1.0f / (1.0f / 298.15f + logf(r_ntc / 10000.0f) / 3445.0f) - 273.15f;
    // 一阶低通滤波
    board_temp_filt = board_temp_filt + alpha * (board_temp_raw - board_temp_filt);
    board_temp = board_temp_filt;

    // 采集电机温度
    r_ntc = 4700.0f * ADC_NTC / (4095.0f - ADC_NTC);
    float motor_temp_raw = 1.0f / (1.0f / 298.15f + logf(r_ntc / 10000.0f) / 3445.0f) - 273.15f;
    // 一阶低通滤波
    motor_temp_filt = motor_temp_filt + alpha * (motor_temp_raw - motor_temp_filt);
    motor_temp = motor_temp_filt;

    // 考虑到实际的在狗上的场景就是他会先触发温度保护然后断掉电机，所以这里只做一个简单的超温错误上报和软关闭的流程的兜底流程，不涉及温度降低自动开启的功能
    if (!(ODObjs.error_code & ERR_OVER_TEMP_MOTOR))
    {
        if (motor_temp > MOTOR_TEMP_MAX)
        {
            set_err(ERR_OVER_TEMP_MOTOR);
            motor_ctrl.state = SOFT_STOP;
        }
    }
    else
    {
        if (motor_temp < MOTOR_TEMP_RECOVER)
        {
            clr_err(ERR_OVER_TEMP_MOTOR);
        }
    }
    if (!(ODObjs.error_code & ERR_OVER_TEMP_DRV)) 
    {
        if (board_temp > BOARD_TEMP_MAX)
        {
            set_err(ERR_OVER_TEMP_DRV);
            motor_ctrl.state = SOFT_STOP;
        }
    }
    else
    {
        if (board_temp < BOARD_TEMP_RECOVER)
        {
            clr_err(ERR_OVER_TEMP_DRV);
        }
    }

}
