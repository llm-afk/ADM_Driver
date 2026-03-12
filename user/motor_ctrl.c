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
            if(RunSignal == 0)
            {
                RunSignal = 1;
            }
            break;
        }
        case CW_CMD_OPERATION_DISABLE: // 失能电机
        {
            if(RunSignal == 1)
            {
                RunSignal = 0;
            }
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
    encoder.degree_q14 = ((float)encoder.enc_turns + ((float)((int16_t)encoder.enc_degree_lined - (int16_t)encoder.in_enc_deg_zero) / 16384) + ((float)encoder.error * 18 / 16384)) / 12 * TWO_PI * 16384;
    encoder.velocity_q14 = (int32_t)(encoder.enc_velocity_q14 * GEAR_RATIO_INV);

    static int64_t degree_q14_last = 0;
    static int16_t first_flag = 0;
    if(!first_flag) 
    {
        first_flag++;
    }
    else
    {
        if(INT_ABS(encoder.degree_q14 - degree_q14_last) > 1000) // 滤除50rps以上的瞬时错误角度数据
        {
            encoder.degree_q14 = degree_q14_last;
        }
    }
    degree_q14_last = encoder.degree_q14;
    
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

            degree_err_q14   = CLAMP(degree_err_q14,  -163840,  163840); // 解决位置目标值和实际值过大导致计算中间过程溢出导致反向转动的问题
            velocity_err_q14 = CLAMP(velocity_err_q14,-1638400, 1638400);

            int32_t out_q14 = (int32_t)(((int64_t)motor_ctrl.Kp_q14 * degree_err_q14) >> 14) \
                            + (int32_t)(((int64_t)motor_ctrl.Kd_q14 * velocity_err_q14) >> 14) \
                            + (int32_t)((int64_t)motor_ctrl.current_ref_q14 * 40960 / (MOTOR_RATED_CUR * 1.41421356f)); // 直接把电流目标值换算成q14格式的输出值，减少一次乘法计算

            out_q14 = CLAMP(out_q14, -67108864, 67108864); // 4096 * 16384 软件限制最大电流为额定电流
            
            Iq = -out_q14 >> 14; 
            Id = 0;
            // static uint16_t mit_cnt = 0;
            // mit_cnt++;
            // if(mit_cnt == 20) // 100hz更新一次mit输出
            // {
            //     Id = 512;
            // }
            // else if(mit_cnt == 40) // 200hz更新一次mit输出
            // {
            //     mit_cnt = 0;
            //     Id = 0;
            // }
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
            
            if(ramp_flag == 0)
            {
                ramp_flag = 1;
                ramp_count = 0;
                Iq_init = Iq;
                Id_init = Id;
            }
            if(ramp_count < RAMP_STEPS)
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
#define R_PULLUP 10000.0f // 10k
#define R25   10000.0f
#define BETA  3445.0f

/**
 * @brief 采集电机控制相关数据
 * @note 放在stimer框架下100hz循环执行
 */
void info_collect_loop(void)
{
    static float r_ntc;
    static float board_temp_filt;
    static float motor_temp_filt;
    static uint16_t temp_filt_inited = 0;

    const float alpha = 0.01f;

    /* -------- 驱动板温度 -------- */
    r_ntc = R_PULLUP * ADC_NTC / (4095.0f - ADC_NTC);
    float board_temp_raw = 1.0f / (1.0f / 298.15f + logf(r_ntc / 10000.0f) / 3445.0f) - 273.15f;

    /* -------- 电机温度 -------- */
    r_ntc = R_PULLUP * ADC_NTC_M / (4095.0f - ADC_NTC_M);
    float motor_temp_raw = 1.0f / (1.0f / 298.15f + logf(r_ntc / 10000.0f) / 3445.0f) - 273.15f;

    /* -------- 一阶滤波（带初始化） -------- */
    if (!temp_filt_inited)
    {
        board_temp_filt = board_temp_raw;
        motor_temp_filt = motor_temp_raw;
        temp_filt_inited = 1;
    }
    else
    {
        board_temp_filt += alpha * (board_temp_raw - board_temp_filt);
        motor_temp_filt += alpha * (motor_temp_raw - motor_temp_filt);
    }

    board_temp = board_temp_filt;
    motor_temp = motor_temp_filt;

    /* -------- 保护逻辑 -------- */
    if(!(ODObjs.error_code & ERR_OVER_TEMP_MOTOR))
    {
        if(motor_temp > MOTOR_TEMP_MAX)
        {
            set_err(ERR_OVER_TEMP_MOTOR);
            motor_ctrl.state = SOFT_STOP;
        }
    }
    else if(motor_temp < MOTOR_TEMP_RECOVER)
    {
        clr_err(ERR_OVER_TEMP_MOTOR);
    }

    if(!(ODObjs.error_code & ERR_OVER_TEMP_DRV))
    {
        if(board_temp > BOARD_TEMP_MAX)
        {
            set_err(ERR_OVER_TEMP_DRV);
            motor_ctrl.state = SOFT_STOP;
        }
    }
    else if(board_temp < BOARD_TEMP_RECOVER)
    {
        clr_err(ERR_OVER_TEMP_DRV);
    }

    // 判断can_phy连通性
    canfd_timeout_cnt++;
    if(canfd_first_flag == 0)
    {
        if(canfd_timeout_cnt > 1000) // 跳过开始的10s
        {
            canfd_timeout_cnt = 0;
            canfd_first_flag = 1;
        }
    }
    else
    {
        if(canfd_timeout_cnt > 100) // canfd通信断开1s报警
        {
            canfd_timeout_cnt = 0;
            canfd_frame_flag = 1;
        }
    }

    // 判断can_bus_off
    static uint16_t can_buf_off_cnt = 0;
    if(CanfdRegs.CFG_STAT.bit.BUSOFF) // 如果检测到canfd总线关闭
    {
        can_buf_off_cnt++;
        if(can_buf_off_cnt > 100) 
        {
            can_buf_off_cnt = 0;
            canfd_buf_off_flag = 1;
        }
    }
    else
    {
        can_buf_off_cnt = 0;
        canfd_buf_off_flag = 0;
    }
}
