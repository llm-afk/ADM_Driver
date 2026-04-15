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

            int32_t out_q14 = (int32_t)((((int64_t)motor_ctrl.Kp_q14 * degree_err_q14) >> 14) * MOTOR_RATED_CUR * 1.41421356f / 40960) \
                            + (int32_t)((((int64_t)motor_ctrl.Kd_q14 * velocity_err_q14) >> 14) * MOTOR_RATED_CUR * 1.41421356f / 40960) \
                            + motor_ctrl.torque_ref_q14; 

            ODObjs.torque_limit = CLAMP(ODObjs.torque_limit, 0.0f, 30.0f); // 限制OD中torque_limit的范围，避免用户设置过大导致电流环输出过大烧坏电机
            out_q14 = CLAMP(out_q14, (int32_t)(ODObjs.torque_limit * -16384.0f), (int32_t)(ODObjs.torque_limit * 16384.0f)); // 30 * 16384
            
            Iq = Torque_To_Iq(-out_q14 / 16384.0f) * 40960 / (MOTOR_RATED_CUR * 1.41421356f); 
            Id = 0;

            // static uint16_t mit_cnt = 0;
            // mit_cnt++;
            // if(mit_cnt == 20) // 100hz更新一次mit输出
            // {
            //     Id = 500;
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
                ResetDSP();
            }
            break;
        }
        case ENCODER_ZERO: 
        {

            break;
        }
        case SOFT_STOP:
        {
            DisableDrive();
            motor_ctrl.state = STOPPED;
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

extern uint16_t heatbeat_flag;

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

    const float alpha = 0.001f;

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
        if(motor_temp > ODObjs.over_temp_motor_level)
        {
            set_err(ERR_OVER_TEMP_MOTOR);
            motor_ctrl.state = SOFT_STOP;
        }
    }

    if(!(ODObjs.error_code & ERR_OVER_TEMP_DRV))
    {
        if(board_temp > ODObjs.over_temp_drv_level)
        {
            set_err(ERR_OVER_TEMP_DRV);
            motor_ctrl.state = SOFT_STOP;
        }
    }

    static uint16_t cnt = 0; // 上电10s后开始检测canfd通信状态和can_bus_off状态，避免上电瞬间没有canfd通信导致误报警
    if(cnt < 1000)
    {
        cnt++;
    }
    else
    {
        // 判断can_phy连通性
        if(ODObjs.heartbeat_consumer_enable) // 只有开启了心跳监测功能才进行canfd通信状态的判断
        {
            canfd_timeout_cnt++;
            if(canfd_timeout_cnt > 250) // 2.5s没有canfd通信了，认为can_phy断开了
            {
                canfd_frame_flag = 1;
                motor_ctrl.state = SOFT_STOP;
            }
            else
            {
                if(motor_ctrl.state == STOPPED && canfd_frame_flag == 1)
                {
                    motor_ctrl.state = MIT;
                }
                canfd_frame_flag = 0;
            }
        }

        // 判断can_bus_off
        static uint32_t can_buf_off_cnt = 0;
        if(CanfdRegs.CFG_STAT.bit.BUSOFF) // 如果检测到canfd总线关闭
        {
            can_buf_off_cnt++;
            if(can_buf_off_cnt > 100) 
            {
                can_buf_off_cnt = 0;
                canfd_buf_off_flag = 1;
                motor_ctrl.state = SOFT_STOP;
            }
        }
        else
        {
            if(motor_ctrl.state == STOPPED && canfd_buf_off_flag == 1)
            {
                motor_ctrl.state = MIT;
            }
            can_buf_off_cnt = 0;
            canfd_buf_off_flag = 0;
        }
    }

    // 生成1hz的心跳帧发送标志
    static uint16_t heartbeat_cnt = 0;
    heartbeat_cnt++;
    if(heartbeat_cnt >= 100) 
    {
        heartbeat_cnt = 0;
        heatbeat_flag = 1; 
    }
}
