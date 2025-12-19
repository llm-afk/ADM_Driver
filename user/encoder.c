#include "encoder.h"
#include "flash_eeprom.h"

encoder_config_t encoder_config = {
    .elec_degree_calib = 13440,
    .linearity_table = {0},
    .encoder_reverse = 0,
};

encoder_t encoder = {0};

/**
 * @brief 编码器数据更新循环
 * @note 在电角度的 20kHz 中断中执行
 */
#pragma CODE_SECTION(encoder_loop,"ramfuncs");
void encoder_loop(void)
{
    // 更新编码器角度
    encoder.enc_degree_raw = get_main_degree_raw();
    encoder.ex_enc_degree_raw = get_ex_degree_raw();

    // 修正旋转方向
    if(encoder_config.encoder_reverse)
    {
        encoder.enc_degree_raw = ENCODER_CPR - encoder.enc_degree_raw;
    }

    // 线性补偿
    encoder.enc_degree_lined = encoder.enc_degree_raw; 

    // 维护多圈累计值
    static uint16_t degree_last = 0;
    int16_t delta = (int16_t)(encoder.enc_degree_lined - degree_last);
    
    if(delta > ENCODER_CPR_DIV) 
    {
        encoder.enc_turns--;
        delta -= ENCODER_CPR; 
    }
    else if(delta < -ENCODER_CPR_DIV) 
    {
        encoder.enc_turns++;
        delta += ENCODER_CPR; 
    }

    degree_last = encoder.enc_degree_lined;

    // 更新编码器速度
    // enc_velocity_q14 = delta / ENCODER_CPR * 2pi * 20000 * 2^14 (rad/s)
    static int32_t velocity_temp = 0;
    encoder.enc_velocity_q14 = (velocity_temp += (delta * 125664 - velocity_temp) >> 6); // 右移越大滤波越强

    // 更新电角度
    if(motor_ctrl.state == MIT)
    {
        // (encoder.enc_degree_lined % (ENCODER_CPR / 8)) * 8 * 4
        encoder.elec_degree = (((encoder.enc_degree_lined & 0x7FF) << 3) - encoder_config.elec_degree_calib) << 2; // 归一化到u16
    }
}

/**
 * @brief 2khz编码器校准程序
 * @return 0 校准中 1 校准完成
 */
uint16_t encoder_calibrate(void)
{
    static uint16_t cnt = 0;
    static uint16_t state = 0;

    switch(state)
    {
        case 0: // lock
        {
            Iq = 0;
            Id = 1024;
            if(cnt >= 2000)
            {
                state = 1;
                cnt = 0;
            }
            break;
        }
        case 1: // cw find direction
        {
            state = 2;
            break;
        }
        case 2: // cw dummy
        {
            state = 3;
            break;
        }
        case 3: // cw loop
        {
            state = 4;
            break;
        }
        case 4: // cw dummy
        {
            state = 5;
            break;
        }
        case 5: // ccw dummy
        {
            state = 6;
            break;
        }
        case 6: // ccw loop
        {
            state = 7;
            break;
        }
        case 7: // calculate 
        {
            state = 0;
            cnt = 0;
            return 1; // 校准完成
        }
        default:
        {
            break;
        }
    }
    cnt++;
    return 0; // 校准中
}
