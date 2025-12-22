#include "encoder.h"
#include "flash_eeprom.h"

encoder_config_t encoder_config = {
    .elec_degree_calib = 1675,
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

    // 修正旋转方向
    if(encoder_config.encoder_reverse)
    {
        encoder.enc_degree_raw = ENCODER_CPR - 1 - encoder.enc_degree_raw;
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
        int16_t elec_raw = (int16_t)((encoder.enc_degree_lined & 0x7FF) - encoder_config.elec_degree_calib);
        if(elec_raw < 0)elec_raw += 2048;

        encoder.elec_degree = ((uint16_t)elec_raw << 5);
        if(encoder_config.encoder_reverse)
        {
            encoder.elec_degree = 65535 - encoder.elec_degree;
        }
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
    static uint16_t enc_degree_raw = 0;

    switch(state)
    {
        case 0: // lock
        {
            Iq = 0;
            Id = 1024;
            encoder.elec_degree = 0;
            if(cnt >= 1000)
            {
                enc_degree_raw = encoder.enc_degree_raw;
                cnt = 0;
                state = 1;
            }
            break;
        }
        case 1: // cw find direction
        {
            encoder.elec_degree += 256; // 旋转4个电周期
            if(cnt >= 1024)
            {
                int16_t degree_dif = encoder.enc_degree_raw - enc_degree_raw;
                if (degree_dif > ENCODER_CPR_DIV) 
                {
                    degree_dif -= ENCODER_CPR;
                } 
                else if(degree_dif < -ENCODER_CPR_DIV) 
                {
                    degree_dif += ENCODER_CPR;
                }

                if(degree_dif > 0)
                {
                    if(encoder_config.encoder_reverse) 
                    {
                        encoder_config.encoder_reverse = 0;
                    } 
                    else
                    {
                        encoder_config.encoder_reverse = 1;
                    }
                }
                cnt = 0;
                state = 2;
            } 
            break;
        }
        case 2: // lock
        {
            Iq = 0;
            Id = 1024;
            encoder.elec_degree = 0;
            if(cnt >= 1000)
            {
                encoder_config.elec_degree_calib = (encoder.enc_degree_raw & 0x7FF);
                enc_degree_raw = encoder.enc_degree_raw;
                cnt = 0;
                state = 3;
            }
            break;
        }
        case 3: // cw dummy
        {
            state = 4;
            break;
        }
        case 4: // cw loop
        {
            state = 5;
            break;
        }
        case 5: // cw dummy
        {
            state = 6;
            break;
        }
        case 6: // ccw dummy
        {
            state = 7;
            break;
        }
        case 7: // ccw loop
        {
            state = 8;
            break;
        }
        case 8: // calculate 
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
