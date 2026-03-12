#include "encoder.h"
#include "flash_eeprom.h"

encoder_config_t encoder_config = {
    .elec_degree_calib = 1675,
    .linearity_table = {0},
    .encoder_reverse = 0,
};

encoder_t encoder = {0};

/**
 * @brief 编码器初始化
 */
void encoder_init(void)
{
    // 获取eeprom中标零时刻两个编码器的值
    encoder.in_enc_deg_zero_conf = ODObjs.in_encoder_offset; 
    encoder.ex_enc_deg_zero_conf = ODObjs.ex_encoder_offset;
    encoder.enc_error_conf = encoder.in_enc_deg_zero_conf - encoder.ex_enc_deg_zero_conf;
    if(encoder.enc_error_conf > ENCODER_CPR_DIV) 
    {
        encoder.enc_error_conf -= ENCODER_CPR; 
    }
    else if(encoder.enc_error_conf < -ENCODER_CPR_DIV) 
    {
        encoder.enc_error_conf += ENCODER_CPR; 
    }

    // 更新主编码器和副编码器角度
    uint16_t enc_temp = get_pri_enc_val(); // 需要确保编码器已经上电启动 55us
    uint16_t ex_enc_temp = get_sec_enc_val();

    // 修正编码器旋转方向
    if(encoder_config.encoder_reverse)
    {
        enc_temp = 16383 - enc_temp;
        ex_enc_temp = 16383 - ex_enc_temp;
    }

    // 记录上电时刻两个编码器的值
    encoder.in_enc_deg_zero = enc_temp;
    encoder.ex_enc_deg_zero = ex_enc_temp;
    encoder.enc_error = encoder.in_enc_deg_zero - encoder.ex_enc_deg_zero;
    if(encoder.enc_error > ENCODER_CPR_DIV) 
    {
        encoder.enc_error -= ENCODER_CPR; 
    }
    else if(encoder.enc_error < -ENCODER_CPR_DIV) 
    {
        encoder.enc_error += ENCODER_CPR; 
    }

    // 计算本次上电时相对标零角度的偏差
    encoder.error = encoder.enc_error - encoder.enc_error_conf;
    if(encoder.error > ENCODER_CPR_DIV) 
    {
        encoder.error -= ENCODER_CPR; 
    }
    else if(encoder.error < -ENCODER_CPR_DIV) 
    {
        encoder.error += ENCODER_CPR; 
    }
}

/**
 * @brief 编码器数据更新循环
 * @note 在电角度的 20kHz 中断中执行
 */
#pragma CODE_SECTION(encoder_loop,"ramfuncs");
void encoder_loop(void)
{
    // 更新主编码器和副编码器角度并修正方向
    if(encoder_config.encoder_reverse)
    {
        encoder.enc_degree_raw = 16383 - get_pri_enc_val();
        encoder.ex_enc_degree_raw = 16383 - get_sec_enc_val();
    }
    else
    {
        encoder.enc_degree_raw = get_pri_enc_val();
        encoder.ex_enc_degree_raw = get_sec_enc_val();
    }

    // 时刻更新主编码器值和副编码器值为了应对编码器标零时需要读取当前的角度值
    ODObjs.in_encoder_offset = encoder.enc_degree_raw;
    ODObjs.ex_encoder_offset = encoder.ex_enc_degree_raw;

    // 线性补偿
    encoder.enc_degree_lined = encoder.enc_degree_raw; 

    // 维护多圈累计值
    static uint16_t degree_last = 0;
    static uint16_t flag = 0; 
    if(flag == 0)
    {
        flag = 1;
        degree_last = encoder.enc_degree_lined; // 解决上电第一次delta偏大的问题
    }
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
        encoder.elec_degree = 65535-(uint16_t)((encoder.enc_degree_lined - encoder_config.elec_degree_calib) & 0x7FF) << 5;

        if(!encoder_config.encoder_reverse)
        {
            encoder.elec_degree = 65535 - encoder.elec_degree;   
        }
    }
}

/**
 * @brief od中触发标零后执行的函数
 * @note 实际抓取上位机的包发现，标零逻辑只执行了一次写0x2070也就是主编码器的值，然后所以在这里完成副编码器的值的存储和复位
 */
int enc_set_zero(void)
{
    load_ram_item_to_eeprom_from_key(3); // 保存副编码器值
    ResetDSP(); // 复位
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
            Id = cnt;
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
            encoder.elec_degree += 256;
            if(cnt >= 512) // 沿着电角度的正方向旋转两个电周期，我们规定这个方向为电机的正方向
            {
                int16_t degree_dif = (int16_t)(encoder.enc_degree_raw - enc_degree_raw);
                if(degree_dif > ENCODER_CPR_DIV) 
                {
                    degree_dif -= ENCODER_CPR;
                } 
                else if(degree_dif < -ENCODER_CPR_DIV) 
                {
                    degree_dif += ENCODER_CPR;
                }

                if(degree_dif > 0)
                {
                    encoder_config.encoder_reverse ^= 1;
                }
                cnt = 0;
                state = 2;
            } 
            break;
        }
        case 2: // lock
        {
            Iq = 0;
            Id = 1024 - cnt;
            encoder.elec_degree = 0;
            if(cnt == 500)
            {
                encoder_config.elec_degree_calib = (encoder.enc_degree_raw & 0x7FF);
            }
            else if(cnt >= 1000)
            {
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
            load_ram_item_to_eeprom_from_key(1); // 保存配置信息到eeprom
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
