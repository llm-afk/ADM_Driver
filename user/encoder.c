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
    encoder.enc_velocity_q14 = (velocity_temp += (delta * 125664 - velocity_temp) >> 8); // 右移越大滤波越强

    // 更新电角度
    if(motor_ctrl.state == MIT)
    {
        // encoder.enc_degree_lined % 2048 * 8 * 4
        encoder.elec_degree = (((encoder.enc_degree_lined & 0x7FF) << 3) - encoder_config.elec_degree_calib) << 2; // 归一化到u16
    }
}

/**
 * @brief 2khz编码器校准程序
 * @return 0 未完成 1 完成
 */
uint16_t encoder_calibrate(void)
{
    // encoder.calibrate.cnt++;
    // switch(encoder.calibrate.state)
    // {
    //     case 0: // lock
    //     {
    //         Iq = 0;
    //         Id = 1024; 
    //         encoder.elec_degree = 0;

    //         if(encoder.calibrate.cnt > 1000)
    //         {
    //             encoder.calibrate.cnt = 0;
    //             encoder.calibrate.state = 1;
    //             encoder.calibrate.count_raw_start = encoder.main_encoder_raw;
    //         }
    //         break;
    //     }
    //     case 1: // cw find direction
    //     {
    //         if(encoder.calibrate.cnt > 1024) // 顺时针旋转2个电周期
    //         {
    //             int32_t diff = encoder.main_encoder_raw - encoder.calibrate.count_raw_start;
    //             if(diff > 32768)       
    //             {
    //                 diff -= 65536;     
    //             }
    //             else if(diff < -32768) 
    //             {
    //                 diff += 65536;  
    //             }
    //             if(diff < 0) 
    //             {
    //                 if(encoder_config.encoder_reverse) 
    //                 {
    //                     encoder_config.encoder_reverse = 0;
    //                 } 
    //                 else
    //                 {
    //                     encoder_config.encoder_reverse = 1;
    //                 }
    //             }
    //             encoder.calibrate.cnt = 0;
    //             encoder.calibrate.state = 2;
    //             break;
    //         }
    //         encoder.elec_degree += 128;
    //         break;
    //     }
    //     case 2: // cw dummy
    //     {
    //         if(encoder.calibrate.cnt > 1024) // 顺时针旋转2个电周期
    //         {
    //             encoder.calibrate.cnt = 0;
    //             encoder.calibrate.state = 3;
    //             break;
    //         }
    //         encoder.elec_degree += 128;
    //         break;
    //     }
    //     case 3: // cw loop
    //     {
    //         encoder.calibrate.cnt = 0;
    //         encoder.calibrate.state = 4;
    //         break;
    //     }
    //     case 4: // cw dummy
    //     {
    //         if(encoder.calibrate.cnt > 1024) // 顺时针旋转2个电周期
    //         {
    //             encoder.calibrate.cnt = 0;
    //             encoder.calibrate.state = 5;
    //             break;
    //         }
    //         encoder.elec_degree += 128;
    //         break;
    //     }
    //     case 5: // ccw dummy
    //     {
    //         if(encoder.calibrate.cnt > 1024) // 逆时针旋转2个电周期
    //         {
    //             encoder.calibrate.cnt = 0;
    //             encoder.calibrate.state = 6;
    //             break;
    //         }
    //         encoder.elec_degree -= 128;
    //         break;
    //     }
    //     case 6: // ccw loop
    //     {
    //         if(encoder.calibrate.cnt > 10240) 
    //         {
    //             encoder.calibrate.cnt = 0;
    //             encoder.calibrate.state = 7;
    //             break;
    //         }
    //         break;
    //     }
    //     case 7: // calculate 
    //     {


    //         encoder.calibrate.cnt = 0;
    //         encoder.calibrate.state = 0;
    //         return 1; // 校准完成
    //     }
    //     default:
    //     {
    //         break;
    //     }
    // }
    return 0; // 校准中
}
