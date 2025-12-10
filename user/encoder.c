#include "encoder.h"

encoder_config_t encoder_config = {
    .elec_degree_calib = 63472,
};

encoder_t encoder = {0};

/**
 * @brief 编码器数据更新loop
 * @note 放在电角度的20Khz中断里执行
 */
#pragma CODE_SECTION(encoder_loop,"ramfuncs");
void encoder_loop(void)
{
    // 读取编码器原始值
    encoder.main_encoder_raw = get_main_degree_raw();

    // 线性化处理
    encoder.main_encoder_lined = encoder.main_encoder_raw; 

    // 维护多圈计数（圈数+圈内位置）
    static uint16_t last_position = 0;
    int16_t delta = (int16_t)(encoder.main_encoder_lined - last_position);
    
    // 检测过零点，更新圈数
    int32_t delta_32 = (int32_t)((int32_t)encoder.main_encoder_lined - (int32_t)last_position);
    if(delta_32 > 32767) encoder.motor_turns--;
    else if(delta_32 < -32767) encoder.motor_turns++;

    last_position = encoder.main_encoder_lined;
    
    // 更新圈内位置
    encoder.motor_position = encoder.main_encoder_lined;

    // 更新速度（Q12格式，一阶低通滤波）
    encoder.volacity_q12 = (((encoder._vel_acc += (((int32_t)delta << 18) - encoder._vel_acc + 32) >> 6)) + 32) >> 6;

    // 更新电角度
    encoder.elec_degree = ((encoder.main_encoder_lined & 0x1FFF) << 3) - encoder_config.elec_degree_calib;
}
