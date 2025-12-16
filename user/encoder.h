#ifndef ENCODER_H
#define ENCODER_H

#include "MainInclude.h"
#include "SPIcomm.h"
#include "motor_ctrl.h"

#define ENCODER_BITS            (14)
#define ENCODER_CPR             (1 << ENCODER_BITS)
#define ENCODER_CPR_DIV         (ENCODER_CPR >> 1)

#define GEAR_RATIO              (12) 
#define GEAR_RATIO_INV          (1.0f / GEAR_RATIO)

#define CALIB_TABLE_SIZE        (512) 

typedef struct{
    uint16_t elec_degree_calib; // 电角度校准值
    int16_t linearity_table[CALIB_TABLE_SIZE];  // 线性化补偿表
    int16_t encoder_reverse; // 编码器方向位
}encoder_config_t;

typedef struct{
    uint16_t enc_degree_raw;      // 原始编码器值
    uint16_t enc_degree_lined;    // 线性化后的编码器值
    int32_t enc_turns;            // 编码器端累加圈数
    int32_t enc_velocity_q14;     // 编码器端速度 (rad/s)
    
    int64_t degree_q14;           // 减速端角度 (rad)
    int32_t velocity_q14;         // 减速端速度 (rad/s)

    int16_t elec_degree;          // 电角度
}encoder_t;

extern encoder_config_t encoder_config;
extern encoder_t encoder;
void encoder_loop(void);
uint16_t encoder_calibrate(void);

#endif
