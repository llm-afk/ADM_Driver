#ifndef ENCODER_H
#define ENCODER_H

#include "MainInclude.h"
#include "SPIcomm.h"

#define GEAR_RATIO 18 // 减速器传动比
#define GEAR_RATIO_INV (1.0f / GEAR_RATIO)

typedef struct{
    uint16_t elec_degree_calib;    
}encoder_config_t;


typedef struct{
    uint16_t main_encoder_raw;      // 原始编码器值
    uint16_t main_encoder_lined;    // 线性化后的编码器值
    
    // 电机端多圈位置（圈数+圈内位置）
    int32_t motor_turns;            // 电机端圈数（正负表示方向）
    uint16_t motor_position;        // 电机端圈内位置 0-65535

    // 电机端速度
    int32_t volacity_q12;           // 电机端速度 Q12格式 (counts/sample)
    int32_t _vel_acc;               // 速度滤波累加器

    // 减速端位置和速度
    int32_t volacity_q12_gear;      // 减速端速度 rad/s Q12
    int32_t position_q12_gear;      // 减速端位置 rad Q12

    int16_t elec_degree;            // 电角度
}encoder_t;

extern encoder_config_t encoder_config;
extern encoder_t encoder;

void encoder_loop(void);

#endif
