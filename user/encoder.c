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
    // =========================================================================
    // 【优化 1】读取硬件与缓存结构体变量 (减少寻址与内存操作开销)
    // 提前读取编码器，避免在if-else分支中调用函数打断流水线预测
    // =========================================================================
    uint16_t pri_enc_val = get_pri_enc_val();
    uint16_t sec_enc_val = get_sec_enc_val();
    uint16_t is_reverse = encoder_config.encoder_reverse; // 缓存常用判断条件

    uint16_t raw_deg_rev;
    uint16_t ex_raw;

    encoder.enc_degree_raw = pri_enc_val;

    if(is_reverse)
    {
        raw_deg_rev = 16383 - pri_enc_val;
        ex_raw      = 16383 - sec_enc_val;
    }
    else
    {
        raw_deg_rev = pri_enc_val;
        ex_raw      = sec_enc_val;
    }

    // 缓存至结构体 (集中写入，避免频繁进出结构体)
    encoder.enc_degree_raw_reversed = raw_deg_rev;
    encoder.ex_enc_degree_raw       = ex_raw;
    ODObjs.in_encoder_offset        = raw_deg_rev;
    ODObjs.ex_encoder_offset        = ex_raw;

    // =========================================================================
    // 【优化 2】线性补偿
    // =========================================================================
    uint16_t lined_deg;
    {
        uint16_t idx1 = raw_deg_rev >> 5; 
        uint16_t idx2 = (idx1 + 1) & 0x1FF; 
        uint16_t fraction = raw_deg_rev & 0x1F; 

        int16_t err1 = encoder_config.linearity_table[idx1];
        int16_t err2 = encoder_config.linearity_table[idx2];

        // 【核心优化 2.1】：将除法 /32 改为带符号算术右移 >>5。
        // TI 编译器处理带符号整数除法时会增加调整指令以保证向零取整，
        // 使用 >> 5 不仅仅只需单周期，且在控制系统连续平滑度上表现更好。
        int32_t interp_error = err1 + ( ((int32_t)(err2 - err1) * fraction) >> 5 );

        // 【核心优化 2.2】：由于 ENCODER_CPR 是 16384 (2的次幂)，
        // 直接使用无符号运算并做按位与，合并减法操作。
        lined_deg = (raw_deg_rev - interp_error) & (ENCODER_CPR - 1);
        encoder.enc_degree_lined = lined_deg;
    }

    // =========================================================================
    // 【优化 3】维护多圈累计值
    // =========================================================================
    static uint16_t degree_last = 0;
    static uint16_t flag = 0; 
    
    // 【注意】虽然这是分支，但仅在上电第一拍执行。若平台支持，最好将它移至初始化函数。
    if(!flag) 
    {
        flag = 1;
        degree_last = lined_deg; 
    }
    
    int16_t delta = (int16_t)(lined_deg - degree_last);
    
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
    degree_last = lined_deg;

    // =========================================================================
    // 【优化 4】计算速度
    // =========================================================================
    {
        #define VEL_WINDOW_BITS      3
        #define VEL_WINDOW_SIZE      (1 << VEL_WINDOW_BITS)
        #define VEL_ALPHA_MIN_Q8     1    
        #define VEL_ALPHA_MAX_Q8     128  
        #define VEL_NOISE_DEADZONE   16000  

        static int16_t  delta_history[VEL_WINDOW_SIZE] = {0};
        static uint16_t history_idx = 0;
        static int32_t  delta_sum = 0;

        static int32_t  velocity_temp_q14 = 0;
        static int32_t  vel_alpha_q8 = VEL_ALPHA_MIN_Q8;
        static int32_t  alpha_target_smooth_q8 = VEL_ALPHA_MIN_Q8; 
        static uint16_t vel_filter_init = 0;

        // 【核心优化 4.1】：合并加减法，省去一次中间存储操作
        delta_sum += delta - delta_history[history_idx];
        delta_history[history_idx] = delta;
        history_idx = (history_idx + 1) & (VEL_WINDOW_SIZE - 1);

        int32_t vel_target_q14 = (delta_sum * 125664) >> VEL_WINDOW_BITS;

        if(!vel_filter_init) {
            vel_filter_init = 1;
            velocity_temp_q14 = vel_target_q14;
            alpha_target_smooth_q8 = VEL_ALPHA_MIN_Q8;
        }

        int32_t error = vel_target_q14 - velocity_temp_q14;
        
        // 【核心优化 4.2】：TI C2000编译器遇到此种三目运算时，
        // 会无分支地直接将其编译为硬件级别的 ABS (绝对值) 和 MAX/MIN 指令。极大地节省开销。
        int32_t abs_error = (error >= 0) ? error : -error;
        
        int32_t active_error = abs_error - VEL_NOISE_DEADZONE;
        active_error = (active_error > 0) ? active_error : 0; // 映射为 MAX 指令

        int32_t alpha_instant = VEL_ALPHA_MIN_Q8 + (active_error >> 9);
        alpha_instant = (alpha_instant < VEL_ALPHA_MAX_Q8) ? alpha_instant : VEL_ALPHA_MAX_Q8; // 映射为 MIN 指令

        alpha_target_smooth_q8 += (alpha_instant - alpha_target_smooth_q8) >> 4;

        // 【核心优化 4.3】：保留 if-else 用于移位（在 F28035 上，基于变量的移位和常量移位开销不同，
        // 显式分离能让编译器直接使用常量移位 SFR 指令）
        int32_t alpha_diff = alpha_target_smooth_q8 - vel_alpha_q8;
        if(alpha_diff > 0) {
            vel_alpha_q8 += alpha_diff >> 1; 
        } else {
            vel_alpha_q8 += alpha_diff >> 3; 
        }

        velocity_temp_q14 += (error * vel_alpha_q8) >> 8;
        encoder.enc_velocity_q14 = velocity_temp_q14;
    }
    
    // =========================================================================
    // 【优化 5】更新电角度
    // =========================================================================
    if(motor_ctrl.state == MIT)
    {
        // 计算基础映射值
        uint16_t base_elec = (lined_deg - encoder_config.elec_degree_calib) & 0x7FF;
        
        // 还原原代码逻辑并缓存计算结果，避免重复解引用 encoder.elec_degree
        // 警告：原代码为 65535 - (uint16_t)(...) << 5 
        // 按照 C 语言优先级，减法 '-' 的优先级比左移 '<<' 高！因此等于 (65535 - 基础值) << 5
        // 此处严格复现了您的运算优先级以防逻辑改变。
        uint16_t e_deg_temp = (65535 - base_elec) << 5;

        // 【核心优化 5.1】：避免冗余的二次取反。
        // 原逻辑若不是 reverse 相当于 65535 - ((65535 - base) << 5)
        // 优化后用单次条件判断决定最终赋值，且直接利用前面已缓存的 is_reverse 寄存器变量。
        if(is_reverse)
        {
            encoder.elec_degree = e_deg_temp;
        }
        else
        {
            encoder.elec_degree = 65535 - e_deg_temp;   
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
    return 0;
} 

int16_t temp_cw[512] = {0};
int16_t temp_ccw[512] = {0};

/**
 * @brief 2khz编码器校准程序
 * @return 0 校准中 1 校准完成
 */
uint16_t encoder_calibrate(void)
{
    static uint16_t cnt = 0;
    static uint16_t state = 0;
    static uint16_t enc_degree_raw_start = 0;
    static int16_t   mech_dir = 1; // 记录电角度正转时，机械角度是增加(1)还是减少(-1)

    #define POLE_PAIRS 8
    #define CALIB_CURRENT 1000 // 保持拖动电流，防止电机不转

    switch(state)
    {
        case 0: // soft lock
        {
            Iq = 0;
            Id = cnt; // 软启动爬升
            encoder.elec_degree = 0;
            if(cnt >= CALIB_CURRENT)
            {
                enc_degree_raw_start = encoder.enc_degree_raw;
                cnt = 0;
                state = 1;
            }
            break;
        }
        case 1: // cw find direction
        {
            Iq = 0; Id = CALIB_CURRENT;
            encoder.elec_degree += 256;
            if(cnt >= 512) 
            {
                int16_t degree_dif = (int16_t)(encoder.enc_degree_raw - enc_degree_raw_start);
                if(degree_dif > ENCODER_CPR_DIV) degree_dif -= ENCODER_CPR;
                else if(degree_dif < -ENCODER_CPR_DIV) degree_dif += ENCODER_CPR;

                // 确立电角度与机械角度的方向映射关系
                if(degree_dif < 0) 
                {
                    encoder_config.encoder_reverse = 0;
                    mech_dir = 1;  // 电角度正转 -> 机械角度增加
                }
                else 
                {
                    encoder_config.encoder_reverse = 1;
                    mech_dir = -1; // 电角度正转 -> 机械角度减少
                }
                
                cnt = 0;
                state = 2;
            } 
            break;
        }
        case 2: // soft lock and get electrical zero
        {
            Iq = 0; Id = CALIB_CURRENT;
            encoder.elec_degree = 0;
            if(cnt >= 1000)
            {
                cnt = 0;
                state = 3;
                encoder_config.elec_degree_calib = (encoder.enc_degree_raw_reversed & 0x7FF);
            }
            break;
        }
        case 3: // cw loop (正转跑一圈记录绝对误差)
        {
            Iq = 0; Id = CALIB_CURRENT;
            static int16_t first_raw_val = 0;
            
            if(cnt == 1)
            {
                first_raw_val = encoder.enc_degree_raw_reversed;
            }
            
            if(cnt <= 4096)
            {
                if((cnt - 1) % 8 == 0) // 1, 9, 17...4089
                {
                    uint16_t idx = (cnt - 1) >> 3; // 时间步进序号 0~511
                    
                    // 1. 计算当前理论上应该到达的【绝对物理位置】 (起点 + 步长)
                    int32_t ideal_pos = first_raw_val + (32 * idx * mech_dir);
                    
                    // 将理想位置限制在 0 ~ 16383 的环形空间内 (假设 ENCODER_CPR 是 16384)
                    ideal_pos = ideal_pos & (ENCODER_CPR - 1); 
                    
                    // 2. 计算误差 = 实际绝对位置 - 理想绝对位置
                    int16_t error = (int16_t)encoder.enc_degree_raw_reversed - (int16_t)ideal_pos;
                    
                    // 3. 对微小误差进行最短路径处理 (如果误差>8192，说明跨越了0点)
                    if(error > ENCODER_CPR_DIV) error -= ENCODER_CPR;
                    else if(error < -ENCODER_CPR_DIV) error += ENCODER_CPR;
                    
                    // 4. 计算当前对应的绝对物理区间 (0~511)
                    uint16_t start_bin = first_raw_val >> 5; 
                    uint16_t abs_bin = (start_bin + idx * mech_dir) & 0x1FF; 
                    
                    // 存入绝对物理区间
                    temp_cw[abs_bin] = error; 
                }
            }
            else
            {
                cnt = 0;
                state = 4;
            }
            encoder.elec_degree += 128; // 驱动电机
            break;
        }
        
        case 4: // dummy
        {
            Iq = 0; Id = CALIB_CURRENT;
            encoder.elec_degree = 0; 
            if(cnt >= 1000)
            {
                cnt = 0;
                state = 5;
            }
            break;
        }
        
        case 5: // ccw loop (反转跑一圈记录绝对误差)
        {
            Iq = 0; Id = CALIB_CURRENT;
            static int16_t first_raw_val = 0;
            
            if(cnt == 1)
            {
                first_raw_val = encoder.enc_degree_raw_reversed;
            }
            
            if(cnt <= 4096)
            {
                if((cnt - 1) % 8 == 0)
                {
                    uint16_t idx = (cnt - 1) >> 3; // 0~511
                    
                    // 1. 计算理论【绝对物理位置】 (反转，所以是 起点 - 步长)
                    int32_t ideal_pos = first_raw_val - (32 * idx * mech_dir);
                    
                    // 将理想位置限制在 0 ~ 16383 之间
                    ideal_pos = ideal_pos & (ENCODER_CPR - 1);
                    
                    // 2. 误差 = 实际位置 - 理想位置
                    int16_t error = (int16_t)encoder.enc_degree_raw_reversed - (int16_t)ideal_pos;
                    
                    // 3. 误差最短路径处理
                    if(error > ENCODER_CPR_DIV) error -= ENCODER_CPR;
                    else if(error < -ENCODER_CPR_DIV) error += ENCODER_CPR;
                    
                    // 4. 计算当前的绝对物理区间 (反向退回)
                    uint16_t start_bin = first_raw_val >> 5;
                    uint16_t abs_bin = (start_bin - idx * mech_dir) & 0x1FF;
                    
                    temp_ccw[abs_bin] = error;
                }
            }
            else
            {
                cnt = 0;
                state = 6;
            }
            encoder.elec_degree -= 128; // 反转驱动
            break;
        }
        case 6: // calculate 
        {
            Iq = 0; Id = 0; // 关闭电流
            int32_t sum_error = 0;
            static int16_t smooth_buf[512]; // 临时缓冲区，用于存储滤波后的结果

            // 1. 初步合成并计算直流偏置
            for(uint16_t i = 0; i < 512; i++)
            {
                int16_t avg_error = (temp_cw[i] + temp_ccw[i]) / 2;
                encoder_config.linearity_table[i] = avg_error;
                sum_error += avg_error;
            }
            
            // 2. 消除直流偏置 (DC Offset)
            int16_t dc_offset = sum_error / 512;
            for(uint16_t i = 0; i < 512; i++)
            {
                encoder_config.linearity_table[i] -= dc_offset;
            }

            // 3. 环形平滑滤波 (3点滑动平均)
            // 目的：消除校准过程中产生的随机噪声和毛刺，让线性插值更丝滑
            // 注意：必须是环形的，即 511 的下一个点是 0
            for(uint16_t i = 0; i < 512; i++)
            {
                uint16_t prev = (i == 0) ? 511 : i - 1;
                uint16_t next = (i == 511) ? 0 : i + 1;
                
                // 3点加权平均 (也可以做成 1/4, 1/2, 1/4 加权，现在是简单的 1/3)
                smooth_buf[i] = (encoder_config.linearity_table[prev] + 
                                 encoder_config.linearity_table[i] + 
                                 encoder_config.linearity_table[next]) / 3;
            }

            // 4. 将平滑后的结果写回线性补偿表
            for(uint16_t i = 0; i < 512; i++)
            {
                encoder_config.linearity_table[i] = smooth_buf[i];
            }

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
