#ifndef UTILS_H
#define UTILS_H

#include "MainInclude.h"

#define M_PI    3.1415927410f
#define TWO_PI  6.2831854820f

#define CLAMP(x, min, max) (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))

uint32_t square(uint64_t x);
float logf(float x);

/**
 * @brief  计算电流模长并转换为 float（上报用，极速）
 *         approx sqrt(M^2 + T^2)
 *
 * @param  M  M轴电流（定点）
 * @param  T  T轴电流（定点）
 * @param  rated_cur  电机额定电流(A)
 *
 * @return 电流值(A)，float
 */
static inline float imt_current_to_float(int16_t M, int16_t T, float rated_cur)
{
    int32_t absM, absT;

    // -------- 安全 abs，避免 -32768 溢出 --------
    if (M == (int16_t)0x8000)
        absM = 32767;
    else
        absM = (M >= 0) ? M : -M;

    if (T == (int16_t)0x8000)
        absT = 32767;
    else
        absT = (T >= 0) ? T : -T;

    // -------- 可选：限幅（强烈建议） --------
    if (absM > 32767) absM = 32767;
    if (absT > 32767) absT = 32767;

    // -------- 近似模长 --------
    int32_t maxv, minv;
    if (absM > absT) {
        maxv = absM;
        minv = absT;
    } else {
        maxv = absT;
        minv = absM;
    }

    // mag ≈ max + 0.375 * min
    int32_t mag = maxv + ((minv * 3) >> 3);

    // -------- 符号（保持你原逻辑） --------
    if ((M + T) < 0)
        mag = -mag;

    return (float)mag * rated_cur / 40960.0f;
}

#endif
