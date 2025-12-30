#include "utils.h"

/**
 * @brief 计算 uint64_t 的整数平方根
 * @param x 输入的64位无符号整数
 * @return 返回 uint32_t 的平方根（取整）
 */
uint32_t square(uint64_t x)
{
    uint64_t res = 0;      // 最终结果
    uint64_t bit = (uint64_t)1 << 62; // 从最高位开始，每次试探

    // 调整 bit 到 <= x
    while (bit > x)
        bit >>= 2;

    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }

    return (uint32_t)res;
}

float logf(float x)
{
    float y = (x-1)/(x+1);
    float y2 = y*y;
    return 2*(y + y*y2/3 + y*y2*y2/5);  // 5阶展开
}
