#ifndef UTILS_H
#define UTILS_H

#include "MainInclude.h"

#define M_PI    3.1415927410f
#define TWO_PI  6.2831854820f

#define CLAMP(x, min, max) (((x) > (max)) ? (max) : (((x) < (min)) ? (min) : (x)))

uint32_t square(uint64_t x);
float logf(float x);

#endif
