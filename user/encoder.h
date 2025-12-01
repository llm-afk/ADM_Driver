#ifndef ENCODER_H
#define ENCODER_H

#include "MainInclude.h"

typedef struct{
    uint16_t main_encoder_raw_val;
    uint16_t main_encoder_lined_val;
    uint16_t elec_degree_calib_val;
}encoder_t;

extern encoder_t encoder;

#endif
