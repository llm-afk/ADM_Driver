
#ifndef _SPI_CANCOMM_H_
#define _SPI_CANCOMM_H_

#include "Define.h"
#include "ADP32F03x_Device.h"
#include "MainInclude.h"

void InitSpi(void);
uint16_t get_pri_enc_val(void);
uint16_t get_sec_enc_val(void);

#endif

