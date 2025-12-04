#ifndef IAP_H
#define IAP_H

#include "MainInclude.h"
#include "flash_eeprom.h"
#include "mm.h"

#define IAP_FLAG_ADDR 0x00BFFF
#define IAP_FLAG_NUM 0xA5A5

#define SET_IAP_FLAG(x)  (*(volatile uint16_t *)IAP_FLAG_ADDR = (uint16_t)(x))
#define GET_IAP_FLAG()   (*(volatile uint16_t *)IAP_FLAG_ADDR)

uint16_t clean_download(void);
void jump_to_download(void);

#endif
