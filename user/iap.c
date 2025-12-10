#include "iap.h"

#if defined(APP)

uint32_t addr_offset = 0;

/**
 * @brief 擦除download空间的三个扇区
 * @return 0:擦除成功
 */
uint16_t clean_download(void)
{
    uint16_t error_mask = 0;
    error_mask |= flash_erase_sector(SECTORB);
    error_mask |= flash_erase_sector(SECTORC);
    error_mask |= flash_erase_sector(SECTORD);
    return error_mask;
}

/**
 * @brief app跳转到bootloader
 */
void jump_to_download(void)
{
    SET_IAP_FLAG(IAP_FLAG_NUM); // 设置更新标志位
    ResetDSP(); // 直接软重启dsp
}

/**
 * @brief 逐个8字节的接收新固件的bin文件，攒满一个扇区擦除写入
 * @param data 新固件的8字节小包的首地址
 */
void write_iap_data(uint16_t *data)
{
    if((addr_offset + 4) < DOWNLOAD_SIZE)
    {
        flash_program((uint16_t*)(DOWNLOAD_ADDR + 4), data, 4);
        addr_offset+=4;
    }
}

#else

/**
 * @brief bootloader跳转到app
 * @note 
 */
static void jump_to_app(void)
{
    SET_IAP_FLAG(IAP_FLAG_NUM); // 设置更新标志位
    ResetDSP(); // 直接软重启dsp
}

void bootloader(void)
{

}

#endif
