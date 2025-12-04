#include "iap.h"

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

}

/**
 * @brief 
 */
void write_iap_data(uint16_t *data)
{

}
