
#ifndef _SPI_CANCOMM_H_
#define _SPI_CANCOMM_H_

#include "Define.h"
#include "ADP32F03x_Device.h"





extern void InitSpi(void);
extern void InitSpiIO(void);
extern Uint16 SPITransfer(Uint16 data);
void SPI_Updata_Angle(void);
#endif /* _SPI_CANCOMM_H_ */

