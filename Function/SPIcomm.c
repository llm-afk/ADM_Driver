#include "SPIcomm.h"
#include "Main.h"
#include "MotorInclude.h"
#include "Record.h"
#include "AngleSensor.h"
#include "encoder.h"

#define PRI_CS_H()      (GpioDataRegs.GPACLEAR.bit.GPIO19 = 1)
#define PRI_CS_L()      (GpioDataRegs.GPASET.bit.GPIO19 = 1)

#define SRC_CS_H()      (GpioDataRegs.GPACLEAR.bit.GPIO1 = 1)
#define SRC_CS_L()      (GpioDataRegs.GPASET.bit.GPIO1 = 1)

void InitSpi(void)
{
    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pull-up on GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
    
    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO18 (SPICLKA)

    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;  // Configure GPIO16 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;  // Configure GPIO17 as SPISOMIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;  // Configure GPIO18 as SPICLKA

    GpioCtrlRegs.GPAPUD.bit.GPIO19  = 0; // pri_cs
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO19  = 1;  
    
    GpioCtrlRegs.GPAPUD.bit.GPIO1   = 0; // sec_cs
    GpioCtrlRegs.GPAMUX1.bit.GPIO1  = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO1   = 1;  

    PRI_CS_H();
    SRC_CS_H();

    EDIS;
}

#pragma CODE_SECTION(spi_transfer_pri,"ramfuncs");
static uint16_t spi_transfer_pri(uint16_t data)
{
    PRI_CS_L();
    while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1);
    SpiaRegs.SPITXBUF = data;
    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0);
    PRI_CS_H();
    return SpiaRegs.SPIRXBUF;
}

#pragma CODE_SECTION(spi_transfer_sec,"ramfuncs");
static uint16_t spi_transfer_sec(uint16_t data)
{
    SEC_CS_L();
    while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1);
    SpiaRegs.SPITXBUF = data;
    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0);
    SEC_CS_H();
    return SpiaRegs.SPIRXBUF;
}

#pragma CODE_SECTION(get_pri_enc_val,"ramfuncs");
uint16_t get_pri_enc_val(void)
{
    
}

#pragma CODE_SECTION(get_sec_enc_val,"ramfuncs");
uint16_t get_sec_enc_val(void)
{

}
