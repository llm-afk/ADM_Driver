#include "SPIcomm.h"
#include "Main.h"
#include "MotorInclude.h"
#include "Record.h"
#include "AngleSensor.h"
#include "encoder.h"

//Define
//SSI片选信号控制，低有效
#define SSI_En()      (GpioDataRegs.GPADAT.bit.GPIO19 = 0)
#define SSI_Dis()     (GpioDataRegs.GPADAT.bit.GPIO19 = 1)

//SSI时钟信号控制，从机在CLK的上升沿驱动，主机应在CLK的下降沿捕获
#define SSI_CLK_L()   (GpioDataRegs.GPADAT.bit.GPIO18 = 0)
#define SSI_CLK_H()   (GpioDataRegs.GPADAT.bit.GPIO18 = 1)

//SSI数据读取
#define SSI_DO_READ() (GpioDataRegs.GPADAT.bit.GPIO17)

//--------------------------------------------------------------

//SSI片选信号控制，低有效
#define SSI_En1()      (GpioDataRegs.GPBDAT.bit.GPIO32 = 0)
#define SSI_Dis1()     (GpioDataRegs.GPBDAT.bit.GPIO32 = 1)

//SSI时钟信号控制，从机在CLK的上升沿驱动，主机应在CLK的下降沿捕获
#define SSI_CLK_L1()   (GpioDataRegs.GPADAT.bit.GPIO1 = 0)
#define SSI_CLK_H1()   (GpioDataRegs.GPADAT.bit.GPIO1 = 1)

//SSI数据读取
#define SSI_DO_READ1() (GpioDataRegs.GPADAT.bit.GPIO16)

//--------------------------------------------------------------

// 数据长度(Bit0 - Bit23)
#define SSI_DATA_BITS 14


void InitSpi(void)
{
    //Init Gpio for Ssi
    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pull-up on GPIO19 (SPISTEA)

    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0; // Configure GPIO17 as GPIO
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0; // Configure GPIO18 as GPIO
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0; // Configure GPIO19 as GPIO

    GpioCtrlRegs.GPADIR.bit.GPIO17 = 0; //DO
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 1; //CLK
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 1; //CSN

//--------------------------------------------------------------

    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pull-up on GPIO19 (SPISTEA)

    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0; // Configure GPIO17 as GPIO
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0; // Configure GPIO18 as GPIO
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0; // Configure GPIO19 as GPIO

    GpioCtrlRegs.GPADIR.bit.GPIO16 = 0; //DO
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1; //CLK
    GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1; //CSN

    EDIS;

    // 初始状态：CLK高电平，CSN高电平（不选中）
    GpioDataRegs.GPADAT.bit.GPIO19 = 1;
    GpioDataRegs.GPADAT.bit.GPIO18 = 1;

    GpioDataRegs.GPADAT.bit.GPIO19 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO32 = 1;
}

// SSI数据读取函数
#pragma CODE_SECTION(SSI_ReadData,"ramfuncs");
uint16_t SSI_ReadData(void)
{
    Uint32 data = 0;
    Uint16 i;

    // 启动传输：拉低片选
    SSI_En();  // CSN = 0
    asm(" RPT #3 || NOP");

    // 确保初始时钟为高电平
    SSI_CLK_H();    // CLK = 1
    asm(" RPT #3 || NOP");

    SSI_CLK_L();    // CLK = 0
    asm(" RPT #10 || NOP");

    SSI_CLK_H();    // CLK = 1
    asm(" RPT #5 || NOP");

    // 读取23位数据
    for(i = 0; i < ( SSI_DATA_BITS); i++)
    {
        // 产生时钟下降沿
        SSI_CLK_L();  // CLK = 0
        asm(" RPT #3 || NOP");
        // 在时钟下降沿读取数据位
        if(SSI_DO_READ() == 1) {
            data |= (1UL << (SSI_DATA_BITS -1 - i));  // 高位在前
        }
        // 产生时钟上升沿（从机在此时更新数据）
        SSI_CLK_H();    // CLK = 1
        asm(" RPT #3 || NOP");
    }
    // 结束传输：拉高片选
    SSI_Dis();  // CSN = 1
    return ((uint16_t)data);
}

// SSI数据读取函数
#pragma CODE_SECTION(SSI_ReadData1,"ramfuncs");
uint16_t SSI_ReadData1(void)
{
    Uint32 data = 0;
    Uint16 i;

    // 启动传输：拉低片选
    SSI_En1();  // CSN = 0
    asm(" RPT #3 || NOP");

    // 确保初始时钟为高电平
    SSI_CLK_H1();    // CLK = 1
    asm(" RPT #3 || NOP");

    SSI_CLK_L1();    // CLK = 0
    asm(" RPT #10 || NOP");

    SSI_CLK_H1();    // CLK = 1
    asm(" RPT #5 || NOP");

    // 读取23位数据
    for(i = 0; i < ( SSI_DATA_BITS); i++)
    {
        // 产生时钟下降沿
        SSI_CLK_L1();  // CLK = 0
        asm(" RPT #3 || NOP");
        // 在时钟下降沿读取数据位
        if(SSI_DO_READ1() == 1) {
            data |= (1UL << (SSI_DATA_BITS -1 - i));  // 高位在前
        }
        // 产生时钟上升沿（从机在此时更新数据）
        SSI_CLK_H1();    // CLK = 1
        asm(" RPT #3 || NOP");
    }
    // 结束传输：拉高片选
    SSI_Dis1();  // CSN = 1
    return ((uint16_t)data);
}

/**
 * @brief 获取主编码器的原始值
 */
#pragma CODE_SECTION(get_main_degree_raw,"ramfuncs");
uint16_t get_main_degree_raw(void)
{
    return SSI_ReadData();
}

/**
 * @brief 获取副编码器的原始角度
 */
#pragma CODE_SECTION(get_ex_degree_raw,"ramfuncs");
uint16_t get_ex_degree_raw(void)
{
    return (16384 - SSI_ReadData1());
}
