#include "SPIcomm.h"
#include "Main.h"
#include "MotorInclude.h"
#include "Record.h"
#include "AngleSensor.h"
#include "encoder.h"

void InitSpi(void)
{
    InitSpiIO();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.SPIAENCLK = 1;   // SPI-A
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;//SPI 软件复位位。在改变 SPI 模块的配置之前应先将该位清零；在恢复 SPI操作之前应将该位置 1
    SpiaRegs.SPICCR.bit.SPICHAR = 0xf;//16位数据长度
    SpiaRegs.SPICCR.bit.SPILBK = 0;//回环控制禁止
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 1;//数据在上升沿输出，在下降沿输入。当没有SPI 数据发送时， SPICLK 为低电 平。数据的输入和输出边沿视
    SpiaRegs.SPICTL.bit.SPIINTENA =0;//禁止中断
    //SpiaRegs.SPICTL.bit.SPIINTENA = 1;//使能中断
    SpiaRegs.SPICTL.bit.TALK = 1;//允许4 脚发送，保证使能接收器的SPISTE 输入管脚。
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;//SPI 被配置为主控制器
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;//标准的SPICLK 信号，其极性由CLOCK POLARITY 位决定
    SpiaRegs.SPICTL.bit.OVERRUNINTENA =0;//禁止RECEIVER OVERRUN Flag 位产生的中断
    SpiaRegs.SPIBRR = 6; // 目前是 100/7 = 14.29Mhz 的spi时钟频率
    SpiaRegs.SPICCR.bit.SPISWRESET =1;
    EDIS;
}

void InitSpiIO(void)
{
    EALLOW;

    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pull-up on GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pull-up on GPIO19 (SPISTEA)

    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO18 (SPICLKA)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // Asynch input GPIO19 (SPISTEA)

    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA

    EDIS;
}

/**
 * @brief spi交换16bit数据的函数
 * @param data 发送的16bit数据
 * @return 换回来的16bit数据
 */
#pragma CODE_SECTION(SPITransfer,"ramfuncs");
Uint16 SPITransfer(Uint16 data)
{
    while(SpiaRegs.SPISTS.bit.BUFFULL_FLAG == 1);
    SpiaRegs.SPITXBUF = data;
    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0);
    return SpiaRegs.SPIRXBUF;
}

/**
 * @brief 获取主编码器的原始角度u16
 */
#pragma CODE_SECTION(get_main_degree_raw,"ramfuncs");
uint16_t get_main_degree_raw(void)
{
    uint16_t h_data = SPITransfer(0x8300); // 获取主编码器03寄存器的高8位值
    uint16_t l_data = SPITransfer(0x8400); // 获取主编码器04寄存器的低6位值
    return (((((h_data & 0x00FF) << 8) | (l_data & 0x00FF)) >> 2) << 2);
}

/**
 * @brief 获取副编码器的原始角度u16
 */
#pragma CODE_SECTION(get_ex_degree_raw,"ramfuncs");
uint16_t get_ex_degree_raw(void)
{
    // uint16_t h_data = SPITransfer(0x8300); // 获取主编码器03寄存器的高8位值
    // uint16_t l_data = SPITransfer(0x8400); // 获取主编码器04寄存器的低6位值
    // return (((((h_data & 0x00FF) << 8) | (l_data & 0x00FF)) >> 2) << 2);
}


