#include "MainInclude.h"
#include "MotorInclude.h"
#include "Parameter.h"
#include "SPIcomm.h"
#include "AngleSensor.h"
#include "stimer.h"
#include "flash_eeprom.h"
#include "canfd.h"
#include "od.h"
#include "iap.h"

stimer_t stimer_main; // 主函数用软定时器模块

void main(void)
{
    InitSysCtrl();                   
    InitInterrupt();                 
    InitPeripherals();             
    InitForMotorApp();        
    InitForFunctionApp();
    EnableDog();
    SetInterruptEnable();        
    EINT; //使能全局中断
    ERTM; //使能实时模式

    OD_init(); // 先初始化OD对象,初始化参数默认值

    eeprom_init();
    load_eeprom_to_ram(); // 初始化参数

    canfd_init(); // 根据eeprom参数初始化canfd
    
    stimer_init(&stimer_main);
    stimer_addTask(&stimer_main, 0, 1, 0, KickDog);
    stimer_addTask(&stimer_main, 1, 1, 0, SystemLeve05msMotor);
    stimer_addTask(&stimer_main, 2, 1, 0, SystemLeve05msFunction);
    stimer_addTask(&stimer_main, 3, 4, 0, SystemLeve2msMotor);
    stimer_addTask(&stimer_main, 4, 4, 2, SystemLeve2msFunction);
    stimer_addTask(&stimer_main, 5, 1, 0, COM_CAN_loop);


    
    while(1)
    {
        stimer_loop(&stimer_main);
    }
}

/***************************************************************
    取代EPWM的ADC 中断，使用下溢中断
    1：清除中断标志，可以使下一个中断进来
    2：中断嵌套，设置可以被其他中断打断（比如SCI 等执行时间非常短的中断）
    3：执行FOC 算法
    4：关中断
    5：Acknowledge this interrupt

    2&4 是可以选择项目，供设计师参考！！！
****************************************************************/
#pragma CODE_SECTION(ZeroOfEPWMISR, "ramfuncs");
interrupt void ZeroOfEPWMISR(void)
{
    // 电流环计算
    EPwm2Regs.ETCLR.bit.INT = 1;
    EINT;
    AngleCal();
    ADCOverInterrupt();
    DINT;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3; // Acknowledge this interrupt

    // 向stimer模块提供心跳
    static Uint16 stimer_main_cnt = 0;
    if(++stimer_main_cnt >= 10)
    {
        stimer_main_cnt = 0;
        stimer_heartbeat(&stimer_main); // 2Khz的心跳频率
    }
}

/***************************************************************
    EPWM的过流中断
         对硬件过流信号处理：

    注意：
    逻辑是：先封锁（硬件已经完成，大概200NS），然后进该中断（执行报错，清除标志位等），
    也可以屏蔽此中断，在外面采用查询的方式，不影响保护
****************************************************************/
#pragma CODE_SECTION(OneShotTZOfEPWMISR, "ramfuncs");
interrupt void OneShotTZOfEPWMISR(void)
{
    HardWareErrorDeal();                    // 处理硬件故障－电机控制模块处理
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2; // Acknowledge this interrupt
}

