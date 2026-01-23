#include "led.h"

const lpg_seg_t led_off[] = {{1, 0}, {1, 1}};

#pragma CODE_SECTION(led_on_,"ramfuncs");
static void led_on_1(void)
{
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
}

#pragma CODE_SECTION(led_off_,"ramfuncs");
static void led_off_1(void)
{
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
}

void led_init(void)
{
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;  
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioDataRegs.GPBSET.bit.GPIO34 = 1; // 上电不亮
    EDIS;

    lpg_init(&lpg); // 10ms心跳周期
    lpg_register(&lpg, led_off, 2, led_on_1, led_off_1); // 注册一个led灯并设置初始的序列
}
