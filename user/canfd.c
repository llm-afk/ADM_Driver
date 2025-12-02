#include "canfd.h"

ringbuffer_t canFrameRxRingbuffer  = {0}; // 接收ringbuffer控制块
uint16_t canFrameRxBuffer[512]     = {0}; // 接收ringbuffer缓冲区
ringbuffer_t canFrameTxRingbuffer  = {0}; // 发送ringbuffer控制块
uint16_t canFrameTxBuffer[512]     = {0}; // 发送ringbuffer缓冲区 

/**
 * @brief 初始化canfd协议需要的环形缓冲区
 */
void canfd_ringbuffer_init(void)
{
    ringbuffer_init(&canFrameRxRingbuffer, canFrameRxBuffer, 512);
    ringbuffer_init(&canFrameTxRingbuffer, canFrameTxBuffer, 512);
}

/**
 * @brief 配置canfd外设
 */
void canfd_init(void)
{
    EALLOW;
#if 0
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 1;      // CANFD_Rx
    GpioCtrlRegs.GPAPUD.bit.GPIO29 = 1;     // CANFD_Tx
    AnalogRegs.PIN_MUX_FLASH.bit.canfd_gpio_mux = 0xA;   //GPIO0 CANFDRXD ; GPIO32 CANFDTXD
#else
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;      // CANFD_Rx
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 1;     // CANFD_Tx
    AnalogRegs.PIN_MUX_FLASH.bit.canfd_gpio_mux = 0x5;   //GPIO0 CANFDRXD ; GPIO32 CANFDTXD
#endif
    EDIS;
    
    CanfdRegs.CFG_STAT.bit.RESET = 1;
    CanfdRegs.CANCFG.bit.CAN_CLK_SEL = 0;   // 0:sysclk 1:pllclk

    // Arbitration Phase：500K
    CanfdRegs.S_CFG.bit.S_PRESC = 4;  
    CanfdRegs.S_SEG.bit.S_Seg_1 = 13; 
    CanfdRegs.S_SEG.bit.S_Seg_2 = 4;  
    CanfdRegs.S_CFG.bit.S_SJW   = 2; 

    // Data Phase：2M
    CanfdRegs.F_CFG.bit.F_PRESC = 0; 
    CanfdRegs.F_SEG.bit.F_Seg_1 = 13;     
    CanfdRegs.F_SEG.bit.F_Seg_2 = 4;    
    CanfdRegs.F_CFG.bit.F_SJW   = 1;     

    CanfdRegs.CFG_STAT.bit.RESET = 0;

    CanfdRegs.CFG_STAT.bit.TBSEL = 0;
    CanfdRegs.CFG_STAT.bit.TPE = 0;
    CanfdRegs.CFG_STAT.bit.TSONE = 0;
    CanfdRegs.CFG_STAT.bit.TSALL = 0;
    //CanfdRegs.CFG_STAT.bit.LBME = 1;      //外回环使能
    //CanfdRegs.CFG_STAT.bit.LBMI = 1;      // 0:disable internal loopback mode ; 1: enable internal loopback mode

    CanfdRegs.TBUF.RID0.bit.ID0 = 0x123;    // standard frame ID
    CanfdRegs.TBUF.RID0.bit.ID1 = 0x0;
    CanfdRegs.TBUF.RID1.bit.ID = 0;
    CanfdRegs.TBUF.RID1.bit.ESI = 0;

    CanfdRegs.TBUF.RIDST.bit.IDE = 0;        // 0: standard frame ID; 1: extended frame ID
    CanfdRegs.TBUF.RIDST.bit.RTR = 0;        // 0: data frame; 1: remote frame(for can)
    CanfdRegs.TBUF.RIDST.bit.FDF_EDL = 1;    // 0: can 2.0; 1:can_fd
    CanfdRegs.TBUF.RIDST.bit.BRS = 1;        // 1：开启发送canfd加速模式

    CanfdRegs.TBUF.RIDST.bit.KOER = 0;

    CanfdRegs.RBUF.RID0.bit.ID0 = 0x15A;     // standard frame ID
    CanfdRegs.RBUF.RID0.bit.ID1 = 0x0;
    CanfdRegs.RBUF.RID1.bit.ID = 0;
    CanfdRegs.RBUF.RID1.bit.ESI = 0;

    CanfdRegs.RBUF.RIDST.bit.IDE = 0;        // 0: standard frame ID; 1: extended frame ID
    CanfdRegs.RBUF.RIDST.bit.RTR = 0;        // 0: data frame; 1: remote frame(for can)
    CanfdRegs.RBUF.RIDST.bit.FDF_EDL = 1;    // 0: can 2.0; 1:can_fd
    CanfdRegs.RBUF.RIDST.bit.BRS = 1;        // 1：开启接收canfd加速模式

    CanfdRegs.RBUF.RIDST.bit.KOER = 0;

    canfd_ringbuffer_init();
}

uint16_t t_cnt = 0,r_cnt = 0;

/**
 * @brief 发送一帧数据
 */
#pragma CODE_SECTION(sendCanFrame, "ramfuncs");
static inline void sendCanFrame(canFrame_t *canFrame)
{
    CanfdRegs.TBUF.RID0.bit.ID0 = canFrame->id; // 写入canid
    CanfdRegs.TBUF.RIDST.bit.DLC = CANFD_LEN_TO_DLC(canFrame->len); // 获取帧数据的dlc
    for(uint16_t i = 0; i < ((canFrame->len + 1) >> 1); i++) // 当奇数个数据的时候向上取整到偶数
    {
        CanfdRegs.TBUF.DATA[i] = canFrame->data[i]; // 复制实际的数据段数据到发送缓冲区
    }
    CanfdRegs.CFG_STAT.bit.TPE = 1; 
    t_cnt++;
}

/**
* @brief 将待发送的帧压入发送缓冲区
* @param frame 待发送的CAN帧指针
*/
#pragma CODE_SECTION(enqueue_tx_frame, "ramfuncs");
inline void enqueue_tx_frame(canFrame_t *frame)
{
    if(ringbuffer_avail(&canFrameTxRingbuffer) >= sizeof(canFrame_t)) // 发送缓冲区还有帧空间
    {
        ringbuffer_in(&canFrameTxRingbuffer, frame, sizeof(canFrame_t)); // 将帧压入发送帧环形缓冲区
    }
}

/**
 * @brief canfd接收总中断
 */
#pragma CODE_SECTION(canfd_IsrHander1, "ramfuncs");
interrupt void canfd_IsrHander1(void)
{
    if(CanfdRegs.RTINTFE.bit.RIF==1) // 接收中断
    {
        if(ringbuffer_avail(&canFrameRxRingbuffer) >= sizeof(canFrame_t)) // 接收缓冲区还有空余的帧空间
        {
            r_cnt++;
            canFrame_t canFrame_temp = {0};
            canFrame_temp.id = CanfdRegs.RBUF.RID0.bit.ID0; // 解析canid
            canFrame_temp.len = CANFD_DLC_TO_LEN(CanfdRegs.RBUF.RIDST.bit.DLC); // 解析数据段实际的字节数
            memcpy(canFrame_temp.data, CanfdRegs.RBUF.DATA, ((canFrame_temp.len + 1) >> 1)); // 复制实际的数据段数据到临时缓冲区
            ringbuffer_in(&canFrameRxRingbuffer, &canFrame_temp, sizeof(canFrame_t)); // 将帧压入接收帧环形缓冲区
        }
        
        CanfdRegs.TCTRL.bit.RREL = 1;
        CanfdRegs.RTINTFE.bit.RIF = 1;
    }
    if(CanfdRegs.RTINTFE.bit.TPIF == 1)
    {
        CanfdRegs.RTINTFE.bit.TPIF = 1;
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

uint16_t node_id = 1;

/**
 * @brief CAN 数据帧解析函数
 * @param frame 待解析的 CAN 数据帧
 */
#pragma CODE_SECTION(parse_frame, "ramfuncs");
static void parse_frame(canFrame_t *frame)
{
    enqueue_tx_frame(frame);
    // uint16_t msg_id = GET_MSG_ID(frame->id);
    // uint16_t node_id = GET_NODE_ID(frame->id);
    // if(node_id == 0) // 软件过滤需要的node_id的帧数据
    // {
    //     switch(msg_id)
    //     {
    //         case 0:
    //         {

    //             break;   
    //         }
    //         default:
    //         {
    //             break;       
    //         }
    //     }
    // }
}

/**
 * @brief can协议通信loop
 * @note 脱离stimer框架直接在main的主loop中不断循环执行
 */
#pragma CODE_SECTION(COM_CAN_loop, "ramfuncs");
void COM_CAN_loop(void)
{
    // RX loop
    if(ringbuffer_used(&canFrameRxRingbuffer) >= sizeof(canFrame_t))
    {
        canFrame_t canFrame_temp = {0};
        ringbuffer_out(&canFrameRxRingbuffer, &canFrame_temp, sizeof(canFrame_t)); 
        parse_frame(&canFrame_temp); // 解析can帧数据
    }
        
    // TX loop
    if(ringbuffer_used(&canFrameTxRingbuffer) >= sizeof(canFrame_t))
    {
        canFrame_t canFrame_temp = {0};
        ringbuffer_out(&canFrameTxRingbuffer, &canFrame_temp, sizeof(canFrame_t)); 
        sendCanFrame(&canFrame_temp); // 发送帧数据
    }

//    #define MSG_ID_HEARTBEAT    0x700
//    // Send heartbeat
//    canFrame_t canFrame_temp;
//    canFrame_temp.id = MSG_ID_HEARTBEAT + node_id;
//    canFrame_temp.len = 1;
//    canFrame_temp.data[0] = 0x05;
//    enqueue_tx_frame(&canFrame_temp);
}

