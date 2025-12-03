#include "canfd.h"

ringbuffer_t canFrameRxRingbuffer  = {0}; // 接收ringbuffer控制块
uint16_t canFrameRxBuffer[1024]    = {0}; // 接收ringbuffer缓冲区
ringbuffer_t canFrameTxRingbuffer  = {0}; // 发送ringbuffer控制块
uint16_t canFrameTxBuffer[1024]    = {0}; // 发送ringbuffer缓冲区 

/**
 * @brief 初始化canfd协议需要的环形缓冲区
 */
void canfd_ringbuffer_init(void)
{
    ringbuffer_init(&canFrameRxRingbuffer, canFrameRxBuffer, 1024);
    ringbuffer_init(&canFrameTxRingbuffer, canFrameTxBuffer, 1024);
}

/**
 * @brief 配置接收滤波器 (只匹配11位标准ID的低7位)
 * @param id1 需要匹配的ID的低7位数值 (0x00 ~ 0x7F)
 * @param id2 需要匹配的ID的低7位数值 (0x00 ~ 0x7F)
 */
static void canfd_config_filter_low7_dual(uint16_t id1, uint16_t id2)
{
    //CanfdRegs.CFG_STAT.bit.RESET = 1;

    CanfdRegs.CIA_ACF_CFG.bit.ACFADR = 0; // 配置滤波器0
    CanfdRegs.CIA_ACF_CFG.bit.SELMASK = 1;// 切换到配置掩码
    CanfdRegs.ACF_0.all = 0xFF80;         // 配置掩码为低7位        
    CanfdRegs.CIA_ACF_CFG.bit.SELMASK = 0;// 配置掩码匹配值
    CanfdRegs.ACF_0.all = (id1 & 0x007F); // 配置低7位的掩码匹配值为id1
    CanfdRegs.ACF_EN.bit.AE_0 = 1;        // 使能滤波器0

    CanfdRegs.CIA_ACF_CFG.bit.ACFADR = 1; // 配置滤波器1
    CanfdRegs.CIA_ACF_CFG.bit.SELMASK = 1;// 切换到配置掩码
    CanfdRegs.ACF_0.all = 0xFF80;         // 配置掩码为低7位        
    CanfdRegs.CIA_ACF_CFG.bit.SELMASK = 0;// 配置掩码匹配值
    CanfdRegs.ACF_0.all = (id2 & 0x007F); // 配置低7位的掩码匹配值为id2
    CanfdRegs.ACF_EN.bit.AE_1 = 1;        // 使能滤波器1

    //CanfdRegs.CFG_STAT.bit.RESET = 0;
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

    CanfdRegs.CANCFG.bit.CAN_CLK_SEL = 1;   // 0:sysclk;1:pllclk

    // Arbitration Phase：1M
    CanfdRegs.S_CFG.bit.S_PRESC = 4;
    CanfdRegs.S_SEG.bit.S_Seg_1 = 13;
    CanfdRegs.S_SEG.bit.S_Seg_2 = 4;
    CanfdRegs.S_CFG.bit.S_SJW   = 2;

    // Data Phase：5M
    CanfdRegs.F_CFG.bit.F_PRESC = 0;
    CanfdRegs.F_SEG.bit.F_Seg_1 = 13;
    CanfdRegs.F_SEG.bit.F_Seg_2 = 4;
    CanfdRegs.F_CFG.bit.F_SJW   = 1;

    CanfdRegs.DELAY_EALCAP.bit.TDCEN = 1;//fifo传输时置1
    CanfdRegs.DELAY_EALCAP.bit.SSPOFF = 0;
    //canfd_config_filter_low7_dual(0, 1);

    CanfdRegs.CFG_STAT.bit.RESET = 0;

    //  TBUF1-15
    CanfdRegs.TCTRL.bit.TSMODE = 0;
    CanfdRegs.CFG_STAT.bit.TBSEL = 1;
    CanfdRegs.CFG_STAT.bit.TPE = 0;         // no transfer to PTB
    //  CanfdRegs.CFG_STAT.bit.TSSS = 1;
   // CanfdRegs.RTINTFE.bit.TSIE  = 1;


    CanfdRegs.TBUF.RID1.bit.ID = 0;
    CanfdRegs.TBUF.RID1.bit.ESI = 0;

    CanfdRegs.TBUF.RIDST.bit.IDE = 0;        // 0: standard frame ID; 1: extended frame ID
    CanfdRegs.TBUF.RIDST.bit.RTR = 0;        // 0: data frame; 1: remote frame(for can)
    CanfdRegs.TBUF.RIDST.bit.FDF_EDL = 1;    // 0: can 2.0; 1:can_fd

        CanfdRegs.TBUF.RIDST.bit.BRS  = 1;


    CanfdRegs.TBUF.RIDST.bit.KOER = 0;

    //CanfdRegs.RTINTFE.bit.TPIE = 0;         // enable interrupt

        CanfdRegs.TBUF.RIDST.bit.IDE = 0;        // 0: standard frame ID; 1: extended frame ID
    CanfdRegs.TBUF.RIDST.bit.RTR = 0;        // 0: data frame; 1: remote frame(for can)

    CanfdRegs.TBUF.RIDST.bit.FDF_EDL = 1;    // 0: can 2.0; 1:can_fd
    CanfdRegs.TBUF.RIDST.bit.BRS = 1;



    CanfdRegs.RBUF.RID1.bit.ID  = 0;
    CanfdRegs.RBUF.RID1.bit.ESI = 0;

    CanfdRegs.RBUF.RIDST.bit.IDE     = 0 ;        // 0: standard frame ID; 1: extended frame ID
    CanfdRegs.RBUF.RIDST.bit.RTR     = 0;        // 0: data frame; 1: remote frame(for can)
    CanfdRegs.RBUF.RIDST.bit.FDF_EDL = 1;    // 0: can 2.0; 1:can_fd
    CanfdRegs.RBUF.RIDST.bit.BRS     = 1;        //1 canfd加速
    CanfdRegs.RBUF.RIDST.bit.KOER    = 0;

    // CanfdRegs.CFG_STAT.bit.RESET = 1; 

    // // clk
    // CanfdRegs.CANCFG.bit.CAN_CLK_SEL = 0;   // 0:sysclk 1:pllclk

    // // Arbitration Phase：1M
    // CanfdRegs.S_CFG.bit.S_PRESC = 4;  
    // CanfdRegs.S_SEG.bit.S_Seg_1 = 13; 
    // CanfdRegs.S_SEG.bit.S_Seg_2 = 4;  
    // CanfdRegs.S_CFG.bit.S_SJW   = 2; 

    // // Data Phase：5M
    // CanfdRegs.F_CFG.bit.F_PRESC = 0; 
    // CanfdRegs.F_SEG.bit.F_Seg_1 = 13;     
    // CanfdRegs.F_SEG.bit.F_Seg_2 = 4;    
    // CanfdRegs.F_CFG.bit.F_SJW   = 1;     

    // // TDC
    // CanfdRegs.DELAY_EALCAP.bit.TDCEN = 1;
    // CanfdRegs.DELAY_EALCAP.bit.SSPOFF = 0;

    // CanfdRegs.CFG_STAT.bit.RESET = 0;
    
    // // 中断配置
     CanfdRegs.RTINTFE.all = 0;
     CanfdRegs.RTINTFE.bit.RIE = 1; // 开启接收缓冲区非空中断

    // // config
    // CanfdRegs.TCTRL.bit.TSMODE = 0;   // 0 = FIFO mode
    // CanfdRegs.CFG_STAT.bit.TBSEL = 1; // 配置为二级缓冲区
    // CanfdRegs.CFG_STAT.bit.TPE = 0;
    // CanfdRegs.RTINTFE.bit.TSIE  = 1;

    // CanfdRegs.TBUF.RIDST.bit.IDE = 0;        // 0: standard frame ID; 1: extended frame ID
    // CanfdRegs.TBUF.RIDST.bit.RTR = 0;        // 0: data frame; 1: remote frame(for can)
    // CanfdRegs.TBUF.RIDST.bit.FDF_EDL = 1;    // 0: can 2.0; 1:can_fd
    // CanfdRegs.TBUF.RIDST.bit.BRS = 1;        // 1：开启发送canfd加速模式

    // CanfdRegs.RBUF.RIDST.bit.KOER = 0;

    // CanfdRegs.RBUF.RIDST.bit.IDE = 0;        // 0: standard frame ID; 1: extended frame ID
    // CanfdRegs.RBUF.RIDST.bit.RTR = 0;        // 0: data frame; 1: remote frame(for can)
    // CanfdRegs.RBUF.RIDST.bit.FDF_EDL = 1;    // 0: can 2.0; 1:can_fd
    // CanfdRegs.RBUF.RIDST.bit.BRS = 1;        // 1：开启接收canfd加速模式

    // CanfdRegs.RBUF.RIDST.bit.KOER = 0;

    
    canfd_ringbuffer_init();   
}

uint16_t t_cnt = 0,r_cnt = 0;

/**
 * @brief 发送一帧数据
 */
#pragma CODE_SECTION(sendCanFrame_fifo, "ramfuncs");
static inline void sendCanFrame_fifo(canFrame_t *canFrame)
{
    CanfdRegs.TBUF.RID0.bit.ID0 = canFrame->id; // 写入canid
    CanfdRegs.TBUF.RIDST.bit.DLC = CANFD_LEN_TO_DLC(canFrame->len); // 获取帧数据的dlc
    memcpy(CanfdRegs.TBUF.DATA, canFrame->data, ((canFrame->len + 1) >> 1)); // 复制实际的数据段数据到临时缓冲区

    CanfdRegs.TCTRL.bit.TSNEXT = 1;
    CanfdRegs.CFG_STAT.bit.TPE = 0;
    CanfdRegs.CFG_STAT.bit.TSONE = 1;
    CanfdRegs.CFG_STAT.bit.TSALL = 0;
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
    if(CanfdRegs.RTINTFE.bit.RIF)
    {
        CanfdRegs.RTINTFE.bit.RIF = 1;
        while(CanfdRegs.TCTRL.bit.RSTAT != 0)
        {
            if(ringbuffer_avail(&canFrameRxRingbuffer) >= sizeof(canFrame_t)) // 接收缓冲区还有空余的帧空间
            {
                canFrame_t canFrame_temp = {0};
                canFrame_temp.id = CanfdRegs.RBUF.RID0.bit.ID0; // 解析canid
                canFrame_temp.len = CANFD_DLC_TO_LEN(CanfdRegs.RBUF.RIDST.bit.DLC); // 解析数据段实际的字节数
                memcpy(canFrame_temp.data, CanfdRegs.RBUF.DATA, ((canFrame_temp.len + 1) >> 1)); // 复制实际的数据段数据到临时缓冲区
                ringbuffer_in(&canFrameRxRingbuffer, &canFrame_temp, sizeof(canFrame_t)); // 将帧压入接收帧环形缓冲区
            }

            CanfdRegs.TCTRL.bit.RREL = 1; // 释放一个槽位
            r_cnt++;
        }
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}

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
 * @note 在stimer框架下面2khz执行
 */
#pragma CODE_SECTION(COM_CAN_loop, "ramfuncs");
void COM_CAN_loop(void)
{
    canFrame_t canFrame_temp = {0};

    // RX loop
    for(uint16_t i = 0; i < 3; i++)
    {
        if(ringbuffer_used(&canFrameRxRingbuffer) < sizeof(canFrame_t)) break;
        ringbuffer_out(&canFrameRxRingbuffer, &canFrame_temp, sizeof(canFrame_t));
        parse_frame(&canFrame_temp);
    }

    // TX loop
    for(uint16_t i = 0; i < 3; i++)
    {
        if(ringbuffer_used(&canFrameTxRingbuffer) < sizeof(canFrame_t)) break;
        ringbuffer_out(&canFrameTxRingbuffer, &canFrame_temp, sizeof(canFrame_t));
        sendCanFrame_fifo(&canFrame_temp);
    }

//    #define MSG_ID_HEARTBEAT    0x700
//    // Send heartbeat
//    canFrame_t canFrame_temp;
//    canFrame_temp.id = MSG_ID_HEARTBEAT + node_id;
//    canFrame_temp.len = 1;
//    canFrame_temp.data[0] = 0x05;
//    enqueue_tx_frame(&canFrame_temp);
}

