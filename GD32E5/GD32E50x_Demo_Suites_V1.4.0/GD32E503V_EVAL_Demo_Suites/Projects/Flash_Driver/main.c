/*!
    \file    main.c
    \brief   SPI Flash demo

    \version 2024-01-09, V1.4.0, demo for GD32E50x
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/



#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "gd32e50x.h"
#include "systick.h"
#include "gd25qxx.h"
#include "gd32e503v_eval.h"



//uint8_t txbuffer1[] = &quot;\n\rUSART DMA transmit example\n\r&quot;

//int fputc(int ch,FILE *f);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/

//void UART0_Init_Test(){
//	
//    gd_eval_com_init(EVAL_COM0);
//}



//uint8_t txbuffer1 []={1,2,3,4,5,6,7,7,7,7,7,7};
//#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))

// void DMA0_Init_Test(){

//     /*DMA初始化*/
//     dma_parameter_struct dma_init_struct;
//     // 时钟开启
//     rcu_periph_clock_enable(RCU_DMA0);
//     dma_deinit(DMA0, DMA_CH3);//dma寄存器初始化
//     dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;//传输模式，存储到外设（发送）
//     dma_init_struct.memory_addr = (uint32_t)txbuffer1;//dma内存地址
//     dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE; //内存地址增量模式
//     dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;//dma外设宽度8位
//     dma_init_struct.number = ARRAYNUM(txbuffer1)-1; //长度
//     dma_init_struct.periph_addr =((uint32_t)(&USART_DATA(USART0)));//外设基地址( (uint32_t)USART_DATA(USART0) )
//     dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;//外设地址增量禁用
//     dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
//     dma_init_struct.priority = DMA_PRIORITY_HIGH; //优先级高
//     dma_init(DMA0, DMA_CH3 , &dma_init_struct);
//	 
//	 
//		

//     /* configure DMA mode */
//     dma_circulation_disable(DMA0, DMA_CH3);//循环模式禁用
//     dma_memory_to_memory_disable(DMA0, DMA_CH3);//通道3   USART0_TX
//		 
//		 
//		 
//	 
//	 
//		
//	 
//		dma_channel_enable(DMA0, DMA_CH3);
//		usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE);
//		//dma_channel_enable(DMA0, DMA_CH3);
// }

#define UART_TX_BUF_SIZE   512          // 发送环形缓冲区大小
#define UART_RX_BUF_SIZE   256          // 单次最大接收帧长度

typedef struct{
    uint8_t  buf[UART_TX_BUF_SIZE];
    volatile uint16_t head;            // 写指针
    volatile uint16_t tail;            // 读指针
}RingBuffer_t;

static RingBuffer_t tx_ring;
static uint8_t  rx_dma_buf[UART_RX_BUF_SIZE];
static uint8_t  rx_user_buf[UART_RX_BUF_SIZE];
static volatile uint16_t rx_len = 0;   // 最新一帧长度
static volatile uint8_t  rx_frame_done = 0;


/* 把数据写进环形缓冲区，返回实际写入字节数 */
static uint16_t ring_write(uint8_t *src, uint16_t len)
{
    uint16_t space, cnt;
    uint16_t tail = tx_ring.tail;
    space = UART_TX_BUF_SIZE - (tx_ring.head - tail);
    if(len > space) len = space;
    cnt = len;
    while(cnt--)
    {
        tx_ring.buf[tx_ring.head & (UART_TX_BUF_SIZE-1)] = *src++;
        tx_ring.head++;
    }
    return len;
}

/* 启动一次 DMA 发送：从 tail 开始搬 head-tail 字节 */
static void uart_start_tx_dma(void)
{
    if(dma_flag_get(DMA0, DMA_CH3, DMA_FLAG_G))
        return;                 // 正在跑，等它自己完成
    uint16_t len = tx_ring.head - tx_ring.tail;
    if(len == 0) return;

    dma_channel_disable(DMA0, DMA_CH3);
    dma_memory_address_config(DMA0, DMA_CH3,
        (uint32_t)&tx_ring.buf[tx_ring.tail & (UART_TX_BUF_SIZE-1)]);
    dma_transfer_number_config(DMA0, DMA_CH3, len);
    dma_channel_enable(DMA0, DMA_CH3);
}

/* 类似 printf 的接口，非阻塞，立即返回 */
void uprintf(const char *fmt, ...)
{
    static uint8_t tmp[128];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf((char *)tmp, sizeof(tmp), fmt, ap);
    va_end(ap);

    if(len > 0)
    {
        ring_write(tmp, len);

        /* 强制启动（如果 DMA 正在运行，先停再启亦可） */
        uart_start_tx_dma();   // 让 DMA 把当前缓冲区一次性搬完
    }
}
void uart_dma_init(void)
{
    /* 时钟 */
    rcu_periph_clock_enable(RCU_USART0);
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_DMA0);

    /* GPIO */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);   // TX
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10); // RX

    /* USART 参数 */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

    /* ---------------- TX DMA ---------------- */
    dma_parameter_struct dma_tx;
    dma_deinit(DMA0, DMA_CH3);
    dma_tx.periph_addr  = (uint32_t)&USART_DATA(USART0);
    dma_tx.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_tx.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_tx.memory_addr  = (uint32_t)tx_ring.buf;
    dma_tx.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_tx.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_tx.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_tx.number       = 0;
    dma_tx.priority     = DMA_PRIORITY_MEDIUM;
    dma_init(DMA0, DMA_CH3, &dma_tx);
    dma_interrupt_enable(DMA0, DMA_CH3, DMA_INT_FTF);
    nvic_irq_enable(DMA0_Channel3_IRQn, 2, 0);

    /* ---------------- RX DMA ---------------- */
    dma_parameter_struct dma_rx;
    dma_deinit(DMA0, DMA_CH4);
    dma_rx.periph_addr  = (uint32_t)&USART_DATA(USART0);
    dma_rx.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_rx.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_rx.memory_addr  = (uint32_t)rx_dma_buf;
    dma_rx.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_rx.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_rx.direction    = DMA_PERIPHERAL_TO_MEMORY;
    dma_rx.number       = UART_RX_BUF_SIZE;
    dma_rx.priority     = DMA_PRIORITY_HIGH;
    dma_init(DMA0, DMA_CH4, &dma_rx);
    dma_channel_enable(DMA0, DMA_CH4);

    /* USART IDLE 中断 */
    usart_interrupt_enable(USART0, USART_INT_IDLE);
    nvic_irq_enable(USART0_IRQn, 1, 0);
}



int main(void)
{
    systick_config();
    uart_dma_init();

    uprintf("System start!\r\n");

    while(1)
    {
        if(rx_frame_done)
        {
            // 回显
            uprintf("Recv %d bytes: ", rx_len);
            for(uint16_t i=0;i<rx_len;i++) uprintf("%02X ", rx_user_buf[i]);
            uprintf("\r\n");
            rx_frame_done = 0;
        }
    }
}


//int main(void)
//{
//    //SystemInit();
////		systick_config();
////    UART0_Init_Test();
////		DMA0_Init_Test();
//	
//		while(RESET == dma_flag_get(DMA0, DMA_CH3, DMA_INTF_FTFIF)){
//			
//    }


//    while(1){
//        //printf("hello world\n");
//        //delay_1ms(1000);
//    }

//}

/* DMA 发送完成中断：更新 tail 指针，如果还有数据继续搬 */
void DMA0_Channel3_IRQHandler(void)
{
    if(dma_interrupt_flag_get(DMA0, DMA_CH3, DMA_INT_FLAG_FTF))
    {
        dma_interrupt_flag_clear(DMA0, DMA_CH3, DMA_INT_FLAG_FTF);

        /* 更新读指针即可，不再次启动 DMA */
        tx_ring.tail += dma_transfer_number_get(DMA0, DMA_CH3);
    }
}

/* USART 总线空闲中断：一帧接收完成 */
void USART0_IRQHandler(void)
{
    if(usart_interrupt_flag_get(USART0, USART_INT_FLAG_IDLE))
    {
        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE); // GD32 清标志需读 SR+DR
        usart_data_receive(USART0);

        dma_channel_disable(DMA0, DMA_CH4);
        uint16_t remain = dma_transfer_number_get(DMA0, DMA_CH4);
        rx_len = UART_RX_BUF_SIZE - remain;

        /* 把数据搬到用户缓冲区并置位标志 */
        memcpy(rx_user_buf, rx_dma_buf, rx_len);
        rx_frame_done = 1;

        /* 重新启动 RX DMA */
        dma_transfer_number_config(DMA0, DMA_CH4, UART_RX_BUF_SIZE);
        dma_channel_enable(DMA0, DMA_CH4);
    }
}


/* retarget the C library printf function to the USART */
//int fputc(int ch, FILE *f)
//{
//    usart_data_transmit(EVAL_COM0,(uint8_t)ch);
//    while (RESET == usart_flag_get(EVAL_COM0, USART_FLAG_TC));
//    return ch;
//}

