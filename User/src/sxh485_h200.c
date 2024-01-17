/**
  ******************************************************************************
  * @file    sxh485_h200.c
  * @author  Alson Tang
  * @version V1.0.0
  * @date    17-January-2024
  * @brief   This file contains all the functions prototypes for SXH485 H200
  *          Modules.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  * Change History
  *  <Date>     | <Version> | <Author>       | <Description>
  *  2024/01/24 | 1.0.0     | Alson.Tang     | Create file
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "stm32f10x.h"

#include "delay.h"
#include "crc.h"
#include "sxh485_h200.h"

#define SXH485_H200_UART_TX                         GPIO_Pin_2
#define SXH485_H200_UART_RX                         GPIO_Pin_3
#define SXH485_H200_UART_PORT                       GPIOA
#define SXH485_H200_UART_RCC_PORT                   RCC_APB2Periph_GPIOA
#define SXH485_H200_UART_RCC_CLK                    RCC_APB1Periph_USART2
#define SXH485_H200_UART_IRQn                       USART2_IRQn

#define SXH485_H200_TIMER                           TIM6
#define SXH485_H200_TIMER_IRQN                      TIM6_IRQn
#define SXH485_H200_TIMER_RCC_CLK                   RCC_APB1Periph_TIM6

#define SXH485_H200_FRAME_HEADER_1                  (0x90)
#define SXH485_H200_FRAME_HEADER_2                  (0xEB)

#define SXH485_H200_FRAME_CMD_TEST                  (0x01)
#define SXH485_H200_FRAME_CMD_CAPTURE               (0x40)
#define SXH485_H200_FRAME_CMD_PIC_GET               (0x48)
#define SXH485_H200_FRAME_CMD_PIC_GET_RECV          (0x49)

#define SXH485_H200_FRAME_CRC1                      (0xC1)
#define SXH485_H200_FRAME_CRC2                      (0xC2)

#define SXH485_H200_FRAME_CMD_TEST_LEN_L            (0x02)
#define SXH485_H200_FRAME_CMD_TEST_LEN_H            (0x00)
#define SXH485_H200_FRAME_CMD_TEST_DATA1            (0x55)
#define SXH485_H200_FRAME_CMD_TEST_DATA2            (0xAA)

#define SXH485_H200_FRAME_CMD_CAPTURE_LEN_L         (0x04)
#define SXH485_H200_FRAME_CMD_CAPTURE_LEN_H         (0x00)
#define SXH485_H200_FRAME_CMD_CAPTURE_PKG_L         (0x00)
#define SXH485_H200_FRAME_CMD_CAPTURE_PKG_H         (0x02)

#define SXH485_H200_FRAME_CMD_PIC_GET_LEN_L         (0x06)
#define SXH485_H200_FRAME_CMD_PIC_GET_LEN_H         (0x00)

#define SXH485_H200_FRAME_HEADER_1_INDEX            (0)
#define SXH485_H200_FRAME_HEADER_2_INDEX            (1)
#define SXH485_H200_FRAME_ADDR_INDEX                (2)
#define SXH485_H200_FRAME_CMD_INDEX                 (3)
#define SXH485_H200_FRAME_LEN_L_INDEX               (4)
#define SXH485_H200_FRAME_LEN_H_INDEX               (5)
#define SXH485_H200_FRAME_DATA_INDEX                (6)

typedef enum
{
    SXH485_H200_FRAME_HEADER_1_STATUS,
    SXH485_H200_FRAME_HEADER_2_STATUS,
    SXH485_H200_FRAME_ADDR_STATUS,
    SXH485_H200_FRAME_CMD_STATUS,
    SXH485_H200_FRAME_LEN_L_STATUS,
    SXH485_H200_FRAME_LEN_H_STATUS,
    SXH485_H200_FRAME_DATA_STATUS,
    SXH485_H200_FRAME_CRC_L_STATUS,
    SXH485_H200_FRAME_CRC_H_STATUS,
    SXH485_H200_FRAME_MAX_STATUS,
} sxh485_h200_frame_status;

typedef struct
{
    uint32_t size;
    sxh485_h200_picture_resolution resolution;
    uint8_t compress_ratio;
} sxh485_h200_picture_info_t;

static USART_InitTypeDef s_st_sxh485_h200_uart = { 0 };
static TIM_TimeBaseInitTypeDef  s_st_sxh485_h200_timer = { 0 };
static uint8_t s_u8_addr = 0x01;
static uint8_t s_u8_cmd = SXH485_H200_FRAME_CMD_TEST;
static sxh485_h200_frame_status s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_HEADER_1_STATUS;
static sxh485_h200_status g_st_sxh485_h200_status = SXH485_H200_OFF;
static sxh485_h200_picture_info_t s_st_sxh485_h200_pic_info = { 0 };
static uint8_t g_arr_tx_buf[SXH485_H200_UART_TX_BUFF_SIZE] = { 0 };
static sxh485_h200_recv_status s_st_recv_flag = SXH485_H200_RECV_OK;

uint8_t g_arr_rx_buf[SXH485_H200_UART_RX_BUFF_SIZE] = { 0 };
uint16_t g_u16_rx_buf_thread_size = SXH485_H200_UART_RX_BUFF_SIZE;

/**
  * @brief  Configures UART.
  * @param  USARTx: Specifies the UART to be configured. 
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None.
  */
static void sxh485_h200_uart_cfg(USART_TypeDef* USARTx, USART_InitTypeDef* USART_InitStruct)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(SXH485_H200_UART_RCC_PORT | RCC_APB2Periph_AFIO, ENABLE);

    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(SXH485_H200_UART_RCC_CLK, ENABLE);

    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = SXH485_H200_UART_TX;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SXH485_H200_UART_PORT, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = SXH485_H200_UART_RX;
    GPIO_Init(SXH485_H200_UART_PORT, &GPIO_InitStructure);

    /* USART configuration */
    USART_Init(SXH485_H200_UART, USART_InitStruct);

    /* Enable USARTy Receive and Transmit interrupts */
    USART_ITConfig(SXH485_H200_UART, USART_IT_RXNE, ENABLE);
    USART_ITConfig(SXH485_H200_UART, USART_IT_IDLE, ENABLE);

    /* Enable USART */
    USART_Cmd(SXH485_H200_UART, ENABLE);

    /* Read from the USART_SR register followed by a write to the USART_DR register to clear TC flag */
    USART_GetFlagStatus(SXH485_H200_UART, USART_FLAG_TC);
}

/**
  * @brief  Init USART buff.
  * @param  None.
  * @retval None.
  */
static void sxh485_h200_buff_init(void)
{
    g_u16_rx_buf_thread_size = 0;

    memset(g_arr_tx_buf, 0, SXH485_H200_UART_TX_BUFF_SIZE);
    memset(g_arr_tx_buf, 0, SXH485_H200_UART_RX_BUFF_SIZE);
}

/**
  * @brief  Init test frame.
  * @param  addr: Device Address. 
  * @param  frame_buf: Pointer to frame buff.
  * @retval Frame buff size.
  */
static uint8_t sxh485_h200_test_frame_init(uint8_t addr, uint8_t* frame_buf)
{
    uint8_t index = 0;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_HEADER_1;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_HEADER_2;

    g_arr_tx_buf[index++] = addr;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_TEST;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_TEST_LEN_L;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_TEST_LEN_H;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_TEST_DATA1;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_TEST_DATA2;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CRC1;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CRC2;

    return index;
}

/**
  * @brief  Init capture frame.
  * @param  addr: Device Address.
  * @param  resolution: Picture resolution.
  *   This parameter can be a value of @ref sxh485_h200_picture_resolution.
  * @param  compress_ratio: Picture compression ratio. Range: 1 ~ 5.
  * @note
  *   - The smaller the number, the clearer the picture, but the size of the picture is approximately.
  * @param  frame_buf: Pointer to frame buff.
  * @retval Frame buff size.
  */
static uint8_t sxh485_h200_capture_frame_init(uint8_t addr, sxh485_h200_picture_resolution resolution, uint8_t compress_ratio, uint8_t* frame_buf)
{
    uint8_t index = 0;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_HEADER_1;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_HEADER_2;

    g_arr_tx_buf[index++] = addr;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_CAPTURE;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_CAPTURE_LEN_L;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_CAPTURE_LEN_H;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_CAPTURE_PKG_L;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_CAPTURE_PKG_H;
    g_arr_tx_buf[index++] = resolution;
    g_arr_tx_buf[index++] = compress_ratio;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CRC1;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CRC2;

    return index;
}

/**
  * @brief  Init get capture frame.
  * @param  start_addr: Start address for getting picture data.
  * @param  size: Size for getting picture data.
  * @retval Frame buff size.
  */
static uint8_t sxh485_h200_pic_get_frame_init(uint8_t addr, uint32_t start_addr, uint32_t size)
{
    uint8_t index = 0;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_HEADER_1;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_HEADER_2;

    g_arr_tx_buf[index++] = addr;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_PIC_GET;

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_PIC_GET_LEN_L;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CMD_PIC_GET_LEN_H;

    g_arr_tx_buf[index++] = ((start_addr >> 0) & 0xFF);
    g_arr_tx_buf[index++] = ((start_addr >> 8) & 0xFF);
    g_arr_tx_buf[index++] = ((start_addr >> 16) & 0xFF);
    g_arr_tx_buf[index++] = ((start_addr >> 24) & 0xFF);

    g_arr_tx_buf[index++] = ((size >> 0) & 0xFF);
    g_arr_tx_buf[index++] = ((size >> 8) & 0xFF);

    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CRC1;
    g_arr_tx_buf[index++] = SXH485_H200_FRAME_CRC2;

    return index;
}

/**
  * @brief  Configures the nested vectored interrupt controller for USART.
  * @param  None.
  * @retval None.
  */
static void sxh485_h200_uart_nvic_cfg(void)
{ 
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    /* Enable the USARTy Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = SXH485_H200_UART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Configures the nested vectored interrupt controller for timer.
  * @param  None.
  * @retval None.
  */
static void sxh485_h200_timer_nvic_cfg(void)
{
    NVIC_InitTypeDef NVIC_InitStructure = { 0 };

    /* Enable the TIM6 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = SXH485_H200_TIMER_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Test command callback
  * @param  buf: Pointing to receive buff.
  * @retval None.
  */
static void sxh485_h200_cmd_test_handle(uint8_t* buf)
{
    if ((buf[0] == 0x00) && (buf[1] == 0xAA) && (buf[2] == 0x55)) {
        g_st_sxh485_h200_status = SXH485_H200_ON;
    } else {
        g_st_sxh485_h200_status = SXH485_H200_OFF;
    }
}

/**
  * @brief  Capture command callback
  * @param  buff: Pointing to receive buff.
  * @retval None.
  */
static void sxh485_h200_cmd_capture_handle(uint8_t* buf)
{
    memset(&s_st_sxh485_h200_pic_info, 0, sizeof(sxh485_h200_picture_info_t));

    s_st_sxh485_h200_pic_info.size = ((uint32_t)buf[1] + (((uint32_t)buf[2]) << 8) + (((uint32_t)buf[3]) << 16));
    s_st_sxh485_h200_pic_info.resolution = (sxh485_h200_picture_resolution)buf[9];
    s_st_sxh485_h200_pic_info.compress_ratio = buf[10];
}

/**
  * @brief  Command callback
  * @param  cmd: Command.
  * @param  buff: Pointing to receive buff.
  * @retval None.
  */
static void sxh485_h200_cmd_handle(uint8_t cmd, uint8_t* buf)
{
    switch (cmd) {
        case SXH485_H200_FRAME_CMD_TEST: {
            sxh485_h200_cmd_test_handle(buf);
            
            break;
        }

        case SXH485_H200_FRAME_CMD_CAPTURE: {
            sxh485_h200_cmd_capture_handle(buf);

            break;
        }

        case SXH485_H200_FRAME_CMD_PIC_GET: {
            break;
        }

        default:
            break;
    }
}

/**
  * @brief  Init timer.
  * @param  None.
  * @retval None.
  */
static void sxh485_h200_timer_init(void)
{
    RCC_APB1PeriphClockCmd(SXH485_H200_TIMER_RCC_CLK, ENABLE);
}

/**
  * @brief  Configure timeout time.
  * @param  timeout: Timeout. Unit in seconds.
  * @retval None.
  */
static void sxh485_h200_timer_cfg(uint32_t timeout)
{
    TIM_DeInit(SXH485_H200_TIMER);

    memset(&s_st_sxh485_h200_timer, 0, sizeof(TIM_TimeBaseInitTypeDef));
    s_st_sxh485_h200_timer.TIM_Period = (timeout * 10000);
    s_st_sxh485_h200_timer.TIM_Prescaler = 7200 - 1;

    TIM_TimeBaseInit(SXH485_H200_TIMER, &s_st_sxh485_h200_timer);
    TIM_ClearFlag(SXH485_H200_TIMER, TIM_FLAG_Update);
    TIM_ITConfig(SXH485_H200_TIMER, TIM_IT_Update, ENABLE);
    TIM_Cmd(SXH485_H200_TIMER, ENABLE);
}

/**
  * @brief  Get recvive flag.
  * @param  None.
  * @retval Receive flag.
  *   This parameter can be a value of @ref sxh485_h200_recv_status.
  */
static sxh485_h200_recv_status sxh485_h200_recv_flag_get(void)
{
    return s_st_recv_flag;
}

/**
  * @brief  Send frame.
  * @param  buf: Frame buff.
  * @param  len: Frame buff size.
  * @retval None.
  */
static void sxh485_h200_send(uint8_t* buf, uint32_t len)
{
    USART_ITConfig(SXH485_H200_UART, USART_IT_RXNE, ENABLE);
    USART_ITConfig(SXH485_H200_UART, USART_IT_IDLE, ENABLE);

    for (uint8_t i = 0; i < len; i++) {
        USART_SendData(SXH485_H200_UART, buf[i]);

        /* Loop until the end of transmission */
        while (USART_GetFlagStatus(SXH485_H200_UART, USART_FLAG_TC) == RESET);
    } 
}

/**
  * @brief  Init SXH485 H200
  * @param  None.
  * @retval None.
  */
void sxh485_h200_init(void)
{
    s_st_sxh485_h200_uart.USART_BaudRate = 115200;
    s_st_sxh485_h200_uart.USART_WordLength = USART_WordLength_8b;
    s_st_sxh485_h200_uart.USART_StopBits = USART_StopBits_1;
    s_st_sxh485_h200_uart.USART_Parity = USART_Parity_No;
    s_st_sxh485_h200_uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    s_st_sxh485_h200_uart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    sxh485_h200_uart_cfg(SXH485_H200_UART, &s_st_sxh485_h200_uart);
    sxh485_h200_uart_nvic_cfg();

    sxh485_h200_timer_init();
    sxh485_h200_timer_nvic_cfg();
}

/**
  * @brief  Stop timer.
  * @param  None.
  * @retval None.
  */
void sxh485_h200_timer_stop(void)
{
    TIM_Cmd(SXH485_H200_TIMER, DISABLE);
}

/**
  * @brief  Check SXH485 H200 status
  * @param  status: Pointer to SXH485 H200 status
  * @param  timeout: Timeout. Unit in seconds.
  * @retval SXH485 H200 operation status.
  *   This parameter can be a value of @ref sxh485_h200_ops_status.
  */
sxh485_h200_ops_status sxh485_h200_check(sxh485_h200_status* status, uint32_t timeout)
{
    uint8_t send_size = 0;
    sxh485_h200_recv_status recv_status = SXH485_H200_RECEIVING;

    s_u8_cmd = SXH485_H200_FRAME_CMD_TEST;

    sxh485_h200_buff_init();
    send_size = sxh485_h200_test_frame_init(s_u8_addr, g_arr_tx_buf);

    sxh485_h200_send(g_arr_tx_buf, send_size);

    sxh485_h200_recv_flag_set(SXH485_H200_RECEIVING);

    sxh485_h200_timer_cfg(timeout);

    recv_status = sxh485_h200_recv_flag_get();
    while (recv_status == SXH485_H200_RECEIVING) {
        recv_status = sxh485_h200_recv_flag_get();
    }

    if (recv_status == SXH485_H200_RECV_OK) {
        *status = g_st_sxh485_h200_status;
        return SXH485_H200_OK;
    } else {
        return SXH485_H200_FAIL;
    }
}

/**
  * @brief  Capture picture.
  * @param  status: Pointer to SXH485 H200 status
  * @param  resolution: Picture resolution.
  *   This parameter can be a value of @ref sxh485_h200_picture_resolution.
  * @param  compress_ratio: Picture compression ratio. Range: 1 ~ 5.
  * @note
  *   - The smaller the number, the clearer the picture, but the size of the picture is approximately.
  * @param  size: Pointer to picture size.
  * @param  timeout: Timeout. Unit in seconds.
  * @retval SXH485 H200 operation status.
  *   This parameter can be a value of @ref sxh485_h200_ops_status.
  */
sxh485_h200_ops_status sxh485_h200_capture(sxh485_h200_picture_resolution resolution, uint8_t compress_ratio, uint32_t* size, uint32_t timeout)
{
    uint8_t send_size = 0;
    sxh485_h200_recv_status recv_status = SXH485_H200_RECEIVING;

    s_u8_cmd = SXH485_H200_FRAME_CMD_CAPTURE;

    sxh485_h200_buff_init();
    send_size = sxh485_h200_capture_frame_init(s_u8_addr, resolution, compress_ratio, g_arr_tx_buf);

    sxh485_h200_send(g_arr_tx_buf, send_size);

    sxh485_h200_recv_flag_set(SXH485_H200_RECEIVING);

    sxh485_h200_timer_cfg(timeout);

    recv_status = sxh485_h200_recv_flag_get();
    while (recv_status == SXH485_H200_RECEIVING) {
        recv_status = sxh485_h200_recv_flag_get();
    }

    if (recv_status == SXH485_H200_RECV_OK) {
        *size = s_st_sxh485_h200_pic_info.size;
        return SXH485_H200_OK;
    } else {
        return SXH485_H200_FAIL;
    }
}

/**
  * @brief  Get picture data.
  * @param  start_addr: Start address for getting picture data.
  * @param  size: Size for getting picture data.
  * @param  size: Point to the buff used for receiving.
  * @param  timeout: Timeout. Unit in seconds.
  * @retval SXH485 H200 operation status.
  *   This parameter can be a value of @ref sxh485_h200_ops_status.
  */
sxh485_h200_ops_status sxh485_h200_pic_get(uint32_t start_addr, uint32_t size, uint8_t* buf, uint32_t timeout)
{
    uint8_t send_size = 0;
    sxh485_h200_recv_status recv_status = SXH485_H200_RECEIVING;

    s_u8_cmd = SXH485_H200_FRAME_CMD_PIC_GET;

    sxh485_h200_buff_init();
    send_size = sxh485_h200_pic_get_frame_init(s_u8_addr, start_addr, size);

    sxh485_h200_send(g_arr_tx_buf, send_size);

    sxh485_h200_recv_flag_set(SXH485_H200_RECEIVING);

    sxh485_h200_timer_cfg(timeout);

    recv_status = sxh485_h200_recv_flag_get();
    while (recv_status == SXH485_H200_RECEIVING) {
        recv_status = sxh485_h200_recv_flag_get();
    }

    if (recv_status == SXH485_H200_RECV_OK) {
        memcpy(buf, &g_arr_rx_buf[SXH485_H200_FRAME_DATA_INDEX], size);
        return SXH485_H200_OK;
    } else {
        return SXH485_H200_FAIL;
    }
}

/**
  * @brief  Set recvive flag.
  * @param  sxh485_h200_recv_status: Receive flag.
  *   This parameter can be a value of @ref sxh485_h200_recv_status.
  * @retval None.
  */
void sxh485_h200_recv_flag_set(sxh485_h200_recv_status recv_status)
{
    s_st_recv_flag = recv_status;
}

/**
  * @brief  Command handle.
  * @retval None.
  * @retval None.
  */
void sxh485_h200_frame_handle(void)
{
    uint16_t len = 0;
    uint16_t recv_crc = 0;
    uint16_t calc_crc = 0;

    while (1) {
        switch (s_st_sxh485_h200_frame_status) {
            case SXH485_H200_FRAME_HEADER_1_STATUS: {
                if (g_arr_rx_buf[SXH485_H200_FRAME_HEADER_1_INDEX] != SXH485_H200_FRAME_HEADER_1) {
                    s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_HEADER_1_STATUS;
                    sxh485_h200_recv_flag_set(SXH485_H200_RECV_ERROR);

                    return;
                }

                s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_HEADER_2_STATUS;

                break;
            }

            case SXH485_H200_FRAME_HEADER_2_STATUS: {
                if (g_arr_rx_buf[SXH485_H200_FRAME_HEADER_2_INDEX] != SXH485_H200_FRAME_HEADER_2) {
                    s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_HEADER_1_STATUS;
                    sxh485_h200_recv_flag_set(SXH485_H200_RECV_ERROR);

                    return;
                }

                s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_ADDR_STATUS;

                break;
            }

            case SXH485_H200_FRAME_ADDR_STATUS: {
                if (g_arr_rx_buf[SXH485_H200_FRAME_ADDR_INDEX] != s_u8_addr) {
                    s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_HEADER_1_STATUS;
                    sxh485_h200_recv_flag_set(SXH485_H200_RECV_ERROR);

                    return;
                }

                s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_CMD_STATUS;

                break;
            }

            case SXH485_H200_FRAME_CMD_STATUS: {
                if (g_arr_rx_buf[SXH485_H200_FRAME_CMD_INDEX] != s_u8_cmd) {
                    if ((g_arr_rx_buf[SXH485_H200_FRAME_CMD_INDEX] == SXH485_H200_FRAME_CMD_PIC_GET_RECV) &&
                        (SXH485_H200_FRAME_CMD_PIC_GET == s_u8_cmd)) {
                        s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_LEN_L_STATUS;
                    } else {
                        s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_HEADER_1_STATUS;
                        sxh485_h200_recv_flag_set(SXH485_H200_RECV_ERROR);

                        return;
                    }

                } else {
                    s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_LEN_L_STATUS;
                }

                break;
            }

            case SXH485_H200_FRAME_LEN_L_STATUS: {
                len = 0;
                len = g_arr_rx_buf[SXH485_H200_FRAME_LEN_L_INDEX];

                s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_LEN_H_STATUS;

                break;
            }

            case SXH485_H200_FRAME_LEN_H_STATUS: {
                uint16_t len_h = g_arr_rx_buf[SXH485_H200_FRAME_LEN_H_INDEX];

                len |= (len_h << 8);

                s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_DATA_STATUS;

                break;
            }

            case SXH485_H200_FRAME_DATA_STATUS: {
                s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_CRC_L_STATUS;

                break;
            }

            case SXH485_H200_FRAME_CRC_L_STATUS: {
                recv_crc = 0;
                recv_crc = g_arr_rx_buf[SXH485_H200_FRAME_DATA_INDEX + len];

                s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_CRC_H_STATUS;

                break;
            }

            case SXH485_H200_FRAME_CRC_H_STATUS: {
                uint16_t recv_crc_h = g_arr_rx_buf[SXH485_H200_FRAME_DATA_INDEX + len + 1];

                recv_crc |= (recv_crc_h << 8);

                calc_crc = crc16(&g_arr_rx_buf[SXH485_H200_FRAME_ADDR_INDEX], len + 4);
                if (recv_crc != calc_crc) {
                    sxh485_h200_recv_flag_set(SXH485_H200_RECV_CRC_ERROR);
                } else {
                    sxh485_h200_cmd_handle(s_u8_cmd, &g_arr_rx_buf[SXH485_H200_FRAME_DATA_INDEX]);
                    sxh485_h200_recv_flag_set(SXH485_H200_RECV_OK);
                }

                s_st_sxh485_h200_frame_status = SXH485_H200_FRAME_HEADER_1_STATUS;

                return;
            }

            default:
                break;
        }
    }
}
