/**
  ******************************************************************************
  * @file    sxh485_h200.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SXH485_H200_H__
#define __SXH485_H200_H__

#ifdef __cplusplus
 extern "C" {
#endif

#define SXH485_H200_UART                    USART2
#define SXH485_H200_UART_TX_BUFF_SIZE       (32)
#define SXH485_H200_UART_RX_BUFF_SIZE       (1024)

typedef enum
{
    SXH485_H200_OFF,
    SXH485_H200_ON,
} sxh485_h200_status;

typedef enum
{
    SXH485_H200_OK,
    SXH485_H200_FAIL,
} sxh485_h200_ops_status;

typedef enum
{
    RESOLUTION_320_240 = 3,
    RESOLUTION_640_480 = 5,
    RESOLUTION_1280_960 = 6,
    RESOLUTION_800_600 = 7,
    RESOLUTION_1024_768 = 8,
    RESOLUTION_1280_800 = 9,
    RESOLUTION_1280_720 = 15,
    RESOLUTION_1920_1080 = 16,
    RESOLUTION_1280_1024 = 17,
} sxh485_h200_picture_resolution;

typedef enum
{
    SXH485_H200_RECEIVING,
    SXH485_H200_RECV_OK,
    SXH485_H200_RECV_ERROR,
    SXH485_H200_RECV_CRC_ERROR,
    SXH485_H200_RECV_TIMEOUT,
} sxh485_h200_recv_status;

extern uint8_t g_arr_rx_buf[SXH485_H200_UART_RX_BUFF_SIZE];
extern uint16_t g_u16_rx_buf_thread_size;

void sxh485_h200_init(void);
void sxh485_h200_timer_stop(void);
sxh485_h200_ops_status sxh485_h200_check(sxh485_h200_status* status, uint32_t timeout);
sxh485_h200_ops_status sxh485_h200_capture(sxh485_h200_picture_resolution resolution, uint8_t compress_ratio, uint32_t* size, uint32_t timeout);
sxh485_h200_ops_status sxh485_h200_pic_get(uint32_t start_addr, uint32_t size, uint8_t* buf, uint32_t timeout);
void sxh485_h200_recv_flag_set(sxh485_h200_recv_status recv_status);
void sxh485_h200_frame_handle(void);


#ifdef __cplusplus
}
#endif

#endif /* __SXH485_H200_H__ */
