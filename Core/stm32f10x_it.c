/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.6.0
  * @date    20-September-2021
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2011 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x_it.h"

#include "delay.h"
#include "sxh485_h200.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    if (g_u32_time != 0) {
        g_u32_time--;
    }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles USARTy global interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(SXH485_H200_UART, USART_IT_RXNE) != RESET) {
        USART_ClearITPendingBit(SXH485_H200_UART, USART_IT_RXNE);

        /* Read one byte from the receive data register */
        g_arr_rx_buf[g_u16_rx_buf_thread_size++] = USART_ReceiveData(SXH485_H200_UART);

        if (g_u16_rx_buf_thread_size == SXH485_H200_UART_RX_BUFF_SIZE) {
            /* Disable the USARTy Receive interrupt */
            USART_ITConfig(SXH485_H200_UART, USART_IT_RXNE, DISABLE);
        }
    } else if (USART_GetITStatus(SXH485_H200_UART, USART_IT_IDLE) != RESET) {
        /* Clear IDLE bit */
        USART_ReceiveData(SXH485_H200_UART);
        USART_ClearITPendingBit(SXH485_H200_UART, USART_IT_IDLE);

        USART_ITConfig(SXH485_H200_UART, USART_IT_RXNE, DISABLE);
        USART_ITConfig(SXH485_H200_UART, USART_IT_IDLE, DISABLE);

        sxh485_h200_timer_stop();
        sxh485_h200_frame_handle();
    }
}

/**
  * @brief  This function handles TIMx global interrupt request.
  * @param  None
  * @retval None
  */
void TIM6_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);

        USART_ITConfig(SXH485_H200_UART, USART_IT_RXNE, DISABLE);
        USART_ITConfig(SXH485_H200_UART, USART_IT_IDLE, DISABLE);

        USART_ClearITPendingBit(SXH485_H200_UART, USART_IT_RXNE);
        USART_ReceiveData(SXH485_H200_UART);
        USART_ClearITPendingBit(SXH485_H200_UART, USART_IT_IDLE);

        sxh485_h200_timer_stop();
        sxh485_h200_recv_flag_set(SXH485_H200_RECV_TIMEOUT);
    }
}

/**
  * @}
  */ 


