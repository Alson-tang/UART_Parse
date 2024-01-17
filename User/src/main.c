/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.6.0
  * @date    20-September-2021
  * @brief   Main program body
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
#include "stm32f10x.h"

#include "printf.h"
#include "delay.h"
#include "sxh485_h200.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
#define PIC_PKG_SIZE    (512)

static uint8_t pic_buf[PIC_PKG_SIZE] = { 0 };

/**
  * @brief  Configures the nested vectored interrupt controller.
  * @param  None
  * @retval None
  */
void NVIC_Configuration(void)
{
    /* Configure the NVIC Preemption Priority Bits */  
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
}

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
    /*!< At this stage the microcontroller clock setting is already configured, 
        this is done through SystemInit() function which is called from startup
        file (startup_stm32f10x_xx.s) before to branch to application main.
        To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f10x.c file
        */

    /* Add your application code here
        */
    sxh485_h200_status status = SXH485_H200_OFF;
    sxh485_h200_ops_status opt_status = SXH485_H200_OK;
    uint32_t pic_size = 0;
    uint32_t start_addr = 0;
    uint32_t recv_pic_size = 0;
    uint32_t pkg_size = PIC_PKG_SIZE;

    NVIC_Configuration();
    SysTick_Config(SystemCoreClock / 1000);

    debug_uart_init();
    printf("Debug UART output success\r\n");

    sxh485_h200_init();
    sxh485_h200_check(&status, 2);
    if (status == SXH485_H200_ON) {
        printf("SXH485 H200 ON\r\n");
    } else {
        printf("SXH485 H200 OFF\r\n");
        while (1);
    }

    opt_status = sxh485_h200_capture(RESOLUTION_640_480, 1, &pic_size, 5);
    if (opt_status == SXH485_H200_OK) {
        printf("Picture size is : %d B, %d KB\r\n", pic_size, pic_size >> 10);
    } else {
        printf("SXH485 H200 Capture error\r\n");
        while (1);
    }

    do {
        if ((recv_pic_size + pkg_size) > pic_size) {
            pkg_size = pic_size - recv_pic_size;
        } else {
            pkg_size = PIC_PKG_SIZE;
        }

        opt_status = sxh485_h200_pic_get(start_addr, pkg_size, pic_buf, 3);
        if (opt_status != SXH485_H200_OK) {
            printf("SXH485 H200 Get picture data error\r\n");
            break;
        }

        for (uint16_t i = 1; i <= pkg_size; i++) {
            printf("0X%02X ", pic_buf[i - 1]);

            if ((i % 32) == 0) {
                printf("\r\n");
            }
        }

        start_addr += pkg_size;
        recv_pic_size += pkg_size;
    } while (recv_pic_size < pic_size);


    /* Infinite loop */
    while (1);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
        ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif
