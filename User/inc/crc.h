/**
  ******************************************************************************
  * @file    crc.h
  * @author  Alson Tang
  * @version V1.0.0
  * @date    17-January-2024
  * @brief   This file contains all the functions prototypes for the CRC 
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
#ifndef __CRC_H__
#define __CRC_H__

#ifdef __cplusplus
 extern "C" {
#endif

uint16_t crc16(uint8_t *buf,  uint32_t len);
uint16_t crc16_new(uint16_t old_crc, uint8_t* buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __CRC_H__ */
