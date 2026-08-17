/**
  ******************************************************************************
  * @file    spi.h
  * @brief   SPI2 全双工主机驱动接口
  *          引脚: PB13(SCK) / PB14(MISO) / PB15(MOSI)，软件片选 PB12
  *          用途: 当前发送 0xAA 做示波器验证；后续用于读写 XV7001BB 陀螺仪
  ******************************************************************************
  */

#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

/* 片选电平控制(软件 NSS，PB12) --------------------------------------------*/
void SPI2_CS_Low(void);
void SPI2_CS_High(void);

/**
  * @brief  初始化 SPI2 为全双工主机模式并配置相关 GPIO
  * @retval HAL_OK 成功，其它为失败
  */
HAL_StatusTypeDef SPI2_Init(void);

/**
  * @brief  SPI2 全双工收发(同时发送与接收 len 个字节)
  * @param  pTxData 发送缓冲区，可为 NULL(此时发送 0xFF 占位)
  * @param  pRxData 接收缓冲区，可为 NULL(此时丢弃接收数据)
  * @param  len     字节数
  * @note   本函数不控制片选，调用者需自行 SPI2_CS_Low/High 包裹一次事务
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef SPI2_TransmitReceive(const uint8_t *pTxData, uint8_t *pRxData, uint16_t len);

/**
  * @brief  发送单个字节(自动包含片选拉低/拉高)，用于 0xAA 示波器测试
  * @param  data 要发送的字节
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef SPI2_SendByte(uint8_t data);

/**
  * @brief  发送一个字节并返回同时接收到的字节(自动包含片选)
  * @param  data 要发送的字节
  * @retval 接收到的字节
  */
uint8_t SPI2_ReadWriteByte(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
