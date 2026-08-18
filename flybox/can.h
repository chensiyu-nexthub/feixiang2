/**
  ******************************************************************************
  * @file    can.h
  * @brief   CAN 总线驱动接口 (STM32F103 CAN1, 1Mbps)
  *
  *          依赖: STM32F1xx HAL
  ******************************************************************************
  */

#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

/* ======================== 接收帧结构 ======================== */
typedef struct
{
    uint32_t stdId;
    uint8_t  dlc;
    uint8_t  data[8];
    uint8_t  newFlag;   /* 1 = 新帧可用, 由中断置位, 任务消费后清零 */
} CAN_RxFrame_t;

/* 全局接收帧 (中断写入, 任务读取) */
extern volatile CAN_RxFrame_t g_can_rx_frame;

/* ======================== 接口函数 ======================== */

/**
  * @brief  初始化 CAN1: 1Mbps, FIFO0 接收中断, 过滤器全通
  * @param  mode  CAN_MODE_NORMAL 或 CAN_MODE_LOOPBACK
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef CAN_Init(uint8_t mode);

/**
  * @brief  发送一个标准数据帧 (阻塞, 等待邮箱空闲)
  * @param  stdId  标准帧 ID (11bit)
  * @param  pData  数据指针 (可为 NULL 当 len=0)
  * @param  len    数据长度 (0~8)
  * @retval HAL_OK 成功, HAL_BUSY 邮箱满, HAL_ERROR 参数错误
  */
HAL_StatusTypeDef CAN_SendFrame(uint32_t stdId, const uint8_t *pData, uint8_t len);

/**
  * @brief  从全局接收帧取出数据 (任务上下文调用)
  * @param  pFrame  输出缓冲区
  * @retval 1 = 有新帧, 0 = 无新帧
  */
uint8_t CAN_GetRxFrame(CAN_RxFrame_t *pFrame);

/**
  * @brief  CAN 收到新帧回调 (__weak, 中断上下文, 应用层可重写)
  *         默认空实现, 应用层重写此函数 (如释放信号量唤醒接收任务)
  * @param  stdId  帧 ID
  * @param  pData  帧数据 (8 字节)
  */
void CAN_OnFrameReceived(uint32_t stdId, const uint8_t *pData);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */