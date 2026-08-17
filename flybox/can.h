/**
  ******************************************************************************
  * @file    can.h
  * @brief   CAN1 驱动接口，波特率 1Mbps
  *          引脚: PA11(RX) / PA12(TX)
  *          支持 正常模式 / 回环模式(loopback) 切换，接收走 FIFO0 中断
  ******************************************************************************
  */

#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

/* CAN 帧数据结构(接收/发送通用) -------------------------------------------*/
typedef struct
{
    uint32_t StdId;      /* 标准帧 ID */
    uint8_t  DLC;        /* 数据长度 0~8 */
    uint8_t  Data[8];    /* 数据 */
    uint8_t  NewFlag;    /* 1 = 收到新帧(供轮询/watch 查看)，读取后应清 0 */
} CAN_Frame_t;

/* 最近一次接收到的帧，供接收任务读取 / 调试 watch 查看 --------------------*/
extern volatile CAN_Frame_t g_can_rx_frame;

/* 默认发送帧参数 -----------------------------------------------------------*/
#define CAN_TX_STD_ID       0x123U

/**
  * @brief  初始化 CAN1(波特率 1Mbps)并启动，使能 FIFO0 接收中断
  * @param  mode 工作模式:
  *              CAN_MODE_NORMAL          正常模式(接 USB-CAN 工具)
  *              CAN_MODE_LOOPBACK        回环模式(无需收发器，自收自发验证)
  *              CAN_MODE_SILENT          静默
  *              CAN_MODE_SILENT_LOOPBACK 静默回环
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef CAN_Init(uint32_t mode);

/**
  * @brief  发送一个标准数据帧
  * @param  stdId 标准帧 ID
  * @param  pData 数据指针
  * @param  len   数据长度 0~8
  * @retval HAL_OK 已放入发送邮箱
  */
HAL_StatusTypeDef CAN_SendFrame(uint32_t stdId, const uint8_t *pData, uint8_t len);

/**
  * @brief  发送单个字节(ID 使用 CAN_TX_STD_ID)，用于 0xAA 测试
  * @param  data 要发送的字节
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef CAN_SendByte(uint8_t data);

/**
  * @brief  收到新帧时的用户回调(在中断上下文调用，__weak 默认空实现)
  *         应用层可重写此函数，用于从 ISR 释放信号量唤醒接收任务
  */
void CAN_OnFrameReceived(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
