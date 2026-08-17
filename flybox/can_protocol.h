/**
  ******************************************************************************
  * @file    can_protocol.h
  * @brief   CAN 协议层: 集中管理帧 ID、字节布局、缩放系数
  *          封装 0x200/0x201 发送帧和 0x85/0x90 接收命令解析
  ******************************************************************************
  */

#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>
#include "imu_core.h"

/* ======================== 帧 ID 定义 ======================== */

/* CAN 命令帧 ID (接收) */
#define CAN_CMD_RECALIBRATE_ID  0x85U    /* 重新零偏校准 (采样200次 + 清零角度) */
#define CAN_CMD_ANGLE_ZERO_ID   0x90U    /* 角度置零 (仅 angleDeg = 0) */

/* CAN 输出帧 ID (发送) */
#define CAN_TX_DATA_ID          0x200U   /* 处理后数据帧 (上位机已解码) */
#define CAN_TX_DIAG_ID          0x201U   /* 诊断数据帧 (原始数据+零偏) */


/* ======================== 命令枚举 ======================== */

/**
  * @brief  CAN 接收命令类型
  */
typedef enum
{
    CAN_CMD_NONE = 0,           /* 无命令 */
    CAN_CMD_RECALIBRATE,        /* 重新校准 */
    CAN_CMD_ANGLE_ZERO,         /* 角度置零 */
} CAN_Command_t;

/* ======================== 接口函数 ======================== */

/**
  * @brief  发送处理后数据帧 (0x200)
  *         帧格式: 角度×100×4 (int16) + 角速度×100 (int16) + 温度 (int8)
  * @param  pOutput   IMU 引擎输出结构体
  * @param  tempDegC  温度 (°C)
  */
void CAN_Proto_SendData(const IMU_Engine_Output_t *pOutput, float tempDegC);

/**
  * @brief  发送诊断数据帧 (0x201)
  *         帧格式: 原始角速度×100 (int16) + 原始角度增量×100×4 (int16) + 零偏×100 (int16)
  * @param  pOutput  IMU 引擎输出结构体
  * @param  biasDps  当前零偏 (°/s)
  */
void CAN_Proto_SendDiag(const IMU_Engine_Output_t *pOutput, float biasDps);

/**
  * @brief  解析接收到的 CAN 命令帧
  * @param  stdId  标准帧 ID
  * @retval 命令类型 (CAN_CMD_NONE / CAN_CMD_RECALIBRATE / CAN_CMD_ANGLE_ZERO)
  */
CAN_Command_t CAN_Proto_HandleRx(uint32_t stdId);



#ifdef __cplusplus
}
#endif

#endif /* __CAN_PROTOCOL_H */
