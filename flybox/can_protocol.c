/**
  ******************************************************************************
  * @file    can_protocol.c
  * @brief   CAN 协议层实现: 帧打包与命令解析
  *
  *         帧格式说明:
  *           0x200 (DLC=5, 50Hz, 上位机已解码):
  *             Byte 0~1 : 角度×100×4   (int16, 大端, 单位 0.01°)
  *             Byte 2~3 : 角速度×100   (int16, 大端, 单位 0.01°/s)
  *             Byte 4   : 温度         (int8,  °C)
  *
  *           0x201 (DLC=6, 50Hz, 诊断帧):
  *             Byte 0~1 : 原始角速度×100       (int16, 大端, 去零偏前, 0.01°/s)
  *             Byte 2~3 : 原始角度增量×100×4   (int16, 大端, 未滤波积分, 0.01°)
  *             Byte 4~5 : 当前零偏×100         (int16, 大端, 0.01°/s)
  ******************************************************************************
  */

#include "can_protocol.h"
#include "can.h"

/* ======================== 发送帧打包 ======================== */

/**
  * @brief  发送处理后数据帧 (0x200)
  *
  *         对应原 imu.c 中帧 0x200 的打包逻辑:
  *           angleCenti = avgAngle * 100 * 4
  *           rateCenti  = avgRate * 100
  *           tempInt    = tempDegC
  *           大端拼接: [angleH, angleL, rateH, rateL, temp]
  */
void CAN_Proto_SendData(const IMU_Engine_Output_t *pOutput, float tempDegC)
{
    if (pOutput == NULL) return;

    /* 缩放并转换为整数
     * 注意: pOutput->angleDeg 已经是真实角度 (IMU_Engine 内部已 ×angleScale),
     *       此处只需 ×100 转为 0.01° 单位 */
    int16_t angleCenti = (int16_t)(pOutput->angleDeg * 100.0f);
    int16_t rateCenti  = (int16_t)(pOutput->rateDps * 100.0f);
    int8_t  tempInt    = (int8_t)tempDegC;

    /* 大端拼接 */
    uint8_t canData[5];
    canData[0] = (uint8_t)(angleCenti >> 8);
    canData[1] = (uint8_t)(angleCenti & 0xFF);
    canData[2] = (uint8_t)(rateCenti >> 8);
    canData[3] = (uint8_t)(rateCenti & 0xFF);
    canData[4] = (uint8_t)tempInt;

    CAN_SendFrame(CAN_TX_DATA_ID, canData, 5U);
}

/**
  * @brief  发送诊断数据帧 (0x201)
  *
  *         对应原 imu.c 中帧 0x201 的打包逻辑:
  *           rawRateCenti  = avgRawRate * 100
  *           rawAngleCenti = avgRawAngle * 100 * 4
  *           biasCenti     = biasDps * 100
  *           大端拼接: [rawRateH, rawRateL, rawAngleH, rawAngleL, biasH, biasL]
  */
void CAN_Proto_SendDiag(const IMU_Engine_Output_t *pOutput, float biasDps)
{
    if (pOutput == NULL) return;

    /* 缩放并转换为整数 */
    int16_t rawRateCenti  = (int16_t)(pOutput->rawRateDps * 100.0f);
    int16_t rawAngleCenti = (int16_t)(pOutput->rawAngleInc * 100.0f * 4.0f);
    int16_t biasCenti     = (int16_t)(biasDps * 100.0f);

    /* 大端拼接 */
    uint8_t canData[6];
    canData[0] = (uint8_t)(rawRateCenti >> 8);
    canData[1] = (uint8_t)(rawRateCenti & 0xFF);
    canData[2] = (uint8_t)(rawAngleCenti >> 8);
    canData[3] = (uint8_t)(rawAngleCenti & 0xFF);
    canData[4] = (uint8_t)(biasCenti >> 8);
    canData[5] = (uint8_t)(biasCenti & 0xFF);

    CAN_SendFrame(CAN_TX_DIAG_ID, canData, 6U);
}

/* ======================== 接收命令解析 ======================== */

/**
  * @brief  解析接收到的 CAN 命令帧
  *
  *         对应原 imu.c Task_CanRx 中的命令判断逻辑:
  *           0x85 → 重新校准
  *           0x90 → 角度置零
  *
  *         返回命令枚举，由调用者决定如何处理（设置标志或直接调用引擎函数）
  */
CAN_Command_t CAN_Proto_HandleRx(uint32_t stdId)
{
    if (stdId == CAN_CMD_RECALIBRATE_ID)
    {
        return CAN_CMD_RECALIBRATE;
    }
    else if (stdId == CAN_CMD_ANGLE_ZERO_ID)
    {
        return CAN_CMD_ANGLE_ZERO;
    }

    return CAN_CMD_NONE;
}
