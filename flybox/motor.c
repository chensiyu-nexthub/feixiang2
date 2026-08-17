/**
  ******************************************************************************
  * @file    motor.c
  * @brief   电机 CAN 通讯驱动实现 (V1.1)
  *
  *          基于 电机CAN通讯协议控制指南 (V1.1):
  *            - 所有位置/速度/加速度均以浮点 mm / mm/s / mm/s² 传入
  *            - 内部转换为协议要求的 0.1mm / 0.1mm/s 整数单位
  *            - int32 数据采用小端序打包 (Data[0]=低字节)
  ******************************************************************************
  */

#include "motor.h"
#include "can.h"
#include <cmsis_os.h>

/* ======================== 全局电机状态 ======================== */
volatile Motor_Status_t g_motor_status[MOTOR_MAX_COUNT] = {0};
volatile Motor_SNQueue_t g_motor_sn_queue = {0};

/* ======================== 内部辅助函数 ======================== */

/**
  * @brief  将 int32_t 打包为 4 字节小端序
  */
static void Motor_PackInt32(uint8_t *buf, int32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

/**
  * @brief  从 4 字节小端序解包 int32_t
  */
static int32_t Motor_UnpackInt32(const uint8_t *buf)
{
    return (int32_t)((uint32_t)buf[0]
                   | ((uint32_t)buf[1] << 8)
                   | ((uint32_t)buf[2] << 16)
                   | ((uint32_t)buf[3] << 24));
}

/**
  * @brief  从 2 字节小端序解包 int16_t
  */
static int16_t Motor_UnpackInt16(const uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

/* ======================== SN 获取与设备号设置 ======================== */

/**
  * @brief  主动获取电机 SN 号 (CAN ID 0x060D, 2字节)
  *         Data[0] = 设备类型, Data[1] = 0
  *         电机收到后通过 0x0312 回复 SN
  */
HAL_StatusTypeDef Motor_RequestSN(uint8_t deviceType)
{
    uint8_t data[2];
    data[0] = deviceType;
    data[1] = 0x00U;
    return CAN_SendFrame(MOTOR_ID_GET_SN, data, 2U);
}

/**
  * @brief  设置电机设备号 (CAN ID 0x0313, 8字节)
  *         Data[0~6] = SN, Data[7] = 设备号
  */
HAL_StatusTypeDef Motor_SetDeviceId(const uint8_t *sn, uint8_t deviceId)
{
    if (sn == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t data[8];
    for (uint8_t i = 0; i < MOTOR_SN_LEN; i++)
    {
        data[i] = sn[i];
    }
    data[7] = deviceId;
    return CAN_SendFrame(MOTOR_ID_SET_DEVICE_ID, data, 8U);
}

/**
  * @brief  等待电机上报 SN (阻塞, 从队列 pop 一个)
  */
HAL_StatusTypeDef Motor_WaitSN(uint8_t *sn, uint32_t timeout_ms)
{
    if (sn == NULL)
    {
        return HAL_ERROR;
    }

    uint32_t elapsed = 0U;
    const uint32_t pollInterval = 5U;

    while (g_motor_sn_queue.count == 0U)
    {
        if (elapsed >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
        osDelay(pollInterval);
        elapsed += pollInterval;
    }

    /* 取出第一个 SN */
    for (uint8_t i = 0; i < MOTOR_SN_LEN; i++)
    {
        sn[i] = g_motor_sn_queue.entries[0].sn[i];
    }

    /* 队列前移 */
    for (uint8_t j = 0; j < g_motor_sn_queue.count - 1U; j++)
    {
        g_motor_sn_queue.entries[j] = g_motor_sn_queue.entries[j + 1U];
    }
    g_motor_sn_queue.count--;

    return HAL_OK;
}

/**
  * @brief  等待 SN 队列中至少有 needCount 个 SN (阻塞)
  */
HAL_StatusTypeDef Motor_WaitSNQueue(uint8_t needCount, uint32_t timeout_ms)
{
    uint32_t elapsed = 0U;
    const uint32_t pollInterval = 5U;

    while (g_motor_sn_queue.count < needCount)
    {
        if (elapsed >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }
        osDelay(pollInterval);
        elapsed += pollInterval;
    }

    return HAL_OK;
}

/**
  * @brief  清空 SN 队列
  */
void Motor_ClearSNQueue(void)
{
    g_motor_sn_queue.count = 0U;
}

/**
  * @brief  获取指定设备号的电机状态指针
  * @param  deviceId  设备号 (1~MOTOR_MAX_COUNT)
  */
volatile Motor_Status_t *Motor_GetStatus(uint8_t deviceId)
{
    if (deviceId < 1U || deviceId > MOTOR_MAX_COUNT)
    {
        return NULL;
    }
    return &g_motor_status[deviceId - 1U];
}

/* ======================== 初始化配置 ======================== */

/**
  * @brief  配置电机参数 (CAN ID 0x0316, 8字节)
  *         Byte0: 0x19 (极对数等)
  *         Byte1: 0x0C (速度PI)
  *         Byte2: 0x03 (转矩PI)
  *         Byte3: 0x07 (电流限制)
  *         Byte4: 0x93 (刹车/阻尼)
  *         Byte5: 0x2D (D参数)
  *         Byte6: 方向(bit7) + 自动返回使能(bit5=0,bit4=0) + 马达号(bit[3:0])
  *         Byte7: 设备号
  */
HAL_StatusTypeDef Motor_ConfigParam(uint8_t deviceId, uint8_t motorId, uint8_t direction)
{
    uint8_t data[8];
    data[0] = MOTOR_CFG_BYTE0;
    data[1] = MOTOR_CFG_BYTE1;
    data[2] = MOTOR_CFG_BYTE2;
    data[3] = MOTOR_CFG_BYTE3;
    data[4] = MOTOR_CFG_BYTE4;
    data[5] = MOTOR_CFG_BYTE5;
    data[6] = (direction & 0x80U) | (motorId & 0x0FU);  /* bit7=方向, bit[3:0]=马达号 */
    data[7] = deviceId;
    return CAN_SendFrame(MOTOR_ID_CONFIG_PARAM, data, 8U);
}

/**
  * @brief  自定义配置电机参数 (CAN ID 0x0316, 8字节)
  *         允许为不同电机设置不同的 PID / 电流 / 阻尼参数
  */
HAL_StatusTypeDef Motor_ConfigParamCustom(uint8_t deviceId, uint8_t motorId, uint8_t direction,
                                          uint8_t byte0, uint8_t byte1, uint8_t byte2,
                                          uint8_t byte3, uint8_t byte4, uint8_t byte5)
{
    uint8_t data[8];
    data[0] = byte0;
    data[1] = byte1;
    data[2] = byte2;
    data[3] = byte3;
    data[4] = byte4;
    data[5] = byte5;
    data[6] = (direction & 0x80U) | (motorId & 0x0FU);
    data[7] = deviceId;
    return CAN_SendFrame(MOTOR_ID_CONFIG_PARAM, data, 8U);
}

/**
  * @brief  配置轮径 (重复 3 次)
  */
HAL_StatusTypeDef Motor_ConfigWheelDiameter(uint8_t deviceId, float wheelDiameter_mm)
{
    /* 轮径单位: 0.01mm (文档说 0.1mm 分辨率，但 Data[2~3] 为 0.01mm) */
    int32_t diameterValue = (int32_t)(wheelDiameter_mm * 100.0f);

    uint8_t data[4];
    data[0] = deviceId;
    data[1] = 0x24U;
    data[2] = (uint8_t)(diameterValue & 0xFF);
    data[3] = (uint8_t)((diameterValue >> 8) & 0xFF);

    HAL_StatusTypeDef status = HAL_OK;

    /* 重复 3 次写入 */
    for (uint8_t i = 0; i < 3; i++)
    {
        status = CAN_SendFrame(MOTOR_ID_QUERY, data, 4U);
        if (status != HAL_OK)
        {
            return status;
        }
        osDelay(10);
    }

    return HAL_OK;
}

/**
  * @brief  锁定电机 (上电)
  */
HAL_StatusTypeDef Motor_Lock(uint8_t deviceId)
{
    uint8_t data[5] = {0};
    data[0] = MOTOR_CMD_LOCK;
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_LOCK, data, 5U);
}

/**
  * @brief  释放电机 (下电)
  */
HAL_StatusTypeDef Motor_Unlock(uint8_t deviceId)
{
    uint8_t data[5] = {0};
    data[0] = MOTOR_CMD_UNLOCK;
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_LOCK, data, 5U);
}


/* ======================== 运动参数设置 ======================== */

/**
  * @brief  设置主速度
  */
HAL_StatusTypeDef Motor_SetSpeed(uint8_t deviceId, float speed_mm_s)
{
    int32_t speedValue = (int32_t)(speed_mm_s * 10.0f);  /* 0.1mm/s */

    uint8_t data[5];
    Motor_PackInt32(data, speedValue);
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_SPEED_MAIN, data, 5U);
}

/**
  * @brief  设置从速度
  */
HAL_StatusTypeDef Motor_SetSpeedSlave(uint8_t deviceId, float speed_mm_s)
{
    int32_t speedValue = (int32_t)(speed_mm_s * 10.0f);

    uint8_t data[5];
    Motor_PackInt32(data, speedValue);
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_SPEED_SLAVE, data, 5U);
}

/**
  * @brief  设置主加速度
  */
HAL_StatusTypeDef Motor_SetAcceleration(uint8_t deviceId, float accel_mm_s2)
{
    int32_t accelValue = (int32_t)(accel_mm_s2 * 10.0f);

    uint8_t data[5];
    Motor_PackInt32(data, accelValue);
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_ACCEL_MAIN, data, 5U);
}

/**
  * @brief  设置从加速度
  */
HAL_StatusTypeDef Motor_SetAccelerationSlave(uint8_t deviceId, float accel_mm_s2)
{
    int32_t accelValue = (int32_t)(accel_mm_s2 * 10.0f);

    uint8_t data[5];
    Motor_PackInt32(data, accelValue);
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_ACCEL_SLAVE, data, 5U);
}

/* ======================== 目标位置控制 ======================== */

/**
  * @brief  设置目标位置 (主通道, 6 字节)
  */
HAL_StatusTypeDef Motor_SetTarget(uint8_t deviceId, float position_mm, uint8_t targetType)
{
    int32_t targetValue = (int32_t)(position_mm * 10.0f);  /* 0.1mm */

    uint8_t data[6];
    Motor_PackInt32(data, targetValue);
    data[4] = deviceId;
    data[5] = targetType;
    return CAN_SendFrame(MOTOR_ID_TARGET_MAIN, data, 6U);
}

/**
  * @brief  设置目标位置 (从通道, 5 字节, 无目标类型)
  */
HAL_StatusTypeDef Motor_SetTargetSlave(uint8_t deviceId, float position_mm)
{
    int32_t targetValue = (int32_t)(position_mm * 10.0f);

    uint8_t data[5];
    Motor_PackInt32(data, targetValue);
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_TARGET_SLAVE, data, 5U);
}

/**
  * @brief  急停
  */
HAL_StatusTypeDef Motor_EmergencyStop(uint8_t deviceId)
{
    return Motor_SetTarget(deviceId, 0.0f, MOTOR_TARGET_EMERGENCY_STOP);
}

/**
  * @brief  将当前位置设为零点 (目标类型 0x0A)
  *         0x0A 语义是"把当前位置定义为零点", 不是"回到零点"
  */
HAL_StatusTypeDef Motor_SetZero(uint8_t deviceId)
{
    return Motor_SetTarget(deviceId, 0.0f, MOTOR_TARGET_ZERO);
}

/**
  * @brief  归零 (兼容别名): 等价于 Motor_SetZero
  */
HAL_StatusTypeDef Motor_Home(uint8_t deviceId)
{
    return Motor_SetZero(deviceId);
}

/**
  * @brief  清零力矩统计 (torquePeak/torqueMin/torqueMax)
  */
void Motor_ResetTorqueStats(uint8_t deviceId)
{
    volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
    if (p != NULL)
    {
        p->torquePeak = 0;
        p->torqueMin  = 0;
        p->torqueMax  = 0;
    }
}

/* ======================== 查询命令 ======================== */

/**
  * @brief  查询当前温度
  */
HAL_StatusTypeDef Motor_QueryTemperature(uint8_t deviceId)
{
    uint8_t data[2];
    data[0] = deviceId;
    data[1] = MOTOR_QUERY_TEMP;
    return CAN_SendFrame(MOTOR_ID_QUERY, data, 2U);
}

/**
  * @brief  查询当前位置
  */
HAL_StatusTypeDef Motor_QueryPosition(uint8_t deviceId)
{
    uint8_t data[2];
    data[0] = deviceId;
    data[1] = MOTOR_QUERY_POSITION;
    return CAN_SendFrame(MOTOR_ID_QUERY, data, 2U);
}

/**
  * @brief  查询状态
  */
HAL_StatusTypeDef Motor_QueryStatus(uint8_t deviceId)
{
    uint8_t data[2];
    data[0] = deviceId;
    data[1] = MOTOR_QUERY_STATUS;
    return CAN_SendFrame(MOTOR_ID_QUERY, data, 2U);
}

/**
  * @brief  查询固件版本号
  */
HAL_StatusTypeDef Motor_QueryVersion(uint8_t deviceId)
{
    uint8_t data[2];
    data[0] = deviceId;
    data[1] = MOTOR_QUERY_VERSION;
    return CAN_SendFrame(MOTOR_ID_QUERY, data, 2U);
}

/* ======================== 接收反馈处理 ======================== */

/**
  * @brief  CAN 接收帧处理: 解析电机反馈帧 (支持多电机)
  */
HAL_StatusTypeDef Motor_ProcessRxFrame(uint32_t stdId, const uint8_t *pData, uint8_t len)
{
    if (pData == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    /* 0. SN 上报: 0x0312 (7字节 SN) → 入队 */
    if (stdId == MOTOR_ID_SN_REPORT)
    {
        if (len >= MOTOR_SN_LEN && g_motor_sn_queue.count < MOTOR_SN_QUEUE_SIZE)
        {
            uint8_t idx = g_motor_sn_queue.count;
            for (uint8_t i = 0; i < MOTOR_SN_LEN; i++)
            {
                g_motor_sn_queue.entries[idx].sn[i] = pData[i];
            }
            g_motor_sn_queue.entries[idx].valid = 1U;
            g_motor_sn_queue.count++;
        }
        return HAL_OK;
    }

    /* 1. 到位位移反馈: 0x0420 + device_id
     *    注意: 范围必须收窄到实际设备数, 不能写 +0x7F!
     *    力矩帧 0x0440+id 与位移帧范围 0x0420~0x049F 重叠,
     *    若位移范围过宽会吞掉力矩帧 (len<8 不解析却 return, 力矩永远为 0) */
    if (stdId > MOTOR_ID_FEEDBACK_BASE &&
        stdId <= MOTOR_ID_FEEDBACK_BASE + MOTOR_MAX_COUNT)
    {
        uint8_t devId = (uint8_t)(stdId - MOTOR_ID_FEEDBACK_BASE);
        if (len >= 8)
        {
            volatile Motor_Status_t *p = &g_motor_status[devId - 1U];
            p->mainPosition    = Motor_UnpackInt32(&pData[0]);
            p->slavePosition   = Motor_UnpackInt32(&pData[4]);
            p->online          = 1U;
            p->feedbackCount++;  /* 新鲜度判据: 等待函数用它区分陈旧/最新剩余位移 */
            /* 剩余位移为 0 → 到位; 非 0 → 清除到位标志 (防止旧帧误触发) */
            if (p->mainPosition == 0)
            {
                p->positionReached = 1U;
            }
            else
            {
                p->positionReached = 0U;
            }
        }
        return HAL_OK;
    }

    /* 2. 力矩数据反馈: 0x0440 + device_id (同样收窄范围, 避免与位移帧混淆) */
    if (stdId > MOTOR_ID_TORQUE_BASE &&
        stdId <= MOTOR_ID_TORQUE_BASE + MOTOR_MAX_COUNT)
    {
        uint8_t devId = (uint8_t)(stdId - MOTOR_ID_TORQUE_BASE);
        if (len >= 4)
        {
            volatile Motor_Status_t *p = &g_motor_status[devId - 1U];
            int16_t tq = Motor_UnpackInt16(&pData[2]);
            p->torque = tq;
            p->torqueFrameCount++;
            for (uint8_t i = 0U; i < 4U; i++)
            {
                p->torqueRaw[i] = pData[i];
            }

            /* 力矩统计 (峰值/最小/最大), 供回零判定与调试观察 */
            int16_t absTq = (tq >= 0) ? tq : (int16_t)(-tq);
            if (absTq > p->torquePeak)
            {
                p->torquePeak = absTq;
            }
            if (tq < p->torqueMin)
            {
                p->torqueMin = tq;
            }
            if (tq > p->torqueMax)
            {
                p->torqueMax = tq;
            }
        }
        return HAL_OK;
    }

    /* 3. 心跳数据反馈: 0x0380 + device_id */
    if (stdId >= MOTOR_ID_HEARTBEAT_BASE && stdId <= MOTOR_ID_HEARTBEAT_BASE + 0x7F)
    {
        uint8_t devId = (uint8_t)(stdId - MOTOR_ID_HEARTBEAT_BASE);
        if (devId >= 1U && devId <= MOTOR_MAX_COUNT)
        {
            g_motor_status[devId - 1U].online = 1U;
        }
        return HAL_OK;
    }

    /* 4. 查询返回: 0x0409 */
    if (stdId == MOTOR_ID_QUERY_REPLY)
    {
        if (len >= 4)
        {
            /* 查询返回的数据取决于之前发送的查询命令，
               此处统一解析为 4 字节值，存入第一个在线电机 */
            for (uint8_t i = 0; i < MOTOR_MAX_COUNT; i++)
            {
                if (g_motor_status[i].deviceId != 0U)
                {
                    g_motor_status[i].currentPosition = Motor_UnpackInt32(&pData[0]);
                    break;
                }
            }
        }
        return HAL_OK;
    }

    /* 5. 参数设置 ACK: 0x041A */
    if (stdId == MOTOR_ID_ACK)
    {
        /* ACK 仅表示参数设置成功 */
        return HAL_OK;
    }

    /* 6. 错误码上报: 0x0100 ~ 0x017F */
    if (stdId >= MOTOR_ID_ERROR_BASE && stdId <= MOTOR_ID_ERROR_BASE + 0x7F)
    {
        uint8_t devId = (uint8_t)(stdId - MOTOR_ID_ERROR_BASE);
        if (devId >= 1U && devId <= MOTOR_MAX_COUNT)
        {
            g_motor_status[devId - 1U].lastErrorCode = (uint16_t)stdId;
        }
        return HAL_OK;
    }

    return HAL_ERROR;
}

/* ======================== 完整初始化序列 ======================== */

/**
  * @brief  单电机完整初始化: 配置参数 → 配置轮径 → 锁定 → 设置默认速度/加速度
  *         注意: 调用前需已通过 Motor_SetDeviceId 完成设备号绑定
  */
HAL_StatusTypeDef Motor_Init(uint8_t deviceId, float wheelDiameter_mm,
                             float speed_mm_s, float accel_mm_s2)
{
    HAL_StatusTypeDef status;

    if (deviceId < 1U || deviceId > MOTOR_MAX_COUNT)
    {
        return HAL_ERROR;
    }

    /* 1. 配置电机参数 (0x0316): 极对数、PID、电流、方向、设备号 */
    status = Motor_ConfigParam(deviceId, deviceId, MOTOR_DIR_CW);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(50);

    /* 2. 配置轮径 (重复 3 次) */
    status = Motor_ConfigWheelDiameter(deviceId, wheelDiameter_mm);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(50);

    /* 3. 锁定电机 */
    status = Motor_Lock(deviceId);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(50);

    /* 4. 设置默认速度 */
    status = Motor_SetSpeed(deviceId, speed_mm_s);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(20);

    /* 5. 设置默认加速度 */
    status = Motor_SetAcceleration(deviceId, accel_mm_s2);
    if (status != HAL_OK)
    {
        return status;
    }

    g_motor_status[deviceId - 1U].deviceId = deviceId;

    return HAL_OK;
}

/**
  * @brief  单电机完整初始化 (自定义 PID 参数):
  *           1. 自定义配置电机参数 (锁定前)
  *           2. 配置轮径
  *           3. 锁定电机
  *           4. 设置默认速度和加速度
  */
HAL_StatusTypeDef Motor_InitCustom(uint8_t deviceId, float wheelDiameter_mm,
                                   float speed_mm_s, float accel_mm_s2,
                                   uint8_t direction,
                                   uint8_t byte0, uint8_t byte1, uint8_t byte2,
                                   uint8_t byte3, uint8_t byte4, uint8_t byte5)
{
    HAL_StatusTypeDef status;

    if (deviceId < 1U || deviceId > MOTOR_MAX_COUNT)
    {
        return HAL_ERROR;
    }

    /* 1. 自定义配置电机参数 (0x0316) — 必须在锁定前 */
    status = Motor_ConfigParamCustom(deviceId, deviceId, direction,
                                     byte0, byte1, byte2, byte3, byte4, byte5);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(50);

    /* 2. 配置轮径 (重复 3 次) */
    status = Motor_ConfigWheelDiameter(deviceId, wheelDiameter_mm);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(50);

    /* 3. 锁定电机 */
    status = Motor_Lock(deviceId);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(50);

    /* 4. 设置默认速度 */
    status = Motor_SetSpeed(deviceId, speed_mm_s);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(20);

    /* 5. 设置默认加速度 */
    status = Motor_SetAcceleration(deviceId, accel_mm_s2);
    if (status != HAL_OK)
    {
        return status;
    }

    g_motor_status[deviceId - 1U].deviceId = deviceId;

    return HAL_OK;
}

/**
  * @brief  扫描总线上已在线的电机 (被动监听)
  *         清除所有 online 标志 → 等待 scanTime_ms → 统计被 CAN RX 任务标记为 online 的电机
  */
uint8_t Motor_ScanOnline(uint32_t scanTime_ms)
{
    /* 清除所有在线标志 */
    for (uint8_t i = 0; i < MOTOR_MAX_COUNT; i++)
    {
        g_motor_status[i].online   = 0U;
        g_motor_status[i].deviceId = 0U;
    }

    /* 被动等待，CAN RX 任务会处理心跳/反馈帧并设置 online 标志 */
    osDelay(scanTime_ms);

    /* 统计在线电机，回填 deviceId */
    uint8_t count = 0;
    for (uint8_t i = 0; i < MOTOR_MAX_COUNT; i++)
    {
        if (g_motor_status[i].online)
        {
            g_motor_status[i].deviceId = i + 1U;
            count++;
        }
    }
    return count;
}

/**
  * @brief  多电机完整初始化 (支持已绑定 + 未绑定混合):
  *           Phase 1: 扫描总线，发现已绑定设备号的电机
  *           Phase 2: 配置已在线电机
  *           Phase 3: 对未在线电机走 SN 绑定流程，再配置
  */
HAL_StatusTypeDef Motor_InitAll(uint8_t motorCount, float wheelDiameter_mm,
                                float speed_mm_s, float accel_mm_s2,
                                uint32_t snTimeout_ms)
{
    if (motorCount == 0U || motorCount > MOTOR_MAX_COUNT)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status;

    /* ===== Phase 1: 扫描已在线电机 (已绑定设备号) ===== */
    uint8_t onlineCount = Motor_ScanOnline(500U);

    /* ===== Phase 2: 配置已在线电机 ===== */
    for (uint8_t devId = 1U; devId <= MOTOR_MAX_COUNT; devId++)
    {
        if (g_motor_status[devId - 1U].online)
        {
            status = Motor_Init(devId, wheelDiameter_mm, speed_mm_s, accel_mm_s2);
            if (status != HAL_OK)
            {
                return status;
            }
            osDelay(50);
        }
    }

    /* ===== Phase 3: 对未在线电机走 SN 绑定流程 ===== */
    uint8_t boundCount = onlineCount;
    uint8_t sn[MOTOR_SN_LEN];

    while (boundCount < motorCount)
    {
        /* 找下一个未绑定的设备号槽位 */
        uint8_t nextDevId = 0;
        for (uint8_t i = 1U; i <= MOTOR_MAX_COUNT; i++)
        {
            if (!g_motor_status[i - 1U].online)
            {
                nextDevId = i;
                break;
            }
        }
        if (nextDevId == 0) break;

        /* 清空 SN 队列 */
        Motor_ClearSNQueue();

        /* 请求 SN (已绑定的电机不会再响应，只有未绑定的会回) */
        Motor_RequestSN(0x07U);

        /* 等待 SN 上报 */
        status = Motor_WaitSN(sn, snTimeout_ms);
        if (status != HAL_OK)
        {
            return status;  /* 没有更多未绑定电机响应 */
        }

        /* 绑定设备号 */
        status = Motor_SetDeviceId(sn, nextDevId);
        if (status != HAL_OK)
        {
            return status;
        }
        osDelay(100);

        /* 保存 SN 到状态结构体 */
        for (uint8_t i = 0; i < MOTOR_SN_LEN; i++)
        {
            g_motor_status[nextDevId - 1U].sn[i] = sn[i];
        }
        g_motor_status[nextDevId - 1U].snValid  = 1U;
        g_motor_status[nextDevId - 1U].online   = 1U;
        g_motor_status[nextDevId - 1U].deviceId = nextDevId;

        /* 配置该电机 */
        status = Motor_Init(nextDevId, wheelDiameter_mm, speed_mm_s, accel_mm_s2);
        if (status != HAL_OK)
        {
            return status;
        }
        osDelay(50);

        boundCount++;
    }

    return HAL_OK;
}
