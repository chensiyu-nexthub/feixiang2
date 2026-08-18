/**
  ******************************************************************************
  * @file    motor.c
  * @brief   电机 CAN 通讯驱动实现 (V1.1)
  *
  *          所有位置/速度/加速度均以浮点 mm / mm/s / mm/s² 传入,
  *          内部转换为协议要求的 0.1mm / 0.1mm/s 整数单位。
  *          int32 数据采用小端序打包 (Data[0]=低字节)。
  ******************************************************************************
  */

#include "motor.h"
#include "can.h"
#include <cmsis_os.h>

/* ======================== 全局电机状态 ======================== */
volatile Motor_Status_t  g_motor_status[MOTOR_MAX_COUNT] = {0};
volatile Motor_SNQueue_t g_motor_sn_queue = {0};

/* Flash 配置 (运行时副本) */
MotorConfig_Run_t g_cfg_motor[MOTOR_MAX_COUNT];

/* 各电机默认轮径 (×100), 来自带教配置 */
static const uint16_t s_default_dia[MOTOR_MAX_COUNT] = {
    3183U,  /* 电机1: 传送带左 31.83mm */
    3183U,  /* 电机2: 传送带右 31.83mm */
    738U,   /* 电机3: 钩爪 7.38mm */
    327U,   /* 电机4: 旋转 3.27mm */
    637U,   /* 电机5: 皮带 6.37mm */
};

/* 各电机默认方向, 来自带教配置 */
static const uint8_t s_default_dir[MOTOR_MAX_COUNT] = {
    0U,  /* 电机1: 传送带左 CCW */
    1U,  /* 电机2: 传送带右 CW */
    0U,  /* 电机3: 钩爪 CCW */
    0U,  /* 电机4: 旋转 CCW */
    0U,  /* 电机5: 皮带 CCW */
};

/* 各电机默认速度 (mm/s), 来自带教配置 */
static const uint16_t s_default_spd[MOTOR_MAX_COUNT] = {
    500U,  /* 电机1: 传送带左 */
    500U,  /* 电机2: 传送带右 */
    200U,  /* 电机3: 钩爪 */
    500U,  /* 电机4: 旋转 */
    500U,  /* 电机5: 皮带 */
};

/* 各电机默认加速度 (mm/s²), 来自带教配置 */
static const uint16_t s_default_acc[MOTOR_MAX_COUNT] = {
    1000U,  /* 电机1: 传送带左 */
    1000U,  /* 电机2: 传送带右 */
    200U,   /* 电机3: 钩爪 */
    500U,   /* 电机4: 旋转 */
    500U,   /* 电机5: 皮带 */
};

/* ======================== 内部辅助函数 ======================== */

/**
  * @brief  int32_t → 4 字节小端序
  */
static void Motor_PackInt32(uint8_t *buf, int32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

/**
  * @brief  4 字节小端序 → int32_t
  */
static int32_t Motor_UnpackInt32(const uint8_t *buf)
{
    return (int32_t)((uint32_t)buf[0]
                   | ((uint32_t)buf[1] << 8)
                   | ((uint32_t)buf[2] << 16)
                   | ((uint32_t)buf[3] << 24));
}

/**
  * @brief  2 字节小端序 → int16_t
  */
static int16_t Motor_UnpackInt16(const uint8_t *buf)
{
    return (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

/* ======================== SN 获取与设备号设置 ======================== */

HAL_StatusTypeDef Motor_RequestSN(uint8_t deviceType)
{
    uint8_t data[2];
    data[0] = deviceType;
    data[1] = 0x00U;
    return CAN_SendFrame(MOTOR_ID_GET_SN, data, 2U);
}

HAL_StatusTypeDef Motor_SetDeviceId(const uint8_t *sn, uint8_t deviceId)
{
    if (sn == NULL) return HAL_ERROR;

    uint8_t data[8];
    for (uint8_t i = 0; i < MOTOR_SN_LEN; i++)
    {
        data[i] = sn[i];
    }
    data[7] = deviceId;
    return CAN_SendFrame(MOTOR_ID_SET_DEVICE_ID, data, 8U);
}

HAL_StatusTypeDef Motor_WaitSN(uint8_t *sn, uint32_t timeout_ms)
{
    if (sn == NULL) return HAL_ERROR;

    uint32_t elapsed = 0U;
    const uint32_t pollInterval = 5U;

    while (g_motor_sn_queue.count == 0U)
    {
        if (elapsed >= timeout_ms) return HAL_TIMEOUT;
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

HAL_StatusTypeDef Motor_WaitSNQueue(uint8_t needCount, uint32_t timeout_ms)
{
    uint32_t elapsed = 0U;
    const uint32_t pollInterval = 5U;

    while (g_motor_sn_queue.count < needCount)
    {
        if (elapsed >= timeout_ms) return HAL_TIMEOUT;
        osDelay(pollInterval);
        elapsed += pollInterval;
    }

    return HAL_OK;
}

void Motor_ClearSNQueue(void)
{
    g_motor_sn_queue.count = 0U;
}

volatile Motor_Status_t *Motor_GetStatus(uint8_t deviceId)
{
    if (deviceId < 1U || deviceId > MOTOR_MAX_COUNT) return NULL;
    return &g_motor_status[deviceId - 1U];
}

/* ======================== 初始化配置 ======================== */

HAL_StatusTypeDef Motor_ConfigParam(uint8_t deviceId, uint8_t motorId, uint8_t direction)
{
    uint8_t data[8];
    data[0] = MOTOR_CFG_BYTE0;
    data[1] = MOTOR_CFG_BYTE1;
    data[2] = MOTOR_CFG_BYTE2;
    data[3] = MOTOR_CFG_BYTE3;
    data[4] = MOTOR_CFG_BYTE4;
    data[5] = MOTOR_CFG_BYTE5;
    data[6] = (direction & 0x80U) | (motorId & 0x0FU);
    data[7] = deviceId;
    return CAN_SendFrame(MOTOR_ID_CONFIG_PARAM, data, 8U);
}

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

HAL_StatusTypeDef Motor_ConfigWheelDiameter(uint8_t deviceId, float wheelDiameter_mm)
{
    int32_t diameterValue = (int32_t)(wheelDiameter_mm * 100.0f);

    uint8_t data[4];
    data[0] = deviceId;
    data[1] = 0x24U;
    data[2] = (uint8_t)(diameterValue & 0xFF);
    data[3] = (uint8_t)((diameterValue >> 8) & 0xFF);

    /* 重复 3 次写入 */
    for (uint8_t i = 0; i < 3; i++)
    {
        HAL_StatusTypeDef status = CAN_SendFrame(MOTOR_ID_QUERY, data, 4U);
        if (status != HAL_OK) return status;
        osDelay(10);
    }

    return HAL_OK;
}

HAL_StatusTypeDef Motor_Lock(uint8_t deviceId)
{
    uint8_t data[5] = {0};
    data[0] = MOTOR_CMD_LOCK;
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_LOCK, data, 5U);
}

HAL_StatusTypeDef Motor_Unlock(uint8_t deviceId)
{
    uint8_t data[5] = {0};
    data[0] = MOTOR_CMD_UNLOCK;
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_LOCK, data, 5U);
}

/* ======================== 运动参数设置 ======================== */

HAL_StatusTypeDef Motor_SetDirection(uint8_t deviceId, uint8_t direction)
{
    if (deviceId < 1U || deviceId > MOTOR_MAX_COUNT) return HAL_ERROR;

    MotorConfig_Run_t *cfg = &g_cfg_motor[deviceId - 1U];
    cfg->saved.dir = direction;

    return Motor_ConfigParamCustom(deviceId, deviceId, direction,
        cfg->saved.cfg[0], cfg->saved.cfg[1], cfg->saved.cfg[2],
        cfg->saved.cfg[3], cfg->saved.cfg[4], cfg->saved.cfg[5]);
}

HAL_StatusTypeDef Motor_SetSpeed(uint8_t deviceId, float speed_mm_s)
{
    int32_t speedValue = (int32_t)(speed_mm_s * 10.0f);

    uint8_t data[5];
    Motor_PackInt32(data, speedValue);
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_SPEED_MAIN, data, 5U);
}

HAL_StatusTypeDef Motor_SetSpeedSlave(uint8_t deviceId, float speed_mm_s)
{
    int32_t speedValue = (int32_t)(speed_mm_s * 10.0f);

    uint8_t data[5];
    Motor_PackInt32(data, speedValue);
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_SPEED_SLAVE, data, 5U);
}

HAL_StatusTypeDef Motor_SetAcceleration(uint8_t deviceId, float accel_mm_s2)
{
    int32_t accelValue = (int32_t)(accel_mm_s2 * 10.0f);

    uint8_t data[5];
    Motor_PackInt32(data, accelValue);
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_ACCEL_MAIN, data, 5U);
}

HAL_StatusTypeDef Motor_SetAccelerationSlave(uint8_t deviceId, float accel_mm_s2)
{
    int32_t accelValue = (int32_t)(accel_mm_s2 * 10.0f);

    uint8_t data[5];
    Motor_PackInt32(data, accelValue);
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_ACCEL_SLAVE, data, 5U);
}

/* ======================== 目标位置控制 ======================== */

HAL_StatusTypeDef Motor_SetTarget(uint8_t deviceId, float position_mm, uint8_t targetType)
{
    int32_t targetValue = (int32_t)(position_mm * 10.0f);

    uint8_t data[6];
    Motor_PackInt32(data, targetValue);
    data[4] = deviceId;
    data[5] = targetType;
    return CAN_SendFrame(MOTOR_ID_TARGET_MAIN, data, 6U);
}

HAL_StatusTypeDef Motor_SetTargetSlave(uint8_t deviceId, float position_mm)
{
    int32_t targetValue = (int32_t)(position_mm * 10.0f);

    uint8_t data[5];
    Motor_PackInt32(data, targetValue);
    data[4] = deviceId;
    return CAN_SendFrame(MOTOR_ID_TARGET_SLAVE, data, 5U);
}

HAL_StatusTypeDef Motor_EmergencyStop(uint8_t deviceId)
{
    return Motor_SetTarget(deviceId, 0.0f, MOTOR_TARGET_EMERGENCY_STOP);
}

HAL_StatusTypeDef Motor_SetZero(uint8_t deviceId)
{
    return Motor_SetTarget(deviceId, 0.0f, MOTOR_TARGET_ZERO);
}

HAL_StatusTypeDef Motor_Home(uint8_t deviceId)
{
    return Motor_SetZero(deviceId);
}

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

HAL_StatusTypeDef Motor_QueryTemperature(uint8_t deviceId)
{
    uint8_t data[2];
    data[0] = deviceId;
    data[1] = MOTOR_QUERY_TEMP;
    return CAN_SendFrame(MOTOR_ID_QUERY, data, 2U);
}

HAL_StatusTypeDef Motor_QueryPosition(uint8_t deviceId)
{
    uint8_t data[2];
    data[0] = deviceId;
    data[1] = MOTOR_QUERY_POSITION;
    return CAN_SendFrame(MOTOR_ID_QUERY, data, 2U);
}

HAL_StatusTypeDef Motor_QueryStatus(uint8_t deviceId)
{
    uint8_t data[2];
    data[0] = deviceId;
    data[1] = MOTOR_QUERY_STATUS;
    return CAN_SendFrame(MOTOR_ID_QUERY, data, 2U);
}

HAL_StatusTypeDef Motor_QueryVersion(uint8_t deviceId)
{
    uint8_t data[2];
    data[0] = deviceId;
    data[1] = MOTOR_QUERY_VERSION;
    return CAN_SendFrame(MOTOR_ID_QUERY, data, 2U);
}

/* ======================== 接收反馈处理 ======================== */

HAL_StatusTypeDef Motor_ProcessRxFrame(uint32_t stdId, const uint8_t *pData)
{
    if (pData == NULL) return HAL_ERROR;

    /* CAN 数据域固定 8 字节, 用于安全检查 */
    const uint8_t len = 8U;

    /* 0. SN 上报: 0x0312 → 入队 */
    if (stdId == MOTOR_ID_SN_REPORT)
    {
        if (8U >= MOTOR_SN_LEN && g_motor_sn_queue.count < MOTOR_SN_QUEUE_SIZE)
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

    /* 1. 到位位移反馈: 0x0420 + device_id (范围收窄到实际设备数, 防止吞掉力矩帧) */
    if (stdId > MOTOR_ID_FEEDBACK_BASE &&
        stdId <= MOTOR_ID_FEEDBACK_BASE + MOTOR_MAX_COUNT)
    {
        uint8_t devId = (uint8_t)(stdId - MOTOR_ID_FEEDBACK_BASE);
        if (len >= 8)
        {
            volatile Motor_Status_t *p = &g_motor_status[devId - 1U];
            p->mainPosition  = Motor_UnpackInt32(&pData[0]);
            p->slavePosition = Motor_UnpackInt32(&pData[4]);
            p->online        = 1U;
            p->feedbackCount++;
            p->positionReached = (p->mainPosition == 0) ? 1U : 0U;
        }
        return HAL_OK;
    }

    /* 2. 力矩数据反馈: 0x0440 + device_id */
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

            /* 力矩统计 (峰值/最小/最大) */
            int16_t absTq = (tq >= 0) ? tq : (int16_t)(-tq);
            if (absTq > p->torquePeak) p->torquePeak = absTq;
            if (tq < p->torqueMin)     p->torqueMin  = tq;
            if (tq > p->torqueMax)     p->torqueMax  = tq;
        }
        return HAL_OK;
    }

    /* 3. 心跳反馈: 0x0380 + device_id */
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

/* ======================== Flash 配置加载 / 保存 ======================== */

/**
  * @brief  从 Flash 加载所有电机配置到 g_cfg_motor[]
  * @param  reset  1=强制使用默认值 (忽略 Flash 内容)
  */
HAL_StatusTypeDef Motor_LoadConfig(uint8_t reset)
{
    /* 默认 PID 参数 (与 0x0316 帧对应) */
    uint8_t cfg_default[6] = {
        MOTOR_CFG_BYTE0, MOTOR_CFG_BYTE1, MOTOR_CFG_BYTE2,
        MOTOR_CFG_BYTE3, MOTOR_CFG_BYTE4, MOTOR_CFG_BYTE5
    };

    for (uint8_t i = 0U; i < MOTOR_MAX_COUNT; i++)
    {
        uint32_t base = CONFIG_FLASH_BASE + (uint32_t)i * CONFIG_MOTOR_ENTRY_SIZE;

        /* cfg[6] */
        cfg_read_mem(g_cfg_motor[i].saved.cfg, base, cfg_default, 6U, reset);

        /* devno */
        g_cfg_motor[i].saved.devno = cfg_read_u8(base + 6U, i + 1U, reset);

        /* dir */
        g_cfg_motor[i].saved.dir = cfg_read_u8(base + 7U, s_default_dir[i], reset);

        /* dia */
        g_cfg_motor[i].saved.dia = cfg_read_u16(base + 8U, s_default_dia[i], reset);

        /* spd */
        g_cfg_motor[i].saved.spd = cfg_read_u16(base + 10U, s_default_spd[i], reset);

        /* acc */
        g_cfg_motor[i].saved.acc = cfg_read_u16(base + 12U, s_default_acc[i], reset);

        /* 运行时副本初始化为 Flash 值 */
        g_cfg_motor[i].sspd = g_cfg_motor[i].saved.spd;
        g_cfg_motor[i].sacc = g_cfg_motor[i].saved.acc;
    }

    return HAL_OK;
}

/**
  * @brief  将当前 g_cfg_motor[] 保存到 Flash
  */
HAL_StatusTypeDef Motor_SaveConfig(void)
{
    MotorConfig_t configs[MOTOR_MAX_COUNT];

    for (uint8_t i = 0U; i < MOTOR_MAX_COUNT; i++)
    {
        configs[i] = g_cfg_motor[i].saved;
    }

    return cfg_write_motor_config(configs, MOTOR_MAX_COUNT);
}

/* ======================== 完整初始化序列 ======================== */

HAL_StatusTypeDef Motor_Init(uint8_t deviceId, float wheelDiameter_mm,
                             float speed_mm_s, float accel_mm_s2)
{
    return Motor_InitCustom(deviceId, wheelDiameter_mm, speed_mm_s, accel_mm_s2,
                            MOTOR_DIR_CW,
                            MOTOR_CFG_BYTE0, MOTOR_CFG_BYTE1, MOTOR_CFG_BYTE2,
                            MOTOR_CFG_BYTE3, MOTOR_CFG_BYTE4, MOTOR_CFG_BYTE5);
}

HAL_StatusTypeDef Motor_InitCustom(uint8_t deviceId, float wheelDiameter_mm,
                                   float speed_mm_s, float accel_mm_s2,
                                   uint8_t direction,
                                   uint8_t byte0, uint8_t byte1, uint8_t byte2,
                                   uint8_t byte3, uint8_t byte4, uint8_t byte5)
{
    HAL_StatusTypeDef status;

    if (deviceId < 1U || deviceId > MOTOR_MAX_COUNT) return HAL_ERROR;

    /* 1. 配置电机参数 (0x0316) — 必须在锁定前 */
    status = Motor_ConfigParamCustom(deviceId, deviceId, direction,
                                     byte0, byte1, byte2, byte3, byte4, byte5);
    if (status != HAL_OK) return status;
    osDelay(50);

    /* 2. 配置轮径 (重复 3 次) */
    status = Motor_ConfigWheelDiameter(deviceId, wheelDiameter_mm);
    if (status != HAL_OK) return status;
    osDelay(50);

    /* 3. 锁定电机 */
    status = Motor_Lock(deviceId);
    if (status != HAL_OK) return status;
    osDelay(50);

    /* 4. 设置默认速度 */
    status = Motor_SetSpeed(deviceId, speed_mm_s);
    if (status != HAL_OK) return status;
    osDelay(20);

    /* 5. 设置默认加速度 */
    status = Motor_SetAcceleration(deviceId, accel_mm_s2);
    if (status != HAL_OK) return status;

    g_motor_status[deviceId - 1U].deviceId = deviceId;

    return HAL_OK;
}

uint8_t Motor_ScanOnline(uint32_t scanTime_ms)
{
    /* 清除所有在线标志 */
    for (uint8_t i = 0; i < MOTOR_MAX_COUNT; i++)
    {
        g_motor_status[i].online   = 0U;
        g_motor_status[i].deviceId = 0U;
    }

    /* 被动等待, CAN RX 任务会处理心跳/反馈帧并设置 online 标志 */
    osDelay(scanTime_ms);

    /* 统计在线电机, 回填 deviceId */
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

HAL_StatusTypeDef Motor_InitAll(uint8_t motorCount, float wheelDiameter_mm,
                                float speed_mm_s, float accel_mm_s2,
                                uint32_t snTimeout_ms)
{
    if (motorCount == 0U || motorCount > MOTOR_MAX_COUNT) return HAL_ERROR;

    HAL_StatusTypeDef status;

    /* 0. 从 Flash 加载配置 (首次上电自动使用默认值) */
    Motor_LoadConfig(0U);

    /* Phase 1: 扫描已在线电机 */
    uint8_t onlineCount = Motor_ScanOnline(500U);

    /* Phase 2: 配置已在线电机 (使用 g_cfg_motor 中的参数) */
    for (uint8_t devId = 1U; devId <= MOTOR_MAX_COUNT; devId++)
    {
        if (g_motor_status[devId - 1U].online)
        {
            MotorConfig_Run_t *cfg = &g_cfg_motor[devId - 1U];
            status = Motor_InitCustom(devId,
                                      (float)cfg->saved.dia / 100.0f,
                                      (float)cfg->sspd,
                                      (float)cfg->sacc,
                                      cfg->saved.dir,
                                      cfg->saved.cfg[0], cfg->saved.cfg[1],
                                      cfg->saved.cfg[2], cfg->saved.cfg[3],
                                      cfg->saved.cfg[4], cfg->saved.cfg[5]);
            if (status != HAL_OK) return status;
            osDelay(50);
        }
    }

    /* Phase 3: 对未在线电机走 SN 绑定流程 */
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

        Motor_ClearSNQueue();
        Motor_RequestSN(0x07U);

        status = Motor_WaitSN(sn, snTimeout_ms);
        if (status != HAL_OK) return status;

        status = Motor_SetDeviceId(sn, nextDevId);
        if (status != HAL_OK) return status;
        osDelay(100);

        for (uint8_t i = 0; i < MOTOR_SN_LEN; i++)
        {
            g_motor_status[nextDevId - 1U].sn[i] = sn[i];
        }
        g_motor_status[nextDevId - 1U].snValid  = 1U;
        g_motor_status[nextDevId - 1U].online   = 1U;
        g_motor_status[nextDevId - 1U].deviceId = nextDevId;

        MotorConfig_Run_t *cfg = &g_cfg_motor[nextDevId - 1U];
        status = Motor_InitCustom(nextDevId,
                                  (float)cfg->saved.dia / 100.0f,
                                  (float)cfg->sspd,
                                  (float)cfg->sacc,
                                  cfg->saved.dir,
                                  cfg->saved.cfg[0], cfg->saved.cfg[1],
                                  cfg->saved.cfg[2], cfg->saved.cfg[3],
                                  cfg->saved.cfg[4], cfg->saved.cfg[5]);
        if (status != HAL_OK) return status;
        osDelay(50);

        boundCount++;
    }

    return HAL_OK;
}