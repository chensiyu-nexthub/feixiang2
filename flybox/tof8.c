/**
  ******************************************************************************
  * @file    tof8.c
  * @brief   TOF8 激光测距传感器 CAN 通讯驱动实现
  *
  *         协议要点:
  *           - 配置帧 ID 0x0316, 8字节 (ROI/光强/旋转/模式/设备号)
  *           - 请求帧 ID 0x0408, 响应帧 ID 0x0409 (通道号区分命令)
  *           - 自动上报 ID 0x0793 (距离) / 0x0794 (距离+温度)
  *           - 距离: u16 小端, 单位 0.1mm
  *           - 无效值: ≤10 / 8888 / 9999
  *
  *         设备号: 25 (tof25), 26 (tof26)
  ******************************************************************************
  */

#include "tof8.h"
#include "can.h"
#include "filter.h"
#include <../CMSIS_RTOS/cmsis_os.h>
#include <stddef.h>

/* ======================== 设备号查找表 ======================== */
static const uint8_t s_deviceIds[TOF8_MAX_DEVICES] = {
    TOF8_DEVICE_ID_0,   /* 25 */
    TOF8_DEVICE_ID_1,   /* 26 */
};

/* ======================== 全局状态 ======================== */
volatile TOF8_Status_t g_tof8_status[TOF8_MAX_DEVICES];

/* 看门狗计时基准 (最近一次收到有效数据的 tick) */
static uint32_t s_lastValidTick = 0;

/* 每个设备的滤波器: 中值(去毛刺) + IIR(平滑) */
static Median_Filter_t s_medianFilter[TOF8_MAX_DEVICES];
static IIR_Filter_t    s_iirFilter[TOF8_MAX_DEVICES];

/* 每个设备自上次无效帧后的连续有效采样计数 (滤波器预热用) */
static uint8_t s_validCount[TOF8_MAX_DEVICES];

/* ======================== 内部辅助函数 ======================== */

/**
  * @brief  根据设备号查找数组索引
  * @param  deviceId  设备号
  * @retval 索引 (0 ~ TOF8_MAX_DEVICES-1), 未找到返回 -1
  */
static int TOF8_FindDeviceIndex(uint8_t deviceId)
{
    for (int i = 0; i < (int)TOF8_MAX_DEVICES; i++)
    {
        if (s_deviceIds[i] == deviceId)
        {
            return i;
        }
    }
    return -1;
}

/**
  * @brief  判断原始距离值是否无效
  * @param  raw  原始距离值 (u16)
  * @retval 1 = 无效, 0 = 有效
  */
static uint8_t TOF8_IsInvalid(uint16_t raw)
{
    if (raw <= TOF8_INVALID_MIN)  return 1;
    if (raw == TOF8_INVALID_8888) return 1;
    if (raw == TOF8_INVALID_9999) return 1;
    return 0;
}

/**
  * @brief  将原始距离值转换为 mm
  * @param  raw  原始距离值 (u16, 0.1mm)
  * @retval 距离 (mm), 无效时返回 9999.0
  */
static float TOF8_RawToMm(uint16_t raw)
{
    if (TOF8_IsInvalid(raw))
    {
        return TOF8_DISTANCE_INVALID_MM;
    }
    return (float)raw / 10.0f;
}

/**
  * @brief  发送请求帧 (ID 0x0408, 8字节)
  * @param  deviceId  设备号 (B0)
  * @param  channel   通道号 (B1)
  * @param  cmd       命令字节 (B2), 其余填 0
  * @retval HAL_OK 成功
  */
static HAL_StatusTypeDef TOF8_SendRequest(uint8_t deviceId, uint8_t channel, uint8_t cmd)
{
    uint8_t data[8] = {0};
    data[0] = deviceId;
    data[1] = channel;
    data[2] = cmd;
    /* data[3]~data[7] = 0x00 */
    return CAN_SendFrame(TOF8_ID_REQUEST, data, 8U);
}

/* ======================== 配置与初始化 ======================== */

/**
  * @brief  发送配置帧 (ID 0x0316, 8字节)
  *
  *         帧格式:
  *           B0: ROI 起始
  *           B1: ROI 结束
  *           B2: 预留 (0x00)
  *           B3: 光强阈值
  *           B4: 旋转角
  *           B5: 模式
  *           B6: 预留 (0x00)
  *           B7: 设备号
  */
HAL_StatusTypeDef TOF8_SendConfig(const TOF8_Config_t *pCfg)
{
    if (pCfg == NULL) return HAL_ERROR;

    uint8_t data[8];
    data[0] = pCfg->roiStart;
    data[1] = pCfg->roiEnd;
    data[2] = 0x00U;              /* 预留 */
    data[3] = pCfg->lightThreshold;
    data[4] = pCfg->rotation;
    data[5] = pCfg->mode;
    data[6] = 0x00U;              /* 预留 */
    data[7] = pCfg->deviceId;

    return CAN_SendFrame(TOF8_ID_CONFIG, data, 8U);
}

/**
  * @brief  初始化 TOF8 驱动: 向所有已注册设备发送配置帧
  *         开机调用一次即可
  */
HAL_StatusTypeDef TOF8_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    /* 清零状态 */
    for (int i = 0; i < (int)TOF8_MAX_DEVICES; i++)
    {
        g_tof8_status[i].deviceId       = s_deviceIds[i];
        g_tof8_status[i].initialized    = 0;
        g_tof8_status[i].continuousOn   = 0;
        g_tof8_status[i].distanceRaw    = 0;
        g_tof8_status[i].distanceMm     = TOF8_DISTANCE_INVALID_MM;
        g_tof8_status[i].temperature    = 0.0f;
        g_tof8_status[i].newDataFlag    = 0;
        g_tof8_status[i].resolution     = 0;
        g_tof8_status[i].frequency      = 0;
        g_tof8_status[i].lastUpdateTick = 0;
    }

    /* 初始化滤波器 */
    for (int i = 0; i < (int)TOF8_MAX_DEVICES; i++)
    {
        Median_Filter_Init(&s_medianFilter[i]);
        IIR_Filter_Init(&s_iirFilter[i], TOF8_FILTER_IIR_FC, TOF8_FILTER_FS);
        s_validCount[i] = 0U;
    }

    /* 向每个设备发送配置帧 */
    for (int i = 0; i < (int)TOF8_MAX_DEVICES; i++)
    {
        TOF8_Config_t cfg = {
            .deviceId       = s_deviceIds[i],
            .roiStart       = TOF8_CFG_ROI_START,
            .roiEnd         = TOF8_CFG_ROI_END,
            .lightThreshold = TOF8_CFG_LIGHT_THRESHOLD,
            .rotation       = TOF8_CFG_ROTATION,
            .mode           = TOF8_CFG_MODE_DEFAULT,
        };

        status = TOF8_SendConfig(&cfg);
        if (status != HAL_OK) return status;

        g_tof8_status[i].initialized = 1;

        /* 设备间短暂间隔 */
        osDelay(10);
    }

    /* 首次启动需 >100ms 才有数据 */
    osDelay(150);

    return status;
}

/* ======================== 高级接口 ======================== */

/**
  * @brief  完整启动序列: 复位 → 配置 → 开启连续测距
  */
HAL_StatusTypeDef TOF8_StartAll(void)
{
    /* 0. 静默等待: 让 Bus-Off 的模块有时间自动恢复
     *    MCU 烧录期间 CAN 外设关闭, 模块发心跳无人 ACK 会进入 Bus-Off,
     *    需要总线静默 (无帧发送) 才能恢复 */
    osDelay(1000);

    /* 1. 复位所有设备 MCU (重试3次, 确保模块已恢复) */
    for (int retry = 0; retry < 3; retry++)
    {
        for (int i = 0; i < (int)TOF8_MAX_DEVICES; i++)
        {
            TOF8_Reset(s_deviceIds[i], TOF8_CH_RESET_MCU);
        }
        osDelay(500);  /* 等待模块复位完成 */
    }

    /* 2. 发送配置帧 */
    TOF8_Init();

    /* 3. 逐个开启连续测距 (间隔确保模块处理完) */
    for (int i = 0; i < (int)TOF8_MAX_DEVICES; i++)
    {
        TOF8_ContinuousStart(s_deviceIds[i]);
        osDelay(100);
    }

    osDelay(200);  /* 等待首次测量完成 */

    /* 重置看门狗计时基准 */
    s_lastValidTick = HAL_GetTick();

    return HAL_OK;
}

/**
  * @brief  轮询所有设备的距离读数
  */
void TOF8_PollAll(void)
{
    for (int i = 0; i < (int)TOF8_MAX_DEVICES; i++)
    {
        TOF8_ReadContinuous(s_deviceIds[i]);
        osDelay(5);  /* 设备间短暂间隔避免总线拥塞 */
    }

    /* 更新看门狗: 任何设备有有效数据则刷新计时 */
    for (int i = 0; i < (int)TOF8_MAX_DEVICES; i++)
    {
        if (g_tof8_status[i].newDataFlag && g_tof8_status[i].distanceRaw > 0)
        {
            s_lastValidTick = HAL_GetTick();
            break;
        }
    }
}

/**
  * @brief  看门狗检查: 超时则自动复位重启
  */
uint8_t TOF8_WatchdogCheck(uint32_t timeoutMs)
{
    if (HAL_GetTick() - s_lastValidTick > timeoutMs)
    {
        TOF8_StartAll();
        return 1;
    }
    return 0;
}

/* ======================== 连续测距 ======================== */

/**
  * @brief  开启连续测距 (通道10, 命令=1)
  *
  *         帧格式 (文档要求 B3/B4 填 ROI):
  *           B0=设备号, B1=10, B2=1(开), B3=ROI起始, B4=ROI结束, B5-B7=0
  */
HAL_StatusTypeDef TOF8_ContinuousStart(uint8_t deviceId)
{
    int idx = TOF8_FindDeviceIndex(deviceId);
    if (idx < 0) return HAL_ERROR;

    uint8_t data[8] = {0};
    data[0] = deviceId;
    data[1] = TOF8_CH_CONT_SWITCH;
    data[2] = 0x01U;              /* 命令: 1=开 */
    data[3] = TOF8_CFG_ROI_START; /* ROI 起始 0x11 */
    data[4] = TOF8_CFG_ROI_END;   /* ROI 结束 0x22 */

    HAL_StatusTypeDef status = CAN_SendFrame(TOF8_ID_REQUEST, data, 8U);
    if (status == HAL_OK)
    {
        g_tof8_status[idx].continuousOn = 1;
    }
    return status;
}

/**
  * @brief  关闭连续测距 (通道10, 命令=0)
  *
  *         帧格式: B0=设备号, B1=10, B2=0(关), B3=ROI起始, B4=ROI结束, B5-B7=0
  */
HAL_StatusTypeDef TOF8_ContinuousStop(uint8_t deviceId)
{
    int idx = TOF8_FindDeviceIndex(deviceId);
    if (idx < 0) return HAL_ERROR;

    uint8_t data[8] = {0};
    data[0] = deviceId;
    data[1] = TOF8_CH_CONT_SWITCH;
    data[2] = 0x00U;              /* 命令: 0=关 */
    data[3] = TOF8_CFG_ROI_START; /* ROI 起始 0x11 */
    data[4] = TOF8_CFG_ROI_END;   /* ROI 结束 0x22 */

    HAL_StatusTypeDef status = CAN_SendFrame(TOF8_ID_REQUEST, data, 8U);
    if (status == HAL_OK)
    {
        g_tof8_status[idx].continuousOn = 0;
    }
    return status;
}

/**
  * @brief  请求连续测距读数 (通道12)
  */
HAL_StatusTypeDef TOF8_ReadContinuous(uint8_t deviceId)
{
    if (TOF8_FindDeviceIndex(deviceId) < 0) return HAL_ERROR;
    return TOF8_SendRequest(deviceId, TOF8_CH_READ_DATA, 0x00U);
}

/* ======================== 单次测距 ======================== */

/**
  * @brief  启动单次测距 (通道11)
  */
HAL_StatusTypeDef TOF8_SingleShot(uint8_t deviceId)
{
    if (TOF8_FindDeviceIndex(deviceId) < 0) return HAL_ERROR;
    return TOF8_SendRequest(deviceId, TOF8_CH_SINGLE_START, 0x00U);
}

/**
  * @brief  查询测距结果 (通道12)
  */
HAL_StatusTypeDef TOF8_QueryResult(uint8_t deviceId)
{
    if (TOF8_FindDeviceIndex(deviceId) < 0) return HAL_ERROR;
    return TOF8_SendRequest(deviceId, TOF8_CH_READ_DATA, 0x00U);
}

/* ======================== 辅助命令 ======================== */

/**
  * @brief  复位 TOF8
  * @param  resetType  TOF8_CH_RESET_VL53(8) 或 TOF8_CH_RESET_MCU(9)
  */
HAL_StatusTypeDef TOF8_Reset(uint8_t deviceId, uint8_t resetType)
{
    if (TOF8_FindDeviceIndex(deviceId) < 0) return HAL_ERROR;
    if (resetType != TOF8_CH_RESET_VL53 && resetType != TOF8_CH_RESET_MCU)
    {
        return HAL_ERROR;
    }
    return TOF8_SendRequest(deviceId, resetType, 0x00U);
}

/**
  * @brief  查询分辨率 (通道15)
  */
HAL_StatusTypeDef TOF8_QueryResolution(uint8_t deviceId)
{
    if (TOF8_FindDeviceIndex(deviceId) < 0) return HAL_ERROR;
    return TOF8_SendRequest(deviceId, TOF8_CH_RESOLUTION, 0x00U);
}

/**
  * @brief  查询测量频率 (通道16)
  */
HAL_StatusTypeDef TOF8_QueryFrequency(uint8_t deviceId)
{
    if (TOF8_FindDeviceIndex(deviceId) < 0) return HAL_ERROR;
    return TOF8_SendRequest(deviceId, TOF8_CH_FREQUENCY, 0x00U);
}

/* ======================== 接收解析 ======================== */

/**
  * @brief  解析 TOF8 响应 / 自动上报帧
  *
  *         处理帧类型:
  *           0x0409: 响应帧 (通道号在 B1)
  *             - 通道12: B2-B3 距离 (u16 LE, 0.1mm), B4 新数据标志
  *             - 通道15: B2 分辨率
  *             - 通道16: B2 频率
  *           0x0793: 自动上报距离, B0=设备号, B2-B3 距离
  *           0x0794: 自动上报距离+温度, B0=设备号, B2-B3 距离, B4=温度
  */
/**
  * @brief  更新指定设备的距离滤波 (三种距离帧共用)
  *
  *         预热保护: 无效帧会重置中值/IIR 滤波器 (y_prev=0, 窗口清零),
  *         此后滤波器从 0 向真实值爬升, 前几帧输出远小于真实距离。
  *         若直接发布, 联动逼近会误判 "已收敛" (remaining 为大负值) 而提前停止。
  *         因此预热期 (连续有效采样 < TOF8_FILTER_WARMUP_SAMPLES) 内发布无效值,
  *         让上层走 "无效 → 等待/盲进重测" 的安全路径。
  */
static void TOF8_UpdateDistance(int idx, uint16_t raw)
{
    if (TOF8_IsInvalid(raw))
    {
        g_tof8_status[idx].distanceMm = TOF8_DISTANCE_INVALID_MM;
        s_validCount[idx] = 0U;
        Median_Filter_Reset(&s_medianFilter[idx]);
        IIR_Filter_Reset(&s_iirFilter[idx]);
        return;
    }

    if (s_validCount[idx] < 0xFFU)
    {
        s_validCount[idx]++;
    }

    float mm = (float)raw / 10.0f;
    mm = Median_Filter_Update(&s_medianFilter[idx], mm);
    mm = IIR_Filter_Update(&s_iirFilter[idx], mm);

    /* 预热未完成: 发布无效值, 防止爬升中的假近值导致上层虚假收敛 */
    if (s_validCount[idx] < TOF8_FILTER_WARMUP_SAMPLES)
    {
        g_tof8_status[idx].distanceMm = TOF8_DISTANCE_INVALID_MM;
    }
    else
    {
        g_tof8_status[idx].distanceMm = mm;
    }
}

uint8_t TOF8_ProcessRxFrame(uint32_t stdId, const uint8_t *pData)
{
    if (pData == NULL) return 0;

    if (stdId == TOF8_ID_RESPONSE)
    {
        /* 响应帧: B0=设备号, B1=通道号 */
        uint8_t deviceId = pData[0];
        uint8_t channel  = pData[1];
        int idx = TOF8_FindDeviceIndex(deviceId);
        if (idx < 0) return 0;  /* 非 TOF8 设备 (可能是电机) */

        switch (channel)
        {
        case TOF8_CH_READ_DATA:
        {
            /* B2-B3: 距离 u16 小端, B4: 新数据标志 */
            uint16_t raw = (uint16_t)pData[2] | ((uint16_t)pData[3] << 8);
            g_tof8_status[idx].distanceRaw    = raw;
            g_tof8_status[idx].newDataFlag    = pData[4];
            g_tof8_status[idx].lastUpdateTick = HAL_GetTick();

            /* 滤波: 无效值直接输出, 有效值经中值+IIR平滑 (含预热保护) */
            TOF8_UpdateDistance(idx, raw);
            break;
        }
        case TOF8_CH_RESOLUTION:
            g_tof8_status[idx].resolution = pData[2];
            break;

        case TOF8_CH_FREQUENCY:
            g_tof8_status[idx].frequency = pData[2];
            break;

        default:
            /* 其他通道响应 (复位等) 不做特殊处理 */
            break;
        }
        return 1;
    }
    else if (stdId == TOF8_ID_REPORT_DIST)
    {
        /* 自动上报距离: B0=设备号, B2-B3 距离 (u16 LE) */
        uint8_t deviceId = pData[0];
        int idx = TOF8_FindDeviceIndex(deviceId);
        if (idx < 0) return 0;

        uint16_t raw = (uint16_t)pData[2] | ((uint16_t)pData[3] << 8);
        g_tof8_status[idx].distanceRaw    = raw;
        g_tof8_status[idx].newDataFlag    = 1;
        g_tof8_status[idx].lastUpdateTick = HAL_GetTick();

        /* 滤波 (含预热保护) */
        TOF8_UpdateDistance(idx, raw);
        return 1;
    }
    else if (stdId == TOF8_ID_REPORT_DIST_TEMP)
    {
        /* 自动上报距离+温度: B0=设备号, B2-B3 距离, B4=温度 */
        uint8_t deviceId = pData[0];
        int idx = TOF8_FindDeviceIndex(deviceId);
        if (idx < 0) return 0;

        uint16_t raw = (uint16_t)pData[2] | ((uint16_t)pData[3] << 8);
        g_tof8_status[idx].distanceRaw    = raw;
        g_tof8_status[idx].temperature    = (float)pData[4];
        g_tof8_status[idx].newDataFlag    = 1;
        g_tof8_status[idx].lastUpdateTick = HAL_GetTick();

        /* 滤波 (含预热保护) */
        TOF8_UpdateDistance(idx, raw);
        return 1;
    }

    return 0;  /* 非 TOF8 帧 */
}

/* ======================== 数据获取 ======================== */

/**
  * @brief  获取指定设备的测距结果
  */
HAL_StatusTypeDef TOF8_GetDistance(uint8_t deviceId, float *pDistMm)
{
    if (pDistMm == NULL) return HAL_ERROR;

    int idx = TOF8_FindDeviceIndex(deviceId);
    if (idx < 0) return HAL_ERROR;

    *pDistMm = g_tof8_status[idx].distanceMm;
    return HAL_OK;
}

/**
  * @brief  多次采样取中值, 抑制 TOF 噪声尖峰
  */
float TOF8_GetDistanceMedian(uint8_t deviceId, uint8_t samples, uint32_t intervalMs)
{
    float buf[7];
    uint8_t valid = 0;

    if (samples > 7) samples = 7;

    for (uint8_t i = 0; i < samples; i++)
    {
        float d = TOF8_DISTANCE_INVALID_MM;
        TOF8_GetDistance(deviceId, &d);
        if (d != TOF8_DISTANCE_INVALID_MM)
        {
            buf[valid++] = d;
        }
        if (i < samples - 1) osDelay(intervalMs);
    }

    if (valid == 0) return TOF8_DISTANCE_INVALID_MM;

    /* 冒泡排序取中值 */
    for (uint8_t i = 0; i < valid - 1; i++)
    {
        for (uint8_t j = 0; j < valid - 1 - i; j++)
        {
            if (buf[j] > buf[j + 1])
            {
                float tmp = buf[j];
                buf[j] = buf[j + 1];
                buf[j + 1] = tmp;
            }
        }
    }
    return buf[valid / 2];
}

/**
  * @brief  获取指定设备的状态结构体指针
  */
volatile TOF8_Status_t *TOF8_GetStatus(uint8_t deviceId)
{
    int idx = TOF8_FindDeviceIndex(deviceId);
    if (idx < 0) return NULL;
    return &g_tof8_status[idx];
}

/* ======================== 健康检查 ======================== */

/**
  * @brief  检查指定 TOF 传感器是否在线 (有数据更新)
  */
uint8_t TOF8_IsAlive(uint8_t deviceId, uint32_t timeoutMs)
{
    int idx = TOF8_FindDeviceIndex(deviceId);
    if (idx < 0) return 0U;
    return (HAL_GetTick() - g_tof8_status[idx].lastUpdateTick < timeoutMs) ? 1U : 0U;
}

/**
  * @brief  检查指定设备的滤波器是否处于预热期
  *         validCount > 0: 有效帧正在到达 (目标在量程内), 滤波器正在收敛
  *         validCount == 0: 无有效帧 (目标可能超出量程或传感器刚复位)
  */
uint8_t TOF8_IsWarmingUp(uint8_t deviceId)
{
    int idx = TOF8_FindDeviceIndex(deviceId);
    if (idx < 0) return 0U;
    return (s_validCount[idx] > 0U && s_validCount[idx] < TOF8_FILTER_WARMUP_SAMPLES) ? 1U : 0U;
}

/**
  * @brief  确保指定 TOF 传感器在线, 不在线则尝试复位重启
  */
HAL_StatusTypeDef TOF8_EnsureAlive(uint8_t deviceId, uint32_t timeoutMs, uint8_t maxRetry)
{
    if (TOF8_IsAlive(deviceId, timeoutMs)) return HAL_OK;

    for (uint8_t retry = 0U; retry < maxRetry; retry++)
    {
        TOF8_Reset(deviceId, TOF8_CH_RESET_MCU);
        osDelay(500);
        TOF8_ContinuousStart(deviceId);
        osDelay(500);

        if (TOF8_IsAlive(deviceId, timeoutMs)) return HAL_OK;
    }

    return HAL_ERROR;
}
