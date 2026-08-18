/**
  ******************************************************************************
  * @file    tof8.h
  * @brief   TOF8 激光测距传感器 CAN 通讯驱动接口
  *
  *          协议要点:
  *            - 配置帧 ID 0x0316, 8 字节 (ROI/光强/旋转/模式/设备号)
  *            - 请求帧 ID 0x0408, 响应帧 ID 0x0409 (通道号区分命令)
  *            - 自动上报 ID 0x0793 (距离) / 0x0794 (距离+温度)
  *            - 距离: u16 小端, 单位 0.1mm
  *            - 无效值: ≤10 / 8888 / 9999
  *
  *          依赖: can.c, filter.c, FreeRTOS (osDelay)
  ******************************************************************************
  */

#ifndef __TOF8_H
#define __TOF8_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

/* ======================== CAN ID 定义 ======================== */
#define TOF8_ID_CONFIG              0x0316U   /* 配置帧 */
#define TOF8_ID_REQUEST             0x0408U   /* 请求帧 */
#define TOF8_ID_RESPONSE            0x0409U   /* 响应帧 */
#define TOF8_ID_REPORT_DIST         0x0793U   /* 自动上报: 距离 */
#define TOF8_ID_REPORT_DIST_TEMP    0x0794U   /* 自动上报: 距离+温度 */

/* ======================== 设备号 ======================== */
#define TOF8_DEVICE_ID_0            25U       /* TOF25 (正面) */
#define TOF8_DEVICE_ID_1            26U       /* TOF26 (侧面) */
#define TOF8_MAX_DEVICES            2U

/* ======================== 通道号 ======================== */
#define TOF8_CH_RESET_VL53          0x08U     /* 复位 VL53Lx */
#define TOF8_CH_RESET_MCU           0x09U     /* 复位 MCU */
#define TOF8_CH_CONT_SWITCH         0x0AU     /* 连续测距开关 */
#define TOF8_CH_SINGLE_START        0x0BU     /* 单次测距启动 */
#define TOF8_CH_READ_DATA           0x0CU     /* 读数通道 */
#define TOF8_CH_RESOLUTION          0x0FU     /* 分辨率查询 */
#define TOF8_CH_FREQUENCY           0x10U     /* 频率查询 */

/* ======================== 默认配置 ======================== */
#define TOF8_CFG_ROI_START          0x11U     /* ROI 起始 */
#define TOF8_CFG_ROI_END            0x22U     /* ROI 结束 */
#define TOF8_CFG_LIGHT_THRESHOLD    0x64U     /* 光强阈值 (100) */
#define TOF8_CFG_ROTATION           0x00U     /* 旋转角 (0°) */
#define TOF8_CFG_MODE_DEFAULT       0x03U     /* 默认模式 */

/* ======================== 无效值定义 ======================== */
#define TOF8_INVALID_MIN            10U       /* ≤10 无效 */
#define TOF8_INVALID_8888           8888U     /* 8888 无效 */
#define TOF8_INVALID_9999           9999U     /* 9999 无效 */
#define TOF8_DISTANCE_INVALID_MM    9999.0f   /* 无效距离 (mm) */

/* ======================== 滤波器参数 ======================== */
#define TOF8_FILTER_IIR_FC          5.0f      /* IIR 截止频率 (Hz) */
#define TOF8_FILTER_FS              50.0f     /* 假设采样率 (Hz) */
#define TOF8_FILTER_WARMUP_SAMPLES  5U        /* 预热所需连续有效采样数 */

/* ======================== TOF8 状态结构体 ======================== */
typedef struct
{
    uint8_t  deviceId;          /* 设备号 (25/26) */
    uint8_t  initialized;       /* 1 = 已初始化 */
    uint8_t  continuousOn;      /* 1 = 连续测距已开启 */
    uint16_t distanceRaw;       /* 原始距离值 (u16, 0.1mm) */
    float    distanceMm;        /* 滤波后距离 (mm), 无效=9999.0 */
    float    temperature;       /* 温度 (°C) */
    uint8_t  newDataFlag;       /* 1 = 有新数据 */
    uint8_t  resolution;        /* 分辨率 */
    uint8_t  frequency;         /* 测量频率 */
    uint32_t lastUpdateTick;    /* 最近更新时刻 (HAL_GetTick) */
} TOF8_Status_t;

/* ======================== 配置结构体 ======================== */
typedef struct
{
    uint8_t deviceId;
    uint8_t roiStart;
    uint8_t roiEnd;
    uint8_t lightThreshold;
    uint8_t rotation;
    uint8_t mode;
} TOF8_Config_t;

/* ======================== 全局变量 ======================== */
extern volatile TOF8_Status_t g_tof8_status[TOF8_MAX_DEVICES];

/* ======================== 配置与初始化 ======================== */

HAL_StatusTypeDef TOF8_SendConfig(const TOF8_Config_t *pCfg);
HAL_StatusTypeDef TOF8_Init(void);
HAL_StatusTypeDef TOF8_StartAll(void);

/* ======================== 连续测距 ======================== */

HAL_StatusTypeDef TOF8_ContinuousStart(uint8_t deviceId);
HAL_StatusTypeDef TOF8_ContinuousStop(uint8_t deviceId);
HAL_StatusTypeDef TOF8_ReadContinuous(uint8_t deviceId);

/* ======================== 单次测距 ======================== */

HAL_StatusTypeDef TOF8_SingleShot(uint8_t deviceId);
HAL_StatusTypeDef TOF8_QueryResult(uint8_t deviceId);

/* ======================== 辅助命令 ======================== */

HAL_StatusTypeDef TOF8_Reset(uint8_t deviceId, uint8_t resetType);
HAL_StatusTypeDef TOF8_QueryResolution(uint8_t deviceId);
HAL_StatusTypeDef TOF8_QueryFrequency(uint8_t deviceId);

/* ======================== 轮询与看门狗 ======================== */

void    TOF8_PollAll(void);
uint8_t TOF8_WatchdogCheck(uint32_t timeoutMs);

/* ======================== 接收解析 ======================== */

uint8_t TOF8_ProcessRxFrame(uint32_t stdId, const uint8_t *pData);

/* ======================== 数据获取 ======================== */

HAL_StatusTypeDef       TOF8_GetDistance(uint8_t deviceId, float *pDistMm);
float                   TOF8_GetDistanceMedian(uint8_t deviceId, uint8_t samples, uint32_t intervalMs);
volatile TOF8_Status_t *TOF8_GetStatus(uint8_t deviceId);

/* ======================== 健康检查 ======================== */

uint8_t          TOF8_IsAlive(uint8_t deviceId, uint32_t timeoutMs);
uint8_t          TOF8_IsWarmingUp(uint8_t deviceId);
HAL_StatusTypeDef TOF8_EnsureAlive(uint8_t deviceId, uint32_t timeoutMs, uint8_t maxRetry);

#ifdef __cplusplus
}
#endif

#endif /* __TOF8_H */