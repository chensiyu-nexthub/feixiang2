/**
  ******************************************************************************
  * @file    tof8.h
  * @brief   TOF8 激光测距传感器 CAN 通讯驱动接口
  *
  *          基于 tof8通讯协议.md 实现:
  *            - 配置帧发送 (ID 0x0316)
  *            - 连续测距 (通道10开关 + 通道12读数)
  *            - 单次测距 (通道11启动 + 通道12查询)
  *            - 自动上报帧解析 (ID 0x0793 / 0x0794)
  *            - 复位 / 状态查询
  *
  *          设备号: 25 (tof25), 26 (tof26)
  *          依赖: can.c (CAN_SendFrame), FreeRTOS (osDelay)
  ******************************************************************************
  */

#ifndef __TOF8_H
#define __TOF8_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

/* ======================== CAN ID 定义 ======================== */
#define TOF8_ID_CONFIG              0x0316U   /* 配置帧 (CFG_SEND) */
#define TOF8_ID_REQUEST             0x0408U   /* 请求帧 */
#define TOF8_ID_RESPONSE            0x0409U   /* 响应帧 */
#define TOF8_ID_REPORT_DIST         0x0793U   /* 自动上报: 距离 */
#define TOF8_ID_REPORT_DIST_TEMP    0x0794U   /* 自动上报: 距离+温度 */

/* ======================== 通道号定义 ======================== */
#define TOF8_CH_RESET_VL53          0x08U     /* 复位 VL53Lx */
#define TOF8_CH_RESET_MCU           0x09U     /* 复位 MCU */
#define TOF8_CH_CONT_SWITCH         0x0AU     /* 连续测距开关 (通道10) */
#define TOF8_CH_SINGLE_START        0x0BU     /* 单次测距启动 (通道11) */
#define TOF8_CH_READ_DATA           0x0CU     /* 连续测距读数 / 单次结果查询 (通道12) */
#define TOF8_CH_RESOLUTION          0x0FU     /* 分辨率查询 (通道15) */
#define TOF8_CH_FREQUENCY           0x10U     /* 频率查询 (通道16) */

/* ======================== 模式定义 ======================== */
#define TOF8_MODE_4X4               0x01U     /* 4x4 模式 */
#define TOF8_MODE_8X8               0x02U     /* 8x8 模式 */
#define TOF8_MODE_ALWAYS            0x04U     /* 持续测量 */

/* ======================== 默认配置参数 ======================== */
#define TOF8_CFG_ROI_START          0x11U     /* ROI 起始 */
#define TOF8_CFG_ROI_END            0x22U     /* ROI 结束 */
#define TOF8_CFG_LIGHT_THRESHOLD    0xFFU     /* 光强阈值 */
#define TOF8_CFG_ROTATION           0x00U     /* 旋转角 (不旋转) */
#define TOF8_CFG_MODE_DEFAULT       (TOF8_MODE_4X4 | TOF8_MODE_ALWAYS)  /* 0x05 */

/* ======================== 无效值定义 ======================== */
#define TOF8_INVALID_MIN            10U       /* 原始值 ≤10 视为无效 */
#define TOF8_INVALID_8888           8888U     /* 无效标志值 */
#define TOF8_INVALID_9999           9999U     /* 无效标志值 */
#define TOF8_DISTANCE_INVALID_MM    9999.0f   /* 无效距离返回值 (mm) */

/* ======================== 滤波参数 ======================== */
#define TOF8_FILTER_IIR_FC          5.0f      /* IIR 截止频率 (Hz) */
#define TOF8_FILTER_FS              20.0f     /* 采样频率 (Hz), 50ms 轮询周期 */
#define TOF8_FILTER_WARMUP_SAMPLES  5U        /* 滤波器复位后需连续有效采样数, 预热完成前发布无效值
                                                 (中值窗口 5 点填满 + IIR 收敛到 ~94%, 防止发布爬升中的假近值) */

/* ======================== 设备配置 ======================== */
#define TOF8_MAX_DEVICES            2U        /* 最大设备数 */
#define TOF8_DEVICE_ID_0            25U       /* tof25 设备号 */
#define TOF8_DEVICE_ID_1            26U       /* tof26 设备号 */

/* ======================== 数据结构 ======================== */

/**
  * @brief  TOF8 配置参数
  */
typedef struct
{
    uint8_t deviceId;         /* 设备号 (25 / 26) */
    uint8_t roiStart;         /* ROI 起始 (默认 0x11) */
    uint8_t roiEnd;           /* ROI 结束 (默认 0x22) */
    uint8_t lightThreshold;   /* 光强阈值 (默认 0xFF) */
    uint8_t rotation;         /* 旋转角 (默认 0x00) */
    uint8_t mode;             /* 模式 (TOF8_MODE_xxx 组合) */
} TOF8_Config_t;

/**
  * @brief  TOF8 运行状态
  */
typedef struct
{
    uint8_t  deviceId;        /* 设备号 */
    uint8_t  initialized;     /* 1 = 已发送配置 */
    uint8_t  continuousOn;    /* 1 = 连续测距已开启 */
    uint16_t distanceRaw;     /* 距离原始值 (u16, 0.1mm) */
    float    distanceMm;      /* 距离 (mm), 无效时为 9999.0 */
    float    temperature;     /* 温度 (°C), 仅 0x0794 上报时更新 */
    uint8_t  newDataFlag;     /* 1 = 新数据, 0 = 旧数据 */
    uint8_t  resolution;      /* 分辨率: 16(4x4) / 64(8x8) */
    uint8_t  frequency;       /* 测量频率 (Hz) */
    uint32_t lastUpdateTick;  /* 最近一次数据更新的系统 tick */
} TOF8_Status_t;

/* ======================== 全局状态 ======================== */
extern volatile TOF8_Status_t g_tof8_status[TOF8_MAX_DEVICES];

/* ======================== 接口函数声明 ======================== */

/**
  * @brief  初始化 TOF8 驱动: 向所有已注册设备发送配置帧
  *         开机调用一次即可，内部含 150ms 等待就绪
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_Init(void);

/**
  * @brief  完整启动序列: 复位所有设备 → 配置 → 开启连续测距
  *         内部含所有必要延时，调用后模块即进入连续测量状态
  *         开机调用一次，或看门狗超时后重新调用
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_StartAll(void);

/**
  * @brief  轮询所有设备的距离读数 (发通道12请求)
  *         响应由 TOF8_ProcessRxFrame() 异步解析到 g_tof8_status
  *         在主循环中周期调用 (建议 50ms)
  */
void TOF8_PollAll(void);

/**
  * @brief  看门狗检查: 若超过 timeoutMs 无任何有效数据则自动复位重启
  *         在主循环中周期调用 (与 TOF8_PollAll 同周期)
  * @param  timeoutMs  超时时间 (ms), 建议 2000
  * @retval 1 = 触发了复位重启, 0 = 正常
  */
uint8_t TOF8_WatchdogCheck(uint32_t timeoutMs);

/**
  * @brief  发送配置帧 (ID 0x0316, 8字节)
  * @param  pCfg  配置参数指针
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_SendConfig(const TOF8_Config_t *pCfg);

/**
  * @brief  开启连续测距 (通道10, 命令=1)
  * @param  deviceId  设备号 (25 / 26)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_ContinuousStart(uint8_t deviceId);

/**
  * @brief  关闭连续测距 (通道10, 命令=0)
  * @param  deviceId  设备号 (25 / 26)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_ContinuousStop(uint8_t deviceId);

/**
  * @brief  请求连续测距读数 (通道12)
  *         响应通过 TOF8_ProcessRxFrame() 异步解析
  * @param  deviceId  设备号 (25 / 26)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_ReadContinuous(uint8_t deviceId);

/**
  * @brief  启动单次测距 (通道11)
  * @param  deviceId  设备号 (25 / 26)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_SingleShot(uint8_t deviceId);

/**
  * @brief  查询测距结果 (通道12)
  *         响应通过 TOF8_ProcessRxFrame() 异步解析
  * @param  deviceId  设备号 (25 / 26)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_QueryResult(uint8_t deviceId);

/**
  * @brief  复位 TOF8
  * @param  deviceId   设备号 (25 / 26)
  * @param  resetType  TOF8_CH_RESET_VL53(8) 或 TOF8_CH_RESET_MCU(9)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_Reset(uint8_t deviceId, uint8_t resetType);

/**
  * @brief  查询分辨率 (通道15)
  *         响应 B2: 16(4x4) / 64(8x8)
  * @param  deviceId  设备号 (25 / 26)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_QueryResolution(uint8_t deviceId);

/**
  * @brief  查询测量频率 (通道16)
  *         响应 B2: 频率 (Hz)
  * @param  deviceId  设备号 (25 / 26)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TOF8_QueryFrequency(uint8_t deviceId);

/**
  * @brief  解析 TOF8 响应 / 自动上报帧
  *         在 CAN 接收任务中调用，处理 ID 0x0409 / 0x0793 / 0x0794
  * @param  stdId  帧 ID
  * @param  pData  数据指针 (8字节)
  * @retval 1 = 已处理 (属于 TOF8 帧), 0 = 非 TOF8 帧
  */
uint8_t TOF8_ProcessRxFrame(uint32_t stdId, const uint8_t *pData);

/**
  * @brief  获取指定设备的测距结果
  * @param  deviceId   设备号 (25 / 26)
  * @param  pDistMm    输出: 距离 (mm), 无效时为 9999.0
  * @retval HAL_OK 成功, HAL_ERROR 设备号无效
  */
HAL_StatusTypeDef TOF8_GetDistance(uint8_t deviceId, float *pDistMm);

/**
  * @brief  多次采样取中值, 抑制 TOF 噪声尖峰
  * @param  deviceId   设备号 (25 / 26)
  * @param  samples    采样次数 (建议 5~7, 最大 7)
  * @param  intervalMs 采样间隔 (ms)
  * @return 中值距离 (mm), 全部无效返回 TOF8_DISTANCE_INVALID_MM
  */
float TOF8_GetDistanceMedian(uint8_t deviceId, uint8_t samples, uint32_t intervalMs);

/**
  * @brief  检查指定 TOF 传感器是否在线 (有数据更新)
  * @param  deviceId   设备号 (25 / 26)
  * @param  timeoutMs  超时时间 (ms), 超过此时间无更新视为离线
  * @retval 1=在线, 0=无数据 (故障或未启动)
  */
uint8_t TOF8_IsAlive(uint8_t deviceId, uint32_t timeoutMs);

/**
  * @brief  确保指定 TOF 传感器在线, 不在线则尝试复位重启
  * @param  deviceId   设备号 (25 / 26)
  * @param  timeoutMs  在线判定超时 (ms)
  * @param  maxRetry   最大重启重试次数
  * @retval HAL_OK=在线, HAL_ERROR=重启后仍无数据
  */
HAL_StatusTypeDef TOF8_EnsureAlive(uint8_t deviceId, uint32_t timeoutMs, uint8_t maxRetry);

/**
  * @brief  检查指定 TOF 传感器的滤波器是否处于预热期
  *         无效帧重置滤波器后, 需连续若干有效采样才能恢复可信距离;
  *         预热期间距离读数为无效值, 但目标可能实际在量程内, 应等待而非盲进
  * @param  deviceId  设备号 (25 / 26)
  * @retval 1=预热中(应等待重测), 0=非预热
  */
uint8_t TOF8_IsWarmingUp(uint8_t deviceId);

/**
  * @brief  获取指定设备的状态结构体指针
  * @param  deviceId  设备号 (25 / 26)
  * @retval 状态指针, 设备号无效时返回 NULL
  */
volatile TOF8_Status_t *TOF8_GetStatus(uint8_t deviceId);

#ifdef __cplusplus
}
#endif

#endif /* __TOF8_H */
