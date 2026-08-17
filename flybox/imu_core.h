/**
  ******************************************************************************
  * @file    imu_core.h
  * @brief   IMU 计算层: 封装数据处理链路
  *          去零偏 → IIR → 中值 → 死区 → 梯形积分 → 归一化
  ******************************************************************************
  */

#ifndef __IMU_CORE_H
#define __IMU_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include "filter.h"

/* ======================== 配置结构体 ======================== */

/**
  * @brief  IMU 引擎运行时配置 (创建时传入，运行中可修改)
  */
typedef struct
{
    float    iirFc;             /* IIR 截止频率 (Hz) */
    float    iirFs;             /* 采样频率 (Hz) */
    float    dtSec;             /* 采样周期 (s), 通常 = 1/iirFs */
    float    deadZoneMin;       /* 死区下限 (°/s) */
    float    deadZoneMax;       /* 死区上限 (°/s) */
    float    deadZoneMargin;    /* 死区余量 (°/s), 在最大振动值基础上增加 */
    uint32_t deadZoneMonCount;  /* 死区监测采样数 (校准后) */
    float    angleScale;        /* 真实角度 = 积分角度 × angleScale */
} IMU_Engine_Config_t;

/* ======================== 输出结构体 ======================== */

/**
  * @brief  IMU 引擎单次采样输出 (供 CAN 协议层使用)
  */
typedef struct
{
    float rateDps;       /* 处理后角速度 (°/s, 已滤波+死区) */
    float rawRateDps;    /* 原始角速度 (°/s, 去零偏前) */
    float angleDeg;      /* 真实角度 (°/s, 已×scale, 已归一化到 [-180,180)) */
    float rawAngleInc;   /* 原始角度增量 (°, 未去零偏, 用于诊断帧) */
} IMU_Engine_Output_t;

/* ======================== 引擎结构体 ======================== */

/**
  * @brief  IMU 引擎实例 (包含所有运行时状态)
  */
typedef struct
{
    /* 运行时状态 */
    float    biasDps;           /* 当前零偏 (°/s) */
    float    deadZone;          /* 当前死区阈值 (°/s), 自适应 */
    float    angleDeg;          /* 积分角度 (内部值, 未×scale) */
    float    prevRateDps;       /* 上次角速度 (梯形积分用) */

    /* 滤波器实例 */
    IIR_Filter_t    iirFilter;  /* IIR 低通滤波器 */
    Median_Filter_t medFilter;  /* 中值滤波器 */

    /* 配置 (运行时可修改) */
    IMU_Engine_Config_t config;
} IMU_Engine_t;

/* ======================== 接口函数 ======================== */

/**
  * @brief  初始化 IMU 引擎
  * @param  pEngine  引擎指针
  * @param  pConfig  配置指针 (内容会被复制到引擎内部)
  */
void IMU_Engine_Init(IMU_Engine_t *pEngine, const IMU_Engine_Config_t *pConfig);

/**
  * @brief  设置零偏
  * @param  pEngine  引擎指针
  * @param  biasDps  零偏值 (°/s)
  */
void IMU_Engine_SetBias(IMU_Engine_t *pEngine, float biasDps);

/**
  * @brief  获取当前零偏
  * @param  pEngine  引擎指针
  * @retval 零偏值 (°/s)
  */
float IMU_Engine_GetBias(const IMU_Engine_t *pEngine);

/**
  * @brief  角度置零 (仅 angleDeg = 0, 不影响积分状态)
  * @param  pEngine  引擎指针
  */
void IMU_Engine_ZeroAngle(IMU_Engine_t *pEngine);

/**
  * @brief  重置引擎状态 (滤波器+积分+角度清零, 零偏和死区保留)
  *         校准/死区监测后调用
  * @param  pEngine  引擎指针
  */
void IMU_Engine_Reset(IMU_Engine_t *pEngine);

/**
  * @brief  根据监测到的最大绝对角速度更新自适应死区
  *         死区 = maxAbsRate + margin, 限制在 [min, max] 范围内
  * @param  pEngine     引擎指针
  * @param  maxAbsRate  监测期间滤波后角速度的最大绝对值 (°/s)
  */
void IMU_Engine_UpdateDeadZone(IMU_Engine_t *pEngine, float maxAbsRate);

/**
  * @brief  单次采样处理: 去零偏 → IIR → 中值 → 死区 → 梯形积分 → 归一化
  * @param  pEngine   引擎指针
  * @param  rawAngle  传感器原始角速度值 (16bit, 由 XV7001bb_ReadAngle 返回)
  * @param  pOutput   输出结构体指针
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef IMU_Engine_Update(IMU_Engine_t *pEngine, int16_t rawAngle,
                                     IMU_Engine_Output_t *pOutput);

/**
  * @brief  死区监测: 校准后连续采样, 统计滤波后角速度的最大绝对值
  *         内部等待信号量 + 读传感器 + 滤波, 阻塞运行
  * @param  pEngine    引擎指针 (使用内部 IIR 滤波器, 监测后自动重置)
  * @param  sampleSem  采样信号量句柄 (由调用者传入, 避免引擎层依赖 timer.h)
  * @param  pMaxAbs    输出: 监测期间最大绝对值 (°/s)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef IMU_Engine_MonitorDeadZone(IMU_Engine_t *pEngine,
                                              SemaphoreHandle_t sampleSem,
                                              float *pMaxAbs);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_CORE_H */
