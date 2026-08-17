/**
  ******************************************************************************
  * @file    imu_core.c
  * @brief   IMU 计算层实现: 封装数据处理链路
  *
  *         数据处理链路:
  *           原始角速度 → ① 减零偏 → ② IIR 滤波 → ③ 中值滤波
  *                      → ④ 死区 → ⑤ 梯形积分 → ⑥ 角度归一化
  *
  *         算法逻辑与重构前 imu.c Task_Main 完全一致，仅做结构重组。
  ******************************************************************************
  */

#include "imu_core.h"
#include "xv7001bb.h"
#include <stddef.h>

/* ======================== 初始化 ======================== */

/**
  * @brief  初始化 IMU 引擎
  */
void IMU_Engine_Init(IMU_Engine_t *pEngine, const IMU_Engine_Config_t *pConfig)
{
    if (pEngine == NULL || pConfig == NULL) return;

    /* 复制配置 */
    pEngine->config = *pConfig;

    /* 清零运行时状态 */
    pEngine->biasDps     = 0.0f;
    pEngine->deadZone    = pConfig->deadZoneMin;
    pEngine->angleDeg    = 0.0f;
    pEngine->prevRateDps = 0.0f;

    /* 初始化滤波器 */
    IIR_Filter_Init(&pEngine->iirFilter, pConfig->iirFc, pConfig->iirFs);
    Median_Filter_Init(&pEngine->medFilter);
}

/* ======================== 零偏管理 ======================== */

void IMU_Engine_SetBias(IMU_Engine_t *pEngine, float biasDps)
{
    if (pEngine != NULL)
    {
        pEngine->biasDps = biasDps;
    }
}

float IMU_Engine_GetBias(const IMU_Engine_t *pEngine)
{
    if (pEngine == NULL) return 0.0f;
    return pEngine->biasDps;
}

/* ======================== 角度置零 ======================== */

void IMU_Engine_ZeroAngle(IMU_Engine_t *pEngine)
{
    if (pEngine != NULL)
    {
        pEngine->angleDeg = 0.0f;
    }
}

/* ======================== 重置状态 ======================== */

void IMU_Engine_Reset(IMU_Engine_t *pEngine)
{
    if (pEngine == NULL) return;

    pEngine->angleDeg    = 0.0f;
    pEngine->prevRateDps = 0.0f;

    IIR_Filter_Reset(&pEngine->iirFilter);
    Median_Filter_Reset(&pEngine->medFilter);
}

/* ======================== 自适应死区 ======================== */

void IMU_Engine_UpdateDeadZone(IMU_Engine_t *pEngine, float maxAbsRate)
{
    if (pEngine == NULL) return;

    float dz = maxAbsRate + pEngine->config.deadZoneMargin;

    if (dz < pEngine->config.deadZoneMin)
    {
        dz = pEngine->config.deadZoneMin;
    }
    if (dz > pEngine->config.deadZoneMax)
    {
        dz = pEngine->config.deadZoneMax;
    }

    pEngine->deadZone = dz;
}

/* ======================== 单次采样处理 ======================== */

/**
  * @brief  单次采样处理
  *
  *         对应原 imu.c Task_Main 主循环中的数据处理部分:
  *           rawRateDps = rawAngle / SCALE_FACTOR
  *           rateDps    = rawRateDps - bias
  *           rateDps    = IIR(rateDps)
  *           rateDps    = Median(rateDps)
  *           if |rateDps| < deadZone → rateDps = 0, reset IIR
  *           angleIncrement = (rateDps + prevRateDps) / 2 × dt
  *           angleDeg  += angleIncrement
  *           realAngle  = angleDeg × scale, 归一化到 [-180, 180)
  */
HAL_StatusTypeDef IMU_Engine_Update(IMU_Engine_t *pEngine, int16_t rawAngle,
                                     IMU_Engine_Output_t *pOutput)
{
    if (pEngine == NULL || pOutput == NULL)
    {
        return HAL_ERROR;
    }

    /* ① 原始角速度换算 + 去零偏 */
    float rawRateDps = (float)rawAngle / XV7001BB_SCALE_FACTOR_16BIT;
    float rateDps    = rawRateDps - pEngine->biasDps;

    /* ② IIR 低通滤波 */
    rateDps = IIR_Filter_Update(&pEngine->iirFilter, rateDps);

    /* ③ 中值滤波 (消除脉冲尖峰) */
    rateDps = Median_Filter_Update(&pEngine->medFilter, rateDps);

    /* ④ 死区处理 */
    if (rateDps > -pEngine->deadZone && rateDps < pEngine->deadZone)
    {
        rateDps = 0.0f;
        IIR_Filter_Reset(&pEngine->iirFilter);
    }

    /* ⑤ 梯形积分: angle += (当前 + 上次) / 2 × dt */
    float angleIncrement = (rateDps + pEngine->prevRateDps) * 0.5f * pEngine->config.dtSec;
    pEngine->angleDeg += angleIncrement;
    pEngine->prevRateDps = rateDps;

    /* ⑥ 真实角度 = 积分角度 × scale, 归一化到 [-180°, +180°) */
    float realAngleDeg = pEngine->angleDeg * pEngine->config.angleScale;

    while (realAngleDeg >= 180.0f)
    {
        realAngleDeg -= 360.0f;
    }
    while (realAngleDeg < -180.0f)
    {
        realAngleDeg += 360.0f;
    }

    /* 存回积分角度 (除以 scale, 保持积分连续性) */
    pEngine->angleDeg = realAngleDeg / pEngine->config.angleScale;

    /* 原始角度增量 (未去零偏, 用于诊断帧) */
    float rawAngleIncrement = rawRateDps * pEngine->config.dtSec;

    /* 填充输出 */
    pOutput->rateDps     = rateDps;
    pOutput->rawRateDps  = rawRateDps;
    pOutput->angleDeg    = realAngleDeg;
    pOutput->rawAngleInc = rawAngleIncrement;

    return HAL_OK;
}

/* ======================== 死区监测 ======================== */

/**
  * @brief  死区监测: 校准后连续采样, 统计滤波后角速度的最大绝对值
  *
  *         对应原 imu.c Task_Main 中校准后的自适应死区监测循环:
  *           - 使用独立的 Median_Filter (不影响主中值滤波器状态)
  *           - 复用主 IIR 滤波器 (监测后由调用者调 Reset 清零)
  *           - 每次等待信号量 → 读传感器 → 换算 → IIR → 中值 → 取绝对值 → 更新 max
  */
HAL_StatusTypeDef IMU_Engine_MonitorDeadZone(IMU_Engine_t *pEngine,
                                              SemaphoreHandle_t sampleSem,
                                              float *pMaxAbs)
{
    if (pEngine == NULL || sampleSem == NULL || pMaxAbs == NULL)
    {
        return HAL_ERROR;
    }

    float maxAbsRate = 0.0f;

    /* 使用独立的中值滤波器 (不影响主滤波器状态) */
    Median_Filter_t monMedian;
    Median_Filter_Init(&monMedian);

    for (uint32_t i = 0; i < pEngine->config.deadZoneMonCount; i++)
    {
        /* 等待下一个采样 */
        if (xSemaphoreTake(sampleSem, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        int16_t monRaw;
        if (XV7001bb_ReadAngle(&monRaw) != HAL_OK)
        {
            continue;
        }

        float monRate = (float)monRaw / XV7001BB_SCALE_FACTOR_16BIT - pEngine->biasDps;
        monRate = IIR_Filter_Update(&pEngine->iirFilter, monRate);
        monRate = Median_Filter_Update(&monMedian, monRate);

        float absRate = (monRate >= 0.0f) ? monRate : -monRate;
        if (absRate > maxAbsRate)
        {
            maxAbsRate = absRate;
        }
    }

    *pMaxAbs = maxAbsRate;
    return HAL_OK;
}
