/**
  ******************************************************************************
  * @file    filter.h
  * @brief   一阶 IIR 低通滤波器
  *          y[n] = α × x[n] + (1-α) × y[n-1]
  ******************************************************************************
  */

#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 一阶 IIR 低通滤波器 */
typedef struct
{
    float y_prev;   /* 上一次输出 */
    float alpha;    /* 滤波系数 (0 < α ≤ 1, α 越小滤波越强) */
} IIR_Filter_t;

/**
  * @brief  初始化 IIR 滤波器
  * @param  pFilter  滤波器指针
  * @param  fc       截止频率 (Hz)
  * @param  fs       采样频率 (Hz)
  */
void IIR_Filter_Init(IIR_Filter_t *pFilter, float fc, float fs);

/**
  * @brief  IIR 滤波更新
  * @param  pFilter  滤波器指针
  * @param  x        当前输入
  * @retval 滤波后的输出
  */
float IIR_Filter_Update(IIR_Filter_t *pFilter, float x);

/**
  * @brief  重置滤波器状态 (输出清零)
  * @param  pFilter  滤波器指针
  */
void IIR_Filter_Reset(IIR_Filter_t *pFilter);

/* 5点中值滤波器 */
typedef struct
{
    float   buf[5]; /* 环形缓冲区 */
    uint8_t idx;    /* 当前写入位置 */
} Median_Filter_t;

/**
  * @brief  初始化中值滤波器
  * @param  pFilter  滤波器指针
  */
void Median_Filter_Init(Median_Filter_t *pFilter);

/**
  * @brief  中值滤波更新 (5点窗口，消除脉冲尖峰)
  * @param  pFilter  滤波器指针
  * @param  val      新输入值
  * @retval 中值
  */
float Median_Filter_Update(Median_Filter_t *pFilter, float val);

/**
  * @brief  重置中值滤波器状态
  * @param  pFilter  滤波器指针
  */
void Median_Filter_Reset(Median_Filter_t *pFilter);

#endif /* __FILTER_H */
