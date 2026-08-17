/**
  ******************************************************************************
  * @file    filter.c
  * @brief   一阶 IIR 低通滤波器实现
  *
  *         公式: y[n] = α × x[n] + (1-α) × y[n-1]
  *         α = dt / (RC + dt),  RC = 1 / (2π × fc)
  ******************************************************************************
  */

#include "filter.h"
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/**
  * @brief  初始化 IIR 滤波器
  */
void IIR_Filter_Init(IIR_Filter_t *pFilter, float fc, float fs)
{
    if (pFilter == NULL) return;

    float dt = 1.0f / fs;
    float rc = 1.0f / (2.0f * M_PI * fc);
    pFilter->alpha  = dt / (rc + dt);
    pFilter->y_prev = 0.0f;
}

/**
  * @brief  IIR 滤波更新
  */
float IIR_Filter_Update(IIR_Filter_t *pFilter, float x)
{
    if (pFilter == NULL) return x;

    pFilter->y_prev = pFilter->alpha * x + (1.0f - pFilter->alpha) * pFilter->y_prev;
    return pFilter->y_prev;
}

/**
  * @brief  重置滤波器状态
  */
void IIR_Filter_Reset(IIR_Filter_t *pFilter)
{
    if (pFilter != NULL)
    {
        pFilter->y_prev = 0.0f;
    }
}

/* ======================== 中值滤波器 ======================== */

/**
  * @brief  初始化中值滤波器
  */
void Median_Filter_Init(Median_Filter_t *pFilter)
{
    if (pFilter == NULL) return;

    pFilter->buf[0] = 0.0f;
    pFilter->buf[1] = 0.0f;
    pFilter->buf[2] = 0.0f;
    pFilter->buf[3] = 0.0f;
    pFilter->buf[4] = 0.0f;
    pFilter->idx    = 0;
}

/**
  * @brief  中值滤波更新 (5点窗口，消除脉冲尖峰)
  */
float Median_Filter_Update(Median_Filter_t *pFilter, float val)
{
    if (pFilter == NULL) return val;

    pFilter->buf[pFilter->idx] = val;
    pFilter->idx = (pFilter->idx + 1) % 5;

    /* 复制并排序取中值 (冒泡排序，5个元素) */
    float tmp[5];
    for (int i = 0; i < 5; i++) tmp[i] = pFilter->buf[i];
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4 - i; j++)
        {
            if (tmp[j] > tmp[j + 1])
            {
                float t = tmp[j];
                tmp[j] = tmp[j + 1];
                tmp[j + 1] = t;
            }
        }
    }
    return tmp[2];  /* 中值 (第3个) */
}

/**
  * @brief  重置中值滤波器状态
  */
void Median_Filter_Reset(Median_Filter_t *pFilter)
{
    if (pFilter != NULL)
    {
        for (int i = 0; i < 5; i++) pFilter->buf[i] = 0.0f;
        pFilter->idx = 0;
    }
}
