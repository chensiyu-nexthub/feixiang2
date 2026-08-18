/**
  ******************************************************************************
  * @file    filter.c
  * @brief   数字滤波器实现 (一阶 IIR 低通 + 5 点中值)
  ******************************************************************************
  */

#include "filter.h"
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ======================== 一阶 IIR 低通滤波器 ======================== */

void IIR_Filter_Init(IIR_Filter_t *pFilter, float fc, float fs)
{
    if (pFilter == NULL) return;

    float dt = 1.0f / fs;
    float rc = 1.0f / (2.0f * M_PI * fc);
    pFilter->alpha  = dt / (rc + dt);
    pFilter->y_prev = 0.0f;
}

float IIR_Filter_Update(IIR_Filter_t *pFilter, float x)
{
    if (pFilter == NULL) return x;

    pFilter->y_prev = pFilter->alpha * x + (1.0f - pFilter->alpha) * pFilter->y_prev;
    return pFilter->y_prev;
}

void IIR_Filter_Reset(IIR_Filter_t *pFilter)
{
    if (pFilter != NULL)
    {
        pFilter->y_prev = 0.0f;
    }
}

/* ======================== 5 点中值滤波器 ======================== */

void Median_Filter_Init(Median_Filter_t *pFilter)
{
    if (pFilter == NULL) return;

    for (int i = 0; i < 5; i++)
    {
        pFilter->buf[i] = 0.0f;
    }
    pFilter->idx = 0;
}

float Median_Filter_Update(Median_Filter_t *pFilter, float val)
{
    if (pFilter == NULL) return val;

    /* 写入窗口 */
    pFilter->buf[pFilter->idx] = val;
    pFilter->idx = (pFilter->idx + 1) % 5;

    /* 复制并排序, 取中值 (冒泡, 5 个元素) */
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
    return tmp[2];  /* 中值 */
}

void Median_Filter_Reset(Median_Filter_t *pFilter)
{
    if (pFilter != NULL)
    {
        for (int i = 0; i < 5; i++) pFilter->buf[i] = 0.0f;
        pFilter->idx = 0;
    }
}