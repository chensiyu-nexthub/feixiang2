/**
  ******************************************************************************
  * @file    filter.h
  * @brief   数字滤波器接口 (一阶 IIR 低通 + 5 点中值)
  ******************************************************************************
  */

#ifndef __FILTER_H
#define __FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== 一阶 IIR 低通滤波器 ======================== */
/* 公式: y[n] = α·x[n] + (1-α)·y[n-1], α = dt / (RC + dt), RC = 1/(2π·fc) */

typedef struct
{
    float alpha;   /* 滤波系数 (0~1, 自动计算) */
    float y_prev;  /* 上一次输出 */
} IIR_Filter_t;

/**
  * @brief  初始化 IIR 滤波器
  * @param  pFilter  滤波器实例
  * @param  fc       截止频率 (Hz)
  * @param  fs       采样频率 (Hz)
  */
void IIR_Filter_Init(IIR_Filter_t *pFilter, float fc, float fs);

/**
  * @brief  IIR 滤波更新
  * @param  pFilter  滤波器实例
  * @param  x        当前输入值
  * @retval 滤波后的输出值
  */
float IIR_Filter_Update(IIR_Filter_t *pFilter, float x);

/**
  * @brief  重置滤波器状态 (y_prev = 0)
  */
void IIR_Filter_Reset(IIR_Filter_t *pFilter);

/* ======================== 5 点中值滤波器 ======================== */
/* 窗口 5 点, 排序取中值, 消除脉冲尖峰 */

typedef struct
{
    float buf[5];  /* 窗口缓冲区 */
    int   idx;     /* 当前写入位置 */
} Median_Filter_t;

/**
  * @brief  初始化中值滤波器
  */
void Median_Filter_Init(Median_Filter_t *pFilter);

/**
  * @brief  中值滤波更新
  * @param  pFilter  滤波器实例
  * @param  val      当前原始值
  * @retval 中值滤波后的值
  */
float Median_Filter_Update(Median_Filter_t *pFilter, float val);

/**
  * @brief  重置中值滤波器 (窗口清零)
  */
void Median_Filter_Reset(Median_Filter_t *pFilter);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H */