/**
  ******************************************************************************
  * @file    timer.h
  * @brief   TIM2 硬件定时器驱动: 200Hz 精确采样触发
  *          用于替代 osDelay，提供 μs 级精度的采样节拍
  ******************************************************************************
  */

#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>
#include <FreeRTOS.h>
#include <semphr.h>

/* 采样周期常量 (秒) */
#define SAMPLE_DT_SEC   0.005f   /* 200Hz → dt = 5ms */

/**
  * @brief  初始化 TIM2 为 200Hz 采样定时器并启动
  *         PCLK1=36MHz, TIM2CLK=72MHz (APB1 分频≠1 时自动×2)
  *         PSC=7200-1 → 10kHz, ARR=50-1 → 200Hz
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef TIM2_SampleTimer_Init(void);

/**
  * @brief  启动采样定时器
  */
void TIM2_SampleTimer_Start(void);

/**
  * @brief  停止采样定时器
  */
void TIM2_SampleTimer_Stop(void);

/**
  * @brief  获取采样信号量句柄 (供 Task_Main 等待)
  * @retval 二值信号量句柄，未初始化时返回 NULL
  */
SemaphoreHandle_t TIM2_GetSampleSem(void);

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */
