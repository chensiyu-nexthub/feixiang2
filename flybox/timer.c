/**
  ******************************************************************************
  * @file    timer.c
  * @brief   TIM2 硬件定时器驱动实现: 200Hz 精确采样触发
  *
  *         时钟计算:
  *           PCLK1 = 36MHz (APB1 分频器 = /2)
  *           TIM2CLK = PCLK1 × 2 = 72MHz (APB1 分频≠1 时自动×2)
  *           PSC = 7200-1 → 计数器频率 = 72MHz / 7200 = 10kHz
  *           ARR = 50-1   → 中断周期 = 50 / 10kHz = 5ms = 200Hz
  ******************************************************************************
  */

#include "timer.h"
#include <FreeRTOS.h>
#include <semphr.h>

/* TIM2 句柄 ----------------------------------------------------------------*/
static TIM_HandleTypeDef htim2;

/* 采样信号量: 定时器中断中释放，Task_Main 中等待 */
static SemaphoreHandle_t g_sample_sem = NULL;

/**
  * @brief  初始化 TIM2 为 200Hz 采样定时器
  */
HAL_StatusTypeDef TIM2_SampleTimer_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* 创建二值信号量 */
    if (g_sample_sem == NULL)
    {
        g_sample_sem = xSemaphoreCreateBinary();
        if (g_sample_sem == NULL)
        {
            return HAL_ERROR;
        }
    }

    /* 使能 TIM2 时钟 */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 基础配置: 200Hz 更新中断 */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 7200U - 1U;   /* 72MHz / 7200 = 10kHz */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 50U - 1U;      /* 10kHz / 50 = 200Hz */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_StatusTypeDef status = HAL_TIM_Base_Init(&htim2);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 配置 NVIC: 优先级 5 (FreeRTOS configMAX_PRIORITIES=8, 可用 0~7) */
    HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    return HAL_OK;
}

/**
  * @brief  启动采样定时器 (使能更新中断并开始计数)
  */
void TIM2_SampleTimer_Start(void)
{
    HAL_TIM_Base_Start_IT(&htim2);
}

/**
  * @brief  停止采样定时器
  */
void TIM2_SampleTimer_Stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
}

/**
  * @brief  获取采样信号量句柄
  */
SemaphoreHandle_t TIM2_GetSampleSem(void)
{
    return g_sample_sem;
}

/* ======================== 中断处理 ======================== */

/**
  * @brief  TIM2 全局中断入口 (由 startup_stm32f103xb 向量表调用)
  */
void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

/**
  * @brief  TIM2 更新中断回调 (HAL 库在 HAL_TIM_IRQHandler 中调用)
  *         释放信号量唤醒 Task_Main
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (g_sample_sem != NULL)
        {
            xSemaphoreGiveFromISR(g_sample_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
