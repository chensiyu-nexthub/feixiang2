/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用 PID 控制器 (纯数学模块, 无硬件依赖)
  *
  *          特性:
  *            - 积分限幅 (anti-windup)
  *            - 输出限幅
  *            - 误差死区 (小于阈值时不累积)
  *            - dt 由调用者传入 (适配不同控制周期)
  ******************************************************************************
  */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ======================== PID 控制器结构体 ======================== */

typedef struct
{
    /* 参数 (由调用者设置) */
    float Kp;             /* 比例系数 */
    float Ki;             /* 积分系数 */
    float Kd;             /* 微分系数 */
    float outputMin;      /* 输出下限 */
    float outputMax;      /* 输出上限 */
    float integralMax;    /* 积分限幅 (绝对值), 0 = 不限幅 */
    float deadZone;       /* 误差死区: |error| < deadZone 时视为 0 */

    /* 运行时状态 (内部维护) */
    float integral;       /* 积分累积 */
    float prevError;      /* 上次误差 (微分用) */
    float lastOutput;     /* 最近一次输出 */
    uint8_t initialized;  /* 1 = 已初始化 */
} PID_Controller_t;

/* ======================== 接口函数 ======================== */

/**
  * @brief  初始化 PID 控制器 (清零运行时状态)
  * @param  pCtrl  控制器指针
  * @param  kp     比例系数
  * @param  ki     积分系数
  * @param  kd     微分系数
  * @param  outMin 输出下限
  * @param  outMax 输出上限
  * @param  integralMax 积分限幅 (0 = 不限幅)
  * @param  deadZone    误差死区 (0 = 无死区)
  */
void PID_Init(PID_Controller_t *pCtrl,
              float kp, float ki, float kd,
              float outMin, float outMax,
              float integralMax, float deadZone);

/**
  * @brief  PID 更新 (每个控制周期调用一次)
  * @param  pCtrl  控制器指针
  * @param  error  当前误差 (设定值 - 测量值)
  * @param  dt     时间步长 (s)
  * @retval 控制输出 (已限幅)
  */
float PID_Update(PID_Controller_t *pCtrl, float error, float dt);

/**
  * @brief  重置运行时状态 (integral=0, prevError=0)
  *         参数 (Kp/Ki/Kd/限幅) 不变
  * @param  pCtrl  控制器指针
  */
void PID_Reset(PID_Controller_t *pCtrl);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
