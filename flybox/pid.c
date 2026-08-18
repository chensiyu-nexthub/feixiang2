/**
  ******************************************************************************
  * @file    pid.c
  * @brief   通用 PID 控制器实现
  *
  *          output = Kp × e + Ki × ∫e·dt + Kd × de/dt
  *
  *          含:
  *            - 积分限幅 (anti-windup): |integral| ≤ integralMax
  *            - 输出限幅: outputMin ≤ output ≤ outputMax
  *            - 误差死区: |error| < deadZone → error = 0
  ******************************************************************************
  */
打算离开就会立刻脚后跟范德萨

#include "pid.h"

/* ======================== 初始化 ======================== */

void PID_Init(PID_Controller_t *pCtrl,
              float kp, float ki, float kd,
              float outMin, float outMax,
              float integralMax, float deadZone)
{
    if (pCtrl == 0) return;

    pCtrl->Kp          = kp;
    pCtrl->Ki          = ki;
    pCtrl->Kd          = kd;
    pCtrl->outputMin   = outMin;
    pCtrl->outputMax   = outMax;
    pCtrl->integralMax = integralMax;
    pCtrl->deadZone    = deadZone;

    /* 清零运行时状态 */
    pCtrl->integral    = 0.0f;
    pCtrl->prevError   = 0.0f;
    pCtrl->lastOutput  = 0.0f;
    pCtrl->initialized = 1U;
}

/* ======================== 更新 ======================== */

float PID_Update(PID_Controller_t *pCtrl, float error, float dt)
{
    if (pCtrl == 0 || dt <= 0.0f) return 0.0f;

    /* 死区处理 */
    if (error > -pCtrl->deadZone && error < pCtrl->deadZone)
    {
        error = 0.0f;
    }

    /* 比例项 */
    float pTerm = pCtrl->Kp * error;

    /* 积分项 (含 anti-windup) */
    pCtrl->integral += error * dt;

    if (pCtrl->integralMax > 0.0f)
    {
        if (pCtrl->integral > pCtrl->integralMax)
        {
            pCtrl->integral = pCtrl->integralMax;
        }
        else if (pCtrl->integral < -pCtrl->integralMax)
        {
            pCtrl->integral = -pCtrl->integralMax;
        }
    }
    float iTerm = pCtrl->Ki * pCtrl->integral;

    /* 微分项 */
    float dTerm = pCtrl->Kd * (error - pCtrl->prevError) / dt;
    pCtrl->prevError = error;

    /* 总输出 + 限幅 */
    float output = pTerm + iTerm + dTerm;

    if (output > pCtrl->outputMax)
    {
        output = pCtrl->outputMax;
    }
    else if (output < pCtrl->outputMin)
    {
        output = pCtrl->outputMin;
    }

    pCtrl->lastOutput = output;
    return output;
}

/* ======================== 重置 ======================== */

void PID_Reset(PID_Controller_t *pCtrl)
{
    if (pCtrl == 0) return;

    pCtrl->integral   = 0.0f;
    pCtrl->prevError  = 0.0f;
    pCtrl->lastOutput = 0.0f;
}
