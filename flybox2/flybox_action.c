/**
  ******************************************************************************
  * @file    flybox_action.c
  * @brief   飞箱动作控制实现 (力矩回零 / 取箱 / 放箱 / 脱困)
  *
  *          所有电机运动均通过 CAN 总线位置控制:
  *            - 绝对定位: 需要先归零 (Motor_SetZero)
  *            - 相对定位: 不需要归零
  *            - 到位判定: 轮询 mainPosition==0 且 motorStarted
  *
  *          依赖: motor.h, tof8.h, filter.h, FreeRTOS (osDelay)
  ******************************************************************************
  */

#include "flybox_action.h"
#include "motor.h"
#include "tof8.h"
#include <cmsis_os.h>

/* ======================== 全局状态 ======================== */
static FlyBox_State_t s_state = FLYBOX_IDLE;
static uint8_t        s_initialized = 0U;
static uint8_t        s_homed = 0U;

/* 全局诊断变量 */
FlyBox_HomeDiag_t  g_home_diag = {0};
FlyBox_PickDiag_t  g_pick_diag = {0};
FlyBox_WaitDiag_t  g_wait_diag = {0};
FlyBox_Tof26Diag_t g_tof26_diag = {0};

/* Step1 累计皮带前进距离 (供 Step3 使用) */
static float  s_belt_fwd_mm = 0.0f;
static uint8_t s_belt_fwd_limit_hit = 0U;

/* ======================== 内部辅助函数声明 ======================== */
static HAL_StatusTypeDef FlyBox_WaitMotorReached(uint8_t deviceId, uint32_t timeout_ms);
static HAL_StatusTypeDef FlyBox_WaitMultipleReached(const uint8_t *ids, uint8_t count, uint32_t timeout_ms);
static HAL_StatusTypeDef FlyBox_FreeTryMove(uint8_t deviceId, int8_t direction, float distance_mm, uint32_t timeout_ms);
static HAL_StatusTypeDef FlyBox_FreeFromLimit(uint8_t deviceId, int8_t dir, int16_t thresh);
static uint8_t FlyBox_GetHomeConfig(uint8_t deviceId, int8_t *dir, float *search_mm, int16_t *thresh);
static HAL_StatusTypeDef FlyBox_VerifyBoxOnConveyor(void);
static HAL_StatusTypeDef FlyBox_CheckClearanceAndFaceFront(void);
static HAL_StatusTypeDef FlyBox_RotateSyncPick(void);
static HAL_StatusTypeDef FlyBox_RotateSyncPlace(void);
static HAL_StatusTypeDef FlyBox_GoInitState(void);

/* ======================== 状态查询 ======================== */

FlyBox_State_t FlyBox_GetState(void)
{
    return s_state;
}

/* ======================== 内部辅助函数: 力矩回零配置 ======================== */

/**
  * @brief  获取电机力矩回零配置 (方向 / 搜索行程 / 力矩阈值)
  */
static uint8_t FlyBox_GetHomeConfig(uint8_t deviceId, int8_t *dir, float *search_mm, int16_t *thresh)
{
    if (dir == NULL || search_mm == NULL || thresh == NULL) return 0U;

    switch (deviceId)
    {
    case FLYBOX_ID_GRIPPER:
        *dir       = 1;   /* CW */
        *search_mm = FLYBOX_HOME_SPEED_MM_S;
        *thresh    = FLYBOX_HOME_TORQUE_THRESH_GRIPPER;
        return 1U;
    case FLYBOX_ID_GRIPPER_ROTATE:
        *dir       = -1;  /* CCW */
        *search_mm = FLYBOX_HOME_SPEED_MM_S;
        *thresh    = FLYBOX_HOME_TORQUE_THRESH_GRIPPER_ROTATE;
        return 1U;
    case FLYBOX_ID_GRIPPER_BELT:
        *dir       = -1;  /* CCW */
        *search_mm = FLYBOX_HOME_SPEED_MM_S_BELT;
        *thresh    = FLYBOX_HOME_TORQUE_THRESH_GRIPPER_BELT;
        return 1U;
    default:
        return 0U;  /* 传送带不支持力矩回零 */
    }
}

/* ======================== 内部辅助函数: 到位等待 ======================== */

/**
  * @brief  等待单个电机到位 (轮询 mainPosition, 含保护帧豁免)
  */
static HAL_StatusTypeDef FlyBox_WaitMotorReached(uint8_t deviceId, uint32_t timeout_ms)
{
    volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
    if (p == NULL) return HAL_ERROR;

    p->lastErrorCode = 0U;
    uint32_t count0 = p->feedbackCount;

    uint32_t elapsed = 0U;
    const uint32_t poll_ms = 5U;
    const uint32_t start_grace_ms = 300U;
    uint8_t motorStarted = 0U;

    while (elapsed < timeout_ms)
    {
        if (p->mainPosition != 0) motorStarted = 1U;

        uint16_t err = p->lastErrorCode;
        if (err == MOTOR_ERR_STALL || err == MOTOR_ERR_OVERTEMP ||
            err == MOTOR_ERR_UNDERVOLTAGE || err == MOTOR_ERR_LINE_BREAK)
        {
            /* 保护帧豁免: 新反馈帧 + 剩余位移 ≤ 容差 → 视为到位 */
            if (p->feedbackCount != count0 &&
                p->mainPosition <= (int32_t)FLYBOX_STALL_REACHED_TOL_RAW &&
                p->mainPosition >= -(int32_t)FLYBOX_STALL_REACHED_TOL_RAW)
            {
                p->lastErrorCode = 0U;
                return HAL_OK;
            }

            g_wait_diag.deviceId     = deviceId;
            g_wait_diag.result       = 1U;
            g_wait_diag.errCode      = err;
            g_wait_diag.mainPosition = p->mainPosition;
            g_wait_diag.motorStarted = motorStarted;
            g_wait_diag.elapsedMs    = elapsed;
            return HAL_ERROR;
        }

        if (p->mainPosition == 0)
        {
            if (motorStarted || elapsed >= start_grace_ms) return HAL_OK;
        }

        osDelay(poll_ms);
        elapsed += poll_ms;
    }

    g_wait_diag.deviceId     = deviceId;
    g_wait_diag.result       = 3U;
    g_wait_diag.errCode      = p->lastErrorCode;
    g_wait_diag.mainPosition = p->mainPosition;
    g_wait_diag.motorStarted = motorStarted;
    g_wait_diag.elapsedMs    = elapsed;
    return HAL_TIMEOUT;
}

/**
  * @brief  等待多个电机同时到位
  */
static HAL_StatusTypeDef FlyBox_WaitMultipleReached(const uint8_t *ids, uint8_t count, uint32_t timeout_ms)
{
    if (count == 0U || count > FLYBOX_MOTOR_COUNT) return HAL_ERROR;

    uint32_t count0[FLYBOX_MOTOR_COUNT] = {0};
    for (uint8_t i = 0U; i < count; i++)
    {
        volatile Motor_Status_t *p = Motor_GetStatus(ids[i]);
        if (p == NULL) return HAL_ERROR;
        p->lastErrorCode = 0U;
        count0[i] = p->feedbackCount;
    }

    uint32_t elapsed = 0U;
    const uint32_t poll_ms = 5U;
    const uint32_t start_grace_ms = 300U;
    uint8_t motorStarted[FLYBOX_MOTOR_COUNT] = {0};

    while (elapsed < timeout_ms)
    {
        uint8_t allReached = 1U;

        for (uint8_t i = 0U; i < count; i++)
        {
            volatile Motor_Status_t *p = Motor_GetStatus(ids[i]);

            if (p->mainPosition != 0) motorStarted[i] = 1U;

            uint16_t err = p->lastErrorCode;
            if (err == MOTOR_ERR_STALL || err == MOTOR_ERR_OVERTEMP ||
                err == MOTOR_ERR_UNDERVOLTAGE || err == MOTOR_ERR_LINE_BREAK)
            {
                if (p->feedbackCount != count0[i] &&
                    p->mainPosition <= (int32_t)FLYBOX_STALL_REACHED_TOL_RAW &&
                    p->mainPosition >= -(int32_t)FLYBOX_STALL_REACHED_TOL_RAW)
                {
                    p->lastErrorCode = 0U;
                    continue;
                }
                else
                {
                    g_wait_diag.deviceId     = ids[i];
                    g_wait_diag.result       = 1U;
                    g_wait_diag.errCode      = err;
                    g_wait_diag.mainPosition = p->mainPosition;
                    g_wait_diag.motorStarted = motorStarted[i];
                    g_wait_diag.elapsedMs    = elapsed;
                    return HAL_ERROR;
                }
            }

            if (p->mainPosition == 0)
            {
                if (!motorStarted[i] && elapsed < start_grace_ms) allReached = 0U;
            }
            else
            {
                allReached = 0U;
            }
        }

        if (allReached) return HAL_OK;

        osDelay(poll_ms);
        elapsed += poll_ms;
    }

    g_wait_diag.deviceId  = ids[0];
    g_wait_diag.result    = 3U;
    g_wait_diag.elapsedMs = elapsed;
    for (uint8_t i = 0U; i < count; i++)
    {
        volatile Motor_Status_t *p = Motor_GetStatus(ids[i]);
        if (p->mainPosition != 0 || !motorStarted[i])
        {
            g_wait_diag.deviceId     = ids[i];
            g_wait_diag.errCode      = p->lastErrorCode;
            g_wait_diag.mainPosition = p->mainPosition;
            g_wait_diag.motorStarted = motorStarted[i];
            break;
        }
    }
    return HAL_TIMEOUT;
}

/* ======================== 内部辅助函数: 脱困 ======================== */

/**
  * @brief  脱困尝试: 急停 → Unlock/Lock → 相对运动 → 等到位
  */
static HAL_StatusTypeDef FlyBox_FreeTryMove(uint8_t deviceId, int8_t direction, float distance_mm, uint32_t timeout_ms)
{
    volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
    if (p == NULL) return HAL_ERROR;

    Motor_EmergencyStop(deviceId);
    osDelay(50);
    Motor_Unlock(deviceId);
    osDelay(100);
    Motor_Lock(deviceId);
    osDelay(100);

    Motor_SetSpeed(deviceId, FLYBOX_HOME_SEARCH_SPEED);
    Motor_SetAcceleration(deviceId, FLYBOX_HOME_SEARCH_ACCEL);
    osDelay(10);

    p->lastErrorCode = 0U;
    float target = (direction > 0) ? distance_mm : -distance_mm;
    if (Motor_SetTarget(deviceId, target, MOTOR_TARGET_RELATIVE) != HAL_OK) return HAL_ERROR;

    uint32_t elapsed = 0U;
    const uint32_t poll_ms = 10U;
    uint8_t motorStarted = 0U;

    while (elapsed < timeout_ms)
    {
        if (p->mainPosition != 0) motorStarted = 1U;

        uint16_t err = p->lastErrorCode;
        if (err == MOTOR_ERR_STALL || err == MOTOR_ERR_OVERTEMP ||
            err == MOTOR_ERR_UNDERVOLTAGE || err == MOTOR_ERR_LINE_BREAK)
            return HAL_ERROR;

        if (motorStarted && p->mainPosition == 0) return HAL_OK;

        osDelay(poll_ms);
        elapsed += poll_ms;
    }

    return HAL_TIMEOUT;
}

/**
  * @brief  多级卡滞脱困: 级1 正向探索 / 级2 反向回退 / 级3 摇摆 ±20/60/100mm / 级4 电流 0xFF 摇摆
  */
static HAL_StatusTypeDef FlyBox_FreeFromLimit(uint8_t deviceId, int8_t dir, int16_t thresh)
{
    volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
    if (p == NULL) return HAL_ERROR;

    int8_t restoreDir = (deviceId == FLYBOX_ID_GRIPPER_BELT) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
    const float rockDist[3] = { FLYBOX_HOME_FREE_ROCK1_MM, FLYBOX_HOME_FREE_ROCK2_MM, FLYBOX_HOME_FREE_ROCK3_MM };

    /* 级1: 正向探索 */
    if (FlyBox_FreeTryMove(deviceId, +dir, FLYBOX_HOME_FREE_MM, FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
    {
        osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
        int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
        if (absTq < thresh) { g_home_diag.freeStage = 11U; return HAL_OK; }
    }

    /* 级2: 反向回退 */
    if (FlyBox_FreeTryMove(deviceId, -dir, FLYBOX_HOME_FREE_MM, FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
    {
        osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
        int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
        if (absTq < thresh) { g_home_diag.freeStage = 12U; return HAL_OK; }
    }

    /* 级3: 摇摆 ± 递增 */
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (FlyBox_FreeTryMove(deviceId, -dir, rockDist[i], FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
        {
            osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
            int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
            if (absTq < thresh) { g_home_diag.freeStage = 13U; return HAL_OK; }
        }
        if (FlyBox_FreeTryMove(deviceId, +dir, rockDist[i], FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
        {
            osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
            int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
            if (absTq < thresh) { g_home_diag.freeStage = 13U; return HAL_OK; }
        }
    }

    /* 级4: 电流限制 0xFF + 摇摆 */
    Motor_ConfigParamCustom(deviceId, deviceId, restoreDir,
                            0xB4U, 0xBFU, 0xCFU, 0xFFU, 0x03U, 0x5FU);
    osDelay(50);

    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (FlyBox_FreeTryMove(deviceId, -dir, rockDist[i], FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
        {
            osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
            int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
            if (absTq < thresh)
            {
                /* 恢复原配置 */
                if (deviceId == FLYBOX_ID_GRIPPER)
                    Motor_ConfigParamCustom(deviceId, deviceId, restoreDir,
                        FLYBOX_GRIPPER_CFG_BYTE0, FLYBOX_GRIPPER_CFG_BYTE1,
                        FLYBOX_GRIPPER_CFG_BYTE2, FLYBOX_GRIPPER_CFG_BYTE3,
                        FLYBOX_GRIPPER_CFG_BYTE4, FLYBOX_GRIPPER_CFG_BYTE5);
                else
                    Motor_ConfigParam(deviceId, deviceId, restoreDir);
                g_home_diag.freeStage = 14U;
                return HAL_OK;
            }
        }
        if (FlyBox_FreeTryMove(deviceId, +dir, rockDist[i], FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
        {
            osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
            int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
            if (absTq < thresh)
            {
                if (deviceId == FLYBOX_ID_GRIPPER)
                    Motor_ConfigParamCustom(deviceId, deviceId, restoreDir,
                        FLYBOX_GRIPPER_CFG_BYTE0, FLYBOX_GRIPPER_CFG_BYTE1,
                        FLYBOX_GRIPPER_CFG_BYTE2, FLYBOX_GRIPPER_CFG_BYTE3,
                        FLYBOX_GRIPPER_CFG_BYTE4, FLYBOX_GRIPPER_CFG_BYTE5);
                else
                    Motor_ConfigParam(deviceId, deviceId, restoreDir);
                g_home_diag.freeStage = 14U;
                return HAL_OK;
            }
        }
    }

    /* 恢复原配置 */
    if (deviceId == FLYBOX_ID_GRIPPER)
        Motor_ConfigParamCustom(deviceId, deviceId, restoreDir,
            FLYBOX_GRIPPER_CFG_BYTE0, FLYBOX_GRIPPER_CFG_BYTE1,
            FLYBOX_GRIPPER_CFG_BYTE2, FLYBOX_GRIPPER_CFG_BYTE3,
            FLYBOX_GRIPPER_CFG_BYTE4, FLYBOX_GRIPPER_CFG_BYTE5);
    else
        Motor_ConfigParam(deviceId, deviceId, restoreDir);

    Motor_EmergencyStop(deviceId);
    osDelay(100);
    Motor_Unlock(deviceId);
    g_home_diag.freeStage = 9U;
    return HAL_ERROR;
}

/* ======================== 力矩回零 ======================== */

/**
  * @brief  力矩回零 (完整流程: 搜索 → 消抖 → 反退 → 判断 → 脱困)
  */
HAL_StatusTypeDef FlyBox_HomeMotorByTorque(uint8_t deviceId)
{
    volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
    if (p == NULL) return HAL_ERROR;

    int8_t    dir;
    float     search_mm;
    int16_t   thresh;
    if (!FlyBox_GetHomeConfig(deviceId, &dir, &search_mm, &thresh)) return HAL_ERROR;

    g_home_diag.deviceId = deviceId;
    g_home_diag.phase    = 0U;
    g_home_diag.failReason = 0U;
    g_home_diag.zeroAtLimit = 0U;
    g_home_diag.errCode  = 0U;
    g_home_diag.torqueAtLimit = 0;
    g_home_diag.torquePeak = 0;
    g_home_diag.searchMs = 0U;
    g_home_diag.backoffElapsed = 0U;
    g_home_diag.backoffPosition = 0;
    g_home_diag.backoffTorque = 0;
    g_home_diag.freeStage = 0U;
    g_home_diag.retryCount = 0U;

    g_home_diag.phase = 1U;

    /* 1. 设置方向 */
    if (Motor_SetDirection(deviceId, dir) != HAL_OK) { g_home_diag.failReason = 1U; return HAL_ERROR; }
    osDelay(10);

    /* 2. 低速搜索 */
    Motor_SetSpeed(deviceId, FLYBOX_HOME_SEARCH_SPEED);
    Motor_SetAcceleration(deviceId, FLYBOX_HOME_SEARCH_ACCEL);
    osDelay(10);

    p->lastErrorCode = 0U;
    if (Motor_SetTarget(deviceId, (dir > 0) ? search_mm : -search_mm,
                        MOTOR_TARGET_RELATIVE) != HAL_OK) { g_home_diag.failReason = 2U; return HAL_ERROR; }

    uint32_t searchStart = HAL_GetTick();
    uint8_t  torqueDebounce = 0U;
    int16_t  torquePeak = 0;
    uint32_t torqueAtLimitMs = 0U;
    int16_t  torqueAtLimitVal = 0;

    while (1)
    {
        uint32_t elapsed = HAL_GetTick() - searchStart;
        if (elapsed > FLYBOX_HOME_TIMEOUT_MS)
        {
            Motor_EmergencyStop(deviceId);
            g_home_diag.searchMs = elapsed;
            g_home_diag.failReason = 3U;
            g_home_diag.torquePeak = torquePeak;
            return HAL_TIMEOUT;
        }

        int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
        if (absTq > torquePeak) torquePeak = absTq;

        if (absTq >= thresh)
        {
            torqueDebounce++;
            if (torqueDebounce == 1U)
            {
                torqueAtLimitMs  = elapsed;
                torqueAtLimitVal = absTq;
            }
            if (torqueDebounce >= FLYBOX_HOME_TORQUE_DEBOUNCE)
            {
                Motor_EmergencyStop(deviceId);
                g_home_diag.searchMs = elapsed;
                g_home_diag.torqueAtLimit = torqueAtLimitVal;
                g_home_diag.torquePeak = torquePeak;
                break;
            }
        }
        else
        {
            torqueDebounce = 0U;
        }

        osDelay(5);
    }

    g_home_diag.phase = 2U;

    /* 3. 反退 */
    Motor_SetSpeed(deviceId, FLYBOX_HOME_SEARCH_SPEED);
    Motor_SetAcceleration(deviceId, FLYBOX_HOME_SEARCH_ACCEL);
    osDelay(10);

    p->lastErrorCode = 0U;
    if (Motor_SetTarget(deviceId, (dir > 0) ? -FLYBOX_HOME_BACKOFF_MM : FLYBOX_HOME_BACKOFF_MM,
                        MOTOR_TARGET_RELATIVE) != HAL_OK) { g_home_diag.failReason = 4U; return HAL_ERROR; }

    if (FlyBox_WaitMotorReached(deviceId, FLYBOX_MOTOR_WAIT_TIMEOUT_MS) != HAL_OK)
    { g_home_diag.phase = 3U; goto FREE; }

    g_home_diag.backoffElapsed  = HAL_GetTick() - searchStart;
    g_home_diag.backoffPosition = p->mainPosition;
    g_home_diag.backoffTorque   = p->torque;

    /* 4. 判断: 反退后力矩是否回落 */
    {
        int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
        if (absTq < thresh)
        {
            /* 正常回零 */
            Motor_SetZero(deviceId);
            g_home_diag.phase = 4U;
            g_home_diag.zeroAtLimit = 1U;
            return HAL_OK;
        }
    }

FREE:
    /* 5. 脱困 */
    g_home_diag.phase = 5U;
    for (uint8_t retry = 0U; retry <= FLYBOX_HOME_RETRY_MAX; retry++)
    {
        g_home_diag.retryCount = retry;
        if (FlyBox_FreeFromLimit(deviceId, dir, thresh) == HAL_OK)
        {
            Motor_SetZero(deviceId);
            g_home_diag.phase = 6U;
            g_home_diag.zeroAtLimit = 0U;
            return HAL_OK;
        }
        osDelay(200);
    }

    g_home_diag.failReason = 5U;
    return HAL_ERROR;
}

/* ======================== 初始化 ======================== */

HAL_StatusTypeDef FlyBox_Init(void)
{
    if (s_initialized) return HAL_OK;

    /* 初始化所有电机 (参数从 Flash 加载, 此处为首次上电兜底) */
    if (Motor_InitAll(FLYBOX_MOTOR_COUNT, FLYBOX_WHEEL_DIAMETER_CONVEYOR,
                      FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL, 3000U) != HAL_OK)
        return HAL_ERROR;

    s_initialized = 1U;
    return HAL_OK;
}

/* ======================== 回零 ======================== */

HAL_StatusTypeDef FlyBox_GripperHome(void)
{
    return FlyBox_HomeMotorByTorque(FLYBOX_ID_GRIPPER);
}

HAL_StatusTypeDef FlyBox_GripperRotateHome(void)
{
    return FlyBox_HomeMotorByTorque(FLYBOX_ID_GRIPPER_ROTATE);
}

HAL_StatusTypeDef FlyBox_GripperBeltHome(void)
{
    return FlyBox_HomeMotorByTorque(FLYBOX_ID_GRIPPER_BELT);
}

HAL_StatusTypeDef FlyBox_HomeAll(void)
{
    s_state = FLYBOX_BUSY;

    /* 传送带只需 Unlock, 不需要力矩回零 */
    for (uint8_t i = FLYBOX_ID_CONVEYOR_L; i <= FLYBOX_ID_CONVEYOR_R; i++)
    {
        Motor_Unlock(i);
        osDelay(50);
        Motor_Lock(i);
        osDelay(50);
    }

    /* 钩爪 / 旋转 / 皮带 力矩回零 */
    const uint8_t homeOrder[] = { FLYBOX_ID_GRIPPER, FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_ID_GRIPPER_BELT };
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (FlyBox_HomeMotorByTorque(homeOrder[i]) != HAL_OK)
        {
            s_state = FLYBOX_ERROR;
            return HAL_ERROR;
        }
        osDelay(300);
    }

    s_homed = 1U;

    /* 初始化 TOF */
    TOF8_StartAll();

    /* 回零后进入初始姿态 */
    if (FlyBox_GoInitState() != HAL_OK)
    {
        s_state = FLYBOX_ERROR;
        return HAL_ERROR;
    }

    s_state = FLYBOX_COMPLETE;
    return HAL_OK;
}

/**
  * @brief  回零后设置初始姿态: 钩爪松开 + 旋转到 0°
  */
static HAL_StatusTypeDef FlyBox_GoInitState(void)
{
    Motor_SetSpeed(FLYBOX_ID_GRIPPER, FLYBOX_DEFAULT_SPEED);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER, FLYBOX_DEFAULT_ACCEL);
    osDelay(10);

    FlyBox_GripperRelease();
    osDelay(100);

    FlyBox_GripperRotateAbs(0.0f, FLYBOX_ROTATE_SPEED, FLYBOX_ROTATE_ACCEL);
    osDelay(100);

    /* 皮带归零 */
    Motor_SetSpeed(FLYBOX_ID_GRIPPER_BELT, FLYBOX_DEFAULT_SPEED);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_BELT, FLYBOX_DEFAULT_ACCEL);
    osDelay(10);
    if (Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, 0.0f, MOTOR_TARGET_ABSOLUTE) != HAL_OK)
        return HAL_ERROR;
    if (FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER_BELT, FLYBOX_MOTOR_WAIT_TIMEOUT_MS) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/* ======================== 钩爪控制 ======================== */

HAL_StatusTypeDef FlyBox_GripperHook(void)
{
    Motor_SetSpeed(FLYBOX_ID_GRIPPER, FLYBOX_DEFAULT_SPEED);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER, FLYBOX_DEFAULT_ACCEL);
    osDelay(10);

    float target_mm = FLYBOX_GRIPPER_DEG_TO_MM(FLYBOX_GRIPPER_HOOK_DEG);
    if (Motor_SetTarget(FLYBOX_ID_GRIPPER, target_mm, MOTOR_TARGET_ABSOLUTE) != HAL_OK) return HAL_ERROR;

    return FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
}

HAL_StatusTypeDef FlyBox_GripperRelease(void)
{
    Motor_SetSpeed(FLYBOX_ID_GRIPPER, FLYBOX_DEFAULT_SPEED);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER, FLYBOX_DEFAULT_ACCEL);
    osDelay(10);

    float target_mm = FLYBOX_GRIPPER_DEG_TO_MM(FLYBOX_GRIPPER_RELEASE_DEG);
    if (Motor_SetTarget(FLYBOX_ID_GRIPPER, target_mm, MOTOR_TARGET_ABSOLUTE) != HAL_OK) return HAL_ERROR;

    return FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
}

/**
  * @brief  钩爪旋转到绝对角度 (0° = 正面, 90° = 侧面)
  */
HAL_StatusTypeDef FlyBox_GripperRotateAbs(float angle_deg, float speed_mm_s, float accel_mm_s2)
{
    Motor_SetSpeed(FLYBOX_ID_GRIPPER_ROTATE, speed_mm_s);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_ROTATE, accel_mm_s2);
    osDelay(10);

    float target_mm = FLYBOX_GRIPPER_ROTATE_DEG_TO_MM(angle_deg);
    if (Motor_SetTarget(FLYBOX_ID_GRIPPER_ROTATE, target_mm, MOTOR_TARGET_ABSOLUTE) != HAL_OK) return HAL_ERROR;

    return FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
}

/**
  * @brief  钩爪旋转相对角度 (逆时针为正)
  */
HAL_StatusTypeDef FlyBox_GripperTurnDeg(float angle_deg, float speed_mm_s, float accel_mm_s2)
{
    Motor_SetSpeed(FLYBOX_ID_GRIPPER_ROTATE, speed_mm_s);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_ROTATE, accel_mm_s2);
    osDelay(10);

    float target_mm = FLYBOX_GRIPPER_ROTATE_DEG_TO_MM(angle_deg);
    if (Motor_SetTarget(FLYBOX_ID_GRIPPER_ROTATE, target_mm, MOTOR_TARGET_RELATIVE) != HAL_OK) return HAL_ERROR;

    return FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
}

/**
  * @brief  钩爪旋转相对角度 (顺时针为正, 兼容旧接口)
  */
HAL_StatusTypeDef FlyBox_GripperRotateDeg(float angle_deg, float speed_mm_s, float accel_mm_s2)
{
    return FlyBox_GripperTurnDeg(-angle_deg, speed_mm_s, accel_mm_s2);
}

/* ======================== 皮带控制 ======================== */

HAL_StatusTypeDef FlyBox_GripperBelttoHome(void)
{
    return FlyBox_HomeMotorByTorque(FLYBOX_ID_GRIPPER_BELT);
}

HAL_StatusTypeDef FlyBox_GripperBeltMove(float distance_mm, float speed_mm_s, float accel_mm_s2)
{
    Motor_SetSpeed(FLYBOX_ID_GRIPPER_BELT, speed_mm_s);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_BELT, accel_mm_s2);
    osDelay(10);

    if (Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, distance_mm, MOTOR_TARGET_RELATIVE) != HAL_OK) return HAL_ERROR;

    return FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER_BELT, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
}

/**
  * @brief  皮带前进直到 TOF25 读数 ≤ target_mm, 或达到最大限程
  */
HAL_StatusTypeDef FlyBox_GripperBeltMoveUntilTof25(float target_mm, float speed_mm_s, float accel_mm_s2)
{
    Motor_SetSpeed(FLYBOX_ID_GRIPPER_BELT, speed_mm_s);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_BELT, accel_mm_s2);
    osDelay(10);

    /* 先走最大限程, 中途 TOF 检测到目标则急停 */
    volatile Motor_Status_t *p = Motor_GetStatus(FLYBOX_ID_GRIPPER_BELT);
    if (p == NULL) return HAL_ERROR;

    p->lastErrorCode = 0U;
    if (Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, FLYBOX_BELT_MAX_FWD_MM,
                        MOTOR_TARGET_RELATIVE) != HAL_OK) return HAL_ERROR;

    uint32_t elapsed = 0U;
    const uint32_t poll_ms = 10U;
    uint8_t motorStarted = 0U;
    uint8_t tofTriggered = 0U;

    while (elapsed < FLYBOX_MOTOR_WAIT_TIMEOUT_MS)
    {
        float dist = TOF8_DISTANCE_INVALID_MM;
        TOF8_GetDistance(TOF8_DEVICE_ID_0, &dist);

        if (dist != TOF8_DISTANCE_INVALID_MM && dist <= target_mm)
        {
            Motor_EmergencyStop(FLYBOX_ID_GRIPPER_BELT);
            tofTriggered = 1U;
            break;
        }

        if (p->mainPosition != 0) motorStarted = 1U;

        uint16_t err = p->lastErrorCode;
        if (err == MOTOR_ERR_STALL || err == MOTOR_ERR_OVERTEMP ||
            err == MOTOR_ERR_UNDERVOLTAGE || err == MOTOR_ERR_LINE_BREAK)
        {
            if (p->feedbackCount != 0U && p->mainPosition == 0) break;
            return HAL_ERROR;
        }

        if (motorStarted && p->mainPosition == 0) break;

        osDelay(poll_ms);
        elapsed += poll_ms;
    }

    if (tofTriggered)
    {
        osDelay(FLYBOX_STEP_SETTLE_MS);
        return HAL_OK;
    }

    return HAL_OK;
}

/* ======================== 传送带控制 ======================== */

HAL_StatusTypeDef FlyBox_ConveyorRun(float distance_mm, float speed_mm_s, float accel_mm_s2)
{
    const uint8_t ids[] = { FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R };

    for (uint8_t i = 0U; i < 2U; i++)
    {
        Motor_SetSpeed(ids[i], speed_mm_s);
        Motor_SetAcceleration(ids[i], accel_mm_s2);
        osDelay(5);
    }

    for (uint8_t i = 0U; i < 2U; i++)
    {
        if (Motor_SetTarget(ids[i], distance_mm, MOTOR_TARGET_RELATIVE) != HAL_OK) return HAL_ERROR;
        osDelay(5);
    }

    return FlyBox_WaitMultipleReached(ids, 2U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
}

/**
  * @brief  传送带运动直到 TOF26 读数 ≤ target_mm (方向由 direction 控制)
  */
HAL_StatusTypeDef FlyBox_ConveyorRunUntilTof26(float target_mm, float speed_mm_s, float accel_mm_s2, int8_t direction)
{
    const uint8_t ids[] = { FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R };
    g_tof26_diag.target    = target_mm;
    g_tof26_diag.direction = direction;
    g_tof26_diag.iters     = 0U;
    g_tof26_diag.warmWaits = 0U;
    g_tof26_diag.blindSteps = 0U;
    g_tof26_diag.exitReason = 0U;

    /* 获取初始距离 */
    float firstDist = TOF8_DISTANCE_INVALID_MM;
    TOF8_GetDistance(TOF8_DEVICE_ID_1, &firstDist);
    g_tof26_diag.firstDist = firstDist;

    for (uint8_t iter = 0U; iter < FLYBOX_TOF_MAX_ITER; iter++)
    {
        g_tof26_diag.iters = iter;

        /* 等待 TOF 预热 */
        if (TOF8_IsWarmingUp(TOF8_DEVICE_ID_1))
        {
            g_tof26_diag.warmWaits++;
            osDelay(FLYBOX_TOF_SETTLE_MS);
            continue;
        }

        float dist = TOF8_DISTANCE_INVALID_MM;
        TOF8_GetDistance(TOF8_DEVICE_ID_1, &dist);

        if (dist != TOF8_DISTANCE_INVALID_MM && dist <= target_mm)
        {
            g_tof26_diag.exitDist   = dist;
            g_tof26_diag.exitReason = 1U;
            return HAL_OK;
        }

        /* 盲进 */
        float step = (dist != TOF8_DISTANCE_INVALID_MM)
                     ? (dist - target_mm) : FLYBOX_TOF_SEARCH_STEP_MM;
        if (step > FLYBOX_TOF_SEARCH_STEP_MM) step = FLYBOX_TOF_SEARCH_STEP_MM;
        if (step < 1.0f) step = 1.0f;

        g_tof26_diag.blindSteps++;

        for (uint8_t i = 0U; i < 2U; i++)
        {
            Motor_SetSpeed(ids[i], speed_mm_s);
            Motor_SetAcceleration(ids[i], accel_mm_s2);
            osDelay(5);
        }

        float target = (direction >= 0) ? step : -step;
        for (uint8_t i = 0U; i < 2U; i++)
        {
            if (Motor_SetTarget(ids[i], target, MOTOR_TARGET_RELATIVE) != HAL_OK) return HAL_ERROR;
            osDelay(5);
        }

        if (FlyBox_WaitMultipleReached(ids, 2U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS) != HAL_OK)
        {
            g_tof26_diag.exitReason = 2U;
            return HAL_ERROR;
        }
    }

    g_tof26_diag.exitReason = 3U;
    return HAL_ERROR;
}

/* ======================== 取箱流程 ======================== */

/**
  * @brief  Step3.5: 验证箱体在传送带上 (TOF25 距离 ≤ FLYBOX_TOF25_VERIFY_MAX_MM)
  */
static HAL_StatusTypeDef FlyBox_VerifyBoxOnConveyor(void)
{
    if (TOF8_IsWarmingUp(TOF8_DEVICE_ID_0))
    {
        osDelay(FLYBOX_TOF_SETTLE_MS);
    }

    float dist = TOF8_GetDistanceMedian(TOF8_DEVICE_ID_0,
                                         FLYBOX_TOF_MEDIAN_SAMPLES,
                                         FLYBOX_TOF_MEDIAN_INTERVAL_MS);
    g_pick_diag.verifyDist = dist;

    if (dist != TOF8_DISTANCE_INVALID_MM && dist <= FLYBOX_TOF25_VERIFY_MAX_MM)
    {
        g_pick_diag.verify = 1U;
        return HAL_OK;
    }

    g_pick_diag.verify = 0U;
    return HAL_ERROR;
}

/**
  * @brief  Step5: 旋转同步 (取箱)
  *         传送带送箱 + 钩爪旋转到 90° → 确保箱体与传送带平行
  */
static HAL_StatusTypeDef FlyBox_RotateSyncPick(void)
{
    g_pick_diag.syncSteps = 0U;

    /* 传送带送箱 (配合旋转同步) */
    const uint8_t ids[] = { FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R };
    for (uint8_t i = 0U; i < 2U; i++)
    {
        Motor_SetSpeed(ids[i], FLYBOX_DEFAULT_SPEED);
        Motor_SetAcceleration(ids[i], FLYBOX_DEFAULT_ACCEL);
        osDelay(5);
    }
    for (uint8_t i = 0U; i < 2U; i++)
    {
        if (Motor_SetTarget(ids[i], FLYBOX_HOOK_FINAL_MM, MOTOR_TARGET_RELATIVE) != HAL_OK)
            return HAL_ERROR;
        osDelay(5);
    }

    /* 同步旋转 */
    if (FlyBox_GripperRotateAbs(FLYBOX_GRIPPER_ROTATE_DEG, FLYBOX_ROTATE_SPEED, FLYBOX_ROTATE_ACCEL) != HAL_OK)
        return HAL_ERROR;

    /* 等待传送带到位 */
    if (FlyBox_WaitMultipleReached(ids, 2U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS) != HAL_OK)
        return HAL_ERROR;

    g_pick_diag.syncSteps = 1U;
    osDelay(FLYBOX_ROTATE_SETTLE_MS);

    g_pick_diag.syncEndMm = 0.0f;
    return HAL_OK;
}

HAL_StatusTypeDef FlyBox_PickBox(void)
{
    if (!s_homed) return HAL_ERROR;
    s_state = FLYBOX_BUSY;

    g_pick_diag.lastStep = 0U;
    g_pick_diag.s1 = g_pick_diag.s2 = g_pick_diag.s3 = g_pick_diag.s4 = 0U;
    g_pick_diag.verify = 0U;
    g_pick_diag.verifyDist = 0.0f;
    g_pick_diag.attempts = 0U;
    g_pick_diag.s6Recoveries = 0U;
    g_pick_diag.s6Clear = 0U;
    g_pick_diag.beltLimitHit = 0U;
    g_pick_diag.syncSteps = 0U;
    g_pick_diag.syncEndMm = 0.0f;
    g_pick_diag.gripperPosAtAbort = 0;
    g_pick_diag.gripperTorqueAtAbort = 0;
    g_pick_diag.abortReleaseResult = 0U;

    s_belt_fwd_mm = 0.0f;
    s_belt_fwd_limit_hit = 0U;

    for (uint8_t attempt = 0U; attempt < FLYBOX_PICK_MAX_ATTEMPTS; attempt++)
    {
        g_pick_diag.attempts = attempt + 1U;

        /* 重试前置: 松开钩爪, 避免上次失败遗留的咬合状态 */
        if (attempt > 0U)
        {
            FlyBox_GripperRelease();
            osDelay(FLYBOX_RETRY_SETTLE_MS);
        }

        /* ---- Step1: 皮带前进直到 TOF25 检测到钩取点 ---- */
        g_pick_diag.lastStep = 1U;
        {
            HAL_StatusTypeDef st = FlyBox_GripperBeltMoveUntilTof25(
                FLYBOX_TOF25_HOOK_THRESH_MM, FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL);
            if (st != HAL_OK) { g_pick_diag.s1 = 0U; continue; }
            g_pick_diag.s1 = 1U;
        }
        osDelay(FLYBOX_STEP_SETTLE_MS);

        /* ---- Step2: 钩爪钩住 ---- */
        g_pick_diag.lastStep = 2U;
        {
            HAL_StatusTypeDef st = FlyBox_GripperHook();
            if (st != HAL_OK) { g_pick_diag.s2 = 0U; continue; }
            g_pick_diag.s2 = 1U;
        }
        osDelay(FLYBOX_STEP_SETTLE_MS);

        /* ---- Step3: 皮带后退 + 传送带前进 (同步拉箱上带) ---- */
        g_pick_diag.lastStep = 3U;
        {
            const uint8_t ids3[] = { FLYBOX_ID_GRIPPER_BELT, FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R };
            for (uint8_t i = 0U; i < 3U; i++)
            {
                Motor_SetSpeed(ids3[i], FLYBOX_DEFAULT_SPEED);
                Motor_SetAcceleration(ids3[i], FLYBOX_DEFAULT_ACCEL);
                osDelay(5);
            }
            /* 皮带后退 */
            if (Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, -FLYBOX_BELT_MAX_FWD_MM, MOTOR_TARGET_RELATIVE) != HAL_OK)
            { g_pick_diag.s3 = 0U; continue; }
            /* 传送带前进 (拉箱) */
            if (Motor_SetTarget(FLYBOX_ID_CONVEYOR_L, FLYBOX_BELT_MAX_FWD_MM, MOTOR_TARGET_RELATIVE) != HAL_OK)
            { g_pick_diag.s3 = 0U; continue; }
            if (Motor_SetTarget(FLYBOX_ID_CONVEYOR_R, FLYBOX_BELT_MAX_FWD_MM, MOTOR_TARGET_RELATIVE) != HAL_OK)
            { g_pick_diag.s3 = 0U; continue; }

            HAL_StatusTypeDef st = FlyBox_WaitMultipleReached(ids3, 3U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
            if (st != HAL_OK) { g_pick_diag.s3 = 0U; continue; }
            g_pick_diag.s3 = 1U;
        }
        osDelay(FLYBOX_STEP_SETTLE_MS);

        /* ---- Step3.5: 验证箱体在传送带上 ---- */
        g_pick_diag.lastStep = 4U;
        {
            HAL_StatusTypeDef st = FlyBox_VerifyBoxOnConveyor();
            if (st != HAL_OK) { g_pick_diag.s4 = 0U; continue; }
            g_pick_diag.s4 = 1U;
        }

        /* ---- Step4: 松开钩爪 (旋转前必须松) ---- */
        g_pick_diag.lastStep = 5U;
        {
            HAL_StatusTypeDef st = FlyBox_GripperRelease();
            if (st != HAL_OK) continue;
        }
        osDelay(FLYBOX_STEP_SETTLE_MS);

        /* ---- Step5: 旋转同步 ---- */
        g_pick_diag.lastStep = 6U;
        {
            HAL_StatusTypeDef st = FlyBox_RotateSyncPick();
            if (st != HAL_OK) continue;
        }

        s_state = FLYBOX_COMPLETE;
        return HAL_OK;
    }

    /* 所有尝试失败, 安全中止 */
    FlyBox_AbortAndSafe();
    s_state = FLYBOX_ERROR;
    return HAL_ERROR;
}

/* ======================== 放箱流程 ======================== */

/**
  * @brief  清障并确保正面朝向 (TOF26 清障 + 旋转到 0°)
  */
static HAL_StatusTypeDef FlyBox_CheckClearanceAndFaceFront(void)
{
    /* TOF26 清障: 传送带后退直到 TOF26 距离 ≥ FLYBOX_PLACE_TOF26_TARGET_MM */
    if (FlyBox_ConveyorRunUntilTof26(FLYBOX_PLACE_TOF26_TARGET_MM,
                                      FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL, -1) != HAL_OK)
    {
        /* 非致命, 继续 */
    }

    /* 旋转到正面 */
    if (FlyBox_GripperRotateAbs(0.0f, FLYBOX_ROTATE_SPEED, FLYBOX_ROTATE_ACCEL) != HAL_OK)
        return HAL_ERROR;

    osDelay(FLYBOX_ROTATE_SETTLE_MS);
    return HAL_OK;
}

/**
  * @brief  旋转同步 (放箱)
  *         传送带后退 + 钩爪旋转到 90° → 同步完成
  */
static HAL_StatusTypeDef FlyBox_RotateSyncPlace(void)
{
    /* 传送带后退同步量 */
    const uint8_t ids[] = { FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R };
    for (uint8_t i = 0U; i < 2U; i++)
    {
        Motor_SetSpeed(ids[i], FLYBOX_DEFAULT_SPEED);
        Motor_SetAcceleration(ids[i], FLYBOX_DEFAULT_ACCEL);
        osDelay(5);
    }
    for (uint8_t i = 0U; i < 2U; i++)
    {
        if (Motor_SetTarget(ids[i], -FLYBOX_PLACE_SYNC_RETREAT_MM,
                            MOTOR_TARGET_RELATIVE) != HAL_OK) return HAL_ERROR;
        osDelay(5);
    }
    if (FlyBox_WaitMultipleReached(ids, 2U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS) != HAL_OK)
        return HAL_ERROR;

    /* 钩爪旋转到 90° */
    if (FlyBox_GripperRotateAbs(FLYBOX_GRIPPER_ROTATE_DEG, FLYBOX_ROTATE_SPEED, FLYBOX_ROTATE_ACCEL) != HAL_OK)
        return HAL_ERROR;

    osDelay(FLYBOX_ROTATE_SETTLE_MS);
    return HAL_OK;
}

HAL_StatusTypeDef FlyBox_PlaceBox(void)
{
    if (!s_homed) return HAL_ERROR;
    s_state = FLYBOX_BUSY;

    /* ---- Step1: 清障 + 正面朝向 ---- */
    if (FlyBox_CheckClearanceAndFaceFront() != HAL_OK) { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    osDelay(FLYBOX_STEP_SETTLE_MS);

    /* ---- Step2: 松开钩爪 (推箱前必须先松) ---- */
    if (FlyBox_GripperRelease() != HAL_OK)
    { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    osDelay(FLYBOX_STEP_SETTLE_MS);

    /* ---- Step3: 皮带前进 + 传送带后退 (三电机同步推出) ---- */
    {
        const uint8_t ids3[] = { FLYBOX_ID_GRIPPER_BELT, FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R };
        for (uint8_t i = 0U; i < 3U; i++)
        {
            Motor_SetSpeed(ids3[i], FLYBOX_DEFAULT_SPEED);
            Motor_SetAcceleration(ids3[i], FLYBOX_DEFAULT_ACCEL);
            osDelay(5);
        }
        /* 皮带前进 */
        if (Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, FLYBOX_PLACE_BELT_FWD_MM, MOTOR_TARGET_RELATIVE) != HAL_OK)
        { s_state = FLYBOX_ERROR; return HAL_ERROR; }
        /* 传送带后退 (配合推出) */
        if (Motor_SetTarget(FLYBOX_ID_CONVEYOR_L, -FLYBOX_PLACE_CONVEYOR_BACK_MM, MOTOR_TARGET_RELATIVE) != HAL_OK)
        { s_state = FLYBOX_ERROR; return HAL_ERROR; }
        if (Motor_SetTarget(FLYBOX_ID_CONVEYOR_R, -FLYBOX_PLACE_CONVEYOR_BACK_MM, MOTOR_TARGET_RELATIVE) != HAL_OK)
        { s_state = FLYBOX_ERROR; return HAL_ERROR; }
        if (FlyBox_WaitMultipleReached(ids3, 3U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS) != HAL_OK)
        { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    }
    osDelay(FLYBOX_STEP_SETTLE_MS);

    /* ---- Step3.5: TOF25 补偿 (距离 > 期望 → 额外前进) ---- */
    {
        float dist = TOF8_GetDistanceMedian(TOF8_DEVICE_ID_0,
                                             FLYBOX_TOF_MEDIAN_SAMPLES,
                                             FLYBOX_TOF_MEDIAN_INTERVAL_MS);
        if (dist != TOF8_DISTANCE_INVALID_MM && dist > FLYBOX_TOF25_PLACE_EXPECT_MM)
        {
            float comp = dist - FLYBOX_TOF25_PLACE_EXPECT_MM;
            if (comp > FLYBOX_PLACE_COMP_MAX_MM) comp = FLYBOX_PLACE_COMP_MAX_MM;
            if (FlyBox_GripperBeltMove(comp, FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL) != HAL_OK)
            { s_state = FLYBOX_ERROR; return HAL_ERROR; }
        }
    }
    osDelay(FLYBOX_STEP_SETTLE_MS);

    /* ---- Step4: 皮带后退 ---- */
    if (FlyBox_GripperBeltMove(-FLYBOX_PLACE_BELT_FWD_MM, FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL) != HAL_OK)
    { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    osDelay(FLYBOX_STEP_SETTLE_MS);

    /* ---- Step5: 旋转同步 (放箱) ---- */
    if (FlyBox_RotateSyncPlace() != HAL_OK)
    { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_COMPLETE;
    return HAL_OK;
}

/* ======================== 安全中止 ======================== */

void FlyBox_AbortAndSafe(void)
{
    /* 急停所有电机 */
    for (uint8_t i = 1U; i <= FLYBOX_MOTOR_COUNT; i++)
    {
        Motor_EmergencyStop(i);
    }

    osDelay(200);

    /* 松开钩爪 */
    {
        volatile Motor_Status_t *p = Motor_GetStatus(FLYBOX_ID_GRIPPER);
        if (p != NULL)
        {
            g_pick_diag.gripperPosAtAbort   = p->mainPosition;
            g_pick_diag.gripperTorqueAtAbort = p->torque;
        }

        HAL_StatusTypeDef st = FlyBox_GripperRelease();
        g_pick_diag.abortReleaseResult = (st == HAL_OK) ? 1U : 0U;
    }

    s_state = FLYBOX_IDLE;
}