/**
  ******************************************************************************
  * @file    motion.c
  * @brief   差速小车运动控制层实现
  *
  *          电机仅支持位置模式:
  *            直行 = 双电机同向等距目标位置
  *            转向 = 双电机反向弧长目标位置 + 陀螺仪校正
  *
  *          依赖: motor.c, imu_core.c (extern g_imu_angle_deg), FreeRTOS
  ******************************************************************************
  */

#include "motion.h"
#include "motor.h"
#include "imu_core.h"
#include "pid.h"
#include <cmsis_os.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ======================== IMU 共享变量 (imu.c 定义) ======================== */
extern volatile float g_imu_angle_deg;
extern volatile float g_imu_rate_dps;

/* ======================== 内部状态 ======================== */
static Motion_Config_t  s_config;
static Motion_State_t   s_state     = MOTION_IDLE;
static uint8_t          s_lastError = 0;
static uint8_t          s_initialized = 0;

/* 轮距标定结果 (全局, 方便调试器查看) */
volatile float g_calibWheelBase_mm = 0.0f;

/* 电机到位标志 (由 Task_CanRx → Motor_ProcessRxFrame 设置 positionReached) */
static SemaphoreHandle_t s_reachSem = NULL;

/* ======================== 第 1 层: 纯读写 ======================== */

Motion_State_t Motion_GetState(void)
{
    return s_state;
}

uint8_t Motion_GetLastError(void)
{
    return s_lastError;
}

SemaphoreHandle_t Motion_GetReachSem(void)
{
    return s_reachSem;
}

/* ======================== 第 2 层: 简单动作 ======================== */

void Motion_NotifyMotorReached(uint8_t deviceId)
{
    (void)deviceId;
    if (s_reachSem != NULL)
    {
        xSemaphoreGive(s_reachSem);
    }
}

HAL_StatusTypeDef Motion_Stop(void)
{
    if (!s_initialized) return HAL_ERROR;

    Motor_EmergencyStop(s_config.leftMotorId);
    Motor_EmergencyStop(s_config.rightMotorId);
    s_state = MOTION_IDLE;
    return HAL_OK;
}

HAL_StatusTypeDef Motion_Unlock(void)
{
    if (!s_initialized) return HAL_ERROR;

    Motor_Unlock(s_config.leftMotorId);
    Motor_Unlock(s_config.rightMotorId);
    s_state = MOTION_IDLE;
    return HAL_OK;
}

/* ======================== 第 3 层: 内部辅助 ======================== */

/**
  * @brief  等待双电机到位 (轮询 positionReached, 10ms 间隔)
  */
static HAL_StatusTypeDef Motion_WaitMotorsReached(uint32_t timeout_ms)
{
    volatile Motor_Status_t *pLeft  = Motor_GetStatus(s_config.leftMotorId);
    volatile Motor_Status_t *pRight = Motor_GetStatus(s_config.rightMotorId);

    if (pLeft == NULL || pRight == NULL) return HAL_ERROR;

    /* 等待电机启动 (避免初始反馈误判) */
    osDelay(300);

    /* 清除到位标志和残留错误码 */
    pLeft->positionReached  = 0;
    pRight->positionReached = 0;
    pLeft->lastErrorCode    = 0;
    pRight->lastErrorCode   = 0;

    uint32_t elapsed = 0;
    const uint32_t poll_ms = 10;

    while (elapsed < timeout_ms)
    {
        /* 只检查致命错误 (堵转/超温/欠压/断线), 忽略 0x0102(未到位) */
        uint16_t errL = pLeft->lastErrorCode;
        uint16_t errR = pRight->lastErrorCode;
        if ((errL == MOTOR_ERR_STALL || errL == MOTOR_ERR_OVERTEMP ||
             errL == MOTOR_ERR_UNDERVOLTAGE || errL == MOTOR_ERR_LINE_BREAK) ||
            (errR == MOTOR_ERR_STALL || errR == MOTOR_ERR_OVERTEMP ||
             errR == MOTOR_ERR_UNDERVOLTAGE || errR == MOTOR_ERR_LINE_BREAK))
        {
            s_lastError = MOTION_ERR_STALL;
            s_state = MOTION_ERROR;
            return HAL_ERROR;
        }

        if (pLeft->positionReached && pRight->positionReached)
        {
            return HAL_OK;
        }

        osDelay(poll_ms);
        elapsed += poll_ms;
    }

    s_lastError = MOTION_ERR_TIMEOUT;
    s_state = MOTION_ERROR;
    return HAL_TIMEOUT;
}

/**
  * @brief  设置双电机速度和加速度
  */
static void Motion_SetBothSpeedAccel(float speed_mm_s, float accel_mm_s2)
{
    Motor_SetSpeed(s_config.leftMotorId, speed_mm_s);
    Motor_SetSpeed(s_config.rightMotorId, speed_mm_s);
    osDelay(5);
    Motor_SetAcceleration(s_config.leftMotorId, accel_mm_s2);
    Motor_SetAcceleration(s_config.rightMotorId, accel_mm_s2);
    osDelay(5);
}

/**
  * @brief  发送双电机相对目标位置
  */
static void Motion_SendBothTarget(float left_mm, float right_mm)
{
    Motor_SetTarget(s_config.leftMotorId, left_mm, MOTOR_TARGET_RELATIVE);
    Motor_SetTarget(s_config.rightMotorId, right_mm, MOTOR_TARGET_RELATIVE);
}

/**
  * @brief  角度 → 轮子弧长 (mm)
  *         arc = angle_rad × (wheelBase / 2)
  */
static float Motion_AngleToWheelDist(float angle_deg)
{
    float angle_rad = angle_deg * (float)M_PI / 180.0f;
    return angle_rad * (s_config.wheelBase_mm / 2.0f);
}

/**
  * @brief  角速度 → 轮子线速度 (mm/s)
  */
static float Motion_AngularSpeedToLinear(float angular_speed_dps)
{
    float angle_rad_s = angular_speed_dps * (float)M_PI / 180.0f;
    return angle_rad_s * (s_config.wheelBase_mm / 2.0f);
}

/**
  * @brief  角加速度 → 轮子线加速度 (mm/s²)
  */
static float Motion_AngularAccelToLinear(float angular_accel_dps2)
{
    float angle_rad_s2 = angular_accel_dps2 * (float)M_PI / 180.0f;
    return angle_rad_s2 * (s_config.wheelBase_mm / 2.0f);
}

/* ======================== 第 4 层: 初始化 ======================== */

/**
  * @brief  比较两个 SN 是否相同
  */
static uint8_t Motion_SNMatch(const uint8_t *a, const uint8_t *b)
{
    for (uint8_t i = 0; i < MOTOR_SN_LEN; i++)
    {
        if (a[i] != b[i]) return 0;
    }
    return 1;
}

HAL_StatusTypeDef Motion_Init(const Motion_Config_t *pConfig)
{
    HAL_StatusTypeDef status;

    /* 填充配置 */
    if (pConfig != NULL)
    {
        s_config = *pConfig;
    }
    else
    {
        s_config.wheelBase_mm         = MOTION_DEFAULT_WHEEL_BASE_MM;
        s_config.wheelDiameter_mm     = MOTION_DEFAULT_WHEEL_DIAMETER_MM;
        s_config.leftMotorId          = MOTION_DEFAULT_LEFT_ID;
        s_config.rightMotorId         = MOTION_DEFAULT_RIGHT_ID;
        s_config.turnCorrectionThresh = MOTION_TURN_CORRECTION_THRESH_DEG;
        s_config.maxCorrections       = MOTION_TURN_MAX_CORRECTIONS;
        s_config.overshootThresh      = MOTION_TURN_OVERSHOOT_THRESH_DEG;
        s_config.invertRightMotor     = 1;

        /* 直行纠偏默认参数 (速度修正方式) */
        s_config.straightKp            = MOTION_STRAIGHT_KP;
        s_config.straightKi            = MOTION_STRAIGHT_KI;
        s_config.straightKd            = MOTION_STRAIGHT_KD;
        s_config.straightCorrInterval_ms = MOTION_STRAIGHT_CORR_INTERVAL_MS;
        s_config.straightMaxSpeedCorr  = MOTION_STRAIGHT_MAX_SPEED_CORR;
        s_config.straightHeadingThresh = MOTION_STRAIGHT_HEADING_THRESH_DEG;
        s_config.straightEndThresh     = MOTION_STRAIGHT_END_THRESH_DEG;

        /* 标定数据默认: 0 = 未标定, 纯 PID */
        s_config.calibBias_deg_per_m   = 0.0f;
    }

    /* 创建信号量 */
    if (s_reachSem == NULL)
    {
        s_reachSem = xSemaphoreCreateBinary();
        if (s_reachSem == NULL) return HAL_ERROR;
    }

    /* ===== SN 绑定流程: 按 SN 内容区分左右轮 ===== */
    static const uint8_t leftSN[MOTOR_SN_LEN]  = MOTION_LEFT_MOTOR_SN;
    static const uint8_t rightSN[MOTOR_SN_LEN] = MOTION_RIGHT_MOTOR_SN;

    /* 清空队列, 请求所有未绑定电机上报 SN */
    Motor_ClearSNQueue();
    Motor_RequestSN(0x07U);

    /* 等待至少 2 个 SN 入队 */
    status = Motor_WaitSNQueue(2U, MOTION_SN_TIMEOUT_MS);
    if (status != HAL_OK) return status;

    /* 遍历队列, 按 SN 匹配分配设备号 */
    uint8_t leftBound = 0, rightBound = 0;

    for (uint8_t i = 0; i < g_motor_sn_queue.count; i++)
    {
        uint8_t *sn = (uint8_t *)g_motor_sn_queue.entries[i].sn;

        if (Motion_SNMatch(sn, leftSN))
        {
            Motor_SetDeviceId(sn, s_config.leftMotorId);
            leftBound = 1;
            osDelay(100);
        }
        else if (Motion_SNMatch(sn, rightSN))
        {
            Motor_SetDeviceId(sn, s_config.rightMotorId);
            rightBound = 1;
            osDelay(100);
        }
    }

    if (!leftBound || !rightBound) return HAL_ERROR;

    /* ===== 配置电机 ===== */
    status = Motor_Init(s_config.leftMotorId, s_config.wheelDiameter_mm*1.005, 500.0f, 1000.0f);
    if (status != HAL_OK) return status;
    osDelay(50);

    status = Motor_Init(s_config.rightMotorId, s_config.wheelDiameter_mm*1.004, 500.0f, 1000.0f);
    if (status != HAL_OK) return status;
    osDelay(50);

    /* 右轮方向取反 (差速车镜像安装) */
    if (s_config.invertRightMotor)
    {
        Motor_ConfigParam(s_config.rightMotorId, s_config.rightMotorId, MOTOR_DIR_CCW);
        osDelay(50);
    }

    s_initialized = 1;
    s_state = MOTION_IDLE;
    s_lastError = 0;
    return HAL_OK;
}

/* ======================== 第 5 层: 核心运动 ======================== */

HAL_StatusTypeDef Motion_GoStraight(float distance_mm, float speed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) { s_lastError = MOTION_ERR_NOT_INIT; return HAL_ERROR; }
    if (speed_mm_s <= 0 || accel_mm_s2 <= 0) { s_lastError = MOTION_ERR_INVALID; return HAL_ERROR; }

    s_state = MOTION_BUSY;

    /* 设置速度和加速度 */
    Motion_SetBothSpeedAccel(speed_mm_s, accel_mm_s2);

    /* 双电机同向等距 */
    Motion_SendBothTarget(distance_mm, distance_mm);

    /* 等待到位 */
    HAL_StatusTypeDef result = Motion_WaitMotorsReached(MOTION_MOTOR_WAIT_TIMEOUT_MS);

    if (result == HAL_OK)
    {
        s_state = MOTION_COMPLETE;
    }
    return result;
}

/**
  * @brief  陀螺仪角度校正 
  *
  *         读取 g_imu_angle_deg, 与目标角度比较, 若误差超限则发修正目标。
  *         精度策略: 只允许过冲(多转), 不允许不足(没转够)。
  *
  * @param  targetAngle_deg  目标角度 (°, 左转正/右转负, 电机约定)
  * @param  corrSpeed_mm_s   修正时的电机线速度 (mm/s)
  * @param  corrAccel_mm_s2  修正时的电机线加速度 (mm/s²)
  * @param  leftOnly         0=双轮反向原地转(默认), 1=仅左轮修正(绕右轮转)
  * @retval HAL_OK 校正完成或精度满足, HAL_ERROR/HAL_TIMEOUT 电机异常
  */
HAL_StatusTypeDef Motion_GyroCorrect(float targetAngle_deg,
                                             float corrSpeed_mm_s,
                                             float corrAccel_mm_s2,
                                             uint8_t leftOnly)
{
    for (uint8_t i = 0; i < s_config.maxCorrections; i++)
    {
        osDelay(50);  /* 等待稳定 */

        /* g_imu_angle_deg 已在源头统一为电机约定 (左转正, 右转负) */
        float currentAngle = g_imu_angle_deg;
        float error = targetAngle_deg - currentAngle;

        /* 归一化误差到 [-180, 180), 避免 ±180° 边界跳变 */
        while (error > 180.0f)  error -= 360.0f;
        while (error < -180.0f) error += 360.0f;

        /* 精度判断:
         * target=0 (末端精调): 对称判断, |error| <= thresh 即可
         * 左转(+): 只允许过冲, error <= 0 且 >= -thresh → 可接受
         * 右转(-): 只允许不足, error <= 0 且 >= -thresh → 可接受 */
        if (targetAngle_deg == 0.0f)
        {
            if (fabsf(error) <= s_config.turnCorrectionThresh) break;
        }
        else if (targetAngle_deg > 0)
        {
            if (error <= 0 && error >= -s_config.turnCorrectionThresh) break;
        }
        else
        {
            if (error <= 0 && error >= -s_config.turnCorrectionThresh) break;
        }

        /* 发送修正目标 */
        Motion_SetBothSpeedAccel(corrSpeed_mm_s, corrAccel_mm_s2);

        if (leftOnly)
        {
            /* 模式 1: 仅左轮修正 (绕右轮转), 弧长 = angle_rad × wheelBase
             * 取反: error<0(需右转) → 左轮前进 → 绕右轮右转 */
            float angle_rad = error * (float)M_PI / 180.0f;
            float leftArc = -angle_rad * s_config.wheelBase_mm;
            Motor_SetTarget(s_config.leftMotorId, leftArc, MOTOR_TARGET_RELATIVE);
            Motor_SetTarget(s_config.rightMotorId, 0.0f, MOTOR_TARGET_RELATIVE);
        }
        else
        {
            /* 模式 0: 双轮反向原地转, 弧长 = angle_rad × (wheelBase/2) */
            float corrArc = Motion_AngleToWheelDist(error);
            Motion_SendBothTarget(-corrArc, corrArc);
        }

        HAL_StatusTypeDef result = Motion_WaitMotorsReached(MOTION_MOTOR_WAIT_TIMEOUT_MS);
        if (result != HAL_OK) return result;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Motion_Turn(float angle_deg, float angular_speed_dps, float angular_accel)
{
    if (!s_initialized) { s_lastError = MOTION_ERR_NOT_INIT; return HAL_ERROR; }
    if (angular_speed_dps <= 0 || angular_accel <= 0) { s_lastError = MOTION_ERR_INVALID; return HAL_ERROR; }
    if (angle_deg == 0.0f) { return HAL_OK; }

    s_state = MOTION_BUSY;

    /* 角度归零: 清零 IMU 引擎内部角度 */
    extern IMU_Engine_t g_engine;
    IMU_Engine_ZeroAngle(&g_engine);
    osDelay(20);  /* 等待 Task_Main 更新一次 g_imu_angle_deg */

    /* 计算轮子线速度/加速度 */
    float linearSpeed = Motion_AngularSpeedToLinear(angular_speed_dps);
    float linearAccel = Motion_AngularAccelToLinear(angular_accel);

    /* 计算弧长 */
    float arc = Motion_AngleToWheelDist(angle_deg);  /* 带符号 */

    /* 设置速度/加速度 */
    Motion_SetBothSpeedAccel(linearSpeed, linearAccel);

    /* 发送目标: 左转(+) → 左轮后退(-), 右轮前进(+) */
    Motion_SendBothTarget(-arc, arc);

    /* 等待电机到位 */
    HAL_StatusTypeDef result = Motion_WaitMotorsReached(MOTION_MOTOR_WAIT_TIMEOUT_MS);
    if (result != HAL_OK) return result;

    /* 陀螺仪校正 (降速精调) */
    result = Motion_GyroCorrect(angle_deg, linearSpeed * 0.3f, linearAccel, 0);
    if (result != HAL_OK) return result;

    s_state = MOTION_COMPLETE;
    return HAL_OK;
}

/* ======================== 第 6 层: 直行 PID 纠偏 ======================== */

/**
  * @brief  直行 + 陀螺仪 PID 速度纠偏 (连续, 不停顿)
  *
  *         原理: 发送完整位置目标后, 运动中每 corrInterval_ms 读一次 IMU 角度,
  *         PID 输出速度修正量, 通过 Motor_SetSpeed 分别设置左右轮速度。
  *         速度命令 (0x0415) 不覆盖位置目标 (0x0416), 电机继续朝原目标走。
  *
  *         角度偏正(左转了) → 左轮减速 / 右轮加速 → 产生右转力矩 → 拉回航向
  */
HAL_StatusTypeDef Motion_GoStraightCorrected(float distance_mm, float speed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) { s_lastError = MOTION_ERR_NOT_INIT; return HAL_ERROR; }
    if (speed_mm_s <= 0 || accel_mm_s2 <= 0) { s_lastError = MOTION_ERR_INVALID; return HAL_ERROR; }

    s_state = MOTION_BUSY;

    /* 清零 IMU 角度 */
    extern IMU_Engine_t g_engine;
    IMU_Engine_ZeroAngle(&g_engine);
    osDelay(20);

    /* 初始化 PID 控制器 (输出 = 速度修正量 mm/s) */
    PID_Controller_t pid;
    PID_Init(&pid,
             s_config.straightKp, s_config.straightKi, s_config.straightKd,
             -s_config.straightMaxSpeedCorr, s_config.straightMaxSpeedCorr,  /* 输出限幅 */
             s_config.straightMaxSpeedCorr * 5.0f,                           /* 积分限幅 */
             s_config.straightHeadingThresh);                                /* 死区 */

    /* 设置初始速度 (双轮等速) 和加速度 */
    Motion_SetBothSpeedAccel(speed_mm_s, accel_mm_s2);

    /* 发送完整位置目标 (一次性, 不再覆盖) */
    Motion_SendBothTarget(distance_mm, distance_mm);

    /* 获取电机状态指针 */
    volatile Motor_Status_t *pLeft  = Motor_GetStatus(s_config.leftMotorId);
    volatile Motor_Status_t *pRight = Motor_GetStatus(s_config.rightMotorId);
    if (pLeft == NULL || pRight == NULL) { s_lastError = MOTION_ERR_MOTOR_OFFLINE; s_state = MOTION_ERROR; return HAL_ERROR; }

    /* 等待电机启动, 清除残留到位标志 */
    osDelay(300);
    pLeft->positionReached  = 0;
    pRight->positionReached = 0;
    pLeft->lastErrorCode    = 0;
    pRight->lastErrorCode   = 0;

    float dt_s = (float)s_config.straightCorrInterval_ms / 1000.0f;
    uint32_t elapsed_ms = 300;
    uint32_t timeout_ms = MOTION_MOTOR_WAIT_TIMEOUT_MS;

    /* 连续速度修正循环 */
    while (elapsed_ms < timeout_ms)
    {
        /* 检查到位 */
        if (pLeft->positionReached && pRight->positionReached)
        {
            break;
        }

        /* 检查致命错误 */
        uint16_t errL = pLeft->lastErrorCode;
        uint16_t errR = pRight->lastErrorCode;
        if ((errL == MOTOR_ERR_STALL || errL == MOTOR_ERR_OVERTEMP ||
             errL == MOTOR_ERR_UNDERVOLTAGE || errL == MOTOR_ERR_LINE_BREAK) ||
            (errR == MOTOR_ERR_STALL || errR == MOTOR_ERR_OVERTEMP ||
             errR == MOTOR_ERR_UNDERVOLTAGE || errR == MOTOR_ERR_LINE_BREAK))
        {
            s_lastError = MOTION_ERR_STALL;
            s_state = MOTION_ERROR;
            return HAL_ERROR;
        }

        /* 读取当前航向角 */
        float heading = g_imu_angle_deg;

        /* 前馈补偿: 减去标定偏差 (calibBias=0 时无效果) */
        float estimatedDist = speed_mm_s * ((float)elapsed_ms / 1000.0f);
        float feedforward = s_config.calibBias_deg_per_m * (estimatedDist / 1000.0f);

        /* PID 误差 = 当前航向 - 前馈补偿 (目标航向 = 0) */
        float error = heading - feedforward;

        /* PID 更新 → 速度修正量 (mm/s) */
        float speedCorr = PID_Update(&pid, error, dt_s);

        /* 调整左右轮速度: * 角度偏正(左转了) → speedCorr > 0 → 左轮加速, 右轮减速 → 右转拉回 */
        float leftSpeed  = speed_mm_s + speedCorr;
        float rightSpeed = speed_mm_s - speedCorr;

        /* 速度下限保护 (不能低于基准的 30%) */
        float minSpeed = speed_mm_s * 0.3f;
        if (leftSpeed < minSpeed)  leftSpeed = minSpeed;
        if (rightSpeed < minSpeed) rightSpeed = minSpeed;

        /* 发送速度修正 (不覆盖位置目标) */
        Motor_SetSpeed(s_config.leftMotorId, leftSpeed);
        Motor_SetSpeed(s_config.rightMotorId, rightSpeed);

        osDelay(s_config.straightCorrInterval_ms);
        elapsed_ms += s_config.straightCorrInterval_ms;
    }

    /* 判断是否超时 */
    if (!(pLeft->positionReached && pRight->positionReached))
    {
        s_lastError = MOTION_ERR_TIMEOUT;
        s_state = MOTION_ERROR;
        return HAL_TIMEOUT;
    }

    /* 恢复双轮等速 (为后续动作准备) */
    Motion_SetBothSpeedAccel(speed_mm_s, accel_mm_s2);

    /* --- 末端精调: 最终角度超阈值则调用 GyroCorrect 修正回 0° --- */
    osDelay(50);
    float finalHeading = g_imu_angle_deg;
    if (fabsf(finalHeading) > s_config.straightEndThresh)
    {
        Motion_GyroCorrect(0.0f, speed_mm_s * 0.2f, accel_mm_s2, 1);
    }

    s_state = MOTION_COMPLETE;
    return HAL_OK;
}

/* ======================== 第 7 层: 标定 + Flash 存储 ======================== */

/**
  * @brief  标定数据结构 (存储在 Flash)
  */
typedef struct
{
    uint32_t magic;               /* 魔数: MOTION_CALIB_MAGIC */
    float    calibBias_deg_per_m; /* 系统偏差 (°/m) */
    float    calibDistance_mm;    /* 标定距离 (mm) */
    uint8_t  calibRuns;           /* 标定次数 */
    uint8_t  reserved[3];         /* 对齐 */
    uint32_t checksum;            /* 校验: 所有前面字段的 XOR */
} Motion_CalibFlash_t;

/**
  * @brief  计算校验和
  */
static uint32_t Motion_CalibChecksum(const Motion_CalibFlash_t *pData)
{
    const uint32_t *p = (const uint32_t *)pData;
    uint32_t sum = 0;
    /* 对 magic + bias + distance + runs 做 XOR (不含 checksum 本身) */
    uint32_t words = (sizeof(Motion_CalibFlash_t) - sizeof(uint32_t)) / sizeof(uint32_t);
    for (uint32_t i = 0; i < words; i++)
    {
        sum ^= p[i];
    }
    return sum;
}

/**
  * @brief  从 Flash 读取标定数据
  * @retval HAL_OK 有效数据已加载, HAL_ERROR 无有效数据
  */
static HAL_StatusTypeDef Motion_LoadCalibFromFlash(void)
{
    const Motion_CalibFlash_t *pFlash = (const Motion_CalibFlash_t *)MOTION_CALIB_FLASH_ADDR;

    /* 检查魔数 */
    if (pFlash->magic != MOTION_CALIB_MAGIC)
    {
        return HAL_ERROR;
    }

    /* 校验和 */
    if (Motion_CalibChecksum(pFlash) != pFlash->checksum)
    {
        return HAL_ERROR;
    }

    /* 加载到配置 */
    s_config.calibBias_deg_per_m = pFlash->calibBias_deg_per_m;
    return HAL_OK;
}

/**
  * @brief  将标定数据写入 Flash
  * @retval HAL_OK 写入成功
  */
static HAL_StatusTypeDef Motion_SaveCalibToFlash(float bias, float dist_mm, uint8_t runs)
{
    Motion_CalibFlash_t data;
    data.magic             = MOTION_CALIB_MAGIC;
    data.calibBias_deg_per_m = bias;
    data.calibDistance_mm  = dist_mm;
    data.calibRuns         = runs;
    data.reserved[0]       = 0;
    data.reserved[1]       = 0;
    data.reserved[2]       = 0;
    data.checksum          = Motion_CalibChecksum(&data);

    /* 解锁 Flash */
    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 擦除页 */
    FLASH_EraseInitTypeDef eraseInit;
    uint32_t pageError = 0;
    eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = MOTION_CALIB_FLASH_ADDR;
    eraseInit.NbPages     = 1;

    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    /* 逐字写入 */
    const uint32_t *pSrc = (const uint32_t *)&data;
    uint32_t addr = MOTION_CALIB_FLASH_ADDR;
    uint32_t words = (sizeof(Motion_CalibFlash_t) + 3) / 4;

    for (uint32_t i = 0; i < words; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, pSrc[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return HAL_OK;
}

/**
  * @brief  标定接口: 加载 Flash 或执行新标定
  */
HAL_StatusTypeDef Motion_Calibrate(Motion_CalibMode_t mode,
                                    float dist_mm, float speed_mm_s,
                                    float accel_mm_s2, uint8_t runs)
{
    if (!s_initialized) { s_lastError = MOTION_ERR_NOT_INIT; return HAL_ERROR; }

    if (mode == MOTION_CALIB_LOAD_FLASH)
    {
        /* 模式 1: 从 Flash 加载, 失败则自动执行标定 */
        if (Motion_LoadCalibFromFlash() == HAL_OK)
        {
            return HAL_OK;  /* Flash 有有效数据, 直接使用 */
        }
        /* Flash 无数据, 继续往下执行标定流程 */
    }

    /* 模式 2: 执行标定 */
    if (runs == 0 || dist_mm <= 0 || speed_mm_s <= 0 || accel_mm_s2 <= 0)
    {
        s_lastError = MOTION_ERR_INVALID;
        return HAL_ERROR;
    }

    float angleSum = 0.0f;

    for (uint8_t i = 0; i < runs; i++)
    {
        /* 清零 IMU */
        extern IMU_Engine_t g_engine;
        IMU_Engine_ZeroAngle(&g_engine);
        osDelay(20);

        /* 用原版 GoStraight (无纠偏, 测量真实偏差) */
        HAL_StatusTypeDef result = Motion_GoStraight(dist_mm, speed_mm_s, accel_mm_s2);
        if (result != HAL_OK)
        {
            return result;
        }

        osDelay(100);  /* 等待稳定 */

        /* 记录终点角度 */
        float endAngle = g_imu_angle_deg;
        angleSum += endAngle;

        /* 掉头 (除了最后一次) */
        if (i < runs - 1)
        {
            result = Motion_Turn(180.0f, 30.0f, 50.0f);
            if (result != HAL_OK) return result;
            osDelay(300);
        }
    }

    /* 计算平均偏差 (°/m) */
    float avgAngle = angleSum / (float)runs;
    float bias = avgAngle / (dist_mm / 1000.0f);  /* °/m */

    /* 存入配置 */
    s_config.calibBias_deg_per_m = bias;

    /* 写入 Flash */
    HAL_StatusTypeDef flashResult = Motion_SaveCalibToFlash(bias, dist_mm, runs);

    /* 恢复原始朝向: 标定结束后小车朝向反了 180°, 转回来 */
    HAL_StatusTypeDef turnResult = Motion_Turn(180.0f, 30.0f, 50.0f);
    if (turnResult != HAL_OK) return turnResult;

    return flashResult;
}

/**
  * @brief  轮距标定: 开环转固定角度, 用陀螺仪测量实际角度, 反推真实轮距
  *
  *         原理: 命令转 angle_deg, 电机按当前 wheelBase 计算弧长执行。
  *         陀螺仪测出实际转角 actual_deg。
  *         实际轮距 = 配置轮距 × angle_deg / actual_deg
  *
  *         例: 配置 384.5mm, 命令 90°, 实际转了 92°
  *             → 真实轮距 = 384.5 × 90/92 = 376.1mm (配置偏大, 导致过冲)
  */
HAL_StatusTypeDef Motion_CalibrateWheelBase(float angle_deg, float speed_mm_s,
                                             float accel_mm_s2, uint8_t runs,
                                             float *pResult)
{
    if (!s_initialized) { s_lastError = MOTION_ERR_NOT_INIT; return HAL_ERROR; }
    if (runs == 0 || angle_deg == 0.0f || speed_mm_s <= 0 || accel_mm_s2 <= 0)
    {
        s_lastError = MOTION_ERR_INVALID;
        return HAL_ERROR;
    }

    extern IMU_Engine_t g_engine;
    float actualAngleSum = 0.0f;

    for (uint8_t i = 0; i < runs; i++)
    {
        /* 清零 IMU */
        IMU_Engine_ZeroAngle(&g_engine);
        osDelay(20);

        /* 开环转弯: 用当前 wheelBase 计算弧长, 不做陀螺仪校正 */
        float arc = Motion_AngleToWheelDist(angle_deg);
        Motion_SetBothSpeedAccel(speed_mm_s, accel_mm_s2);
        Motion_SendBothTarget(-arc, arc);

        HAL_StatusTypeDef result = Motion_WaitMotorsReached(MOTION_MOTOR_WAIT_TIMEOUT_MS);
        if (result != HAL_OK) return result;

        osDelay(200);  /* 等待稳定 */

        /* 读取陀螺仪实际角度 (已统一为电机约定: 左转正, 右转负) */
        float actualAngle = g_imu_angle_deg;
        actualAngleSum += actualAngle;

        /* 转回去 (为下一次做准备) */
        float backArc = Motion_AngleToWheelDist(-angle_deg);
        Motion_SetBothSpeedAccel(speed_mm_s, accel_mm_s2);
        Motion_SendBothTarget(-backArc, backArc);
        result = Motion_WaitMotorsReached(MOTION_MOTOR_WAIT_TIMEOUT_MS);
        if (result != HAL_OK) return result;
        osDelay(200);
    }

    /* 计算平均实际角度 */
    float avgActual = actualAngleSum / (float)runs;

    /* 反推真实轮距: realBase = cfgBase × cmdAngle / actualAngle */
    float realWheelBase = s_config.wheelBase_mm * (angle_deg / avgActual);

    /* 更新配置 + 全局变量 (调试器可查看) */
    s_config.wheelBase_mm = realWheelBase;
    g_calibWheelBase_mm   = realWheelBase;

    if (pResult != NULL)
    {
        *pResult = realWheelBase;
    }

    return HAL_OK;
}
