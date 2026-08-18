/**
  ******************************************************************************
  * @file    flybox_action.h
  * @brief   飞箱动作控制接口 (宏定义 / 函数声明)
  *
  *          5 电机 + 2 TOF 的取箱/放箱完整流程封装。
  *          依赖: motor.h, tof8.h, FreeRTOS (osDelay)
  ******************************************************************************
  */

#ifndef __FLYBOX_ACTION_H
#define __FLYBOX_ACTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

/* ======================== 电机设备号 (固定分配) ======================== */
#define FLYBOX_ID_CONVEYOR_L        1U   /* 传送带左 */
#define FLYBOX_ID_CONVEYOR_R        2U   /* 传送带右 (镜像) */
#define FLYBOX_ID_GRIPPER           3U   /* 钩爪 */
#define FLYBOX_ID_GRIPPER_ROTATE    4U   /* 钩爪旋转 */
#define FLYBOX_ID_GRIPPER_BELT      5U   /* 钩爪皮带 */
#define FLYBOX_MOTOR_COUNT          5U

/* ======================== 电机轮径 (mm, 来自带教配置) ======================== */
#define FLYBOX_WHEEL_DIAMETER_CONVEYOR       31.83f  /* 传送带 L/R */
#define FLYBOX_WHEEL_DIAMETER_GRIPPER         7.38f  /* 钩爪 */
#define FLYBOX_WHEEL_DIAMETER_GRIPPER_ROTATE  3.27f  /* 钩爪旋转 */
#define FLYBOX_WHEEL_DIAMETER_BELT            6.37f  /* 皮带 */

/* ======================== 钩爪角度换算 (mm → 度) ======================== */
/* 实测: 2890mm = 90° (2026-08-18 标定) */
#define FLYBOX_GRIPPER_MM_PER_DEG            32.111f  /* 电机3: 1° = 32.111mm */
#define FLYBOX_GRIPPER_ROTATE_MM_PER_DEG      1.018f  /* 电机4: 1° = 1.018mm (按轮径比例推算, 待实测) */
#define FLYBOX_GRIPPER_DEG_TO_MM(deg)        ((deg) * FLYBOX_GRIPPER_MM_PER_DEG)
#define FLYBOX_GRIPPER_ROTATE_DEG_TO_MM(deg) ((deg) * FLYBOX_GRIPPER_ROTATE_MM_PER_DEG)

/* ======================== 钩爪角度定义 (旧坐标系, 归零前) ======================== */
#define FLYBOX_GRIPPER_RELEASE_DEG          0.0f    /* 松开 */
#define FLYBOX_GRIPPER_HOOK_DEG          -90.0f    /* 钩住 */
#define FLYBOX_GRIPPER_ROTATE_DEG         90.0f    /* 旋转到侧面 */

/* ======================== 机械限位零点偏移 (限位 ≠ 旧零点) ======================== */
#define FLYBOX_HOME_OFFSET_GRIPPER_DEG         45.0f    /* 电机3: 限位=+45° */
#define FLYBOX_HOME_OFFSET_GRIPPER_ROTATE_DEG -45.0f    /* 电机4: 限位=-45° */
#define FLYBOX_HOME_OFFSET_GRIPPER_BELT_MM     0.0f     /* 电机5: 无偏移 */

/* ======================== 力矩回零参数 ======================== */
#define FLYBOX_HOME_SPEED_MM_S      3000.0f   /* 搜索回零方向行程 (电机3/4) */
#define FLYBOX_HOME_SPEED_MM_S_BELT  700.0f   /* 搜索回零方向行程 (电机5) */
#define FLYBOX_HOME_SEARCH_SPEED      30.0f   /* 低速搜索速度 (mm/s) */
#define FLYBOX_HOME_SEARCH_ACCEL     100.0f   /* 低速搜索加速度 (mm/s²) */
#define FLYBOX_HOME_TIMEOUT_MS      150000U   /* 搜索超时 (150s) */
#define FLYBOX_HOME_TORQUE_DEBOUNCE      3U   /* 力矩消抖次数 */
#define FLYBOX_HOME_BACKOFF_MM          2.0f   /* 回退距离 (mm) */
#define FLYBOX_HOME_FREE_MM            30.0f   /* 脱困探索距离 */
#define FLYBOX_HOME_FREE_TIMEOUT_MS   5000U   /* 脱困探索超时 */
#define FLYBOX_HOME_FREE_SETTLE_MS     200U   /* 脱困后稳定延时 */
#define FLYBOX_HOME_RETRY_MAX            2U   /* 脱困重试最大次数 */
#define FLYBOX_HOME_FREE_ROCK1_MM      20.0f   /* 摇摆级1 */
#define FLYBOX_HOME_FREE_ROCK2_MM      60.0f   /* 摇摆级2 */
#define FLYBOX_HOME_FREE_ROCK3_MM     100.0f   /* 摇摆级3 */

/* ======================== 力矩回零阈值 (实测标定值) ======================== */
#define FLYBOX_HOME_TORQUE_THRESH_GRIPPER        90   /* 电机3 */
#define FLYBOX_HOME_TORQUE_THRESH_GRIPPER_ROTATE 90   /* 电机4 */
#define FLYBOX_HOME_TORQUE_THRESH_GRIPPER_BELT   90   /* 电机5 */

/* ======================== 默认运动参数 ======================== */
#define FLYBOX_DEFAULT_SPEED         500.0f   /* 默认速度 (mm/s) */
#define FLYBOX_DEFAULT_ACCEL        1000.0f   /* 默认加速度 (mm/s²) */
#define FLYBOX_ROTATE_SPEED          500.0f   /* 旋转速度 */
#define FLYBOX_ROTATE_ACCEL          500.0f   /* 旋转加速度 */
#define FLYBOX_MOTOR_WAIT_TIMEOUT_MS 30000U  /* 到位等待超时 (30s) */
#define FLYBOX_STALL_REACHED_TOL_RAW    30U  /* 堵转保护帧豁免容差 (0.1mm) */
#define FLYBOX_STEP_SETTLE_MS          100U  /* 步骤间稳定延时 */
#define FLYBOX_RETRY_SETTLE_MS        2000U  /* 重试前稳定延时 */
#define FLYBOX_ROTATE_SETTLE_MS        100U  /* 旋转后稳定延时 */

/* ======================== 取箱参数 ======================== */
#define FLYBOX_PICK_MAX_ATTEMPTS        3U    /* 最大重试次数 */
#define FLYBOX_TOF25_HOOK_THRESH_MM    95.0f  /* Step1: TOF25 钩取阈值 */
#define FLYBOX_TOF25_VERIFY_MAX_MM    150.0f  /* Step3.5: 验证箱体在传送带上 */
#define FLYBOX_BELT_MAX_FWD_MM        500.0f  /* 皮带前进最大限程 */

/* ======================== 放箱参数 ======================== */
#define FLYBOX_PLACE_CONVEYOR_BACK_MM      550.0f  /* Step3: 传送带后退基准量 */
#define FLYBOX_PLACE_BELT_FWD_MM           500.0f  /* Step3: 皮带前进量 */
#define FLYBOX_PLACE_SYNC_RETREAT_MM       180.0f  /* Step1+2: 旋转同步后退量 */
#define FLYBOX_HOOK_MARGIN_MM               15.0f  /* 安全余量 */
#define FLYBOX_TOF25_PLACE_EXPECT_MM       100.0f  /* Step2.5: TOF25 期望距离 */
#define FLYBOX_PLACE_COMP_MAX_MM            30.0f  /* Step2.5: 补偿限幅 */

/* ======================== 旋转同步物理模型 ======================== */
#define FLYBOX_HOOK_RADIUS_MM              215.0f  /* 钩爪旋转半径 */
#define FLYBOX_HOOK_FINAL_MM                50.0f  /* 同步后目标距离 */
#define FLYBOX_PLACE_TOF26_TARGET_MM       215.0f  /* 清障目标距离 */

/* ======================== TOF 联动参数 ======================== */
#define TOF8_DEVICE_ID_0                    25U    /* TOF25 = deviceId 0 */
#define TOF8_DEVICE_ID_1                    26U    /* TOF26 = deviceId 1 */
#define FLYBOX_TOF_MEDIAN_SAMPLES             5U
#define FLYBOX_TOF_MEDIAN_INTERVAL_MS        30U
#define FLYBOX_TOF_CONVERGE_MM              5.0f   /* 收敛容差 */
#define FLYBOX_TOF_SEARCH_STEP_MM           50.0f   /* 盲进步长 */
#define FLYBOX_TOF_SETTLE_MS               500U    /* 机械稳定延时 */
#define FLYBOX_TOF_MAX_ITER                 15U    /* 最大迭代次数 */
#define FLYBOX_TOF_ALIVE_TIMEOUT_MS       1000U    /* 在线判定超时 */
#define FLYBOX_TOF_RESTART_RETRY             3U    /* 重启重试次数 */

/* ======================== 钩爪自定义 PID (0x0316, 电机3 专用, 来自带教配置) ======================== */
#define FLYBOX_GRIPPER_CFG_BYTE0  0xB4U   /* 极对数 */
#define FLYBOX_GRIPPER_CFG_BYTE1  0xBFU   /* 速度 PI */
#define FLYBOX_GRIPPER_CFG_BYTE2  0xCFU   /* 转矩 PI */
#define FLYBOX_GRIPPER_CFG_BYTE3  0xEEU   /* 电流限制 */
#define FLYBOX_GRIPPER_CFG_BYTE4  0x00U   /* 阻尼 */
#define FLYBOX_GRIPPER_CFG_BYTE5  0x6FU   /* D 参数 */

/* ======================== 状态枚举 ======================== */
typedef enum
{
    FLYBOX_IDLE     = 0,   /* 空闲 */
    FLYBOX_BUSY     = 1,   /* 执行中 */
    FLYBOX_COMPLETE = 2,   /* 完成 */
    FLYBOX_ERROR    = 3    /* 错误 */
} FlyBox_State_t;

/* ======================== 回零诊断 ======================== */
typedef struct
{
    uint8_t  deviceId;
    uint8_t  phase;
    uint8_t  failReason;
    uint8_t  zeroAtLimit;
    uint16_t errCode;
    int16_t  torqueAtLimit;
    int16_t  torquePeak;
    uint32_t searchMs;
    uint32_t backoffElapsed;
    int32_t  backoffPosition;
    int16_t  backoffTorque;
    uint8_t  freeStage;
    uint8_t  retryCount;
} FlyBox_HomeDiag_t;

/* ======================== 取箱诊断 ======================== */
typedef struct
{
    uint8_t  lastStep;
    uint8_t  s1, s2, s3, s4;
    uint8_t  verify;
    float    verifyDist;
    uint8_t  attempts;
    uint8_t  s6Recoveries;
    uint8_t  s6Clear;
    uint8_t  beltLimitHit;
    uint8_t  syncSteps;
    float    syncEndMm;
    int32_t  gripperPosAtAbort;
    int16_t  gripperTorqueAtAbort;
    uint8_t  abortReleaseResult;
} FlyBox_PickDiag_t;

/* ======================== 等待诊断 ======================== */
typedef struct
{
    uint8_t  deviceId;
    uint8_t  result;
    uint16_t errCode;
    int32_t  mainPosition;
    uint8_t  motorStarted;
    uint32_t elapsedMs;
} FlyBox_WaitDiag_t;

/* ======================== TOF26 联动诊断 ======================== */
typedef struct
{
    float    firstDist;
    float    exitDist;
    float    target;
    int8_t   direction;
    uint8_t  exitReason;
    uint8_t  iters;
    uint8_t  warmWaits;
    uint8_t  blindSteps;
} FlyBox_Tof26Diag_t;

/* ======================== 全局诊断变量 ======================== */
extern FlyBox_HomeDiag_t  g_home_diag;
extern FlyBox_PickDiag_t  g_pick_diag;
extern FlyBox_WaitDiag_t  g_wait_diag;
extern FlyBox_Tof26Diag_t g_tof26_diag;

/* ======================== 接口函数 ======================== */

/* 初始化 */
HAL_StatusTypeDef FlyBox_Init(void);

/* 力矩回零 */
HAL_StatusTypeDef FlyBox_HomeAll(void);
HAL_StatusTypeDef FlyBox_HomeMotorByTorque(uint8_t deviceId);
HAL_StatusTypeDef FlyBox_GripperHome(void);
HAL_StatusTypeDef FlyBox_GripperRotateHome(void);
HAL_StatusTypeDef FlyBox_GripperBeltHome(void);

/* 钩爪控制 */
HAL_StatusTypeDef FlyBox_GripperHook(void);
HAL_StatusTypeDef FlyBox_GripperRelease(void);
HAL_StatusTypeDef FlyBox_GripperRotateAbs(float angle_deg, float speed_mm_s, float accel_mm_s2);
HAL_StatusTypeDef FlyBox_GripperTurnDeg(float angle_deg, float speed_mm_s, float accel_mm_s2);
HAL_StatusTypeDef FlyBox_GripperRotateDeg(float angle_deg, float speed_mm_s, float accel_mm_s2);

/* 皮带控制 */
HAL_StatusTypeDef FlyBox_GripperBeltHome(void);
HAL_StatusTypeDef FlyBox_GripperBelttoHome(void);
HAL_StatusTypeDef FlyBox_GripperBeltMove(float distance_mm, float speed_mm_s, float accel_mm_s2);
HAL_StatusTypeDef FlyBox_GripperBeltMoveUntilTof25(float target_mm, float speed_mm_s, float accel_mm_s2);

/* 传送带控制 */
HAL_StatusTypeDef FlyBox_ConveyorRun(float distance_mm, float speed_mm_s, float accel_mm_s2);
HAL_StatusTypeDef FlyBox_ConveyorRunUntilTof26(float target_mm, float speed_mm_s, float accel_mm_s2, int8_t direction);

/* 高层流程 */
HAL_StatusTypeDef FlyBox_PickBox(void);
HAL_StatusTypeDef FlyBox_PlaceBox(void);
void              FlyBox_AbortAndSafe(void);

/* 状态查询 */
FlyBox_State_t FlyBox_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLYBOX_ACTION_H */