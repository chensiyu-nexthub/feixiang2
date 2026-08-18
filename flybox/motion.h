/**
  ******************************************************************************
  * @file    motion.h
  * @brief   差速小车运动控制层接口
  *
  *          基于 motor.c (位置模式) + imu_core.c (陀螺仪角度) 实现:
  *            - 电机绑定与初始化 (SN获取 → 设备号分配 → 参数配置)
  *            - 直行控制 (距离/速度/加速度)
  *            - 原地转向控制 (角度/角速度/角加速度, 陀螺仪校正)
  *            - 停止 / 状态查询
  *
  *          依赖: motor.c, imu_core.c, FreeRTOS
  ******************************************************************************
  */



#ifndef __MOTION_H
#define __MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>
#include <FreeRTOS.h>
#include <semphr.h>

/* ======================== 默认参数 ======================== */
#define MOTION_DEFAULT_WHEEL_BASE_MM        384.5f   /* 左右轮间距 (mm) */
#define MOTION_DEFAULT_WHEEL_DIAMETER_MM    25.55f   /* 等效轮径 (mm), 经减速比换算 */
#define MOTION_DEFAULT_LEFT_ID              1U       /* 左轮电机设备号 */
#define MOTION_DEFAULT_RIGHT_ID             2U       /* 右轮电机设备号 */

/* 电机 SN 标定值 (用于区分左右轮) */
#define MOTION_LEFT_MOTOR_SN    {0x15, 0x09, 0x10, 0x07, 0x85, 0x99, 0x00}
#define MOTION_RIGHT_MOTOR_SN   {0x15, 0x09, 0x08, 0x07, 0x84, 0x89, 0x00}

// #define MOTION_RIGHT_MOTOR_SN      {0x19U, 0x0AU, 0x09U, 0x07U, 0x06U, 0xD1U, 0x00U}
// #define MOTION_LEFT_MOTOR_SN       {0x19U, 0x0AU, 0x0AU, 0x07U, 0xFCU, 0xD1U, 0x00U}
//yu
// #define MOTION_LEFT_MOTOR_SN   {0x19, 0x0A, 0x1F, 0x07, 0xA0, 0xFC, 0x00}
// #define MOTION_RIGHT_MOTOR_SN  {0x19, 0x09, 0x17, 0x07, 0x9E, 0xAC, 0x00}

#define MOTION_TURN_CORRECTION_THRESH_DEG   0.1f     /* 转向校正阈值 (°) */
#define MOTION_TURN_MAX_CORRECTIONS         3U       /* 最大校正次数 */
#define MOTION_TURN_OVERSHOOT_THRESH_DEG    2.0f     /* 过冲急停阈值 (°) */

#define MOTION_MOTOR_WAIT_TIMEOUT_MS        30000U   /* 电机到位默认超时 (ms) */
#define MOTION_SN_TIMEOUT_MS                3000U    /* SN 等待超时 (ms) */

/* 直行纠偏默认参数 (速度修正方式) */
#define MOTION_STRAIGHT_KP                  40.0f    /* 航向 PID 比例系数 (输出=速度修正 mm/s) */
#define MOTION_STRAIGHT_KI                  0.0f     /* 航向 PID 积分系数 */
#define MOTION_STRAIGHT_KD                  0.0f     /* 航向 PID 微分系数 */
#define MOTION_STRAIGHT_CORR_INTERVAL_MS    100U     /* 修正周期 (ms) */
#define MOTION_STRAIGHT_MAX_SPEED_CORR      50.0f   /* 单轮最大速度修正量 (mm/s) */
#define MOTION_STRAIGHT_HEADING_THRESH_DEG  0.2f     /* 航向误差死区 (°) */
#define MOTION_STRAIGHT_END_THRESH_DEG      0.1f     /* 末端精调阈值 (°) */

/* 标定 Flash 存储地址 (STM32F103xB 最后一页, 1KB) */
#define MOTION_CALIB_FLASH_ADDR             0x0801FC00U
#define MOTION_CALIB_MAGIC                  0xCA11B001U

/* ======================== 配置结构体 ======================== */

/**
  * @brief  小车运动控制配置 (初始化时传入)
  */
typedef struct
{
    float    wheelBase_mm;         /* 左右轮间距 (mm) */
    float    wheelDiameter_mm;     /* 轮径 (mm) */
    uint8_t  leftMotorId;          /* 左轮电机设备号 */
    uint8_t  rightMotorId;         /* 右轮电机设备号 */
    float    turnCorrectionThresh; /* 转向校正阈值 (°), 残余误差超过此值则修正 */
    uint8_t  maxCorrections;       /* 最大校正轮数 */
    float    overshootThresh;      /* 转向过冲急停阈值 (°) */
    uint8_t  invertRightMotor;     /* 1 = 右轮方向取反 (差速车通常需要) */

    /* 直行纠偏参数 (速度修正方式) */
    float    straightKp;           /* 航向 PID 比例系数 (输出=速度修正 mm/s) */
    float    straightKi;           /* 航向 PID 积分系数 */
    float    straightKd;           /* 航向 PID 微分系数 */
    uint32_t straightCorrInterval_ms; /* 修正周期 (ms) */
    float    straightMaxSpeedCorr; /* 单轮最大速度修正量 (mm/s) */
    float    straightHeadingThresh;/* 航向误差死区 (°) */
    float    straightEndThresh;    /* 末端精调阈值 (°) */
    uint8_t  straightEndMode;      /* 末端精调模式: 0=双轮原地转, 1=左轮单独修正(绕右轮) */

    /* 标定数据 (由 Motion_Calibrate 填充, 0 = 未标定/纯PID) */
    float    calibBias_deg_per_m;  /* 系统偏差 (°/m), 前馈补偿用 */
} Motion_Config_t;

/* ======================== 标定模式 ======================== */

typedef enum
{
    MOTION_CALIB_LOAD_FLASH = 0,  /* 从 Flash 加载标定数据 (无数据则返回错误) */
    MOTION_CALIB_RUN_NEW    = 1   /* 执行标定 + 更新 Flash */
} Motion_CalibMode_t;

/* ======================== 状态枚举 ======================== */

typedef enum
{
    MOTION_IDLE = 0,      /* 空闲, 可接受新命令 */
    MOTION_BUSY,          /* 运动执行中 */
    MOTION_COMPLETE,      /* 最近一次运动完成 */
    MOTION_ERROR          /* 错误 (超时/电机离线/堵转等) */
} Motion_State_t;

/* ======================== 错误码 ======================== */

#define MOTION_OK               HAL_OK
#define MOTION_ERR_TIMEOUT      0x10U   /* 等待电机到位超时 */
#define MOTION_ERR_MOTOR_OFFLINE 0x11U  /* 电机离线 */
#define MOTION_ERR_STALL        0x12U   /* 堵转 */
#define MOTION_ERR_INVALID      0x13U   /* 参数无效 */
#define MOTION_ERR_NOT_INIT     0x14U   /* 未初始化 */

/* ======================== 接口函数 ======================== */

/**
  * @brief  运动控制初始化: 绑定电机 → 配置参数 → 锁定
  *         流程: RequestSN → WaitSN → SetDeviceId × 2 → Motor_Init × 2
  * @param  pConfig  配置指针 (NULL 使用默认值)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motion_Init(const Motion_Config_t *pConfig);

/**
  * @brief  直行 (阻塞, 直到到位或超时)
  * @param  distance_mm   距离 (mm, 正=前进, 负=后退)
  * @param  speed_mm_s    速度 (mm/s, >0)
  * @param  accel_mm_s2   加速度 (mm/s², >0)
  * @retval HAL_OK 到位, HAL_TIMEOUT 超时, HAL_ERROR 其他错误
  */
HAL_StatusTypeDef Motion_GoStraight(float distance_mm, float speed_mm_s, float accel_mm_s2);

/**
  * @brief  原地转向 (阻塞, 位置模式 + 陀螺仪校正)
  * @param  angle_deg         目标角度 (°, 正=逆时针/左转, 负=顺时针/右转)
  * @param  angular_speed_dps 角速度 (°/s, >0)
  * @param  angular_accel     角加速度 (°/s², >0)
  * @retval HAL_OK 到位, HAL_TIMEOUT 超时, HAL_ERROR 其他错误
  */
HAL_StatusTypeDef Motion_Turn(float angle_deg, float angular_speed_dps, float angular_accel);

/**
  * @brief  陀螺仪角度修正 (阻塞, 连续修正)
  *         内部: 发主目标 → 每 corrInterval 读 IMU 角度 → PID → 发修正量
  * @param  targetAngle_deg   目标角度 (°, 正=逆时针/左转, 负=顺时针/右转)
  * @param  speed_mm_s        修正速度 (mm/s, >0)
  * @param  accel_mm_s2       修正加速度 (mm/s², >0)
  * @param  leftOnly          0=双轮反向原地转(默认), 1=仅左轮修正(绕右轮转)
  * @retval HAL_OK 到位, HAL_TIMEOUT 超时, HAL_ERROR 其他错误
  */
HAL_StatusTypeDef Motion_GyroCorrect(float targetAngle_deg, float speed_mm_s, float accel_mm_s2, uint8_t leftOnly); 

/**
  * @brief  直行 + 陀螺仪 PID 纠偏 (阻塞, 连续修正)
  *         内部: 发主目标 → 每 corrInterval 读 IMU 角度 → PID → 发修正量
  *         标定数据 (calibBias) 自动生效: 有则前馈补偿, 无则纯 PID
  * @param  distance_mm   距离 (mm, 正=前进, 负=后退)
  * @param  speed_mm_s    速度 (mm/s, >0)
  * @param  accel_mm_s2   加速度 (mm/s², >0)
  * @retval HAL_OK 到位, HAL_TIMEOUT 超时, HAL_ERROR 其他错误
  */
HAL_StatusTypeDef Motion_GoStraightCorrected(float distance_mm, float speed_mm_s, float accel_mm_s2);

/**
  * @brief  直行标定 (测量系统偏差)
  * @param  mode        MOTION_CALIB_LOAD_FLASH: 从 Flash 加载
  *                     MOTION_CALIB_RUN_NEW: 跑标定 + 写 Flash
  * @param  dist_mm     标定距离 (mm), LOAD_FLASH 模式忽略
  * @param  speed_mm_s  标定速度 (mm/s), LOAD_FLASH 模式忽略
  * @param  accel_mm_s2 标定加速度 (mm/s²), LOAD_FLASH 模式忽略
  * @param  runs        标定次数, LOAD_FLASH 模式忽略
  * @retval HAL_OK 成功, HAL_ERROR 失败 (Flash 无数据/标定异常)
  */
HAL_StatusTypeDef Motion_Calibrate(Motion_CalibMode_t mode,
                                    float dist_mm, float speed_mm_s,
                                    float accel_mm_s2, uint8_t runs);

/**
  * @brief  轮距标定: 开环转固定角度, 用陀螺仪测量实际角度, 反推真实轮距
  *         结果自动更新 s_config.wheelBase_mm
  * @param  angle_deg    标定转角 (°, 建议 90 或 180)
  * @param  speed_mm_s   电机线速度 (mm/s)
  * @param  accel_mm_s2  电机线加速度 (mm/s²)
  * @param  runs         重复次数 (取平均)
  * @param  pResult      输出: 标定得到的轮距 (mm), 可传 NULL
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motion_CalibrateWheelBase(float angle_deg, float speed_mm_s,
                                             float accel_mm_s2, uint8_t runs,
                                             float *pResult);

/**
  * @brief  立即停止 (双电机急停)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motion_Stop(void);

/**
  * @brief  释放电机 (下电)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motion_Unlock(void);

/**
  * @brief  获取当前运动状态
  */
Motion_State_t Motion_GetState(void);

/**
  * @brief  获取最近一次错误码
  */
uint8_t Motion_GetLastError(void);

/* ======================== 共享接口 (供 imu.c Task_CanRx 调用) ======================== */

/**
  * @brief  获取电机到位信号量句柄 (Task_CanRx 中释放)
  * @retval 信号量句柄, 未初始化返回 NULL
  */
SemaphoreHandle_t Motion_GetReachSem(void);

/**
  * @brief  通知电机到位 (由 Task_CanRx 在解析到 0x0420+ID 时调用)
  * @param  deviceId  到位的电机设备号
  */
void Motion_NotifyMotorReached(uint8_t deviceId);

/* ======================== IMU 角度共享 (imu.c 定义, motion.c 只读) ======================== */

/**
  * @brief  IMU 当前角度 (°, 由 imu.c Task_Main 每周期更新)
  *         motion.c 通过 extern 直接读取, 无需额外函数
  */
extern volatile float g_imu_angle_deg;

/**
  * @brief  IMU 当前角速度 (°/s, 由 imu.c Task_Main 每周期更新)
  */
extern volatile float g_imu_rate_dps;

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_H */
