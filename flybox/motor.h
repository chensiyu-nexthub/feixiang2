/**
  ******************************************************************************
  * @file    motor.h
  * @brief   电机 CAN 通讯驱动接口 (V1.1)
  *
  *          基于 电机CAN通讯协议控制指南 (V1.1) 实现:
  *            - 初始化配置 (电机参数 / 轮径 / 锁定)
  *            - 运动参数设置 (速度 / 加速度)
  *            - 目标位置控制 (相对 / 绝对 / 归零 / 急停)
  *            - 反馈解析 (到位位移 / 力矩 / 心跳 / 错误码)
  *            - 查询命令 (温度 / 位置 / 状态 / 固件版本)
  *
  *          依赖: can.c (CAN_SendFrame), FreeRTOS (osDelay)
  ******************************************************************************
  */

#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>
#include "config.h"

/* ======================== CAN ID 定义 ======================== */
#define MOTOR_ID_SN_REPORT          0x0312U   /* 电机上电上报 SN (7字节) */
#define MOTOR_ID_SET_DEVICE_ID      0x0313U   /* 设置设备号 (SN+设备号, 8字节) */
#define MOTOR_ID_GET_SN             0x060DU   /* 主动获取电机 SN */
#define MOTOR_ID_CONFIG_PARAM       0x0316U   /* 配置电机参数 */
#define MOTOR_ID_QUERY              0x0408U   /* 查询 / 配置轮径 */
#define MOTOR_ID_LOCK               0x0413U   /* 锁定 / 释放电机 */
#define MOTOR_ID_ACCEL_MAIN         0x0414U   /* 主加速度 */
#define MOTOR_ID_SPEED_MAIN         0x0415U   /* 主速度 */
#define MOTOR_ID_TARGET_MAIN        0x0416U   /* 主目标位置 */
#define MOTOR_ID_ACCEL_SLAVE        0x0417U   /* 从加速度 */
#define MOTOR_ID_SPEED_SLAVE        0x0418U   /* 从速度 */
#define MOTOR_ID_TARGET_SLAVE       0x0419U   /* 从目标位置 */
#define MOTOR_ID_ACK                0x041AU   /* 参数设置 ACK */
#define MOTOR_ID_FEEDBACK_BASE      0x0420U   /* 到位位移反馈 = base + device_id */
#define MOTOR_ID_QUERY_REPLY        0x0409U   /* 查询返回 */
#define MOTOR_ID_TORQUE_BASE        0x0440U   /* 力矩反馈 = base + device_id */
#define MOTOR_ID_HEARTBEAT_BASE     0x0380U   /* 心跳反馈 = base + device_id */
#define MOTOR_ID_ERROR_BASE         0x0100U   /* 错误码上报范围 0x0100~0x017F */

/* ======================== 多电机配置 ======================== */
#define MOTOR_MAX_COUNT             5U        /* 最大支持电机数量 */
#define MOTOR_SN_LEN                7U        /* SN 号长度 (字节) */
#define MOTOR_SN_QUEUE_SIZE         10U       /* SN 队列容量 */

/* ======================== 方向定义 ======================== */
#define MOTOR_DIR_CW                0x80U     /* 正向 (bit7=1) */
#define MOTOR_DIR_CCW               0x00U     /* 反向 (bit7=0) */

/* ======================== 默认 PID 参数 (0x0316), 来自带教配置 ======================== */
#define MOTOR_CFG_BYTE0             0xB4U     /* 极对数=4 */
#define MOTOR_CFG_BYTE1             0xBFU     /* 速度P=11, I=15 */
#define MOTOR_CFG_BYTE2             0xCFU     /* 转矩P=12, I=15 */
#define MOTOR_CFG_BYTE3             0xEEU     /* 电流限制=14 */
#define MOTOR_CFG_BYTE4             0x00U     /* 刹车关闭, 无阻尼 */
#define MOTOR_CFG_BYTE5             0x6FU     /* 速度D=6, 转矩D=15 */

/* ======================== 锁定 / 释放命令 ======================== */
#define MOTOR_CMD_LOCK              0x5AU     /* 上电锁定 */
#define MOTOR_CMD_UNLOCK            0x4AU     /* 下电释放 */

/* ======================== 目标类型 ======================== */
#define MOTOR_TARGET_RELATIVE       0x00U     /* 相对位移 */
#define MOTOR_TARGET_ABSOLUTE       0x05U     /* 绝对位置 */
#define MOTOR_TARGET_ZERO           0x0AU     /* 置零: 将当前位置设为零点 */
#define MOTOR_TARGET_EMERGENCY_STOP 0x0BU     /* 急停 */

/* ======================== 查询命令 ======================== */
#define MOTOR_QUERY_TEMP            0x06U     /* 当前温度 */
#define MOTOR_QUERY_POSITION        0x08U     /* 当前位置 */
#define MOTOR_QUERY_STATUS          0x25U     /* 状态 */
#define MOTOR_QUERY_VERSION         0x00U     /* 固件版本号 */

/* ======================== 错误码 ======================== */
#define MOTOR_ERR_NOT_REACHED       0x0102U   /* 任务未完成 / 未到位 */
#define MOTOR_ERR_STALL             0x0104U   /* 堵转 */
#define MOTOR_ERR_OVERTEMP          0x0105U   /* 超温 */
#define MOTOR_ERR_UNDERVOLTAGE      0x0107U   /* 欠压 */
#define MOTOR_ERR_LINE_BREAK        0x0108U   /* 断线 */

/* ======================== 电机状态结构体 ======================== */
typedef struct
{
    uint8_t  sn[MOTOR_SN_LEN];   /* 设备 SN 号 (7字节) */
    uint8_t  snValid;            /* 1 = SN 已获取 */
    int32_t  mainPosition;       /* 主通道位移 (0.1mm) */
    int32_t  slavePosition;      /* 从通道位移 (0.1mm) */
    int16_t  torque;             /* 当前扭矩值 */
    int16_t  torquePeak;         /* 扭矩峰值 (绝对值最大) */
    int16_t  torqueMin;          /* 扭矩最小值 */
    int16_t  torqueMax;          /* 扭矩最大值 */
    uint8_t  torqueRaw[4];       /* 力矩帧原始数据 (调试用) */
    uint32_t torqueFrameCount;   /* 力矩帧计数 */
    uint32_t feedbackCount;      /* 位移反馈帧计数 (新鲜度判据) */
    uint8_t  online;             /* 1 = 电机在线 (有心跳/反馈) */
    uint8_t  positionReached;    /* 1 = 到位 (mainPosition==0) */
    uint8_t  deviceId;           /* 设备号 */
    uint16_t lastErrorCode;      /* 最近错误码 */
    int32_t  currentPosition;    /* 查询返回的当前位置 */
} Motor_Status_t;

/* SN 队列条目 */
typedef struct
{
    uint8_t sn[MOTOR_SN_LEN];
    uint8_t valid;
} Motor_SNEntry_t;

/* SN 队列 */
typedef struct
{
    Motor_SNEntry_t entries[MOTOR_SN_QUEUE_SIZE];
    uint8_t         count;
} Motor_SNQueue_t;

/* ======================== 全局变量 ======================== */
extern volatile Motor_Status_t  g_motor_status[MOTOR_MAX_COUNT];
extern volatile Motor_SNQueue_t g_motor_sn_queue;
extern MotorConfig_Run_t        g_cfg_motor[MOTOR_MAX_COUNT];  /* Flash 配置 */

/* ======================== SN 获取与设备号设置 ======================== */

HAL_StatusTypeDef Motor_RequestSN(uint8_t deviceType);
HAL_StatusTypeDef Motor_SetDeviceId(const uint8_t *sn, uint8_t deviceId);
HAL_StatusTypeDef Motor_WaitSN(uint8_t *sn, uint32_t timeout_ms);
HAL_StatusTypeDef Motor_WaitSNQueue(uint8_t needCount, uint32_t timeout_ms);
void              Motor_ClearSNQueue(void);
volatile Motor_Status_t *Motor_GetStatus(uint8_t deviceId);

/* ======================== 初始化配置 ======================== */

HAL_StatusTypeDef Motor_ConfigParam(uint8_t deviceId, uint8_t motorId, uint8_t direction);
HAL_StatusTypeDef Motor_ConfigParamCustom(uint8_t deviceId, uint8_t motorId, uint8_t direction,
                                          uint8_t byte0, uint8_t byte1, uint8_t byte2,
                                          uint8_t byte3, uint8_t byte4, uint8_t byte5);
HAL_StatusTypeDef Motor_ConfigWheelDiameter(uint8_t deviceId, float wheelDiameter_mm);
HAL_StatusTypeDef Motor_Lock(uint8_t deviceId);
HAL_StatusTypeDef Motor_Unlock(uint8_t deviceId);

/* ======================== 运动参数设置 ======================== */

HAL_StatusTypeDef Motor_SetDirection(uint8_t deviceId, uint8_t direction);
HAL_StatusTypeDef Motor_SetSpeed(uint8_t deviceId, float speed_mm_s);
HAL_StatusTypeDef Motor_SetSpeedSlave(uint8_t deviceId, float speed_mm_s);
HAL_StatusTypeDef Motor_SetAcceleration(uint8_t deviceId, float accel_mm_s2);
HAL_StatusTypeDef Motor_SetAccelerationSlave(uint8_t deviceId, float accel_mm_s2);

/* ======================== 目标位置控制 ======================== */

HAL_StatusTypeDef Motor_SetTarget(uint8_t deviceId, float position_mm, uint8_t targetType);
HAL_StatusTypeDef Motor_SetTargetSlave(uint8_t deviceId, float position_mm);
HAL_StatusTypeDef Motor_EmergencyStop(uint8_t deviceId);
HAL_StatusTypeDef Motor_SetZero(uint8_t deviceId);
HAL_StatusTypeDef Motor_Home(uint8_t deviceId);
void              Motor_ResetTorqueStats(uint8_t deviceId);

/* ======================== 查询命令 ======================== */

HAL_StatusTypeDef Motor_QueryTemperature(uint8_t deviceId);
HAL_StatusTypeDef Motor_QueryPosition(uint8_t deviceId);
HAL_StatusTypeDef Motor_QueryStatus(uint8_t deviceId);
HAL_StatusTypeDef Motor_QueryVersion(uint8_t deviceId);

/* ======================== 接收反馈处理 ======================== */

HAL_StatusTypeDef Motor_ProcessRxFrame(uint32_t stdId, const uint8_t *pData);

/* ======================== Flash 配置加载 / 保存 ======================== */

HAL_StatusTypeDef Motor_LoadConfig(uint8_t reset);
HAL_StatusTypeDef Motor_SaveConfig(void);

/* ======================== 完整初始化序列 ======================== */

HAL_StatusTypeDef Motor_Init(uint8_t deviceId, float wheelDiameter_mm,
                             float speed_mm_s, float accel_mm_s2);
HAL_StatusTypeDef Motor_InitCustom(uint8_t deviceId, float wheelDiameter_mm,
                                   float speed_mm_s, float accel_mm_s2,
                                   uint8_t direction,
                                   uint8_t byte0, uint8_t byte1, uint8_t byte2,
                                   uint8_t byte3, uint8_t byte4, uint8_t byte5);
uint8_t          Motor_ScanOnline(uint32_t scanTime_ms);
HAL_StatusTypeDef Motor_InitAll(uint8_t motorCount, float wheelDiameter_mm,
                                float speed_mm_s, float accel_mm_s2,
                                uint32_t snTimeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */