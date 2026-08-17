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

/* ======================== 锁定 / 释放命令 ======================== */
#define MOTOR_CMD_LOCK              0x5AU     /* 上电锁定 */
#define MOTOR_CMD_UNLOCK            0x4AU     /* 下电释放 */

/* ======================== 目标类型 ======================== */
#define MOTOR_TARGET_RELATIVE       0x00U     /* 相对位移 */
#define MOTOR_TARGET_ABSOLUTE       0x05U     /* 绝对位置 */
#define MOTOR_TARGET_ZERO           0x0AU     /* 置零: 将当前位置设为零点 (不是"回到零点") */
#define MOTOR_TARGET_EMERGENCY_STOP 0x0BU     /* 急停 */

/* ======================== 查询命令 Data[1] 值 ======================== */
#define MOTOR_QUERY_TEMP            0x06U     /* 当前温度 */
#define MOTOR_QUERY_POSITION        0x08U     /* 当前位置 */
#define MOTOR_QUERY_STATUS          0x25U     /* 状态 */
#define MOTOR_QUERY_VERSION         0x00U     /* 固件版本号 */

/* ======================== 错误码 ======================== */
#define MOTOR_ERR_NOT_REACHED       0x0102U  /* 任务未完成 / 未到位 */
#define MOTOR_ERR_STALL             0x0104U  /* 堵转 */
#define MOTOR_ERR_OVERTEMP          0x0105U  /* 超温 */
#define MOTOR_ERR_UNDERVOLTAGE      0x0107U  /* 欠压 */
#define MOTOR_ERR_LINE_BREAK        0x0108U  /* 断线 */

/* ======================== 电机状态结构体 ======================== */
typedef struct
{
    uint8_t  sn[MOTOR_SN_LEN];   /* 设备 SN 号 (7字节) */
    uint8_t  snValid;            /* 1 = SN 已获取 */
    int32_t  mainPosition;       /* 主通道位移 (0.1mm) */
    int32_t  slavePosition;      /* 从通道位移 (0.1mm) */
    int16_t  torque;             /* 当前扭矩值 */
    int16_t  torquePeak;         /* 扭矩峰值 (绝对值最大, 调试/回零用) */
    int16_t  torqueMin;          /* 扭矩最小值 (调试用) */
    int16_t  torqueMax;          /* 扭矩最大值 (调试用) */
    uint32_t torqueFrameCount;   /* 力矩帧接收计数 (调试: 0=未收到力矩帧) */
    uint8_t  torqueRaw[4];       /* 最近一帧力矩原始数据 (调试: 核对字节布局) */
    int16_t  temperature;        /* 温度 (0.1°C) */
    int32_t  currentPosition;    /* 当前位置 (0.1mm) */
    uint32_t firmwareVersion;    /* 固件版本号 */
    uint8_t  status;             /* Bit0=位置到位, Bit1=零位 */
    uint8_t  deviceId;           /* 设备号 */
    uint8_t  online;             /* 1 = 电机在线 (收到心跳/反馈) */
    uint8_t  positionReached;    /* 1 = 到位标志 (收到 0x0420+ID) */
    uint32_t feedbackCount;      /* 位移反馈帧累计计数 (新鲜度判据: 区分陈旧/最新剩余位移) */
    uint16_t lastErrorCode;      /* 最近一次错误码 */
} Motor_Status_t;

/* ======================== 全局电机状态 (多电机) ======================== */
extern volatile Motor_Status_t g_motor_status[MOTOR_MAX_COUNT];

/* SN 上报队列 (未分配设备号前, 支持多电机同时上报) */
#define MOTOR_SN_QUEUE_SIZE     5U

typedef struct
{
    uint8_t  sn[MOTOR_SN_LEN];   /* 上报的 SN */
    uint8_t  valid;              /* 1 = 有效 */
} Motor_SNEntry_t;

typedef struct
{
    Motor_SNEntry_t entries[MOTOR_SN_QUEUE_SIZE];
    uint8_t count;               /* 当前队列中有效 SN 数量 */
} Motor_SNQueue_t;

extern volatile Motor_SNQueue_t g_motor_sn_queue;

/* ======================== 方向定义 ======================== */
#define MOTOR_DIR_CW                0x00U     /* 顺时针 (Bit7=0) */
#define MOTOR_DIR_CCW               0x80U     /* 逆时针 (Bit7=1) */

/* ======================== 0x0316 默认配置参数 ======================== */
/* 协议文档 1.1 节示例: B4 BF CF AA 00 5F 01 01 */
#define MOTOR_CFG_BYTE0             0xB4U
#define MOTOR_CFG_BYTE1             0xBFU
#define MOTOR_CFG_BYTE2             0xCFU
#define MOTOR_CFG_BYTE3             0xAAU
#define MOTOR_CFG_BYTE4             0x00U
#define MOTOR_CFG_BYTE5             0x5FU

/* ======================== 接口函数声明 ======================== */

/**
  * @brief  主动获取电机 SN 号 (CAN ID 0x060D)
  *         发送后电机通过 0x0312 回复 SN
  * @param  deviceType  设备类型 (Data[0])
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_RequestSN(uint8_t deviceType);

/**
  * @brief  设置电机设备号 (CAN ID 0x0313)
  *         将 SN 与设备号绑定
  * @param  sn        7字节 SN 号
  * @param  deviceId  要分配的设备号 (1~MOTOR_MAX_COUNT)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_SetDeviceId(const uint8_t *sn, uint8_t deviceId);

/**
  * @brief  等待并获取电机上电上报的 SN (从队列 pop 一个)
  * @param  sn        输出: 7字节 SN
  * @param  timeout_ms 超时时间 (ms)
  * @retval HAL_OK 获取成功, HAL_TIMEOUT 超时
  */
HAL_StatusTypeDef Motor_WaitSN(uint8_t *sn, uint32_t timeout_ms);

/**
  * @brief  等待 SN 队列中至少有 needCount 个 SN (阻塞)
  * @param  needCount  需要的 SN 数量
  * @param  timeout_ms 超时时间 (ms)
  * @retval HAL_OK 成功, HAL_TIMEOUT 超时
  */
HAL_StatusTypeDef Motor_WaitSNQueue(uint8_t needCount, uint32_t timeout_ms);

/**
  * @brief  清空 SN 队列
  */
void Motor_ClearSNQueue(void);

/**
  * @brief  配置电机参数 (CAN ID 0x0316, 8字节)
  * @param  deviceId  设备号 (Byte7)
  * @param  motorId   马达号 0~15 (Byte6 bit[3:0])
  * @param  direction MOTOR_DIR_CW 顺时针 / MOTOR_DIR_CCW 逆时针
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_ConfigParam(uint8_t deviceId, uint8_t motorId, uint8_t direction);

/**
  * @brief  自定义配置电机参数 (CAN ID 0x0316, 8字节)
  *         允许为不同电机设置不同的 PID / 电流 / 阻尼参数
  * @param  deviceId  设备号 (Byte7)
  * @param  motorId   马达号 0~15 (Byte6 bit[3:0])
  * @param  direction MOTOR_DIR_CW / MOTOR_DIR_CCW
  * @param  byte0     极对数等 (通常 0xB4)
  * @param  byte1     速度环 P(高4位) + I(低4位)
  * @param  byte2     转矩环 P(高4位) + I(低4位)
  * @param  byte3     电流限制 顺(高4位) + 逆(低4位)
  * @param  byte4     刹车(bit2) + 阻尼(bit1~0)
  * @param  byte5     速度D(高4位) + 转矩D(低4位)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_ConfigParamCustom(uint8_t deviceId, uint8_t motorId, uint8_t direction,
                                          uint8_t byte0, uint8_t byte1, uint8_t byte2,
                                          uint8_t byte3, uint8_t byte4, uint8_t byte5);

/**
  * @brief  配置轮径 (需重复 3 次写入)
  * @param  deviceId   设备号
  * @param  wheelDiameter_mm  轮径 (mm, 浮点)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_ConfigWheelDiameter(uint8_t deviceId, float wheelDiameter_mm);

/**
  * @brief  锁定电机 (上电)
  * @param  deviceId  设备号
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_Lock(uint8_t deviceId);

/**
  * @brief  释放电机 (下电)
  * @param  deviceId  设备号
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_Unlock(uint8_t deviceId);

/**
  * @brief  设置主速度
  * @param  deviceId  设备号
  * @param  speed_mm_s  速度 (mm/s, 浮点)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_SetSpeed(uint8_t deviceId, float speed_mm_s);

/**
  * @brief  设置从速度
  * @param  deviceId  设备号
  * @param  speed_mm_s  速度 (mm/s, 浮点)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_SetSpeedSlave(uint8_t deviceId, float speed_mm_s);

/**
  * @brief  设置主加速度
  * @param  deviceId  设备号
  * @param  accel_mm_s2  加速度 (mm/s², 浮点)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_SetAcceleration(uint8_t deviceId, float accel_mm_s2);

/**
  * @brief  设置从加速度
  * @param  deviceId  设备号
  * @param  accel_mm_s2  加速度 (mm/s², 浮点)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_SetAccelerationSlave(uint8_t deviceId, float accel_mm_s2);

/**
  * @brief  设置目标位置 (主通道)
  * @param  deviceId    设备号
  * @param  position_mm  目标位置 (mm, 浮点)
  * @param  targetType   目标类型: MOTOR_TARGET_RELATIVE / ABSOLUTE / ZERO / EMERGENCY_STOP
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_SetTarget(uint8_t deviceId, float position_mm, uint8_t targetType);

/**
  * @brief  设置目标位置 (从通道, 无目标类型)
  * @param  deviceId    设备号
  * @param  position_mm  目标位置 (mm, 浮点)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_SetTargetSlave(uint8_t deviceId, float position_mm);

/**
  * @brief  急停
  * @param  deviceId  设备号
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_EmergencyStop(uint8_t deviceId);

/**
  * @brief  将当前位置设为零点 (目标类型 0x0A)
  *         注意: 0x0A 语义是"把当前位置定义为零点", 不是"回到零点"。
  *         机械限位回零流程: 低速撞限位 → 急停 → 回退 → 调用本函数置零。
  * @param  deviceId  设备号
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_SetZero(uint8_t deviceId);

/**
  * @brief  归零 (已废弃语义, 保留兼容): 等价于 Motor_SetZero
  *         新代码请使用 Motor_SetZero + FlyBox 力矩回零接口
  * @param  deviceId  设备号
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_Home(uint8_t deviceId);

/**
  * @brief  清零力矩统计 (torquePeak/torqueMin/torqueMax)
  *         回零/调试前调用, 便于观察本次运动中的力矩极值
  * @param  deviceId  设备号
  */
void Motor_ResetTorqueStats(uint8_t deviceId);

/**
  * @brief  查询当前温度
  * @param  deviceId  设备号
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_QueryTemperature(uint8_t deviceId);

/**
  * @brief  查询当前位置
  * @param  deviceId  设备号
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_QueryPosition(uint8_t deviceId);

/**
  * @brief  查询状态
  * @param  deviceId  设备号
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_QueryStatus(uint8_t deviceId);

/**
  * @brief  查询固件版本号
  * @param  deviceId  设备号
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_QueryVersion(uint8_t deviceId);

/**
  * @brief  CAN 接收帧处理: 在接收任务中调用，解析电机反馈帧
  *         根据 CAN ID 更新 g_motor_status 对应字段 (支持多电机)
  * @param  stdId  接收帧 ID
  * @param  pData  数据指针
  * @param  len    数据长度
  * @retval HAL_OK 已处理, HAL_ERROR 不属于电机帧
  */
HAL_StatusTypeDef Motor_ProcessRxFrame(uint32_t stdId, const uint8_t *pData, uint8_t len);

/**
  * @brief  单电机完整初始化序列:
  *           1. 配置电机参数
  *           2. 配置轮径
  *           3. 锁定电机
  *           4. 设置默认速度和加速度
  * @param  deviceId         设备号 (1~MOTOR_MAX_COUNT)
  * @param  wheelDiameter_mm 轮径 (mm)
  * @param  speed_mm_s       默认速度 (mm/s)
  * @param  accel_mm_s2      默认加速度 (mm/s²)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_Init(uint8_t deviceId, float wheelDiameter_mm,
                             float speed_mm_s, float accel_mm_s2);

/**
  * @brief  单电机完整初始化 (自定义 PID 参数):
  *           1. 自定义配置电机参数 (锁定前)
  *           2. 配置轮径
  *           3. 锁定电机
  *           4. 设置默认速度和加速度
  * @param  deviceId         设备号 (1~MOTOR_MAX_COUNT)
  * @param  wheelDiameter_mm 轮径 (mm)
  * @param  speed_mm_s       默认速度 (mm/s)
  * @param  accel_mm_s2      默认加速度 (mm/s²)
  * @param  direction        MOTOR_DIR_CW / MOTOR_DIR_CCW
  * @param  byte0~byte5      自定义 0x0316 配置字节 (见 Motor_ConfigParamCustom)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef Motor_InitCustom(uint8_t deviceId, float wheelDiameter_mm,
                                   float speed_mm_s, float accel_mm_s2,
                                   uint8_t direction,
                                   uint8_t byte0, uint8_t byte1, uint8_t byte2,
                                   uint8_t byte3, uint8_t byte4, uint8_t byte5);

/**
  * @brief  扫描总线上已在线的电机 (被动监听心跳/反馈帧)
  *         已绑定设备号的电机上电后会主动发送 0x380+ID / 0x420+ID
  * @param  scanTime_ms  扫描等待时间 (ms), 建议 500~1000
  * @retval 发现的在线电机数量
  */
uint8_t Motor_ScanOnline(uint32_t scanTime_ms);

/**
  * @brief  多电机完整初始化 (支持已绑定 + 未绑定混合):
  *           Phase 1: 扫描总线，发现已绑定设备号的电机
  *           Phase 2: 配置已在线电机
  *           Phase 3: 对未在线电机走 SN 绑定流程，再配置
  * @param  motorCount       电机数量 (1~MOTOR_MAX_COUNT)
  * @param  wheelDiameter_mm 轮径 (mm)
  * @param  speed_mm_s       默认速度 (mm/s)
  * @param  accel_mm_s2      默认加速度 (mm/s²)
  * @param  snTimeout_ms     等待每个电机SN的超时时间 (ms)
  * @retval HAL_OK 全部成功, HAL_ERROR 某个电机失败
  */
HAL_StatusTypeDef Motor_InitAll(uint8_t motorCount, float wheelDiameter_mm,
                                float speed_mm_s, float accel_mm_s2,
                                uint32_t snTimeout_ms);

/**
  * @brief  获取指定设备号的电机状态指针
  * @param  deviceId  设备号 (1~MOTOR_MAX_COUNT)
  * @retval 状态指针, 无效设备号返回 NULL
  */
volatile Motor_Status_t *Motor_GetStatus(uint8_t deviceId);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
