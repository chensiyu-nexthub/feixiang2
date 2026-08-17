/**
  ******************************************************************************
  * @file    flybox_action.c
  * @brief   飞箱动作模块实现
  *
  *          管理 5 个电机 + 2 个 TOF 传感器, 实现取箱 / 放箱自动化流程。
  *          电机 SN 绑定与运动控制复用 motor.c 接口。
  *
  *          依赖: motor.c, tof8.c, FreeRTOS
  ******************************************************************************
  */

#include "flybox_action.h"
#include "motor.h"
#include "tof8.h"
#include <math.h>
#include <cmsis_os.h>

/* ======================== 模块状态 ======================== */
static volatile FlyBox_State_t s_state = FLYBOX_IDLE;
static uint8_t s_initialized = 0U;
static uint8_t s_homed = 0U;   /* 归零完成标志: 1=已归零, 绝对定位可用 */
static float   s_belt_fwd_mm = 0.0f;  /* 钩爪传送带累计前进距离 (Step1 记录, Step3 使用) */
static uint8_t s_belt_fwd_limit_hit = 0U;  /* 钩爪带前进超限标志 (GripperBeltMoveUntilTof25 置位, PickBox 读取) */

/* 诊断: 初始化失败阶段 (0=未失败, 1=SN收集超时, 2=SN匹配不足, 3=电机配置失败) */
volatile uint8_t g_flybox_init_diag = 0U;
/* 诊断: SN 队列中实际收到的 SN 数量 */
volatile uint8_t g_flybox_sn_received = 0U;
/* 诊断: TOF26 联动执行记录 (调试用) */
volatile FlyBox_TofDiag_t g_tof26_diag;
/* 诊断: PickBox 各步骤执行状态 (调试用) */
volatile FlyBox_PickDiag_t g_pick_diag;
/* 诊断: 最近一次电机等待失败详情 (调试用, 仅失败时写入) */
volatile FlyBox_WaitDiag_t g_wait_diag;
/* 诊断: 力矩回零执行记录 (调试用) */
volatile FlyBox_HomeDiag_t g_home_diag;
/* 标定: 力矩标定结果 (调试用, Watch 窗口读取) */
volatile FlyBox_TorqueCalib_t g_torque_calib;

/* ======================== 内部辅助函数 ======================== */

/**
  * @brief  比较两个 SN 是否相同
  */
static uint8_t FlyBox_SNMatch(const uint8_t *a, const uint8_t *b)
{
    for (uint8_t i = 0; i < MOTOR_SN_LEN; i++)
    {
        if (a[i] != b[i]) return 0U;
    }
    return 1U;
}

/**
  * @brief  等待单个电机到位 (轮询 positionReached)
  * @param  deviceId    设备号
  * @param  timeout_ms  超时时间 (ms)
  */
static HAL_StatusTypeDef FlyBox_WaitMotorReached(uint8_t deviceId, uint32_t timeout_ms)
{
    volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
    if (p == NULL) return HAL_ERROR;

    p->lastErrorCode = 0U;
    uint32_t count0 = p->feedbackCount;  /* 记录等待开始时的反馈帧计数 (新鲜度基准) */

    uint32_t elapsed = 0U;
    const uint32_t poll_ms = 5U;
    const uint32_t start_grace_ms = 300U;  /* 等电机开始运动的宽限期 */
    uint8_t motorStarted = 0U;             /* 电机是否已经开始运动 */

    while (elapsed < timeout_ms)
    {
        /* 检测电机是否开始运动 (mainPosition 变为非零) */
        if (p->mainPosition != 0)
        {
            motorStarted = 1U;
        }

        /* 致命错误检测 (堵转/超温/欠压/断线), 忽略 0x0102(未到位) */
        uint16_t err = p->lastErrorCode;
        if (err == MOTOR_ERR_STALL || err == MOTOR_ERR_OVERTEMP ||
            err == MOTOR_ERR_UNDERVOLTAGE || err == MOTOR_ERR_LINE_BREAK)
        {
            /* 保护帧豁免 (新鲜度 + 容差 双条件):
             * 驱动器在顶限位/到位瞬间可能上报保护帧 (堵转/超温等), 且可能滞后到达。
             * 豁免条件:
             *   1. feedbackCount != count0: 等待开始后已收到新反馈帧,
             *      mainPosition 反映当前状态而非陈旧值 (防: 发非零目标后驱动器
             *      尚未更新剩余位移时收到真实堵转帧被误豁免 —— 此时无新帧, 不豁免)
             *   2. 剩余位移 ≤ 容差: 物理上已在目标位
             * 附带效果: 断线(0x0108)电机无反馈帧到达, 新鲜度不满足, 永远不会被豁免。
             * 真正的中途故障剩余位移大, 不满足容差, 仍会判失败 */
            if (p->feedbackCount != count0 &&
                p->mainPosition <= (int32_t)FLYBOX_STALL_REACHED_TOL_RAW &&
                p->mainPosition >= -(int32_t)FLYBOX_STALL_REACHED_TOL_RAW)
            {
                p->lastErrorCode = 0U;  /* 清除瞬时保护错误码 */
                return HAL_OK;
            }

            /* 【诊断】记录致命错误失败详情 */
            g_wait_diag.deviceId     = deviceId;
            g_wait_diag.result       = 1U;
            g_wait_diag.errCode      = err;
            g_wait_diag.mainPosition = p->mainPosition;
            g_wait_diag.motorStarted = motorStarted;
            g_wait_diag.elapsedMs    = elapsed;
            s_state = FLYBOX_ERROR;
            return HAL_ERROR;
        }

        /* 到位判定: mainPosition==0 且 (电机已动过 或 宽限期已过) */
        if (p->mainPosition == 0)
        {
            if (motorStarted || elapsed >= start_grace_ms)
            {
                return HAL_OK;
            }
        }

        osDelay(poll_ms);
        elapsed += poll_ms;
    }

    /* 【诊断】记录超时失败详情 */
    g_wait_diag.deviceId     = deviceId;
    g_wait_diag.result       = 3U;
    g_wait_diag.errCode      = p->lastErrorCode;
    g_wait_diag.mainPosition = p->mainPosition;
    g_wait_diag.motorStarted = motorStarted;
    g_wait_diag.elapsedMs    = elapsed;
    s_state = FLYBOX_ERROR;
    return HAL_TIMEOUT;
}

/**
  * @brief  等待多个电机同时到位 (轮询所有 positionReached)
  * @param  ids         设备号数组
  * @param  count       电机数量
  * @param  timeout_ms  超时时间 (ms)
  */
static HAL_StatusTypeDef FlyBox_WaitMultipleReached(const uint8_t *ids, uint8_t count, uint32_t timeout_ms)
{
    if (count == 0U || count > FLYBOX_MOTOR_COUNT) return HAL_ERROR;

    uint32_t count0[FLYBOX_MOTOR_COUNT] = {0};

    for (uint8_t i = 0U; i < count; i++)
    {
        volatile Motor_Status_t *p = Motor_GetStatus(ids[i]);
        if (p == NULL) { s_state = FLYBOX_ERROR; return HAL_ERROR; }
        p->lastErrorCode = 0U;
        count0[i] = p->feedbackCount;  /* 新鲜度基准 */
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

            /* 检测电机是否开始运动 */
            if (p->mainPosition != 0)
            {
                motorStarted[i] = 1U;
            }

            /* 致命错误检测 */
            uint16_t err = p->lastErrorCode;
            if (err == MOTOR_ERR_STALL || err == MOTOR_ERR_OVERTEMP ||
                err == MOTOR_ERR_UNDERVOLTAGE || err == MOTOR_ERR_LINE_BREAK)
            {
                /* 保护帧豁免 (新鲜度 + 容差 双条件):
                 * 需已收到等待开始后的新反馈帧 (mainPosition 非陈旧值),
                 * 且剩余位移 ≤ 容差 (物理上已在目标位)。
                 * 防: 发非零目标后驱动器尚未更新剩余位移时收到真实堵转帧被误豁免 */
                if (p->feedbackCount != count0[i] &&
                    p->mainPosition <= (int32_t)FLYBOX_STALL_REACHED_TOL_RAW &&
                    p->mainPosition >= -(int32_t)FLYBOX_STALL_REACHED_TOL_RAW)
                {
                    p->lastErrorCode = 0U;  /* 清除瞬时保护错误码, 按到位处理 */
                    continue;               /* 该电机视为已到位, 检查下一个 */
                }
                else
                {
                    /* 【诊断】记录致命错误失败详情 */
                    g_wait_diag.deviceId     = ids[i];
                    g_wait_diag.result       = 1U;
                    g_wait_diag.errCode      = err;
                    g_wait_diag.mainPosition = p->mainPosition;
                    g_wait_diag.motorStarted = motorStarted[i];
                    g_wait_diag.elapsedMs    = elapsed;
                    s_state = FLYBOX_ERROR;
                    return HAL_ERROR;
                }
            }

            /* 到位判定: mainPosition==0 且 (电机已动过 或 宽限期已过) */
            if (p->mainPosition == 0)
            {
                if (motorStarted[i] || elapsed >= start_grace_ms)
                {
                    /* 此电机到位 */
                }
                else
                {
                    allReached = 0U;
                }
            }
            else
            {
                allReached = 0U;
            }
        }

        if (allReached)
        {
            return HAL_OK;
        }

        osDelay(poll_ms);
        elapsed += poll_ms;
    }

    /* 【诊断】记录超时失败详情: 找出第一个未到位的电机 */
    g_wait_diag.deviceId     = ids[0];
    g_wait_diag.result       = 3U;
    g_wait_diag.elapsedMs    = elapsed;
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
    s_state = FLYBOX_ERROR;
    return HAL_TIMEOUT;
}

/* ======================== 初始化 ======================== */

/**
  * @brief  飞箱模块初始化: SN 绑定 + 5 电机配置 + 锁定
  *
  *         流程
  *           1. 清空 SN 队列, 请求所有电机上报 SN
  *           2. 等待 5 个 SN 入队
  *           3. 遍历队列, 按 SN 匹配分配设备号 1~5
  *           4. 对每个电机调用 Motor_Init (配置参数 + 轮径 + 锁定 + 默认速度)
  *           5. 传送带右方向取反 (镜像安装)
  *
  * @retval HAL_OK 成功, HAL_ERROR/HAL_TIMEOUT 失败
  */
HAL_StatusTypeDef FlyBox_Init(void)
{
    HAL_StatusTypeDef status;

    /* 5 个电机的 SN 标定值 */
    static const uint8_t snConveyorL[MOTOR_SN_LEN]     = FLYBOX_SN_CONVEYOR_L;
    static const uint8_t snConveyorR[MOTOR_SN_LEN]     = FLYBOX_SN_CONVEYOR_R;
    static const uint8_t snGripper[MOTOR_SN_LEN]       = FLYBOX_SN_GRIPPER;
    static const uint8_t snGripperRotate[MOTOR_SN_LEN] = FLYBOX_SN_GRIPPER_ROTATE;
    static const uint8_t snGripperBelt[MOTOR_SN_LEN]   = FLYBOX_SN_GRIPPER_BELT;

    g_flybox_init_diag = 0U;

    /* 记录每个设备号是否就绪 (扫描在线 或 SN绑定成功), 不依赖 online 标志 */
    uint8_t deviceReady[FLYBOX_MOTOR_COUNT] = {0};

    /* ===== Phase 1: 扫描已在线电机 (已绑定设备号, 不断电重烧时) ===== */
    uint8_t onlineCount = Motor_ScanOnline(500U);
    g_flybox_sn_received = onlineCount;
    for (uint8_t id = 1U; id <= FLYBOX_MOTOR_COUNT; id++)
    {
        if (g_motor_status[id - 1U].online)
        {
            deviceReady[id - 1U] = 1U;
        }
    }

    /* ===== Phase 2: 对未在线电机走 SN 绑定流程 ===== */
    if (onlineCount < FLYBOX_MOTOR_COUNT)
    {
        Motor_ClearSNQueue();
        Motor_RequestSN(0x07U);

        /* 只等待缺失数量的 SN (已绑定的不会再响应) */
        uint8_t needCount = FLYBOX_MOTOR_COUNT - onlineCount;
        status = Motor_WaitSNQueue(needCount, FLYBOX_SN_TIMEOUT_MS);
        g_flybox_sn_received = onlineCount + g_motor_sn_queue.count;

        if (status != HAL_OK)
        {
            g_flybox_init_diag = 1U;  /* SN 收集超时 */
            s_state = FLYBOX_ERROR;
            return status;
        }

        /* 遍历队列, 按 SN 匹配分配设备号, 绑定成功即标记就绪 */
        for (uint8_t i = 0; i < g_motor_sn_queue.count; i++)
        {
            const uint8_t *sn = (const uint8_t *)g_motor_sn_queue.entries[i].sn;

            if (FlyBox_SNMatch(sn, snConveyorL))
            {
                Motor_SetDeviceId(sn, FLYBOX_ID_CONVEYOR_L);
                deviceReady[FLYBOX_ID_CONVEYOR_L - 1U] = 1U;
                osDelay(100);
            }
            else if (FlyBox_SNMatch(sn, snConveyorR))
            {
                Motor_SetDeviceId(sn, FLYBOX_ID_CONVEYOR_R);
                deviceReady[FLYBOX_ID_CONVEYOR_R - 1U] = 1U;
                osDelay(100);
            }
            else if (FlyBox_SNMatch(sn, snGripper))
            {
                Motor_SetDeviceId(sn, FLYBOX_ID_GRIPPER);
                deviceReady[FLYBOX_ID_GRIPPER - 1U] = 1U;
                osDelay(100);
            }
            else if (FlyBox_SNMatch(sn, snGripperRotate))
            {
                Motor_SetDeviceId(sn, FLYBOX_ID_GRIPPER_ROTATE);
                deviceReady[FLYBOX_ID_GRIPPER_ROTATE - 1U] = 1U;
                osDelay(100);
            }
            else if (FlyBox_SNMatch(sn, snGripperBelt))
            {
                Motor_SetDeviceId(sn, FLYBOX_ID_GRIPPER_BELT);
                deviceReady[FLYBOX_ID_GRIPPER_BELT - 1U] = 1U;
                osDelay(100);
            }
        }
    }

    /* ===== 校验: 5 个电机必须全部就绪才能继续 ===== */
    {
        uint8_t readyCount = 0U;
        for (uint8_t id = 0U; id < FLYBOX_MOTOR_COUNT; id++)
        {
            if (deviceReady[id])
            {
                readyCount++;
            }
        }
        if (readyCount < FLYBOX_MOTOR_COUNT)
        {
            g_flybox_init_diag = 2U;  /* 电机未全部就绪 */
            s_state = FLYBOX_ERROR;
            return HAL_ERROR;
        }
    }

    /* ===== 2. 配置每个电机 (参数 + 轮径 + 锁定 + 默认速度/加速度) ===== */
    /* 各电机使用独立轮径宏, 便于分别标定 */
    status = Motor_Init(FLYBOX_ID_CONVEYOR_L, FLYBOX_DIA_CONVEYOR_L,
                        FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL);
    if (status != HAL_OK) { g_flybox_init_diag = 3U; s_state = FLYBOX_ERROR; return status; }
    osDelay(50);

    /* 传送带右: 直接以 CCW 初始化 (镜像安装方向), 避免先用 CW 配置再被覆盖,
     * 消除不断电烧录时中间态配置导致驱动器内部状态不一致的隐患 */
    status = Motor_InitCustom(FLYBOX_ID_CONVEYOR_R, FLYBOX_DIA_CONVEYOR_R,
                        FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL, MOTOR_DIR_CCW,
                        MOTOR_CFG_BYTE0, MOTOR_CFG_BYTE1, MOTOR_CFG_BYTE2,
                        MOTOR_CFG_BYTE3, MOTOR_CFG_BYTE4, MOTOR_CFG_BYTE5);
    if (status != HAL_OK) { g_flybox_init_diag = 3U; s_state = FLYBOX_ERROR; return status; }
    osDelay(50);

    /* 钩爪电机: 默认 PID + 阻尼抑制异响 */
    status = Motor_InitCustom(FLYBOX_ID_GRIPPER, FLYBOX_DIA_GRIPPER,
                        FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL, MOTOR_DIR_CW,
                        FLYBOX_GRIPPER_CFG_BYTE0, FLYBOX_GRIPPER_CFG_BYTE1,
                        FLYBOX_GRIPPER_CFG_BYTE2, FLYBOX_GRIPPER_CFG_BYTE3,
                        FLYBOX_GRIPPER_CFG_BYTE4, FLYBOX_GRIPPER_CFG_BYTE5);
    if (status != HAL_OK) { g_flybox_init_diag = 3U; s_state = FLYBOX_ERROR; return status; }
    osDelay(50);

    status = Motor_Init(FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_DIA_GRIPPER_ROTATE,
                        FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL);
    if (status != HAL_OK) { g_flybox_init_diag = 3U; s_state = FLYBOX_ERROR; return status; }
    osDelay(50);

    /* 钩爪皮带: 直接以 CCW 初始化 (镜像安装方向), 避免先用 CW 配置再被覆盖,
     * 消除不断电烧录时中间态配置导致驱动器内部状态不一致的隐患 */
    status = Motor_InitCustom(FLYBOX_ID_GRIPPER_BELT, FLYBOX_DIA_GRIPPER_BELT,
                        FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL, MOTOR_DIR_CCW,
                        MOTOR_CFG_BYTE0, MOTOR_CFG_BYTE1, MOTOR_CFG_BYTE2,
                        MOTOR_CFG_BYTE3, MOTOR_CFG_BYTE4, MOTOR_CFG_BYTE5);
    if (status != HAL_OK) { g_flybox_init_diag = 3U; s_state = FLYBOX_ERROR; return status; }
    osDelay(50);

    s_initialized = 1U;

    /* ===== 4. 归零: 钩爪(3) → 旋转(4) → 传送带(5) ===== */
    if (!FlyBox_HomeThresholdsReady())
    {
        /* 标定模式: 力矩阈值未配置, 跳过回零 (避免未标定误动作), 等待标定 */
        s_state = FLYBOX_IDLE;
        return HAL_OK;
    }

    status = FlyBox_HomeAll();
    if (status != HAL_OK)
    {
        g_flybox_init_diag = 4U;  /* 归零失败 */
        return status;
    }

    s_state = FLYBOX_IDLE;
    return HAL_OK;
}

/* ======================== 归零与安全归位 ======================== */

/**
  * @brief  检查回零力矩阈值是否已全部配置
  * @retval 1=全部已配置 (可正常回零), 0=标定模式 (存在阈值=0)
  */
uint8_t FlyBox_HomeThresholdsReady(void)
{
    return (FLYBOX_HOME_TORQUE_THRESH_GRIPPER > 0 &&
            FLYBOX_HOME_TORQUE_THRESH_GRIPPER_ROTATE > 0 &&
            FLYBOX_HOME_TORQUE_THRESH_GRIPPER_BELT > 0) ? 1U : 0U;
}

/**
  * @brief  获取电机回零配置 (方向 / 搜索行程 / 力矩阈值)
  * @retval 1=配置有效, 0=该电机不支持力矩回零
  */
static uint8_t FlyBox_GetHomeConfig(uint8_t deviceId, int8_t *dir, float *search_mm, int16_t *thresh)
{
    switch (deviceId)
    {
        case FLYBOX_ID_GRIPPER:
            *dir = FLYBOX_HOME_DIR_GRIPPER;
            *search_mm = FLYBOX_HOME_SEARCH_MM_GRIPPER;
            *thresh = FLYBOX_HOME_TORQUE_THRESH_GRIPPER;
            return 1U;
        case FLYBOX_ID_GRIPPER_ROTATE:
            *dir = FLYBOX_HOME_DIR_GRIPPER_ROTATE;
            *search_mm = FLYBOX_HOME_SEARCH_MM_GRIPPER_ROTATE;
            *thresh = FLYBOX_HOME_TORQUE_THRESH_GRIPPER_ROTATE;
            return 1U;
        case FLYBOX_ID_GRIPPER_BELT:
            *dir = FLYBOX_HOME_DIR_GRIPPER_BELT;
            *search_mm = FLYBOX_HOME_SEARCH_MM_GRIPPER_BELT;
            *thresh = FLYBOX_HOME_TORQUE_THRESH_GRIPPER_BELT;
            return 1U;
        default:
            return 0U;
    }
}

/**
  * @brief  脱困探索: 急停 + Unlock/Lock + 重发速度 + 相对移动
  *         不复用 WaitMotorReached 的完整保护帧豁免 (卡滞时位置不动但
  *         mainPosition≈0, 豁免可能误判成功), 改为独立等待 + 力矩验证。
  * @param  deviceId    设备号
  * @param  direction   +1=正方向, -1=负方向
  * @param  distance_mm 移动距离 (mm)
  * @param  timeout_ms  等待超时 (ms)
  * @retval HAL_OK 移动完成 (但需调用方验证力矩是否回落)
  */
static HAL_StatusTypeDef FlyBox_FreeTryMove(uint8_t deviceId, int8_t direction, float distance_mm, uint32_t timeout_ms)
{
    volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
    if (p == NULL) return HAL_ERROR;

    /* 急停取消当前运动 */
    Motor_EmergencyStop(deviceId);
    osDelay(50);

    /* Unlock/Lock 清驱动器保护态 */
    Motor_Unlock(deviceId);
    osDelay(100);
    Motor_Lock(deviceId);
    osDelay(100);

    /* 重发速度参数 (驱动器冻结后可能丢失) */
    Motor_SetSpeed(deviceId, FLYBOX_HOME_SEARCH_SPEED);
    Motor_SetAcceleration(deviceId, FLYBOX_HOME_SEARCH_ACCEL);
    osDelay(10);

    p->lastErrorCode = 0U;
    float target = (direction > 0) ? distance_mm : -distance_mm;
    if (Motor_SetTarget(deviceId, target, MOTOR_TARGET_RELATIVE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* 独立等待: 轮询 positionReached, 无保护帧豁免
     * 卡滞时电机不动, 超时返回 HAL_TIMEOUT */
    uint32_t elapsed = 0U;
    const uint32_t poll_ms = 10U;
    uint8_t motorStarted = 0U;

    while (elapsed < timeout_ms)
    {
        if (p->mainPosition != 0)
        {
            motorStarted = 1U;
        }

        /* 致命错误: 堵转/超温/欠压/断线 → 失败 (不豁免, 卡滞时堵转是真实故障) */
        uint16_t err = p->lastErrorCode;
        if (err == MOTOR_ERR_STALL || err == MOTOR_ERR_OVERTEMP ||
            err == MOTOR_ERR_UNDERVOLTAGE || err == MOTOR_ERR_LINE_BREAK)
        {
            return HAL_ERROR;
        }

        /* 到位判定: 电机已动过 且 mainPosition≈0 */
        if (motorStarted && p->mainPosition == 0)
        {
            return HAL_OK;
        }

        osDelay(poll_ms);
        elapsed += poll_ms;
    }

    return HAL_TIMEOUT;  /* 超时未到位 */
}

/**
  * @brief  多级卡滞脱困: 从限位/卡死状态挣脱
  *         级1: 远离回零限位方向 (-dir) 30mm
  *         级2: 回零限位方向 (+dir) 30mm
  *         级3: 摇摆 ±20/60/100mm 递增
  *         级4: 电流限制 0xFF + 摇摆后恢复配置
  * @param  deviceId  设备号
  * @param  dir       回零搜索方向 (+1/-1)
  * @param  thresh    力矩阈值
  * @retval HAL_OK 脱困成功 (力矩已回落), HAL_ERROR 全部失败
  */
static HAL_StatusTypeDef FlyBox_FreeFromLimit(uint8_t deviceId, int8_t dir, int16_t thresh)
{
    volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
    if (p == NULL) return HAL_ERROR;

    /* 恢复方向: 皮带=CCW 其余=CW (与 init 保持一致) */
    int8_t restoreDir = (deviceId == FLYBOX_ID_GRIPPER_BELT) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;

    const float rockDist[3] = {
        FLYBOX_HOME_FREE_ROCK1_MM,
        FLYBOX_HOME_FREE_ROCK2_MM,
        FLYBOX_HOME_FREE_ROCK3_MM
    };

    /* ---- 级1: 回零限位方向 (+dir, 同搜索方向) ---- */
    if (FlyBox_FreeTryMove(deviceId, +dir, FLYBOX_HOME_FREE_MM, FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
    {
        osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
        int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
        if (absTq < thresh)
        {
            g_home_diag.freeStage = 11U;  /* FreeFromLimit 级1成功 */
            return HAL_OK;
        }
    }

    /* ---- 级2: 远离回零限位方向 (-dir) ---- */
    if (FlyBox_FreeTryMove(deviceId, -dir, FLYBOX_HOME_FREE_MM, FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
    {
        osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
        int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
        if (absTq < thresh)
        {
            g_home_diag.freeStage = 12U;  /* FreeFromLimit 级2成功 */
            return HAL_OK;
        }
    }

    /* ---- 级3: 摇摆 ± 递增 ---- */
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (FlyBox_FreeTryMove(deviceId, -dir, rockDist[i], FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
        {
            osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
            int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
            if (absTq < thresh)
            {
                g_home_diag.freeStage = 13U;  /* FreeFromLimit 级3成功 */
                return HAL_OK;
            }
        }
        if (FlyBox_FreeTryMove(deviceId, +dir, rockDist[i], FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
        {
            osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
            int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
            if (absTq < thresh)
            {
                g_home_diag.freeStage = 13U;  /* FreeFromLimit 级3成功 */
                return HAL_OK;
            }
        }
    }

    /* ---- 级4: 电流限制 0xFF + 摇摆, 恢复配置 ---- */
    /* 临时放开电流限制, 允许更大转矩挣脱 */
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
                {
                    Motor_ConfigParamCustom(deviceId, deviceId, restoreDir,
                        FLYBOX_GRIPPER_CFG_BYTE0, FLYBOX_GRIPPER_CFG_BYTE1,
                        FLYBOX_GRIPPER_CFG_BYTE2, FLYBOX_GRIPPER_CFG_BYTE3,
                        FLYBOX_GRIPPER_CFG_BYTE4, FLYBOX_GRIPPER_CFG_BYTE5);
                }
                else
                {
                    Motor_ConfigParam(deviceId, deviceId, restoreDir);
                }
                g_home_diag.freeStage = 14U;  /* FreeFromLimit 级4成功 */
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
                {
                    Motor_ConfigParamCustom(deviceId, deviceId, restoreDir,
                        FLYBOX_GRIPPER_CFG_BYTE0, FLYBOX_GRIPPER_CFG_BYTE1,
                        FLYBOX_GRIPPER_CFG_BYTE2, FLYBOX_GRIPPER_CFG_BYTE3,
                        FLYBOX_GRIPPER_CFG_BYTE4, FLYBOX_GRIPPER_CFG_BYTE5);
                }
                else
                {
                    Motor_ConfigParam(deviceId, deviceId, restoreDir);
                }
                g_home_diag.freeStage = 14U;  /* FreeFromLimit 级4成功 */
                return HAL_OK;
            }
        }
    }

    /* 全部失败, 恢复原配置 */
    if (deviceId == FLYBOX_ID_GRIPPER)
    {
        Motor_ConfigParamCustom(deviceId, deviceId, restoreDir,
            FLYBOX_GRIPPER_CFG_BYTE0, FLYBOX_GRIPPER_CFG_BYTE1,
            FLYBOX_GRIPPER_CFG_BYTE2, FLYBOX_GRIPPER_CFG_BYTE3,
            FLYBOX_GRIPPER_CFG_BYTE4, FLYBOX_GRIPPER_CFG_BYTE5);
    }
    else
    {
        Motor_ConfigParam(deviceId, deviceId, restoreDir);
    }

    return HAL_ERROR;
}

/**
  * @brief  单电机力矩回零: 撞机械限位定位零点
  *
  *         流程:
  *           1. 解除可能的保护状态 (Unlock → Lock), 清错误码与力矩统计
  *           2. 低速朝正确方向 (dir) 搜索, 行程 > 全行程, 保证走完全程
  *           3. 10ms 轮询: |力矩| 连续超阈 → 候选"到限位" (不立即置零!)
  *              (错误侧是另一端的机械限位, 若钩爪硬卡死在错误侧, 向正确方向
  *               同样会"力矩变大且无位移" → 仅凭力矩无法区分, 需探索确认)
  *              驱动器堵转 0x0104 作备份判据 (顶住限位属预期, 不当致命错误);
  *              其他致命错误 (超温/欠压/断线) → 失败
  *           4. 候选到限位后按电机是否真正移动过分叉:
  *              移动过 (|mainPosition| 从初始目标值减小) = 真实撞限位 → 反方向回退 2mm 探索确认
  *              从未移动 (|mainPosition| 恒为初始目标值) = 卡滞(错误侧/自锁) →
  *                回退(-dir)探索 → 正向(+dir)探索 → 多级脱困(FreeFromLimit) → 重试搜索
  *           5. 探索确认后置零 (发 0x0A 把当前位置设为零点)
  *             回退后力矩仍 ≥ 阈值 (无论同向/反向) = 未离开限位 → failReason=7, 拒绝置零
  *             只有力矩回落 < 阈值才确认已离开限位, 安全置零
  *           6. 卡滞脱困全失败 → failReason=9, Unlock 释放电机便于人工处理
  *
  *         边界: 开机已在正确零位时电机不动但力矩立即超阈 → 回退探索成功即置零
  */
HAL_StatusTypeDef FlyBox_HomeMotorByTorque(uint8_t deviceId)
{
    volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
    if (p == NULL) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    int8_t  dir;
    float   search_mm;
    int16_t thresh;
    if (!FlyBox_GetHomeConfig(deviceId, &dir, &search_mm, &thresh))
    {
        s_state = FLYBOX_ERROR;
        return HAL_ERROR;  /* 该电机不支持力矩回零 */
    }

    /* 【诊断】初始化回零记录 */
    g_home_diag.deviceId       = deviceId;
    g_home_diag.phase          = 1U;  /* 搜索 */
    g_home_diag.failReason     = 0U;
    g_home_diag.zeroAtLimit    = 0U;
    g_home_diag.errCode        = 0U;
    g_home_diag.torqueAtLimit  = 0;
    g_home_diag.torquePeak     = 0;
    g_home_diag.searchMs       = 0U;
    g_home_diag.backoffElapsed = 0U;
    g_home_diag.backoffPosition = 0;
    g_home_diag.backoffTorque   = 0;
    g_home_diag.freeStage       = 0U;
    g_home_diag.retryCount      = 0U;

    /* 阈值未配置 (标定模式, 阈值=0): 不执行力矩回零, 防止未标定误动作 */
    if (thresh <= 0)
    {
        g_home_diag.phase = 6U;
        g_home_diag.failReason = 6U;
        s_state = FLYBOX_ERROR;
        return HAL_ERROR;
    }

    /* 1. 解除可能的急停保护状态, 清错误码与力矩统计 */
    Motor_Unlock(deviceId);
    osDelay(50);
    Motor_Lock(deviceId);
    osDelay(50);
    p->lastErrorCode = 0U;
    Motor_ResetTorqueStats(deviceId);

    /* 卡滞脱困重试入口: 重新发送搜索目标 */
    uint8_t retryCount = 0U;
retry_search:

    /* 2. 低速相对运动朝正确方向搜索 (搜索专用低速, 避免猛撞进驱动器保护) */
    Motor_SetSpeed(deviceId, FLYBOX_HOME_SEARCH_SPEED);
    Motor_SetAcceleration(deviceId, FLYBOX_HOME_SEARCH_ACCEL);
    osDelay(5);

    const float searchTargetMm = (dir > 0) ? search_mm : -search_mm;
    if (Motor_SetTarget(deviceId, searchTargetMm, MOTOR_TARGET_RELATIVE) != HAL_OK)
    {
        g_home_diag.phase = 6U;
        g_home_diag.failReason = 2U;
        s_state = FLYBOX_ERROR;
        return HAL_ERROR;
    }

    /* 3. 轮询力矩, 等待撞正确零位
     *
     * 卡滞检测: 判断电机是否真正移动过, 关键在于 mainPosition 是否从初始
     * 目标值发生过变化 (剩余位移减小 = 电机真正动了). 用 maxAbsPos 跟踪
     * |mainPosition| 的历史最大值, 若当前值 < 历史最大值, 说明剩余位移
     * 减少了, 电机确实移动过.
     *
     * 注意: 不能用 mainPosition != 0 来判断, 因为 CAN 首帧更新 mainPosition
     * 为目标值后立即非零, 此时电机实际未动 (卡在错误侧限位), 会导致卡滞漏检. */
    const uint32_t poll_ms = 10U;
    uint32_t elapsed = 0U;
    uint32_t debounce = 0U;
    uint8_t  limitReached = 0U;
    uint8_t  stuckDetected = 0U;  /* 卡滞标志: 电机从未移动但力矩超阈 (错误侧/自锁) */
    uint8_t  motorMoved = 0U;     /* 电机是否真正移动过: |mainPosition| 从历史最大值减小 */
    int32_t  maxAbsPos = 0;       /* |mainPosition| 历史最大值, 用于检测移动 */
    uint8_t  stallLimit = 0U;     /* 1=限位由堵转错误(0x0104)检出, 非力矩阈值检出 (需提前清驱动保护态再回退) */

    while (elapsed < FLYBOX_HOME_TIMEOUT_MS)
    {
        int16_t tq = p->torque;
        int16_t absTq = (tq >= 0) ? tq : (int16_t)(-tq);
        int32_t absPos = (p->mainPosition >= 0) ? p->mainPosition : (int32_t)(-p->mainPosition);

        /* 检测电机是否真正移动: |mainPosition| 从历史最大值减小 = 剩余位移减少 = 电机动了 */
        if (absPos > maxAbsPos)
        {
            maxAbsPos = absPos;  /* 更新历史最大值 (首次收到 CAN 帧时从 0→目标值) */
        }
        if (!motorMoved && absPos < maxAbsPos)
        {
            motorMoved = 1U;     /* 剩余位移减少 = 电机真正移动了 */
        }

        /* 力矩超阈消抖: 连续 N 次轮询超阈 → 候选"到限位" (回退探索确认后才置零)
         * 卡滞检测: 若电机从未真正移动过 (mainPosition 恒为初始目标值), 说明
         * 力矩超阈不是撞限位, 而是卡在错误侧/机械自锁 → 进入脱困流程 */
        if (absTq >= thresh)
        {
            debounce++;
            if (debounce >= FLYBOX_HOME_TORQUE_DEBOUNCE)
            {
                g_home_diag.torqueAtLimit = tq;
                if (!motorMoved)
                {
                    /* 电机从未真正移动过但力矩超阈 → 卡滞(错误侧/自锁)
                     * 开机已在正确限位时同样触发此分支, 脱困流程会处理
                     * (回退成功 → 重试搜索 → 正常置零) */
                    stuckDetected = 1U;
                }
                else
                {
                    limitReached = 1U;
                }
                break;
            }
        }
        else
        {
            debounce = 0U;
        }

        /* 驱动器堵转错误作备份判据 (回零期间顶限位属预期, 不当致命错误) */
        uint16_t err = p->lastErrorCode;
        if (err == MOTOR_ERR_STALL)
        {
            g_home_diag.torqueAtLimit = tq;
            stallLimit = 1U;
            limitReached = 1U;
            break;
        }

        /* 其他致命错误: 超温/欠压/断线 → 回零失败 */
        if (err == MOTOR_ERR_OVERTEMP || err == MOTOR_ERR_UNDERVOLTAGE ||
            err == MOTOR_ERR_LINE_BREAK)
        {
            Motor_EmergencyStop(deviceId);
            g_home_diag.phase = 6U;
            g_home_diag.failReason = 2U;
            g_home_diag.errCode = err;
            s_state = FLYBOX_ERROR;
            return HAL_ERROR;
        }

        /* 搜索行程走完仍未撞限位 (搜索距离配置过小): 电机已真正移动过且剩余位移归零 */
        if (motorMoved && p->mainPosition == 0)
        {
            Motor_EmergencyStop(deviceId);
            g_home_diag.searchMs = elapsed;
            g_home_diag.torquePeak = p->torquePeak;
            g_home_diag.phase = 6U;
            g_home_diag.failReason = 5U;
            s_state = FLYBOX_ERROR;
            return HAL_ERROR;
        }

        osDelay(poll_ms);
        elapsed += poll_ms;
    }

    g_home_diag.searchMs   = elapsed;
    g_home_diag.torquePeak = p->torquePeak;

    if (!limitReached)
    {
        /* 超时未撞限位 → 回零失败 */
        Motor_EmergencyStop(deviceId);
        g_home_diag.phase = 6U;
        g_home_diag.failReason = 1U;
        s_state = FLYBOX_ERROR;
        return HAL_TIMEOUT;
    }

    /* ===== 卡滞脱困分支: 电机从未移动 + 力矩超阈 = 卡在错误侧/自锁 ===== */
    if (stuckDetected)
    {
        g_home_diag.phase = 2U;  /* 进入脱困 */

        /* 重试次数超限: 多次脱困-搜索循环仍卡滞 → 真机械卡死, 不再尝试 */
        if (retryCount >= FLYBOX_HOME_RETRY_MAX)
        {
            Motor_EmergencyStop(deviceId);
            osDelay(100);
            Motor_Unlock(deviceId);
            g_home_diag.freeStage = 9U;
            g_home_diag.phase = 6U;
            g_home_diag.failReason = 9U;
            s_state = FLYBOX_ERROR;
            return HAL_ERROR;
        }

        /* 尝试1: 正向探索 (+dir, 同搜索方向)
         * 卡在错误侧限位时: +dir = 朝正确限位 = 远离错误侧 = 自由方向
         * 这是最常见的卡滞场景 (钩爪卡在闭合端, 向张开端探索即可脱困) */
        Motor_EmergencyStop(deviceId);
        osDelay(50);
        if (FlyBox_FreeTryMove(deviceId, +dir, FLYBOX_HOME_FREE_MM, FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
        {
            osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
            int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
            if (absTq < thresh)
            {
                /* 正向探索成功, 力矩回落 → 重试搜索 */
                g_home_diag.freeStage = 1U;
                g_home_diag.retryCount = ++retryCount;
                goto retry_search;
            }
        }

        /* 尝试2: 反方向回退 (-dir, 远离搜索方向)
         * 正向探索失败 (卡在正确侧限位, 推进限位里顶), 反方向远离正确侧 → 脱困
         * 注意: -dir 在错误侧卡死时会往错误侧里顶, 可能加剧卡死/触发保护;
         * 但正确侧卡死时此方向是唯一出路, 必须尝试 */
        if (FlyBox_FreeTryMove(deviceId, -dir, FLYBOX_HOME_FREE_MM, FLYBOX_HOME_FREE_TIMEOUT_MS) == HAL_OK)
        {
            osDelay(FLYBOX_HOME_FREE_SETTLE_MS);
            int16_t absTq = (p->torque >= 0) ? p->torque : (int16_t)(-p->torque);
            if (absTq < thresh)
            {
                g_home_diag.freeStage = 2U;
                g_home_diag.retryCount = ++retryCount;
                goto retry_search;
            }
        }

        /* 尝试3: 多级脱困 (FreeFromLimit)
         * 双向简单探索均失败 → 升级到摇摆/电流恢复 */
        if (FlyBox_FreeFromLimit(deviceId, dir, thresh) == HAL_OK)
        {
            g_home_diag.retryCount = ++retryCount;
            goto retry_search;
        }

        /* 全失败: 机械卡死, 无法脱困 → failReason=9, 不置零
         * Unlock 释放电机便于人工掰开处理 */
        Motor_EmergencyStop(deviceId);
        osDelay(100);
        Motor_Unlock(deviceId);
        g_home_diag.freeStage = 9U;
        g_home_diag.phase = 6U;
        g_home_diag.failReason = 9U;
        s_state = FLYBOX_ERROR;
        return HAL_ERROR;
    }

    /* 4. 候选到限位: 电机已移动过, 真实撞限位 → 直接发反向回退目标 (不急停!) → 探索确认
     *    实测: 电机3/4 直接回退成功; 电机5 (硬限位) 驱动器冻结, 需恢复序列
     *
     *    若限位由堵转错误(0x0104)检出, 电机驱动器已进入保护态, 直接发回退
     *    目标会被忽略 → 先 Unlock/Lock 清保护态, 再发回退目标. */
    g_home_diag.phase = 2U;
    if (stallLimit)
    {
        /* 堵转后驱动器深度保护: 先 Unlock 释放, 再 Lock 重新锁定, 清保护态 */
        Motor_EmergencyStop(deviceId);
        osDelay(100);
        Motor_Unlock(deviceId);
        osDelay(500);
        Motor_Lock(deviceId);
        osDelay(500);
        Motor_SetSpeed(deviceId, FLYBOX_HOME_SEARCH_SPEED);
        Motor_SetAcceleration(deviceId, FLYBOX_HOME_SEARCH_ACCEL);
        osDelay(20);
        p->lastErrorCode = 0U;
    }

    float backoff = (dir > 0) ? -FLYBOX_HOME_BACKOFF_MM : FLYBOX_HOME_BACKOFF_MM;
    p->lastErrorCode = 0U;
    Motor_SetTarget(deviceId, backoff, MOTOR_TARGET_RELATIVE);

    g_home_diag.phase = 3U;
    uint32_t backoffStart = HAL_GetTick();
    HAL_StatusTypeDef result = FlyBox_WaitMotorReached(deviceId, 5000U);

    if (result != HAL_OK)
    {
        /* 尝试2: 急停 + 长时间 Unlock/Lock 释放机械应力 + 重发速度参数
         * (驱动器冻结后可能丢失运动参数, 且需要更长时间退出保护态) */
        Motor_EmergencyStop(deviceId);
        osDelay(100);
        Motor_Unlock(deviceId);
        osDelay(300);
        Motor_Lock(deviceId);
        osDelay(300);
        Motor_SetSpeed(deviceId, FLYBOX_HOME_SEARCH_SPEED);
        Motor_SetAcceleration(deviceId, FLYBOX_HOME_SEARCH_ACCEL);
        osDelay(20);
        p->lastErrorCode = 0U;
        Motor_SetTarget(deviceId, backoff, MOTOR_TARGET_RELATIVE);
        result = FlyBox_WaitMotorReached(deviceId, 10000U);
    }

    if (result != HAL_OK)
    {
        /* 尝试3 (禁止置零): 回退彻底失败, 电机停在未知位置, 此时置零会把
         * 任意位置锚定为零点 → 之后所有绝对定位整体偏移 (零位漂移)。
         * 只记录诊断并报错, 由上层决定何时重新回零 (确保无箱体卡阻) */
        g_home_diag.backoffPosition = p->mainPosition;
        Motor_EmergencyStop(deviceId);
        osDelay(100);
        Motor_Unlock(deviceId);
        osDelay(200);
        Motor_Lock(deviceId);
        osDelay(200);
        g_home_diag.backoffElapsed = HAL_GetTick() - backoffStart;
        g_home_diag.phase = 6U;
        g_home_diag.failReason = 3U;  /* 回退失败: 不回退完不置零 */
        s_state = FLYBOX_ERROR;
        return HAL_ERROR;
    }

    g_home_diag.backoffElapsed = HAL_GetTick() - backoffStart;

    /* 4.5 探索确认: 回退必须真正离开限位 (阈值判据建立在"自由状态+探索过程"上)。
     * 回退后力矩仍与撞限位同向且 ≥ 阈值 → 电机没动, 还顶在限位上 = 无法探索
     * (疑似卡死在错误侧/方向配错) → 失败, 拒绝置零, 防止把错误位置锚定为零点。
     * 方向翻转但幅值仍高 → 同样拒绝置零: 方向翻转只说明电机换了推的方向,
     * 不一定实际移动了 (错误侧卡死时, 搜索正推受阻, 回退反推同样受阻,
     * 力矩方向翻转但电机仍在限位处)。必须力矩回落 < 阈值才确认已离开限位。 */
    osDelay(100);  /* 等力矩反馈稳定 */
    {
        int16_t tqNow = p->torque;
        int16_t absTqNow = (tqNow >= 0) ? tqNow : (int16_t)(-tqNow);
        g_home_diag.backoffTorque = tqNow;

        if (absTqNow >= thresh)
        {
            /* 回退后力矩仍 ≥ 阈值: 电机未离开限位 (无论方向是否翻转)
             * 方向同向 → 未移动, 还在顶限位 (failReason=7)
             * 方向翻转 → 换了推的方向但仍在限位处 (错误侧卡死场景)
             * 两种情况均拒绝置零, 防止零位漂移 */
            Motor_EmergencyStop(deviceId);
            osDelay(100);
            Motor_Unlock(deviceId);
            osDelay(200);
            Motor_Lock(deviceId);
            osDelay(200);
            g_home_diag.backoffPosition = p->mainPosition;
            g_home_diag.phase = 6U;
            g_home_diag.failReason = 7U;  /* 回退后力矩仍超阈: 未离开限位, 无法探索 */
            s_state = FLYBOX_ERROR;
            return HAL_ERROR;
        }
    }

    /* 5. 当前位置置零 */
    g_home_diag.phase = 4U;
    if (Motor_SetZero(deviceId) != HAL_OK)
    {
        g_home_diag.phase = 6U;
        g_home_diag.failReason = 4U;
        s_state = FLYBOX_ERROR;
        return HAL_ERROR;
    }
    osDelay(50);  /* 等驱动器完成置零 */

    g_home_diag.phase = 5U;  /* 完成 */
    return HAL_OK;
}

/**
  * @brief  全部归零: 依次对钩爪(3)/旋转(4)/传送带(5)执行力矩回零,
  *         完成后回到初始状态 (原零位姿态)
  *         顺序: 先松爪(防碰撞) → 再旋转回正面 → 最后缩回皮带
  *         归零用低速 (FLYBOX_HOME_SEARCH_SPEED), 避免碰撞
  */
HAL_StatusTypeDef FlyBox_HomeAll(void)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;
    s_homed = 0U;

    /* 归零顺序: 钩爪(3) → 旋转(4) → 传送带(5) */
    static const uint8_t homeOrder[3] = {
        FLYBOX_ID_GRIPPER, FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_ID_GRIPPER_BELT
    };

    for (uint8_t i = 0U; i < 3U; i++)
    {
        HAL_StatusTypeDef result = FlyBox_HomeMotorByTorque(homeOrder[i]);
        if (result != HAL_OK)
        {
            /* 归零失败: 急停所有电机 */
            for (uint8_t j = 1U; j <= FLYBOX_MOTOR_COUNT; j++)
            {
                Motor_EmergencyStop(j);
            }
            s_state = FLYBOX_ERROR;
            return result;
        }

        osDelay(50);  /* 归零间隔, 等机械稳定 */
    }

    s_homed = 1U;

    /* 回到初始状态: 从机械限位回到工作初始姿态 (原零位),
     * 避免后续动作直接从限位位置开始 */
    HAL_StatusTypeDef result = FlyBox_GoInitState();
    if (result != HAL_OK)
    {
        for (uint8_t j = 1U; j <= FLYBOX_MOTOR_COUNT; j++)
        {
            Motor_EmergencyStop(j);
        }
        s_state = FLYBOX_ERROR;
        return result;
    }

    s_state = FLYBOX_COMPLETE;
    return HAL_OK;
}

/**
  * @brief  回到初始状态 (原零位姿态): 需已完成归零 (s_homed)
  *         钩爪: 机械限位(全张开) → 松开位 (旧零位, 新坐标 -45°)
  *         旋转: 机械限位 → 正面 (旧零位, 新坐标 +45°)
  *         皮带: 机械限位即旧零位, 回位确认
  */
HAL_StatusTypeDef FlyBox_GoInitState(void)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    if (!s_homed) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    HAL_StatusTypeDef result;

    /* 1. 钩爪: 限位 → 松开位 (内部已含偏移换算) */
    result = FlyBox_GripperRelease();
    if (result != HAL_OK) { s_state = FLYBOX_ERROR; return result; }
    osDelay(100);

    /* 1.5 回零结果验证 (防"回零撞错限位→零点锚错→后续绝对定位撞限位"):
     *     松开位必须自由 (力矩低于阈值), 若钩爪仍顶在限位/被卡住,
     *     说明零点锚在错误位置, 立即失败, 避免进入 PickBox 后
     *     钩爪 45° 反向移动顶死另一侧限位。仅在无箱体的 init 流程有效。 */
    {
        volatile Motor_Status_t *p = Motor_GetStatus(FLYBOX_ID_GRIPPER);
        if (p != NULL)
        {
            int16_t tq = p->torque;
            int16_t absTq = (tq >= 0) ? tq : (int16_t)(-tq);
            if (absTq >= FLYBOX_HOME_TORQUE_THRESH_GRIPPER)
            {
                s_state = FLYBOX_ERROR;
                return HAL_ERROR;
            }
        }
    }

    /* 1.6 钩爪行程验证: 松开位 → 钩住位应全程自由 (无箱体时),
     *     若中途撞限位/力矩超高 (零点偏移约 45° 时钩爪只走 45° 便顶死), 提前失败 */
    result = FlyBox_GripperHook();
    if (result != HAL_OK) { s_state = FLYBOX_ERROR; return result; }
    osDelay(100);
    {
        volatile Motor_Status_t *p = Motor_GetStatus(FLYBOX_ID_GRIPPER);
        if (p != NULL)
        {
            int16_t tq = p->torque;
            int16_t absTq = (tq >= 0) ? tq : (int16_t)(-tq);
            if (absTq >= FLYBOX_HOME_TORQUE_THRESH_GRIPPER)
            {
                s_state = FLYBOX_ERROR;
                return HAL_ERROR;
            }
        }
    }

    /* 1.7 验证完成, 回到松开位作为初始姿态 */
    result = FlyBox_GripperRelease();
    if (result != HAL_OK) { s_state = FLYBOX_ERROR; return result; }
    osDelay(100);

    /* 2. 旋转: 限位 → 正面 (旧零位 0°) */
    result = FlyBox_GripperRotateAbs(0.0f, FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL);
    if (result != HAL_OK) { s_state = FLYBOX_ERROR; return result; }
    osDelay(50);

    /* 3. 皮带: 回位确认 (限位即旧零位, 偏移=0, 通常无需移动) */
    result = FlyBox_GripperBelttoHome();
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  安全归位: 软停所有电机 (设定速为 0, 正常减速停止)
  *         用于 PickBox/PlaceBox 中途失败时的安全处理
  *
  *         统一用软停 (Motor_SetSpeed 0) 替代急停 (0x0B):
  *         急停和堵转保护(0x0104)都会让驱动器进入保护态, 破坏内部绝对定位状态,
  *         导致后续绝对定位指令不可靠 (产生 45° 偏移)。
  *         软停让电机正常减速停止, 驱动器状态保持完好, 所有电机均可安全使用。
  */
void FlyBox_AbortAndSafe(void)
{
    /* 【诊断】记录急停前的钩爪状态 */
    {
        volatile Motor_Status_t *p = Motor_GetStatus(FLYBOX_ID_GRIPPER);
        if (p != NULL)
        {
            g_pick_diag.gripperPosAtAbort   = p->mainPosition;
            g_pick_diag.gripperTorqueAtAbort = p->torque;
        }
    }

    /* 软停所有 5 个电机: 设定速为 0, 正常减速停止, 不破坏驱动器状态 */
    for (uint8_t id = 1U; id <= FLYBOX_MOTOR_COUNT; id++)
    {
        Motor_SetSpeed(id, 0.0f);
    }
    osDelay(100);

    /* 清零错误码 */
    for (uint8_t id = 1U; id <= FLYBOX_MOTOR_COUNT; id++)
    {
        volatile Motor_Status_t *p = Motor_GetStatus(id);
        if (p != NULL) p->lastErrorCode = 0U;
    }

    g_pick_diag.abortReleaseResult = 0U;
    s_state = FLYBOX_ERROR;
}

/* ======================== 底层控制接口 ======================== */

/**
  * @brief  传送带左右同步运行指定距离
  *         双电机同向等距, 等待两者均到位
  */
HAL_StatusTypeDef FlyBox_ConveyorRun(float distance_mm, float speed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;

    /* 设置速度和加速度 */
    Motor_SetSpeed(FLYBOX_ID_CONVEYOR_L, speed_mm_s);
    Motor_SetSpeed(FLYBOX_ID_CONVEYOR_R, speed_mm_s);
    osDelay(5);
    Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_L, accel_mm_s2);
    Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_R, accel_mm_s2);
    osDelay(5);

    /* 双电机相对目标位置 */
    Motor_SetTarget(FLYBOX_ID_CONVEYOR_L, distance_mm, MOTOR_TARGET_RELATIVE);
    Motor_SetTarget(FLYBOX_ID_CONVEYOR_R, distance_mm, MOTOR_TARGET_RELATIVE);

    /* 等待双电机到位 */
    HAL_StatusTypeDef rL = FlyBox_WaitMotorReached(FLYBOX_ID_CONVEYOR_L, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    HAL_StatusTypeDef rR = FlyBox_WaitMotorReached(FLYBOX_ID_CONVEYOR_R, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);

    if (rL == HAL_OK && rR == HAL_OK)
    {
        s_state = FLYBOX_COMPLETE;
        return HAL_OK;
    }
    s_state = FLYBOX_ERROR;
    return (rL != HAL_OK) ? rL : rR;
}

/**
  * @brief  传送带运行直到 tof26 (侧面) 距离到达 target_mm
  *         采用 "测距 → 算剩余 → 位置控制走剩余 → 等自然到位 → 再测距" 迭代逼近
  *         电机始终由位置控制停止, 不使用急停, 不进入保护状态
  *         direction: +1=靠近(距离减小), -1=远离(距离增大)
  *
  *         TOF 异常处理:
  *           - TOF 无数据 (传感器故障): 尝试重启, 重启失败则报错
  *           - TOF 在线但滤波器预热中: 原地等待稳定后重测 (不盲进, 防过度盲进)
  *           - TOF 在线但数据无效 (箱体超出测距范围):
  *               靠近方向 (+1): 盲进一段让目标进入视野
  *               远离方向 (-1): 不盲进, 直接报错 (目标本应在近处可见,
  *                 看不到=无箱/超程, 盲进只会把目标推得更远永远救不回)
  *           - 迭代超限: 返回 HAL_TIMEOUT
  */
HAL_StatusTypeDef FlyBox_ConveyorRunUntilTof26(float target_mm, float speed_mm_s, float accel_mm_s2, int8_t direction)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;

    /* 【诊断】重置联动诊断记录 */
    g_tof26_diag.firstDist  = TOF8_DISTANCE_INVALID_MM;
    g_tof26_diag.exitDist   = TOF8_DISTANCE_INVALID_MM;
    g_tof26_diag.target     = target_mm;
    g_tof26_diag.direction  = direction;
    g_tof26_diag.exitReason = 0U;
    g_tof26_diag.iters      = 0U;
    g_tof26_diag.warmWaits  = 0U;
    g_tof26_diag.blindSteps = 0U;

    /* 启动前检查 TOF26 是否在线 */
    if (TOF8_EnsureAlive(TOF8_DEVICE_ID_1, FLYBOX_TOF_ALIVE_TIMEOUT_MS, FLYBOX_TOF_RESTART_RETRY) != HAL_OK)
    {
        g_tof26_diag.exitReason = 4U;  /* 【诊断】入口 TOF 检查失败 */
        s_state = FLYBOX_ERROR;
        return HAL_ERROR;  /* TOF26 故障, 无法执行联动 */
    }

    /* 设置速度和加速度 */
    Motor_SetSpeed(FLYBOX_ID_CONVEYOR_L, speed_mm_s);
    Motor_SetSpeed(FLYBOX_ID_CONVEYOR_R, speed_mm_s);
    osDelay(5);
    Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_L, accel_mm_s2);
    Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_R, accel_mm_s2);
    osDelay(5);

    static const uint8_t convIds[2] = { FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R };

    for (uint32_t iter = 0U; iter < FLYBOX_TOF_MAX_ITER; iter++)
    {
        g_tof26_diag.iters = (uint8_t)(iter + 1U);  /* 【诊断】当前迭代次数 */

        /* 读取 tof26 距离 (中值滤波, 抑制噪声) */
        float dist = TOF8_GetDistanceMedian(TOF8_DEVICE_ID_1,
                                            FLYBOX_TOF_MEDIAN_SAMPLES,
                                            FLYBOX_TOF_MEDIAN_INTERVAL_MS);

        /* TOF 数据无效: 区分 "传感器故障" 和 "目标超出测距范围" */
        if (dist == TOF8_DISTANCE_INVALID_MM)
        {
            if (!TOF8_IsAlive(TOF8_DEVICE_ID_1, FLYBOX_TOF_ALIVE_TIMEOUT_MS))
            {
                /* 传感器故障 (无 CAN 帧): 尝试重启 */
                if (TOF8_EnsureAlive(TOF8_DEVICE_ID_1, FLYBOX_TOF_ALIVE_TIMEOUT_MS, FLYBOX_TOF_RESTART_RETRY) != HAL_OK)
                {
                    g_tof26_diag.exitReason = 2U;  /* 【诊断】TOF 重启失败 */
                    s_state = FLYBOX_ERROR;
                    return HAL_ERROR;  /* TOF26 重启失败 */
                }
                osDelay(FLYBOX_TOF_SETTLE_MS);
                continue;  /* 重启后重新测距 */
            }

            /* 传感器在线但数据无效: 区分 "滤波器预热" 和 "目标超出测距范围" */
            if (TOF8_IsWarmingUp(TOF8_DEVICE_ID_1))
            {
                /* 预热中: 有效帧正在到达, 目标可能在量程内, 原地等待稳定后重测 (防过度盲进) */
                g_tof26_diag.warmWaits++;  /* 【诊断】 */
                osDelay(FLYBOX_TOF_SETTLE_MS);
                continue;
            }

            /* 传感器在线但目标超出测距范围: 按方向区分盲进
             * 靠近方向 (+1): 盲进靠近, 把目标拉进视野 (合理)
             * 远离方向 (-1): 目标本应在近处可见, 看不到=无箱/超程,
             *   盲进只会把目标推得更远永远救不回 → 不移动, 直接报错 */
            if (direction < 0)
            {
                g_tof26_diag.exitReason = 5U;  /* 【诊断】远离方向无目标, 拒绝盲进 */
                s_state = FLYBOX_ERROR;
                return HAL_ERROR;
            }
            g_tof26_diag.blindSteps++;  /* 【诊断】 */
            float blindStep = FLYBOX_TOF_SEARCH_STEP_MM;
            Motor_SetTarget(FLYBOX_ID_CONVEYOR_L, blindStep, MOTOR_TARGET_RELATIVE);
            Motor_SetTarget(FLYBOX_ID_CONVEYOR_R, blindStep, MOTOR_TARGET_RELATIVE);
            HAL_StatusTypeDef result = FlyBox_WaitMultipleReached(convIds, 2U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
            if (result != HAL_OK)
            {
                g_tof26_diag.exitReason = 3U;  /* 【诊断】盲进时电机错误 */
                s_state = FLYBOX_ERROR;
                return result;
            }
            osDelay(FLYBOX_TOF_SETTLE_MS);
            continue;  /* 盲进后重新测距 */
        }

        /* 【诊断】记录第一次有效测距和每次退出前的测距 */
        if (g_tof26_diag.firstDist == TOF8_DISTANCE_INVALID_MM)
        {
            g_tof26_diag.firstDist = dist;
        }
        g_tof26_diag.exitDist = dist;

        /* 计算剩余距离 (根据方向: 靠近=dist-target, 远离=target-dist) */
        float remaining = (direction > 0) ? (dist - target_mm) : (target_mm - dist);

        /* 已收敛: 剩余距离 ≤ 收敛容差 */
        if (remaining <= FLYBOX_TOF_CONVERGE_MM)
        {
            g_tof26_diag.exitReason = 0U;  /* 【诊断】正常收敛 */
            s_state = FLYBOX_COMPLETE;
            return HAL_OK;
        }

        /* 位置控制移动剩余距离 (双电机同步, 方向由 direction 决定) */
        float move = (direction > 0) ? remaining : -remaining;
        Motor_SetTarget(FLYBOX_ID_CONVEYOR_L, move, MOTOR_TARGET_RELATIVE);
        Motor_SetTarget(FLYBOX_ID_CONVEYOR_R, move, MOTOR_TARGET_RELATIVE);

        /* 等待双电机自然到位 */
        HAL_StatusTypeDef result = FlyBox_WaitMultipleReached(convIds, 2U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
        if (result != HAL_OK)
        {
            g_tof26_diag.exitReason = 3U;  /* 【诊断】逼近时电机错误 */
            s_state = FLYBOX_ERROR;
            return result;
        }

        /* 等待机械稳定后再测距 */
        osDelay(FLYBOX_TOF_SETTLE_MS);
    }

    /* 迭代超限, 未收敛 */
    g_tof26_diag.exitReason = 1U;  /* 【诊断】迭代超时 */
    s_state = FLYBOX_ERROR;
    return HAL_TIMEOUT;
}

/**
  * @brief  钩爪电机 (3) 相对旋转指定角度 (调试用, 不需要零点)
  *         内部换算: mm = angle_deg × FLYBOX_GRIPPER_MM_PER_DEG
  */
HAL_StatusTypeDef FlyBox_GripperTurnDeg(float angle_deg, float speed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;

    Motor_SetSpeed(FLYBOX_ID_GRIPPER, speed_mm_s);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER, accel_mm_s2);
    osDelay(5);

    float dist_mm = FLYBOX_GRIPPER_DEG_TO_MM(angle_deg);
    Motor_SetTarget(FLYBOX_ID_GRIPPER, dist_mm, MOTOR_TARGET_RELATIVE);

    HAL_StatusTypeDef result = FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  钩爪旋转电机 (4) 相对旋转指定角度 (调试用, 不需要零点)
  *         内部换算: mm = angle_deg × FLYBOX_GRIPPER_ROTATE_MM_PER_DEG
  */
HAL_StatusTypeDef FlyBox_GripperRotateDeg(float angle_deg, float speed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;

    Motor_SetSpeed(FLYBOX_ID_GRIPPER_ROTATE, speed_mm_s);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_ROTATE, accel_mm_s2);
    osDelay(5);

    float dist_mm = FLYBOX_GRIPPER_ROTATE_DEG_TO_MM(angle_deg);
    Motor_SetTarget(FLYBOX_ID_GRIPPER_ROTATE, dist_mm, MOTOR_TARGET_RELATIVE);

    HAL_StatusTypeDef result = FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  钩爪旋转电机 (4) 绝对旋转到指定角度 (需先归零)
  *         绝对定位: 0°=面对箱体(正面), 90°=侧面让位
  */
HAL_StatusTypeDef FlyBox_GripperRotateAbs(float angle_deg, float speed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    if (!s_homed) { s_state = FLYBOX_ERROR; return HAL_ERROR; }  /* 未归零, 绝对定位不可用 */

    s_state = FLYBOX_BUSY;

    Motor_SetSpeed(FLYBOX_ID_GRIPPER_ROTATE, speed_mm_s);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_ROTATE, accel_mm_s2);
    osDelay(5);

    /* 机械限位零点偏移换算: 新坐标目标 = 旧目标 - 限位偏移
     * (限位=-45°偏移 → 正面0°变为+45°, 侧面90°变为+135°) */
    float dist_mm = FLYBOX_GRIPPER_ROTATE_DEG_TO_MM(angle_deg - FLYBOX_HOME_OFFSET_GRIPPER_ROTATE_DEG);
    Motor_SetTarget(FLYBOX_ID_GRIPPER_ROTATE, dist_mm, MOTOR_TARGET_ABSOLUTE);

    HAL_StatusTypeDef result = FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  钩爪电机 (3) 归零: 力矩回零 (撞机械限位定位零点)
  *         归零后绝对位置才有意义
  */
HAL_StatusTypeDef FlyBox_GripperHome(void)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;

    HAL_StatusTypeDef result = FlyBox_HomeMotorByTorque(FLYBOX_ID_GRIPPER);
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  钩爪旋转电机 (4) 归零: 力矩回零
  */
HAL_StatusTypeDef FlyBox_GripperRotateHome(void)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;

    HAL_StatusTypeDef result = FlyBox_HomeMotorByTorque(FLYBOX_ID_GRIPPER_ROTATE);
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  钩爪传送带 (5) 归零: 力矩回零
  */
HAL_StatusTypeDef FlyBox_GripperBeltHome(void)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;

    HAL_StatusTypeDef result = FlyBox_HomeMotorByTorque(FLYBOX_ID_GRIPPER_BELT);
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  钩爪钩住箱体 (绝对位置, 需先 FlyBox_GripperHome 归零)
  */
HAL_StatusTypeDef FlyBox_GripperHook(void)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    if (!s_homed) { s_state = FLYBOX_ERROR; return HAL_ERROR; }  /* 未归零, 绝对定位不可用 */

    s_state = FLYBOX_BUSY;

    Motor_SetSpeed(FLYBOX_ID_GRIPPER, FLYBOX_DEFAULT_SPEED);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER, FLYBOX_DEFAULT_ACCEL);
    osDelay(5);

    /* 机械限位零点偏移换算: 钩取 -90°(旧) → -135°(新, 限位=+45°偏移) */
    float dist_mm = FLYBOX_GRIPPER_DEG_TO_MM(FLYBOX_GRIPPER_HOOK_DEG - FLYBOX_HOME_OFFSET_GRIPPER_DEG);
    Motor_SetTarget(FLYBOX_ID_GRIPPER, dist_mm, MOTOR_TARGET_ABSOLUTE);

    HAL_StatusTypeDef result = FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  钩爪松开箱体 (绝对位置, 需先 FlyBox_GripperHome 归零)
  */
HAL_StatusTypeDef FlyBox_GripperRelease(void)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    if (!s_homed) { s_state = FLYBOX_ERROR; return HAL_ERROR; }  /* 未归零, 绝对定位不可用 */

    s_state = FLYBOX_BUSY;

    Motor_SetSpeed(FLYBOX_ID_GRIPPER, FLYBOX_DEFAULT_SPEED);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER, FLYBOX_DEFAULT_ACCEL);
    osDelay(5);

    /* 机械限位零点偏移换算: 松开 0°(旧) → -45°(新, 限位=+45°偏移) */
    float dist_mm = FLYBOX_GRIPPER_DEG_TO_MM(FLYBOX_GRIPPER_RELEASE_DEG - FLYBOX_HOME_OFFSET_GRIPPER_DEG);
    Motor_SetTarget(FLYBOX_ID_GRIPPER, dist_mm, MOTOR_TARGET_ABSOLUTE);

    HAL_StatusTypeDef result = FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  钩爪传送带回到原点 (绝对位置 0, 需先 FlyBox_GripperBeltHome 归零)
  */
HAL_StatusTypeDef FlyBox_GripperBelttoHome(void)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    if (!s_homed) { s_state = FLYBOX_ERROR; return HAL_ERROR; }  /* 未归零, 绝对定位不可用 */

    s_state = FLYBOX_BUSY;

    Motor_SetSpeed(FLYBOX_ID_GRIPPER_BELT, FLYBOX_DEFAULT_SPEED);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_BELT, FLYBOX_DEFAULT_ACCEL);
    osDelay(5);

    /* 机械限位零点偏移换算 (皮带限位即旧零点, 偏移=0) */
    Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, FLYBOX_HOME_OFFSET_GRIPPER_BELT_MM, MOTOR_TARGET_ABSOLUTE);

    HAL_StatusTypeDef result = FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER_BELT, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  钩爪传送带前后移动指定距离
  * @param  distance_mm  距离 (mm, 正=前进靠近箱体, 负=后退)
  */
HAL_StatusTypeDef FlyBox_GripperBeltMove(float distance_mm, float speed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;

    Motor_SetSpeed(FLYBOX_ID_GRIPPER_BELT, speed_mm_s);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_BELT, accel_mm_s2);
    osDelay(5);

    Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, distance_mm, MOTOR_TARGET_RELATIVE);

    HAL_StatusTypeDef result = FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER_BELT, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  钩爪传送带前进直到 tof25 (正面) 距离 <= target_mm
  *         采用 "测距 → 算剩余 → 位置控制走剩余 → 等自然到位 → 再测距" 迭代逼近
  *         电机始终由位置控制停止, 不使用急停, 不进入保护状态
  *         s_belt_fwd_mm 累计实际前进距离 (供 PickBox Step3 使用)
  *
  *         TOF 异常处理:
  *           - TOF 无数据 (传感器故障): 尝试重启, 重启失败则报错
  *           - TOF 在线但滤波器预热中: 原地等待稳定后重测 (不盲进, 防过度盲进)
  *           - TOF 在线但数据无效 (箱体超出测距范围): 盲进一段让目标进入视野
  *           - 迭代超限: 返回 HAL_TIMEOUT
  *
  *         限程保护: 累计前进 (盲进+逼近) 将超过 FLYBOX_BELT_MAX_FWD_MM 时,
  *           不移动, 置 s_belt_fwd_limit_hit 并返回 HAL_TIMEOUT
  *           (箱体不在预期位置, 继续前进可能撞限位/超程, 由 PickBox 回零重试)
  */
HAL_StatusTypeDef FlyBox_GripperBeltMoveUntilTof25(float target_mm, float speed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;

    /* 启动前检查 TOF25 是否在线 */
    if (TOF8_EnsureAlive(TOF8_DEVICE_ID_0, FLYBOX_TOF_ALIVE_TIMEOUT_MS, FLYBOX_TOF_RESTART_RETRY) != HAL_OK)
    {
        s_state = FLYBOX_ERROR;
        return HAL_ERROR;  /* TOF25 故障, 无法执行联动 */
    }

    Motor_SetSpeed(FLYBOX_ID_GRIPPER_BELT, speed_mm_s);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_BELT, accel_mm_s2);
    osDelay(5);

    s_belt_fwd_mm = 0.0f;  /* 重置累计前进距离 */
    s_belt_fwd_limit_hit = 0U;  /* 重置超限标志 */

    for (uint32_t iter = 0U; iter < FLYBOX_TOF_MAX_ITER; iter++)
    {
        /* 读取 tof25 距离 (中值滤波, 抑制噪声) */
        float dist = TOF8_GetDistanceMedian(TOF8_DEVICE_ID_0,
                                            FLYBOX_TOF_MEDIAN_SAMPLES,
                                            FLYBOX_TOF_MEDIAN_INTERVAL_MS);

        /* TOF 数据无效: 区分 "传感器故障" 和 "目标超出测距范围" */
        if (dist == TOF8_DISTANCE_INVALID_MM)
        {
            if (!TOF8_IsAlive(TOF8_DEVICE_ID_0, FLYBOX_TOF_ALIVE_TIMEOUT_MS))
            {
                /* 传感器故障 (无 CAN 帧): 尝试重启 */
                if (TOF8_EnsureAlive(TOF8_DEVICE_ID_0, FLYBOX_TOF_ALIVE_TIMEOUT_MS, FLYBOX_TOF_RESTART_RETRY) != HAL_OK)
                {
                    s_state = FLYBOX_ERROR;
                    return HAL_ERROR;  /* TOF25 重启失败 */
                }
                osDelay(FLYBOX_TOF_SETTLE_MS);
                continue;  /* 重启后重新测距 */
            }

            /* 传感器在线但数据无效: 区分 "滤波器预热" 和 "目标超出测距范围" */
            if (TOF8_IsWarmingUp(TOF8_DEVICE_ID_0))
            {
                /* 预热中: 有效帧正在到达, 目标可能在量程内, 原地等待稳定后重测 (防过度盲进) */
                osDelay(FLYBOX_TOF_SETTLE_MS);
                continue;
            }

            /* 传感器在线但目标超出测距范围: 盲进一段, 让目标进入视野 */
            /* 限程保护: 盲进后累计前进将超限 → 不移动, 置标志报错
             * (箱体不在预期位置, 继续盲进可能撞限位/超程) */
            if (s_belt_fwd_mm + FLYBOX_TOF_SEARCH_STEP_MM > FLYBOX_BELT_MAX_FWD_MM)
            {
                s_belt_fwd_limit_hit = 1U;
                s_state = FLYBOX_ERROR;
                return HAL_TIMEOUT;
            }
            Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, FLYBOX_TOF_SEARCH_STEP_MM, MOTOR_TARGET_RELATIVE);
            s_belt_fwd_mm += FLYBOX_TOF_SEARCH_STEP_MM;
            HAL_StatusTypeDef result = FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER_BELT, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
            if (result != HAL_OK)
            {
                s_state = FLYBOX_ERROR;
                return result;
            }
            osDelay(FLYBOX_TOF_SETTLE_MS);
            continue;  /* 盲进后重新测距 */
        }

        /* 计算剩余距离 */
        float remaining = dist - target_mm;

        /* 已收敛: 剩余距离 ≤ 收敛容差 */
        if (remaining <= FLYBOX_TOF_CONVERGE_MM)
        {
            s_state = FLYBOX_COMPLETE;
            return HAL_OK;
        }

        /* 限程保护: 逼近移动后累计前进将超限 → 不移动, 置标志报错 */
        if (s_belt_fwd_mm + remaining > FLYBOX_BELT_MAX_FWD_MM)
        {
            s_belt_fwd_limit_hit = 1U;
            s_state = FLYBOX_ERROR;
            return HAL_TIMEOUT;
        }

        /* 位置控制前进剩余距离 */
        Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, remaining, MOTOR_TARGET_RELATIVE);
        s_belt_fwd_mm += remaining;

        /* 等待电机自然到位 */
        HAL_StatusTypeDef result = FlyBox_WaitMotorReached(FLYBOX_ID_GRIPPER_BELT, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
        if (result != HAL_OK)
        {
            s_state = FLYBOX_ERROR;
            return result;
        }

        /* 等待机械稳定后再测距 */
        osDelay(FLYBOX_TOF_SETTLE_MS);
    }

    /* 迭代超限, 未收敛 */
    s_state = FLYBOX_ERROR;
    return HAL_TIMEOUT;
}

/**
  * @brief  TOF25 判定: 箱体是否已被拉上传送带 (PickBox Step3 后调用)
  *         成功特征: TOF25 仍能看到箱体, 距离 ≤ FLYBOX_TOF25_VERIFY_MAX_MM
  *         失败特征: 数据无效 (箱体不在视野) 或距离 > 阈值 (箱体未被拉动)
  *         判定距离记录到 g_pick_diag.verifyDist, 便于实测调阈值
  * @retval HAL_OK = 箱体已在传送带上, HAL_ERROR = 判定失败 (钩箱未成功)
  */
static HAL_StatusTypeDef FlyBox_VerifyBoxOnConveyor(void)
{
    osDelay(FLYBOX_TOF_SETTLE_MS);  /* 等待机械稳定再测距 */

    float dist = TOF8_GetDistanceMedian(TOF8_DEVICE_ID_0,
                                        FLYBOX_TOF_MEDIAN_SAMPLES,
                                        FLYBOX_TOF_MEDIAN_INTERVAL_MS);
    g_pick_diag.verifyDist = dist;

    if (dist == TOF8_DISTANCE_INVALID_MM || dist > FLYBOX_TOF25_VERIFY_MAX_MM)
    {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
  * @brief  Step6 超限清障 + 钩爪转正 (PickBox 恢复用, 此时钩爪在侧面 90°, 已松开)
  *         1. TOF26 测距 ≥ FLYBOX_PLACE_TOF26_TARGET_MM (215mm) → 箱体离钩爪足够远, 直接转正
  *         2. 否则 → 传送带远离方向把箱体推到 215mm → 再转正
  *         转正后机构回到 "正面+松开" = Step1 合法起始态, 可重新钩取
  *         清障方式记录到 g_pick_diag.s6Clear (1=直接转正 2=推远后转正 3=推远失败)
  * @retval HAL_OK = 已转正可重试, HAL_ERROR = 清障失败
  */
static HAL_StatusTypeDef FlyBox_CheckClearanceAndFaceFront(void)
{
    float dist = TOF8_GetDistanceMedian(TOF8_DEVICE_ID_1,
                                        FLYBOX_TOF_MEDIAN_SAMPLES,
                                        FLYBOX_TOF_MEDIAN_INTERVAL_MS);

    /* 箱体太近 (或测不到): 先用传送带把箱体推远到安全距离
     * 远离方向 allowBlind 语义: 测不到目标会直接报错, 不会盲进 */
    if (dist == TOF8_DISTANCE_INVALID_MM || dist < FLYBOX_PLACE_TOF26_TARGET_MM)
    {
        HAL_StatusTypeDef push = FlyBox_ConveyorRunUntilTof26(FLYBOX_PLACE_TOF26_TARGET_MM,
                                                              FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL, -1);
        if (push != HAL_OK)
        {
            g_pick_diag.s6Clear = 3U;  /* 【诊断】推远失败 */
            return HAL_ERROR;
        }
        g_pick_diag.s6Clear = 2U;  /* 【诊断】推远后转正 */
    }
    else
    {
        g_pick_diag.s6Clear = 1U;  /* 【诊断】直接转正 */
    }

    /* 转正钩爪 (绝对 0°) */
    return FlyBox_GripperRotateAbs(0.0f, FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL);
}

/* ======================== 旋转同步送箱/让位 (取箱 Step5+6 / 放箱 Step1+2) ======================== */

/**
  * @brief  取箱旋转同步: 钩爪转 90° 避让 + 传送带步进送箱 (Step5+6 合并)
  *
  *         原理: 旋转 0°→90° 过程中, 钩爪扫掠半径 215mm 会推开箱体,
  *              传送带在扫掠路径上同步推进, 保证箱体同时被拉进传送带内侧。
  *              旋转到位后不做 TOF26 闭环 (旋转过程中侧面测距不准).
  *
  *         物理模型: 扫掠 0°→90°, 箱体距圆心 d 从 215→~65mm;
  *         同步步进: 每前进 SYNC_STEP_MM (20mm) 联动一次.
  *
  *         @retval HAL_OK      同步到位
  *         @retval HAL_TIMEOUT 电机超时
  *         @retval HAL_ERROR   硬件/参数错误
  */
static HAL_StatusTypeDef FlyBox_RotateSyncPick(void)
{
    s_state = FLYBOX_BUSY;
    g_pick_diag.syncSteps = 0U;

    /* 旋转电机加速 */
    Motor_SetSpeed(FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_ROTATE_SPEED);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_ROTATE_ACCEL);
    osDelay(5);

    /* 先启动旋转, 延时等待旋转到安全角度后再启动传送带
     * 安全模型: 箱体可前进量 = 215 - 215·cos(θ)
     *   (初始箱体前沿 230mm, 臂尖 215mm, 余量 15mm)
     *   θ=30° → 可前进 29mm, 但传送带需走 180mm, 臂仍会扫到箱体
     *   θ=60° → 可前进 108mm, 安全
     *
     * 不依赖 CAN 状态轮询 (位置更新有延迟), 用 osDelay 等待。
     * 300ms @ 1000mm/s+2000mm/s² → ~11°, 实测 300ms 合适。
     *
     * 目标换算 (同 FlyBox_GripperRotateAbs):
     *   target_mm = (角度 - 偏移) × mm/度
     *   90° → (90-(-45))×7.955 = 1074mm */
    Motor_SetTarget(FLYBOX_ID_GRIPPER_ROTATE,
                    FLYBOX_GRIPPER_ROTATE_DEG_TO_MM(FLYBOX_GRIPPER_ROTATE_DEG - FLYBOX_HOME_OFFSET_GRIPPER_ROTATE_DEG),
                    MOTOR_TARGET_ABSOLUTE);
    osDelay(300);

    /* 延时后设置传送带速度/加速度/目标 */
    Motor_SetSpeed(FLYBOX_ID_CONVEYOR_L, FLYBOX_DEFAULT_SPEED);
    Motor_SetSpeed(FLYBOX_ID_CONVEYOR_R, FLYBOX_DEFAULT_SPEED);
    osDelay(5);
    Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_L, FLYBOX_DEFAULT_ACCEL);
    Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_R, FLYBOX_DEFAULT_ACCEL);
    osDelay(5);

    /* 总前进量 = 215(半径) + 15(余量) - 50(目标) = 180mm */
    {
        const float totalFwd = FLYBOX_HOOK_RADIUS_MM + FLYBOX_HOOK_MARGIN_MM - FLYBOX_HOOK_FINAL_MM;
        Motor_SetTarget(FLYBOX_ID_CONVEYOR_L, totalFwd, MOTOR_TARGET_RELATIVE);
        Motor_SetTarget(FLYBOX_ID_CONVEYOR_R, totalFwd, MOTOR_TARGET_RELATIVE);
        g_pick_diag.syncSteps = 1U;
    }

    /* 等待三电机全部到位 */
    {
        static const uint8_t allIds[3] = {
            FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R
        };
        HAL_StatusTypeDef st = FlyBox_WaitMultipleReached(allIds, 3U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
        if (st != HAL_OK) return st;
    }

    osDelay(FLYBOX_ROTATE_SETTLE_MS);
    g_pick_diag.syncEndMm = FLYBOX_HOOK_FINAL_MM;
    return HAL_OK;
}

/* ======================== 高层流程接口 ======================== */

/**
  * @brief  完整取箱流程 (6 步, 单一 attempt 循环 + 两处可重试失败点)
  *
  *         入口安全前置: 松开 + 钩爪转正 (防上次中断后钩爪留在侧面)
  *
  *         Step 1: 钩爪传送带前进靠近箱体 (TOF25 联动)
  *                 前进超限 (s_belt_fwd_limit_hit) → 钩爪带回零 + 等2s → 重试
  *         Step 2: 钩爪钩住 (绝对 -90°)
  *         Step 3: 钩爪传送带归零 + 传送带同速同步前进 → 一起停 (距离一致)
  *         Step 3.5: TOF25 判定箱体是否被拉上传送带
  *                   失败 → 松爪 → 回 Step1 重试 (重试点 A)
  *         Step 4: 松开钩爪 (绝对 0°, 旋转前必须松)
  *         Step 5+6 (合并): 旋转 90° 避让 + 传送带同步送箱 (FlyBox_RotateSyncPick)
  *                 HAL_TIMEOUT → 清障转正 → 回 Step1 重钩 (重试点 B)
  *                 HAL_ERROR → 直接 abort
  *
  *         两处重试共用 FLYBOX_PICK_MAX_ATTEMPTS 预算, 单一 abort 出口;
  *         重试后机构均回到 "正面+松开" = Step1 合法起始态
  */
HAL_StatusTypeDef FlyBox_PickBox(void)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    if (!s_homed) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    HAL_StatusTypeDef status;

    /* 【诊断】重置步骤记录 (0xFF = 未执行) */
    g_pick_diag.lastStep = 0U;
    g_pick_diag.s1 = 0xFFU;
    g_pick_diag.s2 = 0xFFU;
    g_pick_diag.s3 = 0xFFU;
    g_pick_diag.s4 = 0xFFU;
    g_pick_diag.verify = 0xFFU;
    g_pick_diag.attempts = 0U;
    g_pick_diag.s6Recoveries = 0U;
    g_pick_diag.s6Clear = 0xFFU;
    g_pick_diag.beltLimitHit = 0xFFU;
    g_pick_diag.verifyDist = TOF8_DISTANCE_INVALID_MM;
    g_pick_diag.syncSteps = 0U;
    g_pick_diag.syncEndMm = 0.0f;
    g_pick_diag.gripperPosAtAbort = 0;
    g_pick_diag.gripperTorqueAtAbort = 0;
    g_pick_diag.abortReleaseResult = 0xFFU;

    /* ===== 入口安全前置: 松开 + 钩爪转正 (一次性, 循环外) =====
     * 防上次流程中断后钩爪留在侧面/钩住状态, Step1 侧着接近箱体 */
    status = FlyBox_GripperRelease();
    if (status != HAL_OK) { FlyBox_AbortAndSafe(); return status; }
    status = FlyBox_GripperRotateAbs(0.0f, FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL);
    if (status != HAL_OK) { FlyBox_AbortAndSafe(); return status; }
    osDelay(FLYBOX_STEP_SETTLE_MS);

    /* ===== Step 1~6 单一 attempt 循环 ===== */
    for (uint8_t attempt = 1U; attempt <= FLYBOX_PICK_MAX_ATTEMPTS; attempt++)
    {
        g_pick_diag.attempts = attempt;

        /* ===== Step 1: 钩爪传送带前进靠近箱体 (TOF25 联动) ===== */
        g_pick_diag.lastStep = 1U;
        status = FlyBox_GripperBeltMoveUntilTof25(FLYBOX_TOF25_HOOK_THRESH_MM,
                                                  FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL);
        g_pick_diag.s1 = (uint8_t)status;  /* 【诊断】 */
        if (status != HAL_OK)
        {
            if (s_belt_fwd_limit_hit)
            {
                /* 前进超限 (箱体不在预期位置): 皮带回零 (回拉方向, 安全) →
                 * 非最后一次: 等稳定后回 Step1 重试;
                 * 最后一次: 回零后以错误返回, 避免皮带遗留前限位
                 * (下次 Step1 相对前进会从错误起点累计, 再次超限) */
                g_pick_diag.beltLimitHit = 1U;  /* 【诊断】 */
                if (FlyBox_GripperBeltHome() != HAL_OK) { FlyBox_AbortAndSafe(); return HAL_ERROR; }
                if (attempt < FLYBOX_PICK_MAX_ATTEMPTS)
                {
                    osDelay(FLYBOX_RETRY_SETTLE_MS);
                    continue;
                }
                s_state = FLYBOX_ERROR;
                return HAL_TIMEOUT;
            }
            FlyBox_AbortAndSafe();
            return status;
        }
        osDelay(FLYBOX_STEP_SETTLE_MS);

        /* ===== Step 2: 钩爪钩住 (绝对 -90°) ===== */
        g_pick_diag.lastStep = 2U;
        status = FlyBox_GripperHook();
        g_pick_diag.s2 = (uint8_t)status;  /* 【诊断】 */
        if (status != HAL_OK) { FlyBox_AbortAndSafe(); return status; }
        osDelay(FLYBOX_STEP_SETTLE_MS);

        /* ===== Step 3: 钩爪传送带归零 + 传送带前进 s_belt_fwd_mm, 等全部到位 ===== */
        /* 原理: 钩爪传送带用归零 (绝对回零位, 避免相对定位累积超限),
         * 传送带借用 s_belt_fwd_mm 做相对前进, 保证与钩爪移动距离匹配 */
        g_pick_diag.lastStep = 3U;
        s_state = FLYBOX_BUSY;

        /* 三电机设置相同速度和加速度 */
        Motor_SetSpeed(FLYBOX_ID_GRIPPER_BELT, FLYBOX_DEFAULT_SPEED);
        Motor_SetSpeed(FLYBOX_ID_CONVEYOR_L, FLYBOX_DEFAULT_SPEED);
        Motor_SetSpeed(FLYBOX_ID_CONVEYOR_R, FLYBOX_DEFAULT_SPEED);
        osDelay(5);
        Motor_SetAcceleration(FLYBOX_ID_GRIPPER_BELT, FLYBOX_DEFAULT_ACCEL);
        Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_L, FLYBOX_DEFAULT_ACCEL);
        Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_R, FLYBOX_DEFAULT_ACCEL);
        osDelay(5);

        /* 同时发送目标: 钩爪带绝对回零位, 传送带相对前进 s_belt_fwd_mm */
        Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, FLYBOX_HOME_OFFSET_GRIPPER_BELT_MM, MOTOR_TARGET_ABSOLUTE);
        Motor_SetTarget(FLYBOX_ID_CONVEYOR_L, s_belt_fwd_mm, MOTOR_TARGET_RELATIVE);
        Motor_SetTarget(FLYBOX_ID_CONVEYOR_R, s_belt_fwd_mm, MOTOR_TARGET_RELATIVE);

        /* 等待三电机全部到位 */
        {
            static const uint8_t step3Ids[3] = {
                FLYBOX_ID_GRIPPER_BELT, FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R
            };
            status = FlyBox_WaitMultipleReached(step3Ids, 3U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
        }
        g_pick_diag.s3 = (uint8_t)status;  /* 【诊断】 */
        if (status != HAL_OK) { FlyBox_AbortAndSafe(); return status; }
        osDelay(FLYBOX_STEP_SETTLE_MS);

        /* ===== Step 3.5: TOF25 判定箱体是否被拉上传送带 (重试点 A) ===== */
        status = FlyBox_VerifyBoxOnConveyor();
        g_pick_diag.verify = (status == HAL_OK) ? 0U : 1U;  /* 【诊断】 */
        if (status != HAL_OK)
        {
            /* 判定失败: 所有尝试(含最后一次)统一先松爪, 此时电机未急停、驱动器正常响应;
             * 避免最后一次尝试依赖 AbortAndSafe 急停后的松爪 (急停可能令驱动器冻结/忽略目标) */
            status = FlyBox_GripperRelease();  /* 松爪 */
            if (status != HAL_OK)
            {
                FlyBox_AbortAndSafe();  /* 松爪失败: 急停 + 尽力松爪, 不动零点 */
                return status;
            }
            osDelay(FLYBOX_STEP_SETTLE_MS);

            if (attempt < FLYBOX_PICK_MAX_ATTEMPTS)
            {
                continue;  /* 回 Step1 重试 (不回退传送带, 箱体未动) */
            }
            /* 最后一次尝试: 钩爪已在正常状态松开到位, 直接返回错误;
             * 不走 AbortAndSafe——急停可能令驱动器冻结/忽略目标, 且
             * 钩爪已松到位无需再动; 是否需要重新回零由上层决定 */
            s_state = FLYBOX_ERROR;
            return HAL_TIMEOUT;
        }

        /* ===== Step 4: 松开钩爪 (绝对 0°, 旋转前必须松) ===== */
        g_pick_diag.lastStep = 4U;
        status = FlyBox_GripperRelease();
        g_pick_diag.s4 = (uint8_t)status;  /* 【诊断】 */
        if (status != HAL_OK) { FlyBox_AbortAndSafe(); return status; }
        osDelay(FLYBOX_STEP_SETTLE_MS);

        /* ===== Step 5+6 (合并): 旋转 90° + 传送带同步送箱 (重试点 B) ===== */
        g_pick_diag.lastStep = 5U;
        status = FlyBox_RotateSyncPick();
        if (status == HAL_OK)
        {
            s_state = FLYBOX_COMPLETE;
            return HAL_OK;  /* 取箱成功 */
        }

        /* 同步失败 (HAL_TIMEOUT): 清障转正 → 回 Step1 重钩 */
        if (status == HAL_TIMEOUT && attempt < FLYBOX_PICK_MAX_ATTEMPTS)
        {
            if (FlyBox_CheckClearanceAndFaceFront() == HAL_OK)
            {
                g_pick_diag.s6Recoveries++;  /* 【诊断】 */
                osDelay(FLYBOX_STEP_SETTLE_MS);
                continue;
            }
        }

        FlyBox_AbortAndSafe();
        return status;
    }

    /* 循环耗尽 (理论上不可达: 每个分支都已 return/continue) → 兜底报错 */
    FlyBox_AbortAndSafe();
    return HAL_TIMEOUT;
}

/**
  * @brief  放箱旋转同步: 旋转回正面 + 传送带同步后退让位 (Step1+2 合并)
  *
  *         原理: 旋转 90°→0° 过程中, 钩爪扫掠半径 215mm 会推挤箱体,
  *              传送带同步后退让位, 避免箱体被扫掠挤压。
  *              开环后退 180mm (230-50), 不依赖 TOF26 读数。
  *
  *         三电机同时启动, 等待全部到位:
  *           - 旋转电机: 90°→0° (绝对)
  *           - 传送带 L/R: 相对后退 FLYBOX_PLACE_SYNC_RETREAT_MM (180mm)
  *
  *         @retval HAL_OK      同步到位
  *         @retval HAL_TIMEOUT 电机超时
  *         @retval HAL_ERROR   硬件/参数错误
  */
static HAL_StatusTypeDef FlyBox_RotateSyncPlace(void)
{
    s_state = FLYBOX_BUSY;

     /* 设置传送带速度/加速度/目标 */
    Motor_SetSpeed(FLYBOX_ID_CONVEYOR_L, FLYBOX_DEFAULT_SPEED);
    Motor_SetSpeed(FLYBOX_ID_CONVEYOR_R, FLYBOX_DEFAULT_SPEED);
    osDelay(5);
    Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_L, FLYBOX_DEFAULT_ACCEL);
    Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_R, FLYBOX_DEFAULT_ACCEL);
    osDelay(5);
    Motor_SetTarget(FLYBOX_ID_CONVEYOR_L, -FLYBOX_PLACE_SYNC_RETREAT_MM, MOTOR_TARGET_RELATIVE);
    Motor_SetTarget(FLYBOX_ID_CONVEYOR_R, -FLYBOX_PLACE_SYNC_RETREAT_MM, MOTOR_TARGET_RELATIVE);

    osDelay(300);

    /* 延时后旋转电机启动 (回到 0° 开始位置)
     * 换算同 FlyBox_GripperRotateAbs: target = (角度 - 偏移) × mm/度
     * 0° → (0 - (-45)) × 7.955 = 358mm */
    Motor_SetSpeed(FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_ROTATE_SPEED);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_ROTATE_ACCEL);
    osDelay(5);
    Motor_SetTarget(FLYBOX_ID_GRIPPER_ROTATE,
                    FLYBOX_GRIPPER_ROTATE_DEG_TO_MM(0.0f - FLYBOX_HOME_OFFSET_GRIPPER_ROTATE_DEG),
                    MOTOR_TARGET_ABSOLUTE);  /* 0° = 正面 */
  

   
    /* 等待三电机全部到位 */
    {
        static const uint8_t syncIds[3] = {
            FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R
        };
        return FlyBox_WaitMultipleReached(syncIds, 3U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    }
}

/**
  * @brief  完整放箱流程 (5 步, 取箱逆序)
  *
  *         Step 1+2 (合并): 旋转回正面 + 传送带同步后退让位 (FlyBox_RotateSyncPlace)
  *         Step 2.5: TOF25 校验补偿 — 偏差 = 实测 - 期望, 限幅 ±30mm
  *         Step 3: 钩爪带前进 (绝对定位) + 传送带后退 (相对), 三电机同步推出箱体
  *         Step 4: 确认钩爪松开 (绝对 0°)
  *         Step 5: 钩爪传送带归位 (绝对 0)
  *
  *         任一步失败 → FlyBox_AbortAndSafe() → 返回错误
  */
HAL_StatusTypeDef FlyBox_PlaceBox(void)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    if (!s_homed) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    HAL_StatusTypeDef status;
    float convBackMm;  /* Step3 动态后退量, 含 TOF25 补偿 */

    /* ===== Step 1+2 (合并): 旋转回正面 + 传送带同步后退让位 ===== */
    status = FlyBox_RotateSyncPlace();
    if (status != HAL_OK) { FlyBox_AbortAndSafe(); return status; }
    osDelay(FLYBOX_STEP_SETTLE_MS);

    /* ===== Step 2.5: TOF25 校验补偿 ===== */
    {
        float distMm = TOF8_DISTANCE_INVALID_MM;
        TOF8_GetDistance(0U, &distMm);  /* TOF25 = deviceId 0 */

        if (distMm < TOF8_DISTANCE_INVALID_MM)
        {
            float delta = distMm - FLYBOX_TOF25_PLACE_EXPECT_MM;  /* Δ = 实测 - 期望(100mm) */
            if (delta > FLYBOX_PLACE_COMP_MAX_MM)  delta = FLYBOX_PLACE_COMP_MAX_MM;
            if (delta < -FLYBOX_PLACE_COMP_MAX_MM) delta = -FLYBOX_PLACE_COMP_MAX_MM;

            convBackMm = FLYBOX_PLACE_CONVEYOR_BACK_MM - FLYBOX_HOOK_MARGIN_MM - delta;
            /* 550 - 15 - Δ = 535 - Δ, 限幅 505~565 */
        }
        else
        {
            convBackMm = FLYBOX_PLACE_CONVEYOR_BACK_MM - FLYBOX_HOOK_MARGIN_MM;  /* TOF 无效, 不使用补偿 */
        }
    }

    /* ===== Step 3: 钩爪带前进 (绝对) + 传送带后退 (相对), 三电机同步推出 ===== */
    s_state = FLYBOX_BUSY;

    /* 三电机设置相同速度和加速度 */
    Motor_SetSpeed(FLYBOX_ID_GRIPPER_BELT, FLYBOX_DEFAULT_SPEED);
    Motor_SetSpeed(FLYBOX_ID_CONVEYOR_L, FLYBOX_DEFAULT_SPEED);
    Motor_SetSpeed(FLYBOX_ID_CONVEYOR_R, FLYBOX_DEFAULT_SPEED);
    osDelay(5);
    Motor_SetAcceleration(FLYBOX_ID_GRIPPER_BELT, FLYBOX_DEFAULT_ACCEL);
    Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_L, FLYBOX_DEFAULT_ACCEL);
    Motor_SetAcceleration(FLYBOX_ID_CONVEYOR_R, FLYBOX_DEFAULT_ACCEL);
    osDelay(5);

    /* 同时发送目标: 钩爪带绝对定位前进, 传送带相对后退 */
    Motor_SetTarget(FLYBOX_ID_GRIPPER_BELT, FLYBOX_PLACE_BELT_FWD_MM + FLYBOX_HOME_OFFSET_GRIPPER_BELT_MM, MOTOR_TARGET_ABSOLUTE);
    Motor_SetTarget(FLYBOX_ID_CONVEYOR_L, -convBackMm, MOTOR_TARGET_RELATIVE);
    Motor_SetTarget(FLYBOX_ID_CONVEYOR_R, -convBackMm, MOTOR_TARGET_RELATIVE);

    /* 等待三电机全部到位 */
    {
        static const uint8_t step3Ids[3] = {
            FLYBOX_ID_GRIPPER_BELT, FLYBOX_ID_CONVEYOR_L, FLYBOX_ID_CONVEYOR_R
        };
        status = FlyBox_WaitMultipleReached(step3Ids, 3U, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    }
    if (status != HAL_OK) { FlyBox_AbortAndSafe(); return status; }
    osDelay(FLYBOX_STEP_SETTLE_MS);

    /* ===== Step 4: 确认钩爪松开 (绝对 0°) ===== */
    status = FlyBox_GripperRelease();
    if (status != HAL_OK) { FlyBox_AbortAndSafe(); return status; }
    osDelay(FLYBOX_STEP_SETTLE_MS);

    /* ===== Step 5: 钩爪传送带归位 (绝对 0) ===== */
    status = FlyBox_GripperBelttoHome();
    if (status != HAL_OK) { FlyBox_AbortAndSafe(); return status; }

    s_state = FLYBOX_COMPLETE;
    return HAL_OK;
}

/* ======================== 标定测试接口 ======================== */

/* 探测函数运行时阈值覆盖: >0 = 使用此阈值提前停止, -1 = 使用配置阈值 */
static int16_t s_probeThreshOverride = -1;
/* 探测函数停止方式 (供 FlyBox_Debug_CalibrateTorque 读取):
 * 0=未执行 1=力矩提前停 2=驱动器堵转 3=行程走完 4=超时 */
static volatile uint8_t s_probeStopReason = 0U;

/**
  * @brief  单电机往返测试: 前进 dist → 停 → 后退 dist 归位
  *         用于验证电机到位 / 方向 / 错误检测, 以及人工测量实际距离标定轮径
  */
HAL_StatusTypeDef FlyBox_TestMotor(uint8_t deviceId, float dist_mm,
                                   float speed_mm_s, float accel_mm_s2)
{
    if (!s_initialized) { s_state = FLYBOX_ERROR; return HAL_ERROR; }
    if (deviceId < 1U || deviceId > FLYBOX_MOTOR_COUNT) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;

    /* 设置速度和加速度 */
    Motor_SetSpeed(deviceId, speed_mm_s);
    osDelay(5);
    Motor_SetAcceleration(deviceId, accel_mm_s2);
    osDelay(5);

    /* 前进 dist */
    Motor_SetTarget(deviceId, dist_mm, MOTOR_TARGET_RELATIVE);
    HAL_StatusTypeDef result = FlyBox_WaitMotorReached(deviceId, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);
    if (result != HAL_OK)
    {
        s_state = FLYBOX_ERROR;
        return result;
    }

    /* 停留, 便于人工测量实际距离 */
    osDelay(3000);

    /* 后退 dist 归位 */
    Motor_SetTarget(deviceId, -dist_mm, MOTOR_TARGET_RELATIVE);
    result = FlyBox_WaitMotorReached(deviceId, FLYBOX_MOTOR_WAIT_TIMEOUT_MS);

    s_state = (result == HAL_OK) ? FLYBOX_COMPLETE : FLYBOX_ERROR;
    return result;
}

/**
  * @brief  计算轮径修正值
  *         修正轮径 = 当前轮径 × (实测距离 / 指令距离)
  *         将返回值填回对应 FLYBOX_DIA_xxx 宏即可
  */
float FlyBox_CalibrateDiameter(uint8_t deviceId, float currentDia_mm,
                               float cmdDist_mm, float measuredDist_mm)
{
    (void)deviceId;

    if (cmdDist_mm <= 0.0f || measuredDist_mm <= 0.0f)
    {
        return currentDia_mm;
    }

    return currentDia_mm * (measuredDist_mm / cmdDist_mm);
}

/**
  * @brief  【调试】力矩探测: 低速朝指定方向相对运动一段, 用于实测力矩基线/撞限位力矩
  *
  *         停止判据 (任一满足即停, 停止方式记录在 s_probeStopReason):
  *           1. 力矩超阈提前停: 运行时阈值 = s_probeThreshOverride (>0 时),
  *              否则用配置阈值; 阈值=0 且无覆盖 → 不启用力矩停止
  *           2. 驱动器堵转错误 0x0104 (备份)
  *           3. 行程走完 (剩余位移归零)
  *           4. 超时
  *
  *         用法: Watch 窗口观察
  *           g_motor_status[deviceId-1].torque / torquePeak / torqueMin / torqueMax
  *         撞限位时建议用运行时阈值提前停止 (柔和接触), 避免硬顶导致
  *         驱动器堵转保护解除使能 / 电源过流下电
  */
HAL_StatusTypeDef FlyBox_Debug_TorqueProbe(uint8_t deviceId, int8_t dir,
                                           float distance_mm, float speed_mm_s)
{
    volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
    if (p == NULL) { s_state = FLYBOX_ERROR; return HAL_ERROR; }

    s_state = FLYBOX_BUSY;
    s_probeStopReason = 0U;

    /* 确定运行时力矩阈值: 覆盖值优先, 否则配置阈值; ≤0 = 不启用力矩停止 */
    int16_t thresh;
    if (s_probeThreshOverride > 0)
    {
        thresh = s_probeThreshOverride;
    }
    else
    {
        int8_t d; float sm; int16_t cfgThresh;
        thresh = FlyBox_GetHomeConfig(deviceId, &d, &sm, &cfgThresh) ? cfgThresh : 0;
    }

    /* 清保护状态 + 清力矩统计, 便于观察本次运动 */
    Motor_Unlock(deviceId);
    osDelay(50);
    Motor_Lock(deviceId);
    osDelay(50);
    p->lastErrorCode = 0U;
    Motor_ResetTorqueStats(deviceId);

    /* 低速相对运动 */
    Motor_SetSpeed(deviceId, speed_mm_s);
    Motor_SetAcceleration(deviceId, FLYBOX_CALIB_ACCEL);
    osDelay(5);

    float target = (dir > 0) ? distance_mm : -distance_mm;
    Motor_SetTarget(deviceId, target, MOTOR_TARGET_RELATIVE);

    /* 轮询: 到位 / 力矩超阈 / 堵转 */
    const uint32_t poll_ms = 10U;
    uint32_t elapsed = 0U;
    uint32_t debounce = 0U;
    uint8_t  motorStarted = 0U;

    while (elapsed < FLYBOX_CALIB_TIMEOUT_MS)
    {
        int16_t tq = p->torque;
        int16_t absTq = (tq >= 0) ? tq : (int16_t)(-tq);

        /* 力矩超阈消抖 → 提前急停 (柔和接触, 不进驱动器堵转保护) */
        if (thresh > 0 && absTq >= thresh)
        {
            debounce++;
            if (debounce >= FLYBOX_HOME_TORQUE_DEBOUNCE)
            {
                Motor_EmergencyStop(deviceId);
                s_probeStopReason = 1U;  /* 力矩提前停 */
                s_state = FLYBOX_COMPLETE;
                return HAL_OK;
            }
        }
        else
        {
            debounce = 0U;
        }

        /* 驱动器堵转错误 → 备份停止判据 */
        if (p->lastErrorCode == MOTOR_ERR_STALL)
        {
            Motor_EmergencyStop(deviceId);
            s_probeStopReason = 2U;  /* 驱动器堵转 */
            s_state = FLYBOX_COMPLETE;
            return HAL_OK;
        }

        /* 行程走完判定: 电机已动过且剩余位移归零 */
        if (p->mainPosition != 0)
        {
            motorStarted = 1U;
        }
        else if (motorStarted && elapsed >= 300U)
        {
            s_probeStopReason = 3U;  /* 行程走完 */
            s_state = FLYBOX_COMPLETE;
            return HAL_OK;
        }

        osDelay(poll_ms);
        elapsed += poll_ms;
    }

    Motor_EmergencyStop(deviceId);
    s_probeStopReason = 4U;  /* 超时 */
    s_state = FLYBOX_ERROR;
    return HAL_TIMEOUT;
}

/**
  * @brief  【调试】一键力矩标定: 依次对电机 3/4/5 测空载基线 + 撞限位峰值
  *
  *         每个电机三步:
  *           1. 朝回零反方向移动一段 → 确保离开回零限位, 留出基线测量间隙
  *           2. 朝回零方向低速空载移动 → 记录力矩峰值 = baseline
  *           3. 朝回零方向全行程搜索, 用自适应阈值 (baseline×2+裕量) 提前停止
  *              → 记录力矩峰值 = limitPeak
  *
  *         自适应阈值原理: 撞限位时力矩会从基线快速上升, 在上升到
  *         baseline×2+裕量 时提前急停 → 柔和接触, 不会硬顶到驱动器
  *         堵转保护 (解除使能/力矩归零), 也避免冲击电流导致电源下电。
  *         torquePeak 在急停瞬间已捕获, 即为撞限位力矩参考值。
  *
  *         结果存 g_torque_calib, Watch 窗口读取:
  *           baseline[0..2] / limitPeak[0..2] / stopReason[0..2]
  *           step/currentId = 进度 (异常停止时定位用)
  *         阈值建议 = baseline + (limitPeak - baseline) / 2
  *
  *         注意: 标定前手动把机构置于行程中间附近, 箱体内不要有物体
  */
HAL_StatusTypeDef FlyBox_Debug_CalibrateTorque(void)
{
    static const uint8_t ids[3] = {
        FLYBOX_ID_GRIPPER, FLYBOX_ID_GRIPPER_ROTATE, FLYBOX_ID_GRIPPER_BELT
    };

    /* 清空上次结果 */
    for (uint8_t i = 0U; i < 3U; i++)
    {
        g_torque_calib.baseline[i]     = 0;
        g_torque_calib.limitPeak[i]    = 0;
        g_torque_calib.baselineDone[i] = 0U;
        g_torque_calib.limitDone[i]    = 0U;
        g_torque_calib.stopReason[i]   = 0U;
    }
    g_torque_calib.currentId = 0U;
    g_torque_calib.step      = 0U;

    const int16_t adaptiveMargin = 50;  /* 自适应阈值裕量 (力矩单位) */

    for (uint8_t i = 0U; i < 3U; i++)
    {
        uint8_t id = ids[i];
        g_torque_calib.currentId = id;

        int8_t  dir;
        float   search_mm;
        int16_t thresh;
        if (!FlyBox_GetHomeConfig(id, &dir, &search_mm, &thresh))
        {
            continue;
        }

        /* 1. 离开回零限位, 留出基线测量间隙 (无力矩停止, 堵转备份) */
        g_torque_calib.step = 1U;
        s_probeThreshOverride = -1;
        (void)FlyBox_Debug_TorqueProbe(id, (int8_t)-dir, FLYBOX_CALIB_CLEAR_MM, FLYBOX_CALIB_SPEED);
        osDelay(200);

        /* 2. 基线: 朝回零方向空载运动, 记录力矩峰值 (无力矩停止) */
        g_torque_calib.step = 2U;
        s_probeThreshOverride = -1;
        HAL_StatusTypeDef r = FlyBox_Debug_TorqueProbe(id, dir, FLYBOX_CALIB_BASE_MM, FLYBOX_CALIB_SPEED);
        if (r == HAL_OK)
        {
            g_torque_calib.baseline[i]     = g_motor_status[id - 1U].torquePeak;
            g_torque_calib.baselineDone[i] = 1U;
        }
        osDelay(200);

        /* 3. 撞限位: 自适应阈值提前停止, 记录力矩峰值 */
        g_torque_calib.step = 3U;
        {
            int16_t baseline = g_torque_calib.baseline[i];
            int16_t adaptive = (int16_t)(baseline * 2 + adaptiveMargin);
            if (adaptive < 30) adaptive = 30;  /* 下限保护: 基线异常小时仍有效 */
            s_probeThreshOverride = adaptive;
        }
        r = FlyBox_Debug_TorqueProbe(id, dir, search_mm, FLYBOX_CALIB_SPEED);
        g_torque_calib.stopReason[i] = s_probeStopReason;
        if (r == HAL_OK &&
            (s_probeStopReason == 1U || s_probeStopReason == 2U))
        {
            /* 力矩提前停/堵转停止 → 确实撞到了限位, 峰值有效 */
            g_torque_calib.limitPeak[i] = g_motor_status[id - 1U].torquePeak;
            g_torque_calib.limitDone[i] = 1U;
        }
        s_probeThreshOverride = -1;
        osDelay(500);
    }

    g_torque_calib.currentId = 0U;
    g_torque_calib.step      = 4U;  /* 全部完成 */
    return HAL_OK;
}

/**
  * @brief  获取当前状态
  */
FlyBox_State_t FlyBox_GetState(void)
{
    return s_state;
}
