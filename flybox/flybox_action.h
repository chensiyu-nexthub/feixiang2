/**
  ******************************************************************************
  * @file    flybox_action.h
  * @brief   飞箱动作模块接口
  *
  *          管理 5 个电机 + 2 个 TOF 传感器, 实现取箱 / 放箱自动化流程:
  *            - 电机 SN 绑定与初始化 (复用 motor.c)
  *            - 传送带左右同步控制
  *            - 钩爪钩住 / 松开 / 旋转
  *            - 钩爪传送带前后移动
  *            - TOF25 (正面) / TOF26 (侧面) 距离联动
  *
  *          依赖: motor.c, tof8.c, FreeRTOS
  ******************************************************************************
  */

#ifndef __FLYBOX_ACTION_H
#define __FLYBOX_ACTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

/* ======================== 电机设备号分配 ======================== */
#define FLYBOX_ID_CONVEYOR_L        1U   /* 传送带左 */
#define FLYBOX_ID_CONVEYOR_R        2U   /* 传送带右 */
#define FLYBOX_ID_GRIPPER           3U   /* 钩爪 (钩住/松开) */
#define FLYBOX_ID_GRIPPER_ROTATE    4U   /* 钩爪旋转 (正面/侧面) */
#define FLYBOX_ID_GRIPPER_BELT      5U   /* 钩爪传送带 (前后移动) */

#define FLYBOX_MOTOR_COUNT          5U   /* 电机总数 */

/* ======================== 电机 SN 号 (7字节) ======================== */
#define FLYBOX_SN_CONVEYOR_L        {0x17, 0x05, 0x0A, 0x07, 0x52, 0xB8, 0x00}
#define FLYBOX_SN_CONVEYOR_R        {0x17, 0x05, 0x04, 0x07, 0xF5, 0xAE, 0x00}
#define FLYBOX_SN_GRIPPER           {0x17, 0x06, 0x1D, 0x07, 0xEE, 0xF9, 0x00}
#define FLYBOX_SN_GRIPPER_ROTATE    {0x17, 0x06, 0x1E, 0x07, 0x91, 0xFC, 0x00}
#define FLYBOX_SN_GRIPPER_BELT      {0x18, 0x03, 0x1C, 0x07, 0xE3, 0xFD, 0x00}

/* ======================== 每电机独立轮径 (已标定) ======================== */
/* 各电机机构/减速比不同, 轮径需分别标定:
 *   修正轮径 = 当前轮径 × (实测距离 / 指令距离)
 *   传送带: 25.55 × (125/100) = 31.94mm
 *   钩爪传送带: 25.55 × (25/100) = 6.39mm */
#define FLYBOX_DIA_CONVEYOR_L       31.94f   /* 传送带左 等效轮径 (mm) */
#define FLYBOX_DIA_CONVEYOR_R       31.94f   /* 传送带右 等效轮径 (mm) */
/* 钩爪(3)/旋转(4) 是纯旋转机构, "mm" 为虚拟单位, 轮径保持 25.55 不可改:
 * mm/° 换算 (FLYBOX_GRIPPER_MM_PER_DEG 等) 是在轮径=25.55 下标定的,
 * 改轮径会导致所有角度动作按比例偏差 (见下方"钩爪角度换算"注释) */
#define FLYBOX_DIA_GRIPPER          25.55f   /* 钩爪 等效轮径 (mm, mm/°标定基准, 勿改) */
#define FLYBOX_DIA_GRIPPER_ROTATE   25.55f   /* 钩爪旋转 等效轮径 (mm, mm/°标定基准, 勿改) */
#define FLYBOX_DIA_GRIPPER_BELT     6.39f    /* 钩爪传送带 等效轮径 (mm) */

/* ======================== 默认运动参数 ======================== */
#define FLYBOX_DEFAULT_SPEED        1000.0f   /* 默认速度 (mm/s, 调试阶段低速) */
#define FLYBOX_DEFAULT_ACCEL        1500.0f   /* 默认加速度 (mm/s², 调试阶段低速) */

/* ======================== 归零专用运动参数 (低速防碰撞) ======================== */
#define FLYBOX_HOME_SPEED           150.0f   /* 归零速度 (mm/s) */
#define FLYBOX_HOME_ACCEL           300.0f   /* 归零加速度 (mm/s²) */

/* 搜索阶段用更低速度: 150mm/s 撞限位力矩过冲到 300 (阈值 90),
 * 驱动器进入内部堵转保护后不响应回退指令 (failReason=3);
 * 30mm/s 标定实测峰值仅 148, 柔和接触不进保护 */
#define FLYBOX_HOME_SEARCH_SPEED    30.0f    /* 回零搜索速度 (mm/s, 撞限位阶段) */
#define FLYBOX_HOME_SEARCH_ACCEL    100.0f   /* 回零搜索加速度 (mm/s²) */

/* ======================== 力矩回零参数 (撞机械限位定位零点) ======================== */
/* 原理: 低速朝机械限位方向相对运动 → |力矩| 连续超阈判定到限位 → 急停 →
 *       回退一小段 → 发 0x0A 把当前位置设为零点。
 * 力矩单位协议未定义, 阈值需实测标定 (Watch 窗口观察 torque/torquePeak)。 */

/* 回零方向: +1=正方向相对运动, -1=负方向 (已确认: 3正/4负/5负) */
#define FLYBOX_HOME_DIR_GRIPPER         (+1)   /* 钩爪(3): 正方向 = 钩爪未抓(张开)方向 */
#define FLYBOX_HOME_DIR_GRIPPER_ROTATE  (-1)   /* 旋转(4): 负方向 = 转回正面 */
#define FLYBOX_HOME_DIR_GRIPPER_BELT    (-1)   /* 皮带(5): 负方向 = 皮带缩回 */

/* 搜索行程 (mm): 必须大于机构全行程, 保证从任意位置出发一定撞上限位。
 * 全行程 = 机械限位到机构最远端的距离 (实测 2026-08-04):
 *   钩爪: 全行程 ≈ 255° × 11.236 ≈ 2865mm
 *   旋转: 全行程 ≈ 355° × 7.955 ≈ 2824mm
 *   皮带: 限位→放箱推出 550mm (限位即旧零点) */
#define FLYBOX_HOME_SEARCH_MM_GRIPPER       3000.0f /* 钩爪全行程 ≈2865mm + 裕量 */
#define FLYBOX_HOME_SEARCH_MM_GRIPPER_ROTATE 3000.0f /* 旋转全行程 ≈2824mm + 裕量 */
#define FLYBOX_HOME_SEARCH_MM_GRIPPER_BELT  700.0f  /* 皮带全行程 >550mm */

/* 力矩阈值: |torque| ≥ 阈值 判定到限位。
 * 0 = 标定模式: 回零直接报错(未配置阈值), 探测函数不因力矩提前停。
 * 实测标定值 (2026-08-03, 标定速度 30mm/s):
 *   电机3 钩爪: baseline=24,  撞限位峰值=148 → 阈值 90  (中点法 86, 取整留裕量)
 *   电机4 旋转: baseline=27,  撞限位峰值=145 → 阈值 90  (中点法 86)
 *   电机5 皮带: baseline=41,  撞限位峰值=654 → 阈值 350 (中点法 347, 小轮径大力矩属正常) */
#define FLYBOX_HOME_TORQUE_THRESH_GRIPPER       90
#define FLYBOX_HOME_TORQUE_THRESH_GRIPPER_ROTATE 90
#define FLYBOX_HOME_TORQUE_THRESH_GRIPPER_BELT  350

#define FLYBOX_HOME_TORQUE_DEBOUNCE 3U      /* 连续超阈轮询次数 (10ms/次, 30ms 消抖, 低速搜索下加快检测减少顶推) */
#define FLYBOX_HOME_BACKOFF_MM      2.0f    /* 到限位后回退距离 (mm, 减少限位持续受力) */
#define FLYBOX_HOME_TIMEOUT_MS      150000U /* 单电机回零超时 (ms): 最长搜索 3000mm@30mm/s=100s, 留裕量 */

/* ======================== 卡滞脱困参数 (限位/卡死挣脱) ======================== */
/* 探测到卡滞 (电机未动但力矩超阈) 后, 尝试双向探索 + 多级脱困:
 *   - 级1: 正向探索 30mm (同搜索方向, 远离错误侧, 卡滞最常见)
 *   - 级2: 反方向回退 30mm (远离搜索方向, 针对正确侧卡死)
 *   - 级3: 摇摆 ±20/60/100mm 递增
 *   - 级4: 电流限制 0xFF + 摇摆后恢复配置
 * 成功判据 = 力矩回落持续 < 阈值 (200ms 稳定等待) */
#define FLYBOX_HOME_FREE_MM           30.0f    /* 脱困探索移动距离 (mm) */
#define FLYBOX_HOME_FREE_TIMEOUT_MS   3000U    /* 脱困探索超时 (ms) */
#define FLYBOX_HOME_FREE_SETTLE_MS    200U     /* 脱困后力矩回落稳定等待 (ms) */
#define FLYBOX_HOME_RETRY_MAX         2U       /* 脱困后重试搜索最大次数 */
#define FLYBOX_HOME_FREE_ROCK1_MM     20.0f    /* 摇摆脱困级1 (mm) */
#define FLYBOX_HOME_FREE_ROCK2_MM     60.0f    /* 摇摆脱困级2 (mm) */
#define FLYBOX_HOME_FREE_ROCK3_MM     100.0f   /* 摇摆脱困级3 (mm) */


/* ======================== 机械限位零点偏移 (实测 2026-08-04) ======================== */
/* 力矩回零把"机械限位位置"定义为驱动器零点, 与旧零点 (上电位置) 不同。
 * 偏移 = 机械限位在新坐标系的角度/位置, 绝对定位时自动换算:
 *   新目标 = 旧目标 - 偏移
 * 实测: 钩爪限位在旧零点+45°; 旋转限位在旧零点-45°; 皮带限位即旧零点 0 */
#define FLYBOX_HOME_OFFSET_GRIPPER_DEG        45.0f   /* 钩爪: 限位=旧零点+45° (限位→-45°=松开, -135°=钩取) */
#define FLYBOX_HOME_OFFSET_GRIPPER_ROTATE_DEG (-45.0f) /* 旋转: 限位=旧零点-45° (限位→+45°=正面, +135°=侧面) */
#define FLYBOX_HOME_OFFSET_GRIPPER_BELT_MM    0.0f    /* 皮带: 限位即旧零点, 无偏移 */

/* ======================== 力矩标定运动参数 (低速柔和撞击) ======================== */
/* 标定用比回零更低的速度/加速度: 撞击柔和, 给力矩上升留出时间,
 * 避免硬顶导致驱动器堵转保护(解除使能)或电源过流下电 */
#define FLYBOX_CALIB_SPEED          30.0f   /* 标定速度 (mm/s) */
#define FLYBOX_CALIB_ACCEL          100.0f  /* 标定加速度 (mm/s²) */
#define FLYBOX_CALIB_CLEAR_MM       80.0f   /* 离开回零限位的安全间隙 (mm) */
#define FLYBOX_CALIB_BASE_MM        50.0f   /* 基线测量距离 (mm, 空载) */
#define FLYBOX_CALIB_TIMEOUT_MS     15000U  /* 标定单次探测超时 (ms) */

/* ======================== 钩爪电机专用 PID 配置 (0x0316) ======================== */
/* 策略: P/I/D 全部保持默认, 仅 BYTE4 加阻尼抑制锁定后震荡响声
 * 调试记录:
 *   默认(无阻尼) → 能动力但响
 *   0x6A/0x7A/阻尼2 → 不响但不动 (P太低, 电机拒绝使能)
 *   默认+阻尼2 → 当前值, 待验证 */
#define FLYBOX_GRIPPER_CFG_BYTE0    0xB4U    /* 极对数=4 (同默认) */
#define FLYBOX_GRIPPER_CFG_BYTE1    0xBFU    /* 速度P=11, I=15 (同默认) */
#define FLYBOX_GRIPPER_CFG_BYTE2    0xCFU    /* 转矩P=12, I=15 (同默认) */
#define FLYBOX_GRIPPER_CFG_BYTE3    0xAAU    /* 电流限制=10 (同默认) */
#define FLYBOX_GRIPPER_CFG_BYTE4    0x03U    /* 刹车关闭, 阻尼=2 (默认 0x00: 无阻尼) */
#define FLYBOX_GRIPPER_CFG_BYTE5    0x5FU    /* 速度D=5, 转矩D=15 (同默认) */

/* ======================== 钩爪角度换算 (已标定) ======================== */
/* 标定方法: 驱动器轮径保持 25.55 不变, 指令 100mm, 陀螺仪测真实角度,
 *           mm/度 = 100 / 实测角度 (此 mm 为 25.55 轮径下的虚拟单位)
 *   钩爪旋转 (4): 100mm → 12.57° → 7.955 mm/°
 *   钩爪     (3): 100mm →  8.90° → 11.236 mm/°
 * 注意: 与 FLYBOX_DIA_GRIPPER/_ROTATE 绑定, 二者必须同时保持, 改一处另一处失效 */
#define FLYBOX_GRIPPER_MM_PER_DEG       11.236f  /* 钩爪电机: mm/度 */
#define FLYBOX_GRIPPER_ROTATE_MM_PER_DEG 7.955f  /* 钩爪旋转电机: mm/度 */

/* 角度 → mm 行程换算宏 */
#define FLYBOX_GRIPPER_DEG_TO_MM(deg)       ((deg) * FLYBOX_GRIPPER_MM_PER_DEG)
#define FLYBOX_GRIPPER_ROTATE_DEG_TO_MM(deg) ((deg) * FLYBOX_GRIPPER_ROTATE_MM_PER_DEG)

/* ======================== 钩爪动作角度 (占位, 待调整) ======================== */
#define FLYBOX_GRIPPER_HOOK_DEG         (-90.0f) /* 钩爪钩住角度 (°, 绝对定位) */
#define FLYBOX_GRIPPER_RELEASE_DEG      0.0f    /* 钩爪松开角度 (°) */
#define FLYBOX_GRIPPER_ROTATE_DEG       90.0f   /* 钩爪旋转角度 (正面→侧面, °) */

/* ======================== TOF 联动参数 ======================== */
#define FLYBOX_TOF25_HOOK_THRESH_MM     95.0f  /* tof25 正面: 到达此距离触发钩爪 (暂定) */
#define FLYBOX_TOF26_STOP_THRESH_MM     50.0f   /* tof26 侧面: 到达此距离停止传送带 (暂定) */
#define FLYBOX_TOF_ALIVE_TIMEOUT_MS     2000U   /* TOF 无数据超时: 超过此时间无更新视为故障 (ms) */
#define FLYBOX_TOF_RESTART_RETRY        2U      /* TOF 故障重启最大重试次数 */
#define FLYBOX_TOF_MAX_ITER             15U     /* 迭代逼近最大次数 (防死循环, 含滤波器预热等待) */
#define FLYBOX_TOF_SETTLE_MS            500U    /* 每次位置移动后等待稳定再测距 (ms) */
#define FLYBOX_TOF_CONVERGE_MM          5.0f    /* 收敛判定: 剩余距离 ≤ 此值即完成 (mm) */
#define FLYBOX_TOF_SEARCH_STEP_MM       50.0f   /* 目标超出测距范围时的盲进步长 (mm) */
#define FLYBOX_TOF_MEDIAN_SAMPLES       5U      /* 中值滤波采样次数 */
#define FLYBOX_TOF_MEDIAN_INTERVAL_MS   30U     /* 中值滤波采样间隔 (ms) */

/* ======================== 钩箱判定 / 重试 / 钩爪带限程参数 ======================== */
/* PickBox Step3 后用 TOF25 判定箱体是否被拉上传送带:
 *   成功时 TOF25 仍能看到箱体, 距离约 100mm 或更小
 *   判定失败 → 松爪 → 回到 Step1 重试, 最多 FLYBOX_PICK_MAX_ATTEMPTS 次
 * Step6 (TOF26 靠近) 迭代超限 (箱体卡住) 也可恢复:
 *   清障转正 (测距≥215mm 直接转正, 否则传送带推远到 215mm 再转正) → 回 Step1 重钩,
 *   与 Step3.5 重试共用 attempt 预算 (单一循环, 单一 abort 出口) */
#define FLYBOX_TOF25_VERIFY_MAX_MM      150.0f  /* Step3 后 TOF25 ≤ 此值 = 箱体已上传送带 (待实测调) */
#define FLYBOX_PICK_MAX_ATTEMPTS        3U      /* 取箱总尝试次数 (含首次, Step3.5/Step6 共用) */
#define FLYBOX_BELT_MAX_FWD_MM          500.0f  /* 钩爪带 TOF25 联动总前进上限 (盲进+逼近, 全行程≈550~700mm) */

/* ======================== PickBox / PlaceBox 距离参数 (占位, 待实测) ======================== */
#define FLYBOX_PICK_BELT_RETRACT_MM     100.0f  /* 取箱: 钩爪带后退距离 (mm) */
#define FLYBOX_PICK_CONVEYOR_FWD_MM     150.0f  /* 取箱: 传送带前进距离 (mm) */
#define FLYBOX_PLACE_CONVEYOR_PUSH_MM   30.0f   /* 放箱: 传送带推箱让位距离 (mm) */
#define FLYBOX_PLACE_BELT_FWD_MM        500.0f  /* 放箱: 钩爪带前进推箱距离 (mm, 绝对定位) */
#define FLYBOX_PLACE_CONVEYOR_BACK_MM   550.0f  /* 放箱: 传送带后退距离 (mm) */
#define FLYBOX_PLACE_TOF26_TARGET_MM    215.0f  /* 放箱: 推箱后 TOF26 目标距离 (mm, 远离方向) */

/* ======================== 步骤衔接延时 (动作流畅性调参) ======================== */
/* 每步已由电机到位反馈确认停稳, 此延时仅给机构/箱体留最小稳定时间:
 *   普通步骤 30ms; 旋转 90° 后 100ms (防余振扫到箱体); 超限回零重试 500ms */
#define FLYBOX_STEP_SETTLE_MS       30U     /* 普通步骤衔接延时 (ms) */
#define FLYBOX_ROTATE_SETTLE_MS     100U    /* 旋转后稳定延时 (ms) */
#define FLYBOX_RETRY_SETTLE_MS      500U    /* 钩爪带回零后重试等待 (ms) */

/* ======================== 旋转同步送箱/让位 (取箱 Step5+6 / 放箱 Step1+2) ======================== */
/* 物理模型: 钩爪 = 半径 215mm 扫掠 90°; 箱体前沿距旋转圆心 d 必须 ≥ 215·cosθ + 余量
 * (θ = 钩爪当前角度; 取箱 θ 0°→90° 箱体 215→~65, 放箱 θ 90°→0° 箱体 ~65→230) */
#define FLYBOX_HOOK_RADIUS_MM       215.0f   /* 钩爪扫掠半径 (mm) */
#define FLYBOX_HOOK_FINAL_MM        50.0f    /* 取箱同步段末端 (TOF26 读数下限, mm) */
#define FLYBOX_HOOK_MARGIN_MM       15.0f    /* 箱体与扫掠边界安全余量 (mm) */
#define FLYBOX_SYNC_STEP_MM         20.0f    /* 联动步进触发粒度 (mm) */
#define FLYBOX_ROTATE_SPEED         1000.0f  /* 旋转电机行程速度 (mm/s, 原 700) */
#define FLYBOX_ROTATE_ACCEL         2000.0f  /* 旋转电机加速度 (mm/s², 原 1400) */
#define FLYBOX_PLACE_SYNC_RETREAT_MM 180.0f  /* 放箱同步后退距离 (mm, 230-50) */
#define FLYBOX_TOF25_PLACE_EXPECT_MM 100.0f  /* 放箱同步后 TOF25 期望读数 (mm, 同钩箱第一步) */
#define FLYBOX_PLACE_COMP_MAX_MM     30.0f   /* TOF25 偏差补偿上限 (mm) */

/* ======================== 超时参数 ======================== */
#define FLYBOX_SN_TIMEOUT_MS            3000U   /* SN 等待超时 (ms) */
#define FLYBOX_MOTOR_WAIT_TIMEOUT_MS    30000U  /* 电机到位超时 (ms) */
#define FLYBOX_TOF_TIMEOUT_MS           15000U  /* TOF 联动超时 (ms) */

/* 到位瞬间保护帧容差 (原始值, 0.1mm): 电机顶着机械限位停止的瞬间, 驱动器可能上报
 * 一帧保护 (堵转 0x0104 / 超温 0x0105 等)。若电机已运动且剩余位移 ≤ 此值,
 * 说明动作物理上已完成, 视为到位成功 (防 "动作完成却误报失败") */
#define FLYBOX_STALL_REACHED_TOL_RAW    30

/* ======================== 状态定义 ======================== */
typedef enum
{
    FLYBOX_IDLE = 0,
    FLYBOX_BUSY,
    FLYBOX_COMPLETE,
    FLYBOX_ERROR
} FlyBox_State_t;

/* ======================== 诊断变量 (初始化失败定位) ======================== */
/* g_flybox_init_diag: 0=未失败, 1=SN收集超时, 2=SN匹配不足, 3=电机配置失败 */
extern volatile uint8_t g_flybox_init_diag;
/* g_flybox_sn_received: SN 队列中实际收到的 SN 数量 */
extern volatile uint8_t g_flybox_sn_received;

/* ======================== TOF 联动诊断 (调试用, Watch 窗口观察) ======================== */
typedef struct
{
    float    firstDist;   /* 进入联动后第一次有效测距 (mm, 9999=始终未获得有效值) */
    float    exitDist;    /* 退出前最后一次有效测距 (mm) */
    float    target;      /* 目标距离 (mm) */
    int8_t   direction;   /* +1=靠近(距离减小), -1=远离(距离增大) */
    uint8_t  exitReason;  /* 0=收敛 1=迭代超时 2=TOF故障 3=电机错误 4=入口TOF检查失败 5=远离方向无目标(拒绝盲进) */
    uint8_t  iters;       /* 已执行的迭代次数 */
    uint8_t  warmWaits;   /* 滤波器预热等待次数 */
    uint8_t  blindSteps;  /* 盲进次数 */
} FlyBox_TofDiag_t;

extern volatile FlyBox_TofDiag_t g_tof26_diag;

/* ======================== PickBox 步骤诊断 (调试用) ======================== */
/* 每步状态值 = HAL_StatusTypeDef: 0=OK, 1=ERROR, 2=BUSY, 3=TIMEOUT, 0xFF=未执行 */
typedef struct
{
    uint8_t lastStep;      /* 最后执行到的步骤 (1~6), 0=未进入流程 */
    uint8_t s1;            /* Step1 TOF25 联动 */
    uint8_t s2;            /* Step2 钩住 */
    uint8_t s3;            /* Step3 三电机同步到位 */
    uint8_t s4;            /* Step4 松开 */
    uint8_t s5;            /* Step5 旋转 90° */
    uint8_t s6;            /* Step6 TOF26 联动 */
    uint8_t verify;        /* Step3.5 TOF25 判定: 0xFF=未执行, 0=成功, 1=失败 */
    uint8_t attempts;      /* 实际尝试次数 (1~FLYBOX_PICK_MAX_ATTEMPTS) */
    uint8_t s6Recoveries;  /* Step6 超限恢复次数 */
    uint8_t s6Clear;       /* Step6 清障方式: 0xFF=未触发 1=直接转正 2=推远后转正 3=推远失败 */
    uint8_t beltLimitHit;  /* 钩爪带前进超限: 0xFF=未触发, 1=触发 (回零后重试) */
    float   verifyDist;    /* 最近一次 Step3.5 判定的 TOF25 距离 (mm, 9999=无效) */
    uint8_t syncSteps;     /* 同步段 (Step5+6) 联动推进次数 */
    float   syncEndMm;     /* 同步段结束箱体位置估计 (mm) */
    int32_t gripperPosAtAbort;   /* AbortAndSafe 时钩爪的 mainPosition (0.1mm, 调试用) */
    int16_t gripperTorqueAtAbort; /* AbortAndSafe 时钩爪的力矩 (调试用) */
    uint8_t abortReleaseResult;  /* AbortAndSafe 中松爪结果: 0=OK */
} FlyBox_PickDiag_t;

extern volatile FlyBox_PickDiag_t g_pick_diag;

/* ======================== 电机等待失败诊断 (调试用) ======================== */
/* 仅在 WaitMotorReached/WaitMultipleReached 返回失败时写入, 保留最近一次失败详情 */
typedef struct
{
    uint8_t  deviceId;       /* 失败的电机设备号 */
    uint8_t  result;         /* 返回值: 1=HAL_ERROR(致命错误), 3=HAL_TIMEOUT(超时) */
    uint16_t errCode;        /* 失败时的 lastErrorCode (0=无错误) */
    int32_t  mainPosition;   /* 失败时的剩余位移 (0.1mm, 0=已到位) */
    uint8_t  motorStarted;   /* 1=电机曾经开始运动 */
    uint32_t elapsedMs;      /* 失败时已等待时间 (ms) */
} FlyBox_WaitDiag_t;

extern volatile FlyBox_WaitDiag_t g_wait_diag;

/* ======================== 力矩回零诊断 (调试用, Watch 窗口观察) ======================== */
typedef struct
{
    uint8_t  deviceId;       /* 正在/最近回零的电机 */
    uint8_t  phase;          /* 0=空闲 1=搜索 2=到限位 3=回退 4=置零 5=完成 6=失败 */
    uint8_t  failReason;     /* 0=无 1=超时 2=致命错误 3=回退失败 4=置零失败 5=搜索行程走完未撞限位 6=阈值未配置 7=回退后仍同向顶住(未离开限位=无法探索, 疑似卡死在错误侧) 9=卡滞脱困全失败(机械卡死, 需人工处理) */
    uint8_t  zeroAtLimit;    /* 保留字段 (当前未使用); 不再允许在回退后力矩仍超阈时置零 */
    uint16_t errCode;        /* 失败时的 lastErrorCode */
    int16_t  torqueAtLimit;  /* 判定到限位时的力矩值 */
    int16_t  torquePeak;     /* 本次回零中的力矩峰值 */
    uint32_t searchMs;       /* 搜索阶段耗时 (ms) */
    uint32_t backoffElapsed; /* 回退阶段耗时 (ms, 超时值可判断卡在哪次等待) */
    int32_t  backoffPosition; /* 回退失败时的剩余位移 (0.1mm): 大值=驱动器未接受目标(保护态), ±20左右=接受但机械卡死 */
    int16_t  backoffTorque;  /* 回退后力矩: 应回落到基线附近; 仍≥阈值=未离开限位(方向配错/回退失败), 拒绝置零 */
    uint8_t  freeStage;      /* 脱困阶段: 0=无需脱困(正常流程) 1=正向(+dir)成功 2=回退(-dir)成功 9=全失败; FreeFromLimit内部: 11=级1 12=级2 13=级3摇摆 14=级4电流+摇摆 */
    uint8_t  retryCount;     /* 脱困后重试搜索次数 */
} FlyBox_HomeDiag_t;

extern volatile FlyBox_HomeDiag_t g_home_diag;

/* ======================== 力矩标定结果 (调试用, Watch 窗口读取) ======================== */
typedef struct
{
    int16_t  baseline[3];      /* 空载运动力矩峰值 (下标 0/1/2 = 电机 3/4/5) */
    int16_t  limitPeak[3];     /* 撞限位力矩峰值 (下标 0/1/2 = 电机 3/4/5) */
    uint8_t  baselineDone[3];  /* 基线测量完成标志 */
    uint8_t  limitDone[3];     /* 撞限位测量完成标志 */
    uint8_t  stopReason[3];    /* 限位探测停止方式: 0=未执行 1=力矩提前停 2=驱动器堵转 3=行程走完 4=超时 */
    uint8_t  currentId;        /* 当前正在标定的电机 (0=未开始/已完成) */
    uint8_t  step;             /* 当前步骤: 0=空闲 1=离开限位 2=基线 3=撞限位 4=全部完成 */
} FlyBox_TorqueCalib_t;

extern volatile FlyBox_TorqueCalib_t g_torque_calib;

/* ======================== 接口函数声明 ======================== */

/**
  * @brief  飞箱模块初始化: SN 绑定 + 5 电机配置 + 锁定
  * @retval HAL_OK 成功, HAL_ERROR/HAL_TIMEOUT 失败
  */
HAL_StatusTypeDef FlyBox_Init(void);

/**
  * @brief  全部归零: 依次对钩爪(3)/旋转(4)/传送带(5)执行力矩回零,
  *         完成后自动回到初始状态 (原零位: 钩爪松开/旋转正面/皮带回位)
  *         低速撞机械限位 → 回退 → 当前位置置零 (不依赖上电位置)
  *         归零后绝对定位 (Hook/Release/RotateAbs) 才有意义
  *         由 FlyBox_Init() 末尾自动调用, 也可运行中重新调用
  * @retval HAL_OK 成功, HAL_ERROR/HAL_TIMEOUT 失败
  */
HAL_StatusTypeDef FlyBox_HomeAll(void);

/**
  * @brief  回到初始状态 (原零位姿态): 需已完成归零
  *         各机构从机械限位位置回到工作初始姿态:
  *           钩爪→松开位(旧零位) / 旋转→正面(旧零位) / 皮带→回位(旧零位)
  *         由 FlyBox_HomeAll() 末尾自动调用, 避免从机械限位位置直接开始动作
  * @retval HAL_OK 成功, HAL_ERROR/HAL_TIMEOUT 失败
  */
HAL_StatusTypeDef FlyBox_GoInitState(void);

/**
  * @brief  检查回零力矩阈值是否已全部配置
  * @retval 1=全部已配置 (可正常回零), 0=标定模式 (存在阈值=0, FlyBox_Init 跳过回零)
  */
uint8_t FlyBox_HomeThresholdsReady(void);

/**
  * @brief  单电机力矩回零: 撞机械限位定位零点
  *         流程: 清错误/力矩统计 → 低速相对运动朝正确方向 (dir) 搜索 →
  *               |力矩|连续超阈判定候选到限位 →
  *               电机已动过 = 真实撞限位 → 反方向回退 FLYBOX_HOME_BACKOFF_MM 探索确认
  *               电机从未动 + 搜索<最短耗时 = 卡滞(错误侧/自锁) → 双向探索脱困+重试搜索
  *               驱动器堵转 0x0104 作备份判据 → 回退后力矩验证 → 发 0x0A 置零
  *         脱困流程: 回退(-dir) → 正向(+dir) → 多级脱困(摇摆/电流恢复) → 全失败 failReason=9
  * @param  deviceId  设备号 (3/4/5)
  * @retval HAL_OK 成功, HAL_ERROR 致命错误/卡滞脱困失败, HAL_TIMEOUT 搜索超时
  */
HAL_StatusTypeDef FlyBox_HomeMotorByTorque(uint8_t deviceId);

/**
  * @brief  【调试】力矩探测: 低速朝指定方向相对运动一段, 用于实测力矩基线/撞限位力矩
  *         用 Watch 窗口观察 g_motor_status[id-1].torque / torquePeak / torqueMin / torqueMax
  *         标定完成后把实测阈值填入 FLYBOX_HOME_TORQUE_THRESH_xxx
  * @param  deviceId    设备号
  * @param  dir         方向: +1=正方向, -1=负方向
  * @param  distance_mm 运动距离 (mm, 绝对值)
  * @param  speed_mm_s  速度 (mm/s, 建议低速 50~150)
  * @retval HAL_OK 运动完成或中途到限位停止
  */
HAL_StatusTypeDef FlyBox_Debug_TorqueProbe(uint8_t deviceId, int8_t dir,
                                           float distance_mm, float speed_mm_s);

/**
  * @brief  【调试】一键力矩标定: 依次对电机 3/4/5 测空载基线 + 撞限位峰值
  *         结果存 g_torque_calib, 用 Watch 窗口读取:
  *           baseline[i]  = 空载运动力矩峰值
  *           limitPeak[i] = 撞限位力矩峰值 (自适应阈值提前停止时捕获)
  *           stopReason[i] = 停止方式 (1=力矩提前停 2=驱动器堵转 3=行程走完 4=超时)
  *           step/currentId = 标定进度 (异常停止时定位用)
  *         撞限位采用自适应阈值 (基线×2+裕量) 提前停止, 柔和接触,
  *         避免驱动器堵转保护解除使能 / 硬顶冲击导致电源过流
  *         标定后阈值取 baseline + (limitPeak-baseline)/2, 填入
  *         FLYBOX_HOME_TORQUE_THRESH_xxx 后重新编译
  *         注意: 标定前手动把机构置于行程中间附近, 箱体内不要有物体
  * @retval HAL_OK 标定流程执行完毕 (单项失败仍继续, 检查 limitDone)
  */
HAL_StatusTypeDef FlyBox_Debug_CalibrateTorque(void);

/**
  * @brief  安全归位: 软停所有电机 (设定速为 0, 正常减速停止)
  *         用于 PickBox/PlaceBox 中途失败时的安全处理
  */
void FlyBox_AbortAndSafe(void);

/* ---------- 底层控制接口 ---------- */

/**
  * @brief  传送带左右同步运行指定距离
  * @param  distance_mm  距离 (mm, 正=前进, 负=后退)
  * @param  speed_mm_s   速度 (mm/s)
  * @param  accel_mm_s2  加速度 (mm/s²)
  */
HAL_StatusTypeDef FlyBox_ConveyorRun(float distance_mm, float speed_mm_s, float accel_mm_s2);

/**
  * @brief  传送带运行直到 tof26 (侧面) 距离到达 target_mm
  * @param  target_mm    目标距离 (mm)
  * @param  speed_mm_s   速度 (mm/s)
  * @param  accel_mm_s2  加速度 (mm/s²)
  * @param  direction    方向: +1=靠近(距离减小到target), -1=远离(距离增大到target)
  */
HAL_StatusTypeDef FlyBox_ConveyorRunUntilTof26(float target_mm, float speed_mm_s, float accel_mm_s2, int8_t direction);

/**
  * @brief  钩爪电机 (3) 相对旋转指定角度 (钩取动作, 调试用)
  *         相对位移: 从当前位置转 angle_deg, 不需要零点
  * @param  angle_deg    角度 (°, 正=钩住方向, 负=松开方向)
  * @param  speed_mm_s   速度 (mm/s)
  * @param  accel_mm_s2  加速度 (mm/s²)
  */
HAL_StatusTypeDef FlyBox_GripperTurnDeg(float angle_deg, float speed_mm_s, float accel_mm_s2);

/**
  * @brief  钩爪旋转电机 (4) 相对旋转指定角度 (整体结构旋转, 调试用)
  *         相对位移: 从当前位置转 angle_deg, 不需要零点
  * @param  angle_deg    角度 (°, 正=正面→侧面, 负=反向)
  * @param  speed_mm_s   速度 (mm/s)
  * @param  accel_mm_s2  加速度 (mm/s²)
  */
HAL_StatusTypeDef FlyBox_GripperRotateDeg(float angle_deg, float speed_mm_s, float accel_mm_s2);

/**
  * @brief  钩爪旋转电机 (4) 绝对旋转到指定角度 (需先归零)
  *         绝对定位: 0°=面对箱体(正面), 90°=侧面让位
  * @param  angle_deg    目标角度 (°, 0=正面, 90=侧面)
  * @param  speed_mm_s   速度 (mm/s)
  * @param  accel_mm_s2  加速度 (mm/s²)
  */
HAL_StatusTypeDef FlyBox_GripperRotateAbs(float angle_deg, float speed_mm_s, float accel_mm_s2);

/**
  * @brief  钩爪电机 (3) 归零: 力矩回零 (撞机械限位定位零点)
  *         归零后绝对位置才有意义, 生产阶段使用
  */
HAL_StatusTypeDef FlyBox_GripperHome(void);

/**
  * @brief  钩爪旋转电机 (4) 归零: 力矩回零
  */
HAL_StatusTypeDef FlyBox_GripperRotateHome(void);

/**
  * @brief  钩爪传送带 (5) 归零: 力矩回零
  */
HAL_StatusTypeDef FlyBox_GripperBeltHome(void);

/**
  * @brief  钩爪钩住箱体 (绝对位置, 需先 FlyBox_GripperHome 归零)
  */
HAL_StatusTypeDef FlyBox_GripperHook(void);

/**
  * @brief  钩爪松开箱体 (绝对位置, 需先 FlyBox_GripperHome 归零)
  */
HAL_StatusTypeDef FlyBox_GripperRelease(void);

/**
  * @brief  钩爪传送带归零 (绝对位置, 需先 FlyBox_GripperBeltHome 归零)
  */
HAL_StatusTypeDef FlyBox_GripperBelttoHome(void);

/**
  * @brief  钩爪传送带 (5) 前后移动指定距离
  * @param  distance_mm  距离 (mm, 正=前进靠近箱体, 负=后退)
  * @param  speed_mm_s   速度 (mm/s)
  * @param  accel_mm_s2  加速度 (mm/s²)
  */
HAL_StatusTypeDef FlyBox_GripperBeltMove(float distance_mm, float speed_mm_s, float accel_mm_s2);

/**
  * @brief  钩爪传送带前进直到 tof25 (正面) 距离 <= target_mm
  * @param  target_mm    目标距离 (mm)
  * @param  speed_mm_s   速度 (mm/s)
  * @param  accel_mm_s2  加速度 (mm/s²)
  */
HAL_StatusTypeDef FlyBox_GripperBeltMoveUntilTof25(float target_mm, float speed_mm_s, float accel_mm_s2);

/* ---------- 高层流程接口 ---------- */

/**
  * @brief  完整取箱流程 (6 步, 单一 attempt 循环 + 两处可重试失败点)
  *         入口安全前置: 松开 + 钩爪转正
  *         1. 钩爪传送带前进靠近箱体 (TOF25 联动, 前进超限→回零+2s→重试)
  *         2. 钩爪钩住 (绝对 -90°)
  *         3. 钩爪带归零 + 传送带 L/R 前进 (三电机同时) → 拉箱体上传送带
  *         3.5 TOF25 判定箱体是否上传送带, 失败→松爪→回 Step1 (重试点 A)
  *         4. 松开钩爪 (绝对 0°, 旋转前必须松)
  *         5. 旋转 90° 让位 (绝对 90°=侧面)
  *         6. 传送带送到指定位置 (TOF26 联动), HAL_TIMEOUT→清障转正→回 Step1 (重试点 B)
  *         Step5+6 合并: FlyBox_RotateSyncPick 同步旋转 + 传送带前进
  *         两处重试共用 FLYBOX_PICK_MAX_ATTEMPTS 预算, 单一 abort 出口
  */
HAL_StatusTypeDef FlyBox_PickBox(void);

/**
  * @brief  完整放箱流程 (5 步, 取箱逆序)
  *         Step1+2 合并: FlyBox_RotateSyncPlace 同步让位 (传送带后退 + 旋转回正面)
  *         2.5 TOF25 校验补偿, 修正传送带后退量
  *         3. 钩爪带前进 (绝对定位) + 传送带 L/R 后退 (修正量, 三电机同时) → 推出箱体
  *         4. 确认钩爪松开 (绝对 0°)
  *         5. 钩爪传送带归位 (绝对 0)
  */
HAL_StatusTypeDef FlyBox_PlaceBox(void);

/**
  * @brief  获取当前状态
  */
FlyBox_State_t FlyBox_GetState(void);

/* ---------- 标定测试接口 ---------- */

/**
  * @brief  单电机往返测试: 前进 dist → 停 → 后退 dist 归位
  *         用于验证电机到位 / 方向 / 错误检测
  * @param  deviceId    设备号 (1~5)
  * @param  dist_mm     单程距离 (mm)
  * @param  speed_mm_s  速度 (mm/s)
  * @param  accel_mm_s2 加速度 (mm/s²)
  */
HAL_StatusTypeDef FlyBox_TestMotor(uint8_t deviceId, float dist_mm,
                                   float speed_mm_s, float accel_mm_s2);

/**
  * @brief  计算轮径修正值
  *         修正轮径 = 当前轮径 × (实测距离 / 指令距离)
  * @param  deviceId      设备号 (用于读取当前配置轮径参考)
  * @param  currentDia_mm 当前配置轮径 (mm)
  * @param  cmdDist_mm    指令距离 (mm)
  * @param  measuredDist_mm 人工实测距离 (mm)
  * @retval 修正后的轮径 (mm), 应填回对应 FLYBOX_DIA_xxx 宏
  */
float FlyBox_CalibrateDiameter(uint8_t deviceId, float currentDia_mm,
                               float cmdDist_mm, float measuredDist_mm);

#ifdef __cplusplus
}
#endif

#endif /* __FLYBOX_ACTION_H */
