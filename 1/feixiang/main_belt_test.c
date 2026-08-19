/**
  ******************************************************************************
  * @file    main_belt_test.c
  * @brief   皮带电机 (电机5) 最简测试 — 裸机 HAL, 不用 FreeRTOS
  *
  *          用法: 把 main.c 重命名为 main_full.c, 把这个文件重命名为 main.c
  *          烧录后 LED 行为:
  *            闪3次 = 时钟OK
  *            闪1次 = CAN OK
  *            闪2次 = 电机锁定OK
  *            然后 LED 常亮, 皮带前后交替运动:
  *              前进100mm → 等2秒 → 后退100mm → 等2秒 → 循环
  ******************************************************************************
  */

#include <stm32f1xx_hal.h>

/* ======================== 常量 ======================== */
#define DEVICE_BELT         5U       /* 皮带电机设备号 */
#define BELT_WHEEL_DIA      6.37f    /* 皮带轮径 mm */
#define BELT_SPEED_MM_S     200.0f   /* 皮带速度 mm/s */
#define BELT_ACCEL_MM_S2    500.0f   /* 皮带加速度 mm/s² */
#define BELT_MOVE_MM        100.0f   /* 每次移动距离 mm */

/* ======================== 系统时钟配置 ======================== */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscInit = {0};
    RCC_ClkInitTypeDef clkInit = {0};

    oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscInit.HSEState       = RCC_HSE_ON;
    oscInit.PLL.PLLState   = RCC_PLL_ON;
    oscInit.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    oscInit.PLL.PLLMUL     = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&oscInit);

    clkInit.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                           | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clkInit.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clkInit.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clkInit.APB1CLKDivider = RCC_HCLK_DIV2;
    clkInit.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_2);
}

/* ======================== GPIO 初始化 (LED + CAN) ======================== */
static void GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();   /* CAN 引脚 PA11/PA12 */
    __HAL_RCC_AFIO_CLK_ENABLE();    /* AFIO 复用 */

    /* PC13 LED */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_13;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}

/* ======================== CAN 初始化 (1Mbps) ======================== */
static CAN_HandleTypeDef hcan;

static void CAN_Init(void)
{
    __HAL_RCC_CAN1_CLK_ENABLE();

    /* PA11=CAN_RX, PA12=CAN_TX */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio);

    hcan.Instance = CAN1;
    hcan.Init.Prescaler       = 4;            /* 36MHz / 4 = 9MHz */
    hcan.Init.Mode            = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth   = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1        = CAN_BS1_3TQ;   /* 1+3+5=9TQ, 9/9MHz=1μs → 1Mbps */
    hcan.Init.TimeSeg2        = CAN_BS2_5TQ;
    hcan.Init.TimeTriggeredMode  = DISABLE;
    hcan.Init.AutoBusOff         = DISABLE;
    hcan.Init.AutoWakeUp         = DISABLE;
    hcan.Init.AutoRetransmission = ENABLE;
    hcan.Init.ReceiveFifoLocked  = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        for (;;) { } /* 初始化失败, 死循环 */
    }

    /* 过滤器: 全通过 */
    CAN_FilterTypeDef filter = {0};
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0;
    filter.FilterIdLow          = 0;
    filter.FilterMaskIdHigh     = 0;
    filter.FilterMaskIdLow      = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.FilterBank           = 0;
    HAL_CAN_ConfigFilter(&hcan, &filter);

    /* 启动 CAN */
    HAL_CAN_Start(&hcan);
}

/* ======================== CAN 发送 ======================== */
static HAL_StatusTypeDef CAN_Send(uint32_t stdId, const uint8_t *pData, uint8_t len)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;

    header.StdId = stdId;
    header.IDE   = CAN_ID_STD;
    header.RTR   = CAN_RTR_DATA;
    header.DLC   = len;

    return HAL_CAN_AddTxMessage(&hcan, &header, (uint8_t *)pData, &mailbox);
}

/* ======================== 电机 CAN 命令 ======================== */

/* 打包 int32 → 4字节小端 */
static void PackI32(uint8_t *buf, int32_t val)
{
    buf[0] = (uint8_t)(val & 0xFF);
    buf[1] = (uint8_t)((val >> 8) & 0xFF);
    buf[2] = (uint8_t)((val >> 16) & 0xFF);
    buf[3] = (uint8_t)((val >> 24) & 0xFF);
}

/* 锁定电机 */
static void Motor_Lock(uint8_t devId)
{
    uint8_t d[2] = { devId, 1 };
    CAN_Send(0x0413, d, 2);
}

/* 解锁电机 */
static void Motor_Unlock(uint8_t devId)
{
    uint8_t d[2] = { devId, 0 };
    CAN_Send(0x0413, d, 2);
}

/* 设置速度 (mm/s) */
static void Motor_SetSpeed(uint8_t devId, float mm_s)
{
    uint8_t d[8] = {0};
    d[0] = devId;
    PackI32(&d[1], (int32_t)(mm_s * 10.0f));  /* 0.1mm/s 单位 */
    CAN_Send(0x0415, d, 5);
}

/* 设置加速度 (mm/s²) */
static void Motor_SetAccel(uint8_t devId, float mm_s2)
{
    uint8_t d[8] = {0};
    d[0] = devId;
    PackI32(&d[1], (int32_t)(mm_s2 * 10.0f));  /* 0.1mm/s² 单位 */
    CAN_Send(0x0414, d, 5);
}

/* 设置目标位置 (相对, mm) */
static void Motor_SetTarget(uint8_t devId, float mm)
{
    uint8_t d[8] = {0};
    d[0] = devId;
    PackI32(&d[1], (int32_t)(mm * 10.0f));  /* 0.1mm 单位 */
    CAN_Send(0x0416, d, 5);
}

/* 急停 */
static void Motor_Stop(uint8_t devId)
{
    uint8_t d[8] = {0};
    d[0] = devId;
    PackI32(&d[1], 0);
    CAN_Send(0x0416, d, 5);
}

/* ======================== LED 辅助 ======================== */
static void LED_On(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

static void LED_Off(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

static void LED_Blink(int count, uint32_t ms)
{
    for (int i = 0; i < count; i++)
    {
        LED_On();
        HAL_Delay(ms);
        LED_Off();
        HAL_Delay(ms);
    }
}

/* ======================== main ======================== */
int main(void)
{
    /* 1. HAL 初始化 */
    HAL_Init();

    /* 2. 时钟 */
    SystemClock_Config();

    /* 3. GPIO */
    GPIO_Init();

    /* 闪 3 次 = 时钟+GPIO OK */
    LED_Blink(3, 200);
    HAL_Delay(500);

    /* 4. CAN */
    CAN_Init();

    /* 闪 1 次 = CAN OK */
    LED_Blink(1, 300);
    HAL_Delay(500);

    /* 5. 锁定电机 (解锁再锁定, 确保状态正确) */
    Motor_Unlock(DEVICE_BELT);
    HAL_Delay(100);
    Motor_Lock(DEVICE_BELT);
    HAL_Delay(100);

    /* 闪 2 次 = 电机锁定OK */
    LED_Blink(2, 200);
    HAL_Delay(500);

    /* 6. 设置运动参数 */
    Motor_SetSpeed(DEVICE_BELT, BELT_SPEED_MM_S);
    HAL_Delay(10);
    Motor_SetAccel(DEVICE_BELT, BELT_ACCEL_MM_S2);
    HAL_Delay(10);

    /* LED 常亮 = 开始运动 */
    LED_On();

    /* 7. 主循环: 前进 → 后退 → 前进 → ... */
    for (;;)
    {
        /* 皮带前进 */
        Motor_SetTarget(DEVICE_BELT, BELT_MOVE_MM);
        HAL_Delay(2000);  /* 等 2 秒走完 */

        /* 皮带后退 */
        Motor_SetTarget(DEVICE_BELT, -BELT_MOVE_MM);
        HAL_Delay(2000);
    }
}

/* ======================== 中断处理 ======================== */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

void HardFault_Handler(void)
{
    /* 快速闪烁表示 HardFault */
    for (;;)
    {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        for (volatile uint32_t d = 0; d < 200000; d++) { }
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        for (volatile uint32_t d = 0; d < 200000; d++) { }
    }
}