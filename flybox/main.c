/**
  ******************************************************************************
  * @file    main.c
  * @brief   飞箱主程序入口 (STM32F103 + FreeRTOS)
  *
  *          5 任务架构:
  *            - Task_CanRx:   CAN 接收分发 (信号量同步 ISR → 任务)
  *            - Task_Tof8:    TOF8 轮询 + 看门狗
  *            - Task_FlyBox:  取箱/放箱 主循环
  *            - Task_LED:     LED 状态指示
  *            - Task_Idle:    FreeRTOS 空闲任务
  ******************************************************************************
  */

#include <stm32f1xx_hal.h>
#include <cmsis_os.h>
#include "can.h"
#include "motor.h"
#include "tof8.h"
#include "flybox_action.h"

/* ======================== 信号量 ======================== */
static osSemaphoreId s_semCanRx = NULL;

/* ======================== 系统时钟配置 ======================== */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscInit = {0};
    RCC_ClkInitTypeDef clkInit = {0};

    /* HSE 8MHz → PLL ×9 → 72MHz */
    oscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscInit.HSEState       = RCC_HSE_ON;
    oscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    oscInit.PLL.PLLState   = RCC_PLL_ON;
    oscInit.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    oscInit.PLL.PLLMUL     = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&oscInit);

    clkInit.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                           | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clkInit.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clkInit.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clkInit.APB1CLKDivider = RCC_HCLK_DIV2;   /* PCLK1 = 36MHz */
    clkInit.APB2CLKDivider = RCC_HCLK_DIV1;   /* PCLK2 = 72MHz */
    HAL_RCC_ClockConfig(&clkInit, FLASH_LATENCY_2);
}

/* ======================== GPIO 初始化 (LED) ======================== */
static void GPIO_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_13;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}

/* ======================== CAN 接收回调 (ISR 上下文中) ======================== */
void CAN_OnFrameReceived(uint32_t stdId, const uint8_t *pData)
{
    (void)stdId;
    (void)pData;

    /* 信号量通知 (ISR 安全) */
    if (s_semCanRx != NULL)
    {
        osSemaphoreRelease(s_semCanRx);
    }
}

/* ======================== Task_CanRx: CAN 接收分发 ======================== */
static void Task_CanRx(void const *arg)
{
    (void)arg;

    for (;;)
    {
        osSemaphoreWait(s_semCanRx, osWaitForever);

        /* 处理所有待处理帧 */
        CAN_RxFrame_t frame;
        while (CAN_GetRxFrame(&frame))
        {
            /* 先分发给 TOF8 (距离帧优先处理) */
            if (TOF8_ProcessRxFrame(frame.stdId, frame.data) == 0U)
            {
                /* 再分发给电机 */
                Motor_ProcessRxFrame(frame.stdId, frame.data);
            }
        }
    }
}

/* ======================== Task_Tof8: TOF8 轮询 + 看门狗 ======================== */
static void Task_Tof8(void const *arg)
{
    (void)arg;

    /* 等待初始化完成 */
    osDelay(2000);

    TOF8_StartAll();

    for (;;)
    {
        TOF8_PollAll();
        TOF8_WatchdogCheck(FLYBOX_TOF_ALIVE_TIMEOUT_MS);
        osDelay(50);
    }
}

/* ======================== Task_LED: LED 状态指示 ======================== */
static void Task_LED(void const *arg)
{
    (void)arg;

    for (;;)
    {
        FlyBox_State_t state = FlyBox_GetState();

        switch (state)
        {
        case FLYBOX_IDLE:
            /* 常亮 */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
            osDelay(1000);
            break;

        case FLYBOX_BUSY:
            /* 快速闪烁 100ms */
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            osDelay(100);
            break;

        case FLYBOX_COMPLETE:
            /* 慢闪 500ms */
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            osDelay(500);
            break;

        case FLYBOX_ERROR:
            /* 快速闪烁 200ms */
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            osDelay(200);
            break;
        }
    }
}

/* ======================== Task_FlyBox: 取箱/放箱 主循环 ======================== */
static void Task_FlyBox(void const *arg)
{
    (void)arg;

    osDelay(3000);  /* 等待 TOF8 初始化完成 */

    /* 初始化电机 */
    if (FlyBox_Init() != HAL_OK)
    {
        for (;;) { osDelay(1000); }
    }

    osDelay(500);

    /* 回零 */
    if (FlyBox_HomeAll() != HAL_OK)
    {
        for (;;) { osDelay(1000); }
    }

    osDelay(1000);

    /* 主循环: Pick → Place → Pick → Place ... */
    for (;;)
    {
        /* 取箱 */
        HAL_StatusTypeDef st = FlyBox_PickBox();
        if (st != HAL_OK)
        {
            /* 出错后等待一段时间再试 */
            osDelay(FLYBOX_RETRY_SETTLE_MS);
            continue;
        }

        osDelay(FLYBOX_STEP_SETTLE_MS);

        /* 放箱 */
        st = FlyBox_PlaceBox();
        if (st != HAL_OK)
        {
            osDelay(FLYBOX_RETRY_SETTLE_MS);
            continue;
        }

        osDelay(FLYBOX_RETRY_SETTLE_MS);
    }
}

/* ======================== 任务句柄 ======================== */
static osThreadId s_threadCanRx  = NULL;
static osThreadId s_threadTof8   = NULL;
static osThreadId s_threadFlyBox = NULL;
static osThreadId s_threadLED    = NULL;

/* ======================== main ======================== */
int main(void)
{
    /* HAL 初始化 */
    HAL_Init();

    /* 时钟配置 */
    SystemClock_Config();

    /* GPIO 初始化 */
    GPIO_Init();

    /* CAN 初始化 */
    CAN_Init();

    /* 创建信号量 */
    osSemaphoreDef(SEM_CAN_RX);
    s_semCanRx = osSemaphoreCreate(osSemaphore(SEM_CAN_RX), 0);

    /* 创建任务 */
    osThreadDef(TASK_CAN_RX,  Task_CanRx,  osPriorityHigh,   0, 512);
    osThreadDef(TASK_TOF8,    Task_Tof8,   osPriorityNormal, 0, 512);
    osThreadDef(TASK_FLYBOX,  Task_FlyBox, osPriorityNormal, 0, 1024);
    osThreadDef(TASK_LED,     Task_LED,    osPriorityLow,    0, 256);

    s_threadCanRx  = osThreadCreate(osThread(TASK_CAN_RX),  NULL);
    s_threadTof8   = osThreadCreate(osThread(TASK_TOF8),    NULL);
    s_threadFlyBox = osThreadCreate(osThread(TASK_FLYBOX),  NULL);
    s_threadLED    = osThreadCreate(osThread(TASK_LED),     NULL);

    /* 启动 FreeRTOS 调度器 */
    osKernelStart();

    /* 永远不会执行到这里 */
    for (;;) { }
}

/* ======================== HAL 回调 ======================== */

/**
  * @brief  SysTick 中断 (FreeRTOS 使用)
  */
void SysTick_Handler(void)
{
    HAL_IncTick();
    osSystickHandler();
}

/**
  * @brief  HardFault 处理
  */
void HardFault_Handler(void)
{
    /* 紧急停止所有电机 */
    for (uint8_t i = 1U; i <= FLYBOX_MOTOR_COUNT; i++)
    {
        Motor_EmergencyStop(i);
    }
    for (;;) { }
}