/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.2
  * @date    25-May-2015
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stm32f1xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include "spi.h"
#include "can.h"
#include "xv7001bb.h"
#include "timer.h"
#include "imu_core.h"
#include "can_protocol.h"
#include "tof8.h"
#include "motor.h"
#include "motion.h"
#include "flybox_action.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* CAN 工作模式: 独立调试波形用 CAN_MODE_LOOPBACK；接 USB-CAN 工具用 CAN_MODE_NORMAL */
#define CAN_WORK_MODE   CAN_MODE_NORMAL

/* 采样与发送解耦: 200Hz 采样, 50Hz 发送 */
#define SAMPLES_PER_SEND        4U       /* 每 4 个采样发 1 帧 */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId LEDThread1Handle;
osThreadId TaskMainHandle, TaskCanRxHandle;
osThreadId TaskMotorTestHandle;

/* CAN 接收信号量: FIFO0 中断中释放，接收任务中等待 */
static SemaphoreHandle_t g_can_rx_sem = NULL;

/* IMU 引擎实例 (封装零偏/死区/滤波/积分等所有状态) */
IMU_Engine_t g_engine;   /* 非 static: motion.c 通过 extern 访问做角度归零 */

/* CAN 命令标志 (Task_CanRx 设置, Task_Main 消费) */
static volatile uint8_t g_recalibrateRequest = 0; /* 1 = 请求重新校准 */
static volatile uint8_t g_angleZeroRequest = 0;   /* 1 = 请求角度置零 */

/* IMU 共享角度 (Task_Main 写, motion.c 读) */
volatile float g_imu_angle_deg = 0.0f;   /* 无界累加角度 (供 motion.c 转向校正) */
volatile float g_imu_rate_dps  = 0.0f;

/* LED 模式 (Task_MotorTest 设置, LED_Thread1 执行) */
/* 0=灭, 1=慢闪1s(直行), 2=快闪200ms(转弯), 3=常亮(异常) */
static volatile uint8_t g_led_mode = 0;

/* IMU 标定完成标志 (Task_Main 置1, Task_MotorTest 等待) */
static volatile uint8_t g_imu_ready = 0;

/* Private function prototypes -----------------------------------------------*/
static void LED_Thread1(void const *argument);
static void Task_Main(void const *argument);
static void Task_CanRx(void const *argument);
static void Task_MotorTest(void const *argument);
static void Task_Tof8(void const *argument);
static void Task_FlyBox(void const *argument);
static void FlyBox_ReportMotorFault(uint8_t deviceId);
void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, instruction and Data caches
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
	HAL_Init();  

	/* 配置系统时钟为 72MHz(HSE 8MHz × 9)，PCLK1=36MHz */
	SystemClock_Config();
	
	__GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_9;

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 初始化 SPI2(全双工主机)与 CAN1(1Mbps) 驱动 */
	SPI2_Init();
	CAN_Init(CAN_WORK_MODE);

	/* 创建 CAN 接收信号量(二值信号量) */
	g_can_rx_sem = xSemaphoreCreateBinary();

	/* Thread 1 definition */
	osThreadDef(LED1, LED_Thread1, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);

	/* CAN 接收任务与 TOF8 任务定义 */
	osThreadDef(TASKCANRX, Task_CanRx, osPriorityAboveNormal, 0, configMINIMAL_STACK_SIZE * 2);
	osThreadDef(TASKTOF8, Task_Tof8, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);

	/* 飞箱动作测试线程 */
	osThreadDef(TASKFLYBOX, Task_FlyBox, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);

	/* 陀螺仪线程 (角度标定需要) */
	osThreadDef(TASKMAIN, Task_Main, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
	// osThreadDef(TASKMOTOR, Task_MotorTest, osPriorityNormal, 0, configMINIMAL_STACK_SIZE * 2);
  
	/* Start thread 1 */
	LEDThread1Handle = osThreadCreate(osThread(LED1), NULL);

	/* 启动 CAN 接收任务与 TOF8 任务 */
	TaskCanRxHandle = osThreadCreate(osThread(TASKCANRX), NULL);
	osThreadCreate(osThread(TASKTOF8), NULL);

	/* 启动飞箱动作测试线程 */
	osThreadCreate(osThread(TASKFLYBOX), NULL);

	/* 启动陀螺仪线程 (角度标定需要) */
	TaskMainHandle = osThreadCreate(osThread(TASKMAIN), NULL);
	// TaskMotorTestHandle = osThreadCreate(osThread(TASKMOTOR), NULL);
  
	/* Start scheduler */
	osKernelStart();

	  /* We should never get here as control is now taken by the scheduler */
	for (;;)
		;
}

void SysTick_Handler(void)
{
	HAL_IncTick();
	osSystickHandler();
}

/**
  * @brief  系统时钟配置: HSE 8MHz，PLL ×9 → SYSCLK 72MHz
  *         AHB=72MHz, APB1(PCLK1)=36MHz, APB2(PCLK2)=72MHz
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/* 使能 HSE 并配置 PLL: HSE(8MHz) × 9 = 72MHz */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		while (1) { }
	}

	/* 配置总线时钟: SYSCLK=PLL(72MHz), AHB=72MHz, APB1=36MHz, APB2=72MHz
	 * Flash 等待周期: 72MHz 需 2 个等待状态 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
	                              RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		while (1) { }
	}
}

/**
  * @brief  CAN 收到新帧回调(中断上下文): 释放信号量唤醒接收任务
  * @retval None
  */
void CAN_OnFrameReceived(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (g_can_rx_sem != NULL)
	{
		xSemaphoreGiveFromISR(g_can_rx_sem, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}


/**
  * @brief  Task_CanRx: 等待 CAN 接收信号量，解析命令帧
  * @param  argument not used
  * @retval None
  */
static void Task_CanRx(void const *argument)
{
	(void) argument;

	for (;;)
	{
		/* 阻塞等待中断释放信号量 */
		if (xSemaphoreTake(g_can_rx_sem, portMAX_DELAY) == pdTRUE)
		{
			uint32_t rxId = g_can_rx_frame.StdId;

			/* 先尝试解析为 TOF8 响应/上报帧 (0x0409/0x0793/0x0794) */
			if (TOF8_ProcessRxFrame(rxId, (const uint8_t *)g_can_rx_frame.Data))
			{
				/* TOF8 帧已处理 */
			}
			/* 再尝试解析为电机反馈帧 */
			else if (Motor_ProcessRxFrame(rxId, (const uint8_t *)g_can_rx_frame.Data, g_can_rx_frame.DLC) != HAL_OK)
			{
				/* 非电机帧，解析 IMU 命令帧 */
				CAN_Command_t cmd = CAN_Proto_HandleRx(rxId);

				if (cmd == CAN_CMD_RECALIBRATE)
				{
					g_recalibrateRequest = 1;
				}
				else if (cmd == CAN_CMD_ANGLE_ZERO)
				{
					g_angleZeroRequest = 1;
				}
			}

			/* 清除新帧标志 */
			g_can_rx_frame.NewFlag = 0U;
		}
	}
}


/**
  * @brief  LED 指示线程 (由 g_led_mode 控制)
  *         0 = 灭
  *         1 = 慢闪 1s (直行)
  *         2 = 快闪 200ms (转弯)
  *         3 = 常亮 (异常/失败)
  *
  *         基础节拍 50ms, 用计数器实现不同闪烁周期,
  *         模式切换最多 50ms 内响应。
  */
static void LED_Thread1(void const *argument)
{
	(void) argument;
	uint16_t tick = 0;

	for (;;)
	{
		osDelay(50);
		tick++;

		switch (g_led_mode)
		{
			case 1:  /* 慢闪 1s: 每 20 tick 翻转 */
				if (tick % 20 == 0) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
				break;
			case 2:  /* 快闪 200ms: 每 4 tick 翻转 */
				if (tick % 2 == 0) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
				break;
			case 3:  /* 常亮 */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
				break;
			default: /* 灭 */
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
				break;
		}
	}
}



/**
  * @brief  task_main: XV7001BB 陀螺仪初始化与角度积分
  *
  *         数据处理链路 (由 IMU_Engine 封装):
  *           原始角速度 → ① 减零偏 → ② IIR 滤波 → ③ 中值滤波
  *                      → ④ 死区 → ⑤ 梯形积分 → ⑥ 角度归一化
  *
  *         CAN 帧格式 (由 CAN_Proto 封装):
  *           0x200 (DLC=5, 50Hz): 角度×100×4 + 角速度×100 + 温度
  *           0x201 (DLC=6, 50Hz): 原始角速度×100 + 原始角度增量×100×4 + 零偏×100
  *
  * @param  argument not used
  * @retval None
  */
static void Task_Main(void const *argument)
{
	(void) argument;

	HAL_StatusTypeDef status;
	int16_t  rawAngle;
	float    tempDegC;

	/* ===== 1. 初始化 XV7001BB ===== */
	status = XV7001bb_Init();
	(void) status;

	/* ===== 2. 初始化 IMU 引擎 ===== */
	IMU_Engine_Config_t engineCfg = {
		.iirFc            = 10.0f,         /* IIR 截止频率 10Hz */
		.iirFs            = 200.0f,        /* 采样频率 200Hz */
		.dtSec            = SAMPLE_DT_SEC, /* 采样周期 5ms */
		.deadZoneMin      = 0.1f,          /* 死区下限 0.1°/s */
		.deadZoneMax      = 0.15f,         /* 死区上限 0.15°/s */
		.deadZoneMargin   = 0.02f,         /* 死区余量 0.02°/s */
		.deadZoneMonCount = 400U,          /* 死区监测 400 次 (2s) */
		.angleScale       = 4.0f,          /* 真实角度 = 积分角度 × 4 */
	};
	IMU_Engine_Init(&g_engine, &engineCfg);

	/* ===== 3. 零偏校准: 静止状态下采样 400 次取平均 ===== */
	float bias;
	status = XV7001bb_CalibrateBias(400, &bias);
	if (status == HAL_OK)
	{
		IMU_Engine_SetBias(&g_engine, bias);
	}

	/* ===== 4. 初始化并启动 200Hz 采样定时器 ===== */
	status = TIM2_SampleTimer_Init();
	if (status != HAL_OK)
	{
		for (;;) { osDelay(1000); }
	}

	SemaphoreHandle_t sampleSem = TIM2_GetSampleSem();
	if (sampleSem == NULL)
	{
		for (;;) { osDelay(1000); }
	}

	TIM2_SampleTimer_Start();

	/* 标定完成, 通知 Task_MotorTest 可以开始 */
	g_imu_ready = 1;

	/* ===== 5. 主循环: 200Hz 采样 + 50Hz 发送 ===== */
	IMU_Engine_Output_t output;

	/* 发送解耦: 每 SAMPLES_PER_SEND 个采样发 1 帧 (200Hz→50Hz) */
	uint32_t sendCounter = 0;

	/* 发送周期内的累加值 (用于平均) */
	float sendRateSum     = 0.0f;
	float sendAngleSum    = 0.0f;
	float sendRawRateSum  = 0.0f;
	float sendRawAngleSum = 0.0f;

	for (;;)
	{
		/* 等待定时器中断释放信号量 (阻塞，精确 200Hz) */
		if (xSemaphoreTake(sampleSem, portMAX_DELAY) != pdTRUE)
		{
			continue;
		}

		/* 检测重新校准请求 */
		if (g_recalibrateRequest)
		{
			g_recalibrateRequest = 0;
			float newBias;
			status = XV7001bb_CalibrateBias(400, &newBias);
			if (status == HAL_OK)
			{
				IMU_Engine_SetBias(&g_engine, newBias);

				/* 自适应死区: 监测 2s 内滤波后角速度的最大绝对值 */
				float maxAbsRate = 0.0f;
				IMU_Engine_MonitorDeadZone(&g_engine, sampleSem, &maxAbsRate);
				IMU_Engine_UpdateDeadZone(&g_engine, maxAbsRate);

				/* 重置引擎状态，开始正式采样 */
				IMU_Engine_Reset(&g_engine);
			}
		}

		/* 检测角度置零请求 */
		if (g_angleZeroRequest)
		{
			g_angleZeroRequest = 0;
			IMU_Engine_ZeroAngle(&g_engine);
			g_imu_angle_deg = 0.0f;  /* 同步清零无界累加角度 */
		}

		/* 读取角速度 */
		status = XV7001bb_ReadAngle(&rawAngle);
		if (status != HAL_OK)
		{
			continue;
		}

		/* 数据处理: 去零偏 → IIR → 中值 → 死区 → 梯形积分 → 归一化 */
		IMU_Engine_Update(&g_engine, rawAngle, &output);

		/* 更新共享角度: 取反统一为电机约定 (左转正, 右转负)
		 * 陀螺仪物理方向与电机转向相反, 在源头统一 */
		g_imu_angle_deg = -output.angleDeg;
		g_imu_rate_dps  = -output.rateDps;

		/* 读取温度 */
		int16_t rawTemp;
		status = XV7001bb_ReadTemp(&rawTemp);
		if (status == HAL_OK)
		{
			tempDegC = XV7001BB_TEMP_REF +
			           ((float)rawTemp - XV7001BB_TEMP_OFFSET_12BIT) / XV7001BB_TEMP_COEFF_12BIT;
		}
		else
		{
			tempDegC = 0.0f;
		}

		/* 累加到发送缓冲区 */
		sendRateSum     += output.rateDps;
		sendAngleSum    += output.angleDeg;
		sendRawRateSum  += output.rawRateDps;
		sendRawAngleSum += output.rawAngleInc;
		sendCounter++;

		/* 每 SAMPLES_PER_SEND 个采样发送一次 (200Hz → 50Hz) */
		if (sendCounter >= SAMPLES_PER_SEND)
		{
			/* 取平均值 */
			IMU_Engine_Output_t avgOutput;
			avgOutput.rateDps     = sendRateSum     / (float)SAMPLES_PER_SEND;
			avgOutput.angleDeg    = sendAngleSum    / (float)SAMPLES_PER_SEND;
			avgOutput.rawRateDps  = sendRawRateSum  / (float)SAMPLES_PER_SEND;
			avgOutput.rawAngleInc = sendRawAngleSum / (float)SAMPLES_PER_SEND;

			/* 发送处理后数据帧 (0x200) 和诊断帧 (0x201) */
			CAN_Proto_SendData(&avgOutput, tempDegC);
			CAN_Proto_SendDiag(&avgOutput, IMU_Engine_GetBias(&g_engine));

			/* 重置累加器 */
			sendRateSum     = 0.0f;
			sendAngleSum    = 0.0f;
			sendRawRateSum  = 0.0f;
			sendRawAngleSum = 0.0f;
			sendCounter     = 0;
		}
	}
}


/**
  * @brief  Task_Tof8: TOF8 测距传感器任务
  *
  *         启动序列与轮询逻辑封装在 tof8.c 驱动内部:
  *           TOF8_StartAll()  → 复位 + 配置 + 开启连续测距
  *           TOF8_PollAll()   → 轮询所有设备读数
  *           TOF8_WatchdogCheck() → 超时自动复位重启
  *
  *         其他任务通过 TOF8_GetDistance() 或 g_tof8_status[] 获取距离
  *
  * @param  argument not used
  * @retval None
  */
static void Task_Tof8(void const *argument)
{
	(void) argument;

	/* 启动: 复位 → 配置 → 开启连续测距 */
	TOF8_StartAll();

	/* 主循环: 20Hz 轮询 + 看门狗 */
	for (;;)
	{
		TOF8_PollAll();
		TOF8_WatchdogCheck(2000);
		osDelay(50);
	}
}


/**
  * @brief  Task_MotorTest: 运动控制测试线程
  *
  *         LED 指示 (由 LED_Thread1 执行):
  *           慢闪 1s   = 直行中
  *           快闪 200ms = 转弯中
  *           常亮       = 异常/失败
  *           灭         = 全部完成
  * @param  argument not used
  * @retval None
  */
static void Task_MotorTest(void const *argument)
{
	(void) argument;
	HAL_StatusTypeDef status;

	/* 等待 IMU 标定完成 (Task_Main 置 g_imu_ready=1) */
	while (!g_imu_ready) { osDelay(100); }
	osDelay(500);  /* 额外等待 CAN RX 任务稳定 */

	/* ===== 1. Motion_Init: SN绑定 + 配置 + 锁定 (内部完成) ===== */
	status = Motion_Init(NULL);
	if (status != HAL_OK)
	{
		g_led_mode = 3;  /* 常亮: 初始化失败 */
		for (;;) { osDelay(1000); }
	}

	// float wb = 0;
	// Motion_CalibrateWheelBase(90.0f, 300.0f, 1000.0f, 4, &wb);


	/* ===== 标定 ===== */
	// status = Motion_Calibrate(MOTION_CALIB_LOAD_FLASH, 3000.0f, 800.0f, 1500.0f, 4);

	// /* ===== 直线来回测试: 5m × 10次, 每次 180° 掉头 (PID 纠偏) ===== */
	// for (int i = 0; i < 10; i++)
	// {
	// 	/* 前进 5m (带陀螺仪 PID 纠偏) */
	// 	g_led_mode = 1;  /* 慢闪: 直行 */
	// 	status = Motion_GoStraightCorrected(5000.0f, 1000.0f, 1000.0f);
	// 	// status = Motion_GoStraight(5000.0f, 1000.0f, 1000.0f);
	// 	if (status != HAL_OK) { g_led_mode = 3; for (;;) { osDelay(1000); } }
	// 	osDelay(1000);

	// 	/* 180° 掉头 */
	// 	g_led_mode = 2;  /* 快闪: 转弯 */
	// 	status = Motion_Turn(180.0f, 300.0f, 300.0f);
	// 	if (status != HAL_OK) { g_led_mode = 3; for (;;) { osDelay(1000); } }
	// 	osDelay(1000);
	// }


	/* ===== 正方形测试: 边长 2m, 顺时针 5 圈 ===== */
	for (int lap = 0; lap < 5; lap++)
	{
		for (int side = 0; side < 4; side++)
		{
			/* 直行 2m */
			g_led_mode = 1;
			// status = Motion_GoStraight(2000.0f, 1000.0f, 1000.0f);
			status = Motion_GoStraightCorrected(2000.0f, 1000.0f, 1000.0f);			
			if (status != HAL_OK) { g_led_mode = 3; for (;;) { osDelay(1000); } }
			osDelay(1000);

			/* 顺时针转 90° (右转 = 负角度) */
			g_led_mode = 2;
			status = Motion_Turn(-90.0f, 300.0f, 300.0f);
			if (status != HAL_OK) { g_led_mode = 3; for (;;) { osDelay(1000); } }
			osDelay(1000);
		}
	}

	// //陀螺仪校正循环测试
	// for (;;) 
	// { 
	// 	Motion_GyroCorrect(0.0f, 1000.0f, 1000.0f);
	// 	osDelay(1000); 
	// 	Motion_GyroCorrect(180.0f, 1000.0f, 1000.0f);
	// 	osDelay(1000);
	// }

	/* 全部完成: LED 灭 */
	g_led_mode = 0;
	for (;;) { osDelay(1000); }
}


/**
  * @brief  电机测试失败诊断: 读取该电机错误码, CAN 上报 + LED 闪一轮
  *         非阻塞: 闪一轮后返回, 测试循环继续下一个电机
  *         LED 闪烁规则: 先闪"设备号"次 → 长停顿 → 再闪"错误码低字节"次
  *         同时通过 CAN ID 0x2F0 上报: [设备号, 错误码高, 错误码低, 在线, 到位, 0,0,0]
  * @param  deviceId  失败的电机设备号
  */
static void FlyBox_ReportMotorFault(uint8_t deviceId)
{
	volatile Motor_Status_t *p = Motor_GetStatus(deviceId);
	uint16_t errCode = (p != NULL) ? p->lastErrorCode : 0xFFFFU;
	uint8_t  online  = (p != NULL) ? p->online : 0U;
	uint8_t  reached = (p != NULL) ? p->positionReached : 0U;

	/* CAN 上报诊断帧 */
	uint8_t data[8];
	data[0] = deviceId;
	data[1] = (uint8_t)(errCode >> 8);
	data[2] = (uint8_t)(errCode & 0xFF);
	data[3] = online;
	data[4] = reached;
	data[5] = 0; data[6] = 0; data[7] = 0;
	CAN_SendFrame(0x2F0U, data, 8U);

	/* LED 闪一轮: 设备号次 → 停顿 → 错误码低字节次 (非阻塞) */
	uint8_t errLow = (uint8_t)(errCode & 0xFF);
	if (errLow == 0U) errLow = 1U;

	/* 闪设备号次 */
	for (uint8_t n = 0; n < deviceId; n++)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		osDelay(200);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		osDelay(200);
	}
	osDelay(800);

	/* 闪错误码低字节次 */
	for (uint8_t n = 0; n < errLow; n++)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		osDelay(150);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		osDelay(150);
	}
	osDelay(1000);  /* 短暂停顿后返回, 继续测试下一个电机 */
}


/* ===== 力矩标定模式开关 (1=标定, 0=正常 PickBox/PlaceBox 循环) =====
 * 标定流程:
 *   1. 手动把钩爪/旋转/皮带机构置于行程中间附近, 箱体内不要放物体
 *   2. 编译烧录 (FLYBOX_TORQUE_CALIB_ENABLE=1), 开机自动执行标定
 *   3. 标定完成后 LED 快闪, 任务停在等待循环
 *   4. Watch 窗口读 g_torque_calib:
 *        baseline[0..2]  = 电机 3/4/5 空载力矩峰值
 *        limitPeak[0..2] = 电机 3/4/5 撞限位力矩峰值
 *   5. 阈值 = baseline + (limitPeak - baseline) / 2, 填入
 *      flybox_action.h 的 FLYBOX_HOME_TORQUE_THRESH_xxx,
 *      并把本宏改回 0 重新编译 → 开机自动力矩回零 */
#define FLYBOX_TORQUE_CALIB_ENABLE  0

/**
  * @brief  Task_FlyBox: 飞箱电机标定测试线程
  *
  *         流程:
  *           1. FlyBox_Init(): SN 绑定 + 5 电机配置 + 锁定
  *           2. 逐个电机往返测试 (FlyBox_TestMotor), 人工测量实际距离
  *           3. 用 FlyBox_CalibrateDiameter 计算修正轮径, 填回宏
  *
  *         LED 指示 (由 LED_Thread1 执行):
  *           慢闪 1s = 电机运行中
  *           常亮    = 初始化/运行失败
  *           灭      = 全部完成
  *
  *         标定方法:
  *           指令固定距离 (如 100mm) → 尺测实际距离 →
  *           修正轮径 = 当前轮径 × (实测 / 指令) → 填回 FLYBOX_DIA_xxx
  * @param  argument not used
  * @retval None
  */
static void Task_FlyBox(void const *argument)
{
	(void) argument;
	HAL_StatusTypeDef status;

	osDelay(1000);  /* 等待 CAN RX / TOF8 任务稳定 */

	/* ===== 1. 初始化: SN 绑定 + 配置 + 锁定 ===== */
	status = FlyBox_Init();
	if (status != HAL_OK)
	{
#if FLYBOX_TORQUE_CALIB_ENABLE
		/* 标定模式: 归零失败 (diag=4, 阈值未配置属预期) 不阻断, 继续标定;
		 * 其他失败 (1=SN超时 2=SN匹配 3=配置失败) 仍需 LED 报错 */
		if (g_flybox_init_diag != 4U)
#endif
		{
			/* LED 闪烁指示诊断码: 闪 N 次 = g_flybox_init_diag
			 *   1 = SN 收集超时 (电机未上电/接线/设备类型不对)
			 *   2 = SN 匹配不足 (收到的 SN 与硬编码不符)
			 *   3 = 电机配置失败 */
			for (;;)
			{
				for (uint8_t n = 0; n < g_flybox_init_diag; n++)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);  /* 亮 */
					osDelay(300);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);    /* 灭 */
					osDelay(300);
				}
				osDelay(1500);  /* 间隔, 便于计数 */
			}
		}
	}

#if FLYBOX_TORQUE_CALIB_ENABLE
	FlyBox_Debug_CalibrateTorque();

	/* 标定完成: LED 快闪提示, 停在等待循环供 Watch 窗口读取 g_torque_calib */
	for (;;)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		osDelay(100);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		osDelay(100);
	}
#else
	//  FlyBox_GripperTurnDeg(-80.0f, 300.0f, 300.0f); 
	//  FlyBox_PickBox();
	// FlyBox_GripperBeltMove(100.0f, 300.0f, 300.0f);
	// osDelay(200);
	// FlyBox_GripperBelttoHome();
	for(;;)
	{
		g_led_mode = 1;
		status = FlyBox_PickBox();
		if (status != HAL_OK)
		{
			/* 取箱失败: LED 常亮指示, 查看 g_pick_diag.lastStep/s1~s6 定位失败步骤 */
			g_led_mode = 3;
			osDelay(2000);
			continue;
		}
		osDelay(2000);
		// FlyBox_GripperRotateAbs(0.0f, FLYBOX_DEFAULT_SPEED, FLYBOX_DEFAULT_ACCEL);
		g_led_mode = 2;
		status = FlyBox_PlaceBox();
		if (status != HAL_OK)
		{
			/* 放箱失败: LED 常亮指示 */
			g_led_mode = 3;
			osDelay(2000);
			continue;
		}
		osDelay(500);
	}
#endif
}




#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
	while (1)
	{
	}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
