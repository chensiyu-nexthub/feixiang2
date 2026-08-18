/**
  ******************************************************************************
  * @file    can.c
  * @brief   CAN 总线驱动实现 (STM32F103 CAN1, 1Mbps)
  *
  *          硬件: PA11=CAN1_RX, PA12=CAN1_TX
  *          中断: USB_LP_CAN1_RX0_IRQn (与 USB 共享向量, 优先级 5)
  *          过滤器: 全通 (接收所有 ID 的帧), FIFO0
  ******************************************************************************
  */

#include "can.h"

/* CAN1 句柄 (HAL 库需要) */
CAN_HandleTypeDef hcan1;

/* 全局接收帧 */
volatile CAN_RxFrame_t g_can_rx_frame = {0};

/* ======================== 初始化 ======================== */

HAL_StatusTypeDef CAN_Init(uint8_t mode)
{
    HAL_StatusTypeDef status;

    /* 1. 使能 CAN1 时钟 */
    __HAL_RCC_CAN1_CLK_ENABLE();

    /* 2. 配置 GPIO: PA11(RX) 上拉输入, PA12(TX) 推挽复用 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Pin   = GPIO_PIN_12;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Pin   = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* 3. 配置 CAN 外设: 1Mbps, APB1=36MHz */
    hcan1.Instance = CAN1;
    hcan1.Init.Mode           = mode;
    hcan1.Init.AutoBusOff     = ENABLE;   /* 自动从 Bus-Off 恢复 */
    hcan1.Init.AutoWakeUp     = DISABLE;
    hcan1.Init.AutoRetransmission = ENABLE;  /* 硬件自动重发 */
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    /* 时序: 1Mbps = 36MHz / (1+3+5) / 4 — 预分频=4, BS1=3, BS2=5 */
    hcan1.Init.Prescaler      = 4;
    hcan1.Init.SyncJumpWidth  = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1       = CAN_BS1_3TQ;
    hcan1.Init.TimeSeg2       = CAN_BS2_5TQ;

    status = HAL_CAN_Init(&hcan1);
    if (status != HAL_OK) return status;

    /* 4. 配置接收过滤器: 全通 (接收所有 ID) */
    CAN_FilterTypeDef filter;
    filter.FilterActivation = ENABLE;
    filter.FilterBank       = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh     = 0x0000;
    filter.FilterIdLow      = 0x0000;
    filter.FilterMaskIdHigh = 0x0000;
    filter.FilterMaskIdLow  = 0x0000;
    filter.FilterMode       = CAN_FILTERMODE_IDMASK;
    filter.FilterScale      = CAN_FILTERSCALE_32BIT;
    filter.SlaveStartFilterBank = 14;

    status = HAL_CAN_ConfigFilter(&hcan1, &filter);
    if (status != HAL_OK) return status;

    /* 5. 启动 CAN */
    status = HAL_CAN_Start(&hcan1);
    if (status != HAL_OK) return status;

    /* 6. 使能 FIFO0 接收中断 */
    status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (status != HAL_OK) return status;

    /* 7. NVIC 配置: 优先级 5 (FreeRTOS 安全上限), 与 USB 共享向量 */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    return HAL_OK;
}

/* ======================== 发送 ======================== */

HAL_StatusTypeDef CAN_SendFrame(uint32_t stdId, const uint8_t *pData, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t            txMailbox;
    uint8_t             txData[8] = {0};
    uint8_t             i;

    if ((len > 8U) || ((pData == NULL) && (len > 0U)))
    {
        return HAL_ERROR;
    }

    for (i = 0U; i < len; i++)
    {
        txData[i] = pData[i];
    }

    txHeader.StdId              = stdId;
    txHeader.ExtId              = 0U;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = len;
    txHeader.TransmitGlobalTime = DISABLE;

    /* 若三个发送邮箱都满，返回 BUSY */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

/* ======================== 中断回调 ======================== */

__weak void CAN_OnFrameReceived(uint32_t stdId, const uint8_t *pData)
{
    (void)stdId;
    (void)pData;
    /* 应用层重写此函数, 如释放信号量唤醒接收任务 */
}

/**
  * @brief  从全局接收帧取出数据 (任务上下文调用)
  * @retval 1 = 有新帧, 0 = 无新帧
  */
uint8_t CAN_GetRxFrame(CAN_RxFrame_t *pFrame)
{
    if (pFrame == NULL) return 0U;
    if (g_can_rx_frame.newFlag == 0U) return 0U;

    pFrame->stdId = g_can_rx_frame.stdId;
    pFrame->dlc   = g_can_rx_frame.dlc;
    for (uint8_t i = 0U; i < 8U; i++)
    {
        pFrame->data[i] = g_can_rx_frame.data[i];
    }
    g_can_rx_frame.newFlag = 0U;
    return 1U;
}

/**
  * @brief  HAL FIFO0 接收中断回调: 读取帧存入全局变量, 通知应用层
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t             rxData[8];
    uint8_t             i;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        g_can_rx_frame.stdId = rxHeader.StdId;
        g_can_rx_frame.dlc   = (uint8_t)rxHeader.DLC;
        for (i = 0U; i < 8U; i++)
        {
            g_can_rx_frame.data[i] = rxData[i];
        }
        g_can_rx_frame.newFlag = 1U;

        CAN_OnFrameReceived(rxHeader.StdId, rxData);
    }
}

/**
  * @brief  CAN1 RX0 中断服务函数 (与 USB 低优先级共享向量)
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}