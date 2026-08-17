/**
  ******************************************************************************
  * @file    can.c
  * @brief   CAN1 驱动实现，波特率 1Mbps
  *          引脚: PA11(RX) / PA12(TX)
  *
  *  波特率计算(PCLK1 = 36MHz):
  *    tq        = Prescaler / PCLK1 = 4 / 36MHz
  *    1 bit     = SYNC(1) + BS1(6TQ) + BS2(2TQ) = 9 TQ
  *    波特率     = 36MHz / 4 / 9 = 1 Mbps
  *    采样点     = (1 + 6) / 9 ≈ 77.8%
  ******************************************************************************
  */

#include "can.h"

/* CAN1 句柄 ----------------------------------------------------------------*/
static CAN_HandleTypeDef hcan1;

/* 最近一次接收到的帧 -------------------------------------------------------*/
volatile CAN_Frame_t g_can_rx_frame = {0};

/**
  * @brief  初始化 CAN1 并启动
  */
HAL_StatusTypeDef CAN_Init(uint32_t mode)
{
    GPIO_InitTypeDef       GPIO_InitStruct = {0};
    CAN_FilterTypeDef      sFilterConfig   = {0};
    HAL_StatusTypeDef      status;

    /* 使能时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* PA11(CAN_RX): 上拉输入 */
    GPIO_InitStruct.Pin  = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA12(CAN_TX): 复用推挽输出 */
    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN 外设配置: 1Mbps @ PCLK1=36MHz */
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 4;
    hcan1.Init.Mode                 = mode;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_6TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;   /* Bus-Off 后自动恢复 */
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    status = HAL_CAN_Init(&hcan1);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 过滤器: 全通，接收所有帧到 FIFO0 */
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    status = HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 启动 CAN */
    status = HAL_CAN_Start(&hcan1);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 使能 FIFO0 接收中断 */
    status = HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 配置 NVIC:
     * STM32F103C8(中密度)CAN1 与 USB 共享中断向量，
     * FIFO0 接收对应 USB_LP_CAN1_RX0_IRQn。
     * 优先级需不高于 configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(=5)，
     * 才能在 ISR 中安全调用 FreeRTOS API，故设为 5。 */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    return HAL_OK;
}

/**
  * @brief  发送一个标准数据帧
  */
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

    /* 若三个发送邮箱都满，等待其一空闲(简单轮询，避免丢帧) */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        return HAL_BUSY;
    }

    return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

/**
  * @brief  发送单个字节
  */
HAL_StatusTypeDef CAN_SendByte(uint8_t data)
{
    return CAN_SendFrame(CAN_TX_STD_ID, &data, 1U);
}

/**
  * @brief  收到新帧的用户回调(__weak 默认空实现，应用层可重写)
  */
__weak void CAN_OnFrameReceived(void)
{
    /* 应用层重写此函数，例如从 ISR 释放信号量唤醒接收任务 */
}

/**
  * @brief  HAL FIFO0 接收中断回调: 读取帧存入全局变量并通知应用层
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t             rxData[8];
    uint8_t             i;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        g_can_rx_frame.StdId = rxHeader.StdId;
        g_can_rx_frame.DLC   = (uint8_t)rxHeader.DLC;
        for (i = 0U; i < 8U; i++)
        {
            g_can_rx_frame.Data[i] = rxData[i];
        }
        g_can_rx_frame.NewFlag = 1U;

        CAN_OnFrameReceived();
    }
}

/**
  * @brief  CAN1 RX0 中断服务函数(与 USB 低优先级共享向量)
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}
