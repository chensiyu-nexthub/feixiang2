/**
  ******************************************************************************
  * @file    spi.c
  * @brief   SPI2 全双工主机驱动实现
  *          引脚: PB13(SCK) / PB14(MISO) / PB15(MOSI)，软件片选 PB12
  ******************************************************************************
  */

#include "spi.h"

/* SPI2 句柄 ----------------------------------------------------------------*/
static SPI_HandleTypeDef hspi2;

/* 片选引脚定义 -------------------------------------------------------------*/
#define SPI2_CS_GPIO_PORT   GPIOB
#define SPI2_CS_PIN         GPIO_PIN_12

/* 单字节收发超时(ms) -------------------------------------------------------*/
#define SPI2_TIMEOUT_MS     100U

/**
  * @brief  片选拉低(选中从机)
  */
void SPI2_CS_Low(void)
{
    HAL_GPIO_WritePin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN, GPIO_PIN_RESET);
}

/**
  * @brief  片选拉高(释放从机)
  */
void SPI2_CS_High(void)
{
    HAL_GPIO_WritePin(SPI2_CS_GPIO_PORT, SPI2_CS_PIN, GPIO_PIN_SET);
}

/**
  * @brief  初始化 SPI2 GPIO 与外设
  */
HAL_StatusTypeDef SPI2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();

    /* PB13(SCK)/PB15(MOSI): 复用推挽输出 */
    GPIO_InitStruct.Pin   = GPIO_PIN_13 | GPIO_PIN_15;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB14(MISO): 浮空输入(全双工接收) */
    GPIO_InitStruct.Pin  = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB12(NSS): 软件片选，推挽输出，默认拉高(未选中) */
    GPIO_InitStruct.Pin   = SPI2_CS_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SPI2_CS_GPIO_PORT, &GPIO_InitStruct);
    SPI2_CS_High();

    /* SPI2 外设配置: 全双工主机
     * SPI2 时钟源 = PCLK1 = 36MHz，分频 /8 = 4.5MHz。
     * XV7001BB 实际使用 SPI Mode0(CPOL=0, CPHA=0)。 */
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;   /* 全双工 */
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;       /* CPOL=0: 空闲低电平 */
    hspi2.Init.CLKPhase          = SPI_PHASE_1EDGE;        /* CPHA=0: 第一边沿采样 */
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8; /* 36MHz/8 = 4.5MHz */
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 10;

    return HAL_SPI_Init(&hspi2);
}

/**
  * @brief  SPI2 全双工收发(不控制片选)
  * @note   最后一个字节发送完成后，等待 BSY 标志清零，
  *         确保移位寄存器中的数据已完全移出，再返回让调用者拉高 CS
  */
HAL_StatusTypeDef SPI2_TransmitReceive(const uint8_t *pTxData, uint8_t *pRxData, uint16_t len)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t tx;
    uint8_t rx;
    uint16_t i;

    if (len == 0U)
    {
        return HAL_OK;
    }

    for (i = 0U; i < len; i++)
    {
        tx = (pTxData != NULL) ? pTxData[i] : 0xFFU;

        status = HAL_SPI_TransmitReceive(&hspi2, &tx, &rx, 1U, SPI2_TIMEOUT_MS);
        if (status != HAL_OK)
        {
            break;
        }

        if (pRxData != NULL)
        {
            pRxData[i] = rx;
        }
    }

    /* 等待 BSY 标志清零: 确保最后一个字节的移位寄存器已完全移出
     * 否则调用者立即拉高 CS 会导致最后几 bit 数据丢失 */
    if (status == HAL_OK)
    {
        uint32_t timeout = HAL_GetTick() + SPI2_TIMEOUT_MS;
        while (__HAL_SPI_GET_FLAG(&hspi2, SPI_FLAG_BSY) != RESET)
        {
            if (HAL_GetTick() > timeout)
            {
                status = HAL_TIMEOUT;
                break;
            }
        }
    }

    return status;
}

/**
  * @brief  发送单个字节(含片选)，用于示波器测试
  */
HAL_StatusTypeDef SPI2_SendByte(uint8_t data)
{
    HAL_StatusTypeDef status;

    SPI2_CS_Low();
    status = SPI2_TransmitReceive(&data, NULL, 1U);
    SPI2_CS_High();

    return status;
}

/**
  * @brief  发送一个字节并返回接收到的字节(含片选)
  */
uint8_t SPI2_ReadWriteByte(uint8_t data)
{
    uint8_t rx = 0U;

    SPI2_CS_Low();
    (void)SPI2_TransmitReceive(&data, &rx, 1U);
    SPI2_CS_High();

    return rx;
}
