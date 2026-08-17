/**
  ******************************************************************************
  * @file    xv7001bb.c
  * @brief   Epson XV7001BB 陀螺仪驱动实现 (4-wire SPI, Mode3)
  ******************************************************************************
  */

#include "xv7001bb.h"
#include "spi.h"
#include <cmsis_os.h>

/* ======================== 内部辅助函数 ======================== */

/**
  * @brief  构造 SPI 地址字节: MSB=R/W, bit[6:5]=00(单器件), bit[4:0]=寄存器地址
  * @param  regAddr  寄存器地址 (0x00~0x1F)
  * @param  isRead   1=读, 0=写
  * @retval 地址字节
  */
static uint8_t XV7001bb_MakeAddr(uint8_t regAddr, uint8_t isRead)
{
    uint8_t addr = regAddr & 0x1FU;   /* 低 5 位为寄存器地址 */
    if (isRead)
    {
        addr |= 0x80U;   /* MSB=1 表示读 */
    }
    return addr;
}

/* ======================== 接口函数实现 ======================== */

/**
  * @brief  写寄存器
  */
HAL_StatusTypeDef XV7001bb_WriteReg(uint8_t regAddr, uint8_t data)
{
    uint8_t txBuf[2];
    uint8_t rxBuf[2];

    txBuf[0] = XV7001bb_MakeAddr(regAddr, 0);  /* MSB=0: 写 */
    txBuf[1] = data;

    SPI2_CS_Low();
    HAL_StatusTypeDef status = SPI2_TransmitReceive(txBuf, rxBuf, 2);
    SPI2_CS_High();

    return status;
}

/**
  * @brief  读寄存器
  */
HAL_StatusTypeDef XV7001bb_ReadReg(uint8_t regAddr, uint8_t *pData)
{
    uint8_t txBuf[2];
    uint8_t rxBuf[2];

    if (pData == NULL)
    {
        return HAL_ERROR;
    }

    txBuf[0] = XV7001bb_MakeAddr(regAddr, 1);  /* MSB=1: 读 */
    txBuf[1] = 0x00U;  /* 第二字节为 dummy，用于接收数据 */

    SPI2_CS_Low();
    HAL_StatusTypeDef status = SPI2_TransmitReceive(txBuf, rxBuf, 2);
    SPI2_CS_High();

    if (status == HAL_OK)
    {
        *pData = rxBuf[1];  /* 数据在第二字节返回 */
    }

    return status;
}

/**
  * @brief  发送命令(发地址 + dummy 字节)
  */
HAL_StatusTypeDef XV7001bb_WriteCmd(uint8_t regAddr)
{
    uint8_t txBuf[2];
    uint8_t rxBuf[2];

    txBuf[0] = XV7001bb_MakeAddr(regAddr, 0);  /* MSB=0: 命令 */
    txBuf[1] = 0x00U;  /* dummy 字节，触发命令 */

    SPI2_CS_Low();
    HAL_StatusTypeDef status = SPI2_TransmitReceive(txBuf, rxBuf, 2);
    SPI2_CS_High();

    return status;
}

/**
  * @brief  读取状态寄存器 (0x04)
  */
HAL_StatusTypeDef XV7001bb_ReadStatus(uint8_t *pStatus)
{
    return XV7001bb_ReadReg(XV7001BB_REG_STSRD, pStatus);
}

/**
  * @brief  读取温度原始值 (12bit, 2's complement)
  *         手册 Section 5-6: 12bit 模式，读 2 字节，高 12 位有效
  */
HAL_StatusTypeDef XV7001bb_ReadTemp(int16_t *pRawTemp)
{
    uint8_t txBuf[3];
    uint8_t rxBuf[3];

    if (pRawTemp == NULL)
    {
        return HAL_ERROR;
    }

    /* 发送温度读命令 (0x08) */
    txBuf[0] = XV7001bb_MakeAddr(XV7001BB_REG_TEMPRD, 1);
    txBuf[1] = 0x00U;
    txBuf[2] = 0x00U;

    SPI2_CS_Low();
    HAL_StatusTypeDef status = SPI2_TransmitReceive(txBuf, rxBuf, 3);
    SPI2_CS_High();

    if (status == HAL_OK)
    {
        /* 12bit 模式: 第 1 字节为 D[11:4]，第 2 字节高 4 位为 D[3:0] */
        int16_t raw = ((int16_t)rxBuf[1] << 4) | ((rxBuf[2] >> 4) & 0x0FU);

        /* 符号扩展: 12bit 转 16bit */
        if (raw & 0x0800U)
        {
            raw |= 0xF000U;
        }

        *pRawTemp = raw;
    }

    return status;
}

/**
  * @brief  读取温度并换算为 °C
  *         公式: T = 25 + (raw - 400) / 16
  */
HAL_StatusTypeDef XV7001bb_ReadTempDegC(float *pTempDegC)
{
    int16_t raw;
    HAL_StatusTypeDef status = XV7001bb_ReadTemp(&raw);

    if ((status == HAL_OK) && (pTempDegC != NULL))
    {
        *pTempDegC = XV7001BB_TEMP_REF +
                     ((float)raw - XV7001BB_TEMP_OFFSET_12BIT) / XV7001BB_TEMP_COEFF_12BIT;
    }

    return status;
}

/**
  * @brief  读取角速度原始值 (16bit, 2's complement)
  *         手册 Section 5-5: 16bit 模式，读 2 字节
  */
HAL_StatusTypeDef XV7001bb_ReadAngle(int16_t *pRawAngle)
{
    uint8_t txBuf[3];
    uint8_t rxBuf[3];

    if (pRawAngle == NULL)
    {
        return HAL_ERROR;
    }

    /* 发送角速度读命令 (0x0a) */
    txBuf[0] = XV7001bb_MakeAddr(XV7001BB_REG_DATACCON, 1);
    txBuf[1] = 0x00U;
    txBuf[2] = 0x00U;

    SPI2_CS_Low();
    HAL_StatusTypeDef status = SPI2_TransmitReceive(txBuf, rxBuf, 3);
    SPI2_CS_High();

    if (status == HAL_OK)
    {
        /* 16bit 模式: 第 1 字节为 D[15:8]，第 2 字节为 D[7:0] */
        *pRawAngle = ((int16_t)rxBuf[1] << 8) | rxBuf[2];
    }

    return status;
}

/**
  * @brief  读取角速度并换算为 °/s
  *         公式: ω = raw / 280 (16bit 模式)
  */
HAL_StatusTypeDef XV7001bb_ReadAngleDegS(float *pAngleDegS)
{
    int16_t raw;
    HAL_StatusTypeDef status = XV7001bb_ReadAngle(&raw);

    if ((status == HAL_OK) && (pAngleDegS != NULL))
    {
        *pAngleDegS = (float)raw / XV7001BB_SCALE_FACTOR_16BIT;
    }

    return status;
}

/**
  * @brief  零偏校准: 静止状态下多次采样角速度并取平均
  * @note   调用期间传感器必须保持完全静止
  * @param  samples  采样次数 (建议 ≥ 100)
  * @param  pBiasDps 输出零偏值指针 (°/s)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_CalibrateBias(int samples, float *pBiasDps)
{
    int16_t rawAngle;
    float biasSum = 0.0f;
    int validSamples = 0;

    if ((pBiasDps == NULL) || (samples <= 0))
    {
        return HAL_ERROR;
    }

    for (int i = 0; i < samples; i++)
    {
        HAL_StatusTypeDef status = XV7001bb_ReadAngle(&rawAngle);
        if (status == HAL_OK)
        {
            biasSum += (float)rawAngle / XV7001BB_SCALE_FACTOR_16BIT;
            validSamples++;
        }
        osDelay(10);  /* 10ms 间隔采样 */
    }

    if (validSamples == 0)
    {
        return HAL_ERROR;
    }

    *pBiasDps = biasSum / (float)validSamples;
    return HAL_OK;
}

/**
  * @brief  XV7001BB 初始化序列
  *         参考 g 版本: 软复位 -> 退出睡眠 -> 配置 LPF -> DSP 复位 -> 等待稳定 -> 零点校准
  *
  *         1. 软件复位 (SWRst)
  *         2. 退出睡眠 (SleepOut)
  *         3. 配置硬件低通滤波器 (DspCtl2): 3阶 50Hz
  *         4. 复位数字滤波器，使 LPF 设置生效
  *         5. 等待 tSTA (角速度输出稳定 ≥ 200ms)
  *         6. 硬件零点校准 (AutoC) — 板子必须完全静止
  */
HAL_StatusTypeDef XV7001bb_Init(void)
{
    HAL_StatusTypeDef status;

    /* 1. 软件复位: 恢复默认配置 */
    status = XV7001bb_WriteCmd(XV7001BB_REG_SWRST);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(10U);

    /* 2. 退出睡眠模式，进入正常工作 */
    status = XV7001bb_WriteCmd(XV7001BB_REG_SLPOUT);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(10U);

    /* 3. 配置硬件低通滤波器 (DspCtl2): 3阶 50Hz，硬件降噪 */
    status = XV7001bb_WriteReg(XV7001BB_REG_DSPCTL2, XV7001BB_LPF_3RD_50HZ);
    if (status != HAL_OK)
    {
        return status;
    }

    /* 4. 复位数字滤波器，使 LPF 设置生效 */
    status = XV7001bb_WriteCmd(XV7001BB_REG_DSPRES);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(10U);

    /* 5. 等待启动稳定 tSTA ≥ 200ms */
    osDelay(XV7001BB_TSTA_MS);

    /* 6. 硬件零点校准 (AutoC)
     *    ★ 此时板子必须完全静止！ */
    status = XV7001bb_WriteCmd(XV7001BB_REG_AUTOC);
    if (status != HAL_OK)
    {
        return status;
    }
    osDelay(100U);

    return HAL_OK;
}
