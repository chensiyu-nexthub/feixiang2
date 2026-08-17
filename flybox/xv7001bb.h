/**
  ******************************************************************************
  * @file    xv7001bb.h
  * @brief   Epson XV7001BB 陀螺仪驱动接口 (4-wire SPI, Mode3)
  ******************************************************************************
  */

#ifndef __XV7001BB_H
#define __XV7001BB_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

/* ======================== 寄存器地址定义 ======================== */
#define XV7001BB_REG_DSPCTL1        0x01U   /* DSP Settings 1 (HPF)          R/W */
#define XV7001BB_REG_DSPCTL2        0x02U   /* DSP Settings 2 (LPF)          R/W */
#define XV7001BB_REG_DSPCTL3        0x03U   /* DSP Settings 3 (校准/采样率)  R/W */
#define XV7001BB_REG_STSRD          0x04U   /* Status Read                   R   */
#define XV7001BB_REG_SLPIN          0x05U   /* Sleep-in                      C   */
#define XV7001BB_REG_SLPOUT         0x06U   /* Sleep-out                     C   */
#define XV7001BB_REG_STBY           0x07U   /* Standby                       C   */
#define XV7001BB_REG_TEMPRD         0x08U   /* Temperature sensor data read  R   */
#define XV7001BB_REG_SWRST          0x09U   /* Software reset                C   */
#define XV7001BB_REG_DATACCON       0x0AU   /* Angular rate data read        R   */
#define XV7001BB_REG_OUTCTL1        0x0BU   /* Angular rate data read ctrl   R/W */
#define XV7001BB_REG_AUTOC          0x0CU   /* Zero rate level calibration   C   */
#define XV7001BB_REG_DSPRES         0x0DU   /* Digital filter reset          C   */
#define XV7001BB_REG_MEMLOAD        0x1BU   /* Reference data register reset C   */
#define XV7001BB_REG_TSDATAFORMAT   0x1CU   /* Temperature sensor data fmt   R/W */
#define XV7001BB_REG_IFCTL          0x1FU   /* Serial interface settings     R/W */

/* ======================== 状态位定义 ======================== */
/* StsRd (0x04) bit 定义 */
#define XV7001BB_STS_PROCOK         (1U << 3)   /* bit3: 温度数据输出可用 */
#define XV7001BB_STS_MASK_STATE     0x07U       /* bit[2:0]: 状态机 */
#define XV7001BB_STS_POWER_ON       0x04U       /* 100: 上电后 */
#define XV7001BB_STS_STANDBY        0x02U       /* 010: 待机 */
#define XV7001BB_STS_SLEEP          0x00U       /* 000: 睡眠 */
#define XV7001BB_STS_SLEEPOUT       0x01U       /* 001: 睡眠退出(正常工作) */

/* ======================== 灵敏度/换算常量 ======================== */
/* 角速度灵敏度: 16bit 模式 */
#define XV7001BB_SCALE_FACTOR_16BIT     280.0f      /* LSB/(°/s) */
/* 角速度灵敏度: 24bit 模式 */
#define XV7001BB_SCALE_FACTOR_24BIT     71680.0f    /* LSB/(°/s) */

/* 温度传感器: 12bit 模式 */
#define XV7001BB_TEMP_OFFSET_12BIT      400         /* LSB @ 25°C */
#define XV7001BB_TEMP_COEFF_12BIT       16.0f       /* LSB/°C */
#define XV7001BB_TEMP_REF               25.0f       /* 参考温度 °C */

/* ======================== 上电时序常量 ======================== */
#define XV7001BB_TIF_MS             1U      /* 串口通信等待时间 (min) */
#define XV7001BB_TTSEN_MS           80U     /* 温度传感器数据就绪时间 */
#define XV7001BB_TSTA_MS            200U    /* 启动时间(角速度输出稳定) */

/* ======================== LPF 预设值 (DspCtl2 0x02) ======================== */
/* LpfOrder[1:0] = bit[5:4], LpfFc[3:0] = bit[3:0] */
#define XV7001BB_LPF_2ND_50HZ      0x03U   /* 2阶 50Hz  */
#define XV7001BB_LPF_3RD_50HZ      0x13U   /* 3阶 50Hz  */
#define XV7001BB_LPF_4TH_50HZ      0x23U   /* 4阶 50Hz  */
#define XV7001BB_LPF_3RD_100HZ     0x16U   /* 3阶 100Hz */

/* ======================== 接口函数声明 ======================== */

/**
  * @brief  XV7001BB 初始化: 上电时序 → 软件复位 → SleepOut → 配置默认参数
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_Init(void);

/**
  * @brief  写寄存器
  * @param  regAddr  寄存器地址 (0x00~0x1F)
  * @param  data     写入数据
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_WriteReg(uint8_t regAddr, uint8_t data);

/**
  * @brief  读寄存器
  * @param  regAddr  寄存器地址 (0x00~0x1F)
  * @param  pData    读出数据指针
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_ReadReg(uint8_t regAddr, uint8_t *pData);

/**
  * @brief  发送命令(只发地址，无数据)
  * @param  regAddr  命令地址
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_WriteCmd(uint8_t regAddr);

/**
  * @brief  读取状态寄存器 (0x04)
  * @param  pStatus  状态字节指针
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_ReadStatus(uint8_t *pStatus);

/**
  * @brief  读取温度原始值 (12bit, 2's complement)
  * @param  pRawTemp  原始温度值指针
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_ReadTemp(int16_t *pRawTemp);

/**
  * @brief  读取温度并换算为 °C
  * @param  pTempDegC  温度值指针 (°C)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_ReadTempDegC(float *pTempDegC);

/**
  * @brief  读取角速度原始值 (16bit, 2's complement)
  * @param  pRawAngle  原始角速度值指针
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_ReadAngle(int16_t *pRawAngle);

/**
  * @brief  读取角速度并换算为 °/s
  * @param  pAngleDegS  角速度值指针 (°/s)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_ReadAngleDegS(float *pAngleDegS);
/**
  * @brief  零偏校准: 静止状态下多次采样角速度并取平均
  * @note   调用期间传感器必须保持完全静止
  * @param  samples  采样次数 (建议 ≥ 100)
  * @param  pBiasDps 输出零偏值指针 (°/s)
  * @retval HAL_OK 成功
  */
HAL_StatusTypeDef XV7001bb_CalibrateBias(int samples, float *pBiasDps);
#ifdef __cplusplus
}
#endif

#endif /* __XV7001BB_H */
