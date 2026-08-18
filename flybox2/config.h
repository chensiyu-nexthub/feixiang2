/**
  ******************************************************************************
  * @file    config.h
  * @brief   Flash 配置存储接口 (STM32F103C8 内部 Flash 模拟 EEPROM)
  *
  *          使用最后 2 个 Flash 页 (Page62/63) 存储电机配置:
  *            - Page 62 (0x0800F800): 5 台电机配置 (每台 14 字节)
  *            - Page 63 (0x0800FC00): 预留扩展
  *
  *          读取策略: Flash 擦除后为 0xFF, 读到 0xFF 则返回默认值。
  ******************************************************************************
  */

#ifndef __CONFIG_H
#define __CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32f1xx_hal.h>

/* ======================== Flash 基址 ======================== */
#define CONFIG_FLASH_BASE           0x0800F800U  /* Page 62 起始地址 */
#define CONFIG_FLASH_PAGE_SIZE      1024U        /* STM32F103C8 页大小 1KB */
#define CONFIG_FLASH_PAGE62         62U
#define CONFIG_FLASH_PAGE63         63U

/* ======================== 电机配置在 Flash 中的偏移 ======================== */
#define CONFIG_OFFSET_MOTOR0        0x000U   /* 电机1 配置起始 */
#define CONFIG_OFFSET_MOTOR1        0x00EU   /* 电机2 */
#define CONFIG_OFFSET_MOTOR2        0x01CU   /* 电机3 */
#define CONFIG_OFFSET_MOTOR3        0x02AU   /* 电机4 */
#define CONFIG_OFFSET_MOTOR4        0x038U   /* 电机5 */

/* 每台电机配置占用 14 字节: cfg[6] + devno + dir + dia + spd + acc */
#define CONFIG_MOTOR_ENTRY_SIZE     14U

/* ======================== 电机配置结构体 (Flash 存储格式) ======================== */
typedef struct
{
    uint8_t  cfg[6];   /* PID 参数 (与 0x0316 帧对应) */
    uint8_t  devno;    /* 设备号 (CAN ID 后缀) */
    uint8_t  dir;      /* 方向: 0=CCW, 1=CW */
    uint16_t dia;      /* 轮径 ×100 (如 3194 = 31.94mm) */
    uint16_t spd;      /* 默认速度 (mm/s) */
    uint16_t acc;      /* 默认加速度 (mm/s²) */
} MotorConfig_t;

/* 运行时配置 (Flash 加载后 + 可动态修改) */
typedef struct
{
    MotorConfig_t saved;   /* Flash 中的原始值 */
    uint16_t      sspd;    /* 当前速度 (可运行时修改) */
    uint16_t      sacc;    /* 当前加速度 (可运行时修改) */
} MotorConfig_Run_t;

/* ======================== Flash 读取函数 ======================== */

/**
  * @brief  从 Flash 读取字节数组, 若擦除态(0xFF)则返回默认值
  * @param  dest        输出缓冲区
  * @param  flash_addr  Flash 绝对地址
  * @param  default_val 默认值指针
  * @param  len         字节数
  * @param  reset       1=强制使用默认值 (忽略 Flash)
  */
void cfg_read_mem(void *dest, uint32_t flash_addr, const void *default_val, uint16_t len, uint8_t reset);

/**
  * @brief  从 Flash 读取 uint16 (小端序), 擦除态返回默认值
  */
uint16_t cfg_read_u16(uint32_t flash_addr, uint16_t default_val, uint8_t reset);

/**
  * @brief  从 Flash 读取 uint8, 擦除态返回默认值
  */
uint8_t cfg_read_u8(uint32_t flash_addr, uint8_t default_val, uint8_t reset);

/* ======================== Flash 写入函数 ======================== */

/**
  * @brief  将配置写入 Flash (需先擦除 Page62)
  * @note   写入期间会关全局中断, 完成后恢复
  */
HAL_StatusTypeDef cfg_write_motor_config(const MotorConfig_t *configs, uint8_t count);

/**
  * @brief  擦除配置页 (Page62), 恢复出厂默认值
  */
HAL_StatusTypeDef cfg_erase_all(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_H */