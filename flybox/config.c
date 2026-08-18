/**
  ******************************************************************************
  * @file    config.c
  * @brief   Flash 配置存储实现
  *
  *          STM32F103C8 内部 Flash:
  *            - 页大小 1KB, 擦除后为 0xFF
  *            - 写入前必须先擦除整页
  *            - 写入以 16 位半字为单位
  *            - 写/擦期间 Flash 不可读, 需关中断
  ******************************************************************************
  */

#include "config.h"
#include <string.h>

/* ======================== Flash 读取 (无副作用, 可直接读) ======================== */

/**
  * @brief  从 Flash 读取字节数组, 擦除态(全 0xFF)则返回默认值
  */
void cfg_read_mem(void *dest, uint32_t flash_addr, const void *default_val, uint16_t len, uint8_t reset)
{
    if (reset)
    {
        if (default_val != NULL) memcpy(dest, default_val, len);
        return;
    }

    const uint8_t *flash_ptr = (const uint8_t *)flash_addr;

    /* 检查是否为擦除态 (全 0xFF) */
    uint8_t is_erased = 1U;
    for (uint16_t i = 0U; i < len; i++)
    {
        if (flash_ptr[i] != 0xFFU) { is_erased = 0U; break; }
    }

    if (is_erased)
    {
        if (default_val != NULL) memcpy(dest, default_val, len);
    }
    else
    {
        memcpy(dest, flash_ptr, len);
    }
}

/**
  * @brief  从 Flash 读取 uint16 (小端序), 擦除态返回默认值
  */
uint16_t cfg_read_u16(uint32_t flash_addr, uint16_t default_val, uint8_t reset)
{
    if (reset) return default_val;

    const uint8_t *p = (const uint8_t *)flash_addr;
    uint16_t val = (uint16_t)p[0] | ((uint16_t)p[1] << 8);

    if (val == 0xFFFFU) return default_val;
    return val;
}

/**
  * @brief  从 Flash 读取 uint8, 擦除态返回默认值
  */
uint8_t cfg_read_u8(uint32_t flash_addr, uint8_t default_val, uint8_t reset)
{
    if (reset) return default_val;

    uint8_t val = *(const uint8_t *)flash_addr;
    if (val == 0xFFU) return default_val;
    return val;
}

/* ======================== Flash 写入 (需擦除 + 关中断) ======================== */

/**
  * @brief  擦除 Page62 并写入 5 台电机配置
  */
HAL_StatusTypeDef cfg_write_motor_config(const MotorConfig_t *configs, uint8_t count)
{
    if (configs == NULL || count == 0U) return HAL_ERROR;

    HAL_StatusTypeDef status;
    uint32_t pageError = 0U;
    FLASH_EraseInitTypeDef eraseInit = {0};

    /* 1. 解锁 Flash */
    HAL_FLASH_Unlock();

    /* 2. 擦除 Page62 */
    eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = CONFIG_FLASH_BASE;
    eraseInit.NbPages     = 1U;
    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return status;
    }

    /* 3. 逐半字写入 */
    uint32_t addr = CONFIG_FLASH_BASE;
    const uint8_t *raw = (const uint8_t *)configs;
    uint32_t totalBytes = (uint32_t)count * (uint32_t)CONFIG_MOTOR_ENTRY_SIZE;

    for (uint32_t i = 0U; i < totalBytes; i += 2U)
    {
        uint16_t halfWord;
        if (i + 1U < totalBytes)
        {
            halfWord = (uint16_t)raw[i] | ((uint16_t)raw[i + 1U] << 8);
        }
        else
        {
            halfWord = (uint16_t)raw[i] | 0xFF00U;  /* 末字节补齐 0xFF */
        }

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, halfWord);
        if (status != HAL_OK)
        {
            HAL_FLASH_Lock();
            return status;
        }
    }

    /* 4. 锁定 Flash */
    HAL_FLASH_Lock();
    return HAL_OK;
}

/**
  * @brief  擦除全部配置页, 恢复出厂默认值
  */
HAL_StatusTypeDef cfg_erase_all(void)
{
    HAL_StatusTypeDef status;
    uint32_t pageError = 0U;
    FLASH_EraseInitTypeDef eraseInit = {0};

    HAL_FLASH_Unlock();

    eraseInit.TypeErase   = FLASH_TYPEERASE_PAGES;
    eraseInit.PageAddress = CONFIG_FLASH_BASE;
    eraseInit.NbPages     = 2U;  /* Page62 + Page63 */
    status = HAL_FLASHEx_Erase(&eraseInit, &pageError);

    HAL_FLASH_Lock();
    return status;
}