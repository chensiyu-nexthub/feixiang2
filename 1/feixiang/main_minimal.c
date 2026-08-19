/**
  * @brief  极简 LED 闪烁测试 — 纯寄存器, 零依赖
  *         如果这个也不亮, 说明固件根本没跑起来。
  */

/* startup 文件会调用, 必须提供 */
void SystemInit(void) { }

int main(void)
{
    /* 使能 GPIOC 时钟 */
    *(volatile unsigned int *)0x40021018 |= (1 << 4);  /* RCC_APB2ENR |= IOPCEN */

    /* PC13 推挽输出, 50MHz */
    unsigned int tmp = *(volatile unsigned int *)0x40011004;  /* GPIOC_CRH */
    tmp &= ~(0xFU << 20);      /* 清 CNF13[1:0] + MODE13[1:0] */
    tmp |= (0x3U << 20);       /* MODE13=11 (50MHz output) */
    *(volatile unsigned int *)0x40011004 = tmp;

    for (;;)
    {
        /* PC13 = 0 (LED 亮, 低电平驱动) */
        *(volatile unsigned int *)0x40011014 = (1 << 13);  /* BSRR_BR13 */
        for (volatile unsigned int d = 0; d < 400000; d++) { }

        /* PC13 = 1 (LED 灭) */
        *(volatile unsigned int *)0x40011010 = (1 << 13);  /* BSRR_BS13 */
        for (volatile unsigned int d = 0; d < 400000; d++) { }
    }
}