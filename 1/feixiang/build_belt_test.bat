@echo off
REM ==========================================================
REM  皮带测试 编译脚本
REM  用法: 双击运行, 或在 VS Code 终端中执行
REM ==========================================================

set BSP=%LOCALAPPDATA%\VisualGDB\EmbeddedBSPs\arm-eabi\com.sysprogs.arm.stm32\2020.01
set TOOLS=C:\SysGCC\arm-eabi\bin
set PROJ=%~dp0
set OUT=%TEMP%\belt_test

if not exist "%OUT%" mkdir "%OUT%"

set FLAGS=-mcpu=cortex-m3 -mthumb -DSTM32F103C8 -DSTM32F103xB -DUSE_HAL_DRIVER -DUSE_HAL_LEGACY -Dflash_layout -DARM_MATH_CM3 -O0 -ggdb

set INCS=-I"%PROJ%" -I"%BSP%\STM32F1xxxx\STM32F1xx_HAL_Driver\Inc" -I"%BSP%\STM32F1xxxx\STM32F1xx_HAL_Driver\Inc\Legacy" -I"%BSP%\STM32F1xxxx\CMSIS_HAL\Core\Include" -I"%BSP%\STM32F1xxxx\CMSIS_HAL\Device\ST\STM32F1xx\Include" -I"%BSP%\STM32F1xxxx\CMSIS_HAL\Include"

set LDSCRIPT=%BSP%\STM32F1xxxx\LinkerScripts\STM32F103C8_flash.lds
set HAL_SRC=%BSP%\STM32F1xxxx\STM32F1xx_HAL_Driver\Src

echo [1/5] Compiling startup...
"%TOOLS%\arm-none-eabi-gcc" %FLAGS% %INCS% -c "%BSP%\STM32F1xxxx\StartupFiles\startup_stm32f103xb.c" -o "%OUT%\startup.o"
if %ERRORLEVEL% neq 0 goto :fail

echo [2/5] Compiling system...
"%TOOLS%\arm-none-eabi-gcc" %FLAGS% %INCS% -c "%PROJ%system_stm32f1xx.c" -o "%OUT%\system.o"
if %ERRORLEVEL% neq 0 goto :fail

echo [3/5] Compiling HAL...
"%TOOLS%\arm-none-eabi-gcc" %FLAGS% %INCS% -c "%HAL_SRC%\stm32f1xx_hal.c" -o "%OUT%\hal.o"
"%TOOLS%\arm-none-eabi-gcc" %FLAGS% %INCS% -c "%HAL_SRC%\stm32f1xx_hal_can.c" -o "%OUT%\hal_can.o"
"%TOOLS%\arm-none-eabi-gcc" %FLAGS% %INCS% -c "%HAL_SRC%\stm32f1xx_hal_rcc.c" -o "%OUT%\hal_rcc.o"
"%TOOLS%\arm-none-eabi-gcc" %FLAGS% %INCS% -c "%HAL_SRC%\stm32f1xx_hal_gpio.c" -o "%OUT%\hal_gpio.o"
"%TOOLS%\arm-none-eabi-gcc" %FLAGS% %INCS% -c "%HAL_SRC%\stm32f1xx_hal_cortex.c" -o "%OUT%\hal_cortex.o"
"%TOOLS%\arm-none-eabi-gcc" %FLAGS% %INCS% -c "%HAL_SRC%\stm32f1xx_hal_flash.c" -o "%OUT%\hal_flash.o"
if %ERRORLEVEL% neq 0 goto :fail

echo [4/5] Compiling main.c...
"%TOOLS%\arm-none-eabi-gcc" %FLAGS% %INCS% -c "%PROJ%main.c" -o "%OUT%\main.o"
if %ERRORLEVEL% neq 0 goto :fail

echo [5/5] Linking...
"%TOOLS%\arm-none-eabi-gcc" -T"%LDSCRIPT%" -mcpu=cortex-m3 -mthumb --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections "%OUT%\startup.o" "%OUT%\system.o" "%OUT%\hal.o" "%OUT%\hal_can.o" "%OUT%\hal_rcc.o" "%OUT%\hal_gpio.o" "%OUT%\hal_cortex.o" "%OUT%\hal_flash.o" "%OUT%\main.o" -o "%OUT%\belt_test.elf"
if %ERRORLEVEL% neq 0 goto :fail

echo.
echo ========================================
echo  BUILD SUCCESS!
echo  ELF: %OUT%\belt_test.elf
echo  HEX: %OUT%\belt_test.hex
echo.
echo 烧录: 用 ST-LINK Utility 或 VisualGDB
echo       打开 %OUT%\belt_test.elf
echo ========================================
"%TOOLS%\arm-none-eabi-objcopy" -O ihex "%OUT%\belt_test.elf" "%OUT%\belt_test.hex"
"%TOOLS%\arm-none-eabi-size" "%OUT%\belt_test.elf"
goto :end

:fail
echo.
echo ========================================
echo  BUILD FAILED!
echo ========================================
exit /b 1

:end