@echo off
set OPENOCD=%LOCALAPPDATA%\VisualGDB\EmbeddedDebugPackages\com.sysprogs.arm.openocd\bin\openocd.exe
set SCRIPTS=%LOCALAPPDATA%\VisualGDB\EmbeddedDebugPackages\com.sysprogs.arm.openocd\share\openocd\scripts

"%OPENOCD%" -s "%SCRIPTS%" -f interface/cmsis-dap.cfg -c "adapter speed 3000" -f target/stm32f1x.cfg -c "program C:/temp/belt_test.elf verify reset exit"