@echo off

REM STM32MP_SigningTool_CLI.exe -bin FSBL.bin -nk -of 0x80000000 -t fsbl -o FSBL-trusted.bin -hv 2.3 -dump FSBL-trusted.bin

set SIGNING_TOOL_PATH="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_SigningTool_CLI.exe"
set APP_NAME="stm32n6_nucleo_fsbl_lrun_led_toggle_FSBL.bin"

echo %SIGNING_TOOL_PATH% -bin %APP_NAME% -nk -of 0x80000000 -t fsbl -o FSBL-trusted.bin -hv 2.3 -dump FSBL-trusted.bin  
%SIGNING_TOOL_PATH% -bin %APP_NAME% -nk -of 0x80000000 -t fsbl -o FSBL-trusted.bin -hv 2.3 -dump FSBL-trusted.bin

cmd /k