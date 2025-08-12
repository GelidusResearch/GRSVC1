@echo off
REM Build GRSVC1 nRF52840 firmware and generate UF2 file
REM This script builds the firmware and creates UF2 files for easy installation

echo ========================================
echo GRSVC1 nRF52840 Firmware Builder
echo ========================================
echo.

echo Building firmware...
platformio run -e pro_micro_nrf52840

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo Build successful!
    echo ========================================
    echo.
    echo Firmware files created:
    dir /b firmware\grsvc1-r1-v1.0.0.*
    echo.
    echo Installation instructions:
    echo 1. Put your nRF52840 into bootloader mode (double-click reset)
    echo 2. A drive named "nRF52BOOT" should appear
    echo 3. Drag firmware\grsvc1-r1-v1.0.0.uf2 to the drive
    echo 4. The device will restart with new firmware
    echo.
) else (
    echo.
    echo ========================================
    echo Build failed!
    echo ========================================
    echo Check the error messages above.
)

pause
