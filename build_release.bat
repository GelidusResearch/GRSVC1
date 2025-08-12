@echo off
REM Build GRSVC1 firmware for release (UF2 only, no upload)

echo ========================================
echo GRSVC1 Release Build (UF2 Generation)
echo ========================================
echo.

echo Building release firmware (UF2 only)...
platformio run -e pro_micro_nrf52840_uf2

if %ERRORLEVEL% EQU 0 (
    echo.
    echo ========================================
    echo Release build successful!
    echo ========================================
    echo.
    echo Created firmware files:
    if exist firmware\grsvc1-r1-v1.0.0.uf2 (
        echo   ✓ grsvc1-r1-v1.0.0.uf2 [Ready for drag-and-drop installation]
    )
    if exist firmware\grsvc1-r1-v1.0.0.hex (
        echo   ✓ grsvc1-r1-v1.0.0.hex [For nrfutil or Arduino IDE]
    )
    if exist firmware\grsvc1-r1-v1.0.0.bin (
        echo   ✓ grsvc1-r1-v1.0.0.bin [For advanced users]
    )
    echo.
    echo 📦 All files ready for distribution!
    echo.
) else (
    echo.
    echo ========================================
    echo Build failed!
    echo ========================================
)

pause
