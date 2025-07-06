@echo off
echo GRSVC1 Firmware Preparation for OTA and USB Deployment
echo.
echo This script prepares firmware binaries for both OTA and USB deployment
echo.

REM Check if we're in the right directory
if not exist "platformio.ini" (
    echo Error: Please run this script from the GRSVC1 project root directory
    echo (the folder containing platformio.ini)
    pause
    exit /b 1
)

echo Found PlatformIO project
echo.

REM Get version from user or use default
set /p FIRMWARE_VERSION="Enter firmware version (default: 1.0.0): "
if "%FIRMWARE_VERSION%"=="" set FIRMWARE_VERSION=1.0.0

REM Get hardware revision from user or use default
set /p HARDWARE_REV="Enter hardware revision (default: r1): "
if "%HARDWARE_REV%"=="" set HARDWARE_REV=r1

echo.
echo 📋 Building firmware version %FIRMWARE_VERSION% for hardware %HARDWARE_REV%
echo.

REM Create versioned firmware directory structure
set FIRMWARE_DIR=docs\firmware\%FIRMWARE_VERSION%
if not exist "docs\firmware" mkdir "docs\firmware"
if not exist "%FIRMWARE_DIR%" mkdir "%FIRMWARE_DIR%"

echo 📋 Firmware Preparation Steps:
echo.

echo 1️⃣ Cleaning previous build...
pio run -t clean
if errorlevel 1 (
    echo Clean failed
    pause
    exit /b 1
)
echo ✅ Clean completed
echo.

echo 2️⃣ Building firmware with OTA partition table...
pio run
if errorlevel 1 (
    echo Build failed
    pause
    exit /b 1
)
echo ✅ Build completed
echo.

echo 3️⃣ Checking generated firmware files...
if exist ".pio\build\esp32dev\firmware.bin" (
    echo ✅ Main firmware binary found
    
    REM Copy and rename for USB flashing
    copy ".pio\build\esp32dev\firmware.bin" "%FIRMWARE_DIR%\grsvc1-%HARDWARE_REV%-v%FIRMWARE_VERSION%.bin"
    echo ✅ USB firmware: grsvc1-%HARDWARE_REV%-v%FIRMWARE_VERSION%.bin
    
    REM Copy and rename for OTA updates
    copy ".pio\build\esp32dev\firmware.bin" "%FIRMWARE_DIR%\grsvc1-%HARDWARE_REV%-v%FIRMWARE_VERSION%.ota.bin"
    echo ✅ OTA firmware: grsvc1-%HARDWARE_REV%-v%FIRMWARE_VERSION%.ota.bin
    
) else (
    echo ❌ firmware.bin not found in build output
    pause
    exit /b 1
)

REM Check for bootloader and partition table (needed for complete flashing)
if exist ".pio\build\esp32dev\bootloader.bin" (
    copy ".pio\build\esp32dev\bootloader.bin" "%FIRMWARE_DIR%\bootloader.bin"
    echo ✅ Bootloader: bootloader.bin
)

if exist ".pio\build\esp32dev\partitions.bin" (
    copy ".pio\build\esp32dev\partitions.bin" "%FIRMWARE_DIR%\partitions.bin"
    echo ✅ Partitions: partitions.bin
)

echo.
echo 4️⃣ Firmware file information:
for %%f in (%FIRMWARE_DIR%\*.bin) do (
    echo    %%f - %%~zf bytes
)
echo.

echo 5️⃣ Verifying OTA partition configuration...
if exist "partitions_two_ota.csv" (
    echo ✅ OTA partition table configured
    type partitions_two_ota.csv
    echo.
) else (
    echo ⚠️  Warning: OTA partition table not found
    echo    OTA updates may not work properly
)

echo 6️⃣ Creating firmware manifest...
echo { > %FIRMWARE_DIR%\manifest.json
echo   "firmware_version": "%FIRMWARE_VERSION%", >> %FIRMWARE_DIR%\manifest.json
echo   "hardware_revision": "%HARDWARE_REV%", >> %FIRMWARE_DIR%\manifest.json
echo   "build_date": "%date% %time%", >> %FIRMWARE_DIR%\manifest.json
echo   "usb_firmware": "grsvc1-%HARDWARE_REV%-v%FIRMWARE_VERSION%.bin", >> %FIRMWARE_DIR%\manifest.json
echo   "ota_firmware": "grsvc1-%HARDWARE_REV%-v%FIRMWARE_VERSION%.ota.bin", >> %FIRMWARE_DIR%\manifest.json
echo   "partition_table": "partitions.bin", >> %FIRMWARE_DIR%\manifest.json
echo   "bootloader": "bootloader.bin" >> %FIRMWARE_DIR%\manifest.json
echo } >> %FIRMWARE_DIR%\manifest.json
echo ✅ Firmware manifest created
echo.

echo 🎉 Firmware preparation completed successfully!
echo.
echo 📁 Files created in %FIRMWARE_DIR%\:
dir %FIRMWARE_DIR%\ /b
echo.

echo 7️⃣ Updating main firmware list...
REM Create or update the main firmware versions list
echo Creating firmware versions index...
> docs\firmware\versions.json echo {
>> docs\firmware\versions.json echo   "available_versions": [
>> docs\firmware\versions.json echo     {
>> docs\firmware\versions.json echo       "version": "%FIRMWARE_VERSION%",
>> docs\firmware\versions.json echo       "hardware": "%HARDWARE_REV%",
>> docs\firmware\versions.json echo       "build_date": "%date% %time%",
>> docs\firmware\versions.json echo       "path": "%FIRMWARE_VERSION%",
>> docs\firmware\versions.json echo       "usb_firmware": "grsvc1-%HARDWARE_REV%-v%FIRMWARE_VERSION%.bin",
>> docs\firmware\versions.json echo       "ota_firmware": "grsvc1-%HARDWARE_REV%-v%FIRMWARE_VERSION%.ota.bin",
>> docs\firmware\versions.json echo       "description": "Latest firmware with OTA support"
>> docs\firmware\versions.json echo     }
>> docs\firmware\versions.json echo   ],
>> docs\firmware\versions.json echo   "latest_version": "%FIRMWARE_VERSION%",
>> docs\firmware\versions.json echo   "default_hardware": "%HARDWARE_REV%"
>> docs\firmware\versions.json echo }
echo ✅ Firmware versions index updated

echo.
echo 💡 Next Steps:
echo    1. For USB flashing: Use grsvc1-%HARDWARE_REV%-v%FIRMWARE_VERSION%.bin with ESP Web Tools
echo    2. For OTA updates: Use grsvc1-%HARDWARE_REV%-v%FIRMWARE_VERSION%.ota.bin via BLE interface
echo    3. Deploy to GitHub Pages using deploy_web.bat
echo    4. To add more versions, run this script again with different version numbers
echo.

echo 🔍 Important Notes:
echo    - USB firmware includes full image for clean installation
echo    - OTA firmware is application-only for updates
echo    - Both use the same binary in this configuration
echo    - OTA partition table enables incremental updates
echo    - First flash should always be via USB to set up partitions
echo.

pause
