@echo off
REM PlatformIO OTA Partition Setup Script for Windows
REM This script erases the ESP32 flash and reflashes with OTA-capable partition table

echo ================================
echo PlatformIO OTA Partition Setup
echo ================================
echo.

REM Check if PlatformIO is available
where pio >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: PlatformIO CLI not found in PATH
    echo Please install PlatformIO Core or add it to your PATH
    echo Visit: https://platformio.org/install/cli
    pause
    exit /b 1
)

REM Check if we're in the correct directory
if not exist "platformio.ini" (
    echo ERROR: platformio.ini not found in current directory
    echo Please run this script from your PlatformIO project root
    echo Current directory: %CD%
    pause
    exit /b 1
)

REM Check if partition table file exists
if not exist "partitions_two_ota.csv" (
    echo ERROR: partitions_two_ota.csv not found
    echo This file should have been created by the configuration script
    pause
    exit /b 1
)

echo Current directory: %CD%
echo.

echo Checking PlatformIO configuration...
pio project config
if %ERRORLEVEL% neq 0 (
    echo ERROR: Invalid PlatformIO configuration
    pause
    exit /b 1
)

echo.
echo This script will:
echo 1. Erase the entire ESP32 flash memory
echo 2. Build the project with OTA partition table
echo 3. Flash the new firmware with OTA partitions
echo.
echo WARNING: This will erase ALL data on your ESP32!
echo.
set /p confirm="Do you want to continue? (y/N): "
if /i not "%confirm%"=="y" (
    echo Operation cancelled by user
    pause
    exit /b 0
)

echo.
echo Step 1: Cleaning previous build...
pio run -t clean
if %ERRORLEVEL% neq 0 (
    echo ERROR: Failed to clean build
    pause
    exit /b 1
)

echo.
echo Step 2: Erasing ESP32 flash memory...
pio run -t erase
if %ERRORLEVEL% neq 0 (
    echo ERROR: Failed to erase flash. Check if device is connected.
    pause
    exit /b 1
)

echo.
echo Step 3: Building project with OTA partition table...
pio run
if %ERRORLEVEL% neq 0 (
    echo ERROR: Build failed. Check your code for errors.
    pause
    exit /b 1
)

echo.
echo Step 4: Flashing firmware with OTA partitions...
pio run -t upload
if %ERRORLEVEL% neq 0 (
    echo ERROR: Upload failed. Check device connection.
    pause
    exit /b 1
)

echo.
echo ================================
echo SUCCESS: OTA partitions configured!
echo ================================
echo.
echo Your ESP32 now has the following partitions:
echo - factory:  Initial application (1MB)
echo - ota_0:    OTA update slot 0 (1MB)  
echo - ota_1:    OTA update slot 1 (1MB)
echo - ota_data: OTA state data (8KB)
echo - nvs:      Non-volatile storage
echo - spiffs:   File system storage
echo.
echo Next steps:
echo 1. Monitor serial output: pio device monitor
echo 2. Check that OTA partitions are detected in your application
echo 3. Test OTA update functionality
echo.
echo Press any key to start monitoring (Ctrl+C to exit monitor)...
pause >nul

echo Starting serial monitor...
pio device monitor
