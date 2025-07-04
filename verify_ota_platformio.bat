@echo off
REM PlatformIO OTA Verification Script
REM This script builds and uploads to check OTA partition configuration

echo ================================
echo PlatformIO OTA Verification
echo ================================
echo.

REM Check if PlatformIO is available
where pio >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo ERROR: PlatformIO CLI not found in PATH
    pause
    exit /b 1
)

REM Check if we're in the correct directory
if not exist "platformio.ini" (
    echo ERROR: platformio.ini not found in current directory
    pause
    exit /b 1
)

echo Building and uploading to verify OTA configuration...
echo.

echo Step 1: Building project...
pio run
if %ERRORLEVEL% neq 0 (
    echo ERROR: Build failed
    pause
    exit /b 1
)

echo.
echo Step 2: Uploading firmware...
pio run -t upload
if %ERRORLEVEL% neq 0 (
    echo ERROR: Upload failed
    pause
    exit /b 1
)

echo.
echo Step 3: Starting serial monitor to check OTA partitions...
echo Look for partition information in the output.
echo Press Ctrl+C to exit monitor when done.
echo.
pause

pio device monitor
