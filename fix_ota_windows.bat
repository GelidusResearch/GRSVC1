@echo off
echo ========================================
echo ESP32 OTA Partition Fix - Windows
echo ========================================
echo.
echo STEP 1: Checking ESP-IDF availability...
echo.

REM Check if idf.py is available
idf.py --version >nul 2>&1
if %errorlevel% neq 0 (
    echo ERROR: ESP-IDF is not available in this terminal!
    echo.
    echo SOLUTIONS:
    echo 1. Use "ESP-IDF 5.x CMD" from Start Menu
    echo 2. Run this from ESP-IDF Command Prompt
    echo 3. Activate ESP-IDF by running: C:\Espressif\esp-idf\export.bat
    echo.
    echo Please open ESP-IDF Command Prompt and run this script again.
    pause
    exit /b 1
)

echo ESP-IDF is available! Version:
idf.py --version
echo.

echo STEP 2: Checking current directory...
echo Current directory: %CD%
echo.

REM Check if we're in the right directory
if not exist "src\main.c" (
    echo ERROR: Not in the correct project directory!
    echo Please navigate to: c:\Users\mlaspina\Documents\GitHub\GRSVC1
    echo Then run this script again.
    pause
    exit /b 1
)

echo Found main.c - we're in the right directory!
echo.

echo STEP 3: Checking ESP32 connection...
echo Please ensure your ESP32 is connected via USB.
echo.
pause

echo STEP 4: Erasing flash completely...
idf.py erase-flash
if %errorlevel% neq 0 (
    echo ERROR: Failed to erase flash. Check ESP32 connection.
    pause
    exit /b 1
)

echo.
echo STEP 5: Building project...
idf.py build
if %errorlevel% neq 0 (
    echo ERROR: Build failed. Check for compilation errors.
    pause
    exit /b 1
)

echo.
echo STEP 6: Flashing bootloader, partition table, and application...
idf.py flash
if %errorlevel% neq 0 (
    echo ERROR: Flash failed. Check ESP32 connection.
    pause
    exit /b 1
)

echo.
echo ========================================
echo SUCCESS: ESP32 has been reflashed!
echo ========================================
echo.
echo The ESP32 should now have proper OTA partition support.
echo.
echo To verify, you can:
echo 1. Run "idf.py monitor" to see startup logs
echo 2. Connect via BLE and send "OTA INFO" command
echo.
echo Press any key to open monitor...
pause
idf.py monitor
