@echo off
echo 🚀 GRSVC1 Web Interface Deployment Helper
echo.
echo This script helps prepare files for GitHub Pages deployment
echo.

REM Check if we're in the right directory
if not exist "web\index.html" (
    echo ❌ Error: Please run this script from the GRSVC1 project root directory
    echo    (the folder containing the 'web' subfolder)
    pause
    exit /b 1
)

echo ✅ Found web interface files
echo.

REM Check for firmware binaries in firmware folder
if not exist "web\firmware\grsvc1-r1-v1.0.0.bin" (
    echo ⚠️  Warning: grsvc1-r1-v1.0.0.bin not found in web\firmware folder
    echo    You'll need to build and copy your firmware binary to:
    echo    web\firmware\grsvc1-r1-v1.0.0.bin
    echo    for the USB flash tool to work properly.
    echo.
    
    REM Check if there's a built firmware file we can copy
    if exist ".pio\build\esp32dev\firmware.bin" (
        echo 📦 Found firmware binary in PlatformIO build folder
        echo    Copying and renaming firmware binary...
        if not exist "web\firmware" mkdir "web\firmware"
        copy ".pio\build\esp32dev\firmware.bin" "web\firmware\grsvc1-r1-v1.0.0.bin"
        copy ".pio\build\esp32dev\firmware.bin" "web\firmware\grsvc1-r1-v1.0.0.ota.bin"
        echo ✅ Firmware files created:
        echo    - grsvc1-r1-v1.0.0.bin (USB flashing)
        echo    - grsvc1-r1-v1.0.0.ota.bin (OTA updates)
        echo.
    ) else (
        echo 💡 To build and prepare firmware properly, run: prepare_firmware.bat
        echo    This will:
        echo    - Clean and build with OTA partition table
        echo    - Create properly named firmware files
        echo    - Generate USB and OTA versions
        echo    Then re-run this script for deployment
        echo.
    )
) else (
    echo ✅ grsvc1-r1-v1.0.0.bin found in firmware folder
    echo.
)

if not exist "web\firmware\grsvc1-r1-v1.0.0.ota.bin" (
    echo ⚠️  Warning: OTA firmware binary not found
    if exist ".pio\build\esp32dev\firmware.bin" (
        echo    Creating OTA firmware binary...
        copy ".pio\build\esp32dev\firmware.bin" "web\firmware\grsvc1-r1-v1.0.0.ota.bin"
        echo ✅ OTA firmware binary created
    )
    echo.
) else (
    echo ✅ grsvc1-r1-v1.0.0.ota.bin found in firmware folder
    echo.
)

echo 📋 Files in web folder:
dir web /b
echo.

echo 🌐 GitHub Pages Deployment Instructions:
echo.
echo 1. Create a new GitHub repository or use existing one
echo 2. Copy all files from the 'web' folder to your repository root
echo 3. Go to repository Settings → Pages
echo 4. Set source to "Deploy from a branch"
echo 5. Select 'main' branch and '/' (root) folder
echo 6. Click Save
echo.
echo 📱 Your tools will be available at:
echo    https://yourusername.github.io/your-repo/
echo    https://yourusername.github.io/your-repo/usb-web-installer.html
echo    https://yourusername.github.io/your-repo/ble-webui.html
echo.

echo 🔧 Local Testing:
echo    To test locally, run: python -m http.server 8000
echo    Then visit: http://localhost:8000/
echo.

echo 📝 Note: The USB flash tool requires HTTPS to work properly.
echo    GitHub Pages automatically provides HTTPS, so deployment is recommended.
echo.

pause
