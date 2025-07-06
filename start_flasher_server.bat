@echo off
echo 🚀 Starting ESP Web Tools Server...
echo.
echo This server provides proper CORS headers for ESP Web Tools
echo and serves your firmware files over HTTP instead of file://
echo.
echo Available URLs:
echo   ESP Web Tools: http://localhost:8000/usb-web-installer.html
echo.
echo Press Ctrl+C to stop the server
echo.
python serve_flasher.py 8000
