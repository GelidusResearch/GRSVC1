# GRSVC1 Web Interface - GitHub Pages Deployment

## Summary

I've created a complete web interface setup for your GRSVC1 device that's ready for GitHub Pages deployment. The interface provides a professional menu system with two main tools:

## Created Files

### 📁 `/web/` folder structure:
- `index.html` - Main menu page with tool selection and device information
- `usb-web-installer.html` - ESP Web Tools integration for USB firmware flashing
- `ble-webui.html` - Bluetooth Low Energy web interface (updated with navigation)
- `manifest.json` - ESP Web Tools manifest for firmware installation
- `firmware/` - Firmware binaries directory
  - `grsvc1-r1-v1.0.0.bin` - Full firmware for USB flashing
  - `grsvc1-r1-v1.0.0.ota.bin` - OTA update binary for wireless updates
- `README.md` - Complete setup and troubleshooting documentation
- `_config.yml` - Jekyll configuration for GitHub Pages

### 🔧 Project root files:
- `deploy_web.bat` - Deployment helper script

## Key Features

### 🏠 Main Menu (`index.html`)
- Professional landing page with device information
- Tool selection cards with descriptions
- Browser compatibility warnings
- Device specifications and feature lists
- Responsive design for desktop and mobile

### 🔌 USB Flash Tool (`usb-web-installer.html`)
- ESP Web Tools integration for direct firmware flashing
- Step-by-step installation guide
- Browser compatibility checking
- Real-time status updates and debugging
- HTTPS requirement detection
- Automatic manifest and firmware file validation

### 📶 BLE Interface (`ble-webui.html`)
- Updated with navigation back to main menu
- Fixed CSS color issue (#434k54b → #434343)
- All original functionality preserved
- Wireless device connection and control

## Next Steps

### 1. Build Firmware
```bash
pio run
```

### 2. Copy Firmware Binary
```bash
# After building with: pio run
copy ".pio\build\esp32dev\firmware.bin" "web\firmware\grsvc1-r1-v1.0.0.bin"
copy ".pio\build\esp32dev\firmware.bin" "web\firmware\grsvc1-r1-v1.0.0.ota.bin"
```

### 3. Deploy to GitHub Pages
1. Create GitHub repository
2. Upload all files from `/web/` folder to repository root
3. Enable GitHub Pages in repository settings
4. Access your tools at `https://yourusername.github.io/your-repo/`

### 4. Test Locally (Optional)
```bash
cd web
python -m http.server 8000
# Visit http://localhost:8000/
```

## Corrections Made

### ✅ USB Flash Tool Corrections:
- Updated for GitHub Pages HTTPS requirement
- Added comprehensive browser compatibility checking
- Improved error handling and user feedback
- Added step-by-step installation guide
- Enhanced manifest validation

### ✅ BLE Interface Corrections:
- Fixed CSS syntax error in color value
- Added navigation back to main menu
- Preserved all original functionality
- Enhanced mobile responsiveness

### ✅ General Improvements:
- Professional UI with consistent branding
- Clear device information and specifications
- Comprehensive documentation
- Deployment automation script
- Jekyll configuration for optimal GitHub Pages hosting

## Firmware Preparation Requirements

### 🔧 Important: Firmware Must Be Properly Prepared

Before deployment, you **must** prepare the firmware correctly for both OTA and USB deployment:

#### Automated Preparation (Recommended)
```bash
prepare_firmware.bat
```

This script:
- Cleans previous builds
- Builds with OTA partition table (`partitions_two_ota.csv`)
- Creates properly named firmware files:
  - `grsvc1-r1-v1.0.0.bin` (USB flashing)
  - `grsvc1-r1-v1.0.0.ota.bin` (OTA updates)
- Generates firmware manifest
- Copies all necessary files to `web/firmware/`

#### Why This Matters
- **OTA Support**: Enables wireless firmware updates via BLE
- **Dual Partitions**: Uses ota_0 and ota_1 for safe updates
- **First Flash**: Must be via USB to establish partition table
- **Future Updates**: Can be done wirelessly after first USB flash

#### Manual Preparation
```bash
pio run -t clean
pio run
# Then copy and rename firmware files as needed
```

## Browser Requirements

### USB Flash Tool:
- Chrome 89+, Edge 89+, Opera 75+ (desktop only)
- HTTPS connection (automatically provided by GitHub Pages)
- Serial port permissions

### BLE Interface:
- Chrome 56+, Edge 79+, Opera 43+ (desktop/mobile)
- HTTPS connection (automatically provided by GitHub Pages)  
- Bluetooth permissions

## URLs After Deployment

- **Main Menu**: `https://yourusername.github.io/your-repo/`
- **USB Flash**: `https://yourusername.github.io/your-repo/usb-web-installer.html`
- **BLE Interface**: `https://yourusername.github.io/your-repo/ble-webui.html`

## Support

All tools now include comprehensive error handling, status updates, and troubleshooting information. The main menu provides clear guidance on browser requirements and device compatibility.

The web interface is now production-ready for GitHub Pages deployment with professional presentation and full functionality for both USB flashing and BLE device interaction.
