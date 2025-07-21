# GRSVC1 Web Interface

This folder contains the web interface files for the GRSVC1 device, designed to be served from GitHub Pages.

## Files

- `index.html` - Main menu page with tool selection
- `usb-web-installer.html` - USB serial flash tool using ESP Web Tools
- `ble-webui.html` - Bluetooth Low Energy web interface
- `manifest.json` - ESP Web Tools manifest for firmware flashing
- `firmware/` - Firmware binaries directory
  - `grsvc1-r1-v1.0.0.bin` - Full firmware for USB flashing
  - `grsvc1-r1-v1.0.0.ota.bin` - OTA update binary for wireless updates

## GitHub Pages Setup

1. **Repository Setup:**
   - Create a new GitHub repository or use existing one

2. **Enable GitHub Pages:**
   - Go to repository Settings → Pages
   - Set source to "Deploy from a branch"
   - Select `main` branch and `/` (root) or `/docs` folder
   - Click Save

3. **Add Firmware Binary:**
   - Build your GRSVC1 firmware using PlatformIO: `pio run`
   - Copy the generated firmware to: `firmware/grsvc1-r1-v1.0.0.bin`
   - For OTA updates, also copy to: `firmware/grsvc1-r1-v1.0.0.ota.bin`
   - Commit and push to GitHub

4. **Access Your Tools:**
   - USB Flash Tool: `https://yourusername.github.io/your-repo/usb-web-installer.html`
   - BLE Interface: `https://yourusername.github.io/your-repo/ble-webui.html`
   - Main Menu: `https://yourusername.github.io/your-repo/` (index.html)

## Requirements

### USB Flash Tool
- **Browser:** Chrome 89+, Edge 89+, or Opera 75+ (desktop only)
- **Connection:** USB cable to ESP32 device
- **Permissions:** Serial port access when prompted
- **Protocol:** HTTPS required (automatically provided by GitHub Pages)

### BLE Interface
- **Browser:** Chrome 56+, Edge 79+, Opera 43+ (desktop/mobile)
- **Device:** ESP32 with GRSVC1 firmware running
- **Permissions:** Bluetooth access when prompted
- **Protocol:** HTTPS required (automatically provided by GitHub Pages)

## Local Development

To test locally before deploying:

1. **Start Local Server:**
   ```bash
   # Python 3
   python -m http.server 8000
   
   # Python 2
   python -m SimpleHTTPServer 8000
   
   # Node.js
   npx http-server -p 8000
   ```

2. **Access Tools:**
   - Main Menu: `http://localhost:8000/`
   - USB Flash: `http://localhost:8000/usb-web-installer.html`
   - BLE Interface: `http://localhost:8000/ble-webui.html`

## Features

### USB Flash Tool
- One-click firmware installation
- Automatic ESP32 detection
- Progress monitoring
- No additional software required

### BLE Interface
- Wireless device connection
- Real-time sensor monitoring
- Device control interface
- OTA firmware updates
- Flow sensor readings
- Battery level monitoring
- Temperature and humidity data
- Valve and power control

## Troubleshooting

### USB Flash Tool Issues
- Ensure Chrome/Edge 89+ is being used
- Check USB cable connection
- Install ESP32 drivers if needed
- Put device in programming mode if required

### BLE Interface Issues
- Enable Bluetooth on your device
- Ensure GRSVC1 is powered and advertising
- Clear browser cache if connection fails
- Check browser console for error messages

### GitHub Pages Issues
- Ensure HTTPS is being used
- Check that all files are uploaded correctly
- Verify firmware.bin file is present and accessible
- Check GitHub Pages deployment status

## Security Notes

- GitHub Pages automatically provides HTTPS
- Web Serial API requires secure context (HTTPS)
- Web Bluetooth API requires secure context (HTTPS)
- No sensitive data is transmitted or stored

## License

Copyright © 2025 Gelidus Research Inc.
