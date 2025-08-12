# GRSVC1 nRF52840 Firmware Installation Guide

## 🔧 Building the Firmware

### Option 1: Quick Build
Run the build script:
```batch
build_firmware.bat
```

### Option 2: Release Build (UF2 only)
```batch
build_release.bat
```

### Option 3: Manual PlatformIO Build
```bash
# Regular build with upload capability
platformio run -e pro_micro_nrf52840

# UF2-only build (no upload)
platformio run -e pro_micro_nrf52840_uf2
```

## 📦 Generated Files

After building, you'll find these files in the `firmware/` directory:

- **`grsvc1-r1-v1.0.0.uf2`** - Drag-and-drop installation file
- **`grsvc1-r1-v1.0.0.hex`** - For nrfutil or Arduino IDE
- **`grsvc1-r1-v1.0.0.bin`** - Raw binary (advanced users)

## 🚀 Installation Methods

### Method 1: UF2 Drag-and-Drop (Recommended)

1. **Enter Bootloader Mode:**
   - Double-click the RESET button on your nRF52840 board
   - The STATUS LED should start pulsing
   - A drive named "nRF52BOOT" should appear on your computer

2. **Install Firmware:**
   - Drag `firmware/grsvc1-r1-v1.0.0.uf2` to the "nRF52BOOT" drive
   - The device will automatically restart with the new firmware

3. **Verify Installation:**
   - The device should start advertising as "GRSVC1"
   - Connect using the web interface or iOS app

### Method 2: nrfutil (Advanced)

```bash
# Install nrfutil (if not already installed)
pip install nrfutil

# Flash the firmware
nrfutil dfu genpkg --dev-type 0x0052 --application firmware/grsvc1-r1-v1.0.0.hex firmware/grsvc1-r1-v1.0.0_dfu.zip
nrfutil dfu serial --package firmware/grsvc1-r1-v1.0.0_dfu.zip --port COM7 --baudrate 115200
```

### Method 3: Arduino IDE

1. Open Arduino IDE
2. Select your nRF52840 board
3. Use "Sketch" → "Upload Using Programmer"
4. Select the generated .hex file

## 🔍 Troubleshooting

### Device Name Not Showing Up in Browser

**Technical Explanation:** Web Bluetooth has stricter requirements for device name display than native Bluetooth. The updated firmware now includes enhanced advertising data with the device name in multiple locations (advertisement data + scan response data) to maximize Web Bluetooth compatibility.

**Note:** If "GRSVC1" appears on iOS/Android devices but not in your browser, the device is working correctly - this is a Web Bluetooth cache issue.

If "GRSVC1" doesn't appear in the Web Bluetooth device picker, or you get "Must be handling a user gesture" errors:

1. **Upload Latest Firmware (Recommended First Step):**
   ```bash
   # Build and upload enhanced advertising firmware
   platformio run -e pro_micro_nrf52840 --target upload
   ```
   The latest firmware includes enhanced advertising data specifically for Web Bluetooth compatibility.

2. **Clear Browser Bluetooth Cache:**
   - Chrome: Go to `chrome://settings/content/bluetooth`
   - Click "Remove" next to **ANY** old GRSVC1 entries
   - **Also remove "Unknown or Unsupported Device" entries**
   - Refresh your web page and try again
   - This fixes 90% of Web Bluetooth issues including user gesture errors

3. **Always Click Connect Button:**
   - Don't wait for automatic connection
   - **Click the "Connect" button** to initiate pairing
   - Browser requires user click for Bluetooth permissions

3. **If Name Still Doesn't Show - Use "Show All Devices":**
   - When the Web UI shows "Showing all devices" message
   - Look for devices that might be your nRF52840:
     - **"Unknown or Unsupported Device" with MAC starting with C4:77** (Nordic nRF52840)
     - Any device with a similar MAC address pattern
     - Devices that were working on your iPad
     - "Pro Micro" or nRF52840-related names
   - **✅ Connect to "Unknown or Unsupported Device" - this is likely your GRSVC1!**
   - The Web UI will verify it's GRSVC1 after connection
   - Look for "✅ Confirmed device name: GRSVC1" in the log after connection

4. **Check Web Bluetooth API:**
   - Open browser console (F12)
   - Type: `navigator.bluetooth.getAvailability()`
   - Should return `true` - if not, enable Bluetooth in Windows

5. **Browser Compatibility:**
   - Use Chrome (v56+) or Edge (v79+) browsers only
   - Firefox and Safari don't support Web Bluetooth fully
   - Make sure Bluetooth is enabled in browser settings

6. **Only if above fails - Reset Device:**
   - Double-click the reset button on your nRF52840
   - Wait 5-10 seconds for full restart
   - Check serial monitor for "Device name 'GRSVC1' should now appear"

### How to Unpair/Disconnect

#### Method 1: Using Web Interface
1. Open the web UI while connected
2. Click the red **"Disconnect"** button
3. This cleanly disconnects and clears all references

#### Method 2: Browser Bluetooth Settings
1. **Chrome:** 
   - Go to `chrome://settings/content/bluetooth`
   - Find "GRSVC1" in the list
   - Click "Remove" or "Forget"

2. **Edge:**
   - Go to `edge://settings/content/bluetooth`
   - Remove GRSVC1 device from list

#### Method 3: Operating System Level
1. **Windows:**
   - Settings → Bluetooth & devices
   - Find "GRSVC1" and click "Remove device"

2. **macOS:**
   - System Preferences → Bluetooth
   - Find "GRSVC1" and click "X" to forget

3. **iOS:**
   - Settings → Bluetooth
   - Tap "i" next to GRSVC1 → "Forget This Device"

#### Method 4: Full Reset
1. Power off the nRF52840 device
2. Clear browser cache (Ctrl+Shift+Delete)
3. Restart browser
4. Power on device and try fresh connection

### UF2 Drive Not Appearing
- Try different USB cables
- Hold RESET button for 2+ seconds, then release
- Check if Windows recognizes the device in Device Manager

### Build Errors
- Make sure PlatformIO is installed and updated
- Check that all libraries are properly installed
- Verify the COM port in `platformio.ini`

### Connection Issues After Pairing
- Verify the device is advertising as "GRSVC1"
- Use Chrome or Edge browser for Web Bluetooth
- Check that Bluetooth is enabled on your device
- Try the unpair/pair process above if connection is unstable

## 📋 Firmware Features

- **BLE Services:** Battery, Environmental, Flow, Valve Control, Device Info
- **Web Bluetooth:** Compatible with Chrome/Edge browsers
- **iOS Compatible:** Works with Core Bluetooth apps
- **Valve Control:** Dual H-bridge valve control
- **Sensor Monitoring:** DHT22 temperature/humidity, flow counting
- **Power Management:** Optimized for battery operation

## 🔄 Updating Firmware

To update to a newer version:

1. Build the new firmware using the build scripts
2. Follow the same installation process
3. The new firmware will overwrite the old version
4. All settings and pairings may need to be reset

## 🛠️ Development

For development and debugging:

```bash
# Build and upload to connected device
platformio run -e pro_micro_nrf52840 --target upload

# Monitor serial output
platformio device monitor -e pro_micro_nrf52840
```

## 📞 Support

If you encounter issues:

1. Check the serial monitor output for error messages
2. Verify all connections and hardware
3. Try a clean rebuild: `platformio run -e pro_micro_nrf52840 --target clean`
4. Check that all required libraries are installed
