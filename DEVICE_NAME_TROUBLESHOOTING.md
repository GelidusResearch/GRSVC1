# GRSVC1 Device Name & Connection Troubleshooting Guide

## 🚨 Device Name Not Appearing in Browser

### Quick Fixes

1. **Reset Device Advertising:**
   ```
   - Double-click the RESET button on your nRF52840
   - Wait 10 seconds for complete restart
   - Try Web Bluetooth scan again
   ```

2. **Browser Bluetooth Cache:**
   - **Chrome:** `chrome://settings/content/bluetooth` → Remove old GRSVC1 entries
   - **Edge:** `edge://settings/content/bluetooth` → Remove old GRSVC1 entries
   - Refresh web page and try again

3. **Check Device Status:**
   - Power LED should be on
   - If connected via USB, check serial monitor at 115200 baud
   - Look for: "Device name 'GRSVC1' should now appear in Web Bluetooth device picker"

### Browser Requirements
- ✅ **Chrome** v56+ (Recommended)
- ✅ **Edge** v79+ 
- ❌ **Firefox** (Limited Web Bluetooth support)
- ❌ **Safari** (No Web Bluetooth support)

### Advertising Check
Your nRF52840 firmware sets the device name in multiple places:
- `NimBLEDevice::init("GRSVC1")` - Initial BLE stack name
- `pAdvertising->setName("GRSVC1")` - Advertised name for discovery
- `deviceNameChar->setValue("GRSVC1")` - Device Info service name

## 🔌 How to Disconnect/Unpair GRSVC1

### Method 1: Web Interface (Cleanest)
1. **While Connected:**
   - Open GRSVC1 web interface
   - Click red **"Disconnect"** button
   - This properly cleans up all BLE references

### Method 2: Browser Settings
**Chrome:**
```
1. Go to: chrome://settings/content/bluetooth
2. Find "GRSVC1" in device list
3. Click "Remove" or "Forget"
4. Clear browser cache (Ctrl+Shift+Delete)
```

**Edge:**
```
1. Go to: edge://settings/content/bluetooth  
2. Find "GRSVC1" and remove
3. Clear browsing data
```

### Method 3: Operating System
**Windows 10/11:**
```
1. Settings → Bluetooth & devices
2. Find "GRSVC1" 
3. Click three dots → "Remove device"
```

**macOS:**
```
1. System Preferences → Bluetooth
2. Find "GRSVC1"
3. Click "X" next to device name
```

**iOS:**
```
1. Settings → Bluetooth
2. Find "GRSVC1"
3. Tap "i" → "Forget This Device"
```

### Method 4: Complete Reset
```
1. Power off nRF52840 device
2. Clear ALL browser data (Ctrl+Shift+Delete)
3. Restart browser completely
4. Power on device and try fresh connection
```

## 🔧 Firmware Fixes for Name Issues

If the device name still doesn't appear, try updating firmware:

### 1. Check Current Firmware
Connect to serial monitor (115200 baud) and look for:
```
BLE device active, advertising started: SUCCESS
Device name 'GRSVC1' should now appear in Web Bluetooth device picker
```

### 2. Upload Latest Firmware
```batch
# Build and upload new firmware
platformio run -e pro_micro_nrf52840 --target upload

# Or use UF2 method:
build_firmware.bat
# Then drag firmware/grsvc1-r1-v1.0.0.uf2 to nRF52BOOT drive
```

### 3. Force Advertising Restart
The firmware includes automatic advertising restart logic:
- If no clients connected for 10+ seconds
- Advertising restarts with device name
- Serial monitor shows restart status

## 🐛 Advanced Troubleshooting

### Check BLE Scan Results
Run this to see if device is advertising:
```bash
python scan_grsvc.py
```

### Serial Monitor Debug
Connect at 115200 baud and look for:
```
BLE Connected clients: 0
No advertising detected - restarting immediately...
Device name 'GRSVC1' re-advertised
```

### Force Hardware Reset
```
1. Hold RESET button for 5+ seconds
2. Release and wait 10 seconds
3. Check serial output for proper BLE initialization
4. Try browser connection again
```

### Verify Web Bluetooth API
In browser console (F12), check:
```javascript
navigator.bluetooth.getAvailability()
// Should return: true
```

## 📱 Connection Status in Web UI

The web interface shows connection status:
- 🔴 **Red:** "Device not connected" 
- 🟢 **Green:** "Connected to GRSVC1"
- **Disconnect Button:** Appears when connected

## 🔄 When to Use Each Method

| Issue | Recommended Method |
|-------|-------------------|
| Name not in device picker | Browser cache clear + device reset |
| "Already connected" error | Web UI disconnect button |
| Stale connection | OS-level device removal |
| Complete fresh start | Method 4 (Complete reset) |
| Development/testing | Serial monitor + firmware upload |

## ✅ Success Indicators

You'll know it's working when:
- "GRSVC1" appears in browser device picker
- Web UI shows green "Connected" status
- Valve controls respond immediately
- Sensor data updates in real-time

Remember: The nRF52840 firmware is designed to automatically handle connection issues and restart advertising with the proper device name when needed.
