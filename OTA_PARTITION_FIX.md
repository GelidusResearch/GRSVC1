# ESP32 OTA Partition Fix Guide

## Problem
Your ESP32 is reporting "OTA: No available partition" when trying to perform firmware updates via BLE. This indicates that the device doesn't have the proper partition table for OTA (Over-The-Air) updates.

## Root Cause
The ESP32 is likely:
1. Running from a factory partition without OTA partitions configured
2. Has an old partition table that doesn't include OTA partitions
3. The partition table wasn't flashed correctly during initial programming

## Solution
You need to completely erase and reflash the ESP32 with the correct partition table.

### For PlatformIO Users (Recommended)

#### Automatic Setup:
```bash
# Run the PlatformIO setup script (Windows)
setup_ota_platformio.bat
```

This script will:
1. Verify PlatformIO configuration
2. Erase the ESP32 flash completely
3. Build with the correct OTA partition table
4. Flash the new firmware
5. Start serial monitoring to verify

#### Manual PlatformIO Commands:
```bash
# Clean previous build
pio run -t clean

# Erase flash completely
pio run -t erase

# Build with OTA partition table
pio run

# Flash the new firmware
pio run -t upload

# Monitor to verify OTA partitions
pio device monitor
```

#### Verification:
```bash
# Run verification script
verify_ota_platformio.bat
```

### For ESP-IDF Users

#### Automatic Fix (Recommended)

##### For Windows:
```bash
# Run the provided batch script
fix_ota_partitions.bat
```

##### For Linux/Mac:
```bash
# Run the provided shell script
./fix_ota_partitions.sh
```

### Manual Fix
If you prefer to run the commands manually:

1. **Erase the flash completely:**
   ```bash
   idf.py erase-flash
   ```

2. **Build the project:**
   ```bash
   idf.py build
   ```

3. **Flash everything (bootloader, partition table, application):**
   ```bash
   idf.py flash
   ```

4. **Monitor the output to verify:**
   ```bash
   idf.py monitor
   ```

## PlatformIO Configuration Details

The following files have been configured for OTA support:

### platformio.ini
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = espidf

; Partition table configuration for OTA updates
board_build.partitions = partitions_two_ota.csv
board_build.filesystem = littlefs

; Flash configuration
board_build.flash_size = 4MB
board_build.flash_mode = dio
board_build.flash_freq = 40m

; Monitor configuration
monitor_speed = 115200
monitor_port = COM4
upload_port = COM4
monitor_filters = esp32_exception_decoder
```

### partitions_two_ota.csv
Custom partition table with OTA support:
- **factory**: Initial application (1MB)
- **ota_0**: First OTA update partition (1MB)
- **ota_1**: Second OTA update partition (1MB)
- **ota_data**: OTA state data (8KB)
- **nvs**: Non-volatile storage
- **spiffs**: File system storage

For detailed PlatformIO configuration guide, see: `PLATFORMIO_OTA_GUIDE.md`

## Verification
After reflashing, you should see in the startup logs:

```
I (xxx) GRSVC1: Current running partition: ota_0 at offset 0x10000, size: xxxxx bytes
I (xxx) GRSVC1: Next OTA partition: ota_1 at offset 0xxxxxx, size: xxxxx bytes
I (xxx) GRSVC1: OTA update system ready
```

## Testing OTA Functionality
1. Connect to the BLE device
2. Send "OTA INFO" command to the control characteristic (UUID: 0x2A9F)
3. Check the logs for comprehensive partition information
4. The web interface should now work for OTA updates

## Partition Table Configuration
Your `sdkconfig.esp32dev` is correctly configured with:
- `CONFIG_PARTITION_TABLE_TWO_OTA=y`
- `CONFIG_PARTITION_TABLE_FILENAME="partitions_two_ota.csv"`

This creates:
- **factory**: Factory application partition (fallback)
- **ota_0**: First OTA partition
- **ota_1**: Second OTA partition
- **nvs**: Non-volatile storage
- **phy_init**: PHY initialization data

## Enhanced Diagnostics
The firmware now includes enhanced partition diagnostics:

### BLE Commands:
- **"OTA INFO"**: Comprehensive partition analysis
- **"STATUS"**: General device status
- **"RESTART ADV"**: Force BLE advertising restart

### Startup Diagnostics:
The device will automatically log partition information on startup and warn about any OTA-related issues.

## Important Notes
- This process will **erase all data** on the ESP32
- Make sure the ESP32 is properly connected via USB
- Ensure your ESP-IDF environment is properly set up
- The device should be in the project directory when running commands

## Troubleshooting
If you still have issues after reflashing:
1. Check that the ESP-IDF version supports your ESP32 variant
2. Verify the USB connection and drivers
3. Try a different USB cable or port
4. Check for hardware issues with the ESP32 module

## Support
If problems persist, please share the complete startup logs (from `idf.py monitor`) for further analysis.
