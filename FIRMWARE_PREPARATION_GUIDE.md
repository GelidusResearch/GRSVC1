# GRSVC1 Firmware Preparation Guide

## Overview

The GRSVC1 firmware requires specific preparation for both OTA (Over-The-Air) and USB deployment. This guide explains the process and important considerations.

## Key Requirements

### 1. OTA Partition Table
- **Purpose**: Enables firmware updates via BLE without USB cable
- **Configuration**: Uses `partitions_two_ota.csv` with factory, ota_0, and ota_1 partitions
- **First Flash**: Must be done via USB to set up partition table correctly

### 2. Firmware Variants
- **USB Firmware**: `grsvc1-r1-v1.0.0.bin` - Complete firmware image for initial flashing
- **OTA Firmware**: `grsvc1-r1-v1.0.0.ota.bin` - Application-only image for updates

## Preparation Process

### Automated Preparation (Recommended)
```bash
# Run the complete preparation script
prepare_firmware.bat
```

This script:
1. Cleans previous builds
2. Builds with OTA partition table
3. Creates properly named firmware files
4. Generates firmware manifest
5. Copies all necessary files

### Manual Preparation
```bash
# Clean previous build
pio run -t clean

# Build with OTA partition table
pio run

# Copy and rename firmware files
copy ".pio\build\esp32dev\firmware.bin" "web\firmware\grsvc1-r1-v1.0.0.bin"
copy ".pio\build\esp32dev\firmware.bin" "web\firmware\grsvc1-r1-v1.0.0.ota.bin"
```

## Partition Table Configuration

### Current Configuration (`partitions_two_ota.csv`)
```
# Name,   Type, SubType, Offset,   Size,     Flags
nvs,      data, nvs,     0x9000,   0x6000,
phy_init, data, phy,     0xf000,   0x1000,
factory,  app,  factory, 0x10000,  0x100000,
ota_0,    app,  ota_0,   0x110000, 0x100000,
ota_1,    app,  ota_1,   0x210000, 0x100000,
ota_data, data, ota,     0x310000, 0x2000,
spiffs,   data, spiffs,  0x312000, 0xEE000,
```

### Partition Explanation
- **nvs**: Non-volatile storage for configuration
- **phy_init**: WiFi/BLE PHY initialization data
- **factory**: Factory firmware (fallback)
- **ota_0**: Primary OTA partition
- **ota_1**: Secondary OTA partition (ping-pong updates)
- **ota_data**: OTA metadata and active partition info
- **spiffs**: File system for web assets

## Deployment Considerations

### USB Flashing (Initial Setup)
- **File**: `grsvc1-r1-v1.0.0.bin`
- **Purpose**: Complete firmware with partition table
- **Method**: ESP Web Tools via USB cable
- **Requirements**: Chrome/Edge 89+, HTTPS connection

### OTA Updates (Wireless)
- **File**: `grsvc1-r1-v1.0.0.ota.bin`
- **Purpose**: Application-only updates
- **Method**: BLE interface
- **Requirements**: Device already flashed with OTA partitions

## Important Notes

### First Flash Requirements
1. **Must use USB**: Initial flash must be via USB to set up partitions
2. **Complete erase**: Recommended to erase flash completely first
3. **Partition table**: Factory firmware establishes OTA partition structure

### OTA Update Process
1. **Dual partitions**: Updates alternate between ota_0 and ota_1
2. **Rollback**: Failed updates can rollback to previous version
3. **Validation**: Firmware validates before marking as active

### File Size Considerations
- **USB firmware**: ~1MB (complete image)
- **OTA firmware**: ~1MB (application only, same size in this config)
- **Partition limit**: 1MB per OTA partition (0x100000 bytes)

## Troubleshooting

### "No OTA partition available" Error
- **Cause**: Device not flashed with OTA partition table
- **Solution**: Flash via USB first using `grsvc1-r1-v1.0.0.bin`

### OTA Update Fails
- **Check**: Firmware size doesn't exceed 1MB
- **Verify**: Device has stable power during update
- **Retry**: OTA system supports automatic retry

### USB Flashing Issues
- **Driver**: Ensure ESP32 USB drivers are installed
- **Mode**: May need to hold BOOT button while pressing RESET
- **Cable**: Use data cable (not charge-only)

## Build Environment

### PlatformIO Configuration
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = espidf
board_build.partitions = partitions_two_ota.csv
board_build.filesystem = littlefs
board_build.flash_size = 4MB
```

### ESP-IDF Configuration
- **Flash size**: 4MB
- **Partition table**: Custom OTA table
- **Filesystem**: LittleFS
- **OTA support**: Enabled

## Security Considerations

### OTA Security
- **Validation**: Firmware signature validation (if enabled)
- **Rollback**: Automatic rollback on failed boot
- **Authentication**: BLE connection authentication

### USB Security
- **Erase**: New install prompts for complete erase
- **Verification**: ESP Web Tools validates firmware before flash

## Version Management

### Naming Convention
- **Format**: `grsvc1-r{revision}-v{version}.{type}.bin`
- **Example**: `grsvc1-r1-v1.0.0.bin` (USB), `grsvc1-r1-v1.0.0.ota.bin` (OTA)
- **Revision**: Hardware revision (r1, r2, etc.)
- **Version**: Firmware version (1.0.0, 1.0.1, etc.)

### Version Tracking
- **Manifest**: `web/firmware/manifest.json` contains version info
- **Code**: Version defined in `main.c` as `FIRMWARE_VERSION`
- **Build**: Version embedded in firmware binary
