# PlatformIO OTA Configuration Guide

## Overview
This guide explains how to configure PlatformIO for ESP32 OTA (Over-The-Air) updates using a custom partition table.

## Files Modified/Created

### 1. platformio.ini
The main configuration file has been updated with:
- Custom partition table (`partitions_two_ota.csv`)
- Flash configuration optimized for OTA
- Monitor filters for better debugging

### 2. partitions_two_ota.csv
Custom partition table that includes:
- **factory**: Initial application partition (1MB)
- **ota_0**: First OTA update partition (1MB)
- **ota_1**: Second OTA update partition (1MB)
- **ota_data**: OTA state data (8KB)
- **nvs**: Non-volatile storage for configuration
- **spiffs**: File system for data storage

## Building and Flashing

### Clean Build and Flash
```bash
# Clean previous build
pio run -t clean

# Build with new partition table
pio run

# Flash the device (this will erase and flash with new partition table)
pio run -t upload

# Monitor serial output
pio device monitor
```

### Verify Partition Table
After flashing, the device should show partition information in the serial monitor. Look for:
```
Available OTA partitions:
- ota_0: 0x110000 (1048576 bytes)
- ota_1: 0x210000 (1048576 bytes)
```

## Troubleshooting

### Issue: "No available partition" error
**Cause**: Device was flashed with single-app partition table.
**Solution**: 
1. Erase the entire flash: `pio run -t erase`
2. Flash with new partition table: `pio run -t upload`

### Issue: Build fails with partition table errors
**Cause**: Partition table doesn't fit in available flash space.
**Solution**: Check that total partition sizes don't exceed your ESP32's flash size (usually 4MB).

### Issue: OTA update fails during runtime
**Cause**: Application tries to write to wrong partition.
**Solution**: 
1. Verify `esp_ota_get_next_update_partition()` returns valid partition
2. Check that OTA data partition is properly initialized
3. Ensure sufficient free heap memory for OTA operations

## Partition Table Details

| Partition | Type | Size | Purpose |
|-----------|------|------|---------|
| nvs | data | 24KB | Configuration storage |
| phy_init | data | 4KB | RF calibration data |
| factory | app | 1MB | Initial application |
| ota_0 | app | 1MB | OTA update slot 0 |
| ota_1 | app | 1MB | OTA update slot 1 |
| ota_data | data | 8KB | OTA state information |
| spiffs | data | 952KB | File system |

## OTA Update Process

1. **Boot**: Device starts from `factory` partition
2. **Update Check**: Application checks for new firmware
3. **Download**: New firmware downloaded to inactive OTA partition
4. **Verify**: Downloaded firmware is verified
5. **Switch**: OTA data updated to boot from new partition
6. **Reboot**: Device reboots to new firmware

## Important Notes

- Always test OTA updates in a development environment first
- Keep a backup of your working firmware
- Monitor available heap memory during OTA operations
- Consider implementing rollback mechanism for failed updates
- The factory partition serves as a fallback if OTA partitions are corrupted

## Commands Reference

```bash
# Clean build
pio run -t clean

# Build only
pio run

# Upload/flash
pio run -t upload

# Erase flash completely
pio run -t erase

# Monitor serial output
pio device monitor

# Upload and monitor
pio run -t upload -t monitor

# List available partitions (after flashing)
# Check serial monitor output from your application
```

## Next Steps

1. Flash the device with the new partition table
2. Test that OTA partition detection works
3. Implement your OTA update logic
4. Test OTA updates in a controlled environment
