#!/bin/bash

echo "========================================"
echo "ESP32 OTA Partition Fix Script"
echo "========================================"
echo
echo "This script will completely erase and reflash your ESP32"
echo "with the correct partition table for OTA updates."
echo
echo "IMPORTANT: This will erase ALL data on the ESP32!"
echo
echo "Before running this script, ensure:"
echo "1. ESP32 is connected via USB"
echo "2. ESP-IDF environment is set up"
echo "3. You are in the project directory"
echo
read -p "Press Enter to continue or Ctrl+C to cancel..."

echo
echo "Step 1: Completely erasing flash..."
if ! idf.py erase-flash; then
    echo "ERROR: Failed to erase flash. Check connection and try again."
    exit 1
fi

echo
echo "Step 2: Building project..."
if ! idf.py build; then
    echo "ERROR: Build failed. Check for compilation errors."
    exit 1
fi

echo
echo "Step 3: Flashing bootloader, partition table, and application..."
if ! idf.py flash; then
    echo "ERROR: Flash failed. Check connection and try again."
    exit 1
fi

echo
echo "Step 4: Opening monitor to verify OTA partitions..."
echo "Look for partition information in the startup logs."
echo "Use Ctrl+] to exit monitor."
echo
idf.py monitor

echo
echo "========================================"
echo "Flash complete!"
echo "========================================"
echo
echo "The ESP32 should now have proper OTA partition support."
echo "Check the startup logs for partition information."
echo
echo "To test OTA functionality:"
echo "1. Connect to BLE device"
echo "2. Send 'OTA INFO' command to control characteristic (0x2A9F)"
echo "3. Check logs for partition details"
echo
