#!/usr/bin/env python3
"""
GRSVC1 Device Name Verification Script
This script scans for BLE devices and specifically looks for GRSVC1 by name
"""

import asyncio
from bleak import BleakScanner

async def scan_for_grsvc1():
    print("🔍 Scanning for GRSVC1 device by name...")
    print("Make sure your updated nRF52840 firmware is running!")
    print("-" * 60)
    
    try:
        # Scan for devices
        devices = await BleakScanner.discover(timeout=10.0)
        
        print(f"Found {len(devices)} BLE devices:")
        print("-" * 60)
        
        grsvc_found = False
        grsvc_count = 0
        
        for i, device in enumerate(devices, 1):
            name = device.name or "Unknown"
            address = device.address
            rssi = getattr(device, 'rssi', 'N/A')
            
            # Check if this is our GRSVC1 device
            is_grsvc = "GRSVC" in name.upper() if name else False
            
            if is_grsvc:
                print(f"🎯 ** GRSVC DEVICE FOUND! **")
                print(f"   ✅ Name: '{name}' (PERFECT!)")
                print(f"   📍 Address: {address}")
                print(f"   📶 RSSI: {rssi} dBm")
                grsvc_found = True
                grsvc_count += 1
            else:
                print(f"{i:2d}. Name: '{name}' | Address: {address} | RSSI: {rssi} dBm")
        
        print("-" * 60)
        
        if grsvc_found:
            print(f"✅ SUCCESS: Found {grsvc_count} GRSVC device(s)!")
            print("🌐 The device name should now appear properly in Web Bluetooth!")
            print("🔗 Try connecting with the web interface - 'GRSVC1' should be visible in the device picker")
        else:
            print("❌ No GRSVC1 device found by name")
            print("\n🔧 Troubleshooting:")
            print("1. Make sure you uploaded the updated firmware to your nRF52840")
            print("2. Reset/restart your nRF52840 device")
            print("3. Check serial monitor for 'Device name GRSVC1 should now appear' message")
            print("4. Verify the firmware is running (should see debug messages every second)")
            
    except Exception as e:
        print(f"❌ Error during BLE scan: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(scan_for_grsvc1())
    except KeyboardInterrupt:
        print("\n⏹️  Scan cancelled by user")
    except Exception as e:
        print(f"❌ Script error: {e}")
        print("\nMake sure 'bleak' is installed: pip install bleak")
