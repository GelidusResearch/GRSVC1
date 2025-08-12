#!/usr/bin/env python3
"""
BLE Device Scanner for GRSVC1 Troubleshooting
This script scans for BLE devices and checks for GRSVC1 specifically
"""

import asyncio
import bleak
from bleak import BleakScanner
import logging

# Enable debug logging
logging.basicConfig(level=logging.INFO)

async def scan_for_devices():
    print("🔍 Scanning for BLE devices...")
    print("Make sure your nRF52840 GRSVC1 device is powered on and running!")
    print("-" * 60)
    
    try:
        # Scan for 10 seconds
        devices = await BleakScanner.discover(timeout=10.0)
        
        print(f"Found {len(devices)} BLE devices:")
        print("-" * 60)
        
        grsvc_found = False
        
        for i, device in enumerate(devices, 1):
            name = device.name or "Unknown"
            address = device.address
            rssi = device.rssi if hasattr(device, 'rssi') else "N/A"
            
            # Check if this is our GRSVC1 device
            is_grsvc = "GRSVC" in name.upper() if name else False
            
            if is_grsvc:
                print(f"🎯 ** GRSVC DEVICE FOUND! **")
                grsvc_found = True
            
            print(f"{i:2d}. Name: '{name}'")
            print(f"    Address: {address}")
            print(f"    RSSI: {rssi} dBm")
            
            # Print service UUIDs if available
            if hasattr(device, 'metadata') and device.metadata.get('uuids'):
                uuids = device.metadata['uuids']
                print(f"    Services: {len(uuids)} found")
                for uuid in uuids[:3]:  # Show first 3 UUIDs
                    print(f"      - {uuid}")
                if len(uuids) > 3:
                    print(f"      ... and {len(uuids) - 3} more")
            
            print("-" * 40)
        
        if not grsvc_found:
            print("❌ No GRSVC1 device found!")
            print("\n🔧 Troubleshooting suggestions:")
            print("1. Make sure your nRF52840 device is powered on")
            print("2. Check that the firmware is uploaded and running")
            print("3. Look at the serial output for BLE initialization messages")
            print("4. Verify the device name is set to 'GRSVC1' in the firmware")
            print("5. Try pressing the reset button on the nRF52840")
        else:
            print("✅ GRSVC1 device is advertising successfully!")
            
    except Exception as e:
        print(f"❌ Error during BLE scan: {e}")
        print("\nPossible issues:")
        print("- Bluetooth is not enabled on this computer")
        print("- bleak library is not installed (pip install bleak)")
        print("- BLE adapter permission issues")

if __name__ == "__main__":
    try:
        asyncio.run(scan_for_devices())
    except KeyboardInterrupt:
        print("\n⏹️  Scan cancelled by user")
    except Exception as e:
        print(f"❌ Script error: {e}")
