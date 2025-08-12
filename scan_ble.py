#!/usr/bin/env python3
import asyncio
import sys

try:
    from bleak import BleakScanner
    print('Bleak available - scanning for BLE devices...')
except ImportError:
    print('Bleak not installed. Installing...')
    import subprocess
    subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'bleak'])
    from bleak import BleakScanner
    print('Bleak installed - scanning for BLE devices...')

async def scan_devices():
    print('Starting BLE scan...')
    devices = await BleakScanner.discover(timeout=10.0)
    
    print(f'Found {len(devices)} BLE devices:')
    for device in devices:
        print(f'  Name: {device.name or "Unknown"} | Address: {device.address}')
        if device.name and ('GRSVC' in device.name.upper()):
            print(f'    *** FOUND GRSVC DEVICE ***')
            print(f'    Advertisement data: {device.metadata}')
            if hasattr(device, 'metadata') and device.metadata:
                for key, value in device.metadata.items():
                    print(f'      {key}: {value}')

if __name__ == "__main__":
    asyncio.run(scan_devices())
