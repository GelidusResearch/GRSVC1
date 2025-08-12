import asyncio
import sys

async def scan_for_grsvc():
    try:
        from bleak import BleakScanner
        print('🔍 Scanning for GRSVC1 device...')
        
        devices = await BleakScanner.discover(timeout=10.0)
        print(f'📡 Found {len(devices)} BLE devices:')
        
        grsvc_found = False
        for device in devices:
            device_name = device.name or 'Unknown'
            print(f'  📱 {device_name} | {device.address}')
            
            if 'GRSVC' in device_name.upper():
                grsvc_found = True
                print(f'  ✅ FOUND GRSVC DEVICE: {device_name}')
                print(f'     Address: {device.address}')
                if hasattr(device, 'metadata') and device.metadata:
                    print(f'     RSSI: {device.metadata.get("rssi", "N/A")} dBm')
                    if 'uuids' in device.metadata:
                        print(f'     Services: {device.metadata["uuids"]}')
        
        if not grsvc_found:
            print('❌ No GRSVC device found!')
            print('💡 Try:')
            print('   1. Double-click reset button on nRF52840')
            print('   2. Wait 10 seconds and scan again')
            print('   3. Check device is powered on')
        
    except ImportError:
        print('❌ bleak not installed. Installing...')
        import subprocess
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'bleak'])
        print('✅ bleak installed - please run scan again')
    except Exception as e:
        print(f'❌ Error during scan: {e}')

if __name__ == "__main__":
    asyncio.run(scan_for_grsvc())
