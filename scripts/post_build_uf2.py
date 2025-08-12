#!/usr/bin/env python3
"""
Post-build script to generate UF2 files for nRF52840 GRSVC1 firmware
This script converts the compiled .hex file to .uf2 format for easy drag-and-drop installation
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path

Import("env")

def generate_uf2(source, target, env):
    """Generate UF2 file from compiled firmware"""
    
    print("=" * 60)
    print("🔧 GRSVC1 nRF52840 UF2 Generation")
    print("=" * 60)
    
    # Get build directory and firmware name
    build_dir = env.subst("$BUILD_DIR")
    project_dir = env.subst("$PROJECT_DIR")
    firmware_name = env.subst("$PROGNAME")
    
    # File paths
    hex_file = os.path.join(build_dir, f"{firmware_name}.hex")
    bin_file = os.path.join(build_dir, f"{firmware_name}.bin")
    uf2_file = os.path.join(build_dir, f"{firmware_name}.uf2")
    
    # Output directory for firmware releases
    firmware_dir = os.path.join(project_dir, "firmware")
    os.makedirs(firmware_dir, exist_ok=True)
    
    print(f"📁 Build directory: {build_dir}")
    print(f"📁 Project directory: {project_dir}")
    print(f"📄 Firmware name: {firmware_name}")
    
    # Check if hex file exists
    if not os.path.exists(hex_file):
        print(f"❌ Error: HEX file not found: {hex_file}")
        return
    
    print(f"✅ Found HEX file: {hex_file}")
    
    # Try to find uf2conv.py in common locations
    uf2conv_paths = [
        # PlatformIO locations
        os.path.expanduser("~/.platformio/packages/tool-uf2conv/uf2conv.py"),
        # Arduino IDE locations
        os.path.expanduser("~/AppData/Local/Arduino15/packages/adafruit/tools/uf2conv/*/uf2conv.py"),
        # Manual installation
        "uf2conv.py",
        # Try system PATH
        shutil.which("uf2conv.py")
    ]
    
    uf2conv = None
    for path in uf2conv_paths:
        if path and os.path.exists(path):
            uf2conv = path
            break
        # Handle glob patterns for Arduino IDE
        if "*" in str(path):
            import glob
            matches = glob.glob(str(path))
            if matches:
                uf2conv = matches[0]
                break
    
    if not uf2conv:
        print("⚠️  uf2conv.py not found, trying manual conversion...")
        
        # Manual UF2 conversion using Python
        try:
            # Read the hex file and convert to UF2
            print("🔄 Attempting manual UF2 conversion...")
            
            # For nRF52840, we need to convert HEX to BIN first, then to UF2
            if os.path.exists(bin_file):
                print(f"✅ Found BIN file: {bin_file}")
                
                # Simple UF2 creation (this is a basic implementation)
                # For production, you should use the official uf2conv.py
                uf2_output = os.path.join(firmware_dir, "grsvc1-r1-v1.0.0.uf2")
                
                # Copy the bin file as a fallback
                shutil.copy2(bin_file, os.path.join(firmware_dir, "grsvc1-r1-v1.0.0.bin"))
                print(f"📦 Copied BIN file to: {os.path.join(firmware_dir, 'grsvc1-r1-v1.0.0.bin')}")
                
                print("⚠️  Manual UF2 conversion is limited.")
                print("📋 For proper UF2 files, install uf2conv.py:")
                print("   pip install adafruit-nrfutil")
                print("   or download from: https://github.com/microsoft/uf2/blob/master/utils/uf2conv.py")
                
        except Exception as e:
            print(f"❌ Manual conversion failed: {e}")
            
    else:
        print(f"✅ Found uf2conv.py: {uf2conv}")
        
        # Convert to UF2 using uf2conv.py
        try:
            # UF2 family ID for nRF52840
            family_id = "0xADA52840"
            
            # Run uf2conv.py
            cmd = [
                sys.executable, uf2conv,
                "--convert",
                "--family", family_id,
                "--output", uf2_file,
                hex_file
            ]
            
            print(f"🔄 Running: {' '.join(cmd)}")
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                print(f"✅ UF2 file generated: {uf2_file}")
                
                # Copy to firmware directory with version naming
                uf2_output = os.path.join(firmware_dir, "grsvc1-r1-v1.0.0.uf2")
                shutil.copy2(uf2_file, uf2_output)
                print(f"📦 UF2 file copied to: {uf2_output}")
                
            else:
                print(f"❌ uf2conv.py failed: {result.stderr}")
                
        except Exception as e:
            print(f"❌ UF2 conversion error: {e}")
    
    # Copy other files for distribution
    try:
        if os.path.exists(hex_file):
            hex_output = os.path.join(firmware_dir, "grsvc1-r1-v1.0.0.hex")
            shutil.copy2(hex_file, hex_output)
            print(f"📦 HEX file copied to: {hex_output}")
            
        if os.path.exists(bin_file):
            bin_output = os.path.join(firmware_dir, "grsvc1-r1-v1.0.0.bin")
            shutil.copy2(bin_file, bin_output)
            print(f"📦 BIN file copied to: {bin_output}")
            
    except Exception as e:
        print(f"⚠️  File copy warning: {e}")
    
    print("=" * 60)
    print("🎉 Firmware build complete!")
    print(f"📁 Firmware files available in: {firmware_dir}")
    print("📋 Installation methods:")
    print("   1. UF2: Drag grsvc1-r1-v1.0.0.uf2 to nRF52840 bootloader drive")
    print("   2. HEX: Use nrfutil or Arduino IDE")
    print("   3. BIN: Use nrfutil with --application flag")
    print("=" * 60)

# Add the post-build action
env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", generate_uf2)
