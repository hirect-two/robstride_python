#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride CAN Bus Scanner (Python Version)

Usage: python3 scan_bus.py [channel]
Examples: 
    sudo python3 scan_bus.py can0
    sudo python3 scan_bus.py can1

This script will ping all IDs in the range 1 to 254 and report responding motors.
**Note:** Accessing CAN hardware usually requires 'sudo' privileges.
"""

import sys
import os
import time

# --- Import SDK ---
# Assume this script is in the same directory as position_control_mit.py
# (i.e., the parent directory is the SDK root)
try:
    # Try to add SDK root directory to path
    sdk_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)
    
    # 1. Try to import from installed package
    from robstride_dynamics import RobstrideBus
except ImportError:
    # 2. Try to import from local files (if SDK is not installed)
    try:
        print("Package 'robstride_dynamics' not found, trying to import from local files...")
        from bus import RobstrideBus
    except ImportError as e:
        print(f"❌ Failed to import RobstrideBus SDK: {e}")
        print("Please ensure the parent directory of this script is the SDK root directory,")
        print("or the SDK has been installed via 'pip install -e .'.")
        sys.exit(1)

def main():
    # --- 1. Get CAN channel ---
    if len(sys.argv) > 1:
        channel = sys.argv[1]
    else:
        channel = "can0"

    print(f"🚀 RobStride Bus Scanner")
    print(f"📡 Scanning channel: {channel}")
    print(f"🔍 Search range: ID 1 to 254")
    print("...")
    time.sleep(1) # Pause for user to read

    # --- 2. Run scan ---
    found_motors = None
    try:
        # RobstrideBus.scan_channel has already implemented all the logic for us
        # It internally uses tqdm to display a progress bar
        # Scan from 0 to 254 (in case motor changed to invalid ID 0)
        found_motors = RobstrideBus.scan_channel(channel, start_id=1, end_id=12) # end_id=255 will scan up to 254
    
    except Exception as e:
        print(f"\n❌ Scan error: {e}")
        error_str = str(e)
        if "Operation not permitted" in error_str or "Permission denied" in error_str:
            print("🔑 Permission error: Please use 'sudo' to run this script to access CAN hardware.")
            print(f"   Example: sudo python3 {sys.argv[0]} {channel}")
        elif "No such device" in error_str:
            print(f"🔌 Device error: CAN interface '{channel}' not found.")
            print(f"   Check if the interface exists: ip link show {channel}")
        elif "Network is down" in error_str or "Failed to transmit" in error_str:
            print(f"🔌 Network error: CAN interface '{channel}' is down.")
            print(f"   To bring it up, run:")
            print(f"   sudo ip link set {channel} type can bitrate 1000000")
            print(f"   sudo ip link set {channel} up")
            print(f"   Or in one command:")
            print(f"   sudo ip link set {channel} type can bitrate 1000000 && sudo ip link set {channel} up")
            print(f"   Check interface status: ip link show {channel}")
        elif "CAN network" in error_str:
            # This is our custom error message, already contains helpful info
            pass
        else:
            print(f"   Error type: {type(e).__name__}")
        sys.exit(1)

    # --- 3. Print results ---
    if not found_motors:
        print("\n🚫 No responding motors found on the bus.")
    else:
        print("\n✅ Scan complete! Found the following motors:")
        print("=" * 60)
        print(f"{'Motor ID':<10} | {'MCU Unique Identifier (UUID)':<45}")
        print("-" * 60)
        
        # found_motors is a dictionary: {id: (id, uuid_bytearray)}
        # Sort by ID
        for motor_id in sorted(found_motors.keys()):
            # value is a tuple (id, uuid)
            _id, uuid = found_motors[motor_id]
            
            # Convert bytearray to more readable hexadecimal string
            uuid_hex = uuid.hex() # 'hex()' is a method of bytearray
            
            print(f"{motor_id:<10} | {uuid_hex}")
            
        print("=" * 60)

if __name__ == "__main__":
    main()