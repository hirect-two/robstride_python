#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride MIT Mode Position Control (Simplest Reliable Version)
Mode: Mode 0 (MIT Mode)
Communication: Loop calling write_operation_frame

Usage: python3 position_control_mit.py <motor_id>
"""

import sys
import os
import time
import math
import struct
import threading
import signal
from typing import Optional

# Try to import SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType, CommunicationType
    print("Imported SDK from system path")
except ImportError:
        # Assume current directory structure
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType, CommunicationType
    except ImportError as e:
        print(f"❌ Failed to import SDK: {e}")
        sys.exit(1)

class PositionControllerMIT:
    def __init__(self, motor_id: int, channel='can0'):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.channel = channel
        
        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()  # Mutex lock to prevent Socket conflicts
        
        self.running = True
        self.connected = False
        self.target_position = 0.0  # Target position (rad)
        
        # Default parameters (MIT Mode)
        self.kp = 20.0  # Stiffness (Nm/rad)
        self.kd = 3.0   # Damping (Nm/rad/s)

    def _signal_handler(self, signum, frame):
        self.stop_and_exit()

    def _set_mode_raw(self, mode: int):
        """
        Use raw transmit to send mode switch command without waiting for response (avoid connect timeout)
        """
        print(f"⚙️ Switching mode (Mode {mode}) - [Raw Transmit]")
        device_id = self.bus.motors[self.motor_name].id
        param_id, param_dtype, _ = ParameterType.MODE

        # MODE 是 int8
        value_buffer = struct.pack("<bBH", mode, 0, 0)
        data = struct.pack("<HH", param_id, 0x00) + value_buffer

        self.bus.transmit(CommunicationType.WRITE_PARAMETER, self.bus.host_id, device_id, data)
        time.sleep(0.1) # Wait for motor to switch mode
        print(f"✅ Mode switch command sent")

    def connect(self):
        print(f"🔍 Connecting to CAN channel {self.channel}...")
        
        # Define motor
        motors = {
            self.motor_name: Motor(id=self.motor_id, model="rs-03") # Please modify model according to actual hardware
        }
        
        # Simple calibration parameters
        calibration = {
            self.motor_name: {"direction": 1, "homing_offset": 0.0}
        }

        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)
            
            with self.lock:
                # Activate motor
                print(f"⚡ Activating motor ID: {self.motor_id} ...")
                self.bus.enable(self.motor_name)
                time.sleep(0.5)

                # *********************
                # *** Core Logic ***
                # *********************
                # 1. Switch to MIT mode (Mode 0)
                self._set_mode_raw(0)
                
                # 2. Set a known, safe initial target
                print("🏠 Setting initial target to 0.0 ...")
                self.target_position = 0.0 # Set to 0 radians
                
                # 3. Send first MIT frame to maintain position
                self.bus.write_operation_frame(
                    self.motor_name,
                    self.target_position,
                    self.kp,
                    self.kd,
                    0.0, # velocity_ff
                    0.0  # torque_ff
                )
                print(f"🏠 Initial target set to: 0.0°")
            
            self.connected = True
            
            # Start background control thread
            self.control_thread = threading.Thread(target=self.loop, daemon=True)
            self.control_thread.start()
            
            print("✅ Initialization complete (Mode 0)!")
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            self.connected = False
            return False

    def loop(self):
        """Control thread: Continuously send MIT frames to maintain position"""
        print("🔄 Control loop started (Mode 0 @ 50Hz)")
        
        while self.running and self.connected:
            try:
                with self.lock:
                    # 1. Send MIT frame (send only)
                    self.bus.write_operation_frame(
                        self.motor_name,
                        self.target_position,
                        self.kp,
                        self.kd,
                        0.0, # velocity_ff
                        0.0  # torque_ff
                    )
                    
                    # 2. Read status frame (receive only)
                    # This step is crucial for clearing CAN receive buffer to prevent overflow
                    # We can ignore the return value, as we only care about the "clearing" action
                    self.bus.read_operation_frame(self.motor_name)
                    
                time.sleep(0.02) # 50Hz control frequency
                
            except Exception as e:
                # Ignore timeout, as this is common in read_operation_frame
                if "No response from the motor" not in str(e):
                    print(f"⚠️ Communication error: {e}")
                time.sleep(0.5)

    def set_angle(self, angle_degrees: float):
        """Set target angle (unit: degrees)"""
        # Limit range, e.g. +/- 2 turns
        angle_degrees = max(-720.0, min(720.0, angle_degrees))
        # target_position is thread-safe (atomic operation)
        self.target_position = math.radians(angle_degrees)
        print(f" -> Target set: {angle_degrees:.1f}°")

    def set_kp(self, kp: float):
        """Set stiffness"""
        if 0 <= kp <= 500:
            self.kp = kp
            print(f" -> Stiffness (Kp) set: {self.kp:.1f}")
        else:
            print("❌ Kp range must be 0-500")

    def set_kd(self, kd: float):
        """Set damping"""
        if 0 <= kd <= 5:
            self.kd = kd
            print(f" -> Damping (Kd) set: {self.kd:.1f}")
        else:
            print("❌ Kd range must be 0-5")

    def stop_and_exit(self):
        print("\n🛑 Stopping...")
        self.running = False
        
        if self.control_thread:
            self.control_thread.join(timeout=0.5) # Wait for thread to exit
        
        if self.bus and self.connected:
            try:
                with self.lock:
                    # Return to zero position
                    print("🏠 Returning to zero position...")
                    self.bus.write_operation_frame(self.motor_name, 0.0, self.kp, self.kd, 0.0, 0.0)
                    time.sleep(1.0) # Wait for motor to move
                    # Disable
                    print("🚫 Disabling motor...")
                    self.bus.disable(self.motor_name)
            except Exception as e:
                print(f"⚠️ Error during stop: {e}")
            finally:
                self.bus.disconnect()
        
        print("👋 Program ended")
        sys.exit(0)

    def run_interactive(self):
        print("\n" + "="*40)
        print(f"🎮 MIT Position Control Console (ID: {self.motor_id})")
        print("="*40)
        print("👉 Enter a number (unit: degrees) and press Enter to change position")
        print("👉 'kp <value>' (e.g.: kp 20) to adjust stiffness (reduce vibration)")
        print("👉 'kd <value>' (e.g.: kd 0.8) to adjust damping (reduce vibration)")
        print("👉 '0' or 'home' to return to zero")
        print("👉 'q' to quit")
        print(f"⚠️  Current Kp={self.kp} | Kd={self.kd}")
        print("-" * 40)

        while True:
            try:
                cmd = input(f"[{math.degrees(self.target_position):.1f}°] >> ").strip().lower()
                
                if not cmd:
                    continue
                    
                if cmd in ['q', 'quit', 'exit']:
                    break
                
                if cmd in ['0', 'home']:
                    self.set_angle(0.0)
                    continue

                if cmd.startswith("kp "):
                    try:
                        new_kp = float(cmd.split()[1])
                        self.set_kp(new_kp)
                    except Exception:
                        print("❌ Invalid Kp. Example: kp 20.0")
                    continue

                if cmd.startswith("kd "):
                    try:
                        new_kd = float(cmd.split()[1])
                        self.set_kd(new_kd)
                    except Exception:
                        print("❌ Invalid Kd. Example: kd 0.5")
                    continue

                try:
                    angle = float(cmd)
                    self.set_angle(angle)
                except ValueError:
                    print("❌ Invalid input, please enter a number (angle) or 'kp', 'kd'")

            except KeyboardInterrupt:
                break
        
        self.stop_and_exit()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 position_control_mit.py <motor_id>")
        sys.exit(1)
        
    motor_id = int(sys.argv[1])
    
    controller = PositionControllerMIT(motor_id)
    signal.signal(signal.SIGINT, controller._signal_handler)
    signal.signal(signal.SIGTERM, controller._signal_handler)
    
    if controller.connect():
        controller.run_interactive()

if __name__ == "__main__":
    main()