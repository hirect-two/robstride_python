#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride Speed Mode Control Script (Fixed Version)
Mode: Mode 2 (Speed Control Mode)
Communication: Use WRITE_PARAMETER to update VELOCITY_TARGET (spd_ref)

Usage: python3 speed_control.py <motor_id>
"""

import sys
import os
import time
import threading
import signal
from typing import Optional

# Try to import SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType
except ImportError:
    # Assume current directory structure
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType
    except ImportError as e:
        print(f"❌ Failed to import SDK: {e}")
        sys.exit(1)

class SpeedController:
    def __init__(self, motor_id: int, channel='can0'):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.channel = channel
        
        self.bus: Optional[RobstrideBus] = None
        self.lock = threading.Lock()  # Mutex lock to prevent Socket conflicts
        
        self.running = True
        self.connected = False
        self.target_velocity = 0.0
        self.current_status = None
        
        # Default parameters
        self.max_velocity = 20.0  # rad/s safety limit
        self.kp = 4.0  # Velocity proportional gain (increased from 2.0 for better response)
        self.ki = 0.5
        self.torque_limit = 3.0  # Torque limit (Nm) - critical for motor to produce torque

    def _signal_handler(self, signum, frame):
        self.stop_and_exit()

    def connect(self):
        print(f"🔍 Connecting to CAN channel {self.channel}...")
        
        # Define motor
        motors = {
            self.motor_name: Motor(id=self.motor_id, model="rs-06") # Please modify model according to actual hardware
        }
        
        # Simple calibration parameters
        calibration = {
            self.motor_name: {"direction": 1, "homing_offset": 0.0}
        }

        try:
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)
            
            # CRITICAL: Mode change sequence - must disable first, then change mode, then enable
            # Check current mode first
            print("🔍 Checking current mode...")
            try:
                current_mode = self.bus.read(self.motor_name, ParameterType.MODE)
                print(f"   Current mode: {current_mode}")
            except Exception as e:
                print(f"   ⚠️  Could not read current mode: {e}")
            
            # Step 1: Disable motor if it's enabled (some motors require disable before mode change)
            print("🔧 Disabling motor before mode change...")
            try:
                self.bus.disable(self.motor_name)
                print("   ✅ Motor disabled")
                time.sleep(0.2)
            except Exception as e:
                print(f"   ⚠️  Note: Motor may already be disabled: {e}")
            
            # Step 2: Set to speed mode (Mode 2) - MUST be done while disabled
            print("⚙️ Setting to speed control mode (Mode 2)...")
            try:
                # Write mode - this internally calls receive_status_frame() which gets the response
                print("   Sending MODE=2 write command...")
                self.bus.write(self.motor_name, ParameterType.MODE, 2)
                print("   ✅ Mode write completed")
                
                # Wait longer for mode change to take effect
                time.sleep(0.5)  # Increased wait time
                
                # Verify mode was actually set by reading it back
                print("🔍 Verifying mode change (reading MODE parameter)...")
                new_mode = self.bus.read(self.motor_name, ParameterType.MODE)
                print(f"   Mode read back: {new_mode}")
                if new_mode != 2:
                    print(f"   ⚠️  WARNING: Mode is {new_mode}, expected 2!")
                    print(f"   ⚠️  Attempting mode change again...")
                    # Try one more time
                    self.bus.write(self.motor_name, ParameterType.MODE, 2)
                    time.sleep(0.5)
                    new_mode = self.bus.read(self.motor_name, ParameterType.MODE)
                    print(f"   Mode after retry: {new_mode}")
                    if new_mode != 2:
                        print(f"   ❌ CRITICAL: Mode change failed! Motor is still in Mode {new_mode}")
                        print(f"   ❌ Motor will not respond to speed commands in Mode {new_mode}")
                        raise RuntimeError(f"Failed to set motor to Mode 2, motor is in Mode {new_mode}")
                else:
                    print("   ✅ Mode successfully set to 2 (Speed Mode)")
            except RuntimeError:
                raise  # Re-raise if it's our error
            except Exception as e:
                print(f"   ❌ Error setting/verifying mode: {e}")
                import traceback
                traceback.print_exc()
                raise
            
            # Step 3: Enable motor after mode change - this is critical to release brake in Speed Mode
            # Some motors have electromagnetic brakes that are released by enable()
            print("⚡ Enabling motor after mode switch (releases brake)...")
            try:
                self.bus.enable(self.motor_name)
                print("   ✅ First enable completed")
                time.sleep(0.3)
            except Exception as e:
                print(f"   ⚠️  Error during first enable: {e}")
            
            # Try enabling again to ensure brake is released
            try:
                self.bus.enable(self.motor_name)
                print("   ✅ Second enable completed")
                time.sleep(0.3)
            except Exception as e:
                print(f"   ⚠️  Error during second enable: {e}")
            
            # Initialize PID and limits
            print("⚙️ Writing control parameters...")
            # Set torque limit first - CRITICAL for motor to actually produce torque
            # Without this, Iq current will be 0 and motor won't move
            print(f"   Writing TORQUE_LIMIT = {self.torque_limit} Nm...")
            self.bus.write(self.motor_name, ParameterType.TORQUE_LIMIT, self.torque_limit)
            time.sleep(0.1)
            # Verify it was written
            try:
                read_torque_limit = self.bus.read(self.motor_name, ParameterType.TORQUE_LIMIT)
                print(f"      Read back: {read_torque_limit} Nm {'✅' if abs(read_torque_limit - self.torque_limit) < 0.01 else '⚠️ MISMATCH!'}")
            except Exception as e:
                print(f"      ⚠️  Could not verify: {e}")
            
            print(f"   Writing VELOCITY_LIMIT = {self.max_velocity} rad/s...")
            self.bus.write(self.motor_name, ParameterType.VELOCITY_LIMIT, self.max_velocity)
            time.sleep(0.1)
            try:
                read_vel_limit = self.bus.read(self.motor_name, ParameterType.VELOCITY_LIMIT)
                print(f"      Read back: {read_vel_limit} rad/s {'✅' if abs(read_vel_limit - self.max_velocity) < 0.01 else '⚠️ MISMATCH!'}")
            except Exception as e:
                print(f"      ⚠️  Could not verify: {e}")
            
            print(f"   Writing VELOCITY_KP = {self.kp}...")
            self.bus.write(self.motor_name, ParameterType.VELOCITY_KP, self.kp)
            time.sleep(0.1)
            try:
                read_kp = self.bus.read(self.motor_name, ParameterType.VELOCITY_KP)
                print(f"      Read back: {read_kp} {'✅' if abs(read_kp - self.kp) < 0.01 else '⚠️ MISMATCH!'}")
            except Exception as e:
                print(f"      ⚠️  Could not verify: {e}")
            
            print(f"   Writing VELOCITY_KI = {self.ki}...")
            self.bus.write(self.motor_name, ParameterType.VELOCITY_KI, self.ki)
            time.sleep(0.1)
            try:
                read_ki = self.bus.read(self.motor_name, ParameterType.VELOCITY_KI)
                print(f"      Read back: {read_ki} {'✅' if abs(read_ki - self.ki) < 0.01 else '⚠️ MISMATCH!'}")
            except Exception as e:
                print(f"      ⚠️  Could not verify: {e}")
            
            # Final verification: Check mode one more time before proceeding
            print("🔍 Final mode verification...")
            try:
                final_mode = self.bus.read(self.motor_name, ParameterType.MODE)
                if final_mode != 2:
                    print(f"   ❌ CRITICAL ERROR: Motor is in Mode {final_mode}, not Mode 2!")
                    print(f"   ❌ Speed control will NOT work in Mode {final_mode}")
                    print(f"   ❌ Please check motor connection and try again")
                    raise RuntimeError(f"Motor is in wrong mode: {final_mode}, expected 2")
                else:
                    print(f"   ✅ Motor confirmed in Speed Mode (2)")
            except Exception as e:
                if "CRITICAL ERROR" not in str(e):
                    print(f"   ⚠️  Could not verify final mode: {e}")
            
            # Zero target and send a few commands to ensure motor is ready
            print("⚙️ Initializing velocity target...")
            for _ in range(3):
                self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
                time.sleep(0.05)
            
            self.connected = True
            print("✅ Initialization complete!")
            print(f"📊 Configuration:")
            print(f"   - Mode: 2 (Speed Mode) ✅")
            print(f"   - Torque Limit: {self.torque_limit} Nm (CRITICAL: motor won't move if this is 0)")
            print(f"   - Velocity Limit: {self.max_velocity} rad/s")
            print(f"   - Velocity Kp: {self.kp}")
            print(f"   - Velocity Ki: {self.ki}")
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False

    def loop(self):
        """Control thread: Continuously send heartbeat/speed commands and read status"""
        print("🔄 Control loop started")
        
        while self.running and self.connected:
            try:
                with self.lock:
                    # In Mode 2, we need to write VELOCITY_TARGET
                    # bus.write will wait for response (receive_status_frame), so this is itself a status read
                    # Protocol 0x700A = VELOCITY_TARGET
                    
                    self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, self.target_velocity)
                    
                    # If you want to read more detailed status (such as current torque), you can use read_operation_frame
                    # But the write response packet already contains status data, SDK's write internally calls receive_status_frame
                    # We don't do additional reads here to maintain high frequency
                    
                time.sleep(0.05) # 20Hz refresh rate, prevent bus congestion
                
            except Exception as e:
                print(f"⚠️ Communication error: {e}")
                time.sleep(0.5)

    def set_velocity(self, vel: float):
        """Set target velocity (with limiting)"""
        vel = max(-self.max_velocity, min(self.max_velocity, vel))
        self.target_velocity = vel
        print(f" -> Target set: {self.target_velocity:.2f} rad/s")
    
    def print_status(self):
        """Read and display motor status for diagnostics"""
        if not self.bus or not self.connected:
            print("❌ Motor not connected")
            return
        
        try:
            with self.lock:
                # Read key parameters
                iq_target = self.bus.read(self.motor_name, ParameterType.IQ_TARGET)
                torque_target = self.bus.read(self.motor_name, ParameterType.TORQUE_TARGET)
                torque_limit = self.bus.read(self.motor_name, ParameterType.TORQUE_LIMIT)
                
                # Read both velocity measurements
                measured_velocity = self.bus.read(self.motor_name, ParameterType.MEASURED_VELOCITY)
                mechanical_velocity = self.bus.read(self.motor_name, ParameterType.MECHANICAL_VELOCITY)
                
                # Read position and actual torque
                measured_position = self.bus.read(self.motor_name, ParameterType.MEASURED_POSITION)
                measured_torque = self.bus.read(self.motor_name, ParameterType.MEASURED_TORQUE)
                
                # Also read from status frame for comparison
                try:
                    pos_frame, vel_frame, torque_frame, temp_frame = self.bus.read_operation_frame(self.motor_name)
                except Exception:
                    pos_frame = vel_frame = torque_frame = temp_frame = None
                
                print(f"\n📊 Motor Status (ID: {self.motor_id}):")
                print(f"   Target Velocity: {self.target_velocity:.3f} rad/s")
                print(f"   Measured Velocity (0x3017): {measured_velocity:.3f} rad/s")
                print(f"   Mechanical Velocity (0x701B): {mechanical_velocity:.3f} rad/s")
                if vel_frame is not None:
                    print(f"   Status Frame Velocity: {vel_frame:.3f} rad/s")
                
                print(f"   Measured Position: {measured_position:.3f} rad ({measured_position*180/3.14159:.1f}°)")
                
                print(f"   Iq Target (Current): {iq_target:.3f} A")
                print(f"   Torque Target (ref): {torque_target:.3f} Nm")
                print(f"   Measured Torque (actual): {measured_torque:.3f} Nm")
                if torque_frame is not None:
                    print(f"   Status Frame Torque: {torque_frame:.3f} Nm")
                print(f"   Torque Limit: {torque_limit:.3f} Nm")
                
                # Diagnostic warnings
                if abs(iq_target) < 0.01:
                    print(f"   ⚠️  WARNING: Iq current is near zero! Motor may not be producing torque.")
                    print(f"   ⚠️  Check: 1) Torque limit is set correctly, 2) Motor is enabled, 3) No faults")
                elif abs(measured_velocity) < 0.1 and abs(mechanical_velocity) < 0.1 and abs(self.target_velocity) > 0.5:
                    print(f"   ⚠️  WARNING: Motor has current ({iq_target:.3f} A) but not moving!")
                    print(f"   ⚠️  Possible causes:")
                    print(f"      1. Mechanical binding or excessive load")
                    print(f"      2. Motor brake not released (try 'force_mode' command)")
                    print(f"      3. Torque limit too low (current: {torque_limit:.3f} Nm)")
                    print(f"      4. Motor stalled or encoder issue")
                
                # Check velocity consistency
                if abs(measured_velocity - mechanical_velocity) > 0.1:
                    print(f"   ⚠️  Velocity mismatch: MEASURED={measured_velocity:.3f}, MECHANICAL={mechanical_velocity:.3f}")
                
                # Check if torque is being applied
                if abs(measured_torque) > 0.01 and abs(mechanical_velocity) < 0.1:
                    print(f"   ⚠️  Torque is being applied ({measured_torque:.3f} Nm) but velocity is zero!")
                    print(f"   ⚠️  Motor may be stalled or mechanically blocked")
                
                print()
        except Exception as e:
            print(f"❌ Failed to read status: {e}")
            import traceback
            traceback.print_exc()
    
    def check_mode(self):
        """Check and display current motor mode"""
        if not self.bus or not self.connected:
            print("❌ Motor not connected")
            return
        
        try:
            with self.lock:
                current_mode = self.bus.read(self.motor_name, ParameterType.MODE)
                mode_names = {0: "MIT Mode", 1: "Position Mode", 2: "Speed Mode"}
                mode_name = mode_names.get(current_mode, f"Unknown Mode ({current_mode})")
                print(f"\n📊 Motor Mode (ID: {self.motor_id}):")
                print(f"   Current Mode: {current_mode} ({mode_name})")
                if current_mode != 2:
                    print(f"   ⚠️  WARNING: Motor is in Mode {current_mode}, not Speed Mode (2)!")
                    print(f"   ⚠️  Speed control requires Mode 2")
                else:
                    print(f"   ✅ Motor is in Speed Mode (2)")
                print()
        except Exception as e:
            print(f"❌ Failed to read mode: {e}")
    
    def verify_all_parameters(self):
        """Verify all control parameters are set correctly"""
        if not self.bus or not self.connected:
            print("❌ Motor not connected")
            return
        
        try:
            with self.lock:
                print(f"\n🔍 Verifying Parameters (ID: {self.motor_id}):")
                print("-" * 50)
                
                # Check mode
                try:
                    mode = self.bus.read(self.motor_name, ParameterType.MODE)
                    mode_ok = (mode == 2)
                    print(f"Mode: {mode} (expected: 2) {'✅' if mode_ok else '❌ FAIL'}")
                except Exception as e:
                    print(f"Mode: ❌ Read failed: {e}")
                
                # Check torque limit
                try:
                    torque_limit = self.bus.read(self.motor_name, ParameterType.TORQUE_LIMIT)
                    torque_ok = abs(torque_limit - self.torque_limit) < 0.01
                    print(f"Torque Limit: {torque_limit:.3f} Nm (expected: {self.torque_limit:.3f}) {'✅' if torque_ok else '❌ FAIL'}")
                    if not torque_ok:
                        print(f"   ⚠️  CRITICAL: Torque limit mismatch! Motor may not produce torque!")
                except Exception as e:
                    print(f"Torque Limit: ❌ Read failed: {e}")
                
                # Check velocity limit
                try:
                    vel_limit = self.bus.read(self.motor_name, ParameterType.VELOCITY_LIMIT)
                    vel_limit_ok = abs(vel_limit - self.max_velocity) < 0.01
                    print(f"Velocity Limit: {vel_limit:.3f} rad/s (expected: {self.max_velocity:.3f}) {'✅' if vel_limit_ok else '❌ FAIL'}")
                except Exception as e:
                    print(f"Velocity Limit: ❌ Read failed: {e}")
                
                # Check velocity KP
                try:
                    kp = self.bus.read(self.motor_name, ParameterType.VELOCITY_KP)
                    kp_ok = abs(kp - self.kp) < 0.01
                    print(f"Velocity KP: {kp:.3f} (expected: {self.kp:.3f}) {'✅' if kp_ok else '❌ FAIL'}")
                except Exception as e:
                    print(f"Velocity KP: ❌ Read failed: {e}")
                
                # Check velocity KI
                try:
                    ki = self.bus.read(self.motor_name, ParameterType.VELOCITY_KI)
                    ki_ok = abs(ki - self.ki) < 0.01
                    print(f"Velocity KI: {ki:.3f} (expected: {self.ki:.3f}) {'✅' if ki_ok else '❌ FAIL'}")
                except Exception as e:
                    print(f"Velocity KI: ❌ Read failed: {e}")
                
                # Check current state - comprehensive with mechanical velocity
                try:
                    iq_target = self.bus.read(self.motor_name, ParameterType.IQ_TARGET)
                    torque_target = self.bus.read(self.motor_name, ParameterType.TORQUE_TARGET)
                    measured_vel = self.bus.read(self.motor_name, ParameterType.MEASURED_VELOCITY)
                    mechanical_vel = self.bus.read(self.motor_name, ParameterType.MECHANICAL_VELOCITY)
                    measured_torque = self.bus.read(self.motor_name, ParameterType.MEASURED_TORQUE)
                    
                    print(f"\nCurrent State:")
                    print(f"   Target Velocity: {self.target_velocity:.3f} rad/s")
                    print(f"   Measured Velocity (0x3017): {measured_vel:.3f} rad/s")
                    print(f"   Mechanical Velocity (0x701B): {mechanical_vel:.3f} rad/s")
                    print(f"   Velocity Error: {self.target_velocity - mechanical_vel:.3f} rad/s")
                    print(f"   Iq Target: {iq_target:.3f} A")
                    print(f"   Torque Target (ref): {torque_target:.3f} Nm")
                    print(f"   Measured Torque (actual): {measured_torque:.3f} Nm")
                    
                    # Explanation about TORQUE_TARGET in Speed Mode
                    if abs(torque_target) < 0.01 and abs(iq_target) > 0.01:
                        print(f"   ℹ️  NOTE: In Speed Mode, TORQUE_TARGET is typically 0.")
                        print(f"   ℹ️  The motor controller calculates torque internally from velocity error.")
                        print(f"   ℹ️  Actual torque is shown in MEASURED_TORQUE: {measured_torque:.3f} Nm")
                    
                    # Velocity consistency check
                    if abs(measured_vel - mechanical_vel) > 0.1:
                        print(f"   ⚠️  Velocity mismatch detected!")
                        print(f"   ⚠️  MEASURED_VELOCITY ({measured_vel:.3f}) ≠ MECHANICAL_VELOCITY ({mechanical_vel:.3f})")
                        print(f"   ⚠️  This may indicate encoder or filtering differences")
                    
                    if abs(iq_target) < 0.01 and abs(self.target_velocity) > 0.1:
                        print(f"   ⚠️  WARNING: Iq is zero but target velocity is non-zero!")
                        print(f"   ⚠️  This suggests motor is not producing torque.")
                    elif abs(measured_torque) > 0.01 and abs(mechanical_vel) < 0.05:
                        print(f"   ⚠️  WARNING: Torque is being applied ({measured_torque:.3f} Nm) but motor not moving!")
                        print(f"   ⚠️  Check: 1) Mechanical binding, 2) Motor brake, 3) Excessive load")
                except Exception as e:
                    print(f"Current State: ❌ Read failed: {e}")
                    import traceback
                    traceback.print_exc()
                
                print("-" * 50)
                print()
        except Exception as e:
            print(f"❌ Failed to verify parameters: {e}")
    
    def read_status_frame_detailed(self):
        """Read and display detailed status frame response from motor"""
        if not self.bus or not self.connected:
            print("❌ Motor not connected")
            return
        
        try:
            with self.lock:
                print(f"\n📡 Reading Status Frame (ID: {self.motor_id})...")
                print("-" * 50)
                
                # Read status frame
                position, velocity, torque, temperature = self.bus.read_operation_frame(self.motor_name)
                
                print(f"Status Frame Response:")
                print(f"   Position: {position:.4f} rad ({position*180/3.14159:.2f}°)")
                print(f"   Velocity: {velocity:.4f} rad/s")
                print(f"   Torque: {torque:.4f} Nm")
                print(f"   Temperature: {temperature:.1f}°C")
                
                # Also read current parameters for comparison
                try:
                    iq_target = self.bus.read(self.motor_name, ParameterType.IQ_TARGET)
                    torque_target = self.bus.read(self.motor_name, ParameterType.TORQUE_TARGET)
                    measured_vel = self.bus.read(self.motor_name, ParameterType.MEASURED_VELOCITY)
                    mechanical_vel = self.bus.read(self.motor_name, ParameterType.MECHANICAL_VELOCITY)
                    measured_torque = self.bus.read(self.motor_name, ParameterType.MEASURED_TORQUE)
                    
                    print(f"\nParameter Comparison:")
                    print(f"   Target Velocity: {self.target_velocity:.4f} rad/s")
                    print(f"   Status Frame Velocity: {velocity:.4f} rad/s")
                    print(f"   MEASURED_VELOCITY (0x3017): {measured_vel:.4f} rad/s")
                    print(f"   MECHANICAL_VELOCITY (0x701B): {mechanical_vel:.4f} rad/s")
                    print(f"   Status Frame Torque: {torque:.4f} Nm")
                    print(f"   MEASURED_TORQUE (0x302C): {measured_torque:.4f} Nm")
                    print(f"   Iq Target: {iq_target:.4f} A")
                    print(f"   Torque Target (ref): {torque_target:.4f} Nm")
                    
                    # Compare velocities
                    if abs(velocity - measured_vel) > 0.1:
                        print(f"   ⚠️  Velocity mismatch: frame={velocity:.4f}, MEASURED={measured_vel:.4f}")
                    if abs(velocity - mechanical_vel) > 0.1:
                        print(f"   ⚠️  Velocity mismatch: frame={velocity:.4f}, MECHANICAL={mechanical_vel:.4f}")
                    if abs(measured_vel - mechanical_vel) > 0.1:
                        print(f"   ⚠️  Velocity mismatch: MEASURED={measured_vel:.4f}, MECHANICAL={mechanical_vel:.4f}")
                    
                    # Compare torques
                    if abs(torque - measured_torque) > 0.1:
                        print(f"   ⚠️  Torque mismatch: frame={torque:.4f}, MEASURED={measured_torque:.4f}")
                except Exception as e:
                    print(f"   ⚠️  Could not read additional parameters: {e}")
                    import traceback
                    traceback.print_exc()
                
                print("-" * 50)
                print()
        except Exception as e:
            print(f"❌ Failed to read status frame: {e}")
            import traceback
            traceback.print_exc()
    
    def force_mode_change(self):
        """Force mode change to Speed Mode (2) - useful if mode change failed during init"""
        if not self.bus or not self.connected:
            print("❌ Motor not connected")
            return
        
        print(f"\n🔧 Force Mode Change to Speed Mode (2)...")
        print("-" * 50)
        
        try:
            with self.lock:
                # Check current mode
                try:
                    current_mode = self.bus.read(self.motor_name, ParameterType.MODE)
                    print(f"Current mode: {current_mode}")
                except Exception as e:
                    print(f"Could not read current mode: {e}")
                
                # Disable motor
                print("Disabling motor...")
                try:
                    self.bus.disable(self.motor_name)
                    print("✅ Motor disabled")
                    time.sleep(0.3)
                except Exception as e:
                    print(f"⚠️  Disable error (may already be disabled): {e}")
                
                # Set mode to 2
                print("Setting mode to 2...")
                self.bus.write(self.motor_name, ParameterType.MODE, 2)
                print("✅ Mode write sent")
                time.sleep(0.5)
                
                # Verify
                new_mode = self.bus.read(self.motor_name, ParameterType.MODE)
                print(f"Mode after change: {new_mode}")
                
                if new_mode == 2:
                    print("✅ Mode successfully changed to 2!")
                    # Re-enable
                    print("Re-enabling motor...")
                    self.bus.enable(self.motor_name)
                    time.sleep(0.3)
                    self.bus.enable(self.motor_name)
                    print("✅ Motor re-enabled")
                else:
                    print(f"❌ Mode change failed! Motor is still in Mode {new_mode}")
                    print(f"⚠️  Speed control will not work until mode is 2")
                
                print("-" * 50)
                print()
        except Exception as e:
            print(f"❌ Error during force mode change: {e}")
            import traceback
            traceback.print_exc()
    
    def check_mechanical_velocity(self):
        """Dedicated function to verify mechanical velocity reading"""
        if not self.bus or not self.connected:
            print("❌ Motor not connected")
            return
        
        print(f"\n🔍 Verifying Mechanical Velocity (ID: {self.motor_id})...")
        print("-" * 60)
        
        try:
            with self.lock:
                # Read all velocity-related parameters
                target_vel = self.target_velocity
                measured_vel = self.bus.read(self.motor_name, ParameterType.MEASURED_VELOCITY)
                mechanical_vel = self.bus.read(self.motor_name, ParameterType.MECHANICAL_VELOCITY)
                
                # Read from status frame
                try:
                    pos_frame, vel_frame, torque_frame, temp_frame = self.bus.read_operation_frame(self.motor_name)
                except Exception as e:
                    print(f"   ⚠️  Could not read status frame: {e}")
                    vel_frame = None
                
                # Read position to see if motor is moving
                measured_pos = self.bus.read(self.motor_name, ParameterType.MEASURED_POSITION)
                mechanical_pos = self.bus.read(self.motor_name, ParameterType.MECHANICAL_POSITION)
                
                # Read torque to see if motor is trying to move
                measured_torque = self.bus.read(self.motor_name, ParameterType.MEASURED_TORQUE)
                iq_target = self.bus.read(self.motor_name, ParameterType.IQ_TARGET)
                
                print(f"Velocity Measurements:")
                print(f"   Target Velocity:        {target_vel:.4f} rad/s")
                print(f"   MEASURED_VELOCITY (0x3017): {measured_vel:.4f} rad/s")
                print(f"   MECHANICAL_VELOCITY (0x701B): {mechanical_vel:.4f} rad/s ← This is the mechanical shaft velocity")
                if vel_frame is not None:
                    print(f"   Status Frame Velocity:  {vel_frame:.4f} rad/s")
                
                print(f"\nPosition Measurements:")
                print(f"   MEASURED_POSITION:      {measured_pos:.4f} rad ({measured_pos*180/3.14159:.2f}°)")
                print(f"   MECHANICAL_POSITION:     {mechanical_pos:.4f} rad ({mechanical_pos*180/3.14159:.2f}°)")
                
                print(f"\nTorque/Current:")
                print(f"   Iq Target:              {iq_target:.4f} A")
                print(f"   Measured Torque:        {measured_torque:.4f} Nm")
                
                print(f"\nAnalysis:")
                # Check if velocities match
                if abs(measured_vel - mechanical_vel) < 0.01:
                    print(f"   ✅ MEASURED_VELOCITY and MECHANICAL_VELOCITY match (difference: {abs(measured_vel - mechanical_vel):.4f} rad/s)")
                else:
                    diff = abs(measured_vel - mechanical_vel)
                    print(f"   ⚠️  Velocity mismatch: MEASURED={measured_vel:.4f}, MECHANICAL={mechanical_vel:.4f}")
                    print(f"   ⚠️  Difference: {diff:.4f} rad/s")
                    print(f"   ℹ️  This may be due to filtering or different measurement sources")
                
                # Check if motor is actually moving
                if abs(mechanical_vel) < 0.01:
                    print(f"   ⚠️  MECHANICAL_VELOCITY is near zero - motor is not moving")
                    if abs(target_vel) > 0.1:
                        print(f"   ⚠️  Target velocity is {target_vel:.3f} rad/s but motor not moving")
                        if abs(measured_torque) > 0.01:
                            print(f"   ⚠️  Torque is being applied ({measured_torque:.3f} Nm) but no movement")
                            print(f"   ⚠️  Possible causes: mechanical binding, brake, or excessive load")
                        else:
                            print(f"   ⚠️  No torque being applied - check torque limit and motor enable")
                else:
                    print(f"   ✅ Motor is moving at {mechanical_vel:.4f} rad/s")
                    error = target_vel - mechanical_vel
                    if abs(error) > 0.5:
                        print(f"   ⚠️  Large velocity error: {error:.4f} rad/s (target - actual)")
                        print(f"   ℹ️  Motor may need more time to reach target, or check PID gains")
                    else:
                        print(f"   ✅ Velocity error is small: {error:.4f} rad/s")
                
                # Check consistency with status frame
                if vel_frame is not None:
                    if abs(vel_frame - mechanical_vel) < 0.1:
                        print(f"   ✅ Status frame velocity matches MECHANICAL_VELOCITY")
                    else:
                        print(f"   ⚠️  Status frame velocity ({vel_frame:.4f}) differs from MECHANICAL_VELOCITY ({mechanical_vel:.4f})")
                
                print("-" * 60)
                print()
                
        except Exception as e:
            print(f"❌ Failed to verify mechanical velocity: {e}")
            import traceback
            traceback.print_exc()

    def stop_and_exit(self):
        print("\n🛑 Stopping...")
        self.running = False
        self.target_velocity = 0.0
        
        if self.bus and self.connected:
            try:
                with self.lock:
                    # Stop first
                    self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
                    time.sleep(0.2)
                    # Disable
                    self.bus.disable(self.motor_name)
            except Exception:
                pass
            self.bus.disconnect()
        sys.exit(0)

    def run_interactive(self):
        # Start background sending thread
        t = threading.Thread(target=self.loop, daemon=True)
        t.start()

        print("\n" + "="*40)
        print(f"🎮 Speed Control Console (ID: {self.motor_id})")
        print("="*40)
        print("👉 Enter a number (rad/s) and press Enter to change speed")
        print("👉 Enter '0' to stop")
        print("👉 Enter 'q' to quit")
        print("👉 Enter 'status' to read motor status (Iq, torque, velocity)")
        print("👉 Enter 'verify' or 'check' to verify all parameters are set correctly")
        print("👉 Enter 'mode' to check current motor mode")
        print("👉 Enter 'response' or 'frame' to read detailed status frame")
        print("👉 Enter 'force_mode' or 'setmode' to force mode change to Speed Mode (2)")
        print("👉 Enter 'mech_vel' or 'mechanical' to verify mechanical velocity")
        print(f"⚠️  Current safety speed limit: ±{self.max_velocity} rad/s")
        print(f"⚠️  Torque limit: {self.torque_limit} Nm (if motor doesn't move, check this)")
        print("\nℹ️  Speed Mode Notes:")
        print("   - TORQUE_TARGET is typically 0 in Speed Mode (torque calculated internally)")
        print("   - Check MEASURED_TORQUE for actual applied torque")
        print("   - MEASURED_VELOCITY (0x3017) and MECHANICAL_VELOCITY (0x701B) are both available")
        print("   - Use 'mech_vel' command to specifically check mechanical velocity")
        print("-" * 40)

        while True:
            try:
                cmd = input(f"[{self.target_velocity:.1f} rad/s] >> ").strip().lower()
                
                if not cmd:
                    continue
                    
                if cmd in ['q', 'quit', 'exit']:
                    break
                
                if cmd == 'status':
                    self.print_status()
                    continue
                
                if cmd == 'verify' or cmd == 'check':
                    self.verify_all_parameters()
                    continue
                
                if cmd == 'mode':
                    self.check_mode()
                    continue
                
                if cmd == 'response' or cmd == 'frame':
                    self.read_status_frame_detailed()
                    continue
                
                if cmd == 'force_mode' or cmd == 'setmode':
                    self.force_mode_change()
                    continue
                
                if cmd == 'mech_vel' or cmd == 'mechanical':
                    self.check_mechanical_velocity()
                    continue
                
                try:
                    vel = float(cmd)
                    self.set_velocity(vel)
                except ValueError:
                    print("❌ Invalid input, please enter a number or 'status'")

            except KeyboardInterrupt:
                break
        
        self.stop_and_exit()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 speed_control.py <motor_id>")
        sys.exit(1)
        
    motor_id = int(sys.argv[1])
    
    controller = SpeedController(motor_id)
    signal.signal(signal.SIGINT, controller._signal_handler)
    
    if controller.connect():
        controller.run_interactive()

if __name__ == "__main__":
    main()