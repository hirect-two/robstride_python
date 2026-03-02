#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride Actuator Speed Control Script

This script demonstrates sinusoidal velocity control in Speed Mode (Mode 2)
with real-time velocity and position feedback.

Usage: python3 move_actuator_speed.py <motor_id> [--channel can0] [--frequency 1.0] [--amplitude 1.0]
"""

import sys
import os
import time
import math
import argparse
import signal
from typing import Optional

# Try to import SDK
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
try:
    from robstride_dynamics import RobstrideBus, Motor, ParameterType
except ImportError:
    try:
        from bus import RobstrideBus, Motor
        from protocol import ParameterType
    except ImportError as e:
        print(f"❌ Failed to import SDK: {e}")
        sys.exit(1)


class RateLimiter:
    """Simple rate limiter to maintain control loop frequency."""
    def __init__(self, frequency: float):
        self.frequency = frequency
        self.period = 1.0 / frequency
        self.last_time = time.time()
    
    def sleep(self):
        """Sleep to maintain the specified frequency."""
        elapsed = time.time() - self.last_time
        sleep_time = self.period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        self.last_time = time.time()


class MoveActuatorSpeed:
    def __init__(self, motor_id: int, channel: str = 'can0', 
                 frequency: float = 1.0, amplitude: float = 1.0):
        self.motor_id = motor_id
        self.motor_name = f"motor_{motor_id}"
        self.channel = channel
        self.motion_frequency = frequency  # Hz
        self.motion_amplitude = amplitude  # radians (for velocity amplitude calculation)
        
        self.bus: Optional[RobstrideBus] = None
        self.running = True
        
        # Speed mode control parameters
        self.velocity_kp = 2.0  # Velocity proportional gain (recommended: 2.0, was 10.0)
        self.velocity_ki = 0.5  # Velocity integral gain
        self.velocity_limit = 50.0  # Max velocity (rad/s)
        self.torque_limit = 3.0  # Torque limit (Nm) - increased from 5.0 for better response
        
        # Control loop frequency - reduced for speed mode to avoid bus congestion
        self.control_frequency = 50.0  # Hz (reduced from 200Hz for speed mode)
        self.rate_limiter = RateLimiter(self.control_frequency)
        
        # Current state
        self.measured_position = None
        self.measured_velocity = None
        self.target_velocity = 0.0
        self.iq_target = 0.0
        self.torque_target = 0.0
        self.start_time = time.time()
    
    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully."""
        self.stop_and_exit()
    
    def connect(self):
        """Connect to CAN bus and initialize motor in Speed Mode."""
        print(f"🔍 Connecting to CAN channel {self.channel}...")
        
        # Define motor
        motors = {
            self.motor_name: Motor(id=self.motor_id, model="rs-02")
        }
        
        # Calibration parameters
        calibration = {
            self.motor_name: {"direction": 1, "homing_offset": 0.0}
        }
        
        try:
            # Create and connect bus
            self.bus = RobstrideBus(self.channel, motors, calibration)
            self.bus.connect(handshake=True)
            
            old_mode = self.bus.read(self.motor_name, ParameterType.MODE)
            print(f"Old mode: {old_mode}")
            # Enable motor initially
            print(f"⚡ Activating motor ID: {self.motor_id}...")
            self.bus.enable(self.motor_name)
            time.sleep(0.5)

            new_mode_1 = self.bus.read(self.motor_name, ParameterType.MODE)
            print(f"New mode_1: {new_mode_1}")

            self.bus.write(self.motor_name, ParameterType.MODE, 2)
            time.sleep(0.3)
            new_mode_2 = self.bus.read(self.motor_name, ParameterType.MODE)
            print(f"New mode2: {new_mode_2}")
            
            if new_mode_2 != 2:
                print(f"❌ Mode change failed! Motor is still in Mode {new_mode_2}")
                self.bus.disable(self.motor_name)
                time.sleep(0.3)

                # Set mode to Speed (Mode 2) FIRST, before setting parameters
                print("⚙️ Setting mode to Speed (Mode 2)...")
                self.bus.write(self.motor_name, ParameterType.MODE, 2)
                time.sleep(0.3)  # Give motor time to switch modes
            
            new_mode_3 = self.bus.read(self.motor_name, ParameterType.MODE)
            print(f"New mode 3: {new_mode_3}")
            # RE-ENABLE motor after mode change - this is critical to release brake in Speed Mode
            # Some motors have electromagnetic brakes that are released by enable()
            print("⚡ Re-enabling motor after mode switch (releases brake)...")
            self.bus.enable(self.motor_name)
            time.sleep(0.3)

            
            # Set Speed mode control parameters
            print("⚙️ Setting Speed mode control parameters...")
            # Set torque limit first - critical for motor to actually move
            self.bus.write(self.motor_name, ParameterType.TORQUE_LIMIT, self.torque_limit)
            time.sleep(0.1)
            self.bus.write(self.motor_name, ParameterType.VELOCITY_LIMIT, self.velocity_limit)
            time.sleep(0.1)
            self.bus.write(self.motor_name, ParameterType.VELOCITY_KP, self.velocity_kp)
            time.sleep(0.1)
            self.bus.write(self.motor_name, ParameterType.VELOCITY_KI, self.velocity_ki)
            time.sleep(0.1)
            
            # Zero target velocity initially
            # self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
            # time.sleep(0.2)
            
            # # Send a few zero velocity commands to ensure motor is ready
            # for _ in range(3):
            #     self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
            #     time.sleep(0.05)
            
            print("✅ Initialization complete!")
            print("\n⚠️  IMPORTANT: If motor shaft cannot be rotated by hand, check:")
            print("   1. Motor brake may need external power (24V brake release)")
            print("   2. Motor may be in fault state - check for error messages")
            print("   3. Mechanical binding or excessive load")
            print("   4. Try increasing torque_limit if motor vibrates but doesn't rotate\n")
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def write_velocity_target(self, target_velocity: float):
        """Write velocity target command and read status (Speed Mode).
        
        In Speed Mode, bus.write() internally calls receive_status_frame() which
        gets status data. We can then read position and velocity parameters
        directly if needed.
        
        Args:
            target_velocity: Target velocity in rad/s
        
        Returns:
            tuple: (measured_position, measured_velocity) or (None, None) on error
        """
        if self.bus is None:
            return None, None
        
        try:
            # Send velocity target command
            # This internally calls receive_status_frame() which gets status
            self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, target_velocity)
            
            # Try reading position and velocity parameters
            # Both MECHANICAL_* and MEASURED_* are mechanical (shaft) values
            # MEASURED_* (0x3016/0x3017) are from status frames
            # MECHANICAL_* (0x7019/0x701B) are from parameter reads
            # We try MECHANICAL_* first as they're more reliable for parameter reads
            # self.iq_target = self.bus.read(self.motor_name, ParameterType.IQ_TARGET)
            # self.torque_target = self.bus.read(self.motor_name, ParameterType.TORQUE_TARGET)
            try:
                position = self.bus.read(self.motor_name, ParameterType.MECHANICAL_POSITION)
            except Exception:
                try:
                    position = self.bus.read(self.motor_name, ParameterType.MEASURED_POSITION)
                except Exception as e:
                    # If both fail, return None
                    position = None
            
            try:
                velocity = self.bus.read(self.motor_name, ParameterType.MECHANICAL_VELOCITY)
            except Exception:
                try:
                    velocity = self.bus.read(self.motor_name, ParameterType.MEASURED_VELOCITY)
                except Exception as e:
                    velocity = None
            
            if position is not None and velocity is not None:
                self.measured_position = position
                self.measured_velocity = velocity
                return position, velocity
            else:
                return None, None
            
        except Exception as e:
            if "No response from the motor" not in str(e):
                print(f"⚠️ Communication error: {e}")
            return None, None
    
    def run(self):
        """Main control loop."""
        if not self.connect():
            return
        
        print("\n" + "="*60)
        # print(f"🎮 RobStride Actuator Speed Control")
        print("="*60)
        print(f"Motor ID: {self.motor_id}")
        print(f"Control Mode: Speed (Mode 2)")
        print(f"Motion Frequency: {self.motion_frequency} Hz")
        # print(f"Motion Amplitude: {math.degrees(self.motion_amplitude):.1f}° ({self.motion_amplitude:.3f} rad)")
        # print(f"Control Frequency: {self.control_frequency} Hz")
        print(f"Velocity Kp: {self.velocity_kp}")
        print(f"Velocity Ki: {self.velocity_ki}")
        print(f"Velocity Limit: {self.velocity_limit} rad/s")
        print(f"Torque Limit: {self.torque_limit} Nm")
        print("="*60)
        print("Press Ctrl+C to stop")
        print("-"*60)
        
        try:
            while self.running:
                elapsed_time = time.time() - self.start_time
                
                # Calculate target velocity (sinusoidal motion)
                # Derivative of position: v = A * ω * cos(ωt)
                omega = 2 * math.pi * self.motion_frequency * 0.0
                self.target_velocity = self.motion_amplitude * omega * math.cos(omega * elapsed_time) + 3.14
                
                # Clamp to velocity limit
                self.target_velocity = max(-self.velocity_limit, min(self.velocity_limit, self.target_velocity))
                
                # Send command and read status (use ramped velocity)
                measured_pos, measured_vel = self.write_velocity_target(self.target_velocity)
                # iq_target = self.bus.read(self.motor_name, ParameterType.IQ_TARGET)
                # Display status
                if measured_pos is not None and measured_vel is not None:
                    error_vel = self.target_velocity - measured_vel
                    
                    # Check if motor is actually moving (velocity should be significant)
                    vel_ratio = abs(measured_vel / self.target_velocity) if abs(self.target_velocity) > 0.1 else 0.0
                    
                    print(f"Target Vel: {self.target_velocity:7.3f} rad/s | "
                          f"Measured: {measured_vel:7.3f} rad/s| Torque Target: {self.torque_target:7.3f} Nm | IQ Target: {self.iq_target:7.3f} A | "
                          f"Pos: {measured_pos:7.3f} rad ({math.degrees(measured_pos):6.1f}°) | "
                          f"Error: {error_vel:6.3f} rad/s | "
                          f"Ratio: {vel_ratio*100:4.1f}%")
                else:
                    # If read fails, still show target velocity
                    print(f"Target Vel: {self.target_velocity:7.3f} rad/s | "
                          f"Measured Vel: N/A | Position: N/A | Status read failed")
                
                # Rate limiting
                self.rate_limiter.sleep()
                
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_and_exit()
    
    def stop_and_exit(self):
        """Stop motor and disconnect."""
        print("\n🛑 Stopping...")
        self.running = False
        
        if self.bus:
            try:
                # Stop velocity
                print("🛑 Stopping velocity...")
                self.bus.write(self.motor_name, ParameterType.VELOCITY_TARGET, 0.0)
                time.sleep(0.5)
                
                # Disable motor
                print("🚫 Disabling motor...")
                self.bus.disable(self.motor_name)
                
            except Exception as e:
                print(f"⚠️ Error during stop: {e}")
            finally:
                try:
                    self.bus.disconnect()
                except Exception:
                    pass
        
        print("👋 Program ended")
        sys.exit(0)


def main():
    parser = argparse.ArgumentParser(description='RobStride Actuator Speed Control')
    parser.add_argument('motor_id', type=int, help='Motor CAN ID')
    parser.add_argument('--channel', type=str, default='can0', help='CAN channel (default: can0)')
    parser.add_argument('--frequency', type=float, default=1.0, help='Motion frequency in Hz (default: 1.0)')
    parser.add_argument('--amplitude', type=float, default=1.0, help='Motion amplitude in radians (default: 1.0)')
    
    args = parser.parse_args()
    
    # Create controller
    controller = MoveActuatorSpeed(
        motor_id=args.motor_id,
        channel=args.channel,
        frequency=args.frequency,
        amplitude=args.amplitude
    )
    
    # Register signal handlers
    signal.signal(signal.SIGINT, controller._signal_handler)
    signal.signal(signal.SIGTERM, controller._signal_handler)
    
    # Run control loop
    controller.run()


if __name__ == "__main__":
    main()
