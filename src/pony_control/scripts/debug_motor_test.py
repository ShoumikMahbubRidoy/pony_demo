#!/usr/bin/env python3
"""
Debug version of motor test with detailed CAN message logging
"""

import time
import sys
import can

sys.path.append('/home/sivax/pony_quadruped/src/pony_control')
sys.path.append('/home/sivax/pony_quadruped/src/pony_control/pony_control')

from pony_control.motor_controller import MotorController
from pony_control.utils.can_utils import CANInterface, CANUtils
from pony_control.utils.constants import *

def debug_motor_communication(motor_id=0x40):
    """Debug motor communication with detailed logging"""
    
    print(f"🔧 Starting debug for motor ID: 0x{motor_id:02X}")
    
    # Connect to CAN bus
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
        print("✅ Connected to CAN bus")
    except Exception as e:
        print(f"❌ Failed to connect to CAN bus: {e}")
        return
    
    # Send enable command
    enable_msg = can.Message(
        arbitration_id=motor_id,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],
        is_extended_id=False
    )
    
    print(f"📤 Sending enable command to 0x{motor_id:02X}")
    print(f"    Data: {enable_msg.data.hex().upper()}")
    
    try:
        bus.send(enable_msg)
        print("✅ Enable command sent successfully")
    except Exception as e:
        print(f"❌ Failed to send enable command: {e}")
        return
    
    # Send a position command
    time.sleep(0.5)
    pos_msg = CANUtils.encode_motor_command(motor_id, 0.0, 0.0, 0.0, 50.0, 1.0)
    print(f"📤 Sending position command to 0x{motor_id:02X}")
    print(f"    Data: {pos_msg.data.hex().upper()}")
    
    try:
        bus.send(pos_msg)
        print("✅ Position command sent successfully")
    except Exception as e:
        print(f"❌ Failed to send position command: {e}")
        return
    
    # Listen for responses
    print("\n🔍 Listening for responses (10 seconds)...")
    start_time = time.time()
    message_count = 0
    
    while time.time() - start_time < 10.0:
        try:
            msg = bus.recv(timeout=0.1)
            if msg:
                message_count += 1
                print(f"\n📨 Message {message_count}:")
                print(f"    ID: 0x{msg.arbitration_id:02X}")
                print(f"    Data: {msg.data.hex().upper()}")
                print(f"    Length: {len(msg.data)} bytes")
                print(f"    Time: {time.time():.3f}")
                
                # If it's from our motor, try to decode it
                if msg.arbitration_id == motor_id:
                    print(f"    🎯 This is from our target motor!")
                    
                    try:
                        motor_id_dec, pos, vel, torque = CANUtils.decode_motor_feedback(msg)
                        print(f"    Decoded feedback:")
                        print(f"      Position: {pos:.3f} rad ({pos*180/3.14159:.1f}°)")
                        print(f"      Velocity: {vel:.3f} rad/s")
                        print(f"      Torque: {torque:.3f} Nm")
                    except Exception as e:
                        print(f"    ❌ Failed to decode: {e}")
                        # Try raw byte interpretation
                        if len(msg.data) >= 6:
                            print(f"    Raw bytes: {[f'0x{b:02X}' for b in msg.data]}")
                
        except can.CanTimeoutError:
            pass  # Normal timeout
        except Exception as e:
            print(f"❌ Error receiving message: {e}")
    
    print(f"\n📊 Summary:")
    print(f"    Total messages received: {message_count}")
    print(f"    Messages from target motor (0x{motor_id:02X}): {sum(1 for _ in range(message_count) if True)}")  # This is a placeholder
    
    # Send disable command
    disable_msg = can.Message(
        arbitration_id=motor_id,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD],
        is_extended_id=False
    )
    
    print(f"\n📤 Sending disable command")
    try:
        bus.send(disable_msg)
        print("✅ Disable command sent")
    except Exception as e:
        print(f"❌ Failed to send disable: {e}")
    
    bus.shutdown()

if __name__ == "__main__":
    motor_id = 0x40
    if len(sys.argv) > 1:
        try:
            motor_id = int(sys.argv[1], 16)
        except:
            motor_id = int(sys.argv[1])
    
    debug_motor_communication(motor_id)