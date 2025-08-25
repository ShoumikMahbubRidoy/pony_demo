#!/usr/bin/env python3
"""
Test different decode methods for AK60-6 V3.0 feedback
"""

import time
import sys
import can

def decode_method_1(data):
    """Original method from your code"""
    if len(data) != 6:
        return None
    pos_int = (data[1] << 8) | data[2]
    vel_int = (data[3] << 4) | (data[4] >> 4)
    torque_int = ((data[4] & 0xF) << 8) | data[5]
    
    position = uint_to_float(pos_int, -12.5, 12.5, 16)
    velocity = uint_to_float(vel_int, -30.0, 30.0, 12)
    torque = uint_to_float(torque_int, -12.0, 12.0, 12)
    
    return position, velocity, torque

def decode_method_2(data):
    """Alternative method - simple 16-bit values"""
    if len(data) < 5:
        return None
    pos_int = (data[1] << 8) | data[2]
    vel_int = (data[3] << 8) | data[4]
    torque_int = data[5] if len(data) > 5 else 0
    
    position = uint_to_float(pos_int, -12.5, 12.5, 16)
    velocity = uint_to_float(vel_int, -30.0, 30.0, 16)  # Using 16-bit instead of 12
    torque = uint_to_float(torque_int, -12.0, 12.0, 8)   # Using 8-bit instead of 12
    
    return position, velocity, torque

def decode_method_3(data):
    """Another alternative - check if it's 7 or 8 bytes"""
    if len(data) < 6:
        return None
    pos_int = (data[1] << 8) | data[2]
    vel_int = (data[3] << 8) | data[4]
    
    if len(data) >= 7:
        torque_int = (data[5] << 8) | data[6]
        torque = uint_to_float(torque_int, -12.0, 12.0, 16)
    else:
        torque_int = data[5]
        torque = uint_to_float(torque_int, -12.0, 12.0, 8)
    
    position = uint_to_float(pos_int, -12.5, 12.5, 16)
    velocity = uint_to_float(vel_int, -30.0, 30.0, 16)
    
    return position, velocity, torque

def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
    """Convert unsigned integer to float"""
    span = x_max - x_min
    offset = x_int * span / ((1 << bits) - 1)
    return x_min + offset

def test_motor_with_multiple_decoders(motor_id=0x40):
    """Test motor communication with different decode methods"""
    
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
        print("✅ Connected to CAN bus")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return
    
    # Enable motor
    enable_msg = can.Message(
        arbitration_id=motor_id,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
    )
    bus.send(enable_msg)
    print(f"📤 Sent enable command to 0x{motor_id:02X}")
    time.sleep(0.5)
    
    # Send position command
    # Simple zero position command
    pos_msg = can.Message(
        arbitration_id=motor_id,
        data=[0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x10, 0x00]  # Small kp=50, kd=1
    )
    bus.send(pos_msg)
    print("📤 Sent position command")
    
    # Listen for responses
    print("\n🔍 Listening for responses...")
    start_time = time.time()
    
    while time.time() - start_time < 5.0:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == motor_id:
            print(f"\n📨 Received feedback from 0x{motor_id:02X}:")
            print(f"    Raw data: {msg.data.hex().upper()}")
            print(f"    Length: {len(msg.data)} bytes")
            print(f"    Bytes: {[f'0x{b:02X}' for b in msg.data]}")
            
            # Try all decode methods
            try:
                result1 = decode_method_1(msg.data)
                if result1:
                    print(f"    Method 1: pos={result1[0]:.3f}, vel={result1[1]:.3f}, torque={result1[2]:.3f}")
            except Exception as e:
                print(f"    Method 1 failed: {e}")
            
            try:
                result2 = decode_method_2(msg.data)
                if result2:
                    print(f"    Method 2: pos={result2[0]:.3f}, vel={result2[1]:.3f}, torque={result2[2]:.3f}")
            except Exception as e:
                print(f"    Method 2 failed: {e}")
            
            try:
                result3 = decode_method_3(msg.data)
                if result3:
                    print(f"    Method 3: pos={result3[0]:.3f}, vel={result3[1]:.3f}, torque={result3[2]:.3f}")
            except Exception as e:
                print(f"    Method 3 failed: {e}")
            
            break  # Found our response
    
    # Disable motor
    disable_msg = can.Message(
        arbitration_id=motor_id,
        data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]
    )
    bus.send(disable_msg)
    print("\n📤 Sent disable command")
    
    bus.shutdown()

if __name__ == "__main__":
    motor_id = 0x40
    if len(sys.argv) > 1:
        try:
            motor_id = int(sys.argv[1], 16)
        except:
            motor_id = int(sys.argv[1])
    
    test_motor_with_multiple_decoders(motor_id)