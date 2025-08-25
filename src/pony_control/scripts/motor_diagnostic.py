#!/usr/bin/env python3
"""
Comprehensive AK60-6 Motor Diagnostic Tool
Tests multiple CAN IDs, baud rates, and command formats
"""

import can
import time
import sys

class MotorDiagnostic:
    def __init__(self):
        self.bus = None
        self.found_motors = []
        
    def connect_can(self, interface='can0', bitrate=1000000):
        """Connect to CAN bus with specified settings"""
        try:
            self.bus = can.interface.Bus(channel=interface, interface='socketcan', bitrate=bitrate)
            print(f"✅ Connected to {interface} at {bitrate} bps")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to {interface} at {bitrate}: {e}")
            return False
    
    def test_baud_rates(self, interface='can0'):
        """Test different baud rates"""
        baud_rates = [1000000, 500000, 250000, 125000]
        
        for baud in baud_rates:
            print(f"\n🔍 Testing baud rate: {baud} bps")
            
            # Reconfigure interface
            import subprocess
            try:
                subprocess.run(['sudo', 'ip', 'link', 'set', interface, 'down'], check=True)
                subprocess.run(['sudo', 'ip', 'link', 'set', interface, 'type', 'can', 'bitrate', str(baud)], check=True)
                subprocess.run(['sudo', 'ip', 'link', 'set', interface, 'up'], check=True)
            except subprocess.CalledProcessError as e:
                print(f"❌ Failed to set baud rate {baud}: {e}")
                continue
            
            if self.connect_can(interface, baud):
                # Test communication at this baud rate
                found = self.scan_motor_ids()
                if found:
                    print(f"✅ Found {len(found)} responding motors at {baud} bps")
                    self.found_motors.extend([(baud, motor_id) for motor_id in found])
                
                self.disconnect()
            
            time.sleep(1)
    
    def scan_motor_ids(self):
        """Scan for motors across different ID ranges"""
        if not self.bus:
            return []
        
        found_motors = []
        
        # Test ID ranges
        id_ranges = [
            # Standard IDs (3 hex digits)
            range(0x001, 0x010),  # 0x001-0x00F
            range(0x040, 0x050),  # 0x040-0x04F  
            range(0x140, 0x150),  # 0x140-0x14F
            range(0x200, 0x210),  # 0x200-0x20F
            # Extended IDs that might relate to 0x2940
            [0x2940, 0x2941, 0x2942, 0x2943, 0x2944]
        ]
        
        # Different enable command formats to try
        enable_commands = [
            [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC],  # Standard AK60-6
            [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],  # All FF
            [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC],  # Zeros + FC
            [0xFC, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF],  # FC first
            [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],  # Simple enable
        ]
        
        for id_range in id_ranges:
            for motor_id in id_range:
                print(f"Testing ID: 0x{motor_id:02X}")
                
                for cmd_idx, cmd_data in enumerate(enable_commands):
                    try:
                        # Send enable command
                        msg = can.Message(
                            arbitration_id=motor_id,
                            data=cmd_data,
                            is_extended_id=(motor_id > 0x7FF)
                        )
                        
                        self.bus.send(msg)
                        print(f"  Sent command format {cmd_idx+1} to 0x{motor_id:02X}")
                        
                        # Listen for immediate response
                        start_time = time.time()
                        while time.time() - start_time < 0.5:
                            response = self.bus.recv(timeout=0.1)
                            if response and response.arbitration_id == motor_id:
                                print(f"  ✅ Response from 0x{motor_id:02X}: {response.data.hex().upper()}")
                                found_motors.append(motor_id)
                                break
                        
                        time.sleep(0.2)  # Pause between commands
                        
                    except Exception as e:
                        print(f"  ❌ Error with 0x{motor_id:02X}: {e}")
        
        return found_motors
    
    def test_specific_motor(self, motor_id):
        """Test specific motor with multiple command formats"""
        print(f"\n🎯 Testing motor 0x{motor_id:02X} with multiple protocols:")
        
        # Extended command formats for this specific motor
        test_commands = [
            # Standard enable commands
            ("Standard Enable", [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]),
            ("Reset/Disable", [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]),
            ("Zero Position", [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE]),
            
            # Alternative formats
            ("Alt Enable 1", [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            ("Alt Enable 2", [0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            ("Alt Enable 3", [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC]),
            
            # Position commands
            ("Zero Position Cmd", [0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x10, 0x00]),
            ("Small Position", [0x01, 0x00, 0x00, 0x00, 0x32, 0x00, 0x10, 0x00]),
        ]
        
        responses = []
        
        for cmd_name, cmd_data in test_commands:
            print(f"\n  Testing: {cmd_name}")
            print(f"    Data: {' '.join(f'{b:02X}' for b in cmd_data)}")
            
            try:
                msg = can.Message(
                    arbitration_id=motor_id,
                    data=cmd_data,
                    is_extended_id=(motor_id > 0x7FF)
                )
                
                self.bus.send(msg)
                print(f"    📤 Sent successfully")
                
                # Listen for response for 1 second
                start_time = time.time()
                response_found = False
                
                while time.time() - start_time < 1.0:
                    response = self.bus.recv(timeout=0.1)
                    if response:
                        if response.arbitration_id == motor_id:
                            print(f"    📨 Direct response: {response.data.hex().upper()}")
                            responses.append((cmd_name, response.data.hex().upper()))
                            response_found = True
                        elif response.arbitration_id == 0x2940:  # Known broadcast ID
                            # Check if broadcast pattern changed
                            data_hex = response.data.hex().upper()
                            if data_hex != "FD03000000001C00":  # Different from normal broadcast
                                print(f"    📡 Broadcast changed: {data_hex}")
                                responses.append((cmd_name, f"Broadcast: {data_hex}"))
                                response_found = True
                
                if not response_found:
                    print(f"    ❌ No response")
                
                # Ask user to check LED status
                print(f"    💡 Check LEDs now - Green LED on? (Ctrl+C to skip)")
                
            except Exception as e:
                print(f"    ❌ Error: {e}")
            
            time.sleep(1)  # Pause between tests
        
        return responses
    
    def monitor_traffic(self, duration=10):
        """Monitor all CAN traffic to understand the motor's behavior"""
        print(f"\n👀 Monitoring CAN traffic for {duration} seconds...")
        print("Look for patterns, changes, or new message types...")
        
        start_time = time.time()
        message_counts = {}
        unique_messages = set()
        
        while time.time() - start_time < duration:
            msg = self.bus.recv(timeout=0.5)
            if msg:
                msg_id = msg.arbitration_id
                data_hex = msg.data.hex().upper()
                
                # Count messages by ID
                message_counts[msg_id] = message_counts.get(msg_id, 0) + 1
                
                # Track unique message patterns
                pattern = f"0x{msg_id:02X}:{data_hex}"
                if pattern not in unique_messages:
                    unique_messages.add(pattern)
                    print(f"  New pattern: ID=0x{msg_id:02X}, Data={data_hex}")
        
        print(f"\n📊 Traffic Summary:")
        for msg_id, count in message_counts.items():
            print(f"  ID 0x{msg_id:02X}: {count} messages")
    
    def disconnect(self):
        """Disconnect from CAN bus"""
        if self.bus:
            self.bus.shutdown()
            self.bus = None

def main():
    """Main diagnostic routine"""
    print("🔧 AK60-6 Motor Diagnostic Tool")
    print("=" * 50)
    
    diagnostic = MotorDiagnostic()
    
    # Test 1: Current configuration
    print("\n1️⃣ Testing current CAN configuration...")
    if diagnostic.connect_can():
        diagnostic.monitor_traffic(5)
        
        # Test the known broadcast ID as receive ID
        print("\n2️⃣ Testing motor 0x2940 (known broadcaster)...")
        responses = diagnostic.test_specific_motor(0x2940)
        
        # Test common standard IDs
        print("\n3️⃣ Testing common motor IDs...")
        for test_id in [0x001, 0x040, 0x141]:
            diagnostic.test_specific_motor(test_id)
        
        diagnostic.disconnect()
    
    # Test 2: Different baud rates (commented out for now - requires sudo)
    # print("\n4️⃣ Testing different baud rates...")
    # diagnostic.test_baud_rates()
    
    print(f"\n🏁 Diagnostic complete!")
    if diagnostic.found_motors:
        print("Found responsive motors:")
        for baud, motor_id in diagnostic.found_motors:
            print(f"  Baud: {baud}, Motor: 0x{motor_id:02X}")
    else:
        print("No motors responded to standard commands.")
        print("Check motor configuration mode (DIP switches, jumpers)")

if __name__ == "__main__":
    main()