"""
CAN communication utilities for AK60-6 V3.0 motors
Handles low-level CAN message encoding/decoding
"""

import struct
import can
from typing import Tuple, Optional
import logging

# Set up logging
logger = logging.getLogger(__name__)

class CANUtils:
    """Utility class for AK60-6 CAN message handling"""
    
    @staticmethod
    def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> int:
        """
        Convert float to unsigned integer for CAN transmission
        Used for encoding position, velocity, and torque values
        """
        if x < x_min:
            x = x_min
        elif x > x_max:
            x = x_max
        
        span = x_max - x_min
        offset = x - x_min
        return int(offset * ((1 << bits) - 1) / span)
    
    @staticmethod
    def uint_to_float(x_int: int, x_min: float, x_max: float, bits: int) -> float:
        """
        Convert unsigned integer to float for CAN reception  
        Used for decoding position, velocity, and torque feedback
        """
        span = x_max - x_min
        offset = x_int * span / ((1 << bits) - 1)
        return x_min + offset
    
    @staticmethod
    def encode_motor_command(motor_id: int, position: float = 0.0, velocity: float = 0.0, 
                           torque: float = 0.0, kp: float = 0.0, kd: float = 0.0) -> can.Message:
        """
        Encode motor control command into CAN message for AK60-6
        
        Args:
            motor_id: Motor CAN ID (0x40-0x72)
            position: Desired position in radians (-12.5 to 12.5)
            velocity: Desired velocity in rad/s (-30 to 30) 
            torque: Feed-forward torque in Nm (-12 to 12)
            kp: Position gain (0 to 500)
            kd: Velocity gain (0 to 5)
            
        Returns:
            CAN message ready for transmission
        """
        # Convert parameters to integers using AK60-6 encoding ranges
        pos_int = CANUtils.float_to_uint(position, -12.5, 12.5, 16)
        vel_int = CANUtils.float_to_uint(velocity, -30.0, 30.0, 12)
        torque_int = CANUtils.float_to_uint(torque, -12.0, 12.0, 12)
        kp_int = CANUtils.float_to_uint(kp, 0.0, 500.0, 12)
        kd_int = CANUtils.float_to_uint(kd, 0.0, 5.0, 12)
        
        # Pack into 8-byte CAN data frame according to AK60-6 protocol
        data = bytearray(8)
        data[0] = pos_int >> 8          # Position high byte
        data[1] = pos_int & 0xFF        # Position low byte
        data[2] = vel_int >> 4          # Velocity high byte
        data[3] = ((vel_int & 0xF) << 4) | (kp_int >> 8)  # Vel low + Kp high
        data[4] = kp_int & 0xFF         # Kp low byte
        data[5] = kd_int >> 4           # Kd high byte
        data[6] = ((kd_int & 0xF) << 4) | (torque_int >> 8)  # Kd low + Torque high  
        data[7] = torque_int & 0xFF     # Torque low byte
        
        return can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    
    @staticmethod
    def decode_motor_feedback(msg: can.Message) -> Tuple[int, float, float, float]:
        motor_id = msg.arbitration_id
        data = msg.data
        
        # Handle both 6-byte and 8-byte feedback formats
        if len(data) == 8:
            # 8-byte format (your motor's format)
            # Skip first 2 bytes (status), use bytes 2-7 for data
            pos_int = (data[2] << 8) | data[3]
            vel_int = (data[4] << 8) | data[5] 
            torque_int = (data[6] << 8) | data[7]
        elif len(data) == 6:
            # Original 6-byte format
            pos_int = (data[1] << 8) | data[2]
            vel_int = (data[3] << 4) | (data[4] >> 4)
            torque_int = ((data[4] & 0xF) << 8) | data[5]
        else:
            logger.warning(f"Unexpected feedback message length: {len(data)}")
            return motor_id, 0.0, 0.0, 0.0
        
        # Convert back to float values
        position = CANUtils.uint_to_float(pos_int, -12.5, 12.5, 16)
        velocity = CANUtils.uint_to_float(vel_int, -30.0, 30.0, 16)
        torque = CANUtils.uint_to_float(torque_int, -12.0, 12.0, 16)
        
        return motor_id, position, velocity, torque
    
    @staticmethod
    def create_control_message(motor_id: int, command: int) -> can.Message:
        """
        Create control messages for motor enable/disable/zero
        
        Args:
            motor_id: Motor CAN ID
            command: Control command (0xFC=enable, 0xFD=disable, 0xFE=zero)
            
        Returns:
            CAN control message
        """
        data = bytearray(8)
        data[0] = 0xFF
        data[1] = 0xFF
        data[2] = 0xFF
        data[3] = 0xFF
        data[4] = 0xFF
        data[5] = 0xFF
        data[6] = 0xFF
        data[7] = command
        
        return can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)

class CANInterface:
    """
    High-level CAN interface for motor communication
    Manages the CAN bus connection and message handling
    """
    
    def __init__(self, interface: str = 'can0', bitrate: int = 1000000):
        """
        Initialize CAN interface
        
        Args:
            interface: CAN interface name (usually 'can0' for USB-CAN adapters)
            bitrate: CAN bus bitrate in bps (1000000 = 1Mbps)
        """
        self.interface = interface
        self.bitrate = bitrate
        self.bus = None
        self.is_connected = False
        
    def connect(self) -> bool:
        """
        Connect to CAN bus
        
        Returns:
            True if connection successful
        """
        try:
            # Create CAN bus instance
            self.bus = can.interface.Bus(
                channel=self.interface,
                bustype='socketcan',
                bitrate=self.bitrate
            )
            self.is_connected = True
            logger.info(f"Connected to CAN bus {self.interface} at {self.bitrate} bps")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect to CAN bus: {e}")
            self.is_connected = False
            return False
    
    def disconnect(self):
        """Disconnect from CAN bus"""
        if self.bus:
            self.bus.shutdown()
            self.is_connected = False
            logger.info("Disconnected from CAN bus")
    
    def send_message(self, msg: can.Message) -> bool:
        """
        Send CAN message
        
        Args:
            msg: CAN message to send
            
        Returns:
            True if sent successfully
        """
        if not self.is_connected or not self.bus:
            logger.error("CAN bus not connected")
            return False
        
        try:
            self.bus.send(msg)
            return True
        except Exception as e:
            logger.error(f"Failed to send CAN message: {e}")
            return False
    
    def receive_message(self, timeout: float = 0.1) -> Optional[can.Message]:
        """
        Receive CAN message
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Received message or None if timeout
        """
        if not self.is_connected or not self.bus:
            return None
        
        try:
            return self.bus.recv(timeout=timeout)
        except can.CanTimeoutError:
            return None
        except Exception as e:
            logger.error(f"Failed to receive CAN message: {e}")
            return None
    
    def __enter__(self):
        """Context manager entry"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.disconnect()