"""
AK60-6 V3.0 Motor Controller
Handles individual motor communication and control
"""

import time
import threading
import logging
from typing import Optional, Tuple
from dataclasses import dataclass

from .utils.can_utils import CANInterface, CANUtils
from .utils.constants import *

logger = logging.getLogger(__name__)

@dataclass
class MotorState:
    """Data class to hold motor state information"""
    position: float = 0.0      # Current position in radians
    velocity: float = 0.0      # Current velocity in rad/s  
    torque: float = 0.0        # Current torque in Nm
    temperature: int = 0       # Motor temperature in Celsius
    error_code: int = 0        # Error status
    enabled: bool = False      # Motor enable status
    last_update: float = 0.0   # Timestamp of last update

class MotorController:
    """
    Controller for single AK60-6 motor
    Handles communication, control, and state management
    """
    
    def __init__(self, motor_id: int, can_interface: CANInterface, joint_name: str = ""):
        """
        Initialize motor controller
        
        Args:
            motor_id: Motor CAN ID (0x40-0x72)
            can_interface: CAN interface instance
            joint_name: Human-readable joint name (hip/knee/ankle)
        """
        self.motor_id = motor_id
        self.can_interface = can_interface
        self.joint_name = joint_name or f"motor_{motor_id:02X}"
        
        # Motor state
        self.state = MotorState()
        self.target_position = 0.0
        self.target_velocity = 0.0
        self.target_torque = 0.0
        
        # Control parameters
        self.kp = DEFAULT_PID_GAINS['kp']
        self.kd = DEFAULT_PID_GAINS['kd']
        self.ki = DEFAULT_PID_GAINS['ki']
        
        # Safety and limits
        self.position_limits = (-12.5, 12.5)  # Default AK60-6 limits
        self.velocity_limit = 30.0
        self.torque_limit = 12.0
        
        # Communication
        self._lock = threading.Lock()
        self._last_command_time = 0.0
        self._command_rate_limit = 1.0 / FEEDBACK_RATE  # 50 Hz max
        
        logger.info(f"Initialized motor controller for {self.joint_name} (ID: 0x{motor_id:02X})")
    
    def set_limits(self, joint_type: str):
        """
        Set safety limits based on joint type
        
        Args:
            joint_type: 'hip', 'knee', or 'ankle'
        """
        if joint_type in SAFETY_LIMITS:
            limits = SAFETY_LIMITS[joint_type]
            self.position_limits = (limits['min_pos'], limits['max_pos'])
            self.velocity_limit = limits['max_vel']
            self.torque_limit = limits['max_torque']
            logger.info(f"Set safety limits for {joint_type}: pos={self.position_limits}, "
                       f"vel={self.velocity_limit}, torque={self.torque_limit}")
    
    def enable_motor(self) -> bool:
        """
        Enable motor (enter motor mode)
        
        Returns:
            True if command sent successfully
        """
        msg = CANUtils.create_control_message(self.motor_id, CAN_COMMANDS['MOTOR_MODE'])
        if self.can_interface.send_message(msg):
            self.state.enabled = True
            logger.info(f"Enabled motor {self.joint_name}")
            return True
        return False
    
    def disable_motor(self) -> bool:
        """
        Disable motor (exit motor mode)
        
        Returns:
            True if command sent successfully
        """
        msg = CANUtils.create_control_message(self.motor_id, CAN_COMMANDS['RESET_MODE'])
        if self.can_interface.send_message(msg):
            self.state.enabled = False
            logger.info(f"Disabled motor {self.joint_name}")
            return True
        return False
    
    def zero_position(self) -> bool:
        """
        Set current position as zero reference
        
        Returns:
            True if command sent successfully
        """
        msg = CANUtils.create_control_message(self.motor_id, CAN_COMMANDS['ZERO_POSITION'])
        if self.can_interface.send_message(msg):
            logger.info(f"Zeroed position for motor {self.joint_name}")
            return True
        return False
    
    def set_position(self, position: float, velocity: float = 0.0, 
                    torque_ff: float = 0.0) -> bool:
        """
        Send position command to motor
        
        Args:
            position: Target position in radians
            velocity: Target velocity in rad/s (optional)
            torque_ff: Feed-forward torque in Nm (optional)
            
        Returns:
            True if command sent successfully
        """
        # Apply safety limits
        position = self._clamp_position(position)
        velocity = self._clamp_velocity(velocity)
        torque_ff = self._clamp_torque(torque_ff)
        
        # Rate limiting to prevent bus overload
        current_time = time.time()
        # Allow first command to go through immediately
        if self._last_command_time > 0 and current_time - self._last_command_time < self._command_rate_limit:
            return False
        
        with self._lock:
            self.target_position = position
            self.target_velocity = velocity
            self.target_torque = torque_ff
            
            # Create and send command message
            msg = CANUtils.encode_motor_command(
                self.motor_id, position, velocity, torque_ff, self.kp, self.kd
            )
            
            success = self.can_interface.send_message(msg)
            if success:
                self._last_command_time = current_time
            
            return success
    
    def set_velocity(self, velocity: float, torque_ff: float = 0.0) -> bool:
        """
        Send velocity command to motor
        
        Args:
            velocity: Target velocity in rad/s
            torque_ff: Feed-forward torque in Nm (optional)
            
        Returns:
            True if command sent successfully
        """
        return self.set_position(self.state.position, velocity, torque_ff)
    
    def set_torque(self, torque: float) -> bool:
        """
        Send torque command to motor
        
        Args:
            torque: Target torque in Nm
            
        Returns:
            True if command sent successfully
        """
        # For pure torque control, set kp and kd to 0
        old_kp, old_kd = self.kp, self.kd
        self.kp = 0.0
        self.kd = 0.0
        
        success = self.set_position(0.0, 0.0, torque)
        
        # Restore gains
        self.kp = old_kp
        self.kd = old_kd
        
        return success
    
    def update_state_from_feedback(self, msg) -> bool:
        """
        Update motor state from CAN feedback message
        
        Args:
            msg: CAN message with motor feedback
            
        Returns:
            True if state updated successfully
        """
        try:
            motor_id, position, velocity, torque = CANUtils.decode_motor_feedback(msg)
            
            if motor_id != self.motor_id:
                return False
            
            with self._lock:
                self.state.position = position
                self.state.velocity = velocity
                self.state.torque = torque
                self.state.last_update = time.time()
            
            return True
            
        except Exception as e:
            logger.error(f"Error updating motor state: {e}")
            return False
    
    def get_state(self) -> MotorState:
        """
        Get current motor state (thread-safe)
        
        Returns:
            Copy of current motor state
        """
        with self._lock:
            return MotorState(
                position=self.state.position,
                velocity=self.state.velocity,
                torque=self.state.torque,
                temperature=self.state.temperature,
                error_code=self.state.error_code,
                enabled=self.state.enabled,
                last_update=self.state.last_update
            )
    
    def is_alive(self, timeout: float = 1.0) -> bool:
        """
        Check if motor is responding (receiving feedback)
        
        Args:
            timeout: Maximum time since last update
            
        Returns:
            True if motor responded recently
        """
        return (time.time() - self.state.last_update) < timeout
    
    def set_pid_gains(self, kp: float, kd: float, ki: float = 0.0):
        """
        Set PID control gains
        
        Args:
            kp: Position gain
            kd: Velocity gain
            ki: Integral gain (usually 0 for AK motors)
        """
        self.kp = max(0.0, min(kp, 500.0))  # Clamp to AK60-6 limits
        self.kd = max(0.0, min(kd, 5.0))
        self.ki = ki  # Not used in AK60-6 protocol
        logger.info(f"Set PID gains for {self.joint_name}: kp={self.kp}, kd={self.kd}")
    
    def _clamp_position(self, position: float) -> float:
        """Apply position limits"""
        return max(self.position_limits[0], min(position, self.position_limits[1]))
    
    def _clamp_velocity(self, velocity: float) -> float:
        """Apply velocity limits"""  
        return max(-self.velocity_limit, min(velocity, self.velocity_limit))
    
    def _clamp_torque(self, torque: float) -> float:
        """Apply torque limits"""
        return max(-self.torque_limit, min(torque, self.torque_limit))
    
    def __str__(self) -> str:
        """String representation for debugging"""
        return (f"Motor({self.joint_name}, ID=0x{self.motor_id:02X}, "
                f"pos={self.state.position:.3f}, vel={self.state.velocity:.3f}, "
                f"enabled={self.state.enabled})")