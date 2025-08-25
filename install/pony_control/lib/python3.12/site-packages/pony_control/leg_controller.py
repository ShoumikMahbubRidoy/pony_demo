"""
Single leg controller for quadruped robot
Manages coordinated movement of 3 motors (hip, knee, ankle) for one leg
"""

import time
import math
import logging
import threading
from typing import List, Tuple, Optional
from dataclasses import dataclass

from .motor_controller import MotorController, MotorState
from .can_utils import CANInterface
from .constants import *

logger = logging.getLogger(__name__)

@dataclass
class LegState:
    """Data class to hold complete leg state"""
    hip_state: MotorState
    knee_state: MotorState  
    ankle_state: MotorState
    timestamp: float = 0.0

@dataclass
class LegTarget:
    """Data class for leg target positions"""
    hip_pos: float = 0.0
    knee_pos: float = 0.0
    ankle_pos: float = 0.0
    hip_vel: float = 0.0
    knee_vel: float = 0.0
    ankle_vel: float = 0.0

class LegController:
    """
    Controller for single leg (3 motors: hip, knee, ankle)
    Provides coordinated movement and basic inverse kinematics
    """
    
    def __init__(self, leg_name: str, motor_ids: List[int], can_interface: CANInterface):
        """
        Initialize leg controller
        
        Args:
            leg_name: Name of the leg (leg1, leg2, leg3, leg4)
            motor_ids: List of 3 motor IDs [hip, knee, ankle]
            can_interface: CAN interface instance
        """
        self.leg_name = leg_name
        self.motor_ids = motor_ids
        self.can_interface = can_interface
        
        # Initialize motor controllers
        self.motors = {}
        joint_names = JOINT_NAMES  # ['hip', 'knee', 'ankle']
        
        for i, (joint_name, motor_id) in enumerate(zip(joint_names, motor_ids)):
            controller_name = f"{leg_name}_{joint_name}"
            self.motors[joint_name] = MotorController(
                motor_id, can_interface, controller_name
            )
            # Set appropriate safety limits for each joint
            self.motors[joint_name].set_limits(joint_name)
        
        # Leg geometry (from ROBOT_PARAMS)
        self.hip_length = ROBOT_PARAMS['hip_length']
        self.thigh_length = ROBOT_PARAMS['thigh_length']  
        self.shin_length = ROBOT_PARAMS['shin_length']
        
        # Current targets
        self.current_target = LegTarget()
        
        # Thread safety
        self._lock = threading.Lock()
        
        # Movement interpolation
        self._interpolation_active = False
        self._interpolation_thread = None
        
        logger.info(f"Initialized leg controller for {leg_name} with motors: "
                   f"hip=0x{motor_ids[0]:02X}, knee=0x{motor_ids[1]:02X}, ankle=0x{motor_ids[2]:02X}")
    
    def enable_all_motors(self) -> bool:
        """
        Enable all motors in the leg
        
        Returns:
            True if all motors enabled successfully
        """
        logger.info(f"Enabling all motors for {self.leg_name}")
        
        success = True
        for joint_name, motor in self.motors.items():
            if not motor.enable_motor():
                logger.error(f"Failed to enable {joint_name} motor")
                success = False
            time.sleep(0.1)  # Small delay between enables
        
        if success:
            logger.info(f"All motors enabled for {self.leg_name}")
        return success
    
    def disable_all_motors(self) -> bool:
        """
        Disable all motors in the leg
        
        Returns:
            True if all motors disabled successfully
        """
        logger.info(f"Disabling all motors for {self.leg_name}")
        
        success = True
        for joint_name, motor in self.motors.items():
            if not motor.disable_motor():
                logger.error(f"Failed to disable {joint_name} motor")
                success = False
            time.sleep(0.1)
        
        if success:
            logger.info(f"All motors disabled for {self.leg_name}")
        return success
    
    def zero_all_motors(self) -> bool:
        """
        Zero all motor positions
        
        Returns:
            True if all motors zeroed successfully
        """
        logger.info(f"Zeroing all motors for {self.leg_name}")
        
        success = True
        for joint_name, motor in self.motors.items():
            if not motor.zero_position():
                logger.error(f"Failed to zero {joint_name} motor")
                success = False
            time.sleep(0.1)
        
        return success
    
    def set_joint_positions(self, hip_pos: float, knee_pos: float, ankle_pos: float,
                           velocities: Optional[Tuple[float, float, float]] = None) -> bool:
        """
        Set target positions for all joints
        
        Args:
            hip_pos: Hip position in radians
            knee_pos: Knee position in radians  
            ankle_pos: Ankle position in radians
            velocities: Optional target velocities (hip_vel, knee_vel, ankle_vel)
            
        Returns:
            True if all commands sent successfully
        """
        if velocities is None:
            velocities = (0.0, 0.0, 0.0)
        
        positions = [hip_pos, knee_pos, ankle_pos]
        joint_names = ['hip', 'knee', 'ankle']
        
        with self._lock:
            self.current_target.hip_pos = hip_pos
            self.current_target.knee_pos = knee_pos
            self.current_target.ankle_pos = ankle_pos
            self.current_target.hip_vel = velocities[0]
            self.current_target.knee_vel = velocities[1]
            self.current_target.ankle_vel = velocities[2]
        
        success = True
        for i, (joint_name, pos, vel) in enumerate(zip(joint_names, positions, velocities)):
            if not self.motors[joint_name].set_position(pos, vel):
                logger.error(f"Failed to set {joint_name} position to {pos:.3f}")
                success = False
        
        return success
    
    def move_to_cartesian(self, x: float, y: float, z: float, duration: float = 1.0) -> bool:
        """
        Move leg tip to Cartesian coordinates using inverse kinematics
        
        Args:
            x: X coordinate relative to hip (forward/backward)
            y: Y coordinate relative to hip (left/right) 
            z: Z coordinate relative to hip (up/down, negative is down)
            duration: Movement duration in seconds
            
        Returns:
            True if movement initiated successfully
        """
        # Calculate inverse kinematics
        joint_angles = self.inverse_kinematics(x, y, z)
        if joint_angles is None:
            logger.error(f"Inverse kinematics failed for position ({x:.3f}, {y:.3f}, {z:.3f})")
            return False
        
        hip_angle, knee_angle, ankle_angle = joint_angles
        logger.info(f"Moving {self.leg_name} to cartesian ({x:.3f}, {y:.3f}, {z:.3f}) -> "
                   f"joints ({math.degrees(hip_angle):.1f}°, {math.degrees(knee_angle):.1f}°, "
                   f"{math.degrees(ankle_angle):.1f}°)")
        
        # Use interpolated movement for smooth motion
        return self.interpolate_to_position(hip_angle, knee_angle, ankle_angle, duration)
    
    def interpolate_to_position(self, target_hip: float, target_knee: float, 
                              target_ankle: float, duration: float = 1.0) -> bool:
        """
        Smoothly interpolate to target joint positions
        
        Args:
            target_hip: Target hip position in radians
            target_knee: Target knee position in radians
            target_ankle: Target ankle position in radians
            duration: Interpolation duration in seconds
            
        Returns:
            True if interpolation started successfully
        """
        # Stop any existing interpolation
        if self._interpolation_active:
            self._interpolation_active = False
            if self._interpolation_thread and self._interpolation_thread.is_alive():
                self._interpolation_thread.join()
        
        # Start new interpolation in separate thread
        self._interpolation_thread = threading.Thread(
            target=self._interpolation_worker,
            args=(target_hip, target_knee, target_ankle, duration)
        )
        self._interpolation_active = True
        self._interpolation_thread.start()
        
        return True
    
    def _interpolation_worker(self, target_hip: float, target_knee: float, 
                            target_ankle: float, duration: float):
        """Worker function for position interpolation"""
        # Get current positions
        current_state = self.get_leg_state()
        start_hip = current_state.hip_state.position
        start_knee = current_state.knee_state.position
        start_ankle = current_state.ankle_state.position
        
        # Calculate interpolation parameters
        steps = int(duration * FEEDBACK_RATE)  # 50 Hz interpolation
        dt = duration / steps
        
        logger.info(f"Starting interpolation over {duration:.2f}s ({steps} steps)")
        
        for step in range(steps + 1):
            if not self._interpolation_active:
                logger.info("Interpolation stopped early")
                break
            
            # Calculate interpolation factor (0 to 1)
            t = step / steps
            # Use cubic interpolation for smoother motion
            t_smooth = 3 * t * t - 2 * t * t * t
            
            # Interpolate positions
            hip_pos = start_hip + t_smooth * (target_hip - start_hip)
            knee_pos = start_knee + t_smooth * (target_knee - start_knee)
            ankle_pos = start_ankle + t_smooth * (target_ankle - start_ankle)
            
            # Send position commands
            self.set_joint_positions(hip_pos, knee_pos, ankle_pos)
            
            time.sleep(dt)
        
        self._interpolation_active = False
        logger.info("Interpolation completed")
    
    def inverse_kinematics(self, x: float, y: float, z: float) -> Optional[Tuple[float, float, float]]:
        """
        Calculate joint angles for desired foot position
        Simplified 2D IK for leg in sagittal plane
        
        Args:
            x: Forward distance from hip
            y: Lateral distance from hip (not used in 2D IK)
            z: Vertical distance from hip (negative is down)
            
        Returns:
            Tuple of (hip_angle, knee_angle, ankle_angle) or None if unreachable
        """
        # Convert to 2D problem in the sagittal plane
        leg_reach = math.sqrt(x * x + z * z)
        
        # Check if position is reachable
        max_reach = self.thigh_length + self.shin_length
        min_reach = abs(self.thigh_length - self.shin_length)
        
        if leg_reach > max_reach or leg_reach < min_reach:
            logger.warning(f"Position ({x:.3f}, {z:.3f}) not reachable. "
                         f"Reach: {leg_reach:.3f}, limits: [{min_reach:.3f}, {max_reach:.3f}]")
            return None
        
        try:
            # Calculate knee angle using law of cosines
            cos_knee = (self.thigh_length**2 + self.shin_length**2 - leg_reach**2) / \
                       (2 * self.thigh_length * self.shin_length)
            cos_knee = max(-1.0, min(1.0, cos_knee))  # Clamp to valid range
            knee_angle = math.pi - math.acos(cos_knee)  # Knee angle (positive is bent)
            
            # Calculate hip angle
            alpha = math.atan2(-z, x)  # Angle to target from hip
            beta = math.acos((self.thigh_length**2 + leg_reach**2 - self.shin_length**2) / 
                            (2 * self.thigh_length * leg_reach))
            hip_angle = alpha - beta
            
            # Calculate ankle angle to keep foot level
            # Ankle angle compensates for hip and knee to maintain foot orientation
            ankle_angle = -(hip_angle + knee_angle)
            
            return hip_angle, knee_angle, ankle_angle
            
        except (ValueError, ZeroDivisionError) as e:
            logger.error(f"IK calculation failed: {e}")
            return None
    
    def forward_kinematics(self, hip_angle: float, knee_angle: float, 
                          ankle_angle: float) -> Tuple[float, float, float]:
        """
        Calculate foot position from joint angles
        
        Args:
            hip_angle: Hip angle in radians
            knee_angle: Knee angle in radians
            ankle_angle: Ankle angle in radians
            
        Returns:
            Tuple of (x, y, z) foot position relative to hip
        """
        # Calculate position of knee relative to hip
        knee_x = self.thigh_length * math.cos(hip_angle)
        knee_z = -self.thigh_length * math.sin(hip_angle)
        
        # Calculate position of ankle relative to knee
        shin_angle = hip_angle + knee_angle
        ankle_x = knee_x + self.shin_length * math.cos(shin_angle)
        ankle_z = knee_z - self.shin_length * math.sin(shin_angle)
        
        # For this simplified model, y is always 0 (2D kinematics)
        return ankle_x, 0.0, ankle_z
    
    def update_from_feedback(self, msg) -> bool:
        """
        Update motor states from CAN feedback
        
        Args:
            msg: CAN feedback message
            
        Returns:
            True if message was for this leg
        """
        motor_id = msg.arbitration_id
        
        # Check if this message is for one of our motors
        for joint_name, motor in self.motors.items():
            if motor.motor_id == motor_id:
                return motor.update_state_from_feedback(msg)
        
        return False
    
    def get_leg_state(self) -> LegState:
        """
        Get current state of all leg joints
        
        Returns:
            Complete leg state
        """
        with self._lock:
            return LegState(
                hip_state=self.motors['hip'].get_state(),
                knee_state=self.motors['knee'].get_state(), 
                ankle_state=self.motors['ankle'].get_state(),
                timestamp=time.time()
            )
    
    def get_foot_position(self) -> Tuple[float, float, float]:
        """
        Get current foot position using forward kinematics
        
        Returns:
            Tuple of (x, y, z) foot position
        """
        state = self.get_leg_state()
        return self.forward_kinematics(
            state.hip_state.position,
            state.knee_state.position,
            state.ankle_state.position
        )
    
    def are_all_motors_alive(self, timeout: float = 1.0) -> bool:
        """
        Check if all motors are responding
        
        Args:
            timeout: Maximum time since last feedback
            
        Returns:
            True if all motors are alive
        """
        for motor in self.motors.values():
            if not motor.is_alive(timeout):
                return False
        return True
    
    def stop_all_motors(self):
        """Emergency stop - set all velocities to zero"""
        logger.warning(f"Emergency stop for {self.leg_name}")
        for motor in self.motors.values():
            motor.set_velocity(0.0)
        
        # Stop interpolation
        self._interpolation_active = False
    
    def move_to_home_position(self, duration: float = 2.0) -> bool:
        """
        Move leg to home (standing) position
        
        Args:
            duration: Movement duration in seconds
            
        Returns:
            True if movement initiated successfully
        """
        home_positions = TEST_POSITIONS['stand']  # [0.0, -0.5, 0.3]
        logger.info(f"Moving {self.leg_name} to home position")
        return self.interpolate_to_position(
            home_positions[0], home_positions[1], home_positions[2], duration
        )
    
    def __str__(self) -> str:
        """String representation for debugging"""
        state = self.get_leg_state()
        return (f"Leg({self.leg_name}: "
                f"hip={math.degrees(state.hip_state.position):.1f}°, "
                f"knee={math.degrees(state.knee_state.position):.1f}°, "
                f"ankle={math.degrees(state.ankle_state.position):.1f}°)")