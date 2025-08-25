"""
Quadruped robot controller for Pony robot
Manages all 12 AK60-6 motors (4 legs × 3 joints each)
Provides high-level movement commands and gait generation
"""

import time
import math
import logging
import threading
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

from .leg_controller import LegController, LegState
from .can_utils import CANInterface
from .utils.math_utils import GaitGenerator, KinematicsUtils
from .constants import *

logger = logging.getLogger(__name__)

class RobotState(Enum):
    """Robot operation states"""
    DISABLED = "disabled"
    INITIALIZING = "initializing"
    STANDING = "standing"
    WALKING = "walking"
    CROUCHING = "crouching"
    EMERGENCY = "emergency"

@dataclass
class RobotPose:
    """Robot body pose and leg positions"""
    body_height: float = 0.25          # Body height above ground (meters)
    body_roll: float = 0.0             # Body roll angle (radians)
    body_pitch: float = 0.0            # Body pitch angle (radians)
    body_yaw: float = 0.0              # Body yaw angle (radians)
    stance_width: float = 0.15         # Distance of feet from body centerline
    stance_length: float = 0.20        # Distance of feet from body center

@dataclass  
class MovementCommand:
    """High-level movement command"""
    linear_x: float = 0.0              # Forward/backward velocity (m/s)
    linear_y: float = 0.0              # Left/right velocity (m/s) 
    angular_z: float = 0.0             # Yaw rotation velocity (rad/s)
    body_height: float = 0.25          # Desired body height
    gait_type: str = "trot"            # Gait pattern: "trot", "walk", "bound"

class QuadrupedController:
    """
    Main controller for Pony quadruped robot
    Coordinates all 12 motors for walking and body control
    """
    
    def __init__(self, can_interface: CANInterface):
        """
        Initialize quadruped controller
        
        Args:
            can_interface: CAN interface for motor communication
        """
        self.can_interface = can_interface
        
        # Initialize all leg controllers
        self.legs: Dict[str, LegController] = {}
        for leg_name, motor_ids in MOTOR_IDS.items():
            self.legs[leg_name] = LegController(leg_name, motor_ids, can_interface)
            logger.info(f"Initialized {leg_name} controller")
        
        # Robot state
        self.state = RobotState.DISABLED
        self.pose = RobotPose()
        self.current_command = MovementCommand()
        
        # Gait generator
        self.gait_generator = GaitGenerator(
            stride_length=0.10,    # 10cm steps
            stride_height=0.05,    # 5cm foot lift
            stance_duration=0.6,   # 60% of cycle on ground
            swing_duration=0.4     # 40% of cycle in air
        )
        
        # Timing and control
        self.control_frequency = FEEDBACK_RATE  # 50 Hz
        self.gait_cycle_time = 0.0
        self.last_update_time = 0.0
        
        # Threading
        self._control_thread = None
        self._feedback_thread = None
        self._running = False
        self._lock = threading.Lock()
        
        # Default standing positions (relative to each hip joint)
        # These will be adjusted based on leg position (front/rear, left/right)
        self.default_foot_positions = {
            'leg1': (0.20, 0.0, -0.25),   # Front-right: forward, center, down
            'leg2': (0.20, 0.0, -0.25),   # Front-left: forward, center, down
            'leg3': (-0.15, 0.0, -0.25),  # Rear-right: back, center, down  
            'leg4': (-0.15, 0.0, -0.25),  # Rear-left: back, center, down
        }
        
        logger.info("Quadruped controller initialized")
    
    def start(self) -> bool:
        """
        Start robot control system
        
        Returns:
            True if started successfully
        """
        logger.info("Starting quadruped control system...")
        
        try:
            # Enable all motors
            if not self.enable_all_motors():
                logger.error("Failed to enable all motors")
                return False
            
            # Start control threads
            self._running = True
            
            # Start feedback monitoring thread
            self._feedback_thread = threading.Thread(target=self._feedback_worker, daemon=True)
            self._feedback_thread.start()
            
            # Start main control thread
            self._control_thread = threading.Thread(target=self._control_worker, daemon=True)
            self._control_thread.start()
            
            self.state = RobotState.INITIALIZING
            logger.info("Quadruped control system started")
            return True
            
        except Exception as e:
            logger.error(f"Failed to start control system: {e}")
            return False
    
    def stop(self):
        """Stop robot control system and disable motors"""
        logger.info("Stopping quadruped control system...")
        
        self._running = False
        
        # Wait for threads to finish
        if self._control_thread and self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        
        if self._feedback_thread and self._feedback_thread.is_alive():
            self._feedback_thread.join(timeout=2.0)
        
        # Move to safe position and disable motors
        try:
            self.crouch()
            time.sleep(2.0)
            self.disable_all_motors()
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
        
        logger.info("Quadruped control system stopped")
    
    def enable_all_motors(self) -> bool:
        """Enable all leg motors"""
        logger.info("Enabling all motors...")
        
        success = True
        for leg_name, leg in self.legs.items():
            if not leg.enable_all_motors():
                logger.error(f"Failed to enable {leg_name}")
                success = False
            time.sleep(0.2)  # Stagger motor enables
        
        if success:
            logger.info("All motors enabled")
        return success
    
    def disable_all_motors(self) -> bool:
        """Disable all leg motors"""
        logger.info("Disabling all motors...")
        
        success = True
        for leg_name, leg in self.legs.items():
            if not leg.disable_all_motors():
                logger.error(f"Failed to disable {leg_name}")
                success = False
            time.sleep(0.1)
        
        return success
    
    def stand_up(self, duration: float = 3.0) -> bool:
        """
        Stand up to default pose
        
        Args:
            duration: Time to complete standing motion
            
        Returns:
            True if successful
        """
        logger.info("Standing up...")
        
        with self._lock:
            self.state = RobotState.STANDING
        
        # Move all legs to standing positions simultaneously
        success = True
        for leg_name, (x, y, z) in self.default_foot_positions.items():
            if not self.legs[leg_name].move_to_cartesian(x, y, z, duration):
                logger.error(f"Failed to move {leg_name} to standing position")
                success = False
        
        return success
    
    def crouch(self, duration: float = 2.0) -> bool:
        """
        Crouch down (lower body position)
        
        Args:
            duration: Time to complete crouching motion
            
        Returns:
            True if successful
        """
        logger.info("Crouching down...")
        
        with self._lock:
            self.state = RobotState.CROUCHING
        
        # Lower all legs by 8cm from default position
        crouch_offset = -0.08
        success = True
        
        for leg_name, (x, y, z) in self.default_foot_positions.items():
            crouch_z = z + crouch_offset
            if not self.legs[leg_name].move_to_cartesian(x, y, crouch_z, duration):
                logger.error(f"Failed to move {leg_name} to crouch position")
                success = False
        
        return success
    
    def set_movement_command(self, command: MovementCommand):
        """
        Set high-level movement command
        
        Args:
            command: Movement command with velocities and gait
        """
        with self._lock:
            self.current_command = command
            
            # Change state based on command
            if abs(command.linear_x) > 0.01 or abs(command.linear_y) > 0.01 or abs(command.angular_z) > 0.01:
                if self.state == RobotState.STANDING:
                    self.state = RobotState.WALKING
                    logger.info(f"Started walking: vx={command.linear_x:.3f}, vy={command.linear_y:.3f}, "
                              f"vz={command.angular_z:.3f}")
            else:
                if self.state == RobotState.WALKING:
                    self.state = RobotState.STANDING
                    logger.info("Stopped walking, returning to standing")
    
    def emergency_stop(self):
        """Emergency stop - immediately halt all movement"""
        logger.warning("EMERGENCY STOP ACTIVATED!")
        
        with self._lock:
            self.state = RobotState.EMERGENCY
            self.current_command = MovementCommand()  # Zero all commands
        
        # Stop all leg movement
        for leg in self.legs.values():
            leg.stop_all_motors()
    
    def get_robot_status(self) -> Dict:
        """Get current robot status"""
        with self._lock:
            leg_states = {}
            for leg_name, leg in self.legs.items():
                leg_state = leg.get_leg_state()
                foot_pos = leg.get_foot_position()
                leg_states[leg_name] = {
                    'foot_position': foot_pos,
                    'joint_positions': [
                        leg_state.hip_state.position,
                        leg_state.knee_state.position,
                        leg_state.ankle_state.position
                    ],
                    'alive': leg.are_all_motors_alive()
                }
            
            return {
                'state': self.state.value,
                'pose': self.pose,
                'command': self.current_command,
                'legs': leg_states,
                'cycle_time': self.gait_cycle_time
            }
    
    def _control_worker(self):
        """Main control loop running at specified frequency"""
        logger.info("Control worker started")
        
        dt = 1.0 / self.control_frequency
        last_time = time.time()
        
        while self._running:
            current_time = time.time()
            actual_dt = current_time - last_time
            last_time = current_time
            
            try:
                # Update gait cycle time
                self.gait_cycle_time += actual_dt
                
                # Execute control based on current state
                with self._lock:
                    current_state = self.state
                    current_command = self.current_command
                
                if current_state == RobotState.WALKING:
                    self._execute_walking_gait(current_command, actual_dt)
                elif current_state == RobotState.STANDING:
                    self._maintain_standing_position()
                elif current_state == RobotState.INITIALIZING:
                    # Wait for initialization to complete
                    pass
                elif current_state == RobotState.EMERGENCY:
                    # Do nothing in emergency state
                    pass
                
            except Exception as e:
                logger.error(f"Control loop error: {e}")
            
            # Sleep to maintain control frequency
            elapsed = time.time() - current_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
        
        logger.info("Control worker stopped")
    
    def _feedback_worker(self):
        """Background worker to process CAN feedback messages"""
        logger.info("Feedback worker started")
        
        while self._running:
            try:
                # Receive CAN message with short timeout
                msg = self.can_interface.receive_message(timeout=0.1)
                if msg:
                    # Route message to appropriate leg
                    for leg in self.legs.values():
                        if leg.update_from_feedback(msg):
                            break  # Message handled
                            
            except Exception as e:
                if self._running:  # Only log if we should be running
                    logger.error(f"Feedback processing error: {e}")
                break
        
        logger.info("Feedback worker stopped")
    
    def _execute_walking_gait(self, command: MovementCommand, dt: float):
        """
        Execute walking gait based on movement command
        
        Args:
            command: Current movement command
            dt: Time step since last update
        """
        # Calculate gait cycle phase [0.0, 1.0]
        cycle_duration = self.gait_generator.cycle_time
        cycle_phase = (self.gait_cycle_time % cycle_duration) / cycle_duration
        
        # Generate foot positions for each leg based on gait pattern
        for leg_name, leg in self.legs.items():
            try:
                # Get default standing position for this leg
                default_x, default_y, default_z = self.default_foot_positions[leg_name]
                
                # Modify default position based on movement command
                # Add forward/backward movement
                target_x = default_x + command.linear_x * 0.1  # Scale velocity to position offset
                target_z = default_z  # Keep same height
                
                # Get foot position from gait generator
                foot_x, foot_y, foot_z = self.gait_generator.get_foot_position_for_gait(
                    leg_name, cycle_phase, target_x, target_z
                )
                
                # Send position command to leg
                leg.move_to_cartesian(foot_x, foot_y, foot_z, dt * 2)  # Smooth transitions
                
            except Exception as e:
                logger.error(f"Error updating {leg_name} during walking: {e}")
    
    def _maintain_standing_position(self):
        """Maintain stable standing position"""
        # Periodically send standing position commands to counter drift
        current_time = time.time()
        if current_time - self.last_update_time > 1.0:  # Update every second
            self.last_update_time = current_time
            
            for leg_name, (x, y, z) in self.default_foot_positions.items():
                self.legs[leg_name].set_joint_positions(
                    *self.legs[leg_name].inverse_kinematics(x, y, z)
                )