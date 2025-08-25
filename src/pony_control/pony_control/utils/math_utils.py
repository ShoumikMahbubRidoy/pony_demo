"""
Mathematical utilities for quadruped robot kinematics
Includes forward/inverse kinematics, interpolation, and coordinate transforms
"""

import math
import numpy as np
from typing import Tuple, List, Optional
import logging

logger = logging.getLogger(__name__)

class KinematicsUtils:
    """Utility class for kinematic calculations"""
    
    @staticmethod
    def forward_kinematics_3dof(hip_angle: float, knee_angle: float, ankle_angle: float,
                                hip_length: float, thigh_length: float, shin_length: float) -> Tuple[float, float, float]:
        """
        Calculate foot position from joint angles (3DOF leg)
        
        Args:
            hip_angle: Hip joint angle in radians (rotation about Y axis)
            knee_angle: Knee joint angle in radians  
            ankle_angle: Ankle joint angle in radians
            hip_length: Length from body to hip joint (meters)
            thigh_length: Upper leg length (meters)
            shin_length: Lower leg length (meters)
            
        Returns:
            Tuple of (x, y, z) foot position relative to hip joint
            x: forward/backward, y: left/right, z: up/down (negative is down)
        """
        # Calculate knee position relative to hip
        knee_x = thigh_length * math.cos(hip_angle)
        knee_z = -thigh_length * math.sin(hip_angle)  # Negative because down is negative Z
        
        # Calculate shin direction (hip + knee angles)
        shin_angle = hip_angle + knee_angle
        
        # Calculate ankle position relative to knee
        ankle_x = knee_x + shin_length * math.cos(shin_angle)
        ankle_z = knee_z - shin_length * math.sin(shin_angle)
        
        # For simplified 2D kinematics, Y is always 0
        # In full 3D, you would need additional hip abduction joint
        return ankle_x, 0.0, ankle_z
    
    @staticmethod
    def inverse_kinematics_3dof(target_x: float, target_z: float,
                               thigh_length: float, shin_length: float) -> Optional[Tuple[float, float, float]]:
        """
        Calculate joint angles for desired foot position (2D IK in sagittal plane)
        
        Args:
            target_x: Target X position (forward/backward from hip)
            target_z: Target Z position (up/down from hip, negative is down)
            thigh_length: Upper leg length (meters)
            shin_length: Lower leg length (meters)
            
        Returns:
            Tuple of (hip_angle, knee_angle, ankle_angle) in radians, or None if unreachable
        """
        # Calculate distance to target
        leg_reach = math.sqrt(target_x * target_x + target_z * target_z)
        
        # Check if target is reachable
        max_reach = thigh_length + shin_length
        min_reach = abs(thigh_length - shin_length)
        
        if leg_reach > max_reach * 0.99 or leg_reach < min_reach * 1.01:  # Small safety margin
            logger.warning(f"Target ({target_x:.3f}, {target_z:.3f}) not reachable. "
                         f"Distance: {leg_reach:.3f}, limits: [{min_reach:.3f}, {max_reach:.3f}]")
            return None
        
        try:
            # Use law of cosines to find knee angle
            cos_knee = (thigh_length**2 + shin_length**2 - leg_reach**2) / (2 * thigh_length * shin_length)
            cos_knee = max(-1.0, min(1.0, cos_knee))  # Clamp to valid range [-1, 1]
            knee_angle = math.pi - math.acos(cos_knee)  # Knee angle (positive = bent inward)
            
            # Calculate hip angle
            alpha = math.atan2(-target_z, target_x)  # Angle from hip to target
            beta = math.acos((thigh_length**2 + leg_reach**2 - shin_length**2) / (2 * thigh_length * leg_reach))
            hip_angle = alpha - beta
            
            # Calculate ankle angle to keep foot level (horizontal)
            # This compensates for hip and knee rotation to maintain foot orientation
            ankle_angle = -(hip_angle + knee_angle)
            
            return hip_angle, knee_angle, ankle_angle
            
        except (ValueError, ZeroDivisionError) as e:
            logger.error(f"IK calculation failed for target ({target_x:.3f}, {target_z:.3f}): {e}")
            return None
    
    @staticmethod
    def interpolate_positions(start_pos: List[float], end_pos: List[float], 
                             num_steps: int, interpolation_type: str = 'cubic') -> List[List[float]]:
        """
        Generate interpolated positions between start and end
        
        Args:
            start_pos: Starting position [x, y, z] or joint angles
            end_pos: Ending position [x, y, z] or joint angles  
            num_steps: Number of interpolation steps
            interpolation_type: 'linear', 'cubic', or 'quintic'
            
        Returns:
            List of interpolated positions
        """
        if len(start_pos) != len(end_pos):
            raise ValueError("Start and end positions must have same dimensions")
        
        positions = []
        
        for step in range(num_steps + 1):
            t = step / num_steps  # Normalized time [0, 1]
            
            # Apply different interpolation curves
            if interpolation_type == 'linear':
                t_smooth = t
            elif interpolation_type == 'cubic':
                t_smooth = 3 * t * t - 2 * t * t * t  # Smooth start and end
            elif interpolation_type == 'quintic':
                t_smooth = 6 * t**5 - 15 * t**4 + 10 * t**3  # Even smoother
            else:
                t_smooth = t  # Default to linear
            
            # Interpolate each dimension
            current_pos = []
            for i in range(len(start_pos)):
                interp_value = start_pos[i] + t_smooth * (end_pos[i] - start_pos[i])
                current_pos.append(interp_value)
            
            positions.append(current_pos)
        
        return positions
    
    @staticmethod
    def generate_trajectory_arc(start_pos: Tuple[float, float, float], 
                               end_pos: Tuple[float, float, float],
                               arc_height: float, num_steps: int = 20) -> List[Tuple[float, float, float]]:
        """
        Generate arc trajectory for foot lifting (useful for walking gaits)
        
        Args:
            start_pos: Starting foot position (x, y, z)
            end_pos: Ending foot position (x, y, z)
            arc_height: Maximum height of arc above start/end positions
            num_steps: Number of points in trajectory
            
        Returns:
            List of trajectory points
        """
        trajectory = []
        
        for step in range(num_steps + 1):
            t = step / num_steps
            
            # Linear interpolation in X and Y
            x = start_pos[0] + t * (end_pos[0] - start_pos[0])
            y = start_pos[1] + t * (end_pos[1] - start_pos[1])
            
            # Parabolic arc in Z
            z_base = start_pos[2] + t * (end_pos[2] - start_pos[2])
            z_arc = arc_height * math.sin(math.pi * t)  # Parabolic arc
            z = z_base + z_arc
            
            trajectory.append((x, y, z))
        
        return trajectory
    
    @staticmethod
    def clamp_angle(angle: float, min_angle: float, max_angle: float) -> float:
        """
        Clamp angle to specified range
        
        Args:
            angle: Input angle in radians
            min_angle: Minimum allowed angle
            max_angle: Maximum allowed angle
            
        Returns:
            Clamped angle
        """
        return max(min_angle, min(angle, max_angle))
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-pi, pi] range
        
        Args:
            angle: Input angle in radians
            
        Returns:
            Normalized angle in [-pi, pi]
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    @staticmethod
    def degrees_to_radians(degrees: float) -> float:
        """Convert degrees to radians"""
        return degrees * math.pi / 180.0
    
    @staticmethod
    def radians_to_degrees(radians: float) -> float:
        """Convert radians to degrees"""
        return radians * 180.0 / math.pi

class GaitGenerator:
    """Simple gait pattern generator for quadruped locomotion"""
    
    def __init__(self, stride_length: float = 0.1, stride_height: float = 0.05, 
                 stance_duration: float = 0.6, swing_duration: float = 0.4):
        """
        Initialize gait generator
        
        Args:
            stride_length: Forward step distance (meters)
            stride_height: Foot lift height during swing (meters)
            stance_duration: Time foot is on ground (seconds)
            swing_duration: Time foot is in air (seconds)
        """
        self.stride_length = stride_length
        self.stride_height = stride_height
        self.stance_duration = stance_duration
        self.swing_duration = swing_duration
        self.cycle_time = stance_duration + swing_duration
    
    def generate_trot_pattern(self, cycle_phase: float) -> dict:
        """
        Generate trot gait pattern (diagonal legs move together)
        
        Args:
            cycle_phase: Current phase in gait cycle [0.0, 1.0]
            
        Returns:
            Dictionary with leg states: {'leg1': 'swing'|'stance', ...}
        """
        # Normalize phase to [0, 1]
        phase = cycle_phase % 1.0
        
        # Trot pattern: diagonal pairs alternate
        # Pair 1: leg1 (front-right) + leg4 (rear-left)
        # Pair 2: leg2 (front-left) + leg3 (rear-right)
        
        pattern = {}
        
        if phase < 0.5:
            # First half: pair 1 swings, pair 2 in stance
            pattern['leg1'] = 'swing'
            pattern['leg4'] = 'swing'
            pattern['leg2'] = 'stance'
            pattern['leg3'] = 'stance'
        else:
            # Second half: pair 2 swings, pair 1 in stance
            pattern['leg1'] = 'stance'
            pattern['leg4'] = 'stance'
            pattern['leg2'] = 'swing'
            pattern['leg3'] = 'swing'
        
        return pattern
    
    def get_foot_position_for_gait(self, leg_name: str, cycle_phase: float, 
                                  default_x: float, default_z: float) -> Tuple[float, float, float]:
        """
        Calculate foot position for given leg in gait cycle
        
        Args:
            leg_name: Name of leg ('leg1', 'leg2', etc.)
            cycle_phase: Current phase in gait cycle [0.0, 1.0]
            default_x: Default forward position when standing
            default_z: Default down position when standing
            
        Returns:
            Tuple of (x, y, z) foot position
        """
        pattern = self.generate_trot_pattern(cycle_phase)
        leg_state = pattern.get(leg_name, 'stance')
        
        if leg_state == 'stance':
            # Foot on ground, moving backward relative to body
            x_offset = -self.stride_length * (cycle_phase * 2 - 1)  # -stride to +stride
            return (default_x + x_offset, 0.0, default_z)
        else:
            # Foot in swing phase, follow arc trajectory
            swing_phase = (cycle_phase % 0.5) * 2  # Normalize to [0, 1] within swing
            
            # Arc from back to front
            start_x = default_x - self.stride_length / 2
            end_x = default_x + self.stride_length / 2
            
            x = start_x + swing_phase * (end_x - start_x)
            z = default_z + self.stride_height * math.sin(math.pi * swing_phase)
            
            return (x, 0.0, z)