"""
Constants and configuration for Pony Quadruped Robot
AK60-6 V3.0 motors with CAN communication
"""

# CAN Communication Settings
CAN_INTERFACE = "can0"  # Will be mapped to InnoMaker USB-to-CAN
CAN_BITRATE = 1000000   # 1 Mbps
CAN_TIMEOUT = 1.0       # 1 second timeout

# Motor IDs mapping (as specified in your documentation)
# Format: [Hip, Knee, Ankle] for each leg
MOTOR_IDS = {
    'leg1': [0x2940, 0x2941, 0x2942],  # [64, 65, 66]
    'leg2': [0x2950, 0x2951, 0x2952],  # [80, 81, 82]
    'leg3': [0x2960, 0x2961, 0x2962],  # [96, 97, 98]
    'leg4': [0x2970, 0x2971, 0x2972],  # [112, 113, 114]
}

# Joint names for easier reference
JOINT_NAMES = ['hip', 'knee', 'ankle']

# All motor IDs in a flat list for easy iteration
ALL_MOTOR_IDS = [motor_id for leg_motors in MOTOR_IDS.values() for motor_id in leg_motors]

# AK60-6 Motor Specifications
MOTOR_SPECS = {
    'max_torque': 12.0,        # Nm (estimated for AK60-6)
    'max_velocity': 38.2,      # rad/s
    'max_position': 12.5,      # rad (approximately 2π)
    'min_position': -12.5,     # rad
    'gear_ratio': 6.0,         # 6:1 gear ratio
}

# AK60-6 CAN Protocol Commands
CAN_COMMANDS = {
    'MOTOR_MODE': 0xFC,        # Enter motor mode
    'RESET_MODE': 0xFD,        # Reset/exit motor mode
    'ZERO_POSITION': 0xFE,     # Set current position as zero
}

# Control modes for AK60-6
CONTROL_MODES = {
    'POSITION': 0,
    'VELOCITY': 1, 
    'TORQUE': 2,
    'POSITION_VELOCITY': 3,
    'POSITION_TORQUE': 4,
}

# Safety limits (you may need to adjust these based on your robot design)
SAFETY_LIMITS = {
    'hip': {
        'min_pos': -1.57,      # -90 degrees
        'max_pos': 1.57,       # +90 degrees
        'max_vel': 10.0,       # rad/s
        'max_torque': 8.0,     # Nm
    },
    'knee': {
        'min_pos': -2.36,      # -135 degrees  
        'max_pos': 0.0,        # 0 degrees
        'max_vel': 10.0,       # rad/s
        'max_torque': 8.0,     # Nm
    },
    'ankle': {
        'min_pos': -1.57,      # -90 degrees
        'max_pos': 1.57,       # +90 degrees
        'max_vel': 10.0,       # rad/s
        'max_torque': 6.0,     # Nm
    }
}

# Robot physical parameters (you'll need to measure these)
ROBOT_PARAMS = {
    'hip_length': 0.08,        # meters - length from body to hip joint
    'thigh_length': 0.20,      # meters - upper leg length
    'shin_length': 0.20,       # meters - lower leg length
    'body_width': 0.16,        # meters - distance between left/right legs
    'body_length': 0.30,       # meters - distance between front/rear legs
}

# Default PID gains for position control (you may need to tune these)
DEFAULT_PID_GAINS = {
    'kp': 100.0,    # Position gain
    'kd': 1.0,      # Velocity gain  
    'ki': 0.0,      # Integral gain (usually 0 for AK motors)
}

# Feedback rate
FEEDBACK_RATE = 50  # Hz

# ROS2 topic names
ROS_TOPICS = {
    'joint_states': '/joint_states',
    'joint_commands': '/joint_commands', 
    'motor_status': '/motor_status',
    'keyboard_input': '/keyboard_input',
}

# Test positions for motor validation
TEST_POSITIONS = {
    'home': [0.0, 0.0, 0.0],           # All joints at zero
    'crouch': [0.0, -1.0, 0.5],       # Crouched position
    'stand': [0.0, -0.5, 0.3],        # Standing position
}