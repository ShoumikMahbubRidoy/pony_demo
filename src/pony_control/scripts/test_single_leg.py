#!/usr/bin/env python3
"""
Test script for single leg (3 motors: hip, knee, ankle)
Tests coordinated leg movement and basic kinematics
Run this after single motor test passes
"""

import time
import logging
import sys
import signal
import math

# Add the package to path (adjust if needed)
sys.path.append('/home/pi/pony_quadruped/src/pony_control')

from pony_control.leg_controller import LegController
from pony_control.can_utils import CANInterface
from pony_control.constants import *

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SingleLegTest:
    """Test class for single leg functionality"""
    
    def __init__(self, leg_name: str = 'leg1'):
        """
        Initialize test with specified leg
        
        Args:
            leg_name: Name of leg to test (leg1, leg2, leg3, leg4)
        """
        if leg_name not in MOTOR_IDS:
            raise ValueError(f"Invalid leg name: {leg_name}. Must be one of {list(MOTOR_IDS.keys())}")
        
        self.leg_name = leg_name
        self.motor_ids = MOTOR_IDS[leg_name]
        self.can_interface = None
        self.leg = None
        self.running = True
        
        # Setup signal handler for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        logger.info(f"Initialized test for {leg_name} with motors: "
                   f"hip=0x{self.motor_ids[0]:02X}, knee=0x{self.motor_ids[1]:02X}, "
                   f"ankle=0x{self.motor_ids[2]:02X}")
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logger.info("Received shutdown signal, stopping leg...")
        self.running = False
    
    def setup(self) -> bool:
        """
        Initialize CAN interface and leg controller
        
        Returns:
            True if setup successful
        """
        logger.info(f"Setting up test for {self.leg_name}")
        
        try:
            # Initialize CAN interface
            self.can_interface = CANInterface(CAN_INTERFACE, CAN_BITRATE)
            if not self.can_interface.connect():
                logger.error("Failed to connect to CAN interface")
                return False
            
            # Initialize leg controller
            self.leg = LegController(self.leg_name, self.motor_ids, self.can_interface)
            
            logger.info("Setup completed successfully")
            return True
            
        except Exception as e:
            logger.error(f"Setup failed: {e}")
            return False
    
    def test_motor_communication(self) -> bool:
        """
        Test communication with all leg motors
        
        Returns:
            True if all motors communicate successfully
        """
        logger.info("Testing communication with all leg motors...")
        
        # Enable all motors
        if not self.leg.enable_all_motors():
            logger.error("Failed to enable all motors")
            return False
        
        time.sleep(1.0)  # Wait for motors to respond
        
        # Send zero position commands
        if not self.leg.set_joint_positions(0.0, 0.0, 0.0):
            logger.error("Failed to send joint position commands")
            return False
        
        # Listen for feedback from all motors
        start_time = time.time()
        motor_feedback = {motor_id: False for motor_id in self.motor_ids}
        
        while time.time() - start_time < 5.0:  # Wait up to 5 seconds
            msg = self.can_interface.receive_message(timeout=0.1)
            if msg and msg.arbitration_id in self.motor_ids:
                if self.leg.update_from_feedback(msg):
                    motor_feedback[msg.arbitration_id] = True
                    joint_name = JOINT_NAMES[self.motor_ids.index(msg.arbitration_id)]
                    motor = self.leg.motors[joint_name]
                    logger.info(f"Received feedback from {joint_name}: "
                              f"pos={motor.state.position:.3f}, "
                              f"vel={motor.state.velocity:.3f}, "
                              f"torque={motor.state.torque:.3f}")
                
                # Check if all motors responded
                if all(motor_feedback.values()):
                    break
        
        # Check results
        failed_motors = [f"0x{motor_id:02X}" for motor_id, received 
                        in motor_feedback.items() if not received]
        
        if failed_motors:
            logger.error(f"No feedback received from motors: {failed_motors}")
            return False
        
        logger.info("Communication test passed for all motors!")
        return True
    
    def test_individual_joints(self) -> bool:
        """
        Test each joint individually with small movements
        
        Returns:
            True if all joint tests pass
        """
        logger.info("Testing individual joint movements...")
        
        # Test positions for each joint (small safe movements)
        joint_tests = {
            'hip': [0.0, 0.3, -0.3, 0.0],      # ±17 degrees
            'knee': [0.0, -0.5, -1.0, 0.0],    # 0 to -57 degrees (knee bending)
            'ankle': [0.0, 0.4, -0.4, 0.0]     # ±23 degrees
        }
        
        for joint_name, test_positions in joint_tests.items():
            if not self.running:
                break
                
            logger.info(f"Testing {joint_name} joint...")
            
            for target_pos in test_positions:
                if not self.running:
                    break
                    
                logger.info(f"Moving {joint_name} to {target_pos:.3f} rad "
                          f"({math.degrees(target_pos):.1f}°)")
                
                # Create position command with only this joint moving
                positions = [0.0, 0.0, 0.0]  # [hip, knee, ankle]
                joint_index = JOINT_NAMES.index(joint_name)
                positions[joint_index] = target_pos
                
                # Send command
                if not self.leg.set_joint_positions(*positions):
                    logger.error(f"Failed to command {joint_name}")
                    return False
                
                # Monitor movement for 2 seconds
                start_time = time.time()
                while time.time() - start_time < 2.0:
                    if not self.running:
                        break
                        
                    # Receive feedback
                    msg = self.can_interface.receive_message(timeout=0.1)
                    if msg and msg.arbitration_id in self.motor_ids:
                        self.leg.update_from_feedback(msg)
                        
                        # Check if target reached
                        motor = self.leg.motors[joint_name]
                        current_pos = motor.state.position
                        error = abs(current_pos - target_pos)
                        
                        if error < 0.05:  # 0.05 rad ≈ 3 degrees tolerance
                            logger.info(f"{joint_name} reached target! "
                                      f"Current: {current_pos:.3f} rad")
                            break
                    
                    time.sleep(0.02)  # 50 Hz update
                
                time.sleep(0.5)  # Pause between movements
        
        logger.info("Individual joint tests completed!")
        return True
    
    def test_coordinated_movement(self) -> bool:
        """
        Test coordinated movement of all joints
        
        Returns:
            True if coordinated movement test passes
        """
        logger.info("Testing coordinated leg movements...")
        
        # Define test poses: [hip, knee, ankle] in radians
        test_poses = [
            [0.0, 0.0, 0.0],        # Home position
            [0.2, -0.5, 0.3],       # Slight forward lean
            [-0.2, -0.8, 0.6],      # Crouch back
            [0.0, -1.2, 0.8],       # Deep crouch
            [0.1, -0.3, 0.2],       # Standing
            [0.0, 0.0, 0.0],        # Return home
        ]
        
        for i, (hip, knee, ankle) in enumerate(test_poses):
            if not self.running:
                break
                
            logger.info(f"Moving to pose {i+1}: hip={math.degrees(hip):.1f}°, "
                       f"knee={math.degrees(knee):.1f}°, ankle={math.degrees(ankle):.1f}°")
            
            # Use interpolated movement for smooth motion
            if not self.leg.interpolate_to_position(hip, knee, ankle, duration=2.0):
                logger.error(f"Failed to start interpolation to pose {i+1}")
                return False
            
            # Wait for movement to complete
            time.sleep(2.5)
            
            # Monitor feedback during pause
            monitor_start = time.time()
            while time.time() - monitor_start < 0.5:  # Monitor for 0.5 seconds
                msg = self.can_interface.receive_message(timeout=0.05)
                if msg and msg.arbitration_id in self.motor_ids:
                    self.leg.update_from_feedback(msg)
            
            # Print current state
            state = self.leg.get_leg_state()
            logger.info(f"Current positions: hip={math.degrees(state.hip_state.position):.1f}°, "
                       f"knee={math.degrees(state.knee_state.position):.1f}°, "
                       f"ankle={math.degrees(state.ankle_state.position):.1f}°")
        
        logger.info("Coordinated movement test completed!")
        return True
    
    def test_cartesian_movement(self) -> bool:
        """
        Test Cartesian space movement using inverse kinematics
        
        Returns:
            True if Cartesian movement test passes
        """
        logger.info("Testing Cartesian space movements...")
        
        # Define test positions in Cartesian space (x, y, z) relative to hip
        # x: forward/backward, y: left/right, z: up/down (negative is down)
        test_positions = [
            (0.25, 0.0, -0.3),   # Forward, down (standing position)
            (0.30, 0.0, -0.25),  # Forward, slightly up
            (0.15, 0.0, -0.35),  # Back, down
            (0.20, 0.0, -0.20),  # Center, high
            (0.25, 0.0, -0.30),  # Return to start
        ]
        
        for i, (x, y, z) in enumerate(test_positions):
            if not self.running:
                break
                
            logger.info(f"Moving to Cartesian position {i+1}: "
                       f"({x:.3f}, {y:.3f}, {z:.3f}) meters")
            
            # Use Cartesian movement with inverse kinematics
            if not self.leg.move_to_cartesian(x, y, z, duration=2.5):
                logger.error(f"Failed to move to Cartesian position {i+1}")
                return False
            
            # Wait for movement to complete
            time.sleep(3.0)
            
            # Monitor feedback and calculate actual foot position
            monitor_start = time.time()
            while time.time() - monitor_start < 0.5:
                msg = self.can_interface.receive_message(timeout=0.05)
                if msg and msg.arbitration_id in self.motor_ids:
                    self.leg.update_from_feedback(msg)
            
            # Calculate and display actual foot position
            actual_foot_pos = self.leg.get_foot_position()
            logger.info(f"Target position: ({x:.3f}, {y:.3f}, {z:.3f})")
            logger.info(f"Actual position: ({actual_foot_pos[0]:.3f}, "
                       f"{actual_foot_pos[1]:.3f}, {actual_foot_pos[2]:.3f})")
            
            # Calculate error
            error = math.sqrt((x - actual_foot_pos[0])**2 + 
                            (z - actual_foot_pos[2])**2)  # Ignore y for 2D
            logger.info(f"Position error: {error:.3f} meters")
        
        logger.info("Cartesian movement test completed!")
        return True
    
    def test_leg_state_monitoring(self, duration: float = 10.0):
        """
        Monitor leg state for specified duration
        
        Args:
            duration: Monitoring duration in seconds
        """
        logger.info(f"Monitoring leg state for {duration} seconds...")
        
        # Move to a test position first
        self.leg.set_joint_positions(0.1, -0.5, 0.3)
        time.sleep(1.0)
        
        start_time = time.time()
        while time.time() - start_time < duration and self.running:
            # Receive and process feedback
            msg = self.can_interface.receive_message(timeout=0.1)
            if msg and msg.arbitration_id in self.motor_ids:
                self.leg.update_from_feedback(msg)
            
            # Print state every 2 seconds
            if int((time.time() - start_time) * 0.5) != int((time.time() - start_time - 0.1) * 0.5):
                state = self.leg.get_leg_state()
                foot_pos = self.leg.get_foot_position()
                
                logger.info(f"Leg State - Hip: {math.degrees(state.hip_state.position):.1f}°, "
                          f"Knee: {math.degrees(state.knee_state.position):.1f}°, "
                          f"Ankle: {math.degrees(state.ankle_state.position):.1f}°")
                logger.info(f"Foot Position: ({foot_pos[0]:.3f}, {foot_pos[1]:.3f}, "
                          f"{foot_pos[2]:.3f}) meters")
                
                # Check motor health
                if not self.leg.are_all_motors_alive():
                    logger.warning("Some motors are not responding!")
            
            time.sleep(0.1)
    
    def run_all_tests(self) -> bool:
        """
        Run complete leg test suite
        
        Returns:
            True if all tests pass
        """
        logger.info("Starting complete leg test suite...")
        
        if not self.setup():
            return False
        
        try:
            # Test 1: Motor communication
            if not self.test_motor_communication():
                logger.error("Motor communication test failed!")
                return False
            
            # Test 2: Individual joints
            if not self.test_individual_joints():
                logger.error("Individual joint test failed!")
                return False
            
            # Test 3: Coordinated movement
            if not self.test_coordinated_movement():
                logger.error("Coordinated movement test failed!")
                return False
            
            # Test 4: Cartesian movement
            if not self.test_cartesian_movement():
                logger.error("Cartesian movement test failed!")
                return False
            
            # Test 5: State monitoring
            self.test_leg_state_monitoring(5.0)
            
            logger.info("All leg tests completed successfully!")
            return True
            
        except Exception as e:
            logger.error(f"Test failed with exception: {e}")
            return False
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean shutdown"""
        logger.info("Cleaning up...")
        
        if self.leg:
            # Move to safe position and disable
            logger.info("Moving to home position...")
            self.leg.move_to_home_position(duration=2.0)
            time.sleep(2.5)
            
            # Disable all motors
            self.leg.disable_all_motors()
        
        if self.can_interface:
            self.can_interface.disconnect()
        
        logger.info("Cleanup completed")

def main():
    """Main function"""
    # Parse command line arguments
    leg_name = 'leg1'  # Default to leg1
    
    if len(sys.argv) > 1:
        leg_name = sys.argv[1].lower()
        if leg_name not in MOTOR_IDS:
            logger.error(f"Invalid leg name: {leg_name}. "
                        f"Must be one of {list(MOTOR_IDS.keys())}")
            return False
    
    logger.info(f"Testing leg: {leg_name}")
    logger.info(f"Motor IDs: hip=0x{MOTOR_IDS[leg_name][0]:02X}, "
               f"knee=0x{MOTOR_IDS[leg_name][1]:02X}, "
               f"ankle=0x{MOTOR_IDS[leg_name][2]:02X}")
    
    # Run test
    test = SingleLegTest(leg_name)
    success = test.run_all_tests()
    
    if success:
        logger.info("✅ Single leg test PASSED!")
        return True
    else:
        logger.error("❌ Single leg test FAILED!")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)