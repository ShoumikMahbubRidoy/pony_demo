#!/usr/bin/env python3
"""
Test script for single AK60-6 motor
Tests basic motor functionality: enable, position control, feedback
Run this first to verify motor communication before testing full leg
"""

import time
import logging
import sys
import signal
import math

# Add the package to path (adjust if needed)
sys.path.append('/home/sivax/pony_quadruped/src/pony_control')

from pony_control.motor_controller import MotorController
from pony_control.utils.can_utils import CANInterface
from pony_control.utils.constants import *

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SingleMotorTest:
    """Test class for single motor functionality"""
    
    def __init__(self, motor_id: int = 0x40):  # Default to first hip motor
        """
        Initialize test with specified motor
        
        Args:
            motor_id: Motor CAN ID to test (default: 0x40 = leg1 hip)
        """
        self.motor_id = motor_id
        self.can_interface = None
        self.motor = None
        self.running = True
        
        # Determine joint type for safety limits
        self.joint_type = self._get_joint_type(motor_id)
        
        # Setup signal handler for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _get_joint_type(self, motor_id: int) -> str:
        """Determine joint type from motor ID"""
        for leg_name, motor_ids in MOTOR_IDS.items():
            if motor_id in motor_ids:
                joint_index = motor_ids.index(motor_id)
                return JOINT_NAMES[joint_index]
        return "unknown"
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        logger.info("Received shutdown signal, stopping motor...")
        self.running = False
    
    def setup(self) -> bool:
        """
        Initialize CAN interface and motor controller
        
        Returns:
            True if setup successful
        """
        logger.info(f"Setting up test for motor 0x{self.motor_id:02X} ({self.joint_type})")
        
        try:
            # Initialize CAN interface
            self.can_interface = CANInterface(CAN_INTERFACE, CAN_BITRATE)
            if not self.can_interface.connect():
                logger.error("Failed to connect to CAN interface")
                return False
            
            # Initialize motor controller
            self.motor = MotorController(
                self.motor_id, 
                self.can_interface, 
                f"test_{self.joint_type}"
            )
            
            # Set appropriate limits for joint type
            self.motor.set_limits(self.joint_type)
            
            logger.info("Setup completed successfully")
            return True
            
        except Exception as e:
            logger.error(f"Setup failed: {e}")
            return False
    
    def test_communication(self) -> bool:
        """
        Test basic communication with motor
        
        Returns:
            True if communication successful
        """
        logger.info("Testing motor communication...")
        
        # Try to enable motor
        if not self.motor.enable_motor():
            logger.error("Failed to enable motor")
            return False
        
        time.sleep(0.5)  # Wait for motor to respond
        
        # Send a zero position command and check for feedback
        if not self.motor.set_position(0.0):
            logger.error("Failed to send position command")
            return False
        
        # Listen for feedback for a few seconds
        start_time = time.time()
        feedback_received = False
        
        while time.time() - start_time < 3.0:
            msg = self.can_interface.receive_message(timeout=0.1)
            if msg and msg.arbitration_id == self.motor_id:
                if self.motor.update_state_from_feedback(msg):
                    feedback_received = True
                    logger.info(f"Received feedback: pos={self.motor.state.position:.3f}, "
                              f"vel={self.motor.state.velocity:.3f}, "
                              f"torque={self.motor.state.torque:.3f}")
                    break
        
        if not feedback_received:
            logger.error("No feedback received from motor")
            return False
        
        logger.info("Communication test passed!")
        return True
    
    def test_position_control(self) -> bool:
        """
        Test position control with safe movements
        
        Returns:
            True if test successful
        """
        logger.info("Testing position control...")
        
        # Define test positions (small movements for safety)
        test_positions = [0.0, 0.2, -0.2, 0.1, 0.0]  # radians
        
        for target_pos in test_positions:
            if not self.running:
                break
                
            logger.info(f"Moving to position {target_pos:.3f} rad ({math.degrees(target_pos):.1f}°)")
            
            # Send position command
            if not self.motor.set_position(target_pos):
                logger.error(f"Failed to send position command: {target_pos}")
                return False
            
            # Monitor movement for 2 seconds
            start_time = time.time()
            while time.time() - start_time < 2.0:
                if not self.running:
                    break
                    
                # Receive feedback
                msg = self.can_interface.receive_message(timeout=0.1)
                if msg and msg.arbitration_id == self.motor_id:
                    self.motor.update_state_from_feedback(msg)
                    current_pos = self.motor.state.position
                    
                    # Check if close to target
                    error = abs(current_pos - target_pos)
                    if error < 0.05:  # 0.05 rad ≈ 3 degrees tolerance
                        logger.info(f"Reached target! Current pos: {current_pos:.3f}")
                        break
                
                time.sleep(0.02)  # 50 Hz update rate
            
            time.sleep(0.5)  # Pause between movements
        
        logger.info("Position control test completed!")
        return True
    
    def test_velocity_control(self) -> bool:
        """
        Test velocity control
        
        Returns:
            True if test successful
        """
        logger.info("Testing velocity control...")
        
        # Test different velocities (small values for safety)
        test_velocities = [0.5, -0.5, 1.0, -1.0, 0.0]  # rad/s
        
        for target_vel in test_velocities:
            if not self.running:
                break
                
            logger.info(f"Setting velocity to {target_vel:.3f} rad/s")
            
            # Send velocity command
            if not self.motor.set_velocity(target_vel):
                logger.error(f"Failed to send velocity command: {target_vel}")
                return False
            
            # Monitor for 1 second
            start_time = time.time()
            while time.time() - start_time < 1.0:
                if not self.running:
                    break
                    
                msg = self.can_interface.receive_message(timeout=0.1)
                if msg and msg.arbitration_id == self.motor_id:
                    self.motor.update_state_from_feedback(msg)
                    logger.info(f"Current velocity: {self.motor.state.velocity:.3f} rad/s")
                
                time.sleep(0.1)
        
        # Stop motor
        self.motor.set_velocity(0.0)
        logger.info("Velocity control test completed!")
        return True
    
    def monitor_motor(self, duration: float = 10.0):
        """
        Monitor motor state for specified duration
        
        Args:
            duration: Monitoring duration in seconds
        """
        logger.info(f"Monitoring motor for {duration} seconds...")
        
        start_time = time.time()
        while time.time() - start_time < duration and self.running:
            msg = self.can_interface.receive_message(timeout=0.1)
            if msg and msg.arbitration_id == self.motor_id:
                self.motor.update_state_from_feedback(msg)
                state = self.motor.get_state()
                
                logger.info(f"Motor state - Pos: {state.position:.3f} rad "
                          f"({math.degrees(state.position):.1f}°), "
                          f"Vel: {state.velocity:.3f} rad/s, "
                          f"Torque: {state.torque:.3f} Nm")
            
            time.sleep(0.2)  # 5 Hz monitoring
    
    def run_all_tests(self) -> bool:
        """
        Run complete test suite
        
        Returns:
            True if all tests pass
        """
        logger.info("Starting complete motor test suite...")
        
        if not self.setup():
            return False
        
        try:
            # Test 1: Basic communication
            if not self.test_communication():
                logger.error("Communication test failed!")
                return False
            
            # Test 2: Position control  
            if not self.test_position_control():
                logger.error("Position control test failed!")
                return False
            
            # Test 3: Velocity control
            if not self.test_velocity_control():
                logger.error("Velocity control test failed!")
                return False
            
            # Test 4: Monitor motor state
            self.monitor_motor(5.0)
            
            logger.info("All tests completed successfully!")
            return True
            
        except Exception as e:
            logger.error(f"Test failed with exception: {e}")
            return False
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean shutdown"""
        logger.info("Cleaning up...")
        
        if self.motor:
            # Disable motor safely
            self.motor.set_position(0.0)  # Return to zero
            time.sleep(1.0)
            self.motor.disable_motor()
        
        if self.can_interface:
            self.can_interface.disconnect()
        
        logger.info("Cleanup completed")

def main():
    """Main function"""
    # Parse command line arguments
    motor_id = 0x40  # Default to leg1 hip
    
    if len(sys.argv) > 1:
        try:
            motor_id = int(sys.argv[1], 16)  # Parse as hex
        except ValueError:
            try:
                motor_id = int(sys.argv[1])  # Parse as decimal
            except ValueError:
                logger.error("Invalid motor ID format. Use hex (0x40) or decimal (64)")
                return False
    
    logger.info(f"Testing motor ID: 0x{motor_id:02X} ({motor_id})")
    
    # Run test
    test = SingleMotorTest(motor_id)
    success = test.run_all_tests()
    
    if success:
        logger.info("✅ Single motor test PASSED!")
        return True
    else:
        logger.error("❌ Single motor test FAILED!")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)