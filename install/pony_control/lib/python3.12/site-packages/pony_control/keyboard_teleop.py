"""
Keyboard teleoperation interface for Pony quadruped robot
Provides intuitive keyboard control for robot movement and testing
"""

import sys
import termios
import tty
import select
import threading
import time
import logging
from typing import Optional
from dataclasses import dataclass

from .quadruped_controller import QuadrupedController, MovementCommand, RobotState

logger = logging.getLogger(__name__)

@dataclass
class KeyboardSettings:
    """Keyboard control sensitivity settings"""
    linear_speed: float = 0.3          # m/s - forward/backward speed
    angular_speed: float = 1.0         # rad/s - rotation speed  
    height_step: float = 0.02          # m - body height adjustment step
    speed_increment: float = 0.1       # Speed adjustment increment

class KeyboardTeleop:
    """
    Keyboard teleoperation interface for quadruped robot
    Provides real-time keyboard control with intuitive commands
    """
    
    def __init__(self, robot: QuadrupedController):
        """
        Initialize keyboard teleop interface
        
        Args:
            robot: QuadrupedController instance to control
        """
        self.robot = robot
        self.settings = KeyboardSettings()
        self.current_command = MovementCommand()
        
        # Terminal settings for raw keyboard input
        self.old_settings = None
        self.running = False
        
        # Control thread
        self.control_thread = None
        
        logger.info("Keyboard teleop interface initialized")
    
    def setup_terminal(self):
        """Setup terminal for non-blocking raw keyboard input"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
            logger.info("Terminal configured for keyboard input")
        except Exception as e:
            logger.error(f"Failed to setup terminal: {e}")
            raise
    
    def restore_terminal(self):
        """Restore original terminal settings"""
        if self.old_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
                logger.info("Terminal settings restored")
            except Exception as e:
                logger.error(f"Failed to restore terminal: {e}")
    
    def get_key(self, timeout: float = 0.1) -> Optional[str]:
        """
        Get single keypress with timeout (non-blocking)
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Pressed key as string, or None if timeout
        """
        if select.select([sys.stdin], [], [], timeout) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def print_help_screen(self):
        """Display comprehensive help information"""
        help_text = """
╔══════════════════════════════════════════════════════════════╗
║                 🐴 PONY QUADRUPED ROBOT 🤖                  ║  
║                    KEYBOARD CONTROLLER                       ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  MOVEMENT CONTROLS:                                          ║
║    W/S     - Move Forward/Backward                           ║
║    A/D     - Turn Left/Right                                 ║
║    Q/E     - Strafe Left/Right (side stepping)               ║
║                                                              ║
║  BODY CONTROLS:                                              ║
║    R/F     - Raise/Lower body height                         ║
║    T       - Return to default height                        ║
║                                                              ║
║  ROBOT STATES:                                               ║
║    SPACE   - Stand up / Initialize                           ║
║    C       - Crouch down                                     ║
║    H       - Return to home position                         ║
║    ENTER   - Emergency stop                                  ║
║                                                              ║
║  SPEED CONTROL:                                              ║
║    +/=     - Increase movement speed                         ║
║    -/_     - Decrease movement speed                         ║
║                                                              ║
║  INDIVIDUAL LEG TESTING:                                     ║
║    1       - Test Leg 1 (Front Right)                        ║
║    2       - Test Leg 2 (Front Left)                         ║
║    3       - Test Leg 3 (Rear Right)                         ║
║    4       - Test Leg 4 (Rear Left)                          ║
║                                                              ║
║  INFORMATION:                                                ║
║    I       - Show robot status information                   ║
║    ?       - Show this help screen                           ║
║                                                              ║
║  SYSTEM:                                                     ║
║    ESC/Q   - Quit program                                    ║
║                                                              ║
║  CURRENT SETTINGS:                                           ║
║    Linear Speed:  {:.2f} m/s                                 ║
║    Angular Speed: {:.2f} rad/s                               ║
║    Body Height:   {:.2f} m                                   ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
        """.format(
            self.settings.linear_speed,
            self.settings.angular_speed,
            self.current_command.body_height
        )
        print(help_text)
    
    def print_status_info(self):
        """Display current robot status"""
        status = self.robot.get_robot_status()
        
        print("\n" + "="*60)
        print(f"ROBOT STATUS - State: {status['state'].upper()}")
        print("="*60)
        
        # Current command
        cmd = status['command']
        print(f"Movement Command:")
        print(f"  Linear X: {cmd.linear_x:+.3f} m/s")
        print(f"  Linear Y: {cmd.linear_y:+.3f} m/s") 
        print(f"  Angular Z: {cmd.angular_z:+.3f} rad/s")
        print(f"  Body Height: {cmd.body_height:.3f} m")
        
        print(f"\nLeg Positions:")
        for leg_name, leg_data in status['legs'].items():
            pos = leg_data['foot_position']
            joints = leg_data['joint_positions']
            alive = "✓" if leg_data['alive'] else "✗"
            
            print(f"  {leg_name}: ({pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}) m")
            print(f"         Joints: Hip={joints[0]:+.2f} Knee={joints[1]:+.2f} Ankle={joints[2]:+.2f} rad {alive}")
        
        print(f"\nGait Cycle Time: {status['cycle_time']:.2f} s")
        print("="*60 + "\n")
    
    def start_control_loop(self):
        """Start the keyboard control loop"""
        self.running = True
        self.control_thread = threading.Thread(target=self._control_worker, daemon=True)
        self.control_thread.start()
        logger.info("Control loop started")
    
    def stop_control_loop(self):
        """Stop the keyboard control loop"""
        self.running = False
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=1.0)
        logger.info("Control loop stopped")
    
    def _control_worker(self):
        """Control loop that continuously sends commands to robot"""
        while self.running:
            try:
                # Send current command to robot
                self.robot.set_movement_command(self.current_command)
                time.sleep(0.05)  # 20 Hz update rate
            except Exception as e:
                logger.error(f"Control worker error: {e}")
                break
    
    def run(self):
        """
        Main keyboard control interface
        Handles all keyboard input and robot control
        """
        try:
            # Setup terminal
            self.setup_terminal()
            
            # Display help
            self.print_help_screen()
            print("\nPress any key to start, or '?' for help...")
            self.get_key(timeout=10.0)  # Wait for user input
            
            # Start control loop
            self.start_control_loop()
            
            print("\n🤖 Keyboard control active! Press '?' for help, ESC or 'q' to quit.\n")
            
            # Main input loop
            while self.running:
                key = self.get_key(timeout=0.05)
                
                if not key:
                    continue
                
                # Convert to lowercase for consistency
                key_lower = key.lower()
                
                # Handle keyboard input
                if key == '\x1b' or key_lower == 'q':  # ESC or 'q' to quit
                    break
                    
                elif key_lower == 'w':  # Move forward
                    self.current_command.linear_x = self.settings.linear_speed
                    print("Moving forward...")
                    
                elif key_lower == 's':  # Move backward
                    self.current_command.linear_x = -self.settings.linear_speed
                    print("Moving backward...")
                    
                elif key_lower == 'a':  # Turn left
                    self.current_command.angular_z = self.settings.angular_speed
                    print("Turning left...")
                    
                elif key_lower == 'd':  # Turn right
                    self.current_command.angular_z = -self.settings.angular_speed
                    print("Turning right...")
                    
                elif key_lower == 'q':  # Strafe left
                    self.current_command.linear_y = self.settings.linear_speed
                    print("Strafing left...")
                    
                elif key_lower == 'e':  # Strafe right
                    self.current_command.linear_y = -self.settings.linear_speed
                    print("Strafing right...")
                    
                elif key_lower == 'r':  # Raise body
                    self.current_command.body_height += self.settings.height_step
                    self.current_command.body_height = min(self.current_command.body_height, 0.35)
                    print(f"Raising body to {self.current_command.body_height:.3f} m")
                    
                elif key_lower == 'f':  # Lower body
                    self.current_command.body_height -= self.settings.height_step
                    self.current_command.body_height = max(self.current_command.body_height, 0.15)
                    print(f"Lowering body to {self.current_command.body_height:.3f} m")
                    
                elif key_lower == 't':  # Reset height
                    self.current_command.body_height = 0.25
                    print("Reset body height to default")
                    
                elif key == ' ':  # Stand up / Initialize
                    print("Standing up...")
                    self.robot.stand_up()
                    
                elif key_lower == 'c':  # Crouch
                    print("Crouching...")
                    self.robot.crouch()
                    
                elif key_lower == 'h':  # Home position
                    print("Returning to home position...")
                    self.current_command = MovementCommand()  # Reset all commands
                    self.robot.stand_up()
                    
                elif key == '\r' or key == '\n':  # Enter - Emergency stop
                    print("EMERGENCY STOP!")
                    self.current_command = MovementCommand()  # Zero all commands
                    self.robot.emergency_stop()
                    
                elif key == '+' or key == '=':  # Increase speed
                    self.settings.linear_speed += self.settings.speed_increment
                    self.settings.angular_speed += self.settings.speed_increment
                    self.settings.linear_speed = min(self.settings.linear_speed, 0.8)
                    self.settings.angular_speed = min(self.settings.angular_speed, 2.0)
                    print(f"Speed increased: linear={self.settings.linear_speed:.2f}, angular={self.settings.angular_speed:.2f}")
                    
                elif key == '-' or key == '_':  # Decrease speed
                    self.settings.linear_speed -= self.settings.speed_increment
                    self.settings.angular_speed -= self.settings.speed_increment
                    self.settings.linear_speed = max(self.settings.linear_speed, 0.1)
                    self.settings.angular_speed = max(self.settings.angular_speed, 0.2)
                    print(f"Speed decreased: linear={self.settings.linear_speed:.2f}, angular={self.settings.angular_speed:.2f}")
                    
                elif key_lower == '1':  # Test leg 1
                    print("Testing Leg 1 (Front Right)...")
                    self._test_individual_leg('leg1')
                    
                elif key_lower == '2':  # Test leg 2
                    print("Testing Leg 2 (Front Left)...")
                    self._test_individual_leg('leg2')
                    
                elif key_lower == '3':  # Test leg 3
                    print("Testing Leg 3 (Rear Right)...")
                    self._test_individual_leg('leg3')
                    
                elif key_lower == '4':  # Test leg 4
                    print("Testing Leg 4 (Rear Left)...")
                    self._test_individual_leg('leg4')
                    
                elif key_lower == 'i':  # Show info
                    self.print_status_info()
                    
                elif key == '?':  # Show help
                    self.print_help_screen()
                
                else:
                    # Key released or unknown key - stop movement
                    if key_lower in ['w', 's']:
                        self.current_command.linear_x = 0.0
                    elif key_lower in ['a', 'd']:
                        self.current_command.angular_z = 0.0
                    elif key_lower in ['q', 'e']:
                        self.current_command.linear_y = 0.0
            
            print("\nKeyboard control stopped.")
            
        except KeyboardInterrupt:
            print("\nReceived interrupt signal, stopping...")
            
        except Exception as e:
            logger.error(f"Keyboard control error: {e}")
            
        finally:
            self.stop_control_loop()
            self.restore_terminal()
    
    def _test_individual_leg(self, leg_name: str):
        """
        Test individual leg movement
        
        Args:
            leg_name: Name of leg to test
        """
        if leg_name not in self.robot.legs:
            print(f"Error: Invalid leg name {leg_name}")
            return
        
        try:
            leg = self.robot.legs[leg_name]
            print(f"Lifting {leg_name}...")
            
            # Get current position and lift leg
            current_pos = leg.get_foot_position()
            lift_pos = (current_pos[0], current_pos[1], current_pos[2] + 0.05)
            
            # Lift leg
            leg.move_to_cartesian(*lift_pos, 1.0)
            time.sleep(1.2)
            
            # Lower leg back
            leg.move_to_cartesian(*current_pos, 1.0) 
            time.sleep(1.2)
            
            print(f"{leg_name} test completed.")
            
        except Exception as e:
            logger.error(f"Error testing {leg_name}: {e}")
            print(f"Error testing {leg_name}")