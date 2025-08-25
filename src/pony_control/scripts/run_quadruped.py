#!/usr/bin/env python3
"""
Main script to run Pony quadruped robot with keyboard control
This is the final script to run after testing individual motors and legs
"""

import sys
import time
import logging
import signal
import argparse

# Add the package to path if needed
sys.path.append('/home/pi/pony_quadruped/src/pony_control')

from pony_control.quadruped_controller import QuadrupedController
from pony_control.keyboard_teleop import KeyboardTeleop
from pony_control.can_utils import CANInterface
from pony_control.constants import *

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class PonyQuadruped:
    """
    Main application class for Pony quadruped robot
    Integrates all components and provides clean startup/shutdown
    """
    
    def __init__(self, can_interface: str = CAN_INTERFACE, can_bitrate: int = CAN_BITRATE):
        """
        Initialize Pony quadruped application
        
        Args:
            can_interface: CAN interface name (default from constants)
            can_bitrate: CAN bitrate (default from constants)
        """
        self.can_interface = None
        self.robot = None
        self.teleop = None
        self.running = True
        
        # CAN settings
        self.interface_name = can_interface
        self.bitrate = can_bitrate
        
        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        logger.info(f"Pony Quadruped initialized with CAN interface: {can_interface}")
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        logger.info(f"Received signal {signum}, initiating shutdown...")
        self.running = False
    
    def startup_sequence(self) -> bool:
        """
        Execute complete robot startup sequence
        
        Returns:
            True if startup successful
        """
        logger.info("=== PONY QUADRUPED ROBOT STARTUP ===")
        
        try:
            # Step 1: Initialize CAN interface
            logger.info("Step 1: Initializing CAN interface...")
            self.can_interface = CANInterface(self.interface_name, self.bitrate)
            if not self.can_interface.connect():
                logger.error("Failed to connect to CAN interface")
                return False
            logger.info("✓ CAN interface connected successfully")
            
            # Step 2: Initialize robot controller
            logger.info("Step 2: Initializing robot controller...")
            self.robot = QuadrupedController(self.can_interface)
            logger.info("✓ Robot controller initialized")
            
            # Step 3: Initialize keyboard teleop
            logger.info("Step 3: Initializing keyboard control...")
            self.teleop = KeyboardTeleop(self.robot)
            logger.info("✓ Keyboard teleop initialized")
            
            # Step 4: Start robot control system
            logger.info("Step 4: Starting robot control system...")
            if not self.robot.start():
                logger.error("Failed to start robot control system")
                return False
            logger.info("✓ Robot control system started")
            
            # Step 5: Wait for system stabilization
            logger.info("Step 5: Waiting for system stabilization...")
            time.sleep(2.0)
            logger.info("✓ System stabilized")
            
            # Step 6: Move to initial standing position
            logger.info("Step 6: Moving to initial standing position...")
            if not self.robot.stand_up(duration=3.0):
                logger.error("Failed to stand up")
                return False
            time.sleep(3.5)  # Wait for movement to complete
            logger.info("✓ Robot standing and ready")
            
            logger.info("=== STARTUP COMPLETE ===")
            return True
            
        except Exception as e:
            logger.error(f"Startup failed: {e}")
            return False
    
    def shutdown_sequence(self):
        """Execute safe robot shutdown sequence"""
        logger.info("=== PONY QUADRUPED ROBOT SHUTDOWN ===")
        
        try:
            # Stop keyboard control first
            if self.teleop:
                logger.info("Stopping keyboard control...")
                self.teleop.stop_control_loop()
            
            # Move robot to safe position
            if self.robot:
                logger.info("Moving to safe position...")
                self.robot.crouch(duration=2.0)
                time.sleep(2.5)
                
                logger.info("Stopping robot control system...")
                self.robot.stop()
            
            # Disconnect CAN interface
            if self.can_interface:
                logger.info("Disconnecting CAN interface...")
                self.can_interface.disconnect()
            
            logger.info("=== SHUTDOWN COMPLETE ===")
            
        except Exception as e:
            logger.error(f"Error during shutdown: {e}")
    
    def run_keyboard_control(self):
        """
        Run main keyboard control interface
        This is the main user interaction loop
        """
        if not self.teleop:
            logger.error("Keyboard teleop not initialized")
            return False
        
        try:
            logger.info("Starting keyboard control interface...")
            print("\n" + "="*60)
            print("🤖 PONY QUADRUPED ROBOT - KEYBOARD CONTROL READY")
            print("="*60)
            
            # Start keyboard control
            self.teleop.run()
            
            return True
            
        except Exception as e:
            logger.error(f"Keyboard control error: {e}")
            return False
    
    def run(self) -> bool:
        """
        Main application run method
        
        Returns:
            True if successful
        """
        try:
            # Execute startup sequence
            if not self.startup_sequence():
                logger.error("Startup sequence failed!")
                return False
            
            # Run keyboard control interface
            success = self.run_keyboard_control()
            
            return success
            
        except KeyboardInterrupt:
            logger.info("Received keyboard interrupt")
            return True
            
        except Exception as e:
            logger.error(f"Application error: {e}")
            return False
            
        finally:
            # Always execute shutdown sequence
            self.shutdown_sequence()

def main():
    """Main function with command line argument parsing"""
    parser = argparse.ArgumentParser(
        description="Pony Quadruped Robot Control System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Run with default settings (can0, 1Mbps)
    python3 run_quadruped.py
    
    # Use different CAN interface
    python3 run_quadruped.py --interface can1
    
    # Use different bitrate
    python3 run_quadruped.py --bitrate 500000
    
    # Debug mode with verbose logging
    python3 run_quadruped.py --debug
        """
    )
    
    parser.add_argument(
        '--interface', '-i',
        default=CAN_INTERFACE,
        help=f'CAN interface name (default: {CAN_INTERFACE})'
    )
    
    parser.add_argument(
        '--bitrate', '-b',
        type=int,
        default=CAN_BITRATE,
        help=f'CAN bitrate in Hz (default: {CAN_BITRATE})'
    )
    
    parser.add_argument(
        '--debug', '-d',
        action='store_true',
        help='Enable debug logging'
    )
    
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Test mode - initialize but do not enable motors'
    )
    
    args = parser.parse_args()
    
    # Configure logging level
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.info("Debug logging enabled")
    
    # Display startup information
    print("\n" + "="*70)
    print("🐴 PONY QUADRUPED ROBOT CONTROL SYSTEM 🤖")
    print("="*70)
    print(f"CAN Interface: {args.interface}")
    print(f"CAN Bitrate:   {args.bitrate} Hz ({args.bitrate/1000000:.1f} Mbps)")
    print(f"Motors:        {len(ALL_MOTOR_IDS)} AK60-6 V3.0 motors")
    print(f"Debug Mode:    {'ON' if args.debug else 'OFF'}")
    print(f"Dry Run:       {'ON' if args.dry_run else 'OFF'}")
    print("="*70)
    
    if args.dry_run:
        print("\n⚠️  DRY RUN MODE - Motors will NOT be enabled!")
        print("This mode is for testing communication only.")
    
    print("\n⚠️  SAFETY REMINDER:")
    print("• Ensure robot is in a safe, open area")
    print("• Keep emergency stop (ENTER key) accessible")
    print("• Be ready to disconnect power if needed")
    print("• Start with small, slow movements")
    
    input("\nPress Enter to continue or Ctrl+C to abort...")
    
    try:
        # Create and run application
        app = PonyQuadruped(args.interface, args.bitrate)
        
        if args.dry_run:
            logger.info("DRY RUN MODE - Exiting after initialization test")
            return True
        
        success = app.run()
        
        if success:
            logger.info("✅ Pony quadruped session completed successfully")
            print("\n✅ Robot control session completed successfully!")
            return True
        else:
            logger.error("❌ Pony quadruped session failed")
            print("\n❌ Robot control session failed!")
            return False
            
    except KeyboardInterrupt:
        print("\n\nSession interrupted by user")
        logger.info("Session interrupted by user")
        return True
        
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        print(f"\n❌ Unexpected error: {e}")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)