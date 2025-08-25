# Quick test script - save as simple_test.py
import sys
import time
sys.path.append('/home/sivax/pony_quadruped/src/pony_control/pony_control')

from utils.can_utils import CANInterface, CANUtils

can_interface = CANInterface('can0', 1000000)
can_interface.connect()

# Send enable command
enable_msg = CANUtils.create_control_message(0x2940, 0xFC)
can_interface.send_message(enable_msg)
time.sleep(0.5)

# Send position command
pos_msg = CANUtils.encode_motor_command(0x2940, 0.0, 0.0, 0.0, 50.0, 1.0)
result = can_interface.send_message(pos_msg)
print(f"Position command sent: {result}")

can_interface.disconnect()
