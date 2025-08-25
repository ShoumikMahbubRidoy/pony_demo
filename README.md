# Pony Quadruped Robot - Complete Setup Guide

## Overview
This guide provides step-by-step instructions to set up your Pony quadruped robot with AK60-6 V3.0 motors, from basic installation to full robot control.

## Hardware Requirements
- Raspberry Pi 4B with Ubuntu 24.04 LTS
- InnoMaker USB-to-CAN adapter
- 12× AK60-6 V3.0 motors (3 per leg × 4 legs)
- Appropriate power supply for motors (24V recommended)
- CAN termination resistors (120Ω)

## Phase 1: System Installation

### Step 1: System Preparation
```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y git python3-pip can-utils vim htop

# Install ROS2 Humble (if desired)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Python Dependencies
```bash
# Install Python CAN library
pip3 install python-can>=4.0.0

# Install additional Python packages
pip3 install numpy>=1.20.0 matplotlib>=3.0.0
```

### Step 3: CAN Interface Setup
```bash
# Create udev rule for InnoMaker USB-CAN
sudo tee /etc/udev/rules.d/99-can.rules << EOF
SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", MODE="666", GROUP="dialout"
EOF

# Add user to dialout group
sudo usermod -a -G dialout $USER

# Create CAN interface service
sudo tee /etc/systemd/system/can-interface.service << EOF
[Unit]
Description=Bring up SocketCAN can0
After=network.target
ConditionPathExists=/sys/class/net/can0

[Service]
Type=oneshot
RemainAfterExit=yes
Environment=PATH=/usr/sbin:/usr/bin:/sbin:/bin
ExecStart=/usr/sbin/ip link set can0 type can bitrate 1000000
ExecStart=/usr/sbin/ip link set can0 up
ExecStop=/usr/sbin/ip link set can0 down

[Install]
WantedBy=multi-user.target

EOF

# Enable and start CAN service
sudo systemctl daemon-reload
sudo systemctl enable can-interface.service
sudo systemctl start can-interface.service
sudo systemctl status can-interface.service -l --no-pager

# Verify CAN interface
ip link show can0
# Should show: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP
```

### Step 4: Project Setup
```bash
# Create workspace
mkdir -p ~/pony_quadruped/src
cd ~/pony_quadruped

# Create package structure
mkdir -p src/pony_control/pony_control/utils
mkdir -p src/pony_control/scripts
mkdir -p src/pony_control/config
mkdir -p src/pony_control/launch
mkdir -p src/pony_control/resource

# Create resource marker file
touch src/pony_control/resource/pony_control
```

## Phase 2: Code Installation

### Step 5: Copy Core Files
Copy the provided code files to their respective locations:

```
~/pony_quadruped/src/pony_control/
├── package.xml
├── setup.py  
├── resource/pony_control
├── pony_control/
│   ├── __init__.py                 (empty file)
│   ├── constants.py               (from artifact)
│   ├── motor_controller.py        (from artifact)
│   ├── leg_controller.py          (from artifact)
│   ├── quadruped_controller.py    (from artifact)
│   ├── keyboard_teleop.py         (from artifact)
│   └── utils/
│       ├── __init__.py            (empty file)
│       ├── can_utils.py           (from artifact)
│       └── math_utils.py          (from artifact)
├── scripts/
│   ├── test_single_motor.py       (from artifact)
│   ├── test_single_leg.py         (from artifact)
│   └── run_quadruped.py           (from artifact)
└── config/
    ├── motor_params.yaml          (from artifact)
    └── robot_config.yaml          (from artifact)
```

### Step 6: Make Scripts Executable
```bash
cd ~/pony_quadruped/src/pony_control/scripts
chmod +x test_single_motor.py test_single_leg.py run_quadruped.py
```

### Step 7: Build Package (if using ROS2)
```bash
cd ~/pony_quadruped
colcon build --packages-select pony_control
source install/setup.bash
echo "source ~/pony_quadruped/install/setup.bash" >> ~/.bashrc
```

## Phase 3: Progressive Testing

### Phase 3a: Hardware Verification
```bash
# Verify CAN interface is working
candump can0 &  # Start monitoring CAN traffic
# You should see no messages initially (this is normal)

# Kill monitoring
killall candump

# Test CAN interface manually
cansend can0 040#FFFFFFFFFFFFFFFF  # Try to enable motor 0x40
# Monitor for any response (may not work yet, but interface should accept command)
```

### Phase 3b: Single Motor Test (Start Here!)
```bash
cd ~/pony_quadruped/src/pony_control/scripts

# Test leg1 hip motor (motor ID 0x40)
python3 test_single_motor.py 0x40

# Test other motors one by one
python3 test_single_motor.py 0x41  # leg1 knee
python3 test_single_motor.py 0x42  # leg1 ankle

# Test motors from other legs
python3 test_single_motor.py 0x50  # leg2 hip
# ... continue for all 12 motors
```

**Expected Output for Working Motor:**
```
INFO - Setting up test for motor 0x40 (hip)
INFO - Setup completed successfully
INFO - Testing motor communication...
INFO - Received feedback: pos=0.123, vel=0.000, torque=0.456
INFO - Communication test passed!
INFO - Testing position control...
INFO - Moving to position 0.200 rad (11.5°)
INFO - Reached target! Current pos: 0.198
✅ Single motor test PASSED!
```

**If Motor Test Fails:**
1. Check power supply to motors
2. Verify CAN wiring and termination
3. Check motor ID configuration
4. Ensure CAN interface is up: `ip link show can0`
5. Monitor CAN traffic: `candump can0`

### Phase 3c: Single Leg Test
```bash
# Only proceed after at least one full leg (3 motors) passes single motor tests

# Test leg1 (motors 0x40, 0x41, 0x42)
python3 test_single_leg.py leg1

# Test other legs
python3 test_single_leg.py leg2
python3 test_single_leg.py leg3
python3 test_single_leg.py leg4
```

**Expected Output for Working Leg:**
```
INFO - Testing leg: leg1
INFO - Motor IDs: hip=0x40, knee=0x41, ankle=0x42
INFO - Testing communication with all leg motors...
INFO - Received feedback from hip: pos=0.000, vel=0.000, torque=0.123
INFO - Received feedback from knee: pos=0.000, vel=0.000, torque=0.456
INFO - Received feedback from ankle: pos=0.000, vel=0.000, torque=0.789
INFO - Communication test passed for all motors!
INFO - Testing individual joint movements...
INFO - Testing coordinated leg movements...
INFO - Testing Cartesian space movements...
✅ Single leg test PASSED!
```

### Phase 3d: Full Robot Control
```bash
# Only proceed after multiple legs pass individual tests

# Run full quadruped control
python3 run_quadruped.py

# Or with debug output
python3 run_quadruped.py --debug

# Test mode (initializes but doesn't enable motors)
python3 run_quadruped.py --dry-run
```

## Phase 4: Robot Operation

### Keyboard Controls
Once the robot is running, use these keyboard commands:

**Basic Movement:**
- `W` / `S` - Forward / Backward  
- `A` / `D` - Turn Left / Right
- `Q` / `E` - Strafe Left / Right

**Body Control:**
- `R` / `F` - Raise / Lower body
- `SPACE` - Stand up
- `C` - Crouch down
- `H` - Home position

**Individual Leg Testing:**
- `1` / `2` / `3` / `4` - Test individual legs

**System:**
- `I` - Show robot status
- `ENTER` - Emergency stop
- `ESC` / `Q` - Quit

## Troubleshooting Guide

### Motor Communication Issues
```bash
# Check CAN interface status
ip link show can0

# Monitor CAN traffic  
candump can0

# Check for hardware issues
dmesg | grep can

# Restart CAN interface
sudo systemctl restart can-interface.service
```

### Common Problems and Solutions

**Problem: "No CAN interface found"**
- Check USB-CAN adapter connection: `lsusb | grep -i can`
- Verify driver installation
- Check udev rules: `ls -l /dev/ttyACM*` or `/dev/can*`

**Problem: "Motor not responding"**
- Verify motor power supply (24V recommended)
- Check CAN wiring (CANH, CANL, GND)
- Ensure CAN termination resistors (120Ω) at both ends
- Verify motor ID configuration matches constants.py

**Problem: "Permission denied"**
- Add user to dialout group: `sudo usermod -a -G dialout $USER`
- Log out and back in
- Check file permissions: `ls -l /dev/ttyACM*`

**Problem: "Motor movements too aggressive"**
- Reduce PID gains in motor_params.yaml
- Increase movement duration in test scripts  
- Check safety limits in constants.py

**Problem: "CAN bus errors"**
- Check CAN bitrate (must be 1 Mbps for AK60-6)
- Verify termination resistors
- Check for electrical noise
- Ensure proper grounding

## Performance Tuning

### PID Tuning Guidelines
1. **Start Conservative**: Begin with low Kp (50) and Kd (0.5)
2. **Increase Kp**: Gradually increase until response is fast enough
3. **Add Damping**: Increase Kd if oscillations occur
4. **Test Thoroughly**: Test all movements after changes

### Motor Parameters (in motor_params.yaml):
```yaml
joint_gains:
  hip:
    kp: 120.0    # Strong for weight bearing
    kd: 1.2      # Good damping
  knee:
    kp: 100.0    # Standard response
    kd: 1.0      # Moderate damping
  ankle:
    kp: 80.0     # Softer for compliance
    kd: 0.8      # Light damping
```

## Safety Checklist

**Before Every Test:**
- [ ] Robot in safe, open area (minimum 2m clearance)
- [ ] Emergency stop accessible (ENTER key)
- [ ] Power disconnect within reach
- [ ] All personnel at safe distance
- [ ] Motors properly mounted and secured
- [ ] CAN wiring checked and secure
- [ ] Motor temperatures normal (not hot)

**During Testing:**
- [ ] Start with small movements
- [ ] Monitor motor temperatures
- [ ] Listen for unusual sounds
- [ ] Watch for unstable behavior
- [ ] Keep emergency stop ready

**After Testing:**
- [ ] Power down motors properly
- [ ] Check for any loose connections
- [ ] Note any issues for next session

## Next Steps

Once basic operation is working:

1. **Tune Parameters**: Adjust PID gains, movement speeds, safety limits
2. **Add Sensors**: Integrate IMU for body orientation, force sensors for feet
3. **Improve Gaits**: Implement more sophisticated walking patterns
4. **Add Autonomy**: Implement obstacle avoidance, path planning
5. **ROS2 Integration**: Create ROS2 nodes for standard robot interfaces

## Getting Help

**Log Files**: Check `/home/pi/pony_quadruped/logs/` for detailed logs
**CAN Monitoring**: Use `candump can0` to see actual CAN traffic  
**Debug Mode**: Run scripts with `--debug` flag for verbose output
**Community**: ROS2 and python-can communities have extensive documentation

Remember: This is a powerful robot system. Always prioritize safety and start with small, controlled movements.
