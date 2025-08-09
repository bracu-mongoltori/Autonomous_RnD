# Drone Control Package

A ROS2 package for controlling a drone using pymavlink. This package allows you to arm/disarm the drone, control motor speeds for testing purposes, and provides **manual joystick control** using an Xbox controller.

## Features

- **Arm/Disarm**: Control drone arming state via `/arm_disarm` topic
- **Motor Control**: Start/stop motors at configurable speed via `/takeoff_land` topic
- **Manual Joystick Control**: Full manual control using Xbox controller
- **Status Monitoring**: Real-time status updates via `/drone_status` topic
- **Test Client**: Interactive command-line interface for testing

## Topics

### Subscribed Topics
- `/arm_disarm` (std_msgs/Bool): 
  - `true`: Arm the drone
  - `false`: Disarm the drone

- `/takeoff_land` (std_msgs/String):
  - `"takeoff"`: Start motors at configured speed
  - `"land"`: Stop motors

- `/cmd_vel` (geometry_msgs/Twist): Manual control input from joystick
  - `linear.x`: Pitch (forward/backward)
  - `linear.y`: Roll (left/right) 
  - `linear.z`: Throttle (up/down)
  - `angular.z`: Yaw (rotation)

- `/manual_control_enable` (std_msgs/Bool): Enable/disable manual control mode

### Published Topics
- `/drone_status` (std_msgs/String): Current drone status

## Xbox Controller Mapping

### Sticks:
- **Left Stick X**: Roll (left/right movement)
- **Left Stick Y**: Pitch (forward/backward movement)
- **Right Stick X**: Yaw (rotation left/right)
- **Right Stick Y**: Throttle (up/down movement)

### Buttons:
- **A Button**: Enable/Disable manual control mode
- **B Button**: Emergency stop (immediately disable manual control)
- **Start/Back**: Reserved for future features

## Installation

1. Install dependencies:
```bash
pip install pymavlink
```

2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select drone_control
source install/setup.bash
```

## Usage

### Method 1: Using Launch File
```bash
ros2 launch drone_control drone_control.launch.py
```

### Method 2: Running Nodes Separately

1. Start the drone controller:
```bash
ros2 run drone_control drone_controller
```

2. In another terminal, start the test client:
```bash
ros2 run drone_control drone_test_client
```

### Method 3: Using ROS2 Command Line

1. Start the drone controller:
```bash
ros2 run drone_control drone_controller
```

2. In separate terminals, send commands:
```bash
# Arm the drone
ros2 topic pub /arm_disarm std_msgs/Bool "data: true"

# Start motors (takeoff)
ros2 topic pub /takeoff_land std_msgs/String "data: 'takeoff'"

# Stop motors (land)
ros2 topic pub /takeoff_land std_msgs/String "data: 'land'"

# Disarm the drone
ros2 topic pub /arm_disarm std_msgs/Bool "data: false"

# Monitor status
ros2 topic echo /drone_status
```

## Test Client Commands

When using the test client, you can use these commands:
- `arm`: Arm the drone
- `disarm`: Disarm the drone
- `takeoff`: Start motors at 5% speed
- `land`: Stop motors
- `status`: Show current status
- `test`: Run automatic test sequence
- `quit`: Exit the client

## Configuration

The drone controller accepts these parameters:
- `connection_string`: MAVLink connection string (default: `/dev/ttyUSB0`)
- `baud_rate`: Baud rate for serial connection (default: 57600)

Example with different connection:
```bash
ros2 run drone_control drone_controller --ros-args -p connection_string:="udp:127.0.0.1:14550"
```

## Safety Notes

⚠️ **IMPORTANT SAFETY WARNINGS:**
1. **Remove propellers** before testing - this is for motor testing only
2. **Always have a kill switch** ready
3. **Test in a safe, enclosed environment**
4. **Never leave the drone unattended** while armed
5. **Ensure proper power supply** for your drone

## Troubleshooting

1. **Connection Issues**: 
   - Check your connection string (USB port, UDP address, etc.)
   - Verify baud rate matches your flight controller
   - Ensure flight controller is powered and connected

2. **Permission Issues**:
   - Add user to dialout group: `sudo usermod -a -G dialout $USER`
   - Logout and login again

3. **No Heartbeat**:
   - Check flight controller is running
   - Verify MAVLink is enabled on flight controller
   - Check connection cables

## Flight Controller Setup

Make sure your flight controller (PX4, ArduPilot, etc.) has:
1. MAVLink enabled on the appropriate port
2. Correct baud rate configured
3. Manual control enabled (if required)
4. Proper safety switches configured
