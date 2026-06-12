# ARM Robot ROS2 Package - Quick Start Guide

## Prerequisites

- ROS2 Humble or later installed
- Ubuntu 22.04 LTS (or compatible)
- USB camera
- Arduino Mega/Uno (or compatible board)
- Servo motors (4+ joints recommended)
- USB-to-Serial cable for Arduino connection

## Step 1: Install System Dependencies

```bash
# Update package list
sudo apt-get update

# Install ROS2 build tools and dependencies
sudo apt-get install -y \
  build-essential \
  cmake \
  libopencv-dev \
  python3-opencv \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-launch-ros
```

## Step 2: Verify Workspace Setup

```bash
# Navigate to workspace
cd ~/ros2_arm_ws

# Verify src directory exists
ls -la src/
```

## Step 3: Build the Package

```bash
# From workspace root
cd ~/ros2_arm_ws

# Build the pick_n_place_bot package
colcon build --packages-select pick_n_place_bot --symlink-install

# If you need to rebuild from scratch
colcon build --packages-select pick_n_place_bot --cmake-clean-cache
```

## Step 4: Source the Setup

```bash
# Source the workspace
source install/setup.bash

# Verify installation
ros2 pkg find pick_n_place_bot
```

## Step 5: Prepare Hardware

### Connect USB Camera

```bash
# Check if camera is detected
ls -l /dev/video*

# Test camera with OpenCV
python3 -c "import cv2; cap=cv2.VideoCapture(0); print(cap.isOpened())"
```

### Connect Arduino

1. Upload the provided `arduino.ino` to your Arduino board
2. Connect USB cable to your computer
3. Check serial port:

```bash
# List serial ports
ls -l /dev/ttyUSB* /dev/ttyACM*

# Test serial connection (Ctrl+A, then X to exit)
picocom -b 9600 /dev/ttyUSB0
```

4. Update serial port in `config/params.yaml` if different

## Step 6: Configure Parameters

Edit `src/pick_n_place_bot/config/params.yaml`:

```yaml
# For your specific camera
camera_node:
  camera_device_id: 0     # Change if using different camera
  frame_width: 640
  frame_height: 480
  frame_rate: 30

# For your Aruco markers
detector_node:
  aruco_dictionary_id: 0  # Match your marker dictionary
  marker_size: 0.05       # Meters

# For your Arduino and arm
manipulator_node:
  serial_port: "/dev/ttyUSB0"    # Update if different
  serial_baudrate: 9600
  command_delay_ms: 3000         # Milliseconds between commands
  
  pick_commands:
    - "HOME"
    - "MOVE_TO_TARGET"
    - "CLOSE_GRIPPER"
  
  place_commands:
    - "MOVE_TO_PLACE"
    - "OPEN_GRIPPER"
    - "HOME"
```

## Step 7: Test Individual Nodes

### Test Camera Node

```bash
# Terminal 1: Run camera node
ros2 run pick_n_place_bot camera_node

# Terminal 2: View camera feed
ros2 run image_view image_view --ros-args -r image:=/camera/image
```

### Test Detector Node

```bash
# Terminal 1: Run camera node
ros2 run pick_n_place_bot camera_node

# Terminal 2: Run detector node
ros2 run pick_n_place_bot detector_node

# Terminal 3: Monitor Aruco detections
ros2 topic echo /detector/aruco_id
```

### Test Manipulator Node

```bash
# Terminal 1: Run manipulator node (without hardware)
ros2 run pick_n_place_bot manipulator_node

# Terminal 2: Publish test Aruco ID
ros2 topic pub /detector/aruco_id std_msgs/msg/String '{data: "42"}'

# Terminal 3: Monitor commands being sent
ros2 topic list
```

## Step 8: Run Full System

```bash
# Launch all nodes together
ros2 launch pick_n_place_bot robot_arm.launch.py
```

View logs:

```bash
# New terminal: View all node logs
ros2 launch pick_n_place_bot robot_arm.launch.py 2>&1 | tee pick_n_place_bot.log

# View specific node
ros2 node list
ros2 node info /manipulator_node
```

## Step 9: Troubleshooting

### Build Errors

```bash
# Clean build
cd ~/ros2_arm_ws
rm -rf build install log
colcon build --packages-select pick_n_place_bot

# Build with verbose output
colcon build --packages-select pick_n_place_bot --event-handlers console_direct+
```

### Camera Not Found

```bash
# List video devices
v4l2-ctl --list-devices

# Test with specific device
ros2 run pick_n_place_bot camera_node --ros-args -p camera_device_id:=1
```

### Serial Port Permission Denied

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in, then reboot
```

### Aruco Not Detected

```bash
# Check camera feed first
rqt_image_view /camera/image

# Generate test Aruco markers
# Use OpenCV's Aruco generator or online tools
# Print and test with your marker
```

## Step 10: Customize for Your Arm

### Modify Pick/Place Sequence

Edit `config/params.yaml`:

```yaml
manipulator_node:
  pick_commands:
    - "HOME"
    - "MOVE_SLOWLY"      # Add intermediate steps
    - "APPROACH_OBJECT"
    - "MOVE_TO_TARGET"
    - "CLOSE_GRIPPER"
  
  place_commands:
    - "RETRACT_GRIPPER"  # Safety move
    - "MOVE_UP"
    - "MOVE_TO_PLACE"
    - "LOWER"
    - "OPEN_GRIPPER"
    - "HOME"
  
  command_delay_ms: 4000  # Increase if needed
```

### Update Arduino Sketch

Modify `arduino_sketch.ino`:

1. Update servo pin assignments (lines 9-14)
2. Adjust home positions (lines 16-23)
3. Update target positions (lines 25-32)
4. Implement movement functions according to your arm kinematics

## Performance Tips

1. **Reduce frame rate** for faster processing:
   ```yaml
   camera_node:
     frame_rate: 15  # Instead of 30
   ```

2. **Increase marker size** for better detection:
   ```yaml
   detector_node:
     marker_size: 0.10  # Larger markers
   ```

3. **Reduce command delay** for faster operations:
   ```yaml
   manipulator_node:
     command_delay_ms: 1000  # Less delay
   ```


## Common Commands Reference

```bash
# List all running nodes
ros2 node list

# List all topics
ros2 topic list

# Monitor specific topic
ros2 topic echo /detector/aruco_id

# View node info
ros2 node info /camera_node

# View parameter values
ros2 param list

# Set parameter at runtime
ros2 param set /manipulator_node command_delay_ms 2000

# Get ROS logs
ros2 bag record /camera/image /detector/aruco_id

# Kill all nodes
ros2 lifecycle list  # Check lifecycle nodes
pkill -f "ros2 run"  # Force kill all
```

## Documentation

- Main README: See [README.md](README.md)
- ROS2 Documentation: https://docs.ros.org/en/jazzy/
- OpenCV Documentation: https://docs.opencv.org/
- Aruco Documentation: https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html

---