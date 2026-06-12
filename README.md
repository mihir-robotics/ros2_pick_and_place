# ARM Robot - ROS2 Package

A ROS2 package for controlling an Arduino robot arm using Aruco marker detection. This package provides autonomous object pick and place operations triggered by Aruco marker detection from a USB camera.

## Features

- **Camera Node**: Captures video from USB camera and publishes frames
- **Detector Node**: Detects Aruco markers in video stream
- **Manipulator Node**: Controls robot arm via serial port based on detected markers
- **Parameterized Configuration**: All settings configurable via YAML files
- **Serial Communication**: Direct serial port communication with Arduino
- **Safe Operation**: Prevents concurrent pick and place operations

## System Architecture

```
┌─────────────────┐
│   USB Camera    │
└────────┬────────┘
         │ (USB)
         ▼
┌─────────────────┐      /camera/image       ┌──────────────────┐
│  Camera Node    ├─────────────────────────►│  Detector Node   │
└─────────────────┘                          └────────┬─────────┘
                                                      │ /detector/aruco_id
                                                      ▼
                                            ┌──────────────────┐
                                            │ Manipulator Node │
                                            └────────┬─────────┘
                                                     │ (Serial)
                                                     ▼
                                            ┌─────────────────┐
                                            │     Arduino     │
                                            │   Robot Arm     │
                                            └─────────────────┘
```

## Dependencies

- ROS2 (Humble or later)
- OpenCV 4.x
- cv_bridge
- image_transport
- Aruco module (opencv-contrib-python)

### Install Dependencies (Ubuntu)

```bash
sudo apt-get install -y \
  libopencv-dev \
  python3-opencv \
  ros-humble-cv-bridge \
  ros-humble-image-transport
```

## Package Structure

```
pick_n_place_bot/
├── src/
│   ├── camera_node.cpp           # Camera capture and publishing
│   ├── detector_node.cpp         # Aruco marker detection
│   ├── manipulator_node.cpp      # Robot arm control
│   └── serial_port.cpp           # Serial communication utility
├── include/pick_n_place_bot/
│   ├── camera_node.hpp
│   ├── detector_node.hpp
│   ├── manipulator_node.hpp
│   └── serial_port.hpp
├── config/
│   └── params.yaml               # Parameter configuration
├── launch/
│   └── robot_arm.launch.py       # Launch file for all nodes
├── CMakeLists.txt
└── package.xml
```

## Topics

### Published Topics

- `/camera/image` (sensor_msgs/Image)
  - Camera frames from USB camera
  - Published by: Camera Node

- `/detector/aruco_id` (std_msgs/String)
  - Detected Aruco marker ID
  - Published by: Detector Node
  - Format: "123" (marker ID as string)

### Subscribed Topics

- `/camera/image` (sensor_msgs/Image)
  - Subscribed by: Detector Node

- `/detector/aruco_id` (std_msgs/String)
  - Subscribed by: Manipulator Node

## Parameters

All parameters are defined in `config/params.yaml`. You can modify them as needed:

### Camera Node Parameters

```yaml
camera_node:
  camera_device_id: 0        # USB camera device ID
  frame_width: 640           # Frame width (pixels)
  frame_height: 480          # Frame height (pixels)
  frame_rate: 30             # Frames per second
```

### Detector Node Parameters

```yaml
detector_node:
  aruco_dictionary_id: 0     # Aruco dictionary (0=DICT_4X4_50)
  marker_size: 0.05          # Marker size (meters)
```

### Manipulator Node Parameters

```yaml
manipulator_node:
  serial_port: "/dev/ttyUSB0"      # Serial port path
  serial_baudrate: 9600            # Baud rate
  command_delay_ms: 3000           # Delay between commands (3 seconds)
  pick_commands:                   # Pick sequence
    - "HOME"
    - "MOVE_TO_TARGET"
    - "CLOSE_GRIPPER"
  place_commands:                  # Place sequence
    - "MOVE_TO_PLACE"
    - "OPEN_GRIPPER"
    - "HOME"
```

## Building

```bash
# Navigate to workspace
cd ~/ros2_arm_ws

# Build the package
colcon build --packages-select pick_n_place_bot

# Source setup
source install/setup.bash
```

## Running

### Option 1: Using Launch File (Recommended)

```bash
# Launch all nodes
ros2 launch pick_n_place_bot robot_arm.launch.py
```

### Option 2: Running Individual Nodes

```bash
# Terminal 1: Camera Node
ros2 run pick_n_place_bot camera_node

# Terminal 2: Detector Node
ros2 run pick_n_place_bot detector_node

# Terminal 3: Manipulator Node
ros2 run pick_n_place_bot manipulator_node
```

## Serial Communication Protocol

Commands are sent to Arduino via serial port in the following format:

```
COMMAND\n
```

### Predefined Commands

The following commands are examples. Customize them for your Arduino firmware:

- `HOME` - Move arm to home position
- `MOVE_TO_TARGET` - Move arm to detected object location
- `CLOSE_GRIPPER` - Close gripper
- `OPEN_GRIPPER` - Open gripper
- `MOVE_TO_PLACE` - Move to place location

**Note**: You must implement corresponding handlers in your Arduino firmware.

## Manipulator Node - Pick and Place Logic

The manipulator node implements the following workflow:

1. **Wait for Aruco Detection**: Listens on `/detector/aruco_id` topic
2. **Acquire Lock**: When marker detected and arm not busy, acquire state lock
3. **Execute Pick Sequence**:
   - HOME → Move arm to home position
   - MOVE_TO_TARGET → Move to detected object
   - CLOSE_GRIPPER → Grasp object
   - Wait 3s between each command
4. **Execute Place Sequence**:
   - MOVE_TO_PLACE → Move to placement location
   - OPEN_GRIPPER → Release object
   - HOME → Return to home
   - Wait 3s between each command
5. **Release Lock**: Allow next Aruco detection

**Key Features**:
- Thread-safe operation with mutex lock
- Prevents concurrent operations
- Ignores new Aruco detections during active cycle
- 3-second delay between commands (configurable)

## Troubleshooting

### Camera Not Opening

```bash
# Check available cameras
ls -l /dev/video*

# Test with ffplay
ffplay /dev/video0
```

### Aruco Marker Not Detected

1. Ensure adequate lighting
2. Check marker is within camera view
3. Verify `aruco_dictionary_id` matches your marker dictionary
4. Monitor with:
   ```bash
   ros2 topic echo /detector/aruco_id
   ```

### Serial Port Not Opening

```bash
# Check serial ports
ls -l /dev/ttyUSB* /dev/ttyACM*

# Test serial connection
minicom -D /dev/ttyUSB0 -b 9600
```

### Node Won't Start

```bash
# Build with verbose output
colcon build --packages-select pick_n_place_bot --event-handlers console_direct+

# Run with debug logging
ROS_LOG_LEVEL=DEBUG ros2 run pick_n_place_bot camera_node
```

## Advanced Configuration

### Changing Aruco Dictionary

Available dictionaries in OpenCV (cv::aruco namespace):

| ID  | Name                | Size | # Markers |
|-----|-------------------|------|-----------|
| 0   | DICT_4X4_50       | 4x4  | 50        |
| 1   | DICT_5X5_100      | 5x5  | 100       |
| 2   | DICT_6X6_250      | 6x6  | 250       |
| 3   | DICT_7X7_1000     | 7x7  | 1000      |

Change in `config/params.yaml`:

```yaml
detector_node:
  aruco_dictionary_id: 1  # Use DICT_5X5_100
```

### Customizing Commands

Edit `config/params.yaml` to add your custom commands:

```yaml
manipulator_node:
  pick_commands:
    - "HOME"
    - "MOVE_SLOW"
    - "MOVE_TO_TARGET"
    - "APPROACH_OBJECT"
    - "CLOSE_GRIPPER"
  place_commands:
    - "RETRACT"
    - "MOVE_TO_PLACE"
    - "LOWER"
    - "OPEN_GRIPPER"
    - "RETRACT_AGAIN"
    - "HOME"
  command_delay_ms: 4000  # 4 second delay
```

## Arduino Example Sketch

Here's a minimal example Arduino sketch to receive commands:

```cpp
#define BAUD_RATE 9600

void setup() {
  Serial.begin(BAUD_RATE);
  // Initialize servo pins, etc.
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "HOME") {
      moveArmHome();
    } else if (command == "MOVE_TO_TARGET") {
      moveToTarget();
    } else if (command == "CLOSE_GRIPPER") {
      closeGripper();
    } else if (command == "OPEN_GRIPPER") {
      openGripper();
    } else if (command == "MOVE_TO_PLACE") {
      moveToPlace();
    }
  }
}

void moveArmHome() { /* Implementation */ }
void moveToTarget() { /* Implementation */ }
void closeGripper() { /* Implementation */ }
void openGripper() { /* Implementation */ }
void moveToPlace() { /* Implementation */ }
```

## Testing Without Hardware

You can test the package without actual hardware:

1. **Without Arduino**: Manipulator node runs in simulation mode if serial port fails
2. **Without Camera**: Use a video file as camera source (modify camera_node.cpp)
3. **Without Aruco Marker**: Publish test messages:
   ```bash
   ros2 topic pub /detector/aruco_id std_msgs/msg/String '{data: "42"}'
   ```

## Performance Considerations

- Camera capture: ~30 FPS (configurable)
- Aruco detection: ~50-100 ms per frame
- Serial communication: Non-blocking, 3-second command spacing
- CPU usage: Minimal (< 5% on average hardware)

## Safety Notes

1. **Always keep hands clear** when testing arm movements
2. **Test pick_n_place** with empty gripper first
3. **Verify serial communication** before running
4. **Use emergency stop** at Arduino level for safety

## License

This package is provided as-is for educational and robotics projects.

## Author Notes

- Customize pick_commands and place_commands for your specific arm kinematics
- Adjust command_delay_ms based on your motor speeds
- Test serial communication independently before full integration
- Monitor `/camera/image` with `rqt_image_view` for debugging


