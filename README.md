# ROS2 Pick & Place Robot Arm using Computer Vision [![CI](https://github.com/mihir-robotics/ros2_pick_and_place/actions/workflows/ci.yml/badge.svg)](https://github.com/mihir-robotics/ros2_pick_and_place/actions/workflows/ci.yml)

ROS 2 package that runs pick-and-place on an Arduino robot arm. A USB camera feeds frames to an ArUco detector; when a marker is seen, the manipulator node runs a two-stage pick-and-place cycle over serial at 9600 baud. The Arduino firmware supports smooth trajectory motion via `pose-*` and duration-based joint commands.


![arm 2x gif](assets/arm.gif)

>GIF is sped up to save time :)
--- 

### Logic Flow
![flowchart](assets/flowchart.svg)

## How it works

1. **camera_node** — captures frames from a V4L2 USB camera and publishes `sensor_msgs/Image` on `/camera/image`.
2. **detector_node** — detects ArUco markers in the camera stream and publishes the marker ID as `std_msgs/Int32` on `/detector/aruco_id`.
3. **manipulator_node** — on startup, moves to the pre-pick pose (first entry in `pick_commands`). When an ArUco marker is detected, a worker thread runs a two-stage cycle:
   - **Stage 1:** `pick_commands` → `place_commands_1` → return to pre-pick pose
   - **Wait:** blocks until the next ArUco sighting
   - **Stage 2:** `pick_commands` → `place_commands_2`
   
   Concurrent detections are rejected while a cycle is in progress (`is_busy_`). During the wait-for-place2 phase, a new marker sighting unblocks stage 2. The marker ID is logged but does not change which command lists run.

If the serial port cannot be opened, the manipulator node logs commands in simulation mode instead of sending them.

### Rqt Graph
![rqt](assets/rqt-graph.png)

## Package structure

```
pick_n_place_bot/
├── src/
│   ├── camera_node.cpp
│   ├── detector_node.cpp
│   └── manipulator_node.cpp
├── include/pick_n_place_bot/
│   ├── camera_node.hpp
│   ├── detector_node.hpp
│   └── manipulator_node.hpp
├── arduino/
│   └── arduino.ino              # Servo firmware (9600 baud)
├── config/
│   ├── params.yaml
│   └── params.uno_q.yaml      # Arduino UNO Q device paths
├── launch/
│   └── robot_arm.launch.py
├── CMakeLists.txt
└── package.xml
```

## Topics

| Topic | Type | Publisher | Subscriber |
|---|---|---|---|
| `/camera/image` | `sensor_msgs/Image` | camera_node | detector_node |
| `/detector/aruco_id` | `std_msgs/Int32` | detector_node | manipulator_node |

## Parameters

All nodes load parameters from a YAML file. Default: `config/params.yaml`:

```bash
ros2 launch pick_n_place_bot robot_arm.launch.py \
  params_file:=$(ros2 pkg prefix pick_n_place_bot)/share/pick_n_place_bot/config/params.uno_q.yaml
```

```yaml
camera_node:
  ros__parameters:
    camera_device: "/dev/video0"   # V4L2 device path; falls back to camera_device_id if empty
    camera_device_id: 0
    frame_width: 640
    frame_height: 480
    frame_rate: 30

detector_node:
  ros__parameters:
    aruco_dictionary_id: 0         # 0 = DICT_4X4_50
    marker_size: 0.05
    min_marker_perimeter_rate: 0.80

manipulator_node:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    command_delay_ms: 750        # Pause after each command so the arm settles
    pick_commands:
      - "pose-40,50,0,0,180;300" # Pre-pick pose (also used on startup and between stages)
      - "wrist-45-500"
      - "gripper-100"
    place_commands_1:
      - "pose-135,55,0,55,100;300"
      - "gripper-180"
      - "wrist-0-500"
      - "home-500"
    place_commands_2:
      - "pose-110,45,10,60,100;300"
      - "gripper-180"
      - "wrist-0-500"
      - "home-500"
```

## Serial protocol

Commands are newline-terminated strings sent at 9600 baud. The Arduino sketch in `src/pick_n_place_bot/arduino/arduino.ino` drives five servos (`base_x`, `base_y`, `shoulder`, `wrist`, `gripper`) on pins 3, 5, 7, 9, and 10.

| Command | Example | Behavior |
|---|---|---|
| Instant home | `home` | All joints to home angles `[90, 90, 90, 90, 180]` |
| Trajectory home | `home-500` | Smooth return over 500 ms |
| Instant joint | `gripper-100` | Single joint move |
| Trajectory joint | `wrist-45-500` | Single joint over 500 ms |
| Synchronized pose | `pose-40,50,0,0,180;300` | All 5 joints move together over 300 ms |
| Status | `status` | Responds `JOINTS 90,90,90,90,180` |
| Chaining | `gripper-100;wrist-0-500` | Queued sequentially (`pose-*` is atomic) |

The Arduino prints `DONE` when each command or trajectory finishes. The ROS manipulator node uses a fixed `command_delay_ms` sleep after each command and does not wait for `DONE` — tune the delay to match your trajectory durations.

Pick and place sequences are defined in `params.yaml` via `pick_commands`, `place_commands_1`, and `place_commands_2`.

## USB camera on WSL2

This project is developed on WSL2 with a USB camera passed through via `usbipd-win`. See [Setting Up USB Camera In WSL2](Setting%20Up%20USB%20Camera%20In%20WSL2.md) for kernel, driver, and attach steps.

## Arduino UNO Q

Run on the board’s Debian Linux (native or Docker). Device paths differ from WSL (`/dev/video2`, CH340 serial). See [Setting Up On Arduino UNO Q](Setting%20Up%20On%20Arduino%20UNO%20Q.md).

## Dependencies

- ROS 2 Jazzy
- OpenCV 4.x with ArUco (`libopencv-dev`)
- `cv_bridge`

```bash
sudo apt-get install -y \
  libopencv-dev \
  ros-jazzy-cv-bridge
```

## Build

```bash
cd ~/ros2_arm_ws
colcon build --packages-select pick_n_place_bot
source install/setup.bash
```

## Run

```bash
ros2 launch pick_n_place_bot robot_arm.launch.py
```

On Arduino UNO Q, pass `params.uno_q.yaml` (see [Setting Up On Arduino UNO Q](Setting%20Up%20On%20Arduino%20UNO%20Q.md)).

Individual nodes:

```bash
ros2 run pick_n_place_bot camera_node
ros2 run pick_n_place_bot detector_node
ros2 run pick_n_place_bot manipulator_node
```

## Troubleshooting

**Camera not opening**

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

On WSL2, confirm the camera is attached with `usbipd` and that `/dev/video0` exists (see USB camera guide above).

**ArUco ID not published**

```bash
ros2 topic echo /detector/aruco_id
ros2 run image_view image_view --ros-args -r image:=/camera/image
```

**Serial port not opening**

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Upload `arduino/arduino.ino`, match the port in `params.yaml`, and ensure your user is in the `dialout` group.

---