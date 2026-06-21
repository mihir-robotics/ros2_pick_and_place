# Quick Start

## Prerequisites

- ROS 2 Jazzy or Humble on Ubuntu 22.04/24.04
- USB camera (V4L2)
- Arduino Nano with the servo firmware in `src/pick_n_place_bot/arduino/arduino.ino`
- USB serial connection to the Arduino

## 1. Install dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  libopencv-dev \
  ros-jazzy-cv-bridge \
  ros-jazzy-launch-ros
```

Replace `jazzy` with `humble` if needed.

## 2. Build

```bash
cd ~/ros2_arm_ws
colcon build --packages-select pick_n_place_bot --symlink-install
source install/setup.bash
```

## 3. USB camera

Check that the camera is visible:

```bash
ls -l /dev/video*
python3 -c "import cv2; cap=cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2); print(cap.isOpened())"
```

On WSL2, follow [Setting Up USB Camera In WSL2](Setting%20Up%20USB%20Camera%20In%20WSL2.md) to attach the camera and enable the UVC driver.

On **Arduino UNO Q**, use [Setting Up On Arduino UNO Q](Setting%20Up%20On%20Arduino%20UNO%20Q.md) (see that guide).

Set `camera_device` under the `camera_node:` section in `src/pick_n_place_bot/config/params.yaml` to match your device (default: `/dev/video0` on WSL/PC).

## 4. Arduino

1. Upload `src/pick_n_place_bot/arduino/arduino.ino` to the board.
2. Connect the USB serial cable.
3. Confirm the port:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

4. Set `serial_port` under the `manipulator_node:` section in `config/params.yaml` if the port differs from `/dev/ttyUSB0`.

## 5. Configure pick and place

Edit `src/pick_n_place_bot/config/params.yaml`:

- **`pick_commands`** — first entry is the pre-pick / idle pose (wrist up); remaining entries are optional fixed steps (e.g. gripper preset).
- **`pick_point_*`** — where the arm grasps when a marker is seen (IK target in meters).
- **`place_line_origin_*` / `place_line_step_*`** — row of up to `max_line_slots` placements; keep `place_line_step_z: 0.0` for a flat row on the board.
- **`serial_port`**, **`command_delay_ms`**, and IK link params — match your arm geometry and settling time.

Tune placement on hardware with a top-down view: set the origin to the first object center and adjust step X/Y along the row. See [README.md](README.md) for the full parameter list and simulation-mode testing.

## 6. Run

Launch all nodes:

```bash
ros2 launch pick_n_place_bot robot_arm.launch.py
```

Or test nodes separately:

```bash
# Terminal 1
ros2 run pick_n_place_bot camera_node

# Terminal 2
ros2 run pick_n_place_bot detector_node

# Terminal 3 — monitor detections
ros2 topic echo /detector/aruco_id

# Terminal 4 — manipulator (simulation mode if serial is unavailable)
ros2 run pick_n_place_bot manipulator_node

# Optional: trigger one cycle without the camera (use a new ID per object)
ros2 topic pub --once /detector/aruco_id std_msgs/msg/Int32 "{data: 1}"
```

View the camera feed:

```bash
ros2 run image_view image_view --ros-args -r image:=/camera/image
```

## Troubleshooting

**Build errors**

```bash
cd ~/ros2_arm_ws
rm -rf build install log
colcon build --packages-select pick_n_place_bot
```

**Serial permission denied**

```bash
sudo usermod -a -G dialout $USER
```

Log out and back in for the group change to apply.

**Camera not found**

```bash
v4l2-ctl --list-devices
ros2 run pick_n_place_bot camera_node --ros-args -p camera_device:=/dev/video1
```

See [README.md](README.md) for topic, parameter, and protocol details.
