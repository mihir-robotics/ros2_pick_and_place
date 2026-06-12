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
  ros-jazzy-image-transport \
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

Set `camera_device` in `src/pick_n_place_bot/config/params.yaml` to match your device (default: `/dev/video0`).

## 4. Arduino

1. Upload `src/pick_n_place_bot/arduino/arduino.ino` to the board.
2. Connect the USB serial cable.
3. Confirm the port:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

4. Set `manipulator_node.serial_port` in `config/params.yaml` if the port differs from `/dev/ttyUSB0`.

## 5. Configure pick and place

Edit `src/pick_n_place_bot/config/params.yaml`. The command lists define the arm motion; each entry is a serial command understood by the Arduino sketch (e.g. `home`, `gripper-100`, `base_y-110;shoulder-180`).

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
