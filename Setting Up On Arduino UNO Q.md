# Pick & Place on Arduino UNO Q

> **Board:** [Arduino UNO Q](https://docs.arduino.cc/hardware/uno-q) — Debian Linux (MPU) + STM32 (MCU sketches).  
> **Repo:** [mihir-robotics/ros2_pick_and_place](https://github.com/mihir-robotics/ros2_pick_and_place.git)

---

## What you need

- UNO Q on your network (SSH as `arduino@<hostname>`)
- USB-C hub/dongle with **power delivery** if USB devices drop (`usb_vbus: disabling` in `dmesg`)
- USB camera and arm serial over the board’s USB ports
- ROS 2 Jazzy on the host (`/opt/ros/jazzy`) **or** Docker image `ros:jazzy-ros-base`

---

## Device paths (Uno Q vs PC/WSL)

| Device | WSL / PC default | UNO Q (typical) |
|---|---|---|
| USB camera | `/dev/video0` | **`/dev/video2`** (capture); `/dev/video3` metadata |
| SoC codec (ignore) | — | `/dev/video0`, `/dev/video1` (Qualcomm Venus) |
| Arm serial (CH340) | `/dev/ttyUSB0` | **`/dev/ttyCH341USB0`** (after `ch341` driver loaded) |

Use **`params.uno_q.yaml`** on the host, or map devices in Docker so default `params.yaml` still works (see below).

---

## Step 1 — Verify USB on the host (not inside Docker)

```bash
sudo apt install -y usbutils v4l-utils
lsusb
v4l2-ctl --list-devices
ls -l /dev/video*
```

Under **USB CAMERA**, note the capture node (usually **`/dev/video2`**).

Quick capture test:

```bash
v4l2-ctl -d /dev/video2 --set-fmt-video=width=640,height=480,pixelformat=MJPG
v4l2-ctl -d /dev/video2 --stream-mmap --stream-count=5
```

---

## Step 2 — CH340 serial (arm ↔ Linux)

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyCH341USB* 2>/dev/null
```

If `lsusb` shows `1a86:7523` (CH340) but there is **no** tty device:

```bash
sudo modprobe ch341   # often missing on stock Uno Q kernel
```

If `modprobe` fails, build/load the out-of-tree driver from [Arduino linux-qcom ch341](https://github.com/arduino/linux-qcom/blob/qcom-v6.16.7-unoq/drivers/usb/serial/ch341.c), then:

```bash
dmesg | grep -i ch34
ls -l /dev/ttyCH341USB0
```

Permissions:

```bash
sudo usermod -aG dialout,video $USER
# log out and back in
```

---

## Step 3 — Clone and build (host)

```bash
git clone https://github.com/mihir-robotics/ros2_pick_and_place.git ~/ros2_arm_ws
cd ~/ros2_arm_ws
source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libopencv-dev \
  ros-jazzy-cv-bridge ros-jazzy-launch-ros ros-jazzy-image-view
colcon build --packages-select pick_n_place_bot --symlink-install --parallel-workers 1
source install/setup.bash
```

Run with Uno Q parameters:

```bash
ros2 launch pick_n_place_bot robot_arm.launch.py params_file:=$(ros2 pkg prefix pick_n_place_bot)/share/pick_n_place_bot/config/params.uno_q.yaml
```

Install path after build (alternative to `ros2 pkg prefix`):

```bash
ros2 launch pick_n_place_bot robot_arm.launch.py \
  params_file:=~/ros2_arm_ws/src/pick_n_place_bot/config/params.uno_q.yaml
```

---

## Step 4 — Run in Docker

Plain `docker run` does **not** see USB devices. Pass them through and alias names so **`params.yaml`** defaults work:

```bash
docker run -it --rm \
  --device=/dev/video2:/dev/video0 \
  --device=/dev/video3:/dev/video1 \
  --device=/dev/ttyCH341USB0:/dev/ttyUSB0 \
  --group-add video \
  --group-add dialout \
  --network host \
  -v ~/ros2_arm_ws:/ros2_ws \
  -w /ros2_ws \
  ros:jazzy-ros-base \
  bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pick_n_place_bot --symlink-install --parallel-workers 1
source install/setup.bash
ros2 launch pick_n_place_bot robot_arm.launch.py
```

**Note:** Aliasing is **container-only**. On the host, `/dev/video0` is still Venus — do not use `/dev/video0` for the USB camera on bare-metal runs.

Keep the repo on the **host** (`~/ros2_arm_ws`) via `-v`; container filesystem is ephemeral.

---

## Step 5 — MCU firmware

Upload `src/pick_n_place_bot/arduino/arduino.ino` to the **STM32** with Arduino IDE 2 or Arduino App Lab. The Linux side talks to the arm at **9600 baud** over serial (see [README.md](README.md)). This is separate from ROS/Docker.

---

## Pull, build, run (cheat sheet)

**Host:**

```bash
cd ~/ros2_arm_ws && git pull origin main && \
source /opt/ros/jazzy/setup.bash && \
colcon build --packages-select pick_n_place_bot --symlink-install --parallel-workers 1 && \
source install/setup.bash && \
ros2 launch pick_n_place_bot robot_arm.launch.py \
  params_file:=$(ros2 pkg prefix pick_n_place_bot)/share/pick_n_place_bot/config/params.uno_q.yaml
```

**Docker** (after `docker run` with devices + volume as above):

```bash
source /opt/ros/jazzy/setup.bash && \
colcon build --packages-select pick_n_place_bot --symlink-install --parallel-workers 1 && \
source install/setup.bash && \
ros2 launch pick_n_place_bot robot_arm.launch.py
```

Clean builds on ARM often take **1–3 minutes**; unchanged rebuilds are much faster.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Camera opens wrong device | Used `/dev/video0` on host | Use `/dev/video2` or Uno Q params / Docker alias |
| No serial port | CH341 driver not loaded | Load/build `ch341`; check `dmesg \| grep ch34` |
| Works on host, not in Docker | No `--device` / groups | Add device maps, `--group-add video`, `--group-add dialout` |
| Repo gone after reboot | Cloned inside container only | Clone on host; mount `-v ~/ros2_arm_ws:/ros2_ws` |
| Manipulator logs only, no motion | Serial missing | Fix tty; else node runs in simulation mode |
| Slow `colcon build` | ARM CPU vs desktop | Use `--parallel-workers 1` |
