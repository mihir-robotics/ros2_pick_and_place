# USB Camera in WSL2 — Complete Setup Guide (Made by Claude, but worked for me)

> **Environment:** Ubuntu 24.04 on WSL2, Windows 11, `usbipd-win`

---

## Prerequisites

- WSL2 (not WSL1)
- [`usbipd-win`](https://github.com/dorssel/usbipd-win) v4.0+ installed on Windows
- Admin/elevated PowerShell for usbipd commands

---

## Step 1 — Share and Attach the Camera via usbipd

In an **elevated PowerShell**, list connected USB devices:

```powershell
usbipd list
```

Share the camera (one-time step):

```powershell
usbipd bind --busid <busid>
```

Attach it to WSL:

```powershell
usbipd attach --wsl --busid <busid>
```

Verify it shows **Attached**:

```powershell
usbipd list
```

---

## Step 2 — Verify the Camera is Visible in WSL

```bash
ls /dev/video*
v4l2-ctl --list-devices
dmesg | grep -i uvc
```

If `/dev/video*` doesn't exist and `dmesg` returns nothing → the kernel is missing the `uvcvideo` driver. Continue to Step 3.

---

## Step 3 — Build a Custom WSL2 Kernel with UVC Support

The stock WSL2 kernel ships without `uvcvideo` (USB camera driver) and sometimes without `vhci_hcd` (USB-over-IP passthrough). You need to build the kernel yourself with these enabled.

> **Note:** This does NOT affect your WSL filesystem, installed packages, or ROS2. Only the kernel binary changes.

### 3.1 — Find Your Kernel Version

```bash
uname -r
# Example output: 6.18.33.1-microsoft-standard-WSL2
```

### 3.2 — Install Build Dependencies

```bash
sudo apt install build-essential flex bison libssl-dev libelf-dev \
  libncurses-dev bc dwarves pahole git
```

### 3.3 — Clone the Matching Kernel Source

```bash
git clone https://github.com/microsoft/WSL2-Linux-Kernel.git
cd WSL2-Linux-Kernel
git checkout linux-msft-wsl-6.18.33.1   # replace with your version
```

### 3.4 — Set Up the Base Config

```bash
cp Microsoft/config-wsl .config
```

### 3.5 — Enable Required Modules

```bash
scripts/config --enable CONFIG_MEDIA_SUPPORT
scripts/config --enable CONFIG_MEDIA_USB_SUPPORT
scripts/config --enable CONFIG_USB_VIDEO_CLASS
scripts/config --enable CONFIG_USB_VIDEO_CLASS_INPUT_EVDEV
scripts/config --enable CONFIG_USBIP_CORE
scripts/config --enable CONFIG_USBIP_VHCI_HCD
```

### 3.6 — Build the Kernel

```bash
make -j$(nproc) KCONFIG_CONFIG=.config
```

> Takes 20–40 minutes depending on CPU. Uses all available cores.

### 3.7 — Install Kernel Modules

```bash
sudo make modules_install
```

Verify the modules are present:

```bash
find /lib/modules -name "vhci*"
find /lib/modules -name "uvcvideo*"
```

### 3.8 — Copy the Kernel to Windows

```bash
mkdir -p /mnt/c/WSL
cp arch/x86/boot/bzImage /mnt/c/WSL/bzImage-uvc
```

---

## Step 4 — Point WSL2 at the New Kernel

Open `.wslconfig` in PowerShell:

```powershell
notepad $env:USERPROFILE\.wslconfig
```

Add or update:

```ini
[wsl2]
kernel=C:\\WSL\\bzImage-uvc
```

> **Important:** Use double backslashes. The file must be named exactly `.wslconfig` (not `.wslconfig.txt`).

Restart WSL:

```powershell
wsl --shutdown
```

Then reopen your WSL terminal.

---

## Step 5 — Verify Everything Works

**Confirm the new kernel loaded:**

```bash
uname -r
# Should show: 6.18.33.1-microsoft-standard-WSL2+
```

**Re-attach the camera in PowerShell** (needs to be re-done after every `wsl --shutdown`):

```powershell
usbipd attach --wsl --busid <busid>
```

**In WSL, check the module and device:**

```bash
lsmod | grep uvc        # should show uvcvideo
ls /dev/video*          # should show /dev/video0
v4l2-ctl --list-devices # should list your camera
```

---

## Troubleshooting Reference

| Symptom | Cause | Fix |
|---|---|---|
| `dmesg \| grep uvc` returns nothing | `uvcvideo` module missing from kernel | Build custom kernel with UVC enabled (Step 3) |
| `usbipd: error: Loading vhci_hcd failed` | `vhci_hcd` module missing | Enable `CONFIG_USBIP_CORE` + `CONFIG_USBIP_VHCI_HCD` and rebuild |
| `uname -r` still shows old version after rebuild | `.wslconfig` not picked up | Check path, double backslashes, no `.txt` extension; run `wsl --shutdown` |
| `lsmod \| grep uvc` empty after new kernel | Camera not yet attached | Run `usbipd attach --wsl --busid <busid>` in PowerShell first |
| `/dev/video*` not found after attach | Module not installed | Run `sudo make modules_install` and reboot WSL |
| `wsl --update` doesn't fix `vhci_hcd` | Community kernel (e.g. Nevuly) missing `vhci_hcd` | Use the official Microsoft kernel source and build yourself |

---

## Notes

- **Community pre-built kernels** (e.g. Nevuly/WSL2-Linux-Kernel-Rolling) are missing `vhci_hcd`, which breaks `usbipd`. Use the official Microsoft kernel source instead.
- **`usbipd attach` must be re-run** after every `wsl --shutdown` or system reboot — it does not persist automatically.
- If you have multiple cameras, each has its own `busid`. Attach them individually as needed.
- All WSL distros on the machine share the same kernel, so this fix applies to all of them.
