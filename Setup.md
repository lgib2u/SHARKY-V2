## Raspberry Pi Zero 2 W — ROS 2 camera publisher setup (headless)

This guide continues from the point where you’ve already flashed Raspberry Pi OS (64‑bit) to an SD card using Raspberry Pi Imager with:
- SSH enabled
- A non-default username and strong password set
- Wi‑Fi credentials preconfigured

Goal: Bring the Pi online headlessly, install a minimal ROS 2 environment, enable the camera, run a ROS 2 camera publisher on the Pi, and view the stream from another ROS 2 machine.


### 1) First boot and basic system prep

1) Insert the SD card, connect camera ribbon cable, power up the Pi, and SSH in:
   - macOS/Linux: `ssh <username>@raspberrypi.local`
   - Or find the IP on your router: `ssh <username>@<pi-ip>`

2) Update the OS and firmware (reboot after):

```bash
sudo apt update
sudo apt full-upgrade -y
sudo reboot
```

3) Optional hardening and housekeeping:
- Change password if needed: `passwd`
- Set hostname (e.g., `pi-zero2w-cam`): `sudo raspi-config` → System Options → Hostname
- Set locale/timezone: `sudo raspi-config` → Localization Options


### 2) Enable and test the camera (libcamera)

Raspberry Pi OS Bookworm uses the modern libcamera stack.

1) Ensure camera stack is active and test:

```bash
sudo apt install -y libcamera-apps
libcamera-hello
```

You should see the camera preview window briefly open (over SSH it will just report success; for a headless test use a duration and no preview):

```bash
libcamera-hello --nopreview -t 2000
```

If you get camera-not-found errors, re-check the ribbon cable orientation and that the camera is enabled (Bookworm enables libcamera by default). Avoid enabling the legacy camera unless you specifically need V4L2 `/dev/video0` compatibility.


### 3) Make the Pi build-friendly (swap/zram)

Building ROS 2 from source on a Zero 2 W (512 MB RAM) benefits from extra swap. Increase swap temporarily to speed up compilation; you can reduce it later to limit SD wear.

```bash
sudo apt install -y dphys-swapfile
sudo sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
sudo systemctl restart dphys-swapfile
free -h   # verify swap increased
```


### 4) Install build tools and ROS 2 prerequisites

```bash
sudo apt install -y \
  build-essential cmake git wget curl gnupg lsb-release \
  python3-pip python3-venv python3-colcon-common-extensions python3-vcstool \
  python3-rosdep python3-empy python3-numpy \
  libasio-dev libtinyxml2-dev libyaml-cpp-dev libssl-dev \
  libjpeg-dev libpng-dev
```

Initialize rosdep (first time only):

```bash
sudo rosdep init || true
rosdep update
```


### 5) Create a minimal ROS 2 workspace (Humble, from source)

Note: Official binary packages target Ubuntu; on Raspberry Pi OS we build from source. Building on a Zero 2 W can take hours. If possible, cross-build on a faster ARM64 Pi and rsync, or be patient.

1) Create workspace and fetch sources (Humble):

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos
```

2) Install system dependencies via rosdep:

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -y --rosdistro humble --rosdep-yes
```

3) Build a minimal set to keep compile time reasonable. Start with the ROS 2 core and messaging:

```bash
cd ~/ros2_ws
colcon build \
  --merge-install --symlink-install \
  --packages-up-to rclcpp sensor_msgs image_transport \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+
```

Source your workspace:

```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```


### 6) Add a camera driver for ROS 2

Preferred (modern): libcamera-based ROS 2 driver

- `camera_ros` (libcamera for ROS 2): `https://github.com/christianrauch/camera_ros`

1) Fetch and build `camera_ros`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/christianrauch/camera_ros.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build \
  --merge-install --symlink-install \
  --packages-select camera_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+
source ~/ros2_ws/install/setup.bash
```

Fallback (legacy V4L2): If you must use `/dev/video0`, enable the legacy stack and use `v4l2_camera`:

```bash
sudo raspi-config   # Interface Options → Legacy Camera → Enable
sudo reboot

cd ~/ros2_ws/src
# v4l2_camera is maintained at GitLab:
git clone https://gitlab.com/boldhearts/ros2_v4l2_camera.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y --rosdistro humble
colcon build --merge-install --symlink-install --packages-select v4l2_camera \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+
source ~/ros2_ws/install/setup.bash
```

Alternative for USB cams: `usb_cam` (ROS 2) — `https://github.com/ros-drivers/usb_cam`


### 7) Configure networking for ROS 2 discovery

ROS 2 uses DDS discovery (multicast). Ensure the Pi and your development machine are on the same subnet and that Wi‑Fi/APs do not block multicast. For clarity, set a shared Domain ID on both machines, e.g., `7`:

```bash
echo 'export ROS_DOMAIN_ID=7' >> ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
```

Repeat similar environment setup on your viewer machine (Ubuntu 22.04 with ROS 2 Humble recommended).


### 8) Run the camera publisher on the Pi

libcamera (modern) via `camera_ros`:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run camera_ros camera_node --ros-args -p camera.frame_id:=camera_frame
```

Expected topics:

```bash
ros2 topic list
# /image_raw
# /camera_info
```

Legacy V4L2 fallback:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run v4l2_camera v4l2_camera_node \
  --ros-args -p image_size:="[640,480]" -p time_per_frame:="[1,30]"
```

Tip (bandwidth/CPU): Use compressed transport for Wi‑Fi links:

```bash
sudo apt install -y ros-$ROS_DISTRO-image-transport-plugins || true
ros2 run image_transport republish raw in:=/image_raw compressed out:=/image_raw/compressed
```


### 9) View the camera stream from another machine

On your development machine with ROS 2 (same `ROS_DOMAIN_ID`):

- With rqt:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/image_raw` (or `/image_raw/compressed` if using compressed transport).

- With RViz 2:
  - Add an Image display and set the topic to `/image_raw`.

If you don’t see topics, verify both machines can ping each other, multicast is allowed, and the environment variables are set on both ends.


### 10) Post-build cleanup (optional but recommended)

To reduce SD wear after you’re done compiling, restore swap to a smaller value (e.g., 100 MB) or disable it:

```bash
sudo sed -i 's/^CONF_SWAPSIZE=.*/CONF_SWAPSIZE=100/' /etc/dphys-swapfile
sudo systemctl restart dphys-swapfile
free -h
```


### Troubleshooting quick notes
- Camera not detected: reseat ribbon, ensure correct connector, `libcamera-hello --list-cameras`.
- DDS discovery issues: confirm same `ROS_DOMAIN_ID`, disable VPNs/firewalls, ensure multicast allowed.
- Builds failing due to memory: ensure swap is 2 GB+, use `--event-handlers console_direct+` and keep packages minimal (`--packages-up-to`).
- Performance: use lower resolutions (e.g., 640x480 @ 15–30 FPS), prefer compressed transport over Wi‑Fi.


### Where you are now
You should be able to SSH into the Pi, run a ROS 2 camera publisher (`camera_ros` recommended), and view the stream from your development machine using `rqt_image_view` or RViz. This completes the setup “up until viewing the video from the Pi’s camera.”


### Automation

These scripts are intended to run directly on the Pi (local-only). Clone the repo on the Pi and execute:

```bash
cd "Raspi Setup"
chmod +x scripts/*.sh
./scripts/01_first_boot_prep.sh
./scripts/02_install_ros2_minimal.sh

# Optional:
# export AP_SSID="Sharky-AP"
# export AP_PSK="SharkyCam123"
./scripts/03_setup_wap.sh
./scripts/04_enable_camera_service.sh
./scripts/05_setup_web_stream.sh
./scripts/06_setup_boot_wap_fallback.sh
```

After enabling the web stream, open:
- LAN/hostname: `http://localhost:8000/` (or `http://<pi-hostname>.local:8000/`)
- AP (default NetworkManager shared): `http://10.42.0.1:8000/`

Boot fallback behavior:
- On boot: LED fast blink while waiting window elapses.
- If an SSH login is detected during the window: stay on client Wi‑Fi; LED steady on.
- If not: switch to AP `SHARKY` (password `Luna21`); LED slow blink.

### Off‑board ORB_SLAM2 on macOS (recommended)

Run ORB_SLAM2 on your Mac via an Ubuntu VM, and bridge the Pi’s ROS 2 `/image_raw` into ROS 1.

1) Create an Ubuntu 20.04 VM (for ROS 1 Noetic)

```bash
# macOS (Homebrew)
brew install --cask multipass
multipass launch 20.04 --name orbslam --disk 30G --mem 6G --cpus 4
multipass shell orbslam
```

2) Inside the VM, install ROS 1 Noetic and build ORB_SLAM2

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full build-essential cmake git \
  libopencv-dev libeigen3-dev libboost-all-dev libtbb-dev libglew-dev \
  libpython3-dev python3-numpy
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
cd ~

# ORB_SLAM2
git clone https://github.com/raulmur/ORB_SLAM2.git
cd ORB_SLAM2 && chmod +x build.sh && ./build.sh
```

3) Bridge ROS 2 (Pi) → ROS 1 (VM)

- Ensure your Mac/VM and the Pi are on the same network and that DDS multicast is allowed. The Pi publishes `/image_raw` (ROS 2, Domain ID `7` by default).
- Run a ROS 1↔ROS 2 dynamic bridge on any Linux machine reachable by both (your VM is fine). Build from source for Humble↔Noetic:

```bash
# In a second Ubuntu 22.04 machine/VM (recommended) or container:
sudo apt update && sudo apt install -y build-essential cmake git python3-colcon-common-extensions
# Install ROS 2 Humble (standard instructions) and source it, then:
mkdir -p ~/bridge_ws/src && cd ~/bridge_ws/src
git clone https://github.com/ros2/ros1_bridge.git -b humble
cd ..
colcon build --cmake-force-configure
source install/setup.bash
export ROS_DOMAIN_ID=7   # match the Pi
export ROS_MASTER_URI=http://<ros1-master-ip>:11311  # your Noetic VM
ros2 run ros1_bridge dynamic_bridge
```

4) Run ORB_SLAM2, subscribing to the bridged ROS 1 topic

```bash
# In the Ubuntu 20.04 VM (ROS 1)
source /opt/ros/noetic/setup.bash
roscore   # in one terminal

# In another terminal in the same VM:
cd ~/ORB_SLAM2
# Use a camera settings YAML (calibrate your Pi camera; see repo docs)
rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/YourCamera.yaml
```

Verify the ROS 1 topic exists:

```bash
rostopic list | grep image_raw    # should show /camera/image_raw
```

Notes:
- You must provide a correct camera calibration YAML for ORB_SLAM2. Calibrate with ROS’s `camera_calibration` and save a YAML matching the `Examples/Monocular/*.yaml` schema in ORB_SLAM2.
- ORB_SLAM2 details and dependencies: see the repository README [raulmur/ORB_SLAM2](https://github.com/raulmur/ORB_SLAM2).


### Automation (run directly on the Pi)

If you prefer to clone this repo on the Pi and run everything locally (no SSH hop), enable local mode. Local mode maps the helper functions to run commands directly with `sudo` instead of over SSH.

```bash
# On the Pi
git clone <your-repo-url> "Raspi Setup"
cd "Raspi Setup"
chmod +x scripts/*.sh

# Enable local mode (or set HOST=localhost which auto-enables)
export RUN_LOCAL=true

./scripts/01_first_boot_prep.sh
./scripts/02_install_ros2_minimal.sh
# Optional:
./scripts/03_setup_wap.sh
./scripts/04_enable_camera_service.sh
./scripts/05_setup_web_stream.sh
./scripts/06_setup_boot_wap_fallback.sh
```

Notes:
- You can still override configuration via env vars (e.g., `AP_SSID`, `AP_PSK`, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`).
- In local mode, SSH/scp are not required; commands execute directly on the Pi.
