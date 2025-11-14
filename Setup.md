
### Ubuntu 25.10 (Raspberry Pi) — Copy/paste prompt to scaffold a fresh setup

Use the following prompt to generate a new Ubuntu 25.10-specific setup directory and scripts (separate from Raspberry Pi OS). Paste it into your assistant to scaffold the project.

```text
Create Ubuntu 25.10 Raspberry Pi ROS 2 camera setup

Build a new setup directory for Ubuntu Server 25.10 (arm64) on Raspberry Pi using Raspberry Pi Imager. Target: minimal, headless ROS 2 camera publisher with optional web stream and AP fallback. Use ROS 2 Rolling from apt (no source builds for core ROS).

Requirements:
- OS: Ubuntu Server 25.10 for Raspberry Pi (arm64), headless
- Shell: bash
- Package manager: apt
- Network: NetworkManager
- ROS 2: Rolling via apt (ros-rolling-ros-base)
- Camera: libcamera; ROS 2 driver via camera_ros from source
- Optional: lightweight Flask/Picamera2 MJPEG web stream
- Services: systemd units for camera publisher and web stream
- AP fallback: switch to AP via NetworkManager if no SSH login after a wait period
- Clear, idempotent scripts; log progress; safe to re-run

Directory structure:
ubuntu-25.10/
  Setup.md
  scripts/
    bootstrap_env.sh
    01_first_boot_prep.sh
    02_install_ros2_rolling.sh
    03_setup_wap.sh
    04_enable_camera_service.sh
    05_setup_web_stream.sh
    06_setup_boot_wap_fallback.sh

Script contracts:
- scripts/bootstrap_env.sh
  - Defaults: ROS_DISTRO=rolling, ROS_DOMAIN_ID=7, RMW_IMPLEMENTATION=rmw_cyclonedds_cpp, AP_SSID=SHARKY, AP_PSK=Luna21
  - Helpers: remote, remote_sudo (local execution wrappers)
- scripts/01_first_boot_prep.sh
  - apt full-upgrade
  - Install base deps:
    - libcamera-apps libcamera-dev
    - build-essential cmake git wget curl gnupg lsb-release software-properties-common
    - python3-{pip,venv,empy,numpy,rosdep,vcstool,colcon-common-extensions}
    - libasio-dev libtinyxml2-dev libyaml-cpp-dev libssl-dev libjpeg-dev libpng-dev
    - network-manager
  - Add ROS 2 apt repo + key; apt update; install ros-rolling-ros-base
  - rosdep init/update
  - Append to ~/.bashrc: source /opt/ros/rolling/setup.bash, export ROS_DOMAIN_ID/RMW_IMPLEMENTATION
- scripts/02_install_ros2_rolling.sh
  - Create ~/ros2_ws/src, clone https://github.com/christianrauch/camera_ros.git
  - rosdep install --from-paths src --ignore-src -y --rosdistro rolling (use --skip-keys if needed rather than fail)
  - colcon build --merge-install --symlink-install --packages-select camera_ros --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --event-handlers console_direct+
  - Append source ~/ros2_ws/install/setup.bash to ~/.bashrc
- scripts/03_setup_wap.sh
  - Ensure NetworkManager enabled/started
  - Configure AP ${AP_SSID} on wlan0: mode ap, band bg, ipv4.method shared, WPA-PSK ${AP_PSK}
- scripts/04_enable_camera_service.sh
  - Create /etc/systemd/system/libcamera_ros.service (User=${USER}, WorkingDirectory=/home/${USER})
  - Env: ROS_DOMAIN_ID, RMW_IMPLEMENTATION
  - ExecStart: bash -lc 'source /home/${USER}/ros2_ws/install/setup.bash && exec ros2 run camera_ros camera_node --ros-args -p camera.frame_id:=camera_frame'
  - Restart=on-failure; enable --now
- scripts/05_setup_web_stream.sh (optional)
  - apt install python3-flask python3-picamera2 (if picamera2 unavailable, warn and exit 0)
  - Deploy /opt/sharky-webcam/app.py with Picamera2 MJPEG stream at /stream.mjpg and /snapshot.jpg
  - Create sharky_webcam.service; enable --now; print URLs
- scripts/06_setup_boot_wap_fallback.sh
  - /usr/local/sbin/sharky_boot_net.sh: wait ${WAIT_SECS:-60}; fast blink while waiting; if SSH accepted this boot, stay client; else enable AP; slow blink
  - sharky_boot_net.service (Type=oneshot, User=root, After/Wants=network-online.target), enable (start on next boot)

Documentation:
- Setup.md tailored to Ubuntu 25.10 with step-by-step usage and troubleshooting (multicast, camera ribbon, AP connectivity).

Acceptance criteria:
- 01 completes, configures ROS 2 apt repo, installs ros-rolling-ros-base, rosdep initialized
- 02 builds camera_ros and adds workspace sourcing
- Camera service starts on boot and publishes /image_raw
- Hotspot script brings up AP with NAT; boot fallback toggles AP if no SSH login
- Scripts are idempotent, safe heredocs (variables expanded when writing units), and log clearly
```
