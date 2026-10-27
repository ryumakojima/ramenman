# ramenman — Autonomous Drone Software

Software for our autonomous drone. It runs on a **Raspberry Pi 5** with the **Lenna stereo camera** (2× IMX219 cameras + ICM-20948 IMU). The drone figures out where it is from camera + IMU (**VIO**) and sends that to the flight controller. **YOLO** spots targets and decides where to fly.

```
 Stereo camera ─┐
                ├─► OpenVINS (VIO) ─► "where am I" ─► Flight controller (telemetry port, MAVLink)
 IMU ───────────┘                                          ▲
 Camera ─► YOLO ─► "what do I see" ─► decision logic ──────┘ "fly here"
```

---

## Contents

1. [Concepts in plain words](#1-concepts-in-plain-words)
2. [Roadmap and status](#2-roadmap-and-status)
3. [Hardware](#3-hardware)
4. [Working with this repo (git basics)](#4-working-with-this-repo-git-basics)
5. [Raspberry Pi setup](#5-raspberry-pi-setup)
6. [Step 1 — Sensor bring-up](#6-step-1--sensor-bring-up)
7. [Step 2 — Record calibration data](#7-step-2--record-calibration-data)
8. [Step 3 — Camera calibration (Kalibr)](#8-step-3--camera-calibration-kalibr)
9. [Step 4 — IMU noise model](#9-step-4--imu-noise-model)
10. [Step 5 — Camera–IMU calibration (Kalibr)](#10-step-5--cameraimu-calibration-kalibr)
11. [Step 6 — Visual odometry sanity check](#11-step-6--visual-odometry-sanity-check)
12. [Step 7 — OpenVINS (VIO)](#12-step-7--openvins-vio)
13. [Step 8 — Send position to the flight controller](#13-step-8--send-position-to-the-flight-controller)
14. [Step 9 — YOLO autonomy](#14-step-9--yolo-autonomy)
15. [Flight safety rules](#15-flight-safety-rules)
16. [Troubleshooting](#16-troubleshooting)
17. [Reference repos](#17-reference-repos)

---

## 1. Concepts in plain words

| Term | Meaning |
|---|---|
| **Odometry** | Estimating your own movement and position (自己位置推定). Indoors there's no GPS, so the drone has to do this itself. |
| **IMU** | Small sensor measuring acceleration (accelerometer) and rotation speed (gyroscope), like the one in a phone. |
| **Visual Odometry (VO)** | Camera only. Tracks how features in the image move between frames. Fails with motion blur, blank walls, or darkness. |
| **Inertial Odometry** | IMU only. Drifts by metres within seconds, so it's not usable alone. |
| **VIO / VINS** | Visual-Inertial Odometry / Navigation System. Camera + IMU combined: the IMU handles fast motion and the camera corrects IMU drift. This is what we actually fly with. |
| **OpenVINS** | Open-source VIO software. We configure it; we don't write VIO ourselves. |
| **Kalibr** | Calibration tool. Measures camera lens properties, the position of the two cameras relative to each other, and the camera ↔ IMU position and time offset. VIO doesn't work without this. |
| **Stereo** | Two cameras side by side (like two eyes), which gives depth. |
| **ROS 2** | Framework that lets programs (camera driver, VIO, controller) pass data to each other as "topics". |
| **MAVLink** | The message language flight controllers speak. We send position and commands over the telemetry port. |
| **YOLO** | Real-time object detection neural network. |

---

## 2. Roadmap and status

| # | Step | Where it runs | Status |
|---|---|---|---|
| 1 | Sensor bring-up (both cameras + IMU working) | Pi | ✅ code ready, needs hardware test |
| 2 | Record calibration datasets | Pi | ⏳ script coming (`record_dataset.py`) |
| 3 | Camera calibration | Laptop (Kalibr) | ⏳ |
| 4 | IMU noise model (Allan variance) | Pi → laptop | ⏳ |
| 5 | Camera–IMU calibration | Laptop (Kalibr) | ⏳ |
| 6 | Visual odometry sanity check | Laptop or Pi | ⏳ |
| 7 | OpenVINS VIO + ROS 2 sensor node | Pi | ⏳ |
| 8 | VIO → flight controller bridge (MAVLink) | Pi | ⏳ |
| 9 | YOLO → flight commands | Pi | ⏳ |

**Rule: don't skip ahead.** If calibration is bad, everything after it fails in confusing ways.

---

## 3. Hardware

- **Companion computer:** Raspberry Pi 5
- **Camera:** Lenna stereo camera, 2× IMX219 + ICM-20948 IMU
- **Flight controller:** PIXweser v1.0, connected through its telemetry port
- **Detection:** YOLO (already installed on the Pi)

### Important notes

- The [Lenna-Stereo-Camera](https://github.com/Lenna-Robotics-Research-Lab/Lenna-Stereo-Camera) repo targets an **NVIDIA Jetson Orin Nano**. Its camera code uses `nvarguscamerasrc`, which **does not exist on a Raspberry Pi**, so camera capture here is rewritten with **Picamera2 (libcamera)**. The IMU is the same ICM-20948 over I2C, so that part carries over.
- **Pi 5 camera ports use the smaller 22-pin connector.** Standard IMX219 modules use 15-pin, so you need **22-pin → 15-pin ribbon cables** (one per camera).
- The two cameras are **not hardware-synchronised**. The test script shows the timestamp gap between left and right.

### Wiring

**IMU → Pi 5 (I2C)**

| IMU | Pi 5 header pin |
|---|---|
| 3.3V / VCC | Pin 1 (3.3V) |
| GND | Pin 6 (GND) |
| SDA | Pin 3 (GPIO2) |
| SCL | Pin 5 (GPIO3) |

**Flight controller telemetry port → Pi 5 (UART)** — only needed from step 8

| FC telemetry | Pi 5 header pin |
|---|---|
| TX | Pin 10 (GPIO15, RX) |
| RX | Pin 8 (GPIO14, TX) |
| GND | Pin 6 or 14 (GND) |
| 5V | **Do NOT connect** (power the Pi separately) |

TX goes to RX, and RX goes to TX (crossed). Check the FC's telemetry pinout before wiring.

---

## 4. Working with this repo (git basics)

**First time:**
```bash
git clone https://github.com/ryumakojima/ramenman.git
cd ramenman
```

**Every time before you start working:**
```bash
git pull
```

**After you change something:**
```bash
git add .
git commit -m "short description of what you changed"
git push
```

**For bigger changes, work on a branch** so you don't break `main` for everyone:
```bash
git checkout -b my-feature
# ...make changes, commit...
git push -u origin my-feature
# then open a Pull Request on GitHub
```

Never commit large recordings (`.bag`, `.db3`, image folders). `.gitignore` already blocks the common ones.

---

## 5. Raspberry Pi setup

These instructions assume **Raspberry Pi OS Bookworm (64-bit)**. Check which OS you have with:
```bash
cat /etc/os-release
```

> ⚠️ **OS decision for later (step 7):** OpenVINS runs on ROS 2. ROS 2 is easiest on **Ubuntu 24.04 (ROS 2 Jazzy)**. On Raspberry Pi OS, it has to run inside **Docker**. Steps 1–6 work on Raspberry Pi OS, so no decision is needed yet.

### Install

```bash
# 1. Enable I2C (for the IMU)
sudo raspi-config nonint do_i2c 0

# 2. System packages (Picamera2 MUST come from apt, not pip)
sudo apt update
sudo apt install -y python3-picamera2 python3-opencv python3-numpy i2c-tools git

# 3. Get the code
git clone https://github.com/ryumakojima/ramenman.git
cd ramenman

# 4. Python virtual environment that can still see the apt packages
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Run `source .venv/bin/activate` again every time you open a new terminal.

### Check the hardware is detected

```bash
rpicam-hello --list-cameras   # should list 2 cameras (imx219)
i2cdetect -y 1                # should show 68 (or 69) in the grid
```

If you see only 0 or 1 camera, edit `/boot/firmware/config.txt`:
```bash
sudo nano /boot/firmware/config.txt
```
Add these lines at the bottom, then reboot:
```
camera_auto_detect=0
dtoverlay=imx219,cam0
dtoverlay=imx219,cam1
```

---

## 6. Step 1 — Sensor bring-up

**Goal:** prove both cameras and the IMU deliver good data on the Pi.

```bash
python3 test_sensors.py              # window with both images + IMU overlay (needs monitor or VNC)
python3 test_sensors.py --headless   # over SSH: prints stats only
python3 test_sensors.py --no-camera  # IMU only
python3 test_sensors.py --no-imu     # cameras only
python3 test_sensors.py --imu-addr 0x69   # if i2cdetect showed 69
```

Press `q` in the window, or `Ctrl+C`, to quit.

### Pass criteria

- [ ] Both images show, at around 30 fps.
- [ ] Left/right are the correct way round. Cover the left lens and the left image should go dark. If it's reversed, swap the indices in `StereoCamera(left_index=1, right_index=0)`.
- [ ] Board sitting still: `|a|` ≈ **9.8**, and all gyro values are ≈ **0**.
- [ ] Rotating the board moves the gyro values. Tilting it shifts accel x/y/z.
- [ ] IMU rate ≈ **200 Hz**.
- [ ] L/R gap is consistently small (a few ms or less).

When all boxes are ticked, post a screenshot or the terminal output in the team chat.

---

## 7. Step 2 — Record calibration data

⏳ *Script coming:* `record_dataset.py`

It will save data in the folder format Kalibr's `kalibr_bagcreater` expects, so **we don't need ROS on the Pi for calibration**:
```
dataset/
  cam0/<timestamp_ns>.png
  cam1/<timestamp_ns>.png
  imu0.csv        # timestamp_ns, gyro xyz (rad/s), accel xyz (m/s^2)
```

We need **three recordings**:

| Recording | How | Length |
|---|---|---|
| `static_cam` | Move the camera slowly in front of the AprilGrid, covering all parts of both images | ~1–2 min |
| `imu_static` | IMU sitting completely still on a table. Don't touch it. | **2–3 hours** |
| `dynamic` | Move the camera in front of the AprilGrid, exciting **all 6 axes** (up/down, left/right, forward/back, roll, pitch, yaw), smoothly, no blur | ~1–2 min |

### Calibration target (AprilGrid)

1. Download the AprilGrid PDF from the [Kalibr wiki (Calibration targets)](https://github.com/ethz-asl/kalibr/wiki/calibration-targets).
2. Print it at **100% scale** (no "fit to page") on A3 if possible.
3. Stick it to something **flat and rigid** (foam board or a clipboard). A bent target ruins calibration.
4. **Measure one tag with a ruler** and write it in `april_6x6.yaml`:
```yaml
target_type: 'aprilgrid'
tagCols: 6
tagRows: 6
tagSize: 0.088      # metres, MEASURE YOURS
tagSpacing: 0.3     # gap between tags / tag size
```

Recording tips: good even lighting, the whole board in view as often as possible, and no fast jerky motion.

---

## 8. Step 3 — Camera calibration (Kalibr)

**Runs on a laptop, not the Pi.** Kalibr is easiest through Docker.

```bash
# one-time: build the Kalibr docker image
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
docker build -t kalibr -f Dockerfile_ros1_20_04 .
```

```bash
# start a Kalibr container with your data folder mounted
docker run -it -v ~/calib_data:/data kalibr
```

Inside the container:
```bash
source devel/setup.bash

# convert the recorded folder into a ROS bag
rosrun kalibr kalibr_bagcreater --folder /data/static_cam --output-bag /data/static_cam.bag

# calibrate both cameras
rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/static_cam.bag \
  --target /data/april_6x6.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw
```

Output: `camchain-static_cam.yaml` plus a PDF report.

**Check the report:** reprojection error should be **well under 1 pixel** (ideally < 0.5). If it's bad, re-record with better lighting, slower motion, and a flatter target.

Commit the result to `config/calibration/`.

---

## 9. Step 4 — IMU noise model

We need four numbers describing how noisy our specific IMU is:

- `accelerometer_noise_density`
- `accelerometer_random_walk`
- `gyroscope_noise_density`
- `gyroscope_random_walk`

**How:** take the 2–3 hour `imu_static` recording, then run an Allan variance analysis on it. Options are [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros) or a Python script (⏳ coming: `tools/allan_variance.py`).

Write the results into `imu.yaml`:
```yaml
rostopic: /imu0
update_rate: 200.0
accelerometer_noise_density: 0.0   # fill in
accelerometer_random_walk: 0.0     # fill in
gyroscope_noise_density: 0.0       # fill in
gyroscope_random_walk: 0.0         # fill in
```

Common practice is to **multiply the measured values by ~5–10** before using them in Kalibr/OpenVINS, because the drone's vibration and temperature changes make real conditions noisier than a lab table.

---

## 10. Step 5 — Camera–IMU calibration (Kalibr)

On the laptop, in the Kalibr container:

```bash
rosrun kalibr kalibr_bagcreater --folder /data/dynamic --output-bag /data/dynamic.bag

rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/dynamic.bag \
  --target /data/april_6x6.yaml \
  --cam /data/camchain-static_cam.yaml \
  --imu /data/imu.yaml
```

Output: `camchain-imucam-dynamic.yaml`. It contains:

- **`T_cam_imu`**: where each camera is relative to the IMU. Sanity-check it against a ruler measurement of the board. It should be within a centimetre or so.
- **`timeshift_cam_imu`**: the time offset between camera and IMU. It should be small (a few ms). A large value means a timestamp problem.

Commit the result to `config/calibration/`.

---

## 11. Step 6 — Visual odometry sanity check

**Goal:** confirm the calibration is good using the cameras alone, before adding OpenVINS.

⏳ *Script coming:* `tools/stereo_vo_check.py`. It will rectify both images with our calibration, track features, and plot the estimated path.

**Test:** walk the camera in a rectangle about 3 m × 3 m and come back to the start. The plotted path should look like a rectangle and end near where it started.

If the path is badly distorted or scaled wrong, go back to step 3.

---

## 12. Step 7 — OpenVINS (VIO)

OpenVINS runs on ROS 2 and reads Kalibr's output files directly.

### 7a. ROS 2 environment

- **Ubuntu 24.04 on the Pi:** install ROS 2 Jazzy natively following the [official guide](https://docs.ros.org/en/jazzy/Installation.html).
- **Raspberry Pi OS:** run ROS 2 inside Docker (⏳ `docker/Dockerfile` coming).

### 7b. ROS 2 sensor node

⏳ *Coming:* `ros2/stereo_imu_node`, which publishes:
- `/cam0/image_raw`, `/cam1/image_raw` (sensor_msgs/Image)
- `/imu0` (sensor_msgs/Imu)

It reuses the same `sensors/` drivers from step 1.

### 7c. Build OpenVINS

```bash
mkdir -p ~/ws/src && cd ~/ws/src
git clone https://github.com/rpng/open_vins.git
cd ~/ws
colcon build --event-handlers console_cohesion+ --packages-select ov_core ov_init ov_msckf
source install/setup.bash
```

Building on the Pi 5 is slow (30+ minutes). Close other programs while it builds.

### 7d. Configure

Create `config/openvins/` with three files (start by copying one of the example configs in `open_vins/config/`):
- `estimator_config.yaml`: general settings. Set stereo mode, and lower the feature count for Pi performance.
- `kalibr_imucam_chain.yaml`: from step 5.
- `kalibr_imu_chain.yaml`: from step 4.

### 7e. Test

1. **Handheld first.** Walk the camera around a room and check the path in RViz.
2. Then mounted on the drone with **props off**, carrying it by hand.
3. Only then fly (see [safety rules](#15-flight-safety-rules)).

**Performance:** OpenVINS and YOLO share the Pi's CPU. Use 640×480 or lower for VIO, and limit features to roughly 150–200.

---

## 13. Step 8 — Send position to the flight controller

⏳ *Script coming:* `bridge/vio_to_mavlink.py`

It takes the OpenVINS pose and sends it to the flight controller as MAVLink `VISION_POSITION_ESTIMATE` / `ODOMETRY` over the telemetry port, so the FC can hold position without GPS.

### Enable the Pi 5 UART

```bash
sudo raspi-config
# Interface Options → Serial Port
#   Login shell over serial?  → No
#   Serial port hardware?     → Yes
sudo reboot
```
The port will be `/dev/ttyAMA0`.

### Flight controller settings

**Needs confirming:** does the PIXweser v1.0 run **ArduPilot** or **PX4**? The parameters are different.

If it's **ArduPilot**, the relevant parameters are roughly the following (the serial port number depends on which telemetry port is used):
```
SERIALx_PROTOCOL = 2      # MAVLink2
SERIALx_BAUD     = 921    # 921600
VISO_TYPE        = 1      # MAVLink external vision
EK3_SRC1_POSXY   = 6      # ExternalNav
EK3_SRC1_VELXY   = 6
EK3_SRC1_POSZ    = 1      # Baro (or 6)
EK3_SRC1_YAW     = 6
```
Double-check every parameter against the official docs for your firmware version before flying.

---

## 14. Step 9 — YOLO autonomy

⏳ *Coming:* `autonomy/` module.

### Make YOLO faster on the Pi

```bash
pip install ultralytics
yolo export model=yolo11n.pt format=ncnn    # NCNN is much faster on Pi CPU
```
Use the **nano** model and a small input size (e.g. 320–416). If it's still too slow, consider a Raspberry Pi AI HAT (Hailo).

### Basic logic (first version)

1. YOLO finds the target and gives a bounding box.
2. If the box is left or right of centre, yaw toward it.
3. If the box is small, move forward. If it's big enough, stop.
4. If the target is lost, hover in place.

Commands go to the FC as velocity setpoints in **GUIDED** mode (ArduPilot) or **OFFBOARD** mode (PX4).

---

## 15. Flight safety rules

- **Props OFF** for all indoor bench testing.
- A safety pilot holds the RC transmitter with a **mode switch back to manual / stabilize**, plus a **kill switch**, during every autonomous test.
- First flights: tethered, or in a netted area, at low altitude.
- Set a **geofence** and a **failsafe** (what the drone does if VIO or the link drops).
- Test new code **in the air only after it works handheld**.

---

## 16. Troubleshooting

| Problem | Fix |
|---|---|
| `Need 2 cameras, found 0/1` | Check ribbon cables (22-pin end in the Pi, blue side orientation), then the `dtoverlay` lines in [§5](#5-raspberry-pi-setup). Run `rpicam-hello --list-cameras`. |
| `ICM-20948 not found` | Run `i2cdetect -y 1`. If you see `69`, use `--imu-addr 0x69`. If nothing shows, check wiring and that I2C is enabled. |
| `No module named picamera2` | Install it with `sudo apt install python3-picamera2`, and make sure the venv was created with `--system-site-packages`. |
| `No module named smbus2` | Activate the venv, then `pip install -r requirements.txt`. |
| Window doesn't open over SSH | Use `--headless`, or connect with VNC. |
| Colours look blue/orange swapped | Change `"RGB888"` to `"BGR888"` in `sensors/stereo_camera.py`. |
| Low fps | Lower `--width/--height`. Check the Pi isn't throttling with `vcgencmd get_throttled` (should be `0x0`). Use the official 27W power supply. |

---

## 17. Reference repos

- [rpng/open_vins](https://github.com/rpng/open_vins): VIO software ([docs](https://docs.openvins.com/))
- [ethz-asl/kalibr](https://github.com/ethz-asl/kalibr): calibration ([wiki](https://github.com/ethz-asl/kalibr/wiki))
- [Lenna-Robotics-Research-Lab/Lenna-Stereo-Camera](https://github.com/Lenna-Robotics-Research-Lab/Lenna-Stereo-Camera): camera hardware (Jetson-based; we adapted it for Pi 5)

---

## Repo layout

```
sensors/icm20948.py       IMU driver (outputs m/s², rad/s)
sensors/stereo_camera.py  dual IMX219 capture on Pi 5 via Picamera2
test_sensors.py           step 1 sanity check
requirements.txt          pip packages (picamera2/opencv come from apt)
```

Planned:
```
record_dataset.py         step 2
config/calibration/       Kalibr outputs (steps 3, 5)
tools/allan_variance.py   step 4
tools/stereo_vo_check.py  step 6
ros2/stereo_imu_node/     step 7
config/openvins/          step 7
bridge/vio_to_mavlink.py  step 8
autonomy/                 step 9
```
