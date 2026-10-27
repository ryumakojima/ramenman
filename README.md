# ramenman — Autonomous Drone Software

Software for our autonomous drone. It runs on a **Raspberry Pi 5** with the **Waveshare IMX219-83 stereo camera** (2× IMX219 cameras + ICM-20948 IMU). **ORB-SLAM3** works out where the drone is from the cameras (and later the IMU too). **YOLO** spots targets and decides where to fly.

```
 Stereo camera ─┐
                ├─► ORB-SLAM3 ─► "where am I" ─► Flight controller (telemetry port, MAVLink)
 IMU ───────────┘                                         ▲
 Camera ─► YOLO ─► "what do I see" ─► decision logic ─────┘ "fly here"
```

---

## 🚀 Quick start (Raspberry Pi)

Open this page in the Pi's browser. Click the **copy button** on the top-right of each box, then paste it into the Pi's terminal with `Ctrl+Shift+V`.

**1. Download the code and set everything up** (takes a few minutes):
```bash
cd ~ && git clone https://github.com/ryumakojima/ramenman.git && cd ramenman && bash setup.sh
```

**2. Reboot** (first time only):
```bash
sudo reboot
```

**3. Test the cameras and IMU:**
```bash
cd ~/ramenman && source .venv/bin/activate && python3 test_sensors.py
```

**Get the latest code later:**
```bash
cd ~/ramenman && git pull
```

---

## Contents

1. [Concepts in plain words](#1-concepts-in-plain-words)
2. [The plan](#2-the-plan)
3. [Hardware and wiring](#3-hardware-and-wiring)
4. [Git basics for the team](#4-git-basics-for-the-team)
5. [Step 1 — Sensor test](#step-1--sensor-test) ✅
6. [Step 2 — Build ORB-SLAM3 on the Pi](#step-2--build-orb-slam3-on-the-pi)
7. [Step 3 — Print the calibration target](#step-3--print-the-calibration-target)
8. [Step 4 — Calibrate the cameras](#step-4--calibrate-the-cameras)
9. [Step 5 — Stereo VO with ORB-SLAM3](#step-5--stereo-vo-with-orb-slam3) ← **first big goal**
10. [Step 6 — IMU noise model](#step-6--imu-noise-model)
11. [Step 7 — Camera–IMU calibration](#step-7--cameraimu-calibration)
12. [Step 8 — Stereo-inertial VIO with ORB-SLAM3](#step-8--stereo-inertial-vio-with-orb-slam3) ← **second big goal**
13. [Step 9 — Live on the drone](#step-9--live-on-the-drone)
14. [Step 10 — Send position to the flight controller](#step-10--send-position-to-the-flight-controller)
15. [Step 11 — YOLO autonomy](#step-11--yolo-autonomy)
16. [Flight safety rules](#flight-safety-rules)
17. [Troubleshooting](#troubleshooting)
18. [Repo layout](#repo-layout)

---

## 1. Concepts in plain words

| Term | Meaning |
|---|---|
| **Odometry** | Estimating your own movement and position (自己位置推定). Indoors there's no GPS, so the drone has to do this itself. |
| **IMU** | Small sensor measuring acceleration (accelerometer) and rotation speed (gyroscope), like the one in a phone. |
| **Visual Odometry (VO)** | Camera only. Tracks how features in the image move between frames. Fails with motion blur, blank walls, or darkness. |
| **Inertial Odometry** | IMU only. Drifts by metres within seconds, so it's never used alone. |
| **VIO** | Visual-Inertial Odometry: camera + IMU combined. The IMU handles fast motion and the camera corrects IMU drift. This is what we fly with. |
| **SLAM** | Odometry that also builds a map and recognises places it has seen before, which corrects drift. |
| **ORB-SLAM3** | Open-source SLAM software. Its **Stereo** mode is our VO, and its **Stereo-Inertial** mode is our VIO. We configure it; we don't write it. |
| **Kalibr** | Calibration tool. Measures lens properties, how the two cameras sit relative to each other, and the camera ↔ IMU offset. Nothing works well without good calibration. |
| **EuRoC format** | A standard folder layout for recordings. ORB-SLAM3 reads it directly, so our recorder saves in this format. |
| **MAVLink** | The message language flight controllers speak. |

---

## 2. The plan

| Step | What | Where | Status |
|---|---|---|---|
| 1 | Sensor test (both cameras + IMU) | Pi | ✅ done |
| 2 | Build ORB-SLAM3 | Pi | ⏳ |
| 3 | Print calibration target | — | ⏳ |
| 4 | Camera calibration (Kalibr) | Pi records → laptop calibrates | ⏳ |
| 5 | **Stereo VO** (ORB-SLAM3 Stereo) on recordings | Pi | ⏳ |
| 6 | IMU noise model (3 h static recording) | Pi | ⏳ |
| 7 | Camera–IMU calibration (Kalibr) | Pi records → laptop calibrates | ⏳ |
| 8 | **Stereo-inertial VIO** (ORB-SLAM3) on recordings | Pi | ⏳ |
| 9 | Run ORB-SLAM3 live | Pi | ⏳ code coming |
| 10 | Position → flight controller (MAVLink) | Pi | ⏳ code coming |
| 11 | YOLO → flight commands | Pi | ⏳ code coming |

**Rule: don't skip ahead.** If calibration is bad, everything after it fails in confusing ways.

Steps 2–4 can happen in parallel. Step 6 is a 3-hour recording, so run it overnight whenever the Pi is free.

---

## 3. Hardware and wiring

- **Companion computer:** Raspberry Pi 5, running Raspberry Pi OS Trixie (Debian 13, 64-bit)
- **Camera:** Waveshare IMX219-83 stereo camera, 2× IMX219 + ICM-20948 IMU ([wiki](https://waveshare.com/wiki/IMX219-83_Stereo_Camera))
- **Flight controller:** PIXweser v1.0, through its telemetry port
- **Detection:** YOLO

### Cameras
Two ribbon cables go into the Pi 5's **CAM0** and **CAM1** ports. The Pi 5 uses the smaller 22-pin connector, so you need 22-pin → 15-pin cables.

### IMU (I2C)
The board's header has `SDA / SCL / INT / FSYNC`. **There are no VCC/GND pins**, because the IMU gets power and ground through the camera ribbon cables.

| Board pin | Pi 5 header pin |
|---|---|
| SDA | Pin 3 (GPIO2) |
| SCL | Pin 5 (GPIO3) |
| INT | not connected |
| FSYNC | not connected |

Use **short** jumper wires, pushed in fully. Loose wires make the IMU drop out intermittently.

### Flight controller (UART) — only needed from step 10

| FC telemetry | Pi 5 header pin |
|---|---|
| TX | Pin 10 (GPIO15, RX) |
| RX | Pin 8 (GPIO14, TX) |
| GND | Pin 6 or 14 (GND) |
| 5V | **Do NOT connect** |

TX and RX are crossed. Check the FC's telemetry pinout before wiring.

---

## 4. Git basics for the team

```bash
git clone https://github.com/ryumakojima/ramenman.git   # first time
cd ramenman
git pull                                                # every time before you start
git add . && git commit -m "what you changed" && git push   # after you change something
```

**Never commit recordings or build folders.** They're huge. `.gitignore` already blocks `datasets/`.

---

## Step 1 — Sensor test

```bash
cd ~/ramenman && source .venv/bin/activate
python3 test_sensors.py              # window with both images + IMU overlay
python3 test_sensors.py --headless   # over SSH: prints stats only
python3 test_sensors.py --no-camera  # IMU only
python3 test_sensors.py --no-imu     # cameras only
```

**Pass criteria**
- [ ] Both images show at around 30 fps.
- [ ] Cover the **left** lens and the **left** image goes dark. If it's reversed, swap the indices in `StereoCamera(left_index=1, right_index=0)` in `sensors/stereo_camera.py`.
- [ ] Board still: `|a|` ≈ **9.8**, gyro ≈ **0**.
- [ ] Rotating the board makes the gyro values move.
- [ ] IMU rate ≈ **200 Hz**.

---

## Step 2 — Build ORB-SLAM3 on the Pi

```bash
cd ~/ramenman && bash scripts/build_orbslam3.sh
```

This takes **1–2 hours**. Leave it running and keep the Pi on the official 27W power supply.

When it finishes, it should list two programs:
- `stereo_euroc` (VO)
- `stereo_inertial_euroc` (VIO)

> ORB-SLAM3 is old code and our OS is very new, so the build may hit compiler errors the script doesn't handle yet. If it fails, copy the **last ~30 lines** of the output and send them to the team or the software lead.

---

## Step 3 — Print the calibration target

1. Download the **AprilGrid 6x6** PDF from the [Kalibr wiki: calibration targets](https://github.com/ethz-asl/kalibr/wiki/calibration-targets).
2. Print it at **100% scale** (turn off "fit to page"), on A3 if possible.
3. Stick it on something **flat and rigid**, like foam board. A bent target ruins calibration.
4. **Measure one black tag** with a ruler and put the size (in metres) in `config/april_6x6.yaml` as `tagSize`.

---

## Step 4 — Calibrate the cameras

### 4a. Record (on the Pi)
```bash
cd ~/ramenman && source .venv/bin/activate
python3 record_dataset.py ~/datasets/static_cam --duration 90 --no-imu
```
While it records, **move the camera slowly** in front of the AprilGrid. Cover all parts of both images (corners too), tilt it at different angles, and vary the distance. Use good lighting and avoid blur.

### 4b. Copy to your laptop and convert
```bash
# on the Pi
bash tools/euroc_to_kalibr.sh ~/datasets/static_cam ~/calib/static_cam
# then copy ~/calib/static_cam and config/april_6x6.yaml to the laptop, e.g. with a USB stick or:
scp -r fastr1@<pi-ip>:~/calib ~/calib && scp fastr1@<pi-ip>:~/ramenman/config/april_6x6.yaml ~/calib/
```

### 4c. Run Kalibr (on the laptop, with Docker)
One-time setup:
```bash
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr && docker build -t kalibr -f Dockerfile_ros1_20_04 .
```
Each time:
```bash
docker run -it -v ~/calib:/data kalibr
```
Inside the container:
```bash
source devel/setup.bash
rosrun kalibr kalibr_bagcreater --folder /data/static_cam --output-bag /data/static_cam.bag
rosrun kalibr kalibr_calibrate_cameras \
  --bag /data/static_cam.bag --target /data/april_6x6.yaml \
  --models pinhole-radtan pinhole-radtan \
  --topics /cam0/image_raw /cam1/image_raw
```
Output: `camchain-static_cam.yaml` plus a PDF report in `~/calib`.

**Check the report:** reprojection error should be **under 0.5 px**. If it's worse, re-record with better light, slower motion, and a flatter target.

### 4d. Make the ORB-SLAM3 settings file (on the Pi)
Copy `camchain-static_cam.yaml` back to the Pi, then:
```bash
mkdir -p config/calibration && cp <path>/camchain-static_cam.yaml config/calibration/
python3 tools/kalibr_to_orbslam3.py --camchain config/calibration/camchain-static_cam.yaml \
  --out config/orbslam3/stereo.yaml
```
It prints the **stereo baseline**, the distance between the two lenses. Measure it with a ruler; the two should match within a few millimetres.

Commit `config/calibration/` and `config/orbslam3/` so the whole team has them.

---

## Step 5 — Stereo VO with ORB-SLAM3

**First big goal:** the drone knows where it is using the cameras alone.

### Record a test walk
```bash
python3 record_dataset.py ~/datasets/walk1 --duration 60 --no-imu
```
Walk slowly around a room with the camera, **ending where you started**. Rooms with posters, furniture, and texture work best. Blank walls don't.

### Run ORB-SLAM3 (on the Pi's monitor; it opens a 3D viewer)
```bash
~/ORB_SLAM3/Examples/Stereo/stereo_euroc \
  ~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/ramenman/config/orbslam3/stereo.yaml \
  ~/datasets/walk1 ~/datasets/walk1/timestamps.txt
```

**Pass criteria**
- [ ] It initialises (green points appear) within the first few seconds.
- [ ] The path in the viewer matches how you actually walked.
- [ ] It doesn't say "LOST" for long stretches.
- [ ] The end of the path lands near the start.

---

## Step 6 — IMU noise model

ORB-SLAM3 needs to know how noisy our specific IMU is.

```bash
# Put the board on a table where NOTHING will touch or bump it, then:
python3 record_dataset.py ~/datasets/imu_static --no-camera --duration 10800    # 3 hours
python3 tools/allan_variance.py ~/datasets/imu_static --plot
```
This writes `~/datasets/imu_static/imu.yaml` with the 4 noise numbers. They're already multiplied by 5 for real-world conditions (vibration, temperature). Copy it into the repo:
```bash
cp ~/datasets/imu_static/imu.yaml config/calibration/imu.yaml
```

---

## Step 7 — Camera–IMU calibration

### Record (on the Pi)
```bash
python3 record_dataset.py ~/datasets/dynamic --duration 90
```
Hold the camera in front of the AprilGrid and move it in **all 6 directions**: up/down, left/right, forward/back, plus roll, pitch, and yaw. Keep the grid in view and the motion smooth but lively, without blur.

### Convert, copy to the laptop, and run Kalibr
```bash
# Pi
bash tools/euroc_to_kalibr.sh ~/datasets/dynamic ~/calib/dynamic
cp config/calibration/imu.yaml config/calibration/camchain-static_cam.yaml ~/calib/
```
```bash
# laptop, inside the Kalibr container
rosrun kalibr kalibr_bagcreater --folder /data/dynamic --output-bag /data/dynamic.bag
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag /data/dynamic.bag --target /data/april_6x6.yaml \
  --cam /data/camchain-static_cam.yaml --imu /data/imu.yaml
```
Output: `camchain-imucam-dynamic.yaml`.

**Check:**
- `T_cam_imu` translation: where the camera is relative to the IMU chip. It should be within a few cm of what you'd measure with a ruler.
- `timeshift_cam_imu`: should be small (a few ms).

### Make the stereo-inertial settings file (on the Pi)
```bash
cp <path>/camchain-imucam-dynamic.yaml config/calibration/
python3 tools/kalibr_to_orbslam3.py \
  --camchain config/calibration/camchain-imucam-dynamic.yaml \
  --imu config/calibration/imu.yaml \
  --out config/orbslam3/stereo_inertial.yaml
```

---

## Step 8 — Stereo-inertial VIO with ORB-SLAM3

**Second big goal:** cameras + IMU together. This is what the drone flies with.

```bash
python3 record_dataset.py ~/datasets/walk2 --duration 60
```
At the very start, **move the camera around a bit** for 2–3 seconds (tilt and shake gently). ORB-SLAM3 needs IMU motion to initialise.

```bash
~/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc \
  ~/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/ramenman/config/orbslam3/stereo_inertial.yaml \
  ~/datasets/walk2 ~/datasets/walk2/timestamps.txt
```

**Pass criteria:** same as step 5, plus it should **survive fast turns** that made stereo-only VO get lost.

---

## Step 9 — Live on the drone

⏳ *Code coming.* Steps 5 and 8 run ORB-SLAM3 on recordings. For flight, it has to run on the live camera feed, which needs a small C++ program that feeds our cameras + IMU into ORB-SLAM3 directly. It will be added here once step 8 passes.

---

## Step 10 — Send position to the flight controller

⏳ *Code coming:* `bridge/`. It sends ORB-SLAM3's position to the FC as MAVLink `VISION_POSITION_ESTIMATE` over the telemetry port, so the FC can hold position without GPS.

### Enable the Pi 5 UART
```bash
sudo raspi-config
# Interface Options → Serial Port
#   Login shell over serial?  → No
#   Serial port hardware?     → Yes
sudo reboot
```
The port appears as `/dev/ttyAMA0`.

> **Needs confirming:** does the PIXweser v1.0 run **ArduPilot** or **PX4**? The FC settings depend on it.

---

## Step 11 — YOLO autonomy

⏳ *Code coming:* `autonomy/`.

Make YOLO fast enough on the Pi:
```bash
pip install ultralytics
yolo export model=yolo11n.pt format=ncnn    # NCNN runs much faster on the Pi CPU
```
Use the **nano** model and a small input size (320–416). ORB-SLAM3 and YOLO share the CPU.

First version of the logic:
1. YOLO finds the target and gives a bounding box.
2. If the box is left or right of centre, yaw toward it.
3. If the box is small, move forward. If it's big, stop.
4. If the target is lost, hover.

---

## Flight safety rules

- **Props OFF** for all indoor testing.
- A safety pilot holds the RC transmitter with a **mode switch back to manual**, plus a **kill switch**, during every autonomous test.
- First flights: tethered or netted, at low altitude.
- Set a **geofence** and a **failsafe** (what happens if tracking or the link is lost).
- New code must work **handheld** before it flies.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `Need 2 cameras, found 0/1` | Reseat the ribbon cables, then run `rpicam-hello --list-cameras`. If they still don't show, add `camera_auto_detect=0`, `dtoverlay=imx219,cam0`, and `dtoverlay=imx219,cam1` to `/boot/firmware/config.txt` and reboot. |
| IMU: `Errno 121` / `No I2C device answering` | Run `sudo i2cget -y 1 0x68 0x00`. It should print `0xea`. If it errors or only works sometimes, the wires are loose: use short wires, push them in fully, and reseat both ribbons (they power the IMU). |
| IMU works only sometimes | Slow the bus down: add `dtparam=i2c_arm_baudrate=10000` to `/boot/firmware/config.txt`, then reboot. |
| `i2cdetect` shows `69` not `68` | Add `--imu-addr 0x69` to the commands. |
| `No module named picamera2` | `sudo apt install python3-picamera2`, and the venv must be made with `--system-site-packages` (`setup.sh` does this). |
| Recorder says images were dropped | The SD card is too slow. Use `--fps 15` or a faster card. |
| Window doesn't open over SSH | Use `--headless`, or work on the Pi's monitor or VNC. |
| ORB-SLAM3 "LOST" all the time | Usually calibration (redo step 4), a textureless scene, motion that's too fast, or bad lighting. |
| Pi slow or crashing | Check for throttling with `vcgencmd get_throttled` (it should print `0x0`). Use the official 27W supply and a fan. |

---

## Repo layout

```
setup.sh                     one-shot Pi setup
test_sensors.py              step 1: live camera + IMU check
record_dataset.py            record stereo + IMU in EuRoC format (steps 4–8)
sensors/icm20948.py          IMU driver (m/s², rad/s)
sensors/stereo_camera.py     dual IMX219 capture on Pi 5 (Picamera2)
scripts/build_orbslam3.sh    step 2: build Pangolin + ORB-SLAM3
tools/euroc_to_kalibr.sh     convert a recording for Kalibr
tools/allan_variance.py      step 6: IMU noise numbers → imu.yaml
tools/kalibr_to_orbslam3.py  Kalibr results → ORB-SLAM3 settings file
config/april_6x6.yaml        calibration target (edit tagSize!)
config/calibration/          Kalibr outputs (commit these)
config/orbslam3/             ORB-SLAM3 settings (generated, commit these)
```

## Reference

- [UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- [ethz-asl/kalibr](https://github.com/ethz-asl/kalibr) ([wiki](https://github.com/ethz-asl/kalibr/wiki))
- [Waveshare IMX219-83 wiki](https://waveshare.com/wiki/IMX219-83_Stereo_Camera)
