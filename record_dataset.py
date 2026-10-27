#!/usr/bin/env python3
"""Record stereo images + IMU in EuRoC format.

EuRoC is the folder layout ORB-SLAM3's examples read directly, and
tools/euroc_to_kalibr.sh converts it for Kalibr. Output:

  <out>/
    mav0/cam0/data/<timestamp_ns>.png   left  (grayscale)
    mav0/cam1/data/<timestamp_ns>.png   right (same filename as left)
    mav0/cam0/data.csv, mav0/cam1/data.csv
    mav0/imu0/data.csv                  timestamp, gyro xyz (rad/s), accel xyz (m/s^2)
    timestamps.txt                      one timestamp per line (ORB-SLAM3 needs this)
    stereo_gap.csv                      real left/right capture gap, for diagnostics

Examples:
  python3 record_dataset.py ~/datasets/static_cam --duration 90            # Kalibr camera calibration
  python3 record_dataset.py ~/datasets/dynamic --duration 90               # Kalibr camera-IMU calibration
  python3 record_dataset.py ~/datasets/imu_static --no-camera --duration 10800   # 3 h IMU noise
  python3 record_dataset.py ~/datasets/walk1 --duration 60                 # ORB-SLAM3 test run
Press Ctrl+C to stop early. Everything recorded up to that point is kept.
"""
import argparse
import os
import queue
import sys
import threading
import time

IMU_HEADER = ("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
              "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")


def pick_clock(sensor_ts_ns):
    """Find which Linux clock the camera timestamps use, so IMU stamps match."""
    mono = time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    boot = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    if abs(mono - sensor_ts_ns) <= abs(boot - sensor_ts_ns):
        return time.CLOCK_MONOTONIC, "MONOTONIC"
    return time.CLOCK_BOOTTIME, "BOOTTIME"


class ImuRecorder(threading.Thread):
    def __init__(self, imu, path, rate_hz, clock):
        super().__init__(daemon=True)
        self.imu, self.period, self.clock = imu, 1.0 / rate_hz, clock
        self.f = open(path, "w", buffering=1 << 16)
        self.f.write(IMU_HEADER)
        self.count = 0
        self.errors = 0
        self.running = True

    def run(self):
        next_t = time.monotonic()
        while self.running:
            try:
                accel, gyro = self.imu.read()
                t = time.clock_gettime_ns(self.clock)
                self.f.write(f"{t},{gyro[0]:.9f},{gyro[1]:.9f},{gyro[2]:.9f},"
                             f"{accel[0]:.9f},{accel[1]:.9f},{accel[2]:.9f}\n")
                self.count += 1
            except OSError:
                self.errors += 1  # flaky I2C read: skip this sample
            next_t += self.period
            time.sleep(max(0.0, next_t - time.monotonic()))
        self.f.close()


class ImageWriter(threading.Thread):
    """Writes PNGs in the background so capture never waits on the SD card."""

    def __init__(self, cv2):
        super().__init__(daemon=True)
        self.cv2 = cv2
        self.q = queue.Queue(maxsize=300)
        self.dropped = 0

    def put(self, path, img):
        try:
            self.q.put_nowait((path, img))
        except queue.Full:
            self.dropped += 1

    def run(self):
        while True:
            item = self.q.get()
            if item is None:
                break
            path, img = item
            self.cv2.imwrite(path, img, [self.cv2.IMWRITE_PNG_COMPRESSION, 1])


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("out", help="output folder (created if missing)")
    p.add_argument("--duration", type=float, default=0, help="seconds to record (0 = until Ctrl+C)")
    p.add_argument("--no-camera", action="store_true", help="IMU only (for the 3 h noise recording)")
    p.add_argument("--no-imu", action="store_true", help="cameras only")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=20)
    p.add_argument("--imu-rate", type=int, default=200)
    p.add_argument("--imu-addr", type=lambda s: int(s, 0), default=0x68)
    args = p.parse_args()

    if os.path.exists(os.path.join(args.out, "mav0")):
        sys.exit(f"{args.out} already has a recording. Pick a new folder name.")

    mav = os.path.join(args.out, "mav0")
    cam, imu, imu_rec, writer, cv2 = None, None, None, None, None
    clock, clock_name = time.CLOCK_MONOTONIC, "MONOTONIC"

    if not args.no_camera:
        import cv2  # noqa: F811
        from sensors.stereo_camera import StereoCamera
        for c in ("cam0", "cam1"):
            os.makedirs(os.path.join(mav, c, "data"), exist_ok=True)
        cam = StereoCamera(args.width, args.height, args.fps)
        _, _, t0, _ = cam.read()  # first frame: also tells us the camera clock
        clock, clock_name = pick_clock(t0)
        writer = ImageWriter(cv2)
        writer.start()

    if not args.no_imu:
        from sensors.icm20948 import ICM20948
        os.makedirs(os.path.join(mav, "imu0"), exist_ok=True)
        imu = ICM20948(address=args.imu_addr)
        imu_rec = ImuRecorder(imu, os.path.join(mav, "imu0", "data.csv"), args.imu_rate, clock)
        imu_rec.start()

    if cam is None and imu is None:
        sys.exit("Nothing to record (both --no-camera and --no-imu).")

    print(f"Recording to {args.out}  (clock: {clock_name})  Ctrl+C to stop")
    frames = 0
    start = time.monotonic()
    last_print = start
    stamps = []
    try:
        csv0 = open(os.path.join(mav, "cam0", "data.csv"), "w") if cam else None
        csv1 = open(os.path.join(mav, "cam1", "data.csv"), "w") if cam else None
        gap = open(os.path.join(args.out, "stereo_gap.csv"), "w") if cam else None
        if cam:
            for f in (csv0, csv1):
                f.write("#timestamp [ns],filename\n")
            gap.write("#left_ns,right_ns,gap_ms\n")

        while True:
            now = time.monotonic()
            if args.duration and now - start >= args.duration:
                break
            if cam:
                left, right, tl, tr = cam.read()
                name = f"{tl}.png"  # both images share the left timestamp (ORB-SLAM3 pairs by filename)
                writer.put(os.path.join(mav, "cam0", "data", name), cv2.cvtColor(left, cv2.COLOR_BGR2GRAY))
                writer.put(os.path.join(mav, "cam1", "data", name), cv2.cvtColor(right, cv2.COLOR_BGR2GRAY))
                csv0.write(f"{tl},{name}\n")
                csv1.write(f"{tl},{name}\n")
                gap.write(f"{tl},{tr},{(tr - tl) / 1e6:.3f}\n")
                stamps.append(tl)
                frames += 1
            else:
                time.sleep(0.2)

            if now - last_print >= 2.0:
                msg = [f"{now - start:7.1f}s"]
                if cam:
                    msg.append(f"frames {frames}  queue {writer.q.qsize()}  dropped {writer.dropped}")
                if imu_rec:
                    msg.append(f"imu {imu_rec.count}  i2c errors {imu_rec.errors}")
                print("  ".join(msg))
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        if imu_rec:
            imu_rec.running = False
            imu_rec.join(timeout=2)
            imu.close()
        if cam:
            cam.close()
            for f in (csv0, csv1, gap):
                f.close()
            print("Saving remaining images...")
            writer.q.put(None)
            writer.join()
            with open(os.path.join(args.out, "timestamps.txt"), "w") as f:
                f.writelines(f"{t}\n" for t in stamps)

    print(f"Done. {frames} stereo frames" + (f", {imu_rec.count} IMU samples" if imu_rec else "") + f" in {args.out}")
    if writer and writer.dropped:
        print(f"WARNING: {writer.dropped} images dropped (SD card too slow). Try --fps 15.")
    if imu_rec and imu_rec.errors:
        print(f"WARNING: {imu_rec.errors} IMU reads failed. Check the SDA/SCL wires.")


if __name__ == "__main__":
    main()
