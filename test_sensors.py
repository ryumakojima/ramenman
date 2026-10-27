#!/usr/bin/env python3
"""Step 1 sanity check: live stereo images + live IMU numbers on the Raspberry Pi 5.

Usage:
  python3 test_sensors.py              # cameras + IMU, shows a window (needs a monitor / VNC)
  python3 test_sensors.py --headless   # no window, just prints stats (works over SSH)
  python3 test_sensors.py --no-camera  # IMU only
  python3 test_sensors.py --no-imu     # cameras only

What "working" looks like:
  - Both camera images appear side by side.
  - With the board sitting still, |accel| is about 9.8 m/s^2 and gyro is near 0.
  - Rotate the board and the gyro numbers jump; tilt it and accel x/y/z shift.
Press q in the window (or Ctrl+C) to quit.
"""
import argparse
import math
import threading
import time


class ImuReader(threading.Thread):
    """Reads the IMU in the background at a fixed rate and keeps the latest sample."""

    def __init__(self, imu, rate_hz=200):
        super().__init__(daemon=True)
        self.imu = imu
        self.period = 1.0 / rate_hz
        self.lock = threading.Lock()
        self.latest = None  # (stamp_ns, accel_xyz, gyro_xyz)
        self.count = 0
        self.running = True

    def run(self):
        next_t = time.monotonic()
        while self.running:
            accel, gyro = self.imu.read()
            stamp = time.monotonic_ns()
            with self.lock:
                self.latest = (stamp, accel, gyro)
                self.count += 1
            next_t += self.period
            time.sleep(max(0.0, next_t - time.monotonic()))

    def snapshot(self):
        with self.lock:
            return self.latest, self.count


def fmt3(v):
    return "[" + ", ".join(f"{x:+7.3f}" for x in v) + "]"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--no-camera", action="store_true")
    p.add_argument("--no-imu", action="store_true")
    p.add_argument("--headless", action="store_true", help="don't open a window")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--imu-bus", type=int, default=1)
    p.add_argument("--imu-addr", type=lambda s: int(s, 0), default=0x68)
    p.add_argument("--imu-rate", type=int, default=200, help="IMU read rate in Hz")
    args = p.parse_args()

    cam = imu = reader = None
    cv2 = np = None

    if not args.no_camera:
        import cv2  # noqa: F811
        import numpy as np  # noqa: F811
        from sensors.stereo_camera import StereoCamera

        cam = StereoCamera(args.width, args.height, args.fps)
        print(f"Cameras OK ({args.width}x{args.height} @ {args.fps} fps)")

    if not args.no_imu:
        from sensors.icm20948 import ICM20948

        imu = ICM20948(bus=args.imu_bus, address=args.imu_addr)
        reader = ImuReader(imu, args.imu_rate)
        reader.start()
        print(f"IMU OK (ICM-20948 at 0x{args.imu_addr:02X})")

    if cam is None and imu is None:
        print("Nothing to test (both --no-camera and --no-imu given).")
        return

    frames = 0
    last_frames = 0
    last_imu_count = 0
    last_print = time.monotonic()
    lr_gap_ms = 0.0

    try:
        while True:
            if cam is not None:
                left, right, t_left, t_right = cam.read()
                frames += 1
                lr_gap_ms = abs(t_left - t_right) / 1e6

                if not args.headless:
                    view = np.hstack((left, right))
                    cv2.putText(view, f"L/R gap: {lr_gap_ms:.1f} ms", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    if reader is not None:
                        sample, _ = reader.snapshot()
                        if sample:
                            _, a, g = sample
                            cv2.putText(view, f"acc {fmt3(a)} m/s2", (10, 55),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                            cv2.putText(view, f"gyr {fmt3(g)} rad/s", (10, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    cv2.imshow("left | right", view)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
            else:
                time.sleep(0.02)

            now = time.monotonic()
            if now - last_print >= 1.0:
                dt = now - last_print
                parts = []
                if cam is not None:
                    parts.append(f"cam {(frames - last_frames) / dt:5.1f} fps | L/R gap {lr_gap_ms:5.1f} ms")
                    last_frames = frames
                if reader is not None:
                    sample, count = reader.snapshot()
                    parts.append(f"imu {(count - last_imu_count) / dt:6.1f} Hz")
                    last_imu_count = count
                    if sample:
                        _, a, g = sample
                        parts.append(f"acc {fmt3(a)} |a|={math.sqrt(sum(x * x for x in a)):5.2f}")
                        parts.append(f"gyr {fmt3(g)}")
                print("  ".join(parts))
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        if reader is not None:
            reader.running = False
            reader.join(timeout=1)
        if imu is not None:
            imu.close()
        if cam is not None:
            cam.close()
        if cv2 is not None and not args.headless:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
