#!/usr/bin/env python3
"""Turn Kalibr results into an ORB-SLAM3 settings file.

  # stereo only (VO) — camera calibration from kalibr_calibrate_cameras
  python3 tools/kalibr_to_orbslam3.py --camchain camchain-static_cam.yaml \\
      --out config/orbslam3/ours.yaml

  # stereo + IMU (VIO) — camera-IMU calibration + noise model
  python3 tools/kalibr_to_orbslam3.py --camchain camchain-imucam-dynamic.yaml \\
      --imu imu.yaml --out config/orbslam3/ours.yaml

Conventions:
  Kalibr  cam1.T_cn_cnm1 : maps cam0 points -> cam1      ORB-SLAM3 Stereo.T_c1_c2 = inverse of it
  Kalibr  cam0.T_cam_imu : maps IMU points  -> cam0      ORB-SLAM3 IMU.T_b_c1     = inverse of it
"""
import argparse

import numpy as np
import yaml


def mat(m):
    rows = ",\n         ".join(", ".join(f"{v:.9f}" for v in row) for row in m)
    return f"!!opencv-matrix\n  rows: 4\n  cols: 4\n  dt: f\n  data: [{rows}]"


def cam_block(i, cam):
    if cam.get("camera_model") != "pinhole" or cam.get("distortion_model") != "radtan":
        raise SystemExit(f"cam{i - 1}: expected pinhole-radtan, got "
                         f"{cam.get('camera_model')}-{cam.get('distortion_model')}")
    fx, fy, cx, cy = cam["intrinsics"]
    k1, k2, p1, p2 = cam["distortion_coeffs"]
    return (f"Camera{i}.fx: {fx:.6f}\nCamera{i}.fy: {fy:.6f}\nCamera{i}.cx: {cx:.6f}\nCamera{i}.cy: {cy:.6f}\n"
            f"Camera{i}.k1: {k1:.9f}\nCamera{i}.k2: {k2:.9f}\nCamera{i}.p1: {p1:.9f}\nCamera{i}.p2: {p2:.9f}\n")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--camchain", required=True)
    p.add_argument("--imu", help="imu.yaml (from tools/allan_variance.py). Omit for stereo-only.")
    p.add_argument("--out", required=True)
    p.add_argument("--fps", type=float, default=20)
    p.add_argument("--features", type=int, default=1000, help="ORB features per image (lower = faster)")
    args = p.parse_args()

    cc = yaml.safe_load(open(args.camchain))
    cam0, cam1 = cc["cam0"], cc["cam1"]
    w, h = cam0["resolution"]
    T_c1_c2 = np.linalg.inv(np.array(cam1["T_cn_cnm1"]))
    baseline = np.linalg.norm(T_c1_c2[:3, 3])

    s = ["%YAML:1.0", "", 'File.version: "1.0"', "", 'Camera.type: "PinHole"', ""]
    s.append(cam_block(1, cam0))
    s.append(cam_block(2, cam1))
    s += [f"Camera.width: {w}", f"Camera.height: {h}", f"Camera.fps: {args.fps:g}", "Camera.RGB: 0", "",
          "Stereo.ThDepth: 60.0", f"Stereo.T_c1_c2: {mat(T_c1_c2)}", ""]

    if args.imu:
        if "T_cam_imu" not in cam0:
            raise SystemExit("--imu given but camchain has no T_cam_imu. Use camchain-imucam-*.yaml.")
        imu = yaml.safe_load(open(args.imu))
        T_b_c1 = np.linalg.inv(np.array(cam0["T_cam_imu"]))
        s += [f"IMU.T_b_c1: {mat(T_b_c1)}", "", "IMU.InsertKFsWhenLost: 0",
              f"IMU.NoiseGyro: {imu['gyroscope_noise_density']:.6e}",
              f"IMU.NoiseAcc: {imu['accelerometer_noise_density']:.6e}",
              f"IMU.GyroWalk: {imu['gyroscope_random_walk']:.6e}",
              f"IMU.AccWalk: {imu['accelerometer_random_walk']:.6e}",
              f"IMU.Frequency: {float(imu['update_rate']):.1f}", ""]

    s += [f"ORBextractor.nFeatures: {args.features}", "ORBextractor.scaleFactor: 1.2", "ORBextractor.nLevels: 8",
          "ORBextractor.iniThFAST: 20", "ORBextractor.minThFAST: 7", "",
          "Viewer.KeyFrameSize: 0.05", "Viewer.KeyFrameLineWidth: 1.0", "Viewer.GraphLineWidth: 0.9",
          "Viewer.PointSize: 2.0", "Viewer.CameraSize: 0.08", "Viewer.CameraLineWidth: 3.0",
          "Viewer.ViewpointX: 0.0", "Viewer.ViewpointY: -0.7", "Viewer.ViewpointZ: -3.5",
          "Viewer.ViewpointF: 500.0", ""]

    with open(args.out, "w") as f:
        f.write("\n".join(s))
    print(f"Wrote {args.out}  ({w}x{h}, baseline {baseline * 100:.1f} cm"
          + (", with IMU)" if args.imu else ", stereo only)"))
    if not 0.02 < baseline < 0.2:
        print("WARNING: stereo baseline looks wrong. Measure the lens spacing with a ruler and compare.")


if __name__ == "__main__":
    main()
