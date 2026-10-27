#!/usr/bin/env python3
"""Compute IMU noise parameters from a long static recording (Allan deviation).

Input:  a folder recorded with  record_dataset.py <folder> --no-camera --duration 10800
Output: prints the 4 noise numbers and writes  <folder>/imu.yaml  (Kalibr format),
        with values already inflated by --inflate (default x5) for real-world use.

  python3 tools/allan_variance.py ~/datasets/imu_static
  python3 tools/allan_variance.py ~/datasets/imu_static --plot   # also saves allan.png
"""
import argparse
import os
import sys

import numpy as np


def allan_deviation(rate, fs, n_taus=150):
    """Overlapping Allan deviation of rate data (one axis)."""
    n = len(rate)
    theta = np.cumsum(rate) / fs
    m = np.unique(np.logspace(0, np.log10(n / 9), n_taus).astype(int))
    taus = m / fs
    adev = np.empty(len(m))
    for i, mi in enumerate(m):
        d = theta[2 * mi:] - 2 * theta[mi:-mi] + theta[:-2 * mi]
        adev[i] = np.sqrt(np.sum(d ** 2) / (2 * taus[i] ** 2 * (n - 2 * mi)))
    return taus, adev


def line_value(taus, adev, slope, tau_eval):
    """Fit a line of given log-log slope where the curve matches it best, evaluate at tau_eval."""
    lt, la = np.log10(taus), np.log10(adev)
    d = np.gradient(la, lt)
    i = int(np.argmin(np.abs(d - slope)))
    b = la[i] - slope * lt[i]
    return 10 ** (slope * np.log10(tau_eval) + b), abs(d[i] - slope) < 0.15


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("folder")
    p.add_argument("--inflate", type=float, default=5.0, help="safety multiplier for real flight (default 5)")
    p.add_argument("--plot", action="store_true")
    args = p.parse_args()

    path = os.path.join(args.folder, "mav0", "imu0", "data.csv")
    data = np.loadtxt(path, delimiter=",", comments="#")
    t = data[:, 0] * 1e-9
    fs = 1.0 / np.median(np.diff(t))
    hours = (t[-1] - t[0]) / 3600
    print(f"{len(t)} samples, {hours:.2f} h, {fs:.1f} Hz")
    if hours < 1:
        print("WARNING: less than 1 hour of data. Random-walk values will be unreliable; record 2-3 h.")

    results = {}
    curves = {}
    for name, cols in (("gyro", (1, 2, 3)), ("accel", (4, 5, 6))):
        N, K, okN, okK = [], [], True, True
        for c in cols:
            taus, adev = allan_deviation(data[:, c], fs)
            curves[f"{name}{c}"] = (taus, adev)
            n_val, n_ok = line_value(taus, adev, -0.5, 1.0)
            k_val, k_ok = line_value(taus, adev, +0.5, 3.0)
            N.append(n_val)
            K.append(k_val)
            okN &= n_ok
            okK &= k_ok
        results[name] = (float(np.mean(N)), float(np.mean(K)), okN, okK)

    gN, gK, gokN, gokK = results["gyro"]
    aN, aK, aokN, aokK = results["accel"]
    print("\nMeasured (continuous-time):")
    print(f"  gyroscope_noise_density     {gN:.3e} rad/s/sqrt(Hz)")
    print(f"  gyroscope_random_walk       {gK:.3e} rad/s^2/sqrt(Hz)" + ("" if gokK else "   (weak fit, record longer)"))
    print(f"  accelerometer_noise_density {aN:.3e} m/s^2/sqrt(Hz)")
    print(f"  accelerometer_random_walk   {aK:.3e} m/s^3/sqrt(Hz)" + ("" if aokK else "   (weak fit, record longer)"))

    k = args.inflate
    out = os.path.join(args.folder, "imu.yaml")
    with open(out, "w") as f:
        f.write(f"# Allan variance from {hours:.2f} h static recording, values x{k:g}\n")
        f.write("rostopic: /imu0\n")
        f.write(f"update_rate: {fs:.1f}\n")
        f.write(f"accelerometer_noise_density: {aN * k:.6e}\n")
        f.write(f"accelerometer_random_walk: {aK * k:.6e}\n")
        f.write(f"gyroscope_noise_density: {gN * k:.6e}\n")
        f.write(f"gyroscope_random_walk: {gK * k:.6e}\n")
    print(f"\nWrote {out} (x{k:g} inflated). Use it for Kalibr and tools/kalibr_to_orbslam3.py.")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(11, 4))
        for key, (taus, adev) in curves.items():
            ax[0 if key.startswith("gyro") else 1].loglog(taus, adev, label=key)
        ax[0].set_title("Gyro Allan deviation [rad/s]")
        ax[1].set_title("Accel Allan deviation [m/s^2]")
        for a in ax:
            a.set_xlabel("tau [s]")
            a.grid(True, which="both", alpha=0.3)
        png = os.path.join(args.folder, "allan.png")
        fig.tight_layout()
        fig.savefig(png, dpi=120)
        print(f"Saved {png}")


if __name__ == "__main__":
    sys.exit(main())
