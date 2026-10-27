#!/usr/bin/env bash
# Convert a record_dataset.py recording (EuRoC layout) into the flat folder
# layout that Kalibr's kalibr_bagcreater expects:
#   <out>/cam0/<ts>.png  <out>/cam1/<ts>.png  <out>/imu0.csv
#
# Usage: bash tools/euroc_to_kalibr.sh ~/datasets/static_cam ~/calib_data/static_cam
set -e
src="$1"; out="$2"
if [ -z "$src" ] || [ -z "$out" ]; then
  echo "Usage: $0 <recording_folder> <output_folder>"; exit 1
fi
mkdir -p "$out"
for c in cam0 cam1; do
  if [ -d "$src/mav0/$c/data" ]; then
    mkdir -p "$out/$c"
    find "$src/mav0/$c/data" -name "*.png" -exec cp -t "$out/$c/" {} +
  fi
done
if [ -f "$src/mav0/imu0/data.csv" ]; then
  cp "$src/mav0/imu0/data.csv" "$out/imu0.csv"
fi
echo "Kalibr folder ready: $out"
ls "$out"
