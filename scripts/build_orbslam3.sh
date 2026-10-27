#!/usr/bin/env bash
# Build Pangolin + ORB-SLAM3 on the Raspberry Pi 5 (Raspberry Pi OS / Debian 13).
#
#   bash scripts/build_orbslam3.sh
#
# Takes 1-2 hours on a Pi 5. Installs into ~/Pangolin and ~/ORB_SLAM3.
# ORB-SLAM3 is old code, so new compilers may hit errors this script doesn't fix yet.
# If it fails, send the LAST ~30 lines of output to the team.
set -e

JOBS=2   # more parallel jobs can run the Pi out of memory and crash the build

echo "=== 1/4  Dependencies ==="
sudo apt update
sudo apt install -y build-essential cmake git pkg-config \
  libeigen3-dev libopencv-dev libboost-serialization-dev libssl-dev \
  libepoxy-dev libglew-dev libgl1-mesa-dev libwayland-dev libxkbcommon-dev wayland-protocols

echo "=== 2/4  Pangolin (3D viewer used by ORB-SLAM3) ==="
cd ~
if [ ! -d Pangolin ]; then
  git clone --depth 1 https://github.com/stevenlovegrove/Pangolin.git
fi
cd Pangolin
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build build -j"$JOBS"
sudo cmake --install build
sudo ldconfig

echo "=== 3/4  ORB-SLAM3 source + fixes for new compilers ==="
cd ~
if [ ! -d ORB_SLAM3 ]; then
  git clone --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git
fi
cd ORB_SLAM3
# New Pangolin needs C++17 (ORB-SLAM3 asks for C++11). The COMPILEDWITHC11 macro name is untouched.
sed -i 's/++11/++17/g' CMakeLists.txt
# Limit parallel jobs so the Pi doesn't run out of RAM
sed -i "s/make -j[0-9]*/make -j$JOBS/g" build.sh
# C++17 forbids ++ on a bool (LoopClosing.cc: mnFullBAIdx++)
sed -i 's/bool mnFullBAIdx;/int mnFullBAIdx;/' include/LoopClosing.h
# Sophus' own tests fail under new GCC with -Werror (harmless warnings); don't treat them as errors
sed -i 's/ -Werror//' Thirdparty/Sophus/CMakeLists.txt

echo "=== 4/4  Build ORB-SLAM3 (the long part) ==="
chmod +x build.sh
./build.sh

echo
echo "=================================================="
echo "ORB-SLAM3 built. Check these exist:"
ls -la ~/ORB_SLAM3/Examples/Stereo/stereo_euroc ~/ORB_SLAM3/Examples/Stereo-Inertial/stereo_inertial_euroc
echo "=================================================="
