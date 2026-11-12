#!/bin/bash
#
# install_localization_deps.sh - The localization stack's dependencies,
# shared by both images so their dependency lists can't drift apart:
#   Dockerfile      (standalone: FROM osrf/ros:jazzy-desktop, no simulator)
#   Dockerfile.sim  (FROM uwbpx4sim:jazzy, the PX4/Gazebo SITL image)
#
# Runs as root at image build time, in two phases, because rosdep can only
# resolve keys once this repo's own sources are in the workspace -- and
# that copy of the checkout comes after the (slower, rarely changing)
# system dependencies, so editing the repo doesn't rebuild them:
#
#   install_localization_deps.sh system <ros_ws>   # apt + small_gicp + GTSAM + radar packages
#   install_localization_deps.sh rosdep <ros_ws>   # everything under <ros_ws>/src

set -euo pipefail

phase="${1:?usage: $0 <system|rosdep> <ros_ws>}"
ws="${2:?usage: $0 <system|rosdep> <ros_ws>}"

case "$phase" in
  system)
    apt-get update
    # libceres-dev/libomp-dev: rosdep has no entry for the `ceres`,
    # `ceres-solver` and `OpenMP` keys (see the rosdep phase below), so
    # they're installed explicitly here.
    apt-get install -y git build-essential cmake libeigen3-dev libceres-dev libomp-dev

    # small_gicp: point-cloud registration used by uwb_localization's radar
    # pre-processing. A plain third-party CMake project, not a ROS package.
    git clone https://github.com/koide3/small_gicp /root/small_gicp
    ( cd /root/small_gicp && mkdir build && cd build && \
      cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4 && make install )

    # GTSAM 4.3 (optional GTSAM pose-graph backends of uwb_localization).
    # Built from source: the ROS binary package (ros-$ROS_DISTRO-gtsam) is
    # 4.2. Uses the system Eigen, the same one Ceres uses. -j4 keeps memory
    # in check (GTSAM translation units need ~1 GB each).
    git clone --depth 1 -b 4.3.0 https://github.com/borglab/gtsam.git /root/gtsam
    cmake -S /root/gtsam -B /root/gtsam/build -DCMAKE_BUILD_TYPE=Release \
      -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF
    cmake --build /root/gtsam/build -j4 && cmake --install /root/gtsam/build && ldconfig
    rm -rf /root/gtsam

    # ars548_ros (ARS548 4D radar ROS 2 driver/messages) and 4D-Radar-Odom
    # (radar ego-motion odometry, ROS 2 package name `radar_odom` --
    # referenced by uwb_localization/launch/localization_dataset.launch.py).
    mkdir -p "$ws/src"
    git clone -b jazzy https://github.com/robotics-upo/ars548_ros.git "$ws/src/ars548_ros"
    git clone -b arco-drone-integration https://github.com/robotics-upo/4D-Radar-Odom.git "$ws/src/4D-Radar-Odom"
    ;;

  rosdep)
    # Keys skipped -- checked directly with `rosdep resolve <key>`:
    #  - small_gicp: no rosdep entry at all -- built from source above.
    #  - ceres, ceres-solver (uwb_localization's and radar_odom's package.xml)
    #    and OpenMP: absent from the rosdep database; libceres-dev and
    #    libomp-dev are installed explicitly above.
    #  - Eigen3 (radar_odom's package.xml): the registered key is lowercase
    #    `eigen` -- libeigen3-dev is installed explicitly above.
    apt-get update
    rosdep update
    rosdep install --from-paths "$ws/src" --ignore-src -y \
      --skip-keys "small_gicp ceres ceres-solver OpenMP Eigen3"
    ;;

  *)
    echo "Unknown phase: $phase (expected: system or rosdep)" >&2
    exit 1
    ;;
esac
