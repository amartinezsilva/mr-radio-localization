# Builds FROM the UWBPX4Sim base image (see UWBPX4Sim/docker/Dockerfile),
# which provides PX4 SITL, Gazebo Harmonic, and the UWB plugin/ROS 2 bridge
# already built under ~/ros2_ws. This Dockerfile adds only the
# radar/localization-specific dependencies and mr-radio-localization's own
# ROS 2 packages (uwb_localization, uwb_simulator) as siblings in that same
# workspace -- colcon builds them incrementally, so rebuilding after a
# localization-only code change never touches the PX4/Gazebo layers.
#
# `uwbpx4sim:jazzy` must already exist locally -- this repo's own
# docker/setup.sh builds it first automatically if it's missing (see that
# script's comments), rather than this Dockerfile trying to build or pull
# it itself.
FROM uwbpx4sim:jazzy

# The base image ends as USER jazzy -- switch back to root for the
# privileged installs below, same as the base Dockerfile's own structure.
USER root

# small_gicp: point-cloud registration used by uwb_localization's radar
# pre-processing. Not a rosdep-resolvable system package (no rosdep key --
# it's a plain third-party CMake project, not distributed via apt), so
# unlike everything else below it's built from source explicitly rather
# than left to rosdep. Not used by UWBPX4Sim itself, so it doesn't belong
# in the base image.
RUN cd /root/ && git clone https://github.com/koide3/small_gicp
RUN cd /root/small_gicp && mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j4 && make install

# ars548_ros (real ARS548 4D radar ROS 2 driver) and 4D-Radar-Odom (radar
# ego-motion odometry, ROS 2 package name `radar_odom` -- referenced by
# uwb_localization/launch/localization_dataset.launch.py) -- both change
# rarely upstream, so no CACHEBUST needed for these two specifically.
RUN cd /home/jazzy/ros2_ws/src && \
    git clone -b jazzy https://github.com/robotics-upo/ars548_ros.git
RUN cd /home/jazzy/ros2_ws/src && \
    git clone -b arco-drone-integration https://github.com/robotics-upo/4D-Radar-Odom.git

# Docker only invalidates a `RUN git clone` layer when this instruction's own
# text changes, not when the remote it clones actually moves -- so a normal
# cached rebuild would silently keep serving a stale checkout even after new
# commits land upstream. CACHEBUST forces a fresh clone on every build that
# passes a new value (see setup.sh, which passes the current timestamp)
# without invalidating the layers above (including ars548_ros/4D-Radar-Odom,
# which change far less often than this repo does during development).
ARG CACHEBUST=1
RUN cd /home/jazzy/ros2_ws/src && \
    git clone https://github.com/amartinezsilva/mr-radio-localization
SHELL ["/bin/bash", "-c"]

# rosdep, not a hand-picked apt list: each of the three packages just
# cloned declares its own system dependencies in its package.xml (Sophus,
# pcl_ros, libtclap-dev, ...), and letting rosdep resolve those is far
# less error-prone than guessing/maintaining that list by hand here --
# confirmed the hard way, this Dockerfile originally hand-listed packages
# and missed libtclap-dev (ars548_driver's actual build dependency),
# caught only by a real build failure.
#
# Five keys skipped -- checked directly with `rosdep resolve <key>`
# (not guessed) rather than iterating through more failed builds:
#  - small_gicp: a plain third-party CMake project, no rosdep entry at
#    all -- already built from source above.
#  - ceres, ceres-solver (used by uwb_localization's and radar_odom's
#    package.xml respectively -- different names for the same missing
#    thing) and OpenMP: genuinely absent from the rosdep database under
#    any casing, not just non-standard names for otherwise-resolvable
#    keys. Their actual packages (libceres-dev, libomp-dev) are installed
#    explicitly below instead.
#  - Eigen3 (radar_odom's package.xml): the registered key is lowercase
#    `eigen`, which resolves to libeigen3-dev -- already installed by the
#    base image's own Dockerfile, so nothing extra needed here, just skip
#    the mismatched-case key.
RUN apt-get update && apt-get install -y libceres-dev libomp-dev
RUN rosdep update && \
    rosdep install --from-paths /home/jazzy/ros2_ws/src --ignore-src -y \
      --skip-keys "small_gicp ceres ceres-solver OpenMP Eigen3"

RUN chown -R jazzy:jazzy /home/jazzy/ros2_ws/src

# Builds incrementally: UWBPX4Sim's packages are already built (from the
# base image's own colcon build layer) and won't be redone -- only
# uwb_localization, uwb_simulator, and the two radar packages just cloned
# above are new.
RUN cd /home/jazzy/ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install && \
    chown -R jazzy:jazzy /home/jazzy/ros2_ws

# Guided entrypoint -- extends UWBPX4Sim's (sources its function library,
# adds a "Launch localization" menu option). Overwrites the base image's
# thin wrapper at the same path; /uwbpx4sim-entrypoint-lib.sh (from the
# base image) is left in place and sourced from here.
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && chown jazzy:jazzy /entrypoint.sh

USER jazzy

# HOME/WORKDIR/PX4_DIR are already set correctly by the base image.
ENTRYPOINT ["/entrypoint.sh"]
CMD ["menu"]
