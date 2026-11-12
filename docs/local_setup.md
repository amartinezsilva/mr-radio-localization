# Running locally (without Docker)

This is the native alternative to the [Docker images](../README.md#running-with-docker-recommended).

The localization stack and the simulator are built side by side in one ROS 2 workspace. Steps 1-3 are enough to build this repository and run it on recorded bags (including the real-world dataset); step 4 is only needed for the SITL simulation.

**1. Install the dependencies.** Install the packages and libraries listed in [Basic Dependencies](../README.md#basic-dependencies), following the setup instructions in each of their repositories. Ceres, Sophus and small_gicp are system libraries; `eliko_ros`, `ars548_ros` and `4D-Radar-Odom` are ROS 2 packages that go into your workspace in the next step.

**2. Clone this repository and the ROS 2 package dependencies into your workspace:**

```bash
mkdir -p <ros_ws>/src && cd <ros_ws>/src
git clone https://github.com/amartinezsilva/mr-radio-localization.git
git clone https://github.com/robotics-upo/eliko_ros.git
git clone https://github.com/robotics-upo/ars548_ros.git    # on ROS 2 Jazzy, add: -b jazzy
git clone -b arco-drone-integration https://github.com/robotics-upo/4D-Radar-Odom.git
```

**3. Build the workspace:**

```bash
cd <ros_ws>
colcon build
source install/setup.bash
```

You can now run the localization on recorded bags, including the real-world dataset (see [Running the real-world dataset](dataset.md)).

**4. (Optional) Set up the PX4 / Gazebo SITL simulator.** Clone UWBPX4Sim into the **same** workspace, next to this repository:

```bash
cd <ros_ws>/src
git clone https://github.com/amartinezsilva/UWBPX4Sim.git
```

Then follow its [Running without Docker](https://github.com/amartinezsilva/UWBPX4Sim/blob/main/docs/advanced_setup.md#running-without-docker) guide. That guide covers installing Gazebo Harmonic, PX4 and the Micro XRCE-DDS Agent, adding `px4_msgs`/`px4_ros_com` to the workspace, running `setup_simulator.sh`, and rebuilding the workspace with `colcon build`. `eliko_ros` is already in the workspace from step 2.

Once it's set up, run the simulation and the localization in two terminals:

```bash
# Terminal 1: PX4 SITL + Gazebo + UWB bridge/offboard nodes
cd <ros_ws>/src/UWBPX4Sim
./simulator_launcher.sh

# Terminal 2: relative localization
source <ros_ws>/install/setup.bash
ros2 launch uwb_localization localization.launch.py
```

`params_sim.yaml` has to match the layout you simulate (topics, anchor/tag IDs and positions). See [Launch files](../README.md#launch-files).

**Disclaimer**: the PX4 SITL simulation has been tested with ROS 2 Jazzy only, while the rest of the implementation has been tested in both ROS 2 Humble and ROS 2 Jazzy.

**Sharing one UWBPX4Sim checkout between both setups:** `docker/setup.sh` uses `<ros_ws>/src/UWBPX4Sim` by default, so the local and Docker setups can use the same checkout. `setup_simulator.sh` writes paths for the environment it ran in to `UWBPX4Sim/.setup_env`, so re-run it (option 1 in the container menu, or `./setup_simulator.sh` locally) whenever you switch between the two.

## Optional: GTSAM backend

The pose graph can also be solved with [GTSAM](https://github.com/borglab/gtsam) (`posegraph_backend: "gtsam"`, see the [parameter reference](parameters.md#pose-graph-optimization-pose_optimization_node)). It needs GTSAM **4.3 or newer**, built from source: the ROS binary package `ros-$ROS_DISTRO-gtsam` is 4.2, too old. Build it with the system Eigen, so it matches Ceres, then rebuild the workspace:

```bash
git clone --depth 1 -b 4.3.0 https://github.com/borglab/gtsam.git && cd gtsam
cmake -B build -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF
cmake --build build -j4 && sudo cmake --install build && sudo ldconfig
```

Without it, only the Ceres backend is built. The Docker images include it.

## Troubleshooting

* **CMake can't find `Sophus` when building ``uwb_localization``.** Install Sophus through the ROS package (`rosdep install --from-paths src --ignore-src -y` from your workspace, or `sudo apt install ros-<distro>-sophus`). That's what the Docker images use on Jazzy, and it builds as-is. If CMake still can't find it, which has been reported with other package managers such as `vcpkg`, install Sophus from source and build the workspace again:
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake ..
sudo make install
```

* **SITL: the PX4 topics don't show up in ROS 2** (Micro XRCE-DDS Agent not connecting, Ubuntu 24.04 / ROS 2 Jazzy). This is a simulator setup issue, and the workaround is documented in [UWBPX4Sim's troubleshooting](https://github.com/amartinezsilva/UWBPX4Sim/blob/main/docs/advanced_setup.md#troubleshooting).

If you'd rather avoid these potential issues entirely, see [Running with Docker](../README.md#running-with-docker-recommended) for a pre-built, known-working environment.
