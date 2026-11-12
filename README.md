
<h1 align="center"><a href="https://arxiv.org/abs/2509.26558" style="text-decoration:none;color:inherit;">Radio-based Multi-Robot Odometry and Relative Localization</a></h1>

<div align="center">
  <a href="https://www.youtube.com/watch?v=zTdIhjTZPeA"><img src="https://img.shields.io/badge/YouTube-Video-red?logo=youtube" alt="YouTube Video"></a>
  <a href="https://arxiv.org/abs/2509.26558"><img src="https://img.shields.io/badge/arXiv-Paper-b31b1b?logo=arxiv" alt="arXiv Paper"></a>
  </a>
</div>

❗ **This work has been accepted to ICRA 2026!**

### Abstract
Radio-based methods such as Ultra-Wideband (UWB) and RAdio Detection And Ranging (RADAR), which have traditionally seen limited adoption in robotics, are experiencing a boost in popularity thanks to their robustness to harsh environmental conditions and cluttered environments. This work proposes a multi-robot UGV-UAV localization system that leverages the two technologies with inexpensive and readily-available sensors (Inertial Measurement Units, or IMUs, and wheel encoders) to estimate the relative position of an aerial robot with respect to a ground robot. The first stage of the system pipeline includes a nonlinear optimization framework to trilaterate the location of the aerial platform based on UWB range data, and a RADAR pre-processing module with loosely coupled ego-motion estimation which has been adapted for a multi-robot scenario. Then, the pre-processed RADAR data as well as the relative transformation are fed to a pose-graph optimization framework with odometry and inter-robot constraints. The system, implemented for the Robotic Operating System (ROS 2) with the Ceres optimizer, has been validated in Software-in-the-Loop (SITL) simulations and in a real-world dataset. The relative localization module outperforms state-of-the-art closed-form methods which are less robust to noise. Our SITL environment includes a custom Gazebo plugin for generating realistic UWB measurements modeled after real data.  Conveniently, the proposed factor graph formulation makes the system readily extensible to full Simultaneous Localization And Mapping (SLAM).  Finally, all the code and experimental data have been made publicly available to support reproducibility and to serve as a common open dataset for benchmarking.

## Basic Dependencies

* Ubuntu 22.04 LTS and ROS 2 [Humble](https://docs.ros.org/en/humble/index.html) or Ubuntu 24.04 LTS and ROS 2 [Jazzy](https://docs.ros.org/en/jazzy/index.html)
* [Ceres Solver](https://github.com/ceres-solver/ceres-solver)
* [Sophus](https://github.com/strasdat/Sophus)
* [small_gicp](https://github.com/koide3/small_gicp)
* [pcl_ros](https://github.com/ros-perception/perception_pcl)
* [eliko_ros](https://github.com/robotics-upo/eliko_ros)
* [ars548_ros](https://github.com/robotics-upo/ars548_ros)
* [4D-Radar-Odom](https://github.com/robotics-upo/4D-Radar-Odom/tree/arco-drone-integration) branch ```arco_drone_integration```.
* Optional: [GTSAM](https://github.com/borglab/gtsam) **4.3 or newer**, for the GTSAM pose-graph backend. Build it from source (the ROS binary package `ros-$ROS_DISTRO-gtsam` is 4.2, too old), with the system Eigen so it matches Ceres:
  ```bash
  git clone --depth 1 -b 4.3.0 https://github.com/borglab/gtsam.git && cd gtsam
  cmake -B build -DCMAKE_BUILD_TYPE=Release -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF
  cmake --build build -j4 && sudo cmake --install build && sudo ldconfig
  ```
  Without it only the Ceres backend is built. The Docker images include it. Select the backend with `posegraph_backend` in the node's parameters.

Please follow the links above to the mentioned packages for specific setup instructions for each of them. 

## Main components

This repository contains one ROS 2 package:

* ```uwb_localization```: includes the UWB-based relative transformation estimation node and the pose-graph optimization node with radar constraints. The ```config``` folder in this package contains the parameter file for these two nodes, in the simulation variant without radar (``params_sim.yaml``) and the dataset variant with radar (``params_dataset.yaml``), using our setup-specific configuration. Every parameter is described in the [Parameter reference](#parameter-reference).

The PX4/Gazebo SITL simulator lives in its own repository, [UWBPX4Sim](https://github.com/amartinezsilva/UWBPX4Sim). It includes the UWB plugin, modified UAV and UGV models for PX4 SITL, the ROS 2 bridge/offboard package, and the simulator launcher. It is **not** a submodule of this repository: you only need it if you want to run the SITL simulation, not to build this repository or to run the localization on recorded bags (including the real-world dataset). The two communicate only through ROS 2 topics (see [Launch files](#launch-files)). Detailed setup, plugin parameters, and usage instructions are documented in [the ```UWBPX4Sim``` README](https://github.com/amartinezsilva/UWBPX4Sim/blob/main/README.md).

![](images/Combined_Diagram.drawio.png)

Both setups described below put the repositories side by side in the same ROS 2 workspace:

```
<ros_ws>/src/
├── mr-radio-localization/   # this repository
├── UWBPX4Sim/               # only needed for the SITL simulation
└── ...                      # the other dependencies (local setup only)
```

## Running with Docker (Recommended)

A `docker/` folder provides ready-to-use images (Ubuntu 24.04, ROS 2 Jazzy) as an alternative to setting everything up natively. It requires [Docker](https://docs.docker.com/engine/install/) and the [Docker Compose plugin](https://docs.docker.com/compose/install/). There are two variants:

| Variant | Command | What's inside | Use it for |
|---|---|---|---|
| **Standalone** (default) | `./setup.sh` | The localization stack only (`FROM osrf/ros:jazzy-desktop`). No simulator. A few minutes to build. | Running the localization on your own rosbags or on the [real-world dataset](#running-the-real-world-dataset). |
| **SITL** | `./setup.sh --sim` | PX4 SITL, Gazebo Harmonic and the UWB plugin, plus the localization stack. Built `FROM` the [UWBPX4Sim](https://github.com/amartinezsilva/UWBPX4Sim) image. 30-45+ minutes the first time. | Running the full simulation and localizing in it. |

Both images install the localization dependencies with the same script (`docker/install_localization_deps.sh`), so they always have the same localization stack. Both bind-mount `uwb_localization/bags/` and `uwb_localization/config/` from the host: drop your recorded rosbags into `bags/` and tune `params_sim.yaml`/`params_dataset.yaml` in `config/`, and they're picked up the next time you launch, without rebuilding the image.

### Standalone image

```bash
git clone https://github.com/amartinezsilva/mr-radio-localization.git
cd mr-radio-localization/docker
./setup.sh
```

No UWBPX4Sim needed. `setup.sh` builds the image, starts a **persistent** container in the background and drops you into its guided menu:

1. **Launch localization**: pick `localization.launch.py` or `localization_dataset.launch.py`, then a bag from `uwb_localization/bags/`. It runs in the foreground (`localization.launch.py` also opens RViz); press Ctrl-C to stop it and return to the menu. To try it right away, download the [real-world dataset](#running-the-real-world-dataset).
2. **Open a shell**: a plain shell in the container, with the workspace already sourced.
3. **Exit**: stops the container, and asks whether to remove it completely instead (see [Container lifecycle](#container-lifecycle)).

### SITL image

```bash
cd <ros_ws>/src
git clone https://github.com/amartinezsilva/mr-radio-localization.git
git clone https://github.com/amartinezsilva/UWBPX4Sim.git   # optional: setup.sh --sim offers to clone it if missing
cd mr-radio-localization/docker
./setup.sh --sim
```

This image is layered: it's built `FROM` the UWBPX4Sim simulator image (PX4 SITL, Gazebo, the UWB plugin -- see [its own README](https://github.com/amartinezsilva/UWBPX4Sim/blob/main/README.md#getting-started)), adding only this repo's radar/localization dependencies and ROS 2 package on top. You don't need to build that base image yourself first -- `./setup.sh --sim` builds it automatically (asking first) if it isn't already present locally.

`setup.sh --sim` looks for UWBPX4Sim next to this repository (`<ros_ws>/src/UWBPX4Sim`) and offers to clone it there if it's missing. To use a checkout somewhere else, point it there with `UWBPX4SIM_DIR=/path/to/UWBPX4Sim ./setup.sh --sim`. That checkout is used to build the base image and is bind-mounted into the container, so the simulator version you have checked out on the host is the one the container runs: layouts, plugin presets, worlds and trajectories edited on the host are picked up the next time you run setup (option 1). Under WSL2, `setup.sh` also adds `docker-compose.wsl.yml` automatically, so Gazebo renders on the GPU.

The first build compiles the UWBPX4Sim base image plus this repo's own workspace, so it can take a while (30-45+ minutes depending on your machine, mostly the one-time base image); subsequent runs reuse the cached images and start instantly, and rebuilding after a change to this repo's own code doesn't touch the PX4/Gazebo layers at all. The guided menu offers:

1. **Run UWBPX4Sim setup** (`setup_simulator.sh`): configure the PX4/Gazebo plugin, models, and layout inside the container. Opens a browser-based GUI (prints the URL, default `http://localhost:5050`) to build the robot layout and tune plugin parameters; falls back to terminal prompts if the GUI isn't available. Run this once, and again whenever you change the layout or plugin parameters.
2. **Launch simulation** (ticks once setup has been run): launches PX4 SITL, Gazebo, and the UWB bridge/offboard nodes via `simulator_launcher.sh`.
3. **Launch localization**: runs `ros2 launch uwb_localization localization.launch.py` in a new tmux window alongside the running simulation, to see the relative localization estimate. Needs the simulation (option 2) already running.
4. **Open a shell**: skip the menu and get a plain shell in the container.
5. **Exit**: stops the container, and asks whether to remove it completely instead (see [Container lifecycle](#container-lifecycle)).

### Container lifecycle

Both variants work the same way. The images and containers are named per variant (`mr-radio-localization:jazzy-standalone` / `mr-radio-localization-standalone`, and `mr-radio-localization:jazzy-sim` / `mr-radio-localization-sim`), so both can exist side by side.

The container keeps running in the background even after you close the terminal, with everything you configured (for SITL: the compiled PX4 binary, generated models, bridge config, `.setup_env`) still in place — closing your terminal is *not* the same as choosing "Exit". Run `./setup.sh` (or `./setup.sh --sim`) again any time, from any terminal, to reattach straight to the menu; it skips the build/launch prompt entirely once it finds the container already there.

Only "Exit" from the menu (or `docker stop <container>` from the host) stops it. Answering **N** (or just pressing Enter) at its removal prompt only stops it -- the container's filesystem is untouched, and `./setup.sh` (or `docker start`) resumes it exactly as it was. Answering **y** there removes it completely instead, equivalent to `docker rm -f <container>` -- the next `./setup.sh` run then creates a genuinely fresh one.

**Upgrading from an earlier version:** older versions of this repo had a single image, `mr-radio-localization:jazzy`, with a container called `mr-radio-localization`. The new scripts don't use those names, so you can remove them with `docker rm -f mr-radio-localization && docker rmi mr-radio-localization:jazzy`.

## Running locally (without Docker)

The localization stack and the simulator are built side by side in one ROS 2 workspace. Steps 1-3 are enough to build this repository and run it on recorded bags (including the real-world dataset); step 4 is only needed for the SITL simulation.

**1. Install the dependencies.** Install the packages and libraries listed in [Basic Dependencies](#basic-dependencies), following the setup instructions in each of their repositories. Ceres, Sophus and small_gicp are system libraries; `eliko_ros`, `ars548_ros` and `4D-Radar-Odom` are ROS 2 packages that go into your workspace in the next step.

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

You can now run the localization on recorded bags, including the real-world dataset (see [Running the real-world dataset](#running-the-real-world-dataset)).

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

`params_sim.yaml` has to match the layout you simulate (topics, anchor/tag IDs and positions). See [Launch files](#launch-files).

**Disclaimer**: the PX4 SITL simulation has been tested with ROS 2 Jazzy only, while the rest of the implementation has been tested in both ROS 2 Humble and ROS 2 Jazzy.

**Sharing one UWBPX4Sim checkout between both setups:** `docker/setup.sh` uses `<ros_ws>/src/UWBPX4Sim` by default, so the local and Docker setups can use the same checkout. `setup_simulator.sh` writes paths for the environment it ran in to `UWBPX4Sim/.setup_env`, so re-run it (option 1 in the container menu, or `./setup_simulator.sh` locally) whenever you switch between the two.


## Launch files

```uwb_localization``` contains two launch files. Both play a bag from `uwb_localization/bags/` (pass `bag_name:=<path relative to bags/>`) alongside the localization nodes:

* ``localization_dataset.launch.py``: the real-world experiment, with UWB, odometry and radar (radar ego-motion from `4D-Radar-Odom`), configured by ``params_dataset.yaml``. By default it plays the real-world dataset; see [Running the real-world dataset](#running-the-real-world-dataset).

* ``localization.launch.py``: just the main system, without the radar odometry nodes, configured by ``params_sim.yaml`` and with RViz. It is used in the SITL scenario, together with the simulation launched from [UWBPX4Sim](https://github.com/amartinezsilva/UWBPX4Sim) (from the Docker menu, or with `simulator_launcher.sh` in the local setup).

The localization configuration file is the single source of truth for the simulated robot pair. By default, ```params_sim.yaml``` subscribes to the first pair generated by ```UWBPX4Sim```: ```/ugv_0/odom```, ```/uav_0/odom```, and ```/ugv_0/eliko/Distances```. To localize a different pair, update the corresponding topic names and anchor/tag IDs in ```uwb_localization/config/params_sim.yaml```.

## Parameter reference

Both localization nodes read their parameters from the same file: `params_dataset.yaml` (real-world dataset) or `params_sim.yaml` (SITL). The values below are those of `params_dataset.yaml`. `params_sim.yaml` changes the topic names and UWB device IDs, `measurement_stdev` (0.12 m), `using_radar` (false) and the anchor priors (zero). It does not set the parameters added for the pose-graph backends, odometry noise and radar factors, so those take the code defaults, which equal the dataset values. Poses are given as `[x, y, z, roll, pitch, yaw]` in m and rad.

### Relative transform estimation (`eliko_global_opt_node`)

Estimates the relative transform between the two robots' odometry frames (the AGV odometry frame expressed in the UAV odometry frame, as `[x, y, z, yaw]`) from UWB ranges over a sliding window of measurements, and publishes it with its covariance on `/eliko_optimization_node/optimized_T`.

| Parameter | Value | Description |
|---|---|---|
| `distances_topic` | `/eliko/Distances` | UWB range measurements (`eliko_messages/DistancesList`). |
| `odom_topic_agv`, `odom_topic_uav` | `/agv/odom`, `/uav/odom` | Odometry of each robot, used to express the anchor and tag positions in each robot's odometry frame at every measurement. |
| `opt_timer_rate` | 10.0 Hz | Rate of the estimation loop. |
| `min_traveled_distance`, `min_traveled_angle` | 0.5 m, 0.262 rad | A new estimate is computed only once **both** robots have moved at least this much (distance or rotation) since the previous one. |
| `max_traveled_distance` | 10.0 m | Length of the measurement window: the oldest measurements are dropped while either robot has travelled more than this within the window. The window also needs at least 2 m of motion of each robot (fixed in the code). |
| `min_measurements` | 50 | Minimum number of ranges in the window to compute an estimate. |
| `measurement_stdev` | 0.1 m | Standard deviation of a UWB range. |
| `use_prior` | true | Compute a closed-form linear solution first and use it as a prior for the nonlinear refinement. |
| `use_ransac` | true | Use RANSAC in that linear solution. |
| `init_solution` | `[0, 0, 0, 0]` | Initial guess of the relative transform `[x, y, z, yaw]`. |
| `moving_average`, `moving_average_max_samples` | true, 10 | Smooth the estimate over the last N estimates. |
| `odom_error_position`, `odom_error_angle` | 2.0, 2.0 | Odometry drift, in % of the distance / rotation travelled. The resulting drift (proportional to the total distance and rotation of both robots since the start) is added to the covariance of the published transform. That covariance is also the covariance of the UWB factors in the pose graph, so these two parameters set how much the pose graph trusts UWB. |
| `agv_init_pose`, `uav_init_pose` | zeros | Initial pose of each robot's odometry: every odometry pose is composed with it. |
| `anchors.ids`, `anchors.positions` | 4 anchors | UWB anchor IDs (on the UGV) and their positions in the UGV body frame, flattened as `[x1, y1, z1, x2, ...]` in the order of the IDs. |
| `tags.ids`, `tags.positions` | 2 tags | The same for the UWB tags (on the UAV), in the UAV body frame. |
| `debug` | true | Subscribe to the DLL ground-truth poses (`/dll_node_arco/pose_estimation`, `/dll_node/pose_estimation`) to publish ground-truth ranges for comparison. |
| `use_sim_time` | true | Use the bag/simulation clock. |

### Pose-graph optimization (`pose_optimization_node`)

Fuses odometry, radar scan matching and the UWB relative transform in a sliding-window pose graph of both robots. Each robot has its own keyframe nodes (4-DoF `[x, y, z, yaw]`, with roll and pitch from the IMU), and each robot's odometry frame has an *anchor* node that places it in the common `map` frame.

**Topics and timing**

| Parameter | Value | Description |
|---|---|---|
| `odom_topic_agv`, `odom_topic_uav` | `/agv/odom`, `/uav/odom` | Odometry of each robot. |
| `radar_topic_agv`, `radar_topic_uav` | `/agv/filtered_pointcloud`, `/uav/filtered_pointcloud` | Radar point clouds from `4D-Radar-Odom`'s pre-processing, already in the IMU frame. Raw radar clouds (in the radar frame) are not supported. |
| `radar_egovel_topic_agv`, `radar_egovel_topic_uav` | `/agv/Ego_Vel_Twist`, `/uav/Ego_Vel_Twist` | Radar ego-velocity, used to refine the initial guess of the scan matching. |
| `opt_timer_rate` | 10.0 Hz | Rate of the main loop: create keyframes and optimize. |
| `use_sim_time` | true | Use the bag/simulation clock. |

**Keyframes and sliding window**

| Parameter | Value | Description |
|---|---|---|
| `min_traveled_distance`, `min_traveled_angle` | 0.5 m, 0.262 rad | A robot gets a new keyframe when it has moved this distance or rotated this angle since its last one. |
| `max_keyframes` | 7 | Sliding window: the last `max_keyframes + 1` nodes of each robot are optimized; older ones are held fixed. |
| `min_keyframes` | 3 | No optimization until a robot has this many keyframes. |

**Solver**

| Parameter | Value | Description |
|---|---|---|
| `posegraph_backend` | `ceres` | Pose-graph solver: `ceres` or `gtsam`. Both solve the same model; `gtsam` needs the package built with GTSAM ≥ 4.3 (see [Basic Dependencies](#basic-dependencies)). |
| `posegraph_compare_backend` | `""` | Optional second backend that solves a copy of every step's graph, for comparison only (its result is not used). Differences and timings are logged and written to `posegraph_backend_stats.csv` on shutdown. |

**Odometry factors**

| Parameter | Value | Description |
|---|---|---|
| `using_odom` | true | Add odometry factors between consecutive keyframes (and use odometry as the scan-matching initial guess). |
| `odom_noise_agv.sigma_xy`, `.sigma_z`, `.sigma_yaw_deg` | 0.025 m, 0.015 m, 0.8° | Standard deviation of the AGV odometry over one keyframe step (x/y, z, yaw). |
| `odom_noise_uav.sigma_xy`, `.sigma_z`, `.sigma_yaw_deg` | 0.045 m, 0.045 m, 0.8° | The same for the UAV. |

**Radar factors**

| Parameter | Value | Description |
|---|---|---|
| `using_radar` | true | Subscribe to the radar topics and add scan-matching factors between keyframes of the same robot. |
| `icp_type_radar` | 2 | Scan matching: 2 = Generalized ICP (small_gicp), 1 = point-to-plane ICP, other = point-to-point ICP. |
| `radar_history_size` | 3 | Number of previous keyframes each new keyframe's scan is matched against. |
| `radar_stdev` | 0.2 m | Radar point noise: maximum correspondence distance of the scan matching (2.5 × this) and, with GICP, the x/y/z standard deviation of the radar factors (z variance ×10, since radar elevation is less reliable). |
| `radar_yaw_stdev_deg` | 1.0° | Yaw standard deviation of the radar factors (GICP). |
| `radar_min_inliers` | 10 | Minimum matched points for a GICP alignment to become a factor. |

**Frames and priors**

| Parameter | Value | Description |
|---|---|---|
| `agv_init_pose`, `uav_init_pose` | zeros | Initial pose of each robot's odometry: every odometry pose is composed with it. |
| `fixed_anchor` | `agv` | Which anchor (`agv` or `uav`) is fixed with a very strong prior, its `*_anchor_prior`, to remove the gauge freedom of the common frame. The other anchor is estimated from the inter-robot factors (UWB, radar encounters). |
| `agv_anchor_prior` | `[13.351, 27.385, 0.636, 0.001, 0.001, 3.079]` | Pose of the AGV odometry frame in the `map` frame: initial value of the AGV anchor, and its prior if `fixed_anchor` is `agv`. |
| `uav_anchor_prior` | `[15.351, 23.385, 0.9, 0, 0, 0.028]` | Pose of the UAV odometry frame in the `map` frame: initial value of the UAV anchor, and its prior if `fixed_anchor` is `uav`. |
| `imu_agv.position`, `imu_uav.position` | see file | Pose of the IMU in each robot's body frame, used to bring the radar clouds (which arrive in the IMU frame) into the body frame for scan matching. The radar mounting poses are not set here: the radar pre-processing applies them (`radar_to_base_link` in `4D-Radar-Odom`'s configuration), so that's the place to change them. |

### Other nodes

* `uav_odometry_node` (dataset only): `use_attitude` (default true) takes the UAV orientation from the DJI attitude topic instead of integrating the angular velocity. It isn't set in the configuration files.
* `agv_odometry_node` (dataset only) has no parameters.
* The radar pre-processing (`radar_pcl_processor`) is configured by `4D-Radar-Odom`'s own configuration files (`config_agv.yaml`, `config_uav.yaml`), including the radar and IMU mounting poses (`radar_to_base_link`, `imu_to_base_link`).

## Running the real-world dataset

The real-world experiment from the paper (a UGV with four UWB anchors and an ARS548 radar, and a UAV with two UWB tags and its own radar) is published on [Harvard Dataverse](https://dataverse.harvard.edu/dataset.xhtml?persistentId=doi:10.7910/DVN/KNKWMJ) under the CC BY-SA 4.0 license. It's a single archive, `mr_radio_dataset.tar.gz` (2.2 GB download, 4.7 GB extracted), containing one ROS 2 bag of about 3 min 46 s. It works with both the [standalone Docker image](#standalone-image) and the [local setup](#running-locally-without-docker); you don't need UWBPX4Sim.

**1. Download and extract it into `uwb_localization/bags/`.** Either download `mr_radio_dataset.tar.gz` from the Dataverse page in your browser, or from the command line:

```bash
cd mr-radio-localization/uwb_localization/bags
curl -L -o mr_radio_dataset.tar.gz https://dataverse.harvard.edu/api/access/datafile/13185888
echo "0f6512ecd2547814208b914ceebec6c6  mr_radio_dataset.tar.gz" | md5sum -c   # should print: OK
tar -xzf mr_radio_dataset.tar.gz
rm mr_radio_dataset.tar.gz   # optional, once extracted
```

You should now have `uwb_localization/bags/experiment1_gt_v2/`, containing `metadata.yaml` and `experiment1_gt_v2_0.db3`. Keep the folder name as it is: it's the launch file's default `bag_name`.

The bag was recorded on ROS 2 Humble in the SQLite (`.db3`) format. ROS 2 Jazzy still plays it as-is, so there's no need to convert it. If you convert it to `.mcap` anyway (e.g. with [this tool](https://mcap.dev/guides/getting-started/ros-2)), pass the new path with `bag_name:=`.

**2. Run it.**

* **Docker (standalone image):** start the container with `cd docker && ./setup.sh`. In the menu, choose **1) Launch localization**, then **2) localization_dataset.launch.py**, then pick `experiment1_gt_v2` from the list of bags. `uwb_localization/bags/` is bind-mounted, so there's nothing to copy into the container. Press Ctrl-C to stop it and return to the menu.

* **Local setup:** with the workspace sourced (`source <ros_ws>/install/setup.bash`), run:

  ```bash
  ros2 launch uwb_localization localization_dataset.launch.py
  ```

  The launch file plays the bag from the package's installed `share/` folder, not from the source tree, so rebuild once after adding the bag: `colcon build --symlink-install --packages-select uwb_localization`. With `--symlink-install`, that only creates links to the bag files instead of copying 4.7 GB. (In Docker this isn't needed: `bags/` is mounted straight onto the installed folder.)

**3. Look at the results.** The bag lasts 3 min 46 s. Its UWB ranges cover roughly the first 3 minutes, and the UWB relative transform is re-estimated periodically while they last. The pose graph keeps optimizing until the bag ends. This launcher opens no RViz window. Its outputs are:

| Topic | Type | Frame | What it is |
|---|---|---|---|
| `/eliko_optimization_node/optimized_T` | `geometry_msgs/PoseWithCovarianceStamped` | `uav/odom` | UAV-UGV relative transform estimated from UWB |
| `/pose_graph_node/agv_poses`, `/pose_graph_node/uav_poses` | `uwb_localization/PoseWithCovarianceStampedArray` | `agv/odom`, `uav/odom` | Optimized pose-graph trajectories of each robot |
| `/pose_graph_node/agv_anchor`, `/pose_graph_node/uav_anchor` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | Anchor nodes placing each robot's odometry frame in the common `map` frame |
| `/agv_pose_markers`, `/uav_pose_markers` | `visualization_msgs/MarkerArray` | `map` | The optimized trajectories, for RViz |

The bag also contains the ground truth used in the paper (DLL LiDAR localization, in the `map` frame): `/dll_node_arco/pose_estimation` for the UGV and `/dll_node/pose_estimation` for the UAV.

To watch the trajectories, open RViz in another terminal (in Docker: menu option **2) Open a shell**, or `docker exec -it mr-radio-localization-standalone bash`), set **Fixed Frame** to `map`, and add `/agv_pose_markers`, `/uav_pose_markers` and, for comparison, the two ground-truth topics:

```bash
rviz2
```

To keep the results for analysis, record them while the dataset plays:

```bash
ros2 bag record /eliko_optimization_node/optimized_T /pose_graph_node/agv_poses /pose_graph_node/uav_poses \
  /pose_graph_node/agv_anchor /pose_graph_node/uav_anchor /dll_node/pose_estimation /dll_node_arco/pose_estimation
```

**What to expect.** The [paper](https://arxiv.org/abs/2509.26558) reports these results on this dataset, against the DLL ground truth:

| | Translational ATE | Angular RMSE |
|---|---|---|
| UAV (118 m trajectory) | 1.44 m | 5.74° |
| UGV (27 m trajectory) | 0.94 m | 11.4° |

The UWB relative transform converges after about 40 s, to an error of roughly 1.1 m in translation and 5° in rotation. The UGV trajectory is tied to the map while the UAV's isn't, so most of the relative-localization error shows up in the UAV trajectory: about 1.2 % of the distance it travels.

# Troubleshooting

* **CMake can't find `Sophus` when building ``uwb_localization``.** Install Sophus through the ROS package (`rosdep install --from-paths src --ignore-src -y` from your workspace, or `sudo apt install ros-<distro>-sophus`). That's what the Docker images use on Jazzy, and it builds as-is. If CMake still can't find it, which has been reported with other package managers such as `vcpkg`, install Sophus from source and build the workspace again:
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
mkdir build && cd build
cmake ..
sudo make install
```

* **SITL: the PX4 topics don't show up in ROS 2** (Micro XRCE-DDS Agent not connecting, Ubuntu 24.04 / ROS 2 Jazzy). This is a simulator setup issue, and the workaround is documented in [UWBPX4Sim's troubleshooting](https://github.com/amartinezsilva/UWBPX4Sim/blob/main/docs/advanced_setup.md#troubleshooting).

If you'd rather avoid these potential issues entirely, see [Running with Docker](#running-with-docker-recommended) above for a pre-built, known-working environment.

# Disclaimer on AI usage

The work is in this repository is part of a multiple year research project backed with academic works and documented bibliography. While the problem formulation, system architecture and algorithms implemented are original work, it is important to disclose that AI agents have been used in the later stages to improve code readability, efficiency and polish the interfaces (p.e., the implementation of the docker container framework and the implementation and setup of the simulator plugin UWBPX4Sim have been aided by coding agents). We are open to any feedback from the users and will happily note your concerns and tips to make this code better.

# Acknowledgements

This work was supported by the grants PICRAH 4.0 0 (PLEC2023-010353): funded by the Spanish Ministry of Science and Innovation and the Spanish Research Agency (MCIN/AEI/10.13039/501100011033); and INSERTION (PID2021-127648OB-C31), funded by “Agencia Estatal de Investigación – Ministerio de Ciencia, Innovación y Universidades” and the “European Union NextGenerationEU/PRTR”.

![](images/fondos_proyectos.png)
