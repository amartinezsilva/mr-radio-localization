# Parameter reference

Both localization nodes read their parameters from the same file: `params_dataset.yaml` (real-world dataset) or `params_sim.yaml` (SITL). The values below are those of `params_dataset.yaml`. `params_sim.yaml` changes the topic names and UWB device IDs, `measurement_stdev` (0.12 m), `using_radar` (false) and the anchor priors (zero). It does not set the parameters added for the pose-graph backends, odometry noise and radar factors, so those take the code defaults, which equal the dataset values. Poses are given as `[x, y, z, roll, pitch, yaw]` in m and rad.

## Relative transform estimation (`eliko_global_opt_node`)

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

## Pose-graph optimization (`pose_optimization_node`)

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
| `posegraph_backend` | `ceres` | Pose-graph solver: `ceres` or `gtsam`. Both solve the same model; `gtsam` needs the package built with GTSAM ≥ 4.3 (see [Optional: GTSAM backend](local_setup.md#optional-gtsam-backend); the Docker images include it). |
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

## Other nodes

* `uav_odometry_node` (dataset only): `use_attitude` (default true) takes the UAV orientation from the DJI attitude topic instead of integrating the angular velocity. It isn't set in the configuration files.
* `agv_odometry_node` (dataset only) has no parameters.
* The radar pre-processing (`radar_pcl_processor`) is configured by `4D-Radar-Odom`'s own configuration files (`config_agv.yaml`, `config_uav.yaml`), including the radar and IMU mounting poses (`radar_to_base_link`, `imu_to_base_link`).
