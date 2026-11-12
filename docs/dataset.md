# Running the real-world dataset

The real-world experiment from the paper (a UGV with four UWB anchors and an ARS548 radar, and a UAV with two UWB tags and its own radar) is published on [Harvard Dataverse](https://dataverse.harvard.edu/dataset.xhtml?persistentId=doi:10.7910/DVN/KNKWMJ) under the CC BY-SA 4.0 license. It's a single archive, `mr_radio_dataset.tar.gz` (2.2 GB download, 4.7 GB extracted), containing one ROS 2 bag of about 3 min 46 s. It works with both the [standalone Docker image](../README.md#standalone-image) and the [local setup](local_setup.md); you don't need UWBPX4Sim.

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

* **Docker (standalone image):** start the container with `cd docker && ./setup.sh`. In the menu, choose **1) Launch localization**, then **2) localization_dataset.launch.py**, then pick `experiment1_gt_v2` from the list of bags. Your checkout, `bags/` included, is mounted into the container, so there's nothing to copy. Press Ctrl-C to stop it and return to the menu.

* **Local setup:** with the workspace sourced (`source <ros_ws>/install/setup.bash`), run:

  ```bash
  ros2 launch uwb_localization localization_dataset.launch.py
  ```

  The launch file plays the bag from the package's installed `share/` folder. The build installs that `bags/` folder as a link to `uwb_localization/bags/` (never a copy of the 4.7 GB), so a bag added there is found without rebuilding.

**3. Look at the results.** The bag lasts 3 min 46 s. Its UWB ranges cover roughly the first 3 minutes, and the UWB relative transform is re-estimated periodically while they last. The pose graph keeps optimizing until the bag ends. This launcher opens no RViz window. Its outputs are:

| Topic | Type | Frame | What it is |
|---|---|---|---|
| `/eliko_optimization_node/optimized_T` | `geometry_msgs/PoseWithCovarianceStamped` | `uav/odom` | UAV-UGV relative transform estimated from UWB |
| `/pose_graph_node/agv_poses`, `/pose_graph_node/uav_poses` | `uwb_localization/PoseWithCovarianceStampedArray` | `agv/odom`, `uav/odom` | Optimized pose-graph trajectories of each robot |
| `/pose_graph_node/agv_anchor`, `/pose_graph_node/uav_anchor` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | Anchor nodes placing each robot's odometry frame in the common `map` frame |
| `/agv_pose_markers`, `/uav_pose_markers` | `visualization_msgs/MarkerArray` | `map` | The optimized trajectories, for RViz |

The bag also contains the ground truth used in the paper (DLL LiDAR localization, in the `map` frame): `/dll_node_arco/pose_estimation` for the UGV and `/dll_node/pose_estimation` for the UAV.

To watch the trajectories, open RViz in another terminal (in Docker: menu option **3) Open a shell**, or `docker exec -it mr-radio-localization-standalone bash`), set **Fixed Frame** to `map`, and add `/agv_pose_markers`, `/uav_pose_markers` and, for comparison, the two ground-truth topics:

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
