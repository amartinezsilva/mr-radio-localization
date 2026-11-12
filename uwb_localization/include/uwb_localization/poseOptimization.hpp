#pragma once

// ROS
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "uwb_localization/msg/pose_with_covariance_stamped_array.hpp"
#include "uwb_localization/srv/update_point_clouds.hpp"

#include "uwb_localization/utils.hpp"
#include "uwb_localization/posegraph.hpp"
#include "uwb_localization/posegraph_backend.hpp"

#include <visualization_msgs/msg/marker_array.hpp>


namespace uwb_localization {

/**
 * @brief Pose-graph fusion of AGV & UAV odom, radar (ICP/GICP), and UWB encounters.
 *
 * Declarations only; see poseOptimization.cpp for definitions. The pose graph
 * itself is solved by a PoseGraphBackend (Ceres or GTSAM, posegraph_backend.hpp).
 */
class PoseOptimizationNode : public rclcpp::Node {
public:
  using UpdatePointClouds = uwb_localization::srv::UpdatePointClouds;
  using PoseWithCovArray = uwb_localization::msg::PoseWithCovarianceStampedArray;

  // Pose graph/optimizer aliases
  using PriorConstraint       = posegraph::PriorConstraint;
  using MeasurementConstraint = posegraph::MeasurementConstraint;
  using MapOfStates           = posegraph::MapOfStates;
  using VectorOfConstraints   = posegraph::VectorOfConstraints;
  using Measurements          = posegraph::Measurements;
  using RadarMeasurements     = posegraph::RadarMeasurements;

  // Point cloud aliases
  using PointT       = pcl::PointXYZ;
  using Cloud        = pcl::PointCloud<PointT>;
  using CloudPtr     = Cloud::Ptr;
  using CloudConstPtr= Cloud::ConstPtr;

public:
  PoseOptimizationNode();

  /// Returns the AGV pose graph as a message array.
  PoseWithCovArray getAGVPoseGraph();

  /// Returns the UAV pose graph as a message array.
  PoseWithCovArray getUAVPoseGraph();

  /// Logs aggregate solver metrics (runs, avg, stddev).
  void getMetrics() const;

  /// Writes one CSV row per optimization step: solver stats of the backend
  /// (and of the comparison backend, with the largest solution differences).
  void writeBackendStats(const std::string& filename) const;

  /// Writes the whole pose graph (final states, priors, every factor with its
  /// measurement and covariance) as text, for offline analysis (e.g. noise
  /// calibration). One record per line:
  ///   node <a|u> <id> <planar> <roll> <pitch> <x> <y> <z> <yaw>
  ///   anchor <a|u> <planar> <roll> <pitch> <x> <y> <z> <yaw>
  ///   prior <agv|uav|anchor_agv|anchor_uav> <tx ty tz qx qy qz qw> <cov 4x4, row-major>
  ///   (one anchor prior: the one of the fixed anchor)
  ///   factor <group> <id_begin> <id_end> <tx ty tz qx qy qz qw> <cov 4x4, row-major>
  /// with group odom_agv, odom_uav, radar_agv, radar_uav, uwb or radar_encounter.
  void writePoseGraphDump(const std::string& filename) const;

private:
  //---------------- Parameter management ----------------//
  void declareParams();
  void getParams();

  //---------------- ROS Callbacks ----------------//
  void agvOdomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void uavOdomCb(const nav_msgs::msg::Odometry::SharedPtr msg);

  void pclAgvRadarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void pclUavRadarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void AgvEgoVelCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);
  void UavEgoVelCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg);

  void optimizedTfCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /// Timer: runs optimization.
  void globalOptCb();

  //---------------- Constraints & ICP ----------------//
  /**
   * @brief Add ICP/GICP constraint between scans (already in sensor/body frames).
   */
  bool addPointCloudConstraint(const CloudConstPtr& source_scan,
                               const CloudConstPtr& target_scan,
                               const Sophus::SE3d& T_source_sensor,
                               const Sophus::SE3d& T_target_sensor,
                               double sigma,
                               int icp_type,
                               int id_source,
                               int id_target,
                               VectorOfConstraints& constraints_list,
                               bool send_visualization = false,
                               const Eigen::Matrix4f* initial_guess = nullptr);

  /// Send point clouds to external visualizer service (optional).
  void sendPointCloudServiceRequest(const CloudConstPtr& source_cloud,
                                    const CloudConstPtr& target_cloud,
                                    const CloudConstPtr& aligned_cloud);

  /// Downsampling / (optional) outlier removal.
  CloudPtr preprocessPointCloud(const CloudConstPtr& input_cloud,
                                float meanK = 50.0f,
                                float stdevmulthresh = 2.0f,
                                float leaf_size = 0.05f);

  /// Approximate ICP covariance (reduced 4x4) from correspondences.
  Eigen::Matrix4d computeICPCovariance(const CloudConstPtr& source,
                                       const CloudConstPtr& target,
                                       const Eigen::Matrix4f& transformation,
                                       double sensor_variance = 0.01);

  /// Run ICP/GICP with configured settings. For GICP (icp_type 2) also returns
  /// the final Hessian and the number of inlier correspondences.
  bool run_icp(const CloudConstPtr& source_cloud,
               const CloudConstPtr& target_cloud,
               CloudPtr& aligned_cloud,
               Eigen::Matrix4f& transformation,
               const double& pointcloud_sigma,
               double& fitness,
               const int& icp_type,
               Eigen::Matrix<double,6,6>& final_hessian,
               size_t& num_inliers) const;

  /// Build point+normal cloud for point-to-plane ICP.
  pcl::PointCloud<pcl::PointNormal>::Ptr computePointNormalCloud(const CloudConstPtr& cloud,
                                                                 float radius_search = 0.1f) const;

  /// Integrate radar ego-velocity (body frame) over dt to SE(3) delta.
  Sophus::SE3d integrateEgoVelIntoSE3(const Eigen::Vector3d& radar_vel_t,
                                      const Sophus::SE3d& odom_T_s,
                                      const Sophus::SE3d& odom_T_t,
                                      double dt);

  /// Pose-graph inputs over the given states (the node's own, or copies of them).
  PoseGraphInput makeGraphInput(MapOfStates& agv_map, MapOfStates& uav_map,
                                State& anchor_agv, State& anchor_uav) const;

  /// Solves the pose graph with the configured backend; updates states and
  /// covariances. Returns true on convergence.
  bool runPosegraphOptimization();

private:
  //---------------- Subscriptions ----------------//
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr uav_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr agv_odom_sub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_agv_radar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_uav_radar_sub_;

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr agv_egovel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr uav_egovel_sub_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr optimized_tf_sub_;

  //---------------- Publishers / Services / Timers ----------------//
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr anchor_agv_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr anchor_uav_publisher_;
  rclcpp::Publisher<PoseWithCovArray>::SharedPtr poses_uav_publisher_;
  rclcpp::Publisher<PoseWithCovArray>::SharedPtr poses_agv_publisher_;
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr agv_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr uav_marker_pub_;


  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Client<UpdatePointClouds>::SharedPtr pcl_visualizer_client_;
  rclcpp::TimerBase::SharedPtr global_optimization_timer_;

  //---------------- Parameters ----------------//
  double opt_timer_rate_{10.0};
  Sophus::SE3d T_uav_imu_, T_agv_imu_;
  Sophus::SE3d T_agv_anchor_prior_, T_uav_anchor_prior_;
  double pointcloud_radar_sigma_{0.1};

  std::string odom_topic_agv_, odom_topic_uav_;
  std::string pcl_topic_radar_agv_, pcl_topic_radar_uav_;
  std::string egovel_topic_radar_agv_, egovel_topic_radar_uav_;

  int icp_type_radar_{2};
  bool using_radar_{true}, using_odom_{true};
  double min_traveled_distance_{0.5}, min_traveled_angle_{0.524};
  int max_keyframes_{10}, min_keyframes_{3};
  int radar_history_size_{5};
  int radar_min_inliers_{10};
  double radar_yaw_stdev_{1.0 * M_PI / 180.0};  // rad, yaw std of radar factors

  //---------------- Measurements & State ----------------//
  CloudPtr uav_radar_cloud_{CloudPtr(new Cloud)};
  CloudPtr agv_radar_cloud_{CloudPtr(new Cloud)};

  bool uwb_transform_available_{false};
  geometry_msgs::msg::PoseWithCovarianceStamped latest_relative_pose_;
  geometry_msgs::msg::TwistWithCovarianceStamped agv_radar_egovel_, uav_radar_egovel_;

  std::deque<Eigen::Vector3d> agv_velocity_buffer_;
  std::deque<Eigen::Vector3d> uav_velocity_buffer_;

  Sophus::SE3d latest_relative_pose_SE3_;
  Eigen::Matrix<double,6,6> latest_relative_pose_cov_;

  State init_state_uav_, init_state_agv_;
  State anchor_node_uav_, anchor_node_agv_;
  PriorConstraint prior_agv_, prior_uav_, prior_anchor_agv_, prior_anchor_uav_;

  MapOfStates uav_map_, agv_map_;
  VectorOfConstraints proprioceptive_constraints_uav_, extraceptive_constraints_uav_;
  VectorOfConstraints proprioceptive_constraints_agv_, extraceptive_constraints_agv_;
  VectorOfConstraints encounter_constraints_uwb_, encounter_constraints_pointcloud_;

  Measurements agv_measurements_, prev_agv_measurements_;
  Measurements uav_measurements_, prev_uav_measurements_;
  std::deque<RadarMeasurements> radar_history_agv_, radar_history_uav_;

  int uav_id_{0}, agv_id_{0};
  bool graph_initialized_{false};

  std::string eliko_frame_id_, uav_frame_id_, global_frame_graph_;
  std::string odom_tf_agv_t_, odom_tf_uav_t_;

  Sophus::SE3d uav_odom_pose_, last_uav_odom_pose_;
  Sophus::SE3d agv_odom_pose_, last_agv_odom_pose_;
  Sophus::SE3d agv_init_pose_, uav_init_pose_;
  Sophus::SE3d uav_radar_odom_pose_, agv_radar_odom_pose_;

  posegraph::OdometryNoise odom_noise_agv_, odom_noise_uav_;

  // Timing
  double measurement_sync_thr_{0.25};
  double last_agv_odom_time_sec_{0.0}, last_uav_odom_time_sec_{0.0};
  double last_agv_radar_time_sec_{0.0}, last_uav_radar_time_sec_{0.0};
  double last_agv_egovel_time_sec_{0.0}, last_uav_egovel_time_sec_{0.0};
  double last_relative_pose_time_sec_{0.0}, last_relative_pose_used_time_sec_{0.0};

  bool last_agv_odom_initialized_{false}, last_uav_odom_initialized_{false};
  bool relative_pose_initialized_{false};

  double uav_translation_{0.0}, agv_translation_{0.0};
  double uav_rotation_{0.0}, agv_rotation_{0.0};

  rclcpp::Time graph_start_time_;

  // Pose-graph backends
  std::string backend_name_{"ceres"}, compare_backend_name_;
  bool fix_uav_anchor_{false};  // fixed_anchor == "uav"
  std::unique_ptr<PoseGraphBackend> backend_, compare_backend_;

  struct BackendStepRecord {
    double time = 0.0;
    int agv_id = 0, uav_id = 0;
    PoseGraphSolveStats main, compare;
    bool has_compare = false;
    double max_dpos = 0.0, max_dyaw = 0.0, max_dcov = 0.0;
  };
  std::vector<BackendStepRecord> backend_records_;

  // Metrics
  int total_solves_{0};
  double total_solver_time_{0.0};
  std::vector<double> solver_times_;
};

} // namespace uwb_localization
