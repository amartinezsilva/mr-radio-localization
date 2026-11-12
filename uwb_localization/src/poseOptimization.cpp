#include "uwb_localization/poseOptimization.hpp"

// Standard
#include <deque>
#include <utility>
#include <unordered_map>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <fstream>

// ROS MSGS
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "uwb_localization/posegraph.hpp"

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>

// small_gicp
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling_omp.hpp>


using std::placeholders::_1;

namespace uwb_localization {

// --------------------- ctor --------------------- //
PoseOptimizationNode::PoseOptimizationNode() : rclcpp::Node("pose_optimization_node") {

    declareParams();
    getParams();

    // Pose-graph backend(s). Throws (the node does not start) on an unknown or
    // unavailable backend name.
    PoseGraphBackendOptions backend_options;
    backend_ = makePoseGraphBackend(backend_name_, backend_options, this->get_logger());
    if (!compare_backend_name_.empty())
        compare_backend_ = makePoseGraphBackend(compare_backend_name_, backend_options, this->get_logger());
    RCLCPP_INFO(this->get_logger(), "Pose-graph backend: %s%s%s", backend_->name().c_str(),
                compare_backend_ ? ", compared every step against: " : "",
                compare_backend_ ? compare_backend_->name().c_str() : "");

    rclcpp::SensorDataQoS qos; // sensor-friendly QoS

    if (using_odom_) {
        agv_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_agv_, qos, std::bind(&PoseOptimizationNode::agvOdomCb, this, _1));
        uav_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_uav_, qos, std::bind(&PoseOptimizationNode::uavOdomCb, this, _1));
    }

    odom_tf_agv_t_ = "agv/odom";
    odom_tf_uav_t_ = "uav/odom";

    if (using_radar_) {
        pcl_agv_radar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pcl_topic_radar_agv_, qos, std::bind(&PoseOptimizationNode::pclAgvRadarCb, this, _1));
        pcl_uav_radar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pcl_topic_radar_uav_, qos, std::bind(&PoseOptimizationNode::pclUavRadarCb, this, _1));

        agv_egovel_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            egovel_topic_radar_agv_, qos, std::bind(&PoseOptimizationNode::AgvEgoVelCb, this, _1));
        uav_egovel_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            egovel_topic_radar_uav_, qos, std::bind(&PoseOptimizationNode::UavEgoVelCb, this, _1));
    }

    pcl_visualizer_client_ = this->create_client<UpdatePointClouds>(
        "eliko_optimization_node/pcl_visualizer_service");

    optimized_tf_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/eliko_optimization_node/optimized_T", 10,
        std::bind(&PoseOptimizationNode::optimizedTfCb, this, _1));

    // Pose publishers
    anchor_agv_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "pose_graph_node/agv_anchor", 10);
    anchor_uav_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "pose_graph_node/uav_anchor", 10);

    poses_uav_publisher_ = this->create_publisher<PoseWithCovArray>(
        "pose_graph_node/uav_poses", 10);
    poses_agv_publisher_ = this->create_publisher<PoseWithCovArray>(
        "pose_graph_node/agv_poses", 10);

    // Add these publishers in your constructor:
    agv_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("agv_pose_markers", 10);
    uav_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("uav_pose_markers", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    double global_opt_rate_s = 1.0 / opt_timer_rate_;
    global_optimization_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(int(global_opt_rate_s * 1000)),
        std::bind(&PoseOptimizationNode::globalOptCb, this));

    global_frame_graph_ = "map";
    eliko_frame_id_ = "agv_opt"; // simulation: "agv_gt" or odom variant
    uav_frame_id_ = "uav_opt";

    last_agv_odom_initialized_ = false;
    last_uav_odom_initialized_ = false;
    relative_pose_initialized_ = false;

    uav_translation_ = agv_translation_ = 0.0;
    uav_rotation_ = agv_rotation_ = 0.0;

    agv_id_ = uav_id_ = 0;

    graph_initialized_ = false;
    uwb_transform_available_ = false;

    RCLCPP_INFO(this->get_logger(), "Eliko Optimization Node initialized.");
}

// --------------------- public: pose graph getters --------------------- //
PoseOptimizationNode::PoseWithCovArray PoseOptimizationNode::getAGVPoseGraph() {
  PoseWithCovArray poses;
  poses.header.stamp = this->get_clock()->now();
  poses.header.frame_id = odom_tf_agv_t_;
  posegraph::getPoseGraph(agv_map_, poses);
  return poses;
}

PoseOptimizationNode::PoseWithCovArray PoseOptimizationNode::getUAVPoseGraph() {
  PoseWithCovArray poses;
  poses.header.stamp = this->get_clock()->now();
  poses.header.frame_id = odom_tf_uav_t_;
  posegraph::getPoseGraph(uav_map_, poses);
  return poses;
}

void PoseOptimizationNode::getMetrics() const {
  if (total_solves_ > 0 && !solver_times_.empty()) {
    double avg = total_solver_time_ / static_cast<double>(total_solves_);
    double sum_sq = 0.0;
    for (const auto& t : solver_times_) sum_sq += (t - avg) * (t - avg);
    double std_dev = std::sqrt(sum_sq / static_cast<double>(solver_times_.size()));

    RCLCPP_INFO(this->get_logger(),
                "===== Pose Graph Metrics (%s backend) =====\n"
                "Total runs              : %d\n"
                "Total time              : %.6f s\n"
                "Average time per solve  : %.6f s\n"
                "Std dev of solve times  : %.6f s",
                backend_->name().c_str(), total_solves_, total_solver_time_, avg, std_dev);
  } else {
    RCLCPP_WARN(this->get_logger(), "No solver runs have completed yet.");
  }
}

visualization_msgs::msg::Marker createPoseMarker(
    const geometry_msgs::msg::Pose& pose, int id, const std::string& frame_id, float r, float g, float b)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = "poses";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    return marker;
}


// --------------------- parameters --------------------- //
void PoseOptimizationNode::declareParams() {
    // Basic topics and timing.
    this->declare_parameter<std::string>("odom_topic_agv", "/arco/idmind_motors/odom");
    this->declare_parameter<std::string>("odom_topic_uav", "/uav/odom");
    this->declare_parameter<std::string>("radar_topic_agv", "/arco/radar/PointCloudDetection");
    this->declare_parameter<std::string>("radar_egovel_topic_agv", "/agv/Ego_Vel_Twist");
    this->declare_parameter<std::string>("radar_topic_uav", "/drone/radar/PointCloudDetection");
    this->declare_parameter<std::string>("radar_egovel_topic_uav", "/uav/Ego_Vel_Twist");

    this->declare_parameter<double>("opt_timer_rate", 10.0); // Hz

    // KF Management
    this->declare_parameter<double>("min_traveled_distance", 0.5);
    this->declare_parameter<double>("min_traveled_angle", 0.524);
    this->declare_parameter<int64_t>("min_keyframes", 3);
    this->declare_parameter<int64_t>("max_keyframes", 10);

    // Modes
    this->declare_parameter<bool>("using_odom", true);
    this->declare_parameter<bool>("using_radar", true);

    // ICP variables
    this->declare_parameter<double>("radar_stdev", 0.1);
    this->declare_parameter<int64_t>("icp_type_radar", 2);
    this->declare_parameter<int64_t>("radar_history_size", 5);
    this->declare_parameter<int64_t>("radar_min_inliers", 10);
    this->declare_parameter<double>("radar_yaw_stdev_deg", 1.0);

    // Odometry factor noise per keyframe step (posegraph::OdometryNoise), per
    // robot. Typical values: a few % of the keyframe distance in translation,
    // under 1 deg in yaw (from the IMU / DJI attitude).
    auto declare_odom_noise = [this](const std::string &p, double sigma_xy, double sigma_z, double sigma_yaw_deg) {
        this->declare_parameter<double>(p + ".sigma_xy", sigma_xy);
        this->declare_parameter<double>(p + ".sigma_z", sigma_z);
        this->declare_parameter<double>(p + ".sigma_yaw_deg", sigma_yaw_deg);
    };
    declare_odom_noise("odom_noise_agv", 0.025, 0.015, 0.8);
    declare_odom_noise("odom_noise_uav", 0.045, 0.045, 0.8);

    // Pose-graph backend: "ceres" or "gtsam". An optional second backend
    // solves a copy of every step's graph, only to compare against.
    this->declare_parameter<std::string>("posegraph_backend", "ceres");
    this->declare_parameter<std::string>("posegraph_compare_backend", "");

    // Anchor priors: [tx, ty, tz, roll, pitch, yaw]
    this->declare_parameter<std::vector<double>>("agv_anchor_prior",
        std::vector<double>{13.694, 25.197, 0.22, 0.001, 0.001, 3.04});
    this->declare_parameter<std::vector<double>>("uav_anchor_prior",
        std::vector<double>{15.925, 22.978, 0.91, 0.0, 0.0, 0.028});

    // Initial local poses
    this->declare_parameter<std::vector<double>>("agv_init_pose",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("uav_init_pose",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

    // Sensor placement: [tx, ty, tz, roll, pitch, yaw]
    this->declare_parameter<std::vector<double>>("imu_uav.position",
        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("imu_agv.position",
        std::vector<double>{0.0, 0.0, 0.165, 0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("radar_uav.position",
        std::vector<double>{-0.385, -0.02, -0.225, 0.0, 2.417, 3.14});
    this->declare_parameter<std::vector<double>>("radar_agv.position",
        std::vector<double>{0.45, 0.05, 0.65, 0.0, 0.0, 0.0});
}

void PoseOptimizationNode::getParams() {
    this->get_parameter("opt_timer_rate", opt_timer_rate_);
    this->get_parameter("odom_topic_agv", odom_topic_agv_);
    this->get_parameter("odom_topic_uav", odom_topic_uav_);

    this->get_parameter("radar_topic_agv", pcl_topic_radar_agv_);
    this->get_parameter("radar_egovel_topic_agv", egovel_topic_radar_agv_);
    this->get_parameter("radar_topic_uav", pcl_topic_radar_uav_);
    this->get_parameter("radar_egovel_topic_uav", egovel_topic_radar_uav_);

    this->get_parameter("using_odom", using_odom_);
    this->get_parameter("using_radar", using_radar_);

    this->get_parameter("icp_type_radar", icp_type_radar_);
    this->get_parameter("radar_stdev", pointcloud_radar_sigma_);

    this->get_parameter("min_traveled_distance", min_traveled_distance_);
    this->get_parameter("min_traveled_angle", min_traveled_angle_);
    this->get_parameter("min_keyframes", min_keyframes_);
    this->get_parameter("max_keyframes", max_keyframes_);
    this->get_parameter("radar_history_size", radar_history_size_);
    this->get_parameter("radar_min_inliers", radar_min_inliers_);
    double radar_yaw_stdev_deg = 1.0;
    this->get_parameter("radar_yaw_stdev_deg", radar_yaw_stdev_deg);
    radar_yaw_stdev_ = radar_yaw_stdev_deg * M_PI / 180.0;

    auto get_odom_noise = [this](const std::string &p, posegraph::OdometryNoise &n) {
        double sigma_yaw_deg = 0.0;
        this->get_parameter(p + ".sigma_xy", n.sigma_xy);
        this->get_parameter(p + ".sigma_z", n.sigma_z);
        this->get_parameter(p + ".sigma_yaw_deg", sigma_yaw_deg);
        n.sigma_yaw = sigma_yaw_deg * M_PI / 180.0;
        RCLCPP_INFO(this->get_logger(), "  %s: sigma_xy %.3f m, sigma_z %.3f m, sigma_yaw %.2f deg",
                    p.c_str(), n.sigma_xy, n.sigma_z, sigma_yaw_deg);
    };
    get_odom_noise("odom_noise_agv", odom_noise_agv_);
    get_odom_noise("odom_noise_uav", odom_noise_uav_);

    this->get_parameter("posegraph_backend", backend_name_);
    this->get_parameter("posegraph_compare_backend", compare_backend_name_);

    std::vector<double> agv_anchor_prior, uav_anchor_prior, uav_init_pose, agv_init_pose;
    this->get_parameter("agv_anchor_prior", agv_anchor_prior);
    this->get_parameter("uav_anchor_prior", uav_anchor_prior);
    this->get_parameter("agv_init_pose", agv_init_pose);
    this->get_parameter("uav_init_pose", uav_init_pose);

    // Build SE3 transforms
    T_agv_anchor_prior_ = buildTransformationSE3(agv_anchor_prior[3], agv_anchor_prior[4],
        Eigen::Vector4d(agv_anchor_prior[0], agv_anchor_prior[1], agv_anchor_prior[2], agv_anchor_prior[5]));
    T_uav_anchor_prior_ = buildTransformationSE3(uav_anchor_prior[3], uav_anchor_prior[4],
        Eigen::Vector4d(uav_anchor_prior[0], uav_anchor_prior[1], uav_anchor_prior[2], uav_anchor_prior[5]));

    agv_init_pose_ = buildTransformationSE3(agv_init_pose[3], agv_init_pose[4],
        Eigen::Vector4d(agv_init_pose[0], agv_init_pose[1], agv_init_pose[2], agv_init_pose[5]));
    uav_init_pose_ = buildTransformationSE3(uav_init_pose[3], uav_init_pose[4],
        Eigen::Vector4d(uav_init_pose[0], uav_init_pose[1], uav_init_pose[2], uav_init_pose[5]));

    // Sensor placement
    std::vector<double> radar_uav_pos, radar_agv_pos, imu_uav_pos, imu_agv_pos;
    this->get_parameter("imu_uav.position", imu_uav_pos);
    this->get_parameter("imu_agv.position", imu_agv_pos);
    this->get_parameter("radar_uav.position", radar_uav_pos);
    this->get_parameter("radar_agv.position", radar_agv_pos);

    T_uav_imu_ = buildTransformationSE3(imu_uav_pos[3], imu_uav_pos[4],
        Eigen::Vector4d(imu_uav_pos[0], imu_uav_pos[1], imu_uav_pos[2], imu_uav_pos[5]));
    T_agv_imu_ = buildTransformationSE3(imu_agv_pos[3], imu_agv_pos[4],
        Eigen::Vector4d(imu_agv_pos[0], imu_agv_pos[1], imu_agv_pos[2], imu_agv_pos[5]));
    T_uav_radar_ = buildTransformationSE3(radar_uav_pos[3], radar_uav_pos[4],
        Eigen::Vector4d(radar_uav_pos[0], radar_uav_pos[1], radar_uav_pos[2], radar_uav_pos[5]));
    T_agv_radar_ = buildTransformationSE3(radar_agv_pos[3], radar_agv_pos[4],
        Eigen::Vector4d(radar_agv_pos[0], radar_agv_pos[1], radar_agv_pos[2], radar_agv_pos[5]));

    // Logs
    RCLCPP_INFO(this->get_logger(), "PoseOptimizationNode parameters:");
    RCLCPP_INFO(this->get_logger(), "  odom_topic_agv: %s", odom_topic_agv_.c_str());
    RCLCPP_INFO(this->get_logger(), "  odom_topic_uav: %s", odom_topic_uav_.c_str());
    RCLCPP_INFO(this->get_logger(), "  radar_topic_agv: %s", pcl_topic_radar_agv_.c_str());
    RCLCPP_INFO(this->get_logger(), "  radar_topic_uav: %s", pcl_topic_radar_uav_.c_str());
    RCLCPP_INFO(this->get_logger(), "  opt_timer_rate: %f Hz", opt_timer_rate_);
    RCLCPP_INFO(this->get_logger(), "  min_traveled_distance: %f m", min_traveled_distance_);
    RCLCPP_INFO(this->get_logger(), "  min_traveled_angle: %f rad", min_traveled_angle_);
    RCLCPP_INFO(this->get_logger(), "  min_keyframes: %d, max_keyframes: %d", min_keyframes_, max_keyframes_);
    RCLCPP_INFO(this->get_logger(), "  using_odom: %s, using_radar: %s",
                using_odom_ ? "true" : "false",
                using_radar_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "  radar_sigma: %f", pointcloud_radar_sigma_);
    RCLCPP_INFO(this->get_logger(), "  icp_type_radar: %d", icp_type_radar_);
    RCLCPP_INFO(this->get_logger(), "  radar_history_size: %d", radar_history_size_);
    RCLCPP_INFO(this->get_logger(), "  radar_min_inliers: %d", radar_min_inliers_);
    RCLCPP_INFO(this->get_logger(), "  radar_yaw_stdev: %.2f deg", radar_yaw_stdev_ * 180.0 / M_PI);

    RCLCPP_INFO(this->get_logger(), "T_agv_anchor_prior:\n");
    logTransformationMatrix(T_agv_anchor_prior_.matrix(), this->get_logger());
    RCLCPP_INFO(this->get_logger(), "T_uav_anchor_prior:\n");
    logTransformationMatrix(T_uav_anchor_prior_.matrix(), this->get_logger());
    RCLCPP_INFO(this->get_logger(), "AGV initial pose:\n");
    logTransformationMatrix(agv_init_pose_.matrix(), this->get_logger());
    RCLCPP_INFO(this->get_logger(), "UAV initial pose:\n");
    logTransformationMatrix(uav_init_pose_.matrix(), this->get_logger());
    RCLCPP_INFO(this->get_logger(), "T_uav_imu:\n");
    logTransformationMatrix(T_uav_imu_.matrix(), this->get_logger());
    RCLCPP_INFO(this->get_logger(), "T_agv_imu:\n");
    logTransformationMatrix(T_agv_imu_.matrix(), this->get_logger());
    RCLCPP_INFO(this->get_logger(), "T_uav_radar:\n");
    logTransformationMatrix(T_uav_radar_.matrix(), this->get_logger());
    RCLCPP_INFO(this->get_logger(), "T_agv_radar:\n");
    logTransformationMatrix(T_agv_radar_.matrix(), this->get_logger());
}


// --------------------- callbacks --------------------- //
void PoseOptimizationNode::agvOdomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    
    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);

    Eigen::Vector3d t(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    Sophus::SE3d current_pose(q, t);
    agv_odom_pose_ = agv_init_pose_ * current_pose;

    if (!last_agv_odom_initialized_) {
        last_agv_odom_time_sec_ = rclcpp::Time(msg->header.stamp).seconds();
        last_agv_odom_initialized_ = true;
        last_agv_odom_pose_ = agv_odom_pose_;
        return;
    }

    Eigen::Matrix<double, 6, 1> log = (last_agv_odom_pose_.inverse() * agv_odom_pose_).log();
    agv_translation_ += log.head<3>().norm();
    agv_rotation_ += log.tail<3>().norm();

    last_agv_odom_pose_ = agv_odom_pose_;
    last_agv_odom_time_sec_ = rclcpp::Time(msg->header.stamp).seconds();
}

void PoseOptimizationNode::uavOdomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    
    Eigen::Quaterniond q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    Eigen::Vector3d t(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    Sophus::SE3d current_pose(q, t);
    uav_odom_pose_ = uav_init_pose_ * current_pose;

    if (!last_uav_odom_initialized_) {
        last_uav_odom_time_sec_ = rclcpp::Time(msg->header.stamp).seconds();
        last_uav_odom_initialized_ = true;
        last_uav_odom_pose_ = uav_odom_pose_;
        return;
     }

    Eigen::Matrix<double, 6, 1> log = (last_uav_odom_pose_.inverse() * uav_odom_pose_).log();
    uav_translation_ += log.head<3>().norm();
    uav_rotation_ += log.tail<3>().norm();

    last_uav_odom_pose_ = uav_odom_pose_;
    last_uav_odom_time_sec_ = rclcpp::Time(msg->header.stamp).seconds();
}

void PoseOptimizationNode::pclAgvRadarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  
    if (!msg->data.empty()) {
        pcl::fromROSMsg(*msg, *agv_radar_cloud_);
        RCLCPP_DEBUG(this->get_logger(), "AGV cloud received with %zu points", agv_radar_cloud_->points.size());
        last_agv_radar_time_sec_ = rclcpp::Time(msg->header.stamp).seconds();
    } else {
        RCLCPP_WARN(this->get_logger(), "Empty source point cloud received!");
    }

}

void PoseOptimizationNode::pclUavRadarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!msg->data.empty()) {
        pcl::fromROSMsg(*msg, *uav_radar_cloud_);
        RCLCPP_DEBUG(this->get_logger(), "UAV cloud received with %zu points", uav_radar_cloud_->points.size());
        last_uav_radar_time_sec_ = rclcpp::Time(msg->header.stamp).seconds();
    } else {
        RCLCPP_WARN(this->get_logger(), "Empty target point cloud received!");
    }
}

void PoseOptimizationNode::AgvEgoVelCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
  
    Eigen::Vector3d raw(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Vector3d filtered = filterVelocities(agv_velocity_buffer_, raw, 10);
    agv_radar_egovel_ = *msg;
    agv_radar_egovel_.twist.twist.linear.x = filtered.x();
    agv_radar_egovel_.twist.twist.linear.y = filtered.y();
    agv_radar_egovel_.twist.twist.linear.z = filtered.z();
    last_agv_egovel_time_sec_ = rclcpp::Time(msg->header.stamp).seconds();

}

void PoseOptimizationNode::UavEgoVelCb(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
    
    Eigen::Vector3d raw(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Eigen::Vector3d filtered = filterVelocities(uav_velocity_buffer_, raw, 10);
    uav_radar_egovel_ = *msg;
    uav_radar_egovel_.twist.twist.linear.x = filtered.x();
    uav_radar_egovel_.twist.twist.linear.y = filtered.y();
    uav_radar_egovel_.twist.twist.linear.z = filtered.z();
    last_uav_egovel_time_sec_ = rclcpp::Time(msg->header.stamp).seconds();

}

void PoseOptimizationNode::optimizedTfCb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    
    latest_relative_pose_ = *msg;
    last_relative_pose_time_sec_ = rclcpp::Time(msg->header.stamp).seconds();

    if (!relative_pose_initialized_) {
        relative_pose_initialized_ = true;
        last_relative_pose_used_time_sec_ = last_relative_pose_time_sec_;
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Received optimized relative transform.");

    }

// --------------------- point cloud utilities --------------------- //

PoseOptimizationNode::CloudPtr PoseOptimizationNode::preprocessPointCloud(
    const CloudConstPtr& input_cloud, float /*meanK*/, float /*stdevmulthresh*/, float leaf_size) {
  // Example: only downsampling using small_gicp fast voxelgrid
  
    CloudPtr output = small_gicp::voxelgrid_sampling_omp(*input_cloud, leaf_size);
    RCLCPP_DEBUG(this->get_logger(), "Pointcloud downsampled to %zu points", output->points.size());
    return output;

}

Eigen::Matrix4d PoseOptimizationNode::computeICPCovariance(
    const CloudConstPtr& source, const CloudConstPtr& target,
    const Eigen::Matrix4f& transformation, double sensor_variance) {
  
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(target);

    Eigen::Matrix4d H_total = Eigen::Matrix4d::Zero();
    int count = 0;

    double yaw = std::atan2(transformation(1,0), transformation(0,0));

    for (size_t i = 0; i < source->points.size(); ++i) {
        const PointT& p = source->points[i];
        Eigen::Vector4f p_h(p.x, p.y, p.z, 1.0f);
        Eigen::Vector4f p_trans = transformation * p_h;

        PointT p_trans_p; p_trans_p.x = p_trans[0]; p_trans_p.y = p_trans[1]; p_trans_p.z = p_trans[2];

        std::vector<int> indices(1); std::vector<float> sqr_dists(1);
        if (tree.nearestKSearch(p_trans_p, 1, indices, sqr_dists) > 0) {
            const PointT& q = target->points[indices[0]];
            (void)q; // error vector not used for H

            Eigen::Matrix<double,3,4> J = Eigen::Matrix<double,3,4>::Zero();
            J.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
            double dxdtheta = -std::sin(yaw) * p.x - std::cos(yaw) * p.y;
            double dydtheta =  std::cos(yaw) * p.x - std::sin(yaw) * p.y;
            J(0,3) = dxdtheta; J(1,3) = dydtheta;

            H_total += J.transpose() * J;
            count++;
        }
    }

    if (count > 0) H_total /= static_cast<double>(count);
    else H_total = Eigen::Matrix4d::Identity();

    Eigen::Matrix4d cov_icp = sensor_variance * H_total.inverse();
    return cov_icp;

}

pcl::PointCloud<pcl::PointNormal>::Ptr
PoseOptimizationNode::computePointNormalCloud(const CloudConstPtr& cloud, float radius_search) const {

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(radius_search);
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    return cloud_with_normals;
}

bool PoseOptimizationNode::run_icp(const CloudConstPtr& source_cloud,
                                     const CloudConstPtr& target_cloud,
                                     CloudPtr& aligned_cloud,
                                     Eigen::Matrix4f& transformation,
                                     const double& pointcloud_sigma,
                                     double& fitness,
                                     const int& icp_type,
                                     Eigen::Matrix<double,6,6>& final_hessian,
                                     size_t& num_inliers) const {
    num_inliers = 0;
    if (icp_type == 2) {

        // small_gicp GICP
        small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ> reg;
        reg.setRegistrationType("GICP");
        reg.setNumThreads(4);
        reg.setRANSACIterations(15);
        reg.setRANSACOutlierRejectionThreshold(1.5);
        reg.setCorrespondenceRandomness(20);
        reg.setMaxCorrespondenceDistance(2.5 * pointcloud_sigma);
        reg.setMaximumIterations(100);
        reg.setTransformationEpsilon(1e-6);
        reg.setEuclideanFitnessEpsilon(2.5 * pointcloud_sigma);

        reg.setInputSource(source_cloud);
        reg.setInputTarget(target_cloud);

        CloudPtr aligned(new Cloud());
        reg.align(*aligned, transformation);

        if (!reg.hasConverged()) {
            RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
            return false;
        }

        transformation = reg.getFinalTransformation();
        fitness = reg.getFitnessScore();
        const auto& result = reg.getRegistrationResult();
        final_hessian = result.H;
        num_inliers = result.num_inliers;
        aligned_cloud = aligned;

        return true;

    } 
    
    else if (icp_type == 1) {

        // Point-to-plane ICP (PCL)
        auto src_n = computePointNormalCloud(source_cloud, 2.0f * pointcloud_sigma);
        auto tgt_n = computePointNormalCloud(target_cloud, 2.0f * pointcloud_sigma);

        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
        icp.setTransformationEstimation(
            pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>::Ptr(
                new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>));

        icp.setInputSource(src_n);
        icp.setInputTarget(tgt_n);

        pcl::registration::CorrespondenceRejectorTrimmed::Ptr rejector(new pcl::registration::CorrespondenceRejectorTrimmed);
        rejector->setOverlapRatio(0.7);
        icp.addCorrespondenceRejector(rejector);

        icp.setMaxCorrespondenceDistance(2.5 * pointcloud_sigma);
        icp.setMaximumIterations(64);
        icp.setTransformationEpsilon(1e-4);
        icp.setEuclideanFitnessEpsilon(2.5 * pointcloud_sigma);
        icp.setUseSymmetricObjective(true);

        pcl::PointCloud<pcl::PointNormal> aligned_cloud_normals;
        icp.align(aligned_cloud_normals, transformation);

        if (!icp.hasConverged()) return false;

        transformation = icp.getFinalTransformation();
        fitness = icp.getFitnessScore();

        CloudPtr aligned_xyz(new Cloud());
        pcl::copyPointCloud(aligned_cloud_normals, *aligned_xyz);
        aligned_cloud = aligned_xyz;
        return true;
    } 
    
    else {
        // Point-to-point ICP (PCL)
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(source_cloud);
        icp.setInputTarget(target_cloud);

        pcl::registration::CorrespondenceRejectorTrimmed::Ptr rejector(new pcl::registration::CorrespondenceRejectorTrimmed);
        rejector->setOverlapRatio(0.7);
        icp.addCorrespondenceRejector(rejector);

        icp.setMaxCorrespondenceDistance(2.5 * pointcloud_sigma);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-4);
        icp.setEuclideanFitnessEpsilon(2.5 * pointcloud_sigma);

        CloudPtr aligned(new Cloud());
        icp.align(*aligned, transformation);

        if (!icp.hasConverged()) return false;

        transformation = icp.getFinalTransformation();
        fitness = icp.getFitnessScore();
        aligned_cloud = aligned;
        return true;
    }
} 

Sophus::SE3d PoseOptimizationNode::integrateEgoVelIntoSE3(
        const Eigen::Vector3d& radar_vel_t,     // body-frame velocity at time t
        const Sophus::SE3d& odom_T_s,           // odom at source time (earlier)
        const Sophus::SE3d& odom_T_t,           // odom at target time (later)
        double dt){
        // Keep odometry relative rotation and blend odometry translation with
        // velocity-integrated translation in target/body coordinates.
        const Sophus::SE3d t_T_s_odom = odom_T_t.inverse() * odom_T_s;
        const Eigen::Vector3d t_odom = t_T_s_odom.translation();
        const Eigen::Vector3d t_vel = -radar_vel_t * dt;

        constexpr double kVelBlend = 0.4;  // 0 -> odom only, 1 -> velocity only
        const Eigen::Vector3d t_blended = (1.0 - kVelBlend) * t_odom + kVelBlend * t_vel;

        return Sophus::SE3d{t_T_s_odom.unit_quaternion(), t_blended};
    }

// ---------------- Pose Graph Optimization -------------------
//
// The pose graph fuses:
// - A prior on each robot's first node (while it is still in the window).
// - Intra-robot odometry and radar (ICP) factors linking nodes.
// - Inter-robot UWB factors: each new relative transform estimate links the
//   latest node of each robot through the two anchor nodes (added once).
// - Inter-robot radar (ICP) encounter factors.
//
// The node only gathers these inputs (PoseGraphInput); building and solving
// the problem is done by the configured backend (posegraph_backend.hpp):
// Ceres or GTSAM. With posegraph_compare_backend set, a second backend also
// solves a copy of the same inputs every step, for comparison only.

PoseGraphInput PoseOptimizationNode::makeGraphInput(MapOfStates &agv_map, MapOfStates &uav_map,
                                                    State &anchor_agv, State &anchor_uav) const {
    PoseGraphInput in;
    in.agv_map = &agv_map;
    in.uav_map = &uav_map;
    in.anchor_agv = &anchor_agv;
    in.anchor_uav = &anchor_uav;
    in.agv_current_id = agv_id_;
    in.uav_current_id = uav_id_;
    in.max_keyframes = max_keyframes_;
    in.min_keyframes = min_keyframes_;
    in.prior_agv = &prior_agv_;
    in.prior_uav = &prior_uav_;
    in.prior_anchor_agv = &prior_anchor_agv_;
    in.odom_agv = &proprioceptive_constraints_agv_;
    in.odom_uav = &proprioceptive_constraints_uav_;
    in.radar_agv = &extraceptive_constraints_agv_;
    in.radar_uav = &extraceptive_constraints_uav_;
    in.encounters_uwb = &encounter_constraints_uwb_;
    in.encounters_radar = &encounter_constraints_pointcloud_;
    return in;
}

namespace {

// Neither backend wraps yaw during the solve (the residuals are 2*pi-periodic
// in yaw), so bring the optimized yaws back to [-pi, pi].
void normalizeYaws(posegraph::MapOfStates &agv_map, posegraph::MapOfStates &uav_map,
                   State &anchor_agv, State &anchor_uav) {
    for (auto &kv : agv_map) kv.second.state[3] = normalizeAngle(kv.second.state[3]);
    for (auto &kv : uav_map) kv.second.state[3] = normalizeAngle(kv.second.state[3]);
    anchor_agv.state[3] = normalizeAngle(anchor_agv.state[3]);
    anchor_uav.state[3] = normalizeAngle(anchor_uav.state[3]);
}

// Largest difference between two solutions of the same graph: position (m),
// yaw (rad, wrapped) and relative covariance difference ||C_a - C_b|| / ||C_a||.
void compareSolutions(const posegraph::MapOfStates &agv_a, const posegraph::MapOfStates &uav_a,
                      const State &anchor_agv_a, const State &anchor_uav_a,
                      const posegraph::MapOfStates &agv_b, const posegraph::MapOfStates &uav_b,
                      const State &anchor_agv_b, const State &anchor_uav_b,
                      double &max_dpos, double &max_dyaw, double &max_dcov) {
    max_dpos = max_dyaw = max_dcov = 0.0;
    auto cmp = [&](const State &a, const State &b) {
        max_dpos = std::max(max_dpos, (a.state.head<3>() - b.state.head<3>()).norm());
        max_dyaw = std::max(max_dyaw, std::abs(normalizeAngle(a.state[3] - b.state[3])));
        const double n = a.covariance.norm();
        if (n > 0.0) max_dcov = std::max(max_dcov, (a.covariance - b.covariance).norm() / n);
    };
    for (const auto &kv : agv_a) cmp(kv.second, agv_b.at(kv.first));
    for (const auto &kv : uav_a) cmp(kv.second, uav_b.at(kv.first));
    cmp(anchor_agv_a, anchor_agv_b);
    cmp(anchor_uav_a, anchor_uav_b);
}

}  // namespace

bool PoseOptimizationNode::runPosegraphOptimization() {

    // Comparison backend: solves a copy of the same inputs (taken before the
    // main solve modifies them); its result is only compared, never used.
    MapOfStates agv_copy, uav_copy;
    State anchor_agv_copy, anchor_uav_copy;
    if (compare_backend_) {
        agv_copy = agv_map_; uav_copy = uav_map_;
        anchor_agv_copy = anchor_node_agv_; anchor_uav_copy = anchor_node_uav_;
    }

    BackendStepRecord record;
    record.time = this->get_clock()->now().seconds();
    record.agv_id = agv_id_;
    record.uav_id = uav_id_;

    PoseGraphInput in = makeGraphInput(agv_map_, uav_map_, anchor_node_agv_, anchor_node_uav_);
    const bool converged = backend_->optimize(in, record.main);

    if (compare_backend_) {
        PoseGraphInput in_copy = makeGraphInput(agv_copy, uav_copy, anchor_agv_copy, anchor_uav_copy);
        const bool converged_copy = compare_backend_->optimize(in_copy, record.compare);
        record.has_compare = true;
        if (converged && converged_copy) {
            normalizeYaws(agv_copy, uav_copy, anchor_agv_copy, anchor_uav_copy);
            MapOfStates agv_main = agv_map_, uav_main = uav_map_;
            State anchor_agv_main = anchor_node_agv_, anchor_uav_main = anchor_node_uav_;
            normalizeYaws(agv_main, uav_main, anchor_agv_main, anchor_uav_main);
            compareSolutions(agv_main, uav_main, anchor_agv_main, anchor_uav_main,
                             agv_copy, uav_copy, anchor_agv_copy, anchor_uav_copy,
                             record.max_dpos, record.max_dyaw, record.max_dcov);
            RCLCPP_INFO(this->get_logger(),
                        "[%s vs %s] solve %.4f / %.4f s, iterations %d / %d, cost %.6g / %.6g, "
                        "max diff: pos %.2e m, yaw %.2e rad, cov %.2e (rel)",
                        backend_->name().c_str(), compare_backend_->name().c_str(),
                        record.main.solve_time, record.compare.solve_time,
                        record.main.iterations, record.compare.iterations,
                        record.main.final_cost, record.compare.final_cost,
                        record.max_dpos, record.max_dyaw, record.max_dcov);
        }
    }
    backend_records_.push_back(record);

    if (!converged) {
        RCLCPP_WARN(this->get_logger(), "Failed to converge.");
        return false;
    }

    normalizeYaws(agv_map_, uav_map_, anchor_node_agv_, anchor_node_uav_);

    total_solves_++;
    total_solver_time_ += record.main.solve_time;
    solver_times_.push_back(record.main.solve_time);
    return true;
}

void PoseOptimizationNode::writePoseGraphDump(const std::string &filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) return;
    file << std::setprecision(17);

    auto pose = [&](const Sophus::SE3d &T) {
        const Eigen::Quaterniond q = T.unit_quaternion();
        file << " " << T.translation().x() << " " << T.translation().y() << " " << T.translation().z()
             << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    };
    auto cov = [&](const Eigen::Matrix4d &C) {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) file << " " << C(i, j);
    };
    auto state = [&](const State &s, bool planar) {
        file << " " << planar << " " << s.roll << " " << s.pitch << " "
             << s.state[0] << " " << s.state[1] << " " << s.state[2] << " " << s.state[3] << "\n";
    };

    for (const auto &[id, s] : agv_map_) { file << "node a " << id; state(s, s.planar); }
    for (const auto &[id, s] : uav_map_) { file << "node u " << id; state(s, s.planar); }
    // The AGV anchor is planar, the UAV anchor 4-DoF (as in the backends).
    file << "anchor a"; state(anchor_node_agv_, true);
    file << "anchor u"; state(anchor_node_uav_, false);

    auto prior = [&](const char *name, const PriorConstraint &p) {
        file << "prior " << name; pose(p.pose); cov(p.covariance); file << "\n";
    };
    prior("agv", prior_agv_);
    prior("uav", prior_uav_);
    prior("anchor_agv", prior_anchor_agv_);

    auto factors = [&](const char *group, const VectorOfConstraints &v) {
        for (const auto &c : v) {
            file << "factor " << group << " " << c.id_begin << " " << c.id_end;
            pose(c.t_T_s); cov(c.covariance); file << "\n";
        }
    };
    factors("odom_agv", proprioceptive_constraints_agv_);
    factors("odom_uav", proprioceptive_constraints_uav_);
    factors("radar_agv", extraceptive_constraints_agv_);
    factors("radar_uav", extraceptive_constraints_uav_);
    factors("uwb", encounter_constraints_uwb_);
    factors("radar_encounter", encounter_constraints_pointcloud_);
}

void PoseOptimizationNode::writeBackendStats(const std::string &filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) return;

    auto header = [&](const std::string &p) {
        file << "," << p << "converged," << p << "iterations," << p << "factors," << p << "variables,"
             << p << "initial_cost," << p << "final_cost," << p << "build_time," << p << "solve_time,"
             << p << "covariance_time," << p << "covariance_ok";
    };
    auto row = [&](const PoseGraphSolveStats &s) {
        file << "," << s.converged << "," << s.iterations << "," << s.num_factors << "," << s.num_variables
             << "," << s.initial_cost << "," << s.final_cost << "," << s.build_time << "," << s.solve_time
             << "," << s.covariance_time << "," << s.covariance_ok;
    };

    file << "time,agv_id,uav_id";
    header(backend_->name() + "_");
    if (compare_backend_) {
        header(compare_backend_->name() + "_");
        file << ",max_dpos,max_dyaw,max_dcov";
    }
    file << "\n" << std::setprecision(10);

    for (const auto &r : backend_records_) {
        file << r.time << "," << r.agv_id << "," << r.uav_id;
        row(r.main);
        if (compare_backend_) {
            row(r.compare);
            file << "," << r.max_dpos << "," << r.max_dyaw << "," << r.max_dcov;
        }
        file << "\n";
    }
}

// --------------------- visualization service --------------------- //
void PoseOptimizationNode::sendPointCloudServiceRequest(const CloudConstPtr& source_cloud,
                                                          const CloudConstPtr& target_cloud,
                                                          const CloudConstPtr& aligned_cloud) {
    sensor_msgs::msg::PointCloud2 source_msg, target_msg, aligned_msg;
    pcl::toROSMsg(*source_cloud, source_msg);
    pcl::toROSMsg(*target_cloud, target_msg);
    pcl::toROSMsg(*aligned_cloud, aligned_msg);

    auto request = std::make_shared<UpdatePointClouds::Request>();
    request->source_cloud = source_msg;
    request->target_cloud = target_msg;
    request->aligned_cloud = aligned_msg;

    auto cb = [this](rclcpp::Client<UpdatePointClouds>::SharedFuture result) {
    if (result.get()->success) {
        RCLCPP_INFO(this->get_logger(), "Visualizer updated successfully.");
    } else {
        RCLCPP_WARN(this->get_logger(), "Visualizer update failed: %s", result.get()->message.c_str());
    }
    };

    (void)pcl_visualizer_client_->async_send_request(request, cb);
}

// --------------------- point cloud constraint builder --------------------- //
bool PoseOptimizationNode::addPointCloudConstraint(
    const CloudConstPtr& source_scan, const CloudConstPtr& target_scan,
    const Sophus::SE3d& T_source_sensor, const Sophus::SE3d& T_target_sensor,
    double sigma, int icp_type, int id_source, int id_target,
    VectorOfConstraints& constraints_list, bool send_visualization,
    const Eigen::Matrix4f* initial_guess) {

    auto source_body = transformCloudToBody(source_scan, T_source_sensor);
    auto target_body = transformCloudToBody(target_scan, T_target_sensor);

    Eigen::Matrix4f T_icp = (initial_guess != nullptr) ? *initial_guess : Eigen::Matrix4f::Identity();
    Eigen::Matrix<double,6,6> final_hessian = Eigen::Matrix<double,6,6>::Identity() * 1e-6;
    CloudPtr aligned(new Cloud);
    double fitness = 0.0;
    size_t num_inliers = 0;

    bool ok = run_icp(source_body, target_body, aligned, T_icp, sigma, fitness, icp_type,
                      final_hessian, num_inliers);
    if (!ok) return false;

    Sophus::SE3d T_icp_d = Sophus::SE3f(T_icp).cast<double>();

    MeasurementConstraint c; 
    
    c.id_begin = id_source; 
    c.id_end = id_target; 
    c.t_T_s = T_icp_d;

    if (icp_type == 2) {

        // Accept only alignments with enough matched points.
        if (static_cast<int>(num_inliers) < radar_min_inliers_) {
            RCLCPP_DEBUG(this->get_logger(), "GICP rejected: %zu inliers (< %d)", num_inliers, radar_min_inliers_);
            return false;
        }

        // Factor covariance: a fixed sensor-noise heuristic, std radar_stdev for x/y/z and
        // radar_yaw_stdev for yaw. On the real-world dataset this beat both the GICP Hessian
        // (overconfident for sparse radar scans) and a residual-scaled variant; the robust
        // loss handles the outlier tail.
        c.covariance = Eigen::Vector4d(sigma * sigma, sigma * sigma, sigma * sigma,
                                       radar_yaw_stdev_ * radar_yaw_stdev_).asDiagonal();
        // Radar elevation is far less reliable than range/azimuth: inflate the z variance.
        c.covariance(2,2) *= 10.0;
        RCLCPP_INFO(this->get_logger(),
                    "GICP converged with score: %f, inliers: %zu, std [x y z yaw]: [%.3f %.3f %.3f %.3f]",
                    fitness, num_inliers,
                    std::sqrt(c.covariance(0,0)), std::sqrt(c.covariance(1,1)),
                    std::sqrt(c.covariance(2,2)), std::sqrt(c.covariance(3,3)));
        logTransformationMatrix(c.t_T_s.matrix(), this->get_logger());
    } 

    else {

        c.covariance = computeICPCovariance(source_scan, target_scan, T_icp, sigma);
    }

    constraints_list.push_back(c);
    if (send_visualization) sendPointCloudServiceRequest(source_scan, target_scan, aligned);
    return true;
}


void PoseOptimizationNode::globalOptCb() {

        rclcpp::Time current_time = this->get_clock()->now();
        double current_time_sec = current_time.seconds();

        //********************INITIALIZATIONS*********************** */
        if(!last_agv_odom_initialized_ || !last_uav_odom_initialized_){
            return;
        }
        if(!graph_initialized_){

            RCLCPP_INFO(this->get_logger(), "Initializing graph!");

            //Get first measurements
            agv_measurements_.timestamp = current_time;
            // Store a copy per keyframe: the radar history keeps these pointers, so
            // reusing one cloud object would make every "older" scan the newest one.
            agv_measurements_.radar_scan = std::make_shared<Cloud>(*agv_radar_cloud_);
            // 2) grab latest radar ego-velocity from your subscriber
            agv_measurements_.radar_egovel = Eigen::Vector3d{
                agv_radar_egovel_.twist.twist.linear.x,
                agv_radar_egovel_.twist.twist.linear.y,
                agv_radar_egovel_.twist.twist.linear.z
            };
            agv_measurements_.odom_pose = agv_odom_pose_;

            agv_measurements_.odom_ok  = current_time_sec - last_agv_odom_time_sec_ <= measurement_sync_thr_;
            agv_measurements_.radar_ok = current_time_sec - last_agv_radar_time_sec_ <= measurement_sync_thr_;
            agv_measurements_.radar_velocity_ok = current_time_sec - last_agv_egovel_time_sec_ <= measurement_sync_thr_;

            RadarMeasurements radar_agv;
            radar_agv.KF_id = agv_id_;
            radar_agv.odom_pose = agv_measurements_.odom_pose;
            radar_agv.radar_scan = agv_measurements_.radar_scan;
            radar_agv.timestamp = agv_measurements_.timestamp;
            radar_agv.radar_egovel = agv_measurements_.radar_egovel;
            radar_history_agv_.push_back(radar_agv);


            Eigen::Vector3d t_odom_agv = agv_measurements_.odom_pose.translation();
            Eigen::Matrix3d R_odom_agv = agv_measurements_.odom_pose.rotationMatrix();  // or T.so3().matrix()
            // Compute Euler angles in ZYX order: [yaw, pitch, roll]
            Eigen::Vector3d euler_agv = R_odom_agv.eulerAngles(2, 1, 0);

            //Initial values for state AGV
            init_state_agv_.timestamp = current_time;
            init_state_agv_.state = Eigen::Vector4d(t_odom_agv[0], t_odom_agv[1], t_odom_agv[2], euler_agv[0]);
            init_state_agv_.roll = euler_agv[2];
            init_state_agv_.pitch = euler_agv[1];
            init_state_agv_.pose = buildTransformationSE3(init_state_agv_.roll, init_state_agv_.pitch, init_state_agv_.state);
            init_state_agv_.covariance = Eigen::Matrix4d::Identity(); //

            RCLCPP_INFO(this->get_logger(), "Adding initial AGV node at timestamp %.2f: [%f, %f, %f, %f]", current_time.seconds(),
            init_state_agv_.state[0], init_state_agv_.state[1], init_state_agv_.state[2], init_state_agv_.state[3]);

            agv_map_[agv_id_] = init_state_agv_;

            //Anchor node for AGV
            anchor_node_agv_.timestamp = current_time;
            anchor_node_agv_.state = transformSE3ToState(T_agv_anchor_prior_);
            anchor_node_agv_.roll = 0.0;
            anchor_node_agv_.pitch = 0.0;
            anchor_node_agv_.pose = buildTransformationSE3(anchor_node_agv_.roll, anchor_node_agv_.pitch, anchor_node_agv_.state);
            anchor_node_agv_.covariance = Eigen::Matrix4d::Identity(); //
            anchor_node_agv_.planar = false;

            RCLCPP_INFO(this->get_logger(), "Initializing AGV anchor to %.2f: [%f, %f, %f, %f]", current_time.seconds(),
            anchor_node_agv_.state[0], anchor_node_agv_.state[1], anchor_node_agv_.state[2], anchor_node_agv_.state[3]);

            agv_translation_ = agv_rotation_ = 0.0;
            prev_agv_measurements_ = agv_measurements_;

            uav_measurements_.timestamp = current_time;
            // Store a copy per keyframe: the radar history keeps these pointers, so
            // reusing one cloud object would make every "older" scan the newest one.
            uav_measurements_.radar_scan = std::make_shared<Cloud>(*uav_radar_cloud_);
            uav_measurements_.radar_egovel = Eigen::Vector3d{
                uav_radar_egovel_.twist.twist.linear.x,
                uav_radar_egovel_.twist.twist.linear.y,
                uav_radar_egovel_.twist.twist.linear.z
            };
            uav_measurements_.odom_pose = uav_odom_pose_;

            uav_measurements_.odom_ok  = current_time_sec - last_uav_odom_time_sec_ <= measurement_sync_thr_;
            uav_measurements_.radar_ok = current_time_sec - last_uav_radar_time_sec_ <= measurement_sync_thr_;
            uav_measurements_.radar_velocity_ok = current_time_sec - last_uav_egovel_time_sec_ <= measurement_sync_thr_;

            RadarMeasurements radar_uav;
            radar_uav.KF_id = uav_id_;
            radar_uav.odom_pose = uav_measurements_.odom_pose;
            radar_uav.radar_scan = uav_measurements_.radar_scan;
            radar_uav.timestamp = uav_measurements_.timestamp;
            radar_uav.radar_egovel = uav_measurements_.radar_egovel;
            radar_history_uav_.push_back(radar_uav);

        
            Eigen::Vector3d t_odom_uav = uav_measurements_.odom_pose.translation();
            Eigen::Matrix3d R_odom_uav = uav_measurements_.odom_pose.rotationMatrix();  // or T.so3().matrix()
            Eigen::Vector3d euler_uav = R_odom_uav.eulerAngles(2, 1, 0);

            //Initial values for state UAV
            init_state_uav_.timestamp = current_time;
            init_state_uav_.state = Eigen::Vector4d(t_odom_uav[0], t_odom_uav[1], t_odom_uav[2], euler_uav[0]);
            init_state_uav_.roll = euler_uav[2];
            init_state_uav_.pitch = euler_uav[1];
            init_state_uav_.pose = buildTransformationSE3(init_state_uav_.roll, init_state_uav_.pitch, init_state_uav_.state);
            init_state_uav_.covariance = Eigen::Matrix4d::Identity(); //
            init_state_uav_.planar = false;

            RCLCPP_INFO(this->get_logger(), "Adding initial UAV node at timestamp %.2f: [%f, %f, %f, %f]", current_time.seconds(),
            init_state_uav_.state[0], init_state_uav_.state[1], init_state_uav_.state[2], init_state_uav_.state[3]);

            uav_map_[uav_id_] = init_state_uav_;

            uav_radar_odom_pose_ = Sophus::SE3d(Eigen::Matrix4d::Identity());

            //Create anchor node for UAV
            anchor_node_uav_.timestamp = current_time;
            anchor_node_uav_.state = transformSE3ToState(T_uav_anchor_prior_);
            anchor_node_uav_.roll = 0.0;
            anchor_node_uav_.pitch = 0.0;
            anchor_node_uav_.pose = buildTransformationSE3(anchor_node_uav_.roll, anchor_node_uav_.pitch, anchor_node_uav_.state);
            anchor_node_uav_.covariance = Eigen::Matrix4d::Identity(); //
            anchor_node_uav_.planar = false;

            uav_translation_ = uav_rotation_ = 0.0;
            prev_uav_measurements_ = uav_measurements_;

            //Initialize prior constraints
            
            //AGV local trajectory prior
            prior_agv_.pose = init_state_agv_.pose;
            prior_agv_.covariance = Eigen::Matrix4d::Identity() * 1e-6;

            //Anchor AGV prior - we use this one
            prior_anchor_agv_.pose = T_agv_anchor_prior_;
            prior_anchor_agv_.covariance = Eigen::Matrix4d::Identity() * 1e-6;

            //UAV local trajectory prior
            prior_uav_.pose = init_state_uav_.pose;
            prior_uav_.covariance = Eigen::Matrix4d::Identity() * 1e-6;

            //Anchor UAV prior - not used currently
            prior_anchor_uav_.pose = T_uav_anchor_prior_;
            prior_anchor_uav_.covariance = Eigen::Matrix4d::Identity() * 1e-6;

            graph_initialized_ = true;

            graph_start_time_ = current_time;

            RCLCPP_INFO(this->get_logger(), "Initialization done!");

            return;
        }

        //********************ADDING NEW NODES*********************** */

        bool new_agv_node = (agv_translation_ >= min_traveled_distance_ || agv_rotation_ >= min_traveled_angle_);
        bool new_uav_node = (uav_translation_ >= min_traveled_distance_ || uav_rotation_ >= min_traveled_angle_);

        if(!new_agv_node && !new_uav_node){

            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Neither robot moved enough, skipping optimization...");
            return;
        }
        
        if (new_agv_node) {

            agv_id_++;
            
            agv_measurements_.timestamp = current_time;
            agv_measurements_.radar_scan = std::make_shared<Cloud>(*agv_radar_cloud_);  // own copy per keyframe
            agv_measurements_.radar_egovel = Eigen::Vector3d{
                agv_radar_egovel_.twist.twist.linear.x,
                agv_radar_egovel_.twist.twist.linear.y,
                agv_radar_egovel_.twist.twist.linear.z
            };

            agv_measurements_.odom_pose = agv_odom_pose_;

            agv_measurements_.odom_ok  = current_time_sec - last_agv_odom_time_sec_ <= measurement_sync_thr_;
            agv_measurements_.radar_ok = current_time_sec - last_agv_radar_time_sec_ <= measurement_sync_thr_;
            agv_measurements_.radar_velocity_ok = current_time_sec - last_agv_egovel_time_sec_ <= measurement_sync_thr_;

            RadarMeasurements radar_agv;
            radar_agv.KF_id = agv_id_;
            radar_agv.odom_pose = agv_measurements_.odom_pose;
            radar_agv.radar_scan = agv_measurements_.radar_scan;
            radar_agv.timestamp = agv_measurements_.timestamp;
            radar_agv.radar_egovel = agv_measurements_.radar_egovel;
            radar_history_agv_.push_back(radar_agv);
            if (radar_history_agv_.size() > radar_history_size_) {
                radar_history_agv_.pop_front();
            }

            // Create a new AGV node from the current odometry.
            State new_agv;
            new_agv.timestamp = current_time;

            Eigen::Vector3d t_odom_agv = agv_measurements_.odom_pose.translation();
            Eigen::Matrix3d R_odom_agv = agv_measurements_.odom_pose.rotationMatrix();  // or T.so3().matrix()
            // Compute Euler angles in ZYX order: [yaw, pitch, roll]
            Eigen::Vector3d euler_agv = R_odom_agv.eulerAngles(2, 1, 0);

            new_agv.pitch = euler_agv[1];  // rotation around Y-axis
            new_agv.roll = euler_agv[2];  // rotation around X-axis
            
            double tilt = std::acos( R_odom_agv(2,2) );  // dot(body_z, world_z)
            new_agv.planar = ( tilt <= 15.0 * M_PI/180.0 );

            new_agv.state = Eigen::Vector4d(t_odom_agv[0], t_odom_agv[1], t_odom_agv[2], euler_agv[0]);
            new_agv.pose = buildTransformationSE3(new_agv.roll, new_agv.pitch, new_agv.state);
            new_agv.covariance = Eigen::Matrix4d::Identity();
            
            agv_map_[agv_id_] = new_agv;

            RCLCPP_INFO(this->get_logger(), "Adding AGV node %d at timestamp %.2f: [%f, %f, %f, %f] with %s motion", agv_id_, current_time.seconds(),
                            new_agv.state[0], new_agv.state[1], new_agv.state[2], new_agv.state[3], new_agv.planar ? "planar" : "vertical");


            //ADD AGV Proprioceptive constraints
            
            //AGV odom constraints

            if(using_odom_ && agv_measurements_.odom_ok){
                MeasurementConstraint constraint_odom_agv;
                constraint_odom_agv.id_begin = agv_id_ - 1;
                constraint_odom_agv.id_end = agv_id_;

                Sophus::SE3d odom_T_s_agv = prev_agv_measurements_.odom_pose;
                Sophus::SE3d odom_T_t_agv = agv_measurements_.odom_pose;
                constraint_odom_agv.t_T_s = odom_T_t_agv.inverse()*odom_T_s_agv;

                constraint_odom_agv.covariance = odom_noise_agv_.covariance();

                proprioceptive_constraints_agv_.push_back(constraint_odom_agv);
            }

            //AGV Radar ICP constraints

            if (using_radar_ && !agv_measurements_.radar_scan->points.empty() && agv_measurements_.radar_ok) {

                    for (const auto &olderKF : radar_history_agv_) {
                        // Skip the current KF if you wish
                        if (olderKF.KF_id == radar_agv.KF_id) continue;

                        RCLCPP_WARN(this->get_logger(), "Computing Radar ICP for AGV nodes %d and %d", olderKF.KF_id, radar_agv.KF_id);

                        Eigen::Matrix4f T_icp = Eigen::Matrix4f::Identity();
                        bool has_odom_guess = false;
                        // Optionally, compute an initial guess based on the relative odometry:
                        if(using_odom_ && agv_measurements_.odom_ok) {
                            T_icp = (radar_agv.odom_pose.inverse() * olderKF.odom_pose).cast<float>().matrix();
                            has_odom_guess = true;
                        }

                        if (agv_measurements_.radar_velocity_ok) {
                            const double dt_agv = (agv_measurements_.timestamp - olderKF.timestamp).seconds();
                            if (dt_agv > 0.0) {
                                Eigen::Vector3d v_t = T_agv_imu_.rotationMatrix() * agv_measurements_.radar_egovel;
                                // Convert older-keyframe velocity into target frame and average.
                                const Eigen::Matrix3d R_t_s = (radar_agv.odom_pose.inverse() * olderKF.odom_pose).rotationMatrix();
                                Eigen::Vector3d v_s = T_agv_imu_.rotationMatrix() * olderKF.radar_egovel;
                                Eigen::Vector3d v_pair_t = 0.5 * (v_t + R_t_s * v_s);

                                Eigen::Matrix4f T_vel = integrateEgoVelIntoSE3(
                                    v_pair_t,
                                    olderKF.odom_pose,
                                    agv_measurements_.odom_pose,
                                    dt_agv
                                ).cast<float>().matrix();

                                if (has_odom_guess) {
                                    // Keep odometry rotation; blend translations.
                                    T_icp.block<3,1>(0,3) = 0.6f * T_icp.block<3,1>(0,3) + 0.4f * T_vel.block<3,1>(0,3);
                                } else {
                                    T_icp = T_vel;
                                }
                            }
                        }
                        
                        //If you are using radar pre-filter, filtered clouds come in the IMU frame
                        if (!addPointCloudConstraint(olderKF.radar_scan, radar_agv.radar_scan,
                                                    T_agv_imu_, T_agv_imu_, pointcloud_radar_sigma_,
                                                    icp_type_radar_,
                                                    olderKF.KF_id, radar_agv.KF_id,
                                                    extraceptive_constraints_agv_, /*send_visualization=*/false,
                                                    &T_icp))
                        {
                            RCLCPP_WARN(this->get_logger(), "Failed to add radar constraint between KF %d and KF %d", olderKF.KF_id, radar_agv.KF_id);
                        }
                    }
            }

            prev_agv_measurements_ = agv_measurements_;
            agv_translation_ = agv_rotation_ = 0.0;

        }

        if (new_uav_node) {

            uav_id_++;

            uav_measurements_.timestamp = current_time;

            uav_measurements_.radar_scan = std::make_shared<Cloud>(*uav_radar_cloud_);  // own copy per keyframe
            uav_measurements_.radar_egovel = Eigen::Vector3d{
                uav_radar_egovel_.twist.twist.linear.x,
                uav_radar_egovel_.twist.twist.linear.y,
                uav_radar_egovel_.twist.twist.linear.z
            };
            uav_measurements_.odom_pose = uav_odom_pose_;

            uav_measurements_.odom_ok  = current_time_sec - last_uav_odom_time_sec_ <= measurement_sync_thr_;
            uav_measurements_.radar_ok = current_time_sec - last_uav_radar_time_sec_ <= measurement_sync_thr_;
            uav_measurements_.radar_velocity_ok = current_time_sec - last_uav_egovel_time_sec_ <= measurement_sync_thr_;

            RadarMeasurements radar_uav;
            radar_uav.KF_id = uav_id_;
            radar_uav.odom_pose = uav_measurements_.odom_pose;
            radar_uav.radar_scan = uav_measurements_.radar_scan;
            radar_uav.timestamp = uav_measurements_.timestamp;
            radar_uav.radar_egovel = uav_measurements_.radar_egovel;
            radar_history_uav_.push_back(radar_uav);
            if (radar_history_uav_.size() > radar_history_size_) {
                radar_history_uav_.pop_front();
            }

            // Similarly, create a new UAV node.
            State new_uav;  
            new_uav.timestamp = current_time;

            Eigen::Vector3d t_odom_uav = uav_measurements_.odom_pose.translation();
            Eigen::Vector3d prev_t_odom_uav = prev_uav_measurements_.odom_pose.translation();

            Eigen::Matrix3d R_odom_uav = uav_measurements_.odom_pose.rotationMatrix();  // or T.so3().matrix()
            // Compute Euler angles in ZYX order: [yaw, pitch, roll]
            Eigen::Vector3d euler_uav = R_odom_uav.eulerAngles(2, 1, 0);

            new_uav.pitch = euler_uav[1];  // rotation around Y-axis
            new_uav.roll = euler_uav[2];  // rotation around X-axis
            
            new_uav.planar = false;

            new_uav.state = Eigen::Vector4d(t_odom_uav[0], t_odom_uav[1], t_odom_uav[2], euler_uav[0]);
            new_uav.pose = buildTransformationSE3(new_uav.roll, new_uav.pitch, new_uav.state);
            new_uav.covariance = Eigen::Matrix4d::Identity();

            uav_map_[uav_id_] = new_uav;

            RCLCPP_INFO(this->get_logger(), "Adding new UAV node %d at timestamp %.2f: [%f, %f, %f, %f] with %s motion", uav_id_, current_time.seconds(),
                        new_uav.state[0], new_uav.state[1], new_uav.state[2], new_uav.state[3], new_uav.planar ? "planar" : "vertical");

            //ADD UAV Proprioceptive constraints

            if(using_odom_ && uav_measurements_.odom_ok){
                //UAV odom constraints
                MeasurementConstraint constraint_odom_uav;
                constraint_odom_uav.id_begin = uav_id_ - 1;
                constraint_odom_uav.id_end = uav_id_;

                Sophus::SE3d odom_T_s_uav = prev_uav_measurements_.odom_pose;
                Sophus::SE3d odom_T_t_uav = uav_measurements_.odom_pose;
                constraint_odom_uav.t_T_s = odom_T_t_uav.inverse()*odom_T_s_uav;

                constraint_odom_uav.covariance = odom_noise_uav_.covariance();
                proprioceptive_constraints_uav_.push_back(constraint_odom_uav);
            }

            //UAV Radar ICP constraints
            if (using_radar_ && !uav_measurements_.radar_scan->points.empty() && uav_measurements_.radar_ok) {
                    
                    for (const auto &olderKF : radar_history_uav_) {
                        // Skip the current KF if you wish
                        if (olderKF.KF_id == radar_uav.KF_id) continue;
                        
                        RCLCPP_WARN(this->get_logger(), "Computing Radar ICP for UAV nodes %d and %d", olderKF.KF_id, radar_uav.KF_id);

                        Eigen::Matrix4f T_icp = Eigen::Matrix4f::Identity();
                        bool has_odom_guess = false;
                        // Optionally, compute an initial guess based on the relative odometry:
                        if(using_odom_ && uav_measurements_.odom_ok) {
                            T_icp = (radar_uav.odom_pose.inverse() * olderKF.odom_pose).cast<float>().matrix();
                            has_odom_guess = true;
                        }
                        
                        if (uav_measurements_.radar_velocity_ok) {
                            const double dt_uav = (uav_measurements_.timestamp - olderKF.timestamp).seconds();
                            if (dt_uav > 0.0) {
                                Eigen::Vector3d v_t = T_uav_imu_.rotationMatrix() * uav_measurements_.radar_egovel;
                                const Eigen::Matrix3d R_t_s = (radar_uav.odom_pose.inverse() * olderKF.odom_pose).rotationMatrix();
                                Eigen::Vector3d v_s = T_uav_imu_.rotationMatrix() * olderKF.radar_egovel;
                                Eigen::Vector3d v_pair_t = 0.5 * (v_t + R_t_s * v_s);

                                Eigen::Matrix4f T_vel = integrateEgoVelIntoSE3(
                                    v_pair_t,
                                    olderKF.odom_pose,
                                    uav_measurements_.odom_pose,
                                    dt_uav
                                ).cast<float>().matrix();

                                if (has_odom_guess) {
                                    T_icp.block<3,1>(0,3) = 0.6f * T_icp.block<3,1>(0,3) + 0.4f * T_vel.block<3,1>(0,3);
                                } else {
                                    T_icp = T_vel;
                                }
                            }
                        }
                        
                        //If you are using radar pre-filter, filtered clouds come in the IMU frame
                        if (!addPointCloudConstraint(olderKF.radar_scan, radar_uav.radar_scan,
                                                    T_uav_imu_, T_uav_imu_, pointcloud_radar_sigma_,
                                                    icp_type_radar_,
                                                    olderKF.KF_id, radar_uav.KF_id,
                                                    extraceptive_constraints_uav_, false,
                                                    &T_icp))
                        {
                            RCLCPP_WARN(this->get_logger(), "Failed to add radar constraint between KF %d and KF %d", olderKF.KF_id, radar_uav.KF_id);
                        }
                    }

            }

            prev_uav_measurements_ = uav_measurements_;
            uav_translation_ = uav_rotation_ = 0.0;
        }

        //////////////// MANAGE ENCOUNTERS ///////////////

        //Check if there is a new relative position available //TODO: second condition would be new value has arrived
        uwb_transform_available_ = relative_pose_initialized_ && last_relative_pose_time_sec_ > last_relative_pose_used_time_sec_;

        // Each new relative transform estimate becomes one factor between the
        // latest AGV and UAV nodes, through both anchors (see AnchorResidual).
        if(uwb_transform_available_){

            last_relative_pose_used_time_sec_ = last_relative_pose_time_sec_;

            latest_relative_pose_SE3_ = transformSE3FromPoseMsg(latest_relative_pose_.pose.pose);
            //Unflatten matrix to extract the covariance
            for (size_t i = 0; i < 6; ++i) {
                for (size_t j = 0; j < 6; ++j) {
                    latest_relative_pose_cov_(i,j) = latest_relative_pose_.pose.covariance[i * 6 + j];
                }
            }

            double covariance_multiplier = 1.0;

            Sophus::SE3d w_That_target = T_agv_anchor_prior_ * latest_relative_pose_SE3_.inverse() * uav_measurements_.odom_pose;
            Sophus::SE3d w_That_source = T_agv_anchor_prior_ * agv_measurements_.odom_pose;
            Sophus::SE3d That_t_s = w_That_target.inverse() * w_That_source;
            
            MeasurementConstraint uwb_constraint;
            uwb_constraint.id_begin = agv_id_; //relates latest UAV and AGV nodes
            uwb_constraint.id_end = uav_id_;
            uwb_constraint.t_T_s = That_t_s;
            uwb_constraint.covariance = reduceCovarianceMatrix(latest_relative_pose_cov_);
            uwb_constraint.covariance*=covariance_multiplier;
            encounter_constraints_uwb_.push_back(uwb_constraint);

        }
       
        if(!(agv_id_ >= min_keyframes_ || uav_id_ >= min_keyframes_)){
            RCLCPP_INFO(this->get_logger(), "Sliding window not yet full!");
            return;
        }

        //Update transforms after convergence
        if(runPosegraphOptimization()){
            
            // ---------------- Publish All Optimized Poses ----------------
            RCLCPP_INFO(this->get_logger(), "Anchor node AGV:\n"
                        "[%f, %f, %f, %f]", anchor_node_agv_.state[0], anchor_node_agv_.state[1], anchor_node_agv_.state[2], anchor_node_agv_.state[3]);

            RCLCPP_INFO(this->get_logger(), "Anchor node UAV:\n"
                        "[%f, %f, %f, %f]", anchor_node_uav_.state[0], anchor_node_uav_.state[1], anchor_node_uav_.state[2], anchor_node_uav_.state[3]);

            anchor_node_agv_.pose = buildTransformationSE3(anchor_node_agv_.roll, anchor_node_agv_.pitch, anchor_node_agv_.state);
            anchor_node_uav_.pose = buildTransformationSE3(anchor_node_uav_.roll, anchor_node_uav_.pitch, anchor_node_uav_.state);

            // Create PoseWithCovarianceArray messages for AGV and UAV nodes.
            uwb_localization::msg::PoseWithCovarianceStampedArray agv_poses;
            uwb_localization::msg::PoseWithCovarianceStampedArray uav_poses;

            agv_poses.header.stamp = uav_poses.header.stamp = current_time;
            
            agv_poses.header.frame_id = odom_tf_agv_t_;
            uav_poses.header.frame_id = odom_tf_uav_t_;

            posegraph::getPoseGraph(agv_map_, agv_poses);
            posegraph::getPoseGraph(uav_map_, uav_poses);

            geometry_msgs::msg::PoseWithCovarianceStamped anchor_agv = buildPoseMsg(anchor_node_agv_.pose, anchor_node_agv_.covariance, anchor_node_agv_.timestamp, global_frame_graph_);
            geometry_msgs::msg::PoseWithCovarianceStamped anchor_uav = buildPoseMsg(anchor_node_uav_.pose, anchor_node_uav_.covariance, anchor_node_uav_.timestamp, global_frame_graph_);

            poses_agv_publisher_->publish(agv_poses);
            poses_uav_publisher_->publish(uav_poses);

            anchor_agv_publisher_->publish(anchor_agv);
            anchor_uav_publisher_->publish(anchor_uav);

            visualization_msgs::msg::MarkerArray agv_markers, uav_markers;

            // AGV markers
            for (size_t i = 0; i < agv_poses.array.size(); ++i) {
                const auto& pose_msg = agv_poses.array[i].pose.pose;
                Sophus::SE3d local_T = transformSE3FromPoseMsg(pose_msg);
                Sophus::SE3d global_T = anchor_node_agv_.pose * local_T;
                geometry_msgs::msg::Pose global_pose = buildPoseMsg(global_T, Eigen::Matrix4d::Identity(), current_time, global_frame_graph_).pose.pose;
                auto marker = createPoseMarker(global_pose, i, global_frame_graph_, 1.0f, 0.0f, 0.0f);
                marker.ns = "agv_poses";
                agv_markers.markers.push_back(marker);
            }

            // UAV markers
            for (size_t i = 0; i < uav_poses.array.size(); ++i) {
                const auto& pose_msg = uav_poses.array[i].pose.pose;
                Sophus::SE3d local_T = transformSE3FromPoseMsg(pose_msg);
                Sophus::SE3d global_T = anchor_node_uav_.pose * local_T;
                geometry_msgs::msg::Pose global_pose = buildPoseMsg(global_T, Eigen::Matrix4d::Identity(), current_time, global_frame_graph_).pose.pose;
                auto marker = createPoseMarker(global_pose, i, global_frame_graph_, 0.0f, 0.0f, 1.0f);
                marker.ns = "uav_poses";
                uav_markers.markers.push_back(marker);
            }

            agv_marker_pub_->publish(agv_markers);
            uav_marker_pub_->publish(uav_markers);
            
        }

        else{
            RCLCPP_INFO(this->get_logger(), "[Eliko global_opt node] Local optimizer did not converge");
        }


    }

} // namespace uwb_localization
