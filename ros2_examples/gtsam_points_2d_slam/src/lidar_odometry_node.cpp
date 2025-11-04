#include <memory>
#include <deque>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp>
#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>
#include <gtsam_points/d2/registration/alignment_2d.hpp>
#include <gtsam_points/d2/registration/registration_2d.hpp>

using namespace gtsam_points;

/**
 * @brief LiDAR Odometry Node - Scan-to-Scan or Scan-to-Map matching without graph optimization
 *
 * This node performs simple odometry estimation using only LiDAR scan matching.
 * No loop closure, no graph optimization - just dead reckoning with scan matching.
 */
class LidarOdometryNode : public rclcpp::Node
{
public:
  LidarOdometryNode()
  : Node("gtsam_lidar_odometry"),
    initialized_(false),
    frame_count_(0)
  {
    // Declare parameters
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("use_vgicp", true);
    this->declare_parameter("use_scan_to_map", true);
    this->declare_parameter("voxel_resolution", 0.1);
    this->declare_parameter("max_correspondence_distance", 1.0);
    this->declare_parameter("registration_max_iterations", 50);
    this->declare_parameter("registration_transformation_epsilon", 1e-4);

    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    use_vgicp_ = this->get_parameter("use_vgicp").as_bool();
    use_scan_to_map_ = this->get_parameter("use_scan_to_map").as_bool();
    voxel_resolution_ = this->get_parameter("voxel_resolution").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
    registration_max_iterations_ = this->get_parameter("registration_max_iterations").as_int();
    registration_transformation_epsilon_ = this->get_parameter("registration_transformation_epsilon").as_double();

    // Initialize gridmap for VGICP or scan-to-map
    if (use_vgicp_ || use_scan_to_map_) {
      gridmap_ = std::make_shared<IncrementalGridMap2D>(voxel_resolution_);
    }

    // Create subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&LidarOdometryNode::scanCallback, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("lidar_path", 10);

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize current pose
    current_pose_ = gtsam::Pose2(0.0, 0.0, 0.0);

    RCLCPP_INFO(this->get_logger(), "LiDAR Odometry Node initialized");
    RCLCPP_INFO(this->get_logger(), "Method: %s", use_vgicp_ ? "VGICP" : "GICP");
    RCLCPP_INFO(this->get_logger(), "Mode: %s", use_scan_to_map_ ? "Scan-to-Map" : "Scan-to-Scan");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Convert LaserScan to PointCloud2D
    auto scan = convertLaserScan(msg);
    if (!scan || scan->size() < 10) {
      RCLCPP_WARN(this->get_logger(), "Scan has too few points");
      return;
    }

    if (!initialized_) {
      initializeFirstScan(scan, msg->header.stamp);
      return;
    }

    // Perform scan matching
    gtsam::Pose2 relative_pose;
    if (use_scan_to_map_) {
      relative_pose = performScanToMapMatching(scan);
    } else {
      relative_pose = performScanToScanMatching(scan);
    }

    // Update pose (dead reckoning)
    current_pose_ = current_pose_ * relative_pose;

    // Update reference scan for next iteration
    if (!use_scan_to_map_) {
      prev_scan_ = scan;
    } else {
      // Add scan to map
      auto transformed_scan = std::make_shared<PointCloud2DCPU>();
      std::vector<Eigen::Vector3d> transformed_points;
      for (size_t i = 0; i < scan->size(); ++i) {
        Eigen::Vector3d point = scan->points[i];
        double x = current_pose_.x() + point.x() * std::cos(current_pose_.theta()) - point.y() * std::sin(current_pose_.theta());
        double y = current_pose_.y() + point.x() * std::sin(current_pose_.theta()) + point.y() * std::cos(current_pose_.theta());
        transformed_points.push_back(Eigen::Vector3d(x, y, 1.0));
      }
      transformed_scan->add_points(transformed_points);
      gridmap_->insert(transformed_scan);
    }

    // Publish odometry and TF
    publishOdometry(msg->header.stamp);
    publishTransform(msg->header.stamp);
    publishPath(msg->header.stamp);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    frame_count_++;
    if (frame_count_ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "Frame %zu | Pose: (%.2f, %.2f, %.2f) | Time: %ld ms",
        frame_count_,
        current_pose_.x(),
        current_pose_.y(),
        current_pose_.theta(),
        duration);
    }
  }

  std::shared_ptr<PointCloud2DCPU> convertLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr& msg)
  {
    auto cloud = std::make_shared<PointCloud2DCPU>();

    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->ranges.size());

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float range = msg->ranges[i];
      if (std::isnan(range) || range < msg->range_min || range > msg->range_max) {
        continue;
      }

      const double angle = msg->angle_min + i * msg->angle_increment;
      const double x = range * std::cos(angle);
      const double y = range * std::sin(angle);

      points.push_back(Eigen::Vector3d(x, y, 1.0));
    }

    cloud->add_points(points);
    cloud->add_times(std::vector<double>(points.size(), 0.0));

    return cloud;
  }

  void initializeFirstScan(
    const std::shared_ptr<PointCloud2DCPU>& scan,
    const rclcpp::Time& stamp)
  {
    prev_scan_ = scan;

    if (use_scan_to_map_) {
      gridmap_->insert(scan);
    }

    initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "First scan initialized with %zu points", scan->size());
  }

  gtsam::Pose2 performScanToScanMatching(
    const std::shared_ptr<PointCloud2DCPU>& scan)
  {
    if (!prev_scan_) {
      return gtsam::Pose2(0.0, 0.0, 0.0);
    }

    // Use simple registration
    RegistrationSetting2D setting;
    setting.type = use_vgicp_ ? RegistrationType2D::VGICP : RegistrationType2D::GICP;
    setting.voxel_resolution = voxel_resolution_;
    setting.max_correspondence_distance = max_correspondence_distance_;
    setting.max_iterations = registration_max_iterations_;
    setting.transformation_epsilon = registration_transformation_epsilon_;

    auto result = align_scans_2d(prev_scan_, scan, gtsam::Pose2(0.0, 0.0, 0.0), setting);

    return result.T_target_source;
  }

  gtsam::Pose2 performScanToMapMatching(
    const std::shared_ptr<PointCloud2DCPU>& scan)
  {
    if (!gridmap_) {
      return gtsam::Pose2(0.0, 0.0, 0.0);
    }

    // Create factor graph for scan-to-map matching
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial_estimate;

    // Use current pose as initial guess
    gtsam::Symbol pose_key('x', 0);
    initial_estimate.insert(pose_key, current_pose_);

    // Create VGICP factor matching scan to map
    auto vgicp_factor = gtsam::make_shared<IntegratedVGICPFactor2D>(
      pose_key,
      scan,
      gridmap_
    );
    vgicp_factor->set_max_correspondence_distance(max_correspondence_distance_);
    graph.add(vgicp_factor);

    // Optimize using Levenberg-Marquardt
    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.setMaxIterations(registration_max_iterations_);
    lm_params.setRelativeErrorTol(registration_transformation_epsilon_);
    lm_params.setAbsoluteErrorTol(registration_transformation_epsilon_);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, lm_params);
    gtsam::Values result = optimizer.optimize();

    // Extract optimized pose
    gtsam::Pose2 optimized_pose = result.at<gtsam::Pose2>(pose_key);

    // Compute relative transformation from current pose
    gtsam::Pose2 relative_pose = current_pose_.between(optimized_pose);

    return relative_pose;
  }

  void publishOdometry(const rclcpp::Time& stamp)
  {
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = current_pose_.x();
    odom_msg.pose.pose.position.y = current_pose_.y();
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.theta());
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_pub_->publish(odom_msg);
  }

  void publishTransform(const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;

    transform.transform.translation.x = current_pose_.x();
    transform.transform.translation.y = current_pose_.y();
    transform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.theta());
    transform.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(transform);
  }

  void publishPath(const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = odom_frame_;
    pose_stamped.pose.position.x = current_pose_.x();
    pose_stamped.pose.position.y = current_pose_.y();
    pose_stamped.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.theta());
    pose_stamped.pose.orientation = tf2::toMsg(q);

    path_.header.stamp = stamp;
    path_.header.frame_id = odom_frame_;
    path_.poses.push_back(pose_stamped);

    // Keep only last 1000 poses
    if (path_.poses.size() > 1000) {
      path_.poses.erase(path_.poses.begin());
    }

    path_pub_->publish(path_);
  }

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  bool use_vgicp_;
  bool use_scan_to_map_;
  double voxel_resolution_;
  double max_correspondence_distance_;
  int registration_max_iterations_;
  double registration_transformation_epsilon_;

  // Odometry state
  std::shared_ptr<IncrementalGridMap2D> gridmap_;
  std::shared_ptr<PointCloud2DCPU> prev_scan_;
  gtsam::Pose2 current_pose_;
  nav_msgs::msg::Path path_;

  bool initialized_;
  size_t frame_count_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarOdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
