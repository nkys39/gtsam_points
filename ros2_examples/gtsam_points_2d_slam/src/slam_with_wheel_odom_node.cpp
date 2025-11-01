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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp>
#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>

using namespace gtsam_points;

/**
 * @brief SLAM with Wheel Odometry Integration
 *
 * This node combines wheel odometry from TurtleBot3 with LiDAR scan matching.
 * Wheel odometry is added as BetweenFactors in the graph for smoother trajectories.
 */
class SlamWithWheelOdomNode : public rclcpp::Node
{
public:
  SlamWithWheelOdomNode()
  : Node("gtsam_slam_with_wheel_odom"),
    key_counter_(0),
    initialized_(false),
    has_wheel_odom_(false)
  {
    // Declare parameters
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("odom_topic", "odom");
    this->declare_parameter("use_vgicp", true);
    this->declare_parameter("keyframe_distance", 0.5);
    this->declare_parameter("keyframe_angle", 0.3);
    this->declare_parameter("voxel_resolution", 0.1);
    this->declare_parameter("max_correspondence_distance", 1.0);

    // Odometry fusion parameters
    this->declare_parameter("use_wheel_odom", true);
    this->declare_parameter("wheel_odom_weight", 1.0);
    this->declare_parameter("lidar_odom_weight", 10.0);

    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    use_vgicp_ = this->get_parameter("use_vgicp").as_bool();
    keyframe_distance_ = this->get_parameter("keyframe_distance").as_double();
    keyframe_angle_ = this->get_parameter("keyframe_angle").as_double();
    voxel_resolution_ = this->get_parameter("voxel_resolution").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();

    use_wheel_odom_ = this->get_parameter("use_wheel_odom").as_bool();
    wheel_odom_weight_ = this->get_parameter("wheel_odom_weight").as_double();
    lidar_odom_weight_ = this->get_parameter("lidar_odom_weight").as_double();

    // Initialize ISAM2
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.1;
    isam2_params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);

    // Initialize gridmap for VGICP
    if (use_vgicp_) {
      gridmap_ = std::make_shared<IncrementalGridMap2D>(voxel_resolution_);
    }

    // Create subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&SlamWithWheelOdomNode::scanCallback, this, std::placeholders::_1));

    if (use_wheel_odom_) {
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&SlamWithWheelOdomNode::odomCallback, this, std::placeholders::_1));
    }

    // Create publishers
    slam_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize current pose
    current_pose_ = gtsam::Pose2(0.0, 0.0, 0.0);
    last_keyframe_pose_ = current_pose_;
    last_wheel_odom_pose_ = gtsam::Pose2(0.0, 0.0, 0.0);

    RCLCPP_INFO(this->get_logger(), "SLAM with Wheel Odometry Node initialized");
    RCLCPP_INFO(this->get_logger(), "Using %s for scan matching", use_vgicp_ ? "VGICP" : "GICP");
    RCLCPP_INFO(this->get_logger(), "Wheel odometry: %s", use_wheel_odom_ ? "Enabled" : "Disabled");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Extract pose from odometry message
    const auto& pos = msg->pose.pose.position;
    const auto& ori = msg->pose.pose.orientation;

    // Convert quaternion to yaw
    tf2::Quaternion q(ori.x, ori.y, ori.z, ori.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_wheel_odom_pose_ = gtsam::Pose2(pos.x, pos.y, yaw);
    current_wheel_odom_stamp_ = msg->header.stamp;

    if (!has_wheel_odom_) {
      last_wheel_odom_pose_ = current_wheel_odom_pose_;
      has_wheel_odom_ = true;
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
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

    // Update current pose with wheel odometry if available
    if (use_wheel_odom_ && has_wheel_odom_) {
      // Calculate wheel odometry delta
      gtsam::Pose2 wheel_odom_delta = last_wheel_odom_pose_.between(current_wheel_odom_pose_);
      current_pose_ = current_pose_ * wheel_odom_delta;
    }

    // Check if we need a new keyframe
    if (shouldAddKeyframe()) {
      addKeyframe(scan, msg->header.stamp);
    }

    // Publish odometry and TF
    publishOdometry(msg->header.stamp);
    publishTransform(msg->header.stamp);
    publishPath(msg->header.stamp);
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
    // Add prior factor
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.01, 0.01, 0.01));

    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(
      gtsam::Symbol('x', key_counter_), current_pose_, prior_noise));

    initial_estimates_.insert(gtsam::Symbol('x', key_counter_), current_pose_);

    // Update ISAM2
    isam2_->update(graph_, initial_estimates_);
    graph_.resize(0);
    initial_estimates_.clear();

    // Store keyframe
    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_);
    last_keyframe_pose_ = current_pose_;

    // Add to gridmap
    if (use_vgicp_) {
      gridmap_->insert(scan);
    }

    key_counter_++;
    initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "First scan initialized");
  }

  bool shouldAddKeyframe()
  {
    const double dx = current_pose_.x() - last_keyframe_pose_.x();
    const double dy = current_pose_.y() - last_keyframe_pose_.y();
    const double distance = std::sqrt(dx * dx + dy * dy);

    const double dtheta = std::abs(
      current_pose_.theta() - last_keyframe_pose_.theta());

    return (distance > keyframe_distance_) || (dtheta > keyframe_angle_);
  }

  void addKeyframe(
    const std::shared_ptr<PointCloud2DCPU>& scan,
    const rclcpp::Time& stamp)
  {
    // Create factors
    if (!keyframes_.empty()) {
      const auto prev_pose = keyframe_poses_.back();
      const auto relative_pose = prev_pose.between(current_pose_);

      // Add wheel odometry factor (if available)
      if (use_wheel_odom_ && has_wheel_odom_) {
        gtsam::Pose2 wheel_odom_delta = last_wheel_odom_pose_.between(current_wheel_odom_pose_);

        // Wheel odometry noise (higher uncertainty)
        auto wheel_odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector3(
            0.2 / wheel_odom_weight_,
            0.2 / wheel_odom_weight_,
            0.1 / wheel_odom_weight_));

        graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
          gtsam::Symbol('x', key_counter_ - 1),
          gtsam::Symbol('x', key_counter_),
          wheel_odom_delta,
          wheel_odom_noise));

        last_wheel_odom_pose_ = current_wheel_odom_pose_;
      }

      // Add GICP/VGICP factor (LiDAR odometry with higher weight)
      if (use_vgicp_) {
        auto factor = gtsam::make_shared<IntegratedVGICPFactor2D>(
          gtsam::Symbol('x', key_counter_),
          scan,
          gridmap_);

        // Scale factor error by LiDAR weight
        // Note: IntegratedVGICPFactor2D handles its own noise model internally
        graph_.add(factor);

        // Update gridmap with new scan
        gridmap_->insert(scan);
      } else {
        auto target = keyframes_.back();
        auto factor = gtsam::make_shared<IntegratedGICPFactor2D>(
          gtsam::Symbol('x', key_counter_),
          target,
          scan);
        graph_.add(factor);
      }
    }

    // Add initial estimate
    initial_estimates_.insert(gtsam::Symbol('x', key_counter_), current_pose_);

    // Update ISAM2
    isam2_->update(graph_, initial_estimates_);

    // Get optimized result
    const auto result = isam2_->calculateEstimate();
    current_pose_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', key_counter_));

    // Update keyframe poses
    for (size_t i = 0; i < keyframe_poses_.size(); ++i) {
      keyframe_poses_[i] = result.at<gtsam::Pose2>(gtsam::Symbol('x', i));
    }

    graph_.resize(0);
    initial_estimates_.clear();

    // Store keyframe
    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_);
    last_keyframe_pose_ = current_pose_;

    key_counter_++;

    RCLCPP_INFO(this->get_logger(),
      "Added keyframe %zu at (%.2f, %.2f, %.2f) | Wheel+LiDAR fusion",
      key_counter_ - 1,
      current_pose_.x(),
      current_pose_.y(),
      current_pose_.theta());
  }

  void publishOdometry(const rclcpp::Time& stamp)
  {
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = current_pose_.x();
    odom_msg.pose.pose.position.y = current_pose_.y();
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.theta());
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    slam_odom_pub_->publish(odom_msg);
  }

  void publishTransform(const rclcpp::Time& stamp)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
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
    path_.header.stamp = stamp;
    path_.header.frame_id = map_frame_;
    path_.poses.clear();

    for (const auto& pose : keyframe_poses_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path_.header;
      pose_stamped.pose.position.x = pose.x();
      pose_stamped.pose.position.y = pose.y();
      pose_stamped.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, pose.theta());
      pose_stamped.pose.orientation = tf2::toMsg(q);

      path_.poses.push_back(pose_stamped);
    }

    path_pub_->publish(path_);
  }

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr slam_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string odom_topic_;
  bool use_vgicp_;
  double keyframe_distance_;
  double keyframe_angle_;
  double voxel_resolution_;
  double max_correspondence_distance_;

  bool use_wheel_odom_;
  double wheel_odom_weight_;
  double lidar_odom_weight_;

  // SLAM state
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
  std::shared_ptr<IncrementalGridMap2D> gridmap_;

  std::deque<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::deque<gtsam::Pose2> keyframe_poses_;

  gtsam::Pose2 current_pose_;
  gtsam::Pose2 last_keyframe_pose_;
  gtsam::Pose2 last_wheel_odom_pose_;
  gtsam::Pose2 current_wheel_odom_pose_;
  rclcpp::Time current_wheel_odom_stamp_;
  nav_msgs::msg::Path path_;

  size_t key_counter_;
  bool initialized_;
  bool has_wheel_odom_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamWithWheelOdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
