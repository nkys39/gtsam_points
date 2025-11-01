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
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp>
#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>

using namespace gtsam_points;

/**
 * @brief SLAM with Fixed-lag Smoothing for Long-term Operation
 *
 * This node uses IncrementalFixedLagSmootherExt to maintain a sliding
 * window of recent keyframes, automatically marginalizing old states.
 * Memory usage remains constant even for long-term operation.
 */
class SlamWithFixedLagNode : public rclcpp::Node
{
public:
  SlamWithFixedLagNode()
  : Node("gtsam_slam_with_fixed_lag"),
    key_counter_(0),
    initialized_(false)
  {
    // Declare parameters
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("use_vgicp", true);
    this->declare_parameter("keyframe_distance", 0.5);
    this->declare_parameter("keyframe_angle", 0.3);
    this->declare_parameter("voxel_resolution", 0.1);
    this->declare_parameter("max_correspondence_distance", 1.0);

    // Fixed-lag smoother parameters
    this->declare_parameter("smoother_lag", 30.0);  // seconds
    this->declare_parameter("max_keyframes", 100);   // maximum keyframes in window

    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    use_vgicp_ = this->get_parameter("use_vgicp").as_bool();
    keyframe_distance_ = this->get_parameter("keyframe_distance").as_double();
    keyframe_angle_ = this->get_parameter("keyframe_angle").as_double();
    voxel_resolution_ = this->get_parameter("voxel_resolution").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();

    smoother_lag_ = this->get_parameter("smoother_lag").as_double();
    max_keyframes_ = this->get_parameter("max_keyframes").as_int();

    // Initialize Fixed-lag Smoother
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.1;
    isam2_params.relinearizeSkip = 1;
    smoother_ = std::make_shared<IncrementalFixedLagSmootherExt>(
      smoother_lag_, isam2_params);

    // Initialize gridmap for VGICP
    if (use_vgicp_) {
      gridmap_ = std::make_shared<IncrementalGridMap2D>(voxel_resolution_);
    }

    // Create subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&SlamWithFixedLagNode::scanCallback, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize current pose
    current_pose_ = gtsam::Pose2(0.0, 0.0, 0.0);
    last_keyframe_pose_ = current_pose_;

    RCLCPP_INFO(this->get_logger(), "SLAM with Fixed-lag Smoothing Node initialized");
    RCLCPP_INFO(this->get_logger(), "Smoother lag: %.1f seconds, Max keyframes: %d",
      smoother_lag_, max_keyframes_);
    RCLCPP_INFO(this->get_logger(), "Using %s for scan matching", use_vgicp_ ? "VGICP" : "GICP");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Convert LaserScan to PointCloud2D
    auto scan = convertLaserScan(msg);
    if (!scan || scan->size() < 10) {
      RCLCPP_WARN(this->get_logger(), "Scan has too few points");
      return;
    }

    const double current_time = rclcpp::Time(msg->header.stamp).seconds();

    if (!initialized_) {
      initializeFirstScan(scan, msg->header.stamp, current_time);
      return;
    }

    // Check if we need a new keyframe
    if (shouldAddKeyframe()) {
      addKeyframe(scan, msg->header.stamp, current_time);
    }

    // Publish odometry and TF
    publishOdometry(msg->header.stamp);
    publishTransform(msg->header.stamp);
    publishPath(msg->header.stamp);

    // Log smoother statistics
    if (key_counter_ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "Keyframes: %zu, Window size: %zu, Marginalized: %zu",
        key_counter_,
        keyframe_timestamps_.size(),
        marginalized_count_);
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
    const rclcpp::Time& stamp,
    double current_time)
  {
    // Add prior factor
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.01, 0.01, 0.01));

    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(
      gtsam::Symbol('x', key_counter_), current_pose_, prior_noise));

    initial_estimates_.insert(gtsam::Symbol('x', key_counter_), current_pose_);

    // Add timestamp for fixed-lag smoother
    timestamps_[gtsam::Symbol('x', key_counter_)] = current_time;

    // Update smoother
    smoother_->update(graph_, initial_estimates_, timestamps_);
    graph_.resize(0);
    initial_estimates_.clear();
    timestamps_.clear();

    // Store keyframe
    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_);
    keyframe_ids_.push_back(key_counter_);
    keyframe_timestamps_.push_back(current_time);
    last_keyframe_pose_ = current_pose_;

    // Add to gridmap
    if (use_vgicp_) {
      gridmap_->insert(scan);
    }

    key_counter_++;
    initialized_ = true;
    marginalized_count_ = 0;

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
    const rclcpp::Time& stamp,
    double current_time)
  {
    // Create odometry factor between consecutive keyframes
    if (!keyframes_.empty()) {
      const auto prev_pose = keyframe_poses_.back();
      const auto relative_pose = prev_pose.between(current_pose_);

      auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.1, 0.1, 0.05));

      // Add between factor
      graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
        gtsam::Symbol('x', key_counter_ - 1),
        gtsam::Symbol('x', key_counter_),
        relative_pose,
        odom_noise));

      // Add GICP/VGICP factor
      if (use_vgicp_) {
        auto factor = gtsam::make_shared<IntegratedVGICPFactor2D>(
          gtsam::Symbol('x', key_counter_),
          scan,
          gridmap_);
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

    // Add timestamp for fixed-lag smoother
    timestamps_[gtsam::Symbol('x', key_counter_)] = current_time;

    // Update smoother (automatically marginalizes old states)
    smoother_->update(graph_, initial_estimates_, timestamps_);

    // Get optimized result
    const auto result = smoother_->calculateEstimate();
    current_pose_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', key_counter_));

    // Update keyframe poses with optimized values
    for (size_t i = 0; i < keyframe_ids_.size(); ++i) {
      if (result.exists(gtsam::Symbol('x', keyframe_ids_[i]))) {
        keyframe_poses_[i] = result.at<gtsam::Pose2>(gtsam::Symbol('x', keyframe_ids_[i]));
      }
    }

    graph_.resize(0);
    initial_estimates_.clear();
    timestamps_.clear();

    // Store keyframe
    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_);
    keyframe_ids_.push_back(key_counter_);
    keyframe_timestamps_.push_back(current_time);
    last_keyframe_pose_ = current_pose_;

    // Remove old keyframes that are outside the window
    pruneOldKeyframes(current_time);

    key_counter_++;

    RCLCPP_DEBUG(this->get_logger(),
      "Added keyframe %zu at (%.2f, %.2f, %.2f)",
      key_counter_ - 1,
      current_pose_.x(),
      current_pose_.y(),
      current_pose_.theta());
  }

  void pruneOldKeyframes(double current_time)
  {
    // Remove keyframes older than the smoother lag
    const double cutoff_time = current_time - smoother_lag_;

    while (!keyframe_timestamps_.empty() &&
           keyframe_timestamps_.front() < cutoff_time &&
           keyframes_.size() > static_cast<size_t>(max_keyframes_))
    {
      keyframes_.pop_front();
      keyframe_poses_.pop_front();
      keyframe_ids_.pop_front();
      keyframe_timestamps_.pop_front();
      marginalized_count_++;
    }
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

    odom_pub_->publish(odom_msg);
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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  bool use_vgicp_;
  double keyframe_distance_;
  double keyframe_angle_;
  double voxel_resolution_;
  double max_correspondence_distance_;

  double smoother_lag_;
  int max_keyframes_;

  // SLAM state
  std::shared_ptr<IncrementalFixedLagSmootherExt> smoother_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps_;
  std::shared_ptr<IncrementalGridMap2D> gridmap_;

  std::deque<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::deque<gtsam::Pose2> keyframe_poses_;
  std::deque<size_t> keyframe_ids_;
  std::deque<double> keyframe_timestamps_;

  gtsam::Pose2 current_pose_;
  gtsam::Pose2 last_keyframe_pose_;
  nav_msgs::msg::Path path_;

  size_t key_counter_;
  size_t marginalized_count_;
  bool initialized_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamWithFixedLagNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
