#include <memory>
#include <deque>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>

#include <gtsam_points/d2/types/laser_scan.hpp>
#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp>
#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>
#include <gtsam_points/d2/registration/registration_2d.hpp>

using namespace gtsam_points;

class SlamNode : public rclcpp::Node
{
public:
  SlamNode()
  : Node("gtsam_points_2d_slam"),
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
      std::bind(&SlamNode::scanCallback, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize current pose
    current_pose_ = gtsam::Pose2(0.0, 0.0, 0.0);
    last_keyframe_pose_ = current_pose_;

    RCLCPP_INFO(this->get_logger(), "GTSAM Points 2D SLAM Node initialized");
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

    if (!initialized_) {
      initializeFirstScan(scan, msg->header.stamp);
      return;
    }

    // Perform scan matching
    auto result = performScanMatching(scan);

    // Update pose
    current_pose_ = current_pose_ * result.relative_pose;

    // Check if we need a new keyframe
    if (shouldAddKeyframe()) {
      addKeyframe(scan, msg->header.stamp);
    }

    // Publish odometry and TF
    publishOdometry(msg->header.stamp);
    publishTransform(msg->header.stamp);
  }

  std::shared_ptr<PointCloud2DCPU> convertLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr& msg)
  {
    auto cloud = std::make_shared<PointCloud2DCPU>();

    const size_t num_points =
      static_cast<size_t>((msg->angle_max - msg->angle_min) / msg->angle_increment);

    std::vector<Eigen::Vector3d> points;
    points.reserve(num_points);

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

  struct ScanMatchingResult {
    gtsam::Pose2 relative_pose;
    double fitness_score;
  };

  ScanMatchingResult performScanMatching(
    const std::shared_ptr<PointCloud2DCPU>& scan)
  {
    ScanMatchingResult result;
    result.relative_pose = gtsam::Pose2(0.0, 0.0, 0.0);
    result.fitness_score = 0.0;

    if (keyframes_.empty()) {
      return result;
    }

    // Get the last keyframe as target
    auto target = keyframes_.back();

    // Configure registration settings
    RegistrationSetting2D setting;
    setting.type = use_vgicp_ ? RegistrationType2D::VGICP : RegistrationType2D::GICP;
    setting.voxel_resolution = voxel_resolution_;
    setting.max_correspondence_distance = max_correspondence_distance_;
    setting.max_iterations = 64;
    setting.transformation_epsilon = 1e-3;

    // Initial guess: identity (assume small motion between scans)
    gtsam::Pose2 initial_guess(0.0, 0.0, 0.0);

    // Perform scan-to-scan matching using align_scans_2d
    auto registration_result = align_scans_2d(target, scan, initial_guess, setting);

    // Extract results
    result.relative_pose = registration_result.T_target_source;
    result.fitness_score = registration_result.converged ?
                          static_cast<double>(registration_result.num_inliers) / scan->size() : 0.0;

    // Log if matching failed
    if (!registration_result.converged) {
      RCLCPP_WARN(this->get_logger(), "Scan matching did not converge");
    }

    return result;
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

    // Update ISAM2
    isam2_->update(graph_, initial_estimates_);

    // Get optimized result
    const auto result = isam2_->calculateEstimate();
    current_pose_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', key_counter_));

    graph_.resize(0);
    initial_estimates_.clear();

    // Store keyframe
    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_);
    last_keyframe_pose_ = current_pose_;

    key_counter_++;

    RCLCPP_INFO(this->get_logger(),
      "Added keyframe %zu at (%.2f, %.2f, %.2f)",
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

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
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

  // SLAM state
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
  std::shared_ptr<IncrementalGridMap2D> gridmap_;

  std::deque<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::deque<gtsam::Pose2> keyframe_poses_;

  gtsam::Pose2 current_pose_;
  gtsam::Pose2 last_keyframe_pose_;

  size_t key_counter_;
  bool initialized_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
