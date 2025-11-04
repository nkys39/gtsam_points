#include <memory>
#include <deque>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/factors/integrated_ct_gicp_factor_2d.hpp>
#include <gtsam_points/d2/features/normal_estimation_2d.hpp>
#include <gtsam_points/d2/features/covariance_estimation_2d.hpp>
#include <gtsam_points/d2/ann/kdtree_2d.hpp>

using namespace gtsam_points;

class SlamWithCTGICPNode : public rclcpp::Node
{
public:
  SlamWithCTGICPNode()
  : Node("slam_with_ct_gicp"),
    key_counter_(0),
    initialized_(false)
  {
    // Declare parameters
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("keyframe_distance", 0.5);
    this->declare_parameter("keyframe_angle", 0.3);
    this->declare_parameter("max_correspondence_distance", 1.0);
    this->declare_parameter("k_neighbors", 10);

    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    keyframe_distance_ = this->get_parameter("keyframe_distance").as_double();
    keyframe_angle_ = this->get_parameter("keyframe_angle").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
    k_neighbors_ = this->get_parameter("k_neighbors").as_int();

    // Initialize ISAM2
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.1;
    isam2_params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);

    // Create subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&SlamWithCTGICPNode::scanCallback, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize current pose (scan start pose)
    current_pose_t0_ = gtsam::Pose2(0.0, 0.0, 0.0);
    current_pose_t1_ = gtsam::Pose2(0.0, 0.0, 0.0);
    last_keyframe_pose_ = current_pose_t0_;

    RCLCPP_INFO(this->get_logger(), "CT-GICP SLAM Node initialized");
    RCLCPP_INFO(this->get_logger(), "Continuous-Time GICP with covariance estimation");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Convert LaserScan to timestamped PointCloud2D
    auto scan = convertLaserScanWithTimestamps(msg);
    if (!scan || scan->size() < 10) {
      RCLCPP_WARN(this->get_logger(), "Scan has too few points");
      return;
    }

    // Estimate normals and covariances for GICP
    estimate_normals_2d(*scan, k_neighbors_);
    estimate_covariances_2d(*scan);

    if (!initialized_) {
      initializeFirstScan(scan, msg->header.stamp);
      return;
    }

    // Check if we should add a new keyframe
    const double dist = std::hypot(
      current_pose_t1_.x() - last_keyframe_pose_.x(),
      current_pose_t1_.y() - last_keyframe_pose_.y());
    const double angle_diff = std::abs(current_pose_t1_.theta() - last_keyframe_pose_.theta());

    if (dist < keyframe_distance_ && angle_diff < keyframe_angle_) {
      // Update current pose estimate but don't add keyframe
      publishOdometry(msg->header.stamp);
      return;
    }

    // Add new keyframe with CT-GICP
    addKeyframeWithCTGICP(scan, msg->header.stamp);

    // Publish results
    publishOdometry(msg->header.stamp);
    publishPath(msg->header.stamp);
  }

  std::shared_ptr<PointCloud2DCPU> convertLaserScanWithTimestamps(
    const sensor_msgs::msg::LaserScan::SharedPtr& msg)
  {
    auto cloud = std::make_shared<PointCloud2DCPU>();

    const size_t num_points =
      static_cast<size_t>((msg->angle_max - msg->angle_min) / msg->angle_increment);

    std::vector<Eigen::Vector3d> points;
    std::vector<double> timestamps;
    points.reserve(num_points);
    timestamps.reserve(num_points);

    // Calculate total scan duration
    const double scan_duration = msg->time_increment * msg->ranges.size();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float range = msg->ranges[i];
      if (std::isnan(range) || range < msg->range_min || range > msg->range_max) {
        continue;
      }

      const double angle = msg->angle_min + i * msg->angle_increment;
      const double x = range * std::cos(angle);
      const double y = range * std::sin(angle);

      points.push_back(Eigen::Vector3d(x, y, 1.0));

      // Normalize timestamp to [0, 1]
      const double normalized_time = (scan_duration > 0.0) ?
        (msg->time_increment * i) / scan_duration : 0.0;
      timestamps.push_back(normalized_time);
    }

    cloud->add_points(points);
    cloud->add_times(timestamps);

    return cloud;
  }

  void initializeFirstScan(
    const std::shared_ptr<PointCloud2DCPU>& scan,
    const rclcpp::Time& stamp)
  {
    // For the first scan, t0 and t1 are the same (stationary)
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.01, 0.01, 0.01));

    // Add prior factors for both t0 and t1
    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(
      gtsam::Symbol('x', key_counter_ * 2), current_pose_t0_, prior_noise));
    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(
      gtsam::Symbol('x', key_counter_ * 2 + 1), current_pose_t1_, prior_noise));

    initial_estimates_.insert(gtsam::Symbol('x', key_counter_ * 2), current_pose_t0_);
    initial_estimates_.insert(gtsam::Symbol('x', key_counter_ * 2 + 1), current_pose_t1_);

    // Update ISAM2
    isam2_->update(graph_, initial_estimates_);
    graph_.resize(0);
    initial_estimates_.clear();

    // Store keyframe
    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_t0_);
    last_keyframe_pose_ = current_pose_t1_;

    key_counter_++;
    initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "First scan initialized with CT-GICP");
  }

  void addKeyframeWithCTGICP(
    const std::shared_ptr<PointCloud2DCPU>& scan,
    const rclcpp::Time& stamp)
  {
    // Get target keyframe (last keyframe) - also has normals and covariances
    auto target = keyframes_.back();

    // Initial guess: constant velocity model
    const gtsam::Pose2 pose_t0 = current_pose_t1_;  // Start where last scan ended

    // Estimate motion during scan (assume small motion)
    const gtsam::Pose2 delta = gtsam::Pose2(0.05, 0.0, 0.01);  // Small forward motion
    const gtsam::Pose2 pose_t1 = pose_t0 * delta;

    // Current keyframe index
    const int current_key = key_counter_;
    const int prev_key = key_counter_ - 1;

    // Create CT-GICP factor
    // This factor uses Mahalanobis distance with covariances for robust matching
    auto ct_gicp_factor = gtsam::make_shared<IntegratedCT_GICPFactor2D>(
      gtsam::Symbol('x', current_key * 2),      // current scan start pose
      gtsam::Symbol('x', current_key * 2 + 1),  // current scan end pose
      target,  // target keyframe (with normals and covariances)
      scan     // source scan (with timestamps, normals, and covariances)
    );
    ct_gicp_factor->set_max_correspondence_distance(max_correspondence_distance_);
    ct_gicp_factor->set_num_threads(4);
    graph_.add(ct_gicp_factor);

    // Add between factor connecting previous scan end to current scan start
    auto between_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.1, 0.1, 0.1));

    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
      gtsam::Symbol('x', prev_key * 2 + 1),  // previous scan end
      gtsam::Symbol('x', current_key * 2),    // current scan start
      gtsam::Pose2(0.0, 0.0, 0.0),            // should be the same
      between_noise
    ));

    // Add initial estimates
    initial_estimates_.insert(gtsam::Symbol('x', current_key * 2), pose_t0);
    initial_estimates_.insert(gtsam::Symbol('x', current_key * 2 + 1), pose_t1);

    // Update ISAM2
    isam2_->update(graph_, initial_estimates_);
    graph_.resize(0);
    initial_estimates_.clear();

    // Get optimized poses
    auto result = isam2_->calculateEstimate();
    current_pose_t0_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', current_key * 2));
    current_pose_t1_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', current_key * 2 + 1));

    // Store keyframe
    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_t0_);
    last_keyframe_pose_ = current_pose_t1_;

    key_counter_++;

    // Log motion during scan
    const double motion_x = current_pose_t1_.x() - current_pose_t0_.x();
    const double motion_y = current_pose_t1_.y() - current_pose_t0_.y();
    const double motion_theta = current_pose_t1_.theta() - current_pose_t0_.theta();
    RCLCPP_DEBUG(this->get_logger(),
      "Motion during scan: dx=%.3f, dy=%.3f, dθ=%.3f rad",
      motion_x, motion_y, motion_theta);

    RCLCPP_INFO(this->get_logger(),
      "Added keyframe %d (total: %zu)", key_counter_ - 1, keyframes_.size());
  }

  void publishOdometry(const rclcpp::Time& stamp)
  {
    // Publish odometry (use end pose of current scan)
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = current_pose_t1_.x();
    odom.pose.pose.position.y = current_pose_t1_.y();
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_t1_.theta());
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom_pub_->publish(odom);

    // Broadcast TF
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = current_pose_t1_.x();
    transform.transform.translation.y = current_pose_t1_.y();
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(transform);
  }

  void publishPath(const rclcpp::Time& stamp)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = map_frame_;

    auto result = isam2_->calculateEstimate();

    for (int i = 0; i < key_counter_; ++i) {
      // Use scan end pose for visualization
      auto pose = result.at<gtsam::Pose2>(gtsam::Symbol('x', i * 2 + 1));

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.stamp = stamp;
      pose_stamped.header.frame_id = map_frame_;
      pose_stamped.pose.position.x = pose.x();
      pose_stamped.pose.position.y = pose.y();
      pose_stamped.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, pose.theta());
      pose_stamped.pose.orientation = tf2::toMsg(q);

      path_msg.poses.push_back(pose_stamped);
    }

    path_pub_->publish(path_msg);
  }

private:
  // ROS parameters
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  double keyframe_distance_;
  double keyframe_angle_;
  double max_correspondence_distance_;
  int k_neighbors_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // GTSAM
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;

  // SLAM state
  int key_counter_;
  bool initialized_;
  gtsam::Pose2 current_pose_t0_;  // Current scan start pose
  gtsam::Pose2 current_pose_t1_;  // Current scan end pose
  gtsam::Pose2 last_keyframe_pose_;

  // Keyframes
  std::deque<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::deque<gtsam::Pose2> keyframe_poses_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamWithCTGICPNode>());
  rclcpp::shutdown();
  return 0;
}
