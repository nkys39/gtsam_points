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
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/registration/graduated_non_convexity_2d.hpp>

using namespace gtsam_points;

class SlamWithGncNode : public rclcpp::Node
{
public:
  SlamWithGncNode()
  : Node("slam_with_gnc"),
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

    // GNC parameters
    this->declare_parameter("gnc_mu_init", 1.0);
    this->declare_parameter("gnc_mu_step", 1.4);
    this->declare_parameter("gnc_max_iterations", 100);
    this->declare_parameter("gnc_cost_threshold", 1e-5);
    this->declare_parameter("gnc_inlier_threshold", 0.1);

    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    keyframe_distance_ = this->get_parameter("keyframe_distance").as_double();
    keyframe_angle_ = this->get_parameter("keyframe_angle").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();

    gnc_mu_init_ = this->get_parameter("gnc_mu_init").as_double();
    gnc_mu_step_ = this->get_parameter("gnc_mu_step").as_double();
    gnc_max_iterations_ = this->get_parameter("gnc_max_iterations").as_int();
    gnc_cost_threshold_ = this->get_parameter("gnc_cost_threshold").as_double();
    gnc_inlier_threshold_ = this->get_parameter("gnc_inlier_threshold").as_double();

    // Initialize ISAM2
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.1;
    isam2_params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);

    // Create subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&SlamWithGncNode::scanCallback, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "GNC SLAM node initialized");
    RCLCPP_INFO(this->get_logger(), "GNC parameters: mu_init=%.2f, mu_step=%.2f, max_iter=%d",
                gnc_mu_init_, gnc_mu_step_, gnc_max_iterations_);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto start = std::chrono::high_resolution_clock::now();

    // Convert LaserScan to PointCloud2DCPU
    auto cloud = std::make_shared<PointCloud2DCPU>();
    cloud->points.reserve(msg->ranges.size());

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float range = msg->ranges[i];

      if (std::isnan(range) || std::isinf(range) ||
          range < msg->range_min || range > msg->range_max) {
        continue;
      }

      float angle = msg->angle_min + i * msg->angle_increment;
      double x = range * std::cos(angle);
      double y = range * std::sin(angle);

      cloud->points.push_back(Eigen::Vector3d(x, y, 1.0));
    }

    if (cloud->points.size() < 50) {
      RCLCPP_WARN(this->get_logger(), "Too few points in scan: %zu", cloud->points.size());
      return;
    }

    if (!initialized_) {
      initializeFirstKeyframe(cloud, msg->header.stamp);
      return;
    }

    // Use GNC for robust matching
    gtsam::Pose2 relative_pose;
    double final_cost = 0.0;
    int num_inliers = 0;
    bool match_success = performGNCMatching(keyframes_.back(), cloud, relative_pose, final_cost, num_inliers);

    if (!match_success) {
      RCLCPP_WARN(this->get_logger(), "GNC matching failed");
      return;
    }

    // Update current pose
    current_pose_ = current_pose_.compose(relative_pose);

    // Check if we should create a new keyframe
    gtsam::Pose2 delta = last_keyframe_pose_.between(current_pose_);
    double distance = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
    double angle = std::abs(delta.theta());

    if (distance > keyframe_distance_ || angle > keyframe_angle_) {
      addKeyframe(cloud, msg->header.stamp, relative_pose);
    }

    publishTransformAndOdometry(msg->header.stamp);
    publishPath(msg->header.stamp);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    if (key_counter_ % 10 == 0) {
      double inlier_ratio = static_cast<double>(num_inliers) / static_cast<double>(cloud->size());
      RCLCPP_INFO(this->get_logger(), "Keyframes: %d, Inliers: %d/%.0f (%.1f%%), Cost: %.6f, Time: %ld ms",
                  key_counter_, num_inliers, static_cast<double>(cloud->size()),
                  inlier_ratio * 100.0, final_cost, duration);
    }
  }

  void initializeFirstKeyframe(const std::shared_ptr<PointCloud2DCPU>& cloud, const rclcpp::Time& stamp)
  {
    // Create prior factor for first pose
    auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << 0.01, 0.01, 0.01).finished());

    gtsam::Symbol key('x', key_counter_);
    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(key, gtsam::Pose2(0, 0, 0), noise_model));

    initial_estimates_.insert(key, gtsam::Pose2(0, 0, 0));

    // Update ISAM2
    isam2_->update(graph_, initial_estimates_);
    graph_.resize(0);
    initial_estimates_.clear();

    // Store keyframe
    keyframes_.push_back(cloud);
    keyframe_poses_.push_back(gtsam::Pose2(0, 0, 0));
    keyframe_stamps_.push_back(stamp);

    current_pose_ = gtsam::Pose2(0, 0, 0);
    last_keyframe_pose_ = current_pose_;

    key_counter_++;
    initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "First keyframe initialized with %zu points", cloud->points.size());
  }

  bool performGNCMatching(
    const std::shared_ptr<PointCloud2DCPU>& target,
    const std::shared_ptr<PointCloud2DCPU>& source,
    gtsam::Pose2& relative_pose,
    double& final_cost,
    int& num_inliers)
  {
    // Configure GNC
    GraduatedNonConvexity2DParams params;
    params.mu_init = gnc_mu_init_;
    params.mu_step = gnc_mu_step_;
    params.max_iterations = gnc_max_iterations_;
    params.cost_threshold = gnc_cost_threshold_;
    params.inlier_threshold = gnc_inlier_threshold_;

    GraduatedNonConvexity2D gnc(params);

    // Initial guess (identity transform)
    Eigen::Isometry2d initial_guess = Eigen::Isometry2d::Identity();

    // Perform GNC optimization
    Eigen::Isometry2d T_estimate;
    std::vector<double> weights;
    bool success = gnc.estimate(target, source, initial_guess, T_estimate, weights, final_cost);

    if (!success) {
      return false;
    }

    // Count inliers (weight > 0.5)
    num_inliers = 0;
    for (double w : weights) {
      if (w > 0.5) {
        num_inliers++;
      }
    }

    // Convert Eigen::Isometry2d to gtsam::Pose2
    Eigen::Matrix3d T = T_estimate.matrix();
    double x = T(0, 2);
    double y = T(1, 2);
    double theta = std::atan2(T(1, 0), T(0, 0));

    relative_pose = gtsam::Pose2(x, y, theta);

    return true;
  }

  void addKeyframe(
    const std::shared_ptr<PointCloud2DCPU>& cloud,
    const rclcpp::Time& stamp,
    const gtsam::Pose2& relative_pose)
  {
    gtsam::Symbol current_key('x', key_counter_);
    gtsam::Symbol previous_key('x', key_counter_ - 1);

    // Add between factor using GNC result
    auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << 0.1, 0.1, 0.1).finished());
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(previous_key, current_key, relative_pose, noise_model));

    // Add GICP factor for refinement
    auto gicp_factor = gtsam::make_shared<IntegratedGICPFactor2D>(
      previous_key, current_key,
      keyframes_.back(), cloud);
    gicp_factor->set_max_correspondence_distance(max_correspondence_distance_);
    graph_.add(gicp_factor);

    // Add initial estimate
    initial_estimates_.insert(current_key, current_pose_);

    // Update ISAM2
    isam2_->update(graph_, initial_estimates_);
    auto result = isam2_->calculateEstimate();
    graph_.resize(0);
    initial_estimates_.clear();

    // Update poses with optimized values
    for (int i = 0; i < key_counter_; ++i) {
      keyframe_poses_[i] = result.at<gtsam::Pose2>(gtsam::Symbol('x', i));
    }

    current_pose_ = result.at<gtsam::Pose2>(current_key);

    // Store keyframe
    keyframes_.push_back(cloud);
    keyframe_poses_.push_back(current_pose_);
    keyframe_stamps_.push_back(stamp);
    last_keyframe_pose_ = current_pose_;

    key_counter_++;
  }

  void publishTransformAndOdometry(const rclcpp::Time& stamp)
  {
    // Publish TF
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

    // Publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = current_pose_.x();
    odom.pose.pose.position.y = current_pose_.y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom_pub_->publish(odom);
  }

  void publishPath(const rclcpp::Time& stamp)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = stamp;
    path.header.frame_id = map_frame_;

    for (size_t i = 0; i < keyframe_poses_.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path.header;
      pose_stamped.pose.position.x = keyframe_poses_[i].x();
      pose_stamped.pose.position.y = keyframe_poses_[i].y();
      pose_stamped.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, keyframe_poses_[i].theta());
      pose_stamped.pose.orientation = tf2::toMsg(q);

      path.poses.push_back(pose_stamped);
    }

    path_pub_->publish(path);
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

  // GNC parameters
  double gnc_mu_init_;
  double gnc_mu_step_;
  int gnc_max_iterations_;
  double gnc_cost_threshold_;
  double gnc_inlier_threshold_;

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
  gtsam::Pose2 current_pose_;
  gtsam::Pose2 last_keyframe_pose_;

  // Keyframes
  std::deque<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::deque<gtsam::Pose2> keyframe_poses_;
  std::deque<rclcpp::Time> keyframe_stamps_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamWithGncNode>());
  rclcpp::shutdown();
  return 0;
}
