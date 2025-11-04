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
#include <gtsam_points/d2/registration/ransac_2d.hpp>

using namespace gtsam_points;

class SlamWithRansacNode : public rclcpp::Node
{
public:
  SlamWithRansacNode()
  : Node("slam_with_ransac"),
    key_counter_(0),
    initialized_(false),
    relocalization_enabled_(true)
  {
    // Declare parameters
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("keyframe_distance", 0.5);
    this->declare_parameter("keyframe_angle", 0.3);
    this->declare_parameter("max_correspondence_distance", 1.0);

    // RANSAC parameters
    this->declare_parameter("ransac_iterations", 100);
    this->declare_parameter("ransac_inlier_threshold", 0.1);
    this->declare_parameter("ransac_min_inliers", 50);
    this->declare_parameter("ransac_confidence", 0.99);

    // Relocalization parameters
    this->declare_parameter("enable_relocalization", true);
    this->declare_parameter("relocalization_distance", 3.0);
    this->declare_parameter("relocalization_score_threshold", 0.7);

    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    keyframe_distance_ = this->get_parameter("keyframe_distance").as_double();
    keyframe_angle_ = this->get_parameter("keyframe_angle").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();

    ransac_iterations_ = this->get_parameter("ransac_iterations").as_int();
    ransac_inlier_threshold_ = this->get_parameter("ransac_inlier_threshold").as_double();
    ransac_min_inliers_ = this->get_parameter("ransac_min_inliers").as_int();
    ransac_confidence_ = this->get_parameter("ransac_confidence").as_double();

    relocalization_enabled_ = this->get_parameter("enable_relocalization").as_bool();
    relocalization_distance_ = this->get_parameter("relocalization_distance").as_double();
    relocalization_score_threshold_ = this->get_parameter("relocalization_score_threshold").as_double();

    // Initialize ISAM2
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.1;
    isam2_params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);

    // Create subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&SlamWithRansacNode::scanCallback, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "RANSAC SLAM node initialized");
    RCLCPP_INFO(this->get_logger(), "RANSAC parameters: iterations=%d, inlier_threshold=%.3f, min_inliers=%d",
                ransac_iterations_, ransac_inlier_threshold_, ransac_min_inliers_);
    if (relocalization_enabled_) {
      RCLCPP_INFO(this->get_logger(), "Relocalization enabled: distance=%.2fm, score_threshold=%.2f",
                  relocalization_distance_, relocalization_score_threshold_);
    }
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

    // Try RANSAC-based matching with last keyframe
    gtsam::Pose2 relative_pose;
    double inlier_ratio = 0.0;
    bool match_success = performRANSACMatching(keyframes_.back(), cloud, relative_pose, inlier_ratio);

    if (!match_success) {
      // RANSAC failed - possible kidnapping or large motion
      RCLCPP_WARN(this->get_logger(), "RANSAC matching failed (inlier_ratio=%.3f)", inlier_ratio);

      if (relocalization_enabled_) {
        // Try relocalization against all keyframes
        bool relocalized = tryRelocalization(cloud, relative_pose, inlier_ratio);

        if (relocalized) {
          RCLCPP_INFO(this->get_logger(), "Relocalization successful!");
        } else {
          RCLCPP_ERROR(this->get_logger(), "Relocalization failed - skipping scan");
          return;
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Scan matching failed - skipping scan");
        return;
      }
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
      RCLCPP_INFO(this->get_logger(), "Keyframes: %d, Inlier ratio: %.3f, Processing time: %ld ms",
                  key_counter_, inlier_ratio, duration);
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

  bool performRANSACMatching(
    const std::shared_ptr<PointCloud2DCPU>& target,
    const std::shared_ptr<PointCloud2DCPU>& source,
    gtsam::Pose2& relative_pose,
    double& inlier_ratio)
  {
    // Use RANSAC for robust matching
    RANSAC2DParams params;
    params.ransac_iterations = ransac_iterations_;
    params.inlier_threshold = ransac_inlier_threshold_;
    params.min_inliers = ransac_min_inliers_;
    params.confidence = ransac_confidence_;

    RANSAC2D ransac(params);

    // Estimate transformation
    Eigen::Isometry2d T_estimate;
    std::vector<int> inliers;
    bool success = ransac.estimate(target, source, T_estimate, inliers);

    if (!success) {
      inlier_ratio = 0.0;
      return false;
    }

    // Calculate inlier ratio
    inlier_ratio = static_cast<double>(inliers.size()) / static_cast<double>(source->size());

    // Check if enough inliers
    if (inliers.size() < static_cast<size_t>(ransac_min_inliers_)) {
      return false;
    }

    // Convert Eigen::Isometry2d to gtsam::Pose2
    Eigen::Matrix3d T = T_estimate.matrix();
    double x = T(0, 2);
    double y = T(1, 2);
    double theta = std::atan2(T(1, 0), T(0, 0));

    relative_pose = gtsam::Pose2(x, y, theta);

    return true;
  }

  bool tryRelocalization(
    const std::shared_ptr<PointCloud2DCPU>& scan,
    gtsam::Pose2& relative_pose,
    double& best_inlier_ratio)
  {
    best_inlier_ratio = 0.0;
    int best_match_idx = -1;
    gtsam::Pose2 best_pose;

    // Try matching against all keyframes (except very recent ones)
    int num_keyframes_to_check = std::max(0, static_cast<int>(keyframes_.size()) - 5);

    for (int i = 0; i < num_keyframes_to_check; ++i) {
      gtsam::Pose2 pose;
      double inlier_ratio;
      bool success = performRANSACMatching(keyframes_[i], scan, pose, inlier_ratio);

      if (success && inlier_ratio > best_inlier_ratio) {
        best_inlier_ratio = inlier_ratio;
        best_match_idx = i;
        best_pose = pose;
      }
    }

    // Check if we found a good match
    if (best_match_idx >= 0 && best_inlier_ratio > relocalization_score_threshold_) {
      // Update current pose to the matched keyframe pose + relative transformation
      current_pose_ = keyframe_poses_[best_match_idx].compose(best_pose);
      relative_pose = best_pose;

      RCLCPP_INFO(this->get_logger(),
                  "Relocalized to keyframe %d with inlier ratio %.3f",
                  best_match_idx, best_inlier_ratio);
      return true;
    }

    return false;
  }

  void addKeyframe(
    const std::shared_ptr<PointCloud2DCPU>& cloud,
    const rclcpp::Time& stamp,
    const gtsam::Pose2& relative_pose)
  {
    gtsam::Symbol current_key('x', key_counter_);
    gtsam::Symbol previous_key('x', key_counter_ - 1);

    // Add between factor using RANSAC result
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

  // RANSAC parameters
  int ransac_iterations_;
  double ransac_inlier_threshold_;
  int ransac_min_inliers_;
  double ransac_confidence_;

  // Relocalization parameters
  bool relocalization_enabled_;
  double relocalization_distance_;
  double relocalization_score_threshold_;

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
  rclcpp::spin(std::make_shared<SlamWithRansacNode>());
  rclcpp::shutdown();
  return 0;
}
