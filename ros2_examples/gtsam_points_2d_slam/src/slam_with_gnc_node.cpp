/**
 * @file slam_with_gnc_node.cpp
 * @brief 2D SLAM with Graduated Non-Convexity for Robust Matching
 *
 * This implementation uses weighted ICP with graduated non-convexity (GNC)
 * to handle outliers. The GNC approach gradually increases robustness by
 * adjusting the μ parameter in the Geman-McClure robust cost function.
 *
 * Key Features:
 * - Weighted point-to-point ICP using align_points_se2
 * - Geman-McClure robust cost function for outlier rejection
 * - Graduated non-convexity: μ increases from mu_init to large value
 * - Per-point weights computed based on residuals
 * - More robust than standard ICP in presence of outliers
 */

#include <memory>
#include <deque>
#include <chrono>
#include <cmath>

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
#include <gtsam_points/d2/registration/alignment_2d.hpp>
#include <gtsam_points/d2/ann/kdtree2d_tbb.hpp>

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
    this->declare_parameter("gnc_max_iterations", 50);
    this->declare_parameter("gnc_convergence_threshold", 1e-6);
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
    gnc_convergence_threshold_ = this->get_parameter("gnc_convergence_threshold").as_double();
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
      const float range = msg->ranges[i];
      if (range >= msg->range_min && range <= msg->range_max && !std::isnan(range)) {
        const float angle = msg->angle_min + i * msg->angle_increment;
        cloud->add_point(Eigen::Vector2d(range * std::cos(angle), range * std::sin(angle)));
      }
    }

    if (cloud->size() < 10) {
      RCLCPP_WARN(this->get_logger(), "Too few points in scan: %zu", cloud->size());
      return;
    }

    // Initialize or add keyframe
    if (!initialized_) {
      initializeFirstKeyframe(cloud, msg->header.stamp);
    } else {
      // Perform GNC matching with previous keyframe
      gtsam::Pose2 relative_pose;
      double final_cost = 0.0;
      int num_inliers = 0;

      bool success = performGNCMatching(
        keyframes_.back(),
        cloud,
        relative_pose,
        final_cost,
        num_inliers
      );

      if (!success) {
        RCLCPP_WARN(this->get_logger(), "GNC matching failed");
        return;
      }

      // Update current pose estimate
      current_pose_ = current_pose_.compose(relative_pose);

      // Check if we should create a new keyframe
      if (shouldCreateKeyframe()) {
        addKeyframe(cloud, relative_pose, msg->header.stamp);
      }
    }

    // Publish odometry and path
    publishOdometry(msg->header.stamp);
    publishPath(msg->header.stamp);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    if (key_counter_ % 10 == 0) {
      RCLCPP_INFO(this->get_logger(), "Keyframes: %d, Time: %ld ms", key_counter_, duration);
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

  /**
   * @brief Compute Geman-McClure weight for a residual
   *
   * The Geman-McClure function is: ρ(r) = r^2 / (μ + r^2)
   * Weight is: w(r) = μ / (μ + r^2)^2
   *
   * As μ increases, the function becomes less robust (closer to L2)
   */
  double computeGemanMcClureWeight(double squared_residual, double mu) const
  {
    return mu / std::pow(mu + squared_residual, 2.0);
  }

  /**
   * @brief Perform GNC matching between target and source
   *
   * This implements graduated non-convexity by:
   * 1. Starting with small μ (robust to outliers)
   * 2. Finding correspondences
   * 3. Computing weights using Geman-McClure function
   * 4. Weighted alignment using align_points_se2
   * 5. Increasing μ and repeating until convergence
   */
  bool performGNCMatching(
    const std::shared_ptr<PointCloud2DCPU>& target,
    const std::shared_ptr<PointCloud2DCPU>& source,
    gtsam::Pose2& relative_pose,
    double& final_cost,
    int& num_inliers)
  {
    // Build kdtree for target
    auto target_tree = std::make_shared<KdTree2dTBB>(target);

    // Initialize transformation
    Eigen::Isometry2d T = Eigen::Isometry2d::Identity();

    double mu = gnc_mu_init_;
    double prev_cost = std::numeric_limits<double>::max();

    for (int iter = 0; iter < gnc_max_iterations_; ++iter) {
      // Find correspondences
      std::vector<Eigen::Vector3d> target_points;
      std::vector<Eigen::Vector3d> source_points;
      std::vector<double> weights;

      target_points.reserve(source->size());
      source_points.reserve(source->size());
      weights.reserve(source->size());

      for (size_t i = 0; i < source->size(); ++i) {
        // Transform source point
        Eigen::Vector2d src_transformed = T * source->points[i];

        // Find nearest neighbor in target
        size_t nearest_idx;
        double sq_dist;
        if (!target_tree->knn_search(src_transformed.data(), 1, &nearest_idx, &sq_dist)) {
          continue;
        }

        // Check distance threshold
        if (sq_dist > max_correspondence_distance_ * max_correspondence_distance_) {
          continue;
        }

        // Compute Geman-McClure weight
        double weight = computeGemanMcClureWeight(sq_dist, mu);

        // Store correspondence
        target_points.push_back(Eigen::Vector3d(target->points[nearest_idx].x(),
                                                target->points[nearest_idx].y(), 1.0));
        source_points.push_back(Eigen::Vector3d(source->points[i].x(),
                                                source->points[i].y(), 1.0));
        weights.push_back(weight);
      }

      if (target_points.size() < 10) {
        RCLCPP_WARN(this->get_logger(), "Too few correspondences: %zu", target_points.size());
        return false;
      }

      // Weighted alignment
      T = align_points_se2(target_points.data(), source_points.data(), weights.data(), target_points.size());

      // Compute cost
      double cost = 0.0;
      for (size_t i = 0; i < target_points.size(); ++i) {
        Eigen::Vector2d diff = target_points[i].head<2>() -
                               (T * source_points[i].head<2>());
        cost += weights[i] * diff.squaredNorm();
      }
      cost /= target_points.size();

      // Check convergence
      if (std::abs(prev_cost - cost) < gnc_convergence_threshold_) {
        final_cost = cost;
        num_inliers = 0;

        // Count inliers (points with high weight)
        for (double w : weights) {
          if (w > 0.5) num_inliers++;
        }

        break;
      }

      prev_cost = cost;

      // Increase μ (graduated non-convexity)
      mu *= gnc_mu_step_;
    }

    // Convert to gtsam::Pose2
    double x = T.translation().x();
    double y = T.translation().y();
    double theta = std::atan2(T.linear()(1, 0), T.linear()(0, 0));
    relative_pose = gtsam::Pose2(x, y, theta);

    return true;
  }

  bool shouldCreateKeyframe()
  {
    const double dx = current_pose_.x() - last_keyframe_pose_.x();
    const double dy = current_pose_.y() - last_keyframe_pose_.y();
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double dtheta = std::abs(current_pose_.theta() - last_keyframe_pose_.theta());

    return (dist > keyframe_distance_) || (dtheta > keyframe_angle_);
  }

  void addKeyframe(
    const std::shared_ptr<PointCloud2DCPU>& cloud,
    const gtsam::Pose2& relative_pose,
    const rclcpp::Time& stamp)
  {
    const int previous_key = key_counter_ - 1;
    const int current_key = key_counter_;

    // Add between factor
    auto between_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(3) << 0.05, 0.05, 0.05).finished());
    graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
      gtsam::Symbol('x', previous_key),
      gtsam::Symbol('x', current_key),
      relative_pose,
      between_noise
    ));

    // Add GICP factor for refinement
    auto gicp_factor = gtsam::make_shared<IntegratedGICPFactor2D>(
      gtsam::Symbol('x', previous_key),
      gtsam::Symbol('x', current_key),
      keyframes_.back(),
      cloud
    );
    graph_.add(gicp_factor);

    initial_estimates_.insert(gtsam::Symbol('x', current_key), current_pose_);

    // Update ISAM2
    isam2_->update(graph_, initial_estimates_);
    graph_.resize(0);
    initial_estimates_.clear();

    // Get optimized result
    gtsam::Values result = isam2_->calculateEstimate();
    current_pose_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', current_key));

    // Update all keyframe poses from optimized result
    for (size_t i = 0; i < keyframe_poses_.size(); ++i) {
      keyframe_poses_[i] = result.at<gtsam::Pose2>(gtsam::Symbol('x', i));
    }
    keyframe_poses_.push_back(current_pose_);

    // Store keyframe
    keyframes_.push_back(cloud);
    keyframe_stamps_.push_back(stamp);

    last_keyframe_pose_ = current_pose_;
    key_counter_++;

    RCLCPP_INFO(this->get_logger(), "Added keyframe %d at (%.2f, %.2f, %.2f)",
                current_key, current_pose_.x(), current_pose_.y(), current_pose_.theta());
  }

  void publishOdometry(const rclcpp::Time& stamp)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = current_pose_.x();
    odom_msg.pose.pose.position.y = current_pose_.y();
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.theta());
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom_msg);

    // Publish TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = current_pose_.x();
    tf_msg.transform.translation.y = current_pose_.y();
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishPath(const rclcpp::Time& stamp)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = map_frame_;

    for (const auto& pose : keyframe_poses_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path_msg.header;
      pose_stamped.pose.position.x = pose.x();
      pose_stamped.pose.position.y = pose.y();
      pose_stamped.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, pose.theta());
      pose_stamped.pose.orientation.x = q.x();
      pose_stamped.pose.orientation.y = q.y();
      pose_stamped.pose.orientation.z = q.z();
      pose_stamped.pose.orientation.w = q.w();

      path_msg.poses.push_back(pose_stamped);
    }

    path_pub_->publish(path_msg);
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
  double keyframe_distance_;
  double keyframe_angle_;
  double max_correspondence_distance_;

  double gnc_mu_init_;
  double gnc_mu_step_;
  int gnc_max_iterations_;
  double gnc_convergence_threshold_;
  double gnc_inlier_threshold_;

  // SLAM state
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;

  std::deque<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::vector<gtsam::Pose2> keyframe_poses_;
  std::deque<rclcpp::Time> keyframe_stamps_;

  gtsam::Pose2 current_pose_;
  gtsam::Pose2 last_keyframe_pose_;
  int key_counter_;
  bool initialized_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamWithGncNode>());
  rclcpp::shutdown();
  return 0;
}
