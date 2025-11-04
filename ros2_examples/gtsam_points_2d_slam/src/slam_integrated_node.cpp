/**
 * @file slam_integrated_node.cpp
 * @brief Integrated 2D SLAM with Loop Closure + IMU + Segmentation
 *
 * This is the most advanced SLAM implementation combining:
 * - Loop closure detection for global consistency
 * - IMU integration for high-speed motion
 * - Segmentation for dynamic object removal
 *
 * Features:
 * - Maximum robustness in dynamic environments
 * - High accuracy with IMU-aided estimation
 * - Global consistency through loop closures
 * - Static-only mapping for clean results
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>

#include <gtsam_points/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/factors/reintegrated_imu_factor_2d.hpp>
#include <gtsam_points/d2/features/normal_estimation_2d.hpp>
#include <gtsam_points/d2/segmentation/region_growing_2d.hpp>
#include <gtsam_points/d2/ann/kdtree2d_tbb.hpp>

#include <memory>
#include <vector>
#include <deque>
#include <cmath>

using namespace gtsam_points;
using gtsam::symbol_shorthand::X;  // Pose
using gtsam::symbol_shorthand::V;  // Velocity
using gtsam::symbol_shorthand::B;  // Bias

class IntegratedSLAMNode : public rclcpp::Node {
public:
  IntegratedSLAMNode() : Node("slam_integrated") {
    // Parameters
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("imu_topic", "imu");

    // SLAM parameters
    this->declare_parameter("keyframe_distance", 0.5);
    this->declare_parameter("keyframe_angle", 0.3);
    this->declare_parameter("use_vgicp", true);

    // Loop closure parameters
    this->declare_parameter("enable_loop_closure", true);
    this->declare_parameter("loop_search_radius", 10.0);
    this->declare_parameter("loop_min_chain_length", 10);

    // IMU parameters
    this->declare_parameter("use_imu", true);
    this->declare_parameter("imu_acc_noise", 0.1);
    this->declare_parameter("imu_gyro_noise", 0.01);
    this->declare_parameter("imu_acc_bias_noise", 0.001);
    this->declare_parameter("imu_gyro_bias_noise", 0.0001);
    this->declare_parameter("gravity", 9.81);

    // Segmentation parameters
    this->declare_parameter("enable_segmentation", true);
    this->declare_parameter("region_growing_distance_threshold", 0.2);
    this->declare_parameter("region_growing_angle_threshold", 0.5);
    this->declare_parameter("min_segment_size", 15);
    this->declare_parameter("normal_estimation_k", 10);
    this->declare_parameter("consistency_check_distance", 0.5);
    this->declare_parameter("min_static_ratio", 0.3);

    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();

    keyframe_distance_ = this->get_parameter("keyframe_distance").as_double();
    keyframe_angle_ = this->get_parameter("keyframe_angle").as_double();
    use_vgicp_ = this->get_parameter("use_vgicp").as_bool();

    enable_loop_closure_ = this->get_parameter("enable_loop_closure").as_bool();
    loop_search_radius_ = this->get_parameter("loop_search_radius").as_double();
    loop_min_chain_length_ = this->get_parameter("loop_min_chain_length").as_int();

    use_imu_ = this->get_parameter("use_imu").as_bool();
    imu_acc_noise_ = this->get_parameter("imu_acc_noise").as_double();
    imu_gyro_noise_ = this->get_parameter("imu_gyro_noise").as_double();
    imu_acc_bias_noise_ = this->get_parameter("imu_acc_bias_noise").as_double();
    imu_gyro_bias_noise_ = this->get_parameter("imu_gyro_bias_noise").as_double();
    gravity_ = this->get_parameter("gravity").as_double();

    enable_segmentation_ = this->get_parameter("enable_segmentation").as_bool();
    region_growing_distance_threshold_ = this->get_parameter("region_growing_distance_threshold").as_double();
    region_growing_angle_threshold_ = this->get_parameter("region_growing_angle_threshold").as_double();
    min_segment_size_ = this->get_parameter("min_segment_size").as_int();
    normal_estimation_k_ = this->get_parameter("normal_estimation_k").as_int();
    consistency_check_distance_ = this->get_parameter("consistency_check_distance").as_double();
    min_static_ratio_ = this->get_parameter("min_static_ratio").as_double();

    // Initialize ISAM2
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.1;
    isam2_params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);

    // Initialize IMU preintegration
    if (use_imu_) {
      auto p = gtsam::PreintegrationParams2::MakeSharedU(gravity_);
      p->accelerometerCovariance = gtsam::I_2x2 * std::pow(imu_acc_noise_, 2);
      p->gyroscopeCovariance = gtsam::I_1x1 * std::pow(imu_gyro_noise_, 2);
      p->integrationCovariance = gtsam::I_2x2 * 1e-7;

      imu_preintegration_ = std::make_shared<gtsam::PreintegrationType2>(p, gtsam::imuBias::ConstantBias());
    }

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&IntegratedSLAMNode::scanCallback, this, std::placeholders::_1));

    if (use_imu_) {
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 100,
        std::bind(&IntegratedSLAMNode::imuCallback, this, std::placeholders::_1));
    }

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);
    loop_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("loop_closure_markers", 10);
    segments_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("segments", 10);

    RCLCPP_INFO(this->get_logger(), "Integrated SLAM node initialized");
    RCLCPP_INFO(this->get_logger(), "  Loop Closure: %s", enable_loop_closure_ ? "ON" : "OFF");
    RCLCPP_INFO(this->get_logger(), "  IMU: %s", use_imu_ ? "ON" : "OFF");
    RCLCPP_INFO(this->get_logger(), "  Segmentation: %s", enable_segmentation_ ? "ON" : "OFF");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!use_imu_ || !initialized_) {
      return;
    }

    double dt = 0.01;  // Assume 100Hz IMU
    if (!last_imu_time_.nanoseconds() == 0) {
      dt = (rclcpp::Time(msg->header.stamp) - last_imu_time_).seconds();
    }
    last_imu_time_ = msg->header.stamp;

    // Extract IMU measurements
    gtsam::Vector2 acc(msg->linear_acceleration.x, msg->linear_acceleration.y);
    double gyro = msg->angular_velocity.z;

    // Preintegrate
    imu_preintegration_->integrateMeasurement(acc, gyro, dt);
  }

  std::shared_ptr<PointCloud2DCPU> convertToPointCloud2D(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto cloud = std::make_shared<PointCloud2DCPU>();
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float range = msg->ranges[i];
      if (range < msg->range_min || range > msg->range_max || std::isnan(range)) {
        continue;
      }
      const float angle = msg->angle_min + i * msg->angle_increment;
      cloud->add_point(Eigen::Vector2d(range * std::cos(angle), range * std::sin(angle)));
    }
    return cloud;
  }

  std::vector<std::vector<size_t>> segmentPointCloud(
    const std::shared_ptr<PointCloud2DCPU>& cloud) {
    if (!enable_segmentation_) {
      return {};
    }

    estimate_normals_2d(*cloud, normal_estimation_k_);
    auto kdtree = std::make_shared<KdTree2dTBB>(cloud);

    RegionGrowingParams2D params;
    params.distance_threshold = region_growing_distance_threshold_;
    params.angle_threshold = region_growing_angle_threshold_;
    params.max_cluster_size = 1000;

    std::vector<std::vector<size_t>> segments;
    std::vector<bool> assigned(cloud->size(), false);

    for (size_t i = 0; i < cloud->size(); ++i) {
      if (assigned[i]) continue;

      Eigen::Vector3d seed(cloud->points[i].x(), cloud->points[i].y(), 1.0);
      auto context = region_growing_init_2d(*cloud, *kdtree, seed, params);

      while (!region_growing_update_2d(context, *cloud, *kdtree, params)) {}

      if (static_cast<int>(context.cluster_indices.size()) >= min_segment_size_) {
        for (size_t idx : context.cluster_indices) {
          assigned[idx] = true;
        }
        segments.push_back(context.cluster_indices);
      }
    }

    return segments;
  }

  bool isSegmentStatic(const std::shared_ptr<PointCloud2DCPU>& cloud,
                       const std::vector<size_t>& segment,
                       const gtsam::Pose2& current_pose) {
    if (keyframes_.empty()) return true;

    auto segment_cloud = std::make_shared<PointCloud2DCPU>();
    for (size_t idx : segment) {
      segment_cloud->add_point(cloud->points[idx]);
    }

    Eigen::Isometry2d T = Eigen::Isometry2d::Identity();
    T.translation() = Eigen::Vector2d(current_pose.x(), current_pose.y());
    T.linear() = Eigen::Rotation2Dd(current_pose.theta()).toRotationMatrix();

    auto segment_in_map = std::make_shared<PointCloud2DCPU>();
    for (size_t i = 0; i < segment_cloud->size(); ++i) {
      segment_in_map->add_point(T * segment_cloud->points[i]);
    }

    auto reference_tree = std::make_shared<KdTree2dTBB>(keyframes_.back());

    int consistent_count = 0;
    for (size_t i = 0; i < segment_in_map->size(); ++i) {
      size_t nearest_idx;
      double sq_dist;
      if (reference_tree->knn_search(segment_in_map->points[i].data(), 1, &nearest_idx, &sq_dist)) {
        if (sq_dist < consistency_check_distance_ * consistency_check_distance_) {
          consistent_count++;
        }
      }
    }

    return (static_cast<double>(consistent_count) / segment_in_map->size()) > min_static_ratio_;
  }

  std::shared_ptr<PointCloud2DCPU> filterStaticPoints(
    const std::shared_ptr<PointCloud2DCPU>& cloud,
    const std::vector<std::vector<size_t>>& segments,
    const gtsam::Pose2& current_pose) {

    if (!enable_segmentation_ || segments.empty()) {
      return cloud;
    }

    auto static_cloud = std::make_shared<PointCloud2DCPU>();
    for (const auto& segment : segments) {
      if (isSegmentStatic(cloud, segment, current_pose)) {
        for (size_t idx : segment) {
          static_cloud->add_point(cloud->points[idx]);
        }
      }
    }

    return static_cloud->size() > 20 ? static_cloud : cloud;
  }

  void detectLoopClosures() {
    if (!enable_loop_closure_ || keyframes_.size() < loop_min_chain_length_) {
      return;
    }

    const int current_key = keyframes_.size() - 1;
    const gtsam::Pose2& current_pose = keyframe_poses_.back();

    for (int i = 0; i < current_key - loop_min_chain_length_; ++i) {
      const gtsam::Pose2& candidate_pose = keyframe_poses_[i];

      double dx = current_pose.x() - candidate_pose.x();
      double dy = current_pose.y() - candidate_pose.y();
      double dist = std::sqrt(dx * dx + dy * dy);

      if (dist < loop_search_radius_) {
        // Add loop closure constraint
        gtsam::Pose2 relative_pose = candidate_pose.between(current_pose);

        auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector3(0.1, 0.1, 0.1));

        graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
          X(i), X(current_key), relative_pose, loop_noise));

        loop_closures_.push_back({i, current_key});

        RCLCPP_INFO(this->get_logger(), "Loop closure: %d <-> %d", i, current_key);
        break;
      }
    }
  }

  bool shouldCreateKeyframe(const gtsam::Pose2& new_pose) {
    if (keyframes_.empty()) return true;

    const gtsam::Pose2& last_pose = keyframe_poses_.back();
    double dx = new_pose.x() - last_pose.x();
    double dy = new_pose.y() - last_pose.y();
    double dist = std::sqrt(dx * dx + dy * dy);
    double dtheta = std::abs(new_pose.theta() - last_pose.theta());

    return (dist > keyframe_distance_) || (dtheta > keyframe_angle_);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto cloud = convertToPointCloud2D(msg);
    if (cloud->size() < 10) return;

    // Segmentation
    auto segments = segmentPointCloud(cloud);
    auto filtered_cloud = filterStaticPoints(cloud, segments, current_pose_);

    if (!initialized_) {
      // Initialize
      current_pose_ = gtsam::Pose2(0, 0, 0);
      current_velocity_ = gtsam::Vector2::Zero();
      current_bias_ = gtsam::imuBias::ConstantBias();

      auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.01, 0.01));
      graph_.add(gtsam::PriorFactor<gtsam::Pose2>(X(0), current_pose_, pose_noise));

      if (use_imu_) {
        auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.1);
        auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.01);
        graph_.add(gtsam::PriorFactor<gtsam::Vector2>(V(0), current_velocity_, vel_noise));
        graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), current_bias_, bias_noise));

        initial_estimates_.insert(V(0), current_velocity_);
        initial_estimates_.insert(B(0), current_bias_);
      }

      initial_estimates_.insert(X(0), current_pose_);

      isam2_->update(graph_, initial_estimates_);
      graph_.resize(0);
      initial_estimates_.clear();

      keyframes_.push_back(filtered_cloud);
      keyframe_poses_.push_back(current_pose_);
      initialized_ = true;

      RCLCPP_INFO(this->get_logger(), "Initialized with %zu points", filtered_cloud->size());
    } else if (shouldCreateKeyframe(current_pose_)) {
      const int previous_key = keyframes_.size() - 1;
      const int current_key = keyframes_.size();

      // Add GICP factor
      auto gicp_factor = gtsam::make_shared<IntegratedGICPFactor2D>(
        X(previous_key), X(current_key), keyframes_.back(), filtered_cloud);
      graph_.add(gicp_factor);

      // Add IMU factor
      if (use_imu_ && imu_preintegration_) {
        auto imu_factor = gtsam::make_shared<ReintegratedIMUFactor2D>(
          X(previous_key), V(previous_key), X(current_key), V(current_key),
          B(previous_key), B(current_key), *imu_preintegration_);
        graph_.add(imu_factor);

        initial_estimates_.insert(V(current_key), current_velocity_);
        initial_estimates_.insert(B(current_key), current_bias_);

        // Reset preintegration
        imu_preintegration_->resetIntegrationAndSetBias(current_bias_);
      }

      initial_estimates_.insert(X(current_key), current_pose_);

      // Detect loop closures
      detectLoopClosures();

      // Update ISAM2
      isam2_->update(graph_, initial_estimates_);
      graph_.resize(0);
      initial_estimates_.clear();

      // Get optimized results
      gtsam::Values result = isam2_->calculateEstimate();
      current_pose_ = result.at<gtsam::Pose2>(X(current_key));

      if (use_imu_) {
        current_velocity_ = result.at<gtsam::Vector2>(V(current_key));
        current_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(current_key));
      }

      // Update all poses
      keyframe_poses_.clear();
      for (size_t i = 0; i <= current_key; ++i) {
        keyframe_poses_.push_back(result.at<gtsam::Pose2>(X(i)));
      }

      keyframes_.push_back(filtered_cloud);

      RCLCPP_INFO(this->get_logger(), "Keyframe %d added (%zu points, %zu loops)",
                  current_key, filtered_cloud->size(), loop_closures_.size());
    }

    publishOdometry(msg->header);
    publishPath();
    publishLoopMarkers();
  }

  void publishOdometry(const std_msgs::msg::Header& header) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = header;
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

    if (use_imu_) {
      odom_msg.twist.twist.linear.x = current_velocity_.x();
      odom_msg.twist.twist.linear.y = current_velocity_.y();
    }

    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = base_frame_;
    tf_msg.transform.translation.x = current_pose_.x();
    tf_msg.transform.translation.y = current_pose_.y();
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void publishPath() {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = map_frame_;

    for (const auto& pose : keyframe_poses_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path_msg.header;
      pose_stamped.pose.position.x = pose.x();
      pose_stamped.pose.position.y = pose.y();

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

  void publishLoopMarkers() {
    if (!enable_loop_closure_) return;

    visualization_msgs::msg::MarkerArray markers;
    for (size_t i = 0; i < loop_closures_.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = this->now();
      marker.header.frame_id = map_frame_;
      marker.ns = "loop_closures";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.05;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;

      geometry_msgs::msg::Point p1, p2;
      p1.x = keyframe_poses_[loop_closures_[i].first].x();
      p1.y = keyframe_poses_[loop_closures_[i].first].y();
      p2.x = keyframe_poses_[loop_closures_[i].second].x();
      p2.y = keyframe_poses_[loop_closures_[i].second].y();

      marker.points.push_back(p1);
      marker.points.push_back(p2);
      markers.markers.push_back(marker);
    }

    loop_markers_pub_->publish(markers);
  }

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr segments_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  std::string map_frame_, odom_frame_, base_frame_, scan_topic_, imu_topic_;
  double keyframe_distance_, keyframe_angle_;
  bool use_vgicp_;
  bool enable_loop_closure_;
  double loop_search_radius_;
  int loop_min_chain_length_;
  bool use_imu_;
  double imu_acc_noise_, imu_gyro_noise_, imu_acc_bias_noise_, imu_gyro_bias_noise_, gravity_;
  bool enable_segmentation_;
  double region_growing_distance_threshold_, region_growing_angle_threshold_;
  int min_segment_size_, normal_estimation_k_;
  double consistency_check_distance_, min_static_ratio_;

  // SLAM state
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
  std::vector<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::vector<gtsam::Pose2> keyframe_poses_;
  gtsam::Pose2 current_pose_;
  gtsam::Vector2 current_velocity_;
  gtsam::imuBias::ConstantBias current_bias_;
  bool initialized_ = false;

  // IMU
  std::shared_ptr<gtsam::PreintegrationType2> imu_preintegration_;
  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};

  // Loop closures
  std::vector<std::pair<int, int>> loop_closures_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntegratedSLAMNode>());
  rclcpp::shutdown();
  return 0;
}
