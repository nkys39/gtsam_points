/**
 * @file slam_with_segmentation_node.cpp
 * @brief 2D SLAM with Region Growing Segmentation for Dynamic Object Removal
 *
 * This node performs 2D SLAM while segmenting scans into regions and filtering
 * out dynamic objects. Only static environment points are used for SLAM.
 *
 * Features:
 * - Region Growing segmentation for each scan (using region_growing_2d API)
 * - Dynamic object detection via scan-to-map consistency check
 * - Static-only SLAM for robust mapping in dynamic environments
 * - Semantic labeling of point cloud segments
 * - Visualization of segmented regions
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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

#include <gtsam_points/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/factors/integrated_vgicp_factor_2d.hpp>
#include <gtsam_points/d2/features/normal_estimation_2d.hpp>
#include <gtsam_points/d2/segmentation/region_growing_2d.hpp>
#include <gtsam_points/d2/ann/kdtree2d_tbb.hpp>

#include <memory>
#include <vector>
#include <cmath>
#include <unordered_set>

using namespace gtsam_points;

class SLAMWithSegmentationNode : public rclcpp::Node {
public:
  SLAMWithSegmentationNode() : Node("slam_with_segmentation") {
    // Parameters
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("scan_topic", "scan");

    this->declare_parameter("keyframe_distance", 0.5);
    this->declare_parameter("keyframe_angle", 0.3);
    this->declare_parameter("use_vgicp", true);
    this->declare_parameter("voxel_resolution", 0.1);

    // Segmentation parameters
    this->declare_parameter("region_growing_distance_threshold", 0.2);
    this->declare_parameter("region_growing_angle_threshold", 0.5);
    this->declare_parameter("min_segment_size", 15);
    this->declare_parameter("max_segment_size", 1000);
    this->declare_parameter("normal_estimation_k", 10);

    // Dynamic object detection parameters
    this->declare_parameter("consistency_check_distance", 0.5);
    this->declare_parameter("min_static_ratio", 0.3);

    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();

    keyframe_distance_ = this->get_parameter("keyframe_distance").as_double();
    keyframe_angle_ = this->get_parameter("keyframe_angle").as_double();
    use_vgicp_ = this->get_parameter("use_vgicp").as_bool();
    voxel_resolution_ = this->get_parameter("voxel_resolution").as_double();

    region_growing_distance_threshold_ = this->get_parameter("region_growing_distance_threshold").as_double();
    region_growing_angle_threshold_ = this->get_parameter("region_growing_angle_threshold").as_double();
    min_segment_size_ = this->get_parameter("min_segment_size").as_int();
    max_segment_size_ = this->get_parameter("max_segment_size").as_int();
    normal_estimation_k_ = this->get_parameter("normal_estimation_k").as_int();

    consistency_check_distance_ = this->get_parameter("consistency_check_distance").as_double();
    min_static_ratio_ = this->get_parameter("min_static_ratio").as_double();

    // Initialize ISAM2
    gtsam::ISAM2Params isam2_params;
    isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);

    // Initialize TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&SLAMWithSegmentationNode::scanCallback, this, std::placeholders::_1)
    );

    // Publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);
    segments_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("segments", 10);

    RCLCPP_INFO(this->get_logger(), "SLAM with Segmentation node initialized");
    RCLCPP_INFO(this->get_logger(), "Region Growing distance: %.2f m, angle: %.2f rad",
                region_growing_distance_threshold_, region_growing_angle_threshold_);
    RCLCPP_INFO(this->get_logger(), "Min segment size: %d points", min_segment_size_);
  }

private:
  /**
   * @brief Convert LaserScan to PointCloud2D
   */
  std::shared_ptr<PointCloud2DCPU> convertToPointCloud2D(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    auto cloud = std::make_shared<PointCloud2DCPU>();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float range = msg->ranges[i];

      if (range < msg->range_min || range > msg->range_max || std::isnan(range)) {
        continue;
      }

      const float angle = msg->angle_min + i * msg->angle_increment;
      const float x = range * std::cos(angle);
      const float y = range * std::sin(angle);

      cloud->add_point(Eigen::Vector2d(x, y));
    }

    return cloud;
  }

  /**
   * @brief Segment point cloud using Region Growing
   *
   * Uses the gtsam_points region_growing_2d API which requires:
   * 1. Normal estimation first
   * 2. KD-tree for neighbor search
   * 3. Multiple seed points and region growing loops
   */
  std::vector<std::vector<size_t>> segmentPointCloud(
    const std::shared_ptr<PointCloud2DCPU>& cloud) {

    // Step 1: Estimate normals
    estimate_normals_2d(*cloud, normal_estimation_k_);

    // Step 2: Build KD-tree for neighbor search
    auto kdtree = std::make_shared<KdTree2dTBB>(cloud);

    // Step 3: Region growing parameters
    RegionGrowingParams2D params;
    params.distance_threshold = region_growing_distance_threshold_;
    params.angle_threshold = region_growing_angle_threshold_;
    params.max_cluster_size = max_segment_size_;

    // Step 4: Perform region growing for multiple segments
    std::vector<std::vector<size_t>> segments;
    std::vector<bool> assigned(cloud->size(), false);

    // Try each unassigned point as a seed
    for (size_t i = 0; i < cloud->size(); ++i) {
      if (assigned[i]) {
        continue;
      }

      // Use this point as seed
      Eigen::Vector3d seed_point(cloud->points[i].x(), cloud->points[i].y(), 1.0);

      // Initialize region growing from this seed
      auto context = region_growing_init_2d(*cloud, *kdtree, seed_point, params);

      // Grow the region
      while (!region_growing_update_2d(context, *cloud, *kdtree, params)) {
        // Continue growing
      }

      // Check segment size
      if (static_cast<int>(context.cluster_indices.size()) >= min_segment_size_ &&
          static_cast<int>(context.cluster_indices.size()) <= max_segment_size_) {
        // Mark points as assigned
        for (size_t idx : context.cluster_indices) {
          assigned[idx] = true;
        }
        segments.push_back(context.cluster_indices);
      }
    }

    return segments;
  }

  /**
   * @brief Check if a segment is static by comparing with the map
   */
  bool isSegmentStatic(
    const std::shared_ptr<PointCloud2DCPU>& cloud,
    const std::vector<size_t>& segment,
    const gtsam::Pose2& current_pose) {

    // If no map yet, consider all segments as static
    if (keyframes_.empty()) {
      return true;
    }

    // Create a point cloud from the segment
    auto segment_cloud = std::make_shared<PointCloud2DCPU>();
    for (size_t idx : segment) {
      segment_cloud->add_point(cloud->points[idx]);
    }

    // Transform segment to map frame
    Eigen::Isometry2d T_map_sensor = Eigen::Isometry2d::Identity();
    T_map_sensor.translation() = Eigen::Vector2d(current_pose.x(), current_pose.y());
    T_map_sensor.linear() = Eigen::Rotation2Dd(current_pose.theta()).toRotationMatrix();

    auto segment_in_map = std::make_shared<PointCloud2DCPU>();
    for (size_t i = 0; i < segment_cloud->size(); ++i) {
      Eigen::Vector2d p_map = T_map_sensor * segment_cloud->points[i];
      segment_in_map->add_point(p_map);
    }

    // Check consistency with the latest keyframe
    const auto& reference_cloud = keyframes_.back();

    // Build KD-tree for reference cloud
    auto reference_tree = std::make_shared<KdTree2dTBB>(reference_cloud);

    // Simple consistency check: for each point in segment, find nearest in reference
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

    double consistency_ratio = static_cast<double>(consistent_count) / segment_in_map->size();

    // Segment is static if most points are consistent with the map
    return consistency_ratio > min_static_ratio_;
  }

  /**
   * @brief Filter point cloud to keep only static points
   */
  std::shared_ptr<PointCloud2DCPU> filterStaticPoints(
    const std::shared_ptr<PointCloud2DCPU>& cloud,
    const std::vector<std::vector<size_t>>& segments,
    const gtsam::Pose2& current_pose,
    std::vector<bool>& segment_labels) {

    auto static_cloud = std::make_shared<PointCloud2DCPU>();
    segment_labels.clear();

    // Check each segment
    for (const auto& segment : segments) {
      bool is_static = isSegmentStatic(cloud, segment, current_pose);
      segment_labels.push_back(is_static);

      if (is_static) {
        // Add all points from static segment
        for (size_t idx : segment) {
          static_cloud->add_point(cloud->points[idx]);
        }
      }
    }

    return static_cloud;
  }

  /**
   * @brief Publish visualization of segments
   */
  void publishSegmentVisualization(
    const std::shared_ptr<PointCloud2DCPU>& cloud,
    const std::vector<std::vector<size_t>>& segments,
    const std::vector<bool>& segment_labels,
    const gtsam::Pose2& pose,
    const std_msgs::msg::Header& header) {

    visualization_msgs::msg::MarkerArray marker_array;

    // Transform to map frame
    Eigen::Isometry2d T_map_sensor = Eigen::Isometry2d::Identity();
    T_map_sensor.translation() = Eigen::Vector2d(pose.x(), pose.y());
    T_map_sensor.linear() = Eigen::Rotation2Dd(pose.theta()).toRotationMatrix();

    for (size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx) {
      visualization_msgs::msg::Marker marker;
      marker.header = header;
      marker.header.frame_id = map_frame_;
      marker.ns = "segments";
      marker.id = seg_idx;
      marker.type = visualization_msgs::msg::Marker::POINTS;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;

      // Color based on static/dynamic
      bool is_static = segment_labels[seg_idx];
      if (is_static) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
      } else {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
      }

      // Add points
      for (size_t idx : segments[seg_idx]) {
        geometry_msgs::msg::Point p;
        Eigen::Vector2d p_map = T_map_sensor * cloud->points[idx];
        p.x = p_map.x();
        p.y = p_map.y();
        p.z = 0.0;
        marker.points.push_back(p);
      }

      marker_array.markers.push_back(marker);
    }

    segments_pub_->publish(marker_array);
  }

  /**
   * @brief Check if we should create a new keyframe
   */
  bool shouldCreateKeyframe(const gtsam::Pose2& new_pose) {
    if (keyframes_.empty()) {
      return true;
    }

    const gtsam::Pose2& last_pose = keyframe_poses_.back();
    const double dx = new_pose.x() - last_pose.x();
    const double dy = new_pose.y() - last_pose.y();
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double dtheta = std::abs(new_pose.theta() - last_pose.theta());

    return (dist > keyframe_distance_) || (dtheta > keyframe_angle_);
  }

  /**
   * @brief Main scan callback with segmentation
   */
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Convert to point cloud
    auto cloud = convertToPointCloud2D(msg);

    if (cloud->size() < 10) {
      RCLCPP_WARN(this->get_logger(), "Too few points in scan");
      return;
    }

    // Segment the point cloud
    auto segments = segmentPointCloud(cloud);

    RCLCPP_DEBUG(this->get_logger(), "Segmented scan into %zu regions", segments.size());

    if (keyframes_.empty()) {
      // First scan - initialize
      gtsam::Pose2 initial_pose(0.0, 0.0, 0.0);
      current_pose_ = initial_pose;

      // Filter static points
      std::vector<bool> segment_labels;
      auto static_cloud = filterStaticPoints(cloud, segments, current_pose_, segment_labels);

      // Add prior factor
      auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.01, 0.01, 0.01));
      graph_.add(gtsam::PriorFactor<gtsam::Pose2>(
        gtsam::Symbol('x', 0), initial_pose, noise_model));

      initial_estimates_.insert(gtsam::Symbol('x', 0), initial_pose);

      // Update ISAM2
      isam2_->update(graph_, initial_estimates_);
      graph_.resize(0);
      initial_estimates_.clear();

      // Store keyframe (static points only)
      keyframes_.push_back(static_cloud);
      keyframe_poses_.push_back(initial_pose);

      // Publish visualization
      publishSegmentVisualization(cloud, segments, segment_labels, current_pose_, msg->header);

      RCLCPP_INFO(this->get_logger(),
        "Initialized with first keyframe (%zu static points from %zu total)",
        static_cloud->size(), cloud->size());

    } else {
      // Filter static points using current estimate
      std::vector<bool> segment_labels;
      auto static_cloud = filterStaticPoints(cloud, segments, current_pose_, segment_labels);

      // Count static vs dynamic points
      size_t static_points = static_cloud->size();
      size_t total_points = cloud->size();
      double static_ratio = static_cast<double>(static_points) / total_points;

      RCLCPP_DEBUG(this->get_logger(),
        "Static points: %zu / %zu (%.1f%%)",
        static_points, total_points, static_ratio * 100.0);

      // Check if we have enough static points
      if (static_points < 20) {
        RCLCPP_WARN(this->get_logger(), "Too few static points, skipping frame");
        return;
      }

      // Check if we need a new keyframe
      if (shouldCreateKeyframe(current_pose_)) {
        const int previous_key = keyframes_.size() - 1;
        const int current_key = keyframes_.size();

        // Create GICP/VGICP factor (using static points only)
        std::shared_ptr<gtsam::NoiseModelFactor> gicp_factor;
        if (use_vgicp_) {
          gicp_factor = gtsam::make_shared<IntegratedVGICPFactor2D>(
            gtsam::Symbol('x', previous_key),
            gtsam::Symbol('x', current_key),
            keyframes_.back(),
            static_cloud
          );
        } else {
          gicp_factor = gtsam::make_shared<IntegratedGICPFactor2D>(
            gtsam::Symbol('x', previous_key),
            gtsam::Symbol('x', current_key),
            keyframes_.back(),
            static_cloud
          );
        }

        graph_.add(gicp_factor);
        initial_estimates_.insert(gtsam::Symbol('x', current_key), current_pose_);

        // Update ISAM2
        isam2_->update(graph_, initial_estimates_);
        graph_.resize(0);
        initial_estimates_.clear();

        // Get optimized result
        gtsam::Values result = isam2_->calculateEstimate();
        current_pose_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', current_key));

        // Store keyframe
        keyframes_.push_back(static_cloud);
        keyframe_poses_.push_back(current_pose_);

        RCLCPP_INFO(this->get_logger(),
          "Added keyframe %d (%.1f%% static, %zu static points)",
          current_key, static_ratio * 100.0, static_points);
      }

      // Publish visualization
      publishSegmentVisualization(cloud, segments, segment_labels, current_pose_, msg->header);
    }

    // Publish odometry and path
    publishOdometry(msg->header);
    publishPath();
  }

  /**
   * @brief Publish odometry message
   */
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

  /**
   * @brief Publish path message
   */
  void publishPath() {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr segments_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;

  double keyframe_distance_;
  double keyframe_angle_;
  bool use_vgicp_;
  double voxel_resolution_;

  double region_growing_distance_threshold_;
  double region_growing_angle_threshold_;
  int min_segment_size_;
  int max_segment_size_;
  int normal_estimation_k_;

  double consistency_check_distance_;
  double min_static_ratio_;

  // SLAM state
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;

  std::vector<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::vector<gtsam::Pose2> keyframe_poses_;
  gtsam::Pose2 current_pose_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SLAMWithSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
