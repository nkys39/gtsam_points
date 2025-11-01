#include <memory>
#include <deque>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>

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

#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp>
#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>

// Include generated service headers
#include "gtsam_points_2d_slam/srv/save_map.hpp"
#include "gtsam_points_2d_slam/srv/load_map.hpp"

using namespace gtsam_points;
namespace fs = std::filesystem;

class SlamWithMapSaveNode : public rclcpp::Node
{
public:
  SlamWithMapSaveNode()
  : Node("slam_with_map_save"),
    key_counter_(0),
    initialized_(false)
  {
    // Declare parameters (same as slam_node)
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
      std::bind(&SlamWithMapSaveNode::scanCallback, this, std::placeholders::_1));

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Create services for map save/load
    save_map_service_ = this->create_service<gtsam_points_2d_slam::srv::SaveMap>(
      "save_map",
      std::bind(&SlamWithMapSaveNode::saveMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    load_map_service_ = this->create_service<gtsam_points_2d_slam::srv::LoadMap>(
      "load_map",
      std::bind(&SlamWithMapSaveNode::loadMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    // Initialize current pose
    current_pose_ = gtsam::Pose2(0.0, 0.0, 0.0);
    last_keyframe_pose_ = current_pose_;

    RCLCPP_INFO(this->get_logger(), "SLAM with Map Save/Load Node initialized");
    RCLCPP_INFO(this->get_logger(), "Services: ~/save_map, ~/load_map");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto scan = convertLaserScan(msg);
    if (!scan || scan->size() < 10) {
      return;
    }

    if (!initialized_) {
      initializeFirstScan(scan, msg->header.stamp);
      return;
    }

    // Check if we should add a new keyframe
    const double dist = std::hypot(
      current_pose_.x() - last_keyframe_pose_.x(),
      current_pose_.y() - last_keyframe_pose_.y());
    const double angle_diff = std::abs(current_pose_.theta() - last_keyframe_pose_.theta());

    if (dist >= keyframe_distance_ || angle_diff >= keyframe_angle_) {
      addKeyframe(scan, msg->header.stamp);
    }

    publishOdometry(msg->header.stamp);
    publishPath(msg->header.stamp);
  }

  std::shared_ptr<PointCloud2DCPU> convertLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr& msg)
  {
    auto cloud = std::make_shared<PointCloud2DCPU>();

    std::vector<Eigen::Vector3d> points;
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
    auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.01, 0.01, 0.01));

    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(
      gtsam::Symbol('x', key_counter_), current_pose_, prior_noise));

    initial_estimates_.insert(gtsam::Symbol('x', key_counter_), current_pose_);

    isam2_->update(graph_, initial_estimates_);
    graph_.resize(0);
    initial_estimates_.clear();

    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_);
    keyframe_stamps_.push_back(stamp);
    last_keyframe_pose_ = current_pose_;

    if (use_vgicp_) {
      gridmap_->insert(scan);
    }

    key_counter_++;
    initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "First scan initialized");
  }

  void addKeyframe(
    const std::shared_ptr<PointCloud2DCPU>& scan,
    const rclcpp::Time& stamp)
  {
    // Simple odometry estimation
    const gtsam::Pose2 delta(0.1, 0.0, 0.0);
    current_pose_ = last_keyframe_pose_ * delta;

    // Add factor
    auto target = keyframes_.back();

    if (use_vgicp_) {
      auto factor = gtsam::make_shared<IntegratedVGICPFactor2D>(
        gtsam::Symbol('x', key_counter_),
        target, scan, gridmap_);
      factor->set_max_correspondence_distance(max_correspondence_distance_);
      factor->set_num_threads(4);
      graph_.add(factor);
    } else {
      auto factor = gtsam::make_shared<IntegratedGICPFactor2D>(
        gtsam::Symbol('x', key_counter_),
        target, scan, nullptr);
      factor->set_max_correspondence_distance(max_correspondence_distance_);
      factor->set_num_threads(4);
      graph_.add(factor);
    }

    initial_estimates_.insert(gtsam::Symbol('x', key_counter_), current_pose_);

    isam2_->update(graph_, initial_estimates_);
    graph_.resize(0);
    initial_estimates_.clear();

    auto result = isam2_->calculateEstimate();
    current_pose_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', key_counter_));

    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_);
    keyframe_stamps_.push_back(stamp);
    last_keyframe_pose_ = current_pose_;

    if (use_vgicp_) {
      gridmap_->insert(scan);
    }

    key_counter_++;

    RCLCPP_INFO(this->get_logger(), "Added keyframe %d", key_counter_ - 1);
  }

  void publishOdometry(const rclcpp::Time& stamp)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = current_pose_.x();
    odom.pose.pose.position.y = current_pose_.y();
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, current_pose_.theta());
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = current_pose_.x();
    transform.transform.translation.y = current_pose_.y();
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
      auto pose = result.at<gtsam::Pose2>(gtsam::Symbol('x', i));

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

  // Map save callback
  void saveMapCallback(
    const std::shared_ptr<gtsam_points_2d_slam::srv::SaveMap::Request> request,
    std::shared_ptr<gtsam_points_2d_slam::srv::SaveMap::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Saving map to: %s/%s",
                request->directory_path.c_str(), request->map_name.c_str());

    try {
      // Create directory structure
      fs::path map_dir = fs::path(request->directory_path) / request->map_name;
      fs::path keyframes_dir = map_dir / "keyframes";
      fs::create_directories(keyframes_dir);

      // Save keyframes as PCD files
      for (size_t i = 0; i < keyframes_.size(); ++i) {
        std::ostringstream filename;
        filename << "keyframe_" << std::setw(6) << std::setfill('0') << i << ".pcd";
        fs::path pcd_path = keyframes_dir / filename.str();

        savePointCloudPCD(keyframes_[i], pcd_path.string());
      }

      // Save poses as JSON
      fs::path poses_path = map_dir / "poses.json";
      savePosesJSON(poses_path.string());

      // Save map info
      fs::path info_path = map_dir / "map_info.json";
      saveMapInfoJSON(info_path.string(), request->map_name);

      response->success = true;
      response->message = "Map saved successfully";
      response->num_keyframes = keyframes_.size();

      RCLCPP_INFO(this->get_logger(), "Map saved: %zu keyframes", keyframes_.size());
    }
    catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to save map: ") + e.what();
      response->num_keyframes = 0;

      RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", e.what());
    }
  }

  // Map load callback (basic implementation)
  void loadMapCallback(
    const std::shared_ptr<gtsam_points_2d_slam::srv::LoadMap::Request> request,
    std::shared_ptr<gtsam_points_2d_slam::srv::LoadMap::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Loading map from: %s/%s",
                request->directory_path.c_str(), request->map_name.c_str());

    // TODO: Implement map loading
    // This is a placeholder - full implementation would:
    // 1. Load all PCD files
    // 2. Load poses from JSON
    // 3. Reconstruct ISAM2 graph
    // 4. Set initialized_ = true

    response->success = false;
    response->message = "Map loading not yet implemented";
    response->num_keyframes = 0;

    RCLCPP_WARN(this->get_logger(), "Map loading not yet implemented");
  }

  // Helper: Save point cloud as PCD
  void savePointCloudPCD(
    const std::shared_ptr<PointCloud2DCPU>& cloud,
    const std::string& filename)
  {
    std::ofstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file: " + filename);
    }

    const size_t num_points = cloud->size();

    // PCD header
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y\n";
    file << "SIZE 4 4\n";
    file << "TYPE F F\n";
    file << "COUNT 1 1\n";
    file << "WIDTH " << num_points << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << num_points << "\n";
    file << "DATA ascii\n";

    // Point data
    for (size_t i = 0; i < num_points; ++i) {
      const auto& pt = cloud->points[i];
      file << pt.x() << " " << pt.y() << "\n";
    }

    file.close();
  }

  // Helper: Save poses as JSON
  void savePosesJSON(const std::string& filename)
  {
    std::ofstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file: " + filename);
    }

    auto result = isam2_->calculateEstimate();

    file << "{\n";
    file << "  \"poses\": [\n";

    for (int i = 0; i < key_counter_; ++i) {
      auto pose = result.at<gtsam::Pose2>(gtsam::Symbol('x', i));

      file << "    {\n";
      file << "      \"key\": " << i << ",\n";
      file << "      \"x\": " << pose.x() << ",\n";
      file << "      \"y\": " << pose.y() << ",\n";
      file << "      \"theta\": " << pose.theta() << "\n";
      file << "    }";

      if (i < key_counter_ - 1) {
        file << ",";
      }
      file << "\n";
    }

    file << "  ]\n";
    file << "}\n";

    file.close();
  }

  // Helper: Save map info as JSON
  void saveMapInfoJSON(const std::string& filename, const std::string& map_name)
  {
    std::ofstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file: " + filename);
    }

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    file << "{\n";
    file << "  \"map_name\": \"" << map_name << "\",\n";
    file << "  \"num_keyframes\": " << keyframes_.size() << ",\n";
    file << "  \"created_at\": \"" << std::ctime(&time_t) << "\",\n";
    file << "  \"use_vgicp\": " << (use_vgicp_ ? "true" : "false") << "\n";
    file << "}\n";

    file.close();
  }

private:
  // ROS parameters
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  bool use_vgicp_;
  double keyframe_distance_;
  double keyframe_angle_;
  double voxel_resolution_;
  double max_correspondence_distance_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Services
  rclcpp::Service<gtsam_points_2d_slam::srv::SaveMap>::SharedPtr save_map_service_;
  rclcpp::Service<gtsam_points_2d_slam::srv::LoadMap>::SharedPtr load_map_service_;

  // GTSAM
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
  std::shared_ptr<IncrementalGridMap2D> gridmap_;

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
  rclcpp::spin(std::make_shared<SlamWithMapSaveNode>());
  rclcpp::shutdown();
  return 0;
}
