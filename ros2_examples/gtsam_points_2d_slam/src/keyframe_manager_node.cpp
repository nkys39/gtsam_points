/**
 * @file keyframe_manager_node.cpp
 * @brief Keyframe Management Utilities for Map Optimization
 *
 * This tool provides advanced keyframe management operations:
 * - Merge redundant keyframes
 * - Remove unnecessary keyframes
 * - Downsample pose graph (thinning)
 * - Optimize map structure
 *
 * Use cases:
 * - Reduce map size for storage efficiency
 * - Remove duplicate/redundant data
 * - Simplify graphs for faster processing
 * - Clean up maps from multiple sessions
 */

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam_points/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/ann/kdtree2d_tbb.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include <cmath>

#include "gtsam_points_2d_slam/srv/load_map.hpp"
#include "gtsam_points_2d_slam/srv/save_map.hpp"
#include "gtsam_points_2d_slam/srv/manage_keyframes.hpp"

using namespace gtsam_points;
namespace fs = std::filesystem;

class KeyframeManagerNode : public rclcpp::Node {
public:
  KeyframeManagerNode() : Node("keyframe_manager") {
    // Parameters
    this->declare_parameter("merge_distance_threshold", 0.3);
    this->declare_parameter("merge_angle_threshold", 0.2);
    this->declare_parameter("thinning_factor", 2);
    this->declare_parameter("min_keyframes", 10);

    merge_distance_threshold_ = this->get_parameter("merge_distance_threshold").as_double();
    merge_angle_threshold_ = this->get_parameter("merge_angle_threshold").as_double();
    thinning_factor_ = this->get_parameter("thinning_factor").as_int();
    min_keyframes_ = this->get_parameter("min_keyframes").as_int();

    // Services
    load_map_service_ = this->create_service<gtsam_points_2d_slam::srv::LoadMap>(
      "load_map",
      std::bind(&KeyframeManagerNode::loadMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    save_map_service_ = this->create_service<gtsam_points_2d_slam::srv::SaveMap>(
      "save_map",
      std::bind(&KeyframeManagerNode::saveMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    merge_keyframes_service_ = this->create_service<std_srvs::srv::Empty>(
      "merge_redundant_keyframes",
      std::bind(&KeyframeManagerNode::mergeKeyframesCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    thin_keyframes_service_ = this->create_service<std_srvs::srv::Empty>(
      "thin_keyframes",
      std::bind(&KeyframeManagerNode::thinKeyframesCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Keyframe Manager node initialized");
    RCLCPP_INFO(this->get_logger(), "  Merge threshold: %.2f m, %.2f rad",
                merge_distance_threshold_, merge_angle_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Thinning factor: %d", thinning_factor_);
  }

private:
  struct PoseData {
    int key;
    double x;
    double y;
    double theta;
  };

  struct MapInfo {
    std::string map_name;
    int num_keyframes;
    std::string created_at;
    bool use_vgicp;
  };

  void loadMapCallback(
    const std::shared_ptr<gtsam_points_2d_slam::srv::LoadMap::Request> request,
    std::shared_ptr<gtsam_points_2d_slam::srv::LoadMap::Response> response) {

    fs::path map_path = fs::path(request->directory_path) / request->map_name;

    if (!fs::exists(map_path)) {
      response->success = false;
      response->message = "Map directory not found: " + map_path.string();
      return;
    }

    // Load map info
    MapInfo map_info = loadMapInfo(map_path / "map_info.json");
    if (map_info.num_keyframes == 0) {
      response->success = false;
      response->message = "Failed to load map info";
      return;
    }

    // Load poses
    auto poses = loadPoses(map_path / "poses.json");
    if (poses.empty()) {
      response->success = false;
      response->message = "Failed to load poses";
      return;
    }

    // Load keyframe point clouds
    keyframes_.clear();
    poses_.clear();

    for (int i = 0; i < map_info.num_keyframes; ++i) {
      std::string filename = "keyframe_" + std::to_string(i).insert(0, 6 - std::to_string(i).length(), '0') + ".pcd";
      fs::path pcd_path = map_path / "keyframes" / filename;

      auto cloud = loadPointCloudPCD(pcd_path);
      if (!cloud || cloud->size() == 0) {
        RCLCPP_WARN(this->get_logger(), "Failed to load keyframe %d", i);
        continue;
      }

      keyframes_.push_back(cloud);
      poses_.push_back(gtsam::Pose2(poses[i].x, poses[i].y, poses[i].theta));
    }

    response->success = true;
    response->message = "Loaded map: " + request->map_name;
    response->num_keyframes = keyframes_.size();

    RCLCPP_INFO(this->get_logger(), "Loaded %zu keyframes from %s",
                keyframes_.size(), request->map_name.c_str());
  }

  void saveMapCallback(
    const std::shared_ptr<gtsam_points_2d_slam::srv::SaveMap::Request> request,
    std::shared_ptr<gtsam_points_2d_slam::srv::SaveMap::Response> response) {

    if (keyframes_.empty()) {
      response->success = false;
      response->message = "No keyframes to save";
      return;
    }

    fs::path map_path = fs::path(request->directory_path) / request->map_name;
    fs::create_directories(map_path / "keyframes");

    // Save map info
    std::ofstream info_file(map_path / "map_info.json");
    info_file << "{\n";
    info_file << "  \"map_name\": \"" << request->map_name << "\",\n";
    info_file << "  \"num_keyframes\": " << keyframes_.size() << ",\n";
    info_file << "  \"created_at\": \"" << std::string(std::ctime(&std::time(nullptr))) << "\",\n";
    info_file << "  \"use_vgicp\": true\n";
    info_file << "}\n";
    info_file.close();

    // Save poses
    std::ofstream poses_file(map_path / "poses.json");
    poses_file << "{\n  \"poses\": [\n";
    for (size_t i = 0; i < poses_.size(); ++i) {
      poses_file << "    {\"key\": " << i << ", ";
      poses_file << "\"x\": " << poses_[i].x() << ", ";
      poses_file << "\"y\": " << poses_[i].y() << ", ";
      poses_file << "\"theta\": " << poses_[i].theta() << "}";
      if (i < poses_.size() - 1) poses_file << ",";
      poses_file << "\n";
    }
    poses_file << "  ]\n}\n";
    poses_file.close();

    // Save keyframes
    for (size_t i = 0; i < keyframes_.size(); ++i) {
      std::string filename = "keyframe_" + std::to_string(i).insert(0, 6 - std::to_string(i).length(), '0') + ".pcd";
      savePointCloudPCD(keyframes_[i], map_path / "keyframes" / filename);
    }

    response->success = true;
    response->message = "Saved map: " + request->map_name;
    response->num_keyframes = keyframes_.size();

    RCLCPP_INFO(this->get_logger(), "Saved %zu keyframes to %s",
                keyframes_.size(), request->map_name.c_str());
  }

  void mergeKeyframesCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {

    if (keyframes_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No keyframes loaded");
      return;
    }

    size_t original_count = keyframes_.size();
    std::vector<std::shared_ptr<PointCloud2DCPU>> merged_keyframes;
    std::vector<gtsam::Pose2> merged_poses;
    std::vector<bool> merged(keyframes_.size(), false);

    for (size_t i = 0; i < keyframes_.size(); ++i) {
      if (merged[i]) continue;

      // Start a new merged keyframe
      auto merged_cloud = std::make_shared<PointCloud2DCPU>();
      *merged_cloud = *keyframes_[i];
      gtsam::Pose2 avg_pose = poses_[i];
      int merge_count = 1;

      // Find nearby keyframes to merge
      for (size_t j = i + 1; j < keyframes_.size(); ++j) {
        if (merged[j]) continue;

        double dx = poses_[j].x() - poses_[i].x();
        double dy = poses_[j].y() - poses_[i].y();
        double dist = std::sqrt(dx * dx + dy * dy);
        double dtheta = std::abs(poses_[j].theta() - poses_[i].theta());

        if (dist < merge_distance_threshold_ && dtheta < merge_angle_threshold_) {
          // Merge this keyframe
          for (const auto& point : keyframes_[j]->points) {
            merged_cloud->add_point(point);
          }
          avg_pose = gtsam::Pose2(
            (avg_pose.x() * merge_count + poses_[j].x()) / (merge_count + 1),
            (avg_pose.y() * merge_count + poses_[j].y()) / (merge_count + 1),
            (avg_pose.theta() * merge_count + poses_[j].theta()) / (merge_count + 1)
          );
          merge_count++;
          merged[j] = true;
        }
      }

      merged_keyframes.push_back(merged_cloud);
      merged_poses.push_back(avg_pose);
      merged[i] = true;
    }

    keyframes_ = merged_keyframes;
    poses_ = merged_poses;

    RCLCPP_INFO(this->get_logger(), "Merged keyframes: %zu -> %zu (removed %zu redundant)",
                original_count, keyframes_.size(), original_count - keyframes_.size());
  }

  void thinKeyframesCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {

    if (keyframes_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No keyframes loaded");
      return;
    }

    if (static_cast<int>(keyframes_.size()) < min_keyframes_) {
      RCLCPP_WARN(this->get_logger(), "Too few keyframes to thin");
      return;
    }

    size_t original_count = keyframes_.size();
    std::vector<std::shared_ptr<PointCloud2DCPU>> thinned_keyframes;
    std::vector<gtsam::Pose2> thinned_poses;

    // Always keep first and last
    thinned_keyframes.push_back(keyframes_.front());
    thinned_poses.push_back(poses_.front());

    // Keep every Nth keyframe
    for (size_t i = thinning_factor_; i < keyframes_.size() - 1; i += thinning_factor_) {
      thinned_keyframes.push_back(keyframes_[i]);
      thinned_poses.push_back(poses_[i]);
    }

    thinned_keyframes.push_back(keyframes_.back());
    thinned_poses.push_back(poses_.back());

    keyframes_ = thinned_keyframes;
    poses_ = thinned_poses;

    RCLCPP_INFO(this->get_logger(), "Thinned keyframes: %zu -> %zu (removed %zu)",
                original_count, keyframes_.size(), original_count - keyframes_.size());
  }

  // Helper functions
  MapInfo loadMapInfo(const fs::path& path) {
    MapInfo info;
    std::ifstream file(path);
    if (!file.is_open()) return info;

    std::string line;
    while (std::getline(file, line)) {
      if (line.find("\"num_keyframes\":") != std::string::npos) {
        size_t pos = line.find(":");
        info.num_keyframes = std::stoi(line.substr(pos + 1));
      }
    }
    return info;
  }

  std::vector<PoseData> loadPoses(const fs::path& path) {
    std::vector<PoseData> poses;
    std::ifstream file(path);
    if (!file.is_open()) return poses;

    std::string line;
    while (std::getline(file, line)) {
      if (line.find("\"key\":") != std::string::npos) {
        PoseData pose;
        size_t key_pos = line.find("\"key\":");
        size_t x_pos = line.find("\"x\":");
        size_t y_pos = line.find("\"y\":");
        size_t theta_pos = line.find("\"theta\":");

        pose.key = std::stoi(line.substr(key_pos + 6));
        pose.x = std::stod(line.substr(x_pos + 4));
        pose.y = std::stod(line.substr(y_pos + 4));
        pose.theta = std::stod(line.substr(theta_pos + 8));
        poses.push_back(pose);
      }
    }
    return poses;
  }

  std::shared_ptr<PointCloud2DCPU> loadPointCloudPCD(const fs::path& path) {
    auto cloud = std::make_shared<PointCloud2DCPU>();
    std::ifstream file(path);
    if (!file.is_open()) return cloud;

    std::string line;
    bool data_section = false;

    while (std::getline(file, line)) {
      if (line.find("DATA ascii") != std::string::npos) {
        data_section = true;
        continue;
      }
      if (data_section && !line.empty()) {
        std::istringstream iss(line);
        double x, y;
        if (iss >> x >> y) {
          cloud->add_point(Eigen::Vector2d(x, y));
        }
      }
    }
    return cloud;
  }

  void savePointCloudPCD(const std::shared_ptr<PointCloud2DCPU>& cloud, const fs::path& path) {
    std::ofstream file(path);
    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y\n";
    file << "SIZE 4 4\n";
    file << "TYPE F F\n";
    file << "COUNT 1 1\n";
    file << "WIDTH " << cloud->size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << cloud->size() << "\n";
    file << "DATA ascii\n";

    for (const auto& point : cloud->points) {
      file << point.x() << " " << point.y() << "\n";
    }
  }

  // ROS2 services
  rclcpp::Service<gtsam_points_2d_slam::srv::LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<gtsam_points_2d_slam::srv::SaveMap>::SharedPtr save_map_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr merge_keyframes_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr thin_keyframes_service_;

  // Parameters
  double merge_distance_threshold_;
  double merge_angle_threshold_;
  int thinning_factor_;
  int min_keyframes_;

  // Map data
  std::vector<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::vector<gtsam::Pose2> poses_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyframeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
