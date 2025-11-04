#include <memory>
#include <vector>
#include <deque>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp>
#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>

// Include service headers
#include "gtsam_points_2d_slam/srv/load_map.hpp"
#include "gtsam_points_2d_slam/srv/save_map.hpp"
#include "gtsam_points_2d_slam/srv/add_loop_closure.hpp"
#include "gtsam_points_2d_slam/srv/optimize_map.hpp"

using namespace gtsam_points;
namespace fs = std::filesystem;

// Structures for loading map data
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

class OfflineMapOptimizer : public rclcpp::Node
{
public:
  OfflineMapOptimizer()
  : Node("offline_map_optimizer"),
    map_loaded_(false)
  {
    // Declare parameters
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("use_vgicp", true);
    this->declare_parameter("voxel_resolution", 0.1);
    this->declare_parameter("max_correspondence_distance", 1.0);
    this->declare_parameter("loop_closure_threshold", 0.8);

    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    use_vgicp_ = this->get_parameter("use_vgicp").as_bool();
    voxel_resolution_ = this->get_parameter("voxel_resolution").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
    loop_closure_threshold_ = this->get_parameter("loop_closure_threshold").as_double();

    // Initialize gridmap for VGICP
    if (use_vgicp_) {
      gridmap_ = std::make_shared<IncrementalGridMap2D>(voxel_resolution_);
    }

    // Create publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("optimized_path", 10);
    loop_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("loop_closures", 10);

    // Create services
    load_map_service_ = this->create_service<gtsam_points_2d_slam::srv::LoadMap>(
      "load_map",
      std::bind(&OfflineMapOptimizer::loadMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    save_map_service_ = this->create_service<gtsam_points_2d_slam::srv::SaveMap>(
      "save_map",
      std::bind(&OfflineMapOptimizer::saveMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    add_loop_closure_service_ = this->create_service<gtsam_points_2d_slam::srv::AddLoopClosure>(
      "add_loop_closure",
      std::bind(&OfflineMapOptimizer::addLoopClosureCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    optimize_map_service_ = this->create_service<gtsam_points_2d_slam::srv::OptimizeMap>(
      "optimize_map",
      std::bind(&OfflineMapOptimizer::optimizeMapCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    // Create timer for publishing visualization
    viz_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&OfflineMapOptimizer::publishVisualization, this));

    RCLCPP_INFO(this->get_logger(), "Offline Map Optimizer initialized");
    RCLCPP_INFO(this->get_logger(), "Services available:");
    RCLCPP_INFO(this->get_logger(), "  - ~/load_map: Load a saved map");
    RCLCPP_INFO(this->get_logger(), "  - ~/add_loop_closure: Add manual loop closure");
    RCLCPP_INFO(this->get_logger(), "  - ~/optimize_map: Re-optimize the graph");
    RCLCPP_INFO(this->get_logger(), "  - ~/save_map: Save optimized map");
  }

private:
  // Load map callback
  void loadMapCallback(
    const std::shared_ptr<gtsam_points_2d_slam::srv::LoadMap::Request> request,
    std::shared_ptr<gtsam_points_2d_slam::srv::LoadMap::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Loading map from: %s/%s",
                request->directory_path.c_str(), request->map_name.c_str());

    try {
      fs::path map_dir = fs::path(request->directory_path) / request->map_name;
      fs::path keyframes_dir = map_dir / "keyframes";
      fs::path poses_path = map_dir / "poses.json";
      fs::path info_path = map_dir / "map_info.json";

      if (!fs::exists(map_dir)) {
        throw std::runtime_error("Map directory does not exist: " + map_dir.string());
      }

      // Load map info
      auto map_info = loadMapInfoJSON(info_path.string());
      RCLCPP_INFO(this->get_logger(), "Loading map: %s with %d keyframes",
                  map_info.map_name.c_str(), map_info.num_keyframes);

      // Load poses
      auto poses = loadPosesJSON(poses_path.string());
      if (poses.empty()) {
        throw std::runtime_error("No poses found in JSON file");
      }

      // Clear existing state
      keyframes_.clear();
      original_poses_.clear();
      optimized_poses_.clear();
      graph_ = gtsam::NonlinearFactorGraph();
      initial_estimates_.clear();
      loop_closures_.clear();
      if (gridmap_) {
        gridmap_ = std::make_shared<IncrementalGridMap2D>(voxel_resolution_);
      }

      // Load keyframes and build initial graph
      for (int i = 0; i < map_info.num_keyframes; ++i) {
        // Load PCD file
        std::ostringstream filename;
        filename << "keyframe_" << std::setw(6) << std::setfill('0') << i << ".pcd";
        fs::path pcd_path = keyframes_dir / filename.str();

        auto cloud = loadPointCloudPCD(pcd_path.string());
        keyframes_.push_back(cloud);

        // Get pose
        if (i >= static_cast<int>(poses.size())) {
          throw std::runtime_error("Pose index out of range");
        }
        gtsam::Pose2 pose(poses[i].x, poses[i].y, poses[i].theta);
        original_poses_.push_back(pose);
        optimized_poses_.push_back(pose);

        // Add to graph
        gtsam::Symbol key('x', i);

        if (i == 0) {
          // Add prior factor for first pose
          auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(3) << 0.01, 0.01, 0.01).finished());
          graph_.add(gtsam::PriorFactor<gtsam::Pose2>(key, pose, noise_model));
        } else {
          // Add GICP/VGICP factor between consecutive keyframes
          if (use_vgicp_) {
            gridmap_->insert(*keyframes_[i-1]);
            auto vgicp_factor = gtsam::make_shared<IntegratedVGICPFactor2D>(
              gtsam::Symbol('x', i-1), gtsam::Symbol('x', i),
              keyframes_[i-1], keyframes_[i], gridmap_);
            vgicp_factor->set_max_correspondence_distance(max_correspondence_distance_);
            graph_.add(vgicp_factor);
          } else {
            auto gicp_factor = gtsam::make_shared<IntegratedGICPFactor2D>(
              gtsam::Symbol('x', i-1), gtsam::Symbol('x', i),
              keyframes_[i-1], keyframes_[i]);
            gicp_factor->set_max_correspondence_distance(max_correspondence_distance_);
            graph_.add(gicp_factor);
          }
        }

        // Add initial estimate
        initial_estimates_.insert(key, pose);
      }

      num_keyframes_ = map_info.num_keyframes;
      map_loaded_ = true;

      response->success = true;
      response->message = "Map loaded successfully";
      response->num_keyframes = num_keyframes_;

      RCLCPP_INFO(this->get_logger(), "Map loaded: %d keyframes, ready for optimization",
                  num_keyframes_);
      RCLCPP_INFO(this->get_logger(), "Use ~/add_loop_closure to add manual loop closures");
      RCLCPP_INFO(this->get_logger(), "Use ~/optimize_map to re-optimize the graph");
    }
    catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to load map: ") + e.what();
      response->num_keyframes = 0;

      RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s", e.what());
    }
  }

  // Add loop closure callback
  void addLoopClosureCallback(
    const std::shared_ptr<gtsam_points_2d_slam::srv::AddLoopClosure::Request> request,
    std::shared_ptr<gtsam_points_2d_slam::srv::AddLoopClosure::Response> response)
  {
    if (!map_loaded_) {
      response->success = false;
      response->message = "No map loaded. Load a map first using ~/load_map";
      response->match_score = 0.0;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Adding loop closure between keyframes %d and %d",
                request->from_key, request->to_key);

    try {
      // Validate keyframe indices
      if (request->from_key < 0 || request->from_key >= num_keyframes_ ||
          request->to_key < 0 || request->to_key >= num_keyframes_) {
        throw std::runtime_error("Invalid keyframe indices");
      }

      if (request->from_key == request->to_key) {
        throw std::runtime_error("Cannot add loop closure to the same keyframe");
      }

      gtsam::Pose2 relative_pose;
      double match_score = 0.0;

      if (request->auto_match) {
        // Use scan matching to compute relative pose
        RCLCPP_INFO(this->get_logger(), "Computing relative pose using scan matching...");

        // Get initial guess from optimized poses
        gtsam::Pose2 pose_from = optimized_poses_[request->from_key];
        gtsam::Pose2 pose_to = optimized_poses_[request->to_key];
        gtsam::Pose2 initial_guess = pose_from.between(pose_to);

        // Perform scan matching (GICP)
        auto factor = gtsam::make_shared<IntegratedGICPFactor2D>(
          gtsam::Symbol('x', 0),  // Dummy keys for scan matching
          keyframes_[request->from_key],
          keyframes_[request->to_key]);
        factor->set_max_correspondence_distance(max_correspondence_distance_);

        // Optimize to find relative pose
        gtsam::NonlinearFactorGraph temp_graph;
        temp_graph.add(factor);
        gtsam::Values temp_values;
        temp_values.insert(gtsam::Symbol('x', 0), initial_guess);

        gtsam::LevenbergMarquardtOptimizer optimizer(temp_graph, temp_values);
        gtsam::Values result = optimizer.optimize();
        relative_pose = result.at<gtsam::Pose2>(gtsam::Symbol('x', 0));

        // Compute match score (inverse of error)
        double error = temp_graph.error(result);
        match_score = 1.0 / (1.0 + error);

        RCLCPP_INFO(this->get_logger(), "Scan matching result: [%.3f, %.3f, %.3f°], score: %.3f",
                    relative_pose.x(), relative_pose.y(),
                    relative_pose.theta() * 180.0 / M_PI, match_score);

        if (match_score < loop_closure_threshold_) {
          RCLCPP_WARN(this->get_logger(),
                      "Match score %.3f below threshold %.3f - loop closure may be unreliable",
                      match_score, loop_closure_threshold_);
        }
      } else {
        // Use provided relative pose
        relative_pose = gtsam::Pose2(request->x, request->y, request->theta);
        match_score = 1.0;
        RCLCPP_INFO(this->get_logger(), "Using provided relative pose: [%.3f, %.3f, %.3f°]",
                    relative_pose.x(), relative_pose.y(),
                    relative_pose.theta() * 180.0 / M_PI);
      }

      // Add loop closure factor to graph
      auto noise_model = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(3) << 0.1, 0.1, 0.1).finished());
      auto between_factor = gtsam::make_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
        gtsam::Symbol('x', request->from_key),
        gtsam::Symbol('x', request->to_key),
        relative_pose,
        noise_model);
      graph_.add(between_factor);

      // Store loop closure for visualization
      LoopClosure lc;
      lc.from_key = request->from_key;
      lc.to_key = request->to_key;
      lc.relative_pose = relative_pose;
      lc.match_score = match_score;
      loop_closures_.push_back(lc);

      response->success = true;
      response->message = "Loop closure added successfully";
      response->match_score = match_score;

      RCLCPP_INFO(this->get_logger(), "Loop closure added. Total: %zu. Use ~/optimize_map to optimize.",
                  loop_closures_.size());
    }
    catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to add loop closure: ") + e.what();
      response->match_score = 0.0;

      RCLCPP_ERROR(this->get_logger(), "Failed to add loop closure: %s", e.what());
    }
  }

  // Optimize map callback
  void optimizeMapCallback(
    const std::shared_ptr<gtsam_points_2d_slam::srv::OptimizeMap::Request> /*request*/,
    std::shared_ptr<gtsam_points_2d_slam::srv::OptimizeMap::Response> response)
  {
    if (!map_loaded_) {
      response->success = false;
      response->message = "No map loaded. Load a map first using ~/load_map";
      response->num_iterations = 0;
      response->initial_error = 0.0;
      response->final_error = 0.0;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Optimizing graph with %zu factors...",
                graph_.size());

    try {
      // Compute initial error
      double initial_error = graph_.error(initial_estimates_);

      // Optimize using Levenberg-Marquardt
      gtsam::LevenbergMarquardtParams params;
      params.setVerbosity("TERMINATION");
      gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_estimates_, params);
      gtsam::Values result = optimizer.optimize();

      // Compute final error
      double final_error = graph_.error(result);

      // Update optimized poses
      for (int i = 0; i < num_keyframes_; ++i) {
        optimized_poses_[i] = result.at<gtsam::Pose2>(gtsam::Symbol('x', i));
      }

      // Update initial estimates for next optimization
      initial_estimates_ = result;

      response->success = true;
      response->message = "Graph optimized successfully";
      response->num_iterations = optimizer.iterations();
      response->initial_error = initial_error;
      response->final_error = final_error;

      double improvement = (1.0 - final_error / initial_error) * 100.0;
      RCLCPP_INFO(this->get_logger(),
                  "Optimization complete: %d iterations, error %.6f -> %.6f (%.1f%% improvement)",
                  optimizer.iterations(), initial_error, final_error, improvement);
      RCLCPP_INFO(this->get_logger(), "Use ~/save_map to save the optimized map");
    }
    catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to optimize graph: ") + e.what();
      response->num_iterations = 0;
      response->initial_error = 0.0;
      response->final_error = 0.0;

      RCLCPP_ERROR(this->get_logger(), "Failed to optimize graph: %s", e.what());
    }
  }

  // Save map callback
  void saveMapCallback(
    const std::shared_ptr<gtsam_points_2d_slam::srv::SaveMap::Request> request,
    std::shared_ptr<gtsam_points_2d_slam::srv::SaveMap::Response> response)
  {
    if (!map_loaded_) {
      response->success = false;
      response->message = "No map loaded. Load a map first using ~/load_map";
      response->num_keyframes = 0;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Saving optimized map to: %s/%s",
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

      // Save optimized poses as JSON
      fs::path poses_path = map_dir / "poses.json";
      savePosesJSON(poses_path.string());

      // Save map info
      fs::path info_path = map_dir / "map_info.json";
      saveMapInfoJSON(info_path.string(), request->map_name);

      response->success = true;
      response->message = "Optimized map saved successfully";
      response->num_keyframes = keyframes_.size();

      RCLCPP_INFO(this->get_logger(), "Optimized map saved: %zu keyframes", keyframes_.size());
    }
    catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Failed to save map: ") + e.what();
      response->num_keyframes = 0;

      RCLCPP_ERROR(this->get_logger(), "Failed to save map: %s", e.what());
    }
  }

  // Publish visualization
  void publishVisualization()
  {
    if (!map_loaded_) return;

    // Publish optimized path
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->now();
    path_msg.header.frame_id = map_frame_;

    for (const auto& pose : optimized_poses_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path_msg.header;
      pose_stamped.pose.position.x = pose.x();
      pose_stamped.pose.position.y = pose.y();
      pose_stamped.pose.position.z = 0.0;

      double yaw = pose.theta();
      pose_stamped.pose.orientation.w = std::cos(yaw / 2.0);
      pose_stamped.pose.orientation.x = 0.0;
      pose_stamped.pose.orientation.y = 0.0;
      pose_stamped.pose.orientation.z = std::sin(yaw / 2.0);

      path_msg.poses.push_back(pose_stamped);
    }

    path_pub_->publish(path_msg);

    // Publish loop closure markers
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < loop_closures_.size(); ++i) {
      const auto& lc = loop_closures_[i];

      visualization_msgs::msg::Marker marker;
      marker.header.stamp = this->now();
      marker.header.frame_id = map_frame_;
      marker.ns = "loop_closures";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.scale.x = 0.05;  // Line width
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.8;

      geometry_msgs::msg::Point p1, p2;
      p1.x = optimized_poses_[lc.from_key].x();
      p1.y = optimized_poses_[lc.from_key].y();
      p1.z = 0.2;

      p2.x = optimized_poses_[lc.to_key].x();
      p2.y = optimized_poses_[lc.to_key].y();
      p2.z = 0.2;

      marker.points.push_back(p1);
      marker.points.push_back(p2);

      marker_array.markers.push_back(marker);
    }

    loop_marker_pub_->publish(marker_array);
  }

  // Helper: Load point cloud from PCD
  std::shared_ptr<PointCloud2DCPU> loadPointCloudPCD(const std::string& filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file: " + filename);
    }

    auto cloud = std::make_shared<PointCloud2DCPU>();
    std::string line;
    bool data_section = false;
    size_t num_points = 0;

    // Parse header
    while (std::getline(file, line)) {
      if (line.find("POINTS") == 0) {
        std::istringstream iss(line);
        std::string keyword;
        iss >> keyword >> num_points;
      }
      else if (line.find("DATA ascii") == 0) {
        data_section = true;
        break;
      }
    }

    if (!data_section) {
      throw std::runtime_error("Invalid PCD file format: " + filename);
    }

    // Read point data
    cloud->points.reserve(num_points);
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      double x, y;
      if (iss >> x >> y) {
        cloud->points.push_back(Eigen::Vector3d(x, y, 1.0));
      }
    }

    file.close();
    return cloud;
  }

  // Helper: Load poses from JSON
  std::vector<PoseData> loadPosesJSON(const std::string& filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file: " + filename);
    }

    std::vector<PoseData> poses;
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    // Simple JSON parsing for poses array
    size_t poses_start = content.find("\"poses\"");
    if (poses_start == std::string::npos) {
      throw std::runtime_error("No 'poses' field found in JSON");
    }

    size_t array_start = content.find("[", poses_start);
    size_t array_end = content.find("]", array_start);

    if (array_start == std::string::npos || array_end == std::string::npos) {
      throw std::runtime_error("Invalid poses array in JSON");
    }

    std::string poses_array = content.substr(array_start + 1, array_end - array_start - 1);

    // Parse each pose object
    size_t pos = 0;
    while (true) {
      size_t obj_start = poses_array.find("{", pos);
      if (obj_start == std::string::npos) break;

      size_t obj_end = poses_array.find("}", obj_start);
      if (obj_end == std::string::npos) break;

      std::string pose_obj = poses_array.substr(obj_start, obj_end - obj_start + 1);

      PoseData pose;

      // Parse key
      size_t key_pos = pose_obj.find("\"key\"");
      if (key_pos != std::string::npos) {
        size_t colon = pose_obj.find(":", key_pos);
        size_t comma = pose_obj.find(",", colon);
        std::string key_str = pose_obj.substr(colon + 1, comma - colon - 1);
        pose.key = std::stoi(key_str);
      }

      // Parse x
      size_t x_pos = pose_obj.find("\"x\"");
      if (x_pos != std::string::npos) {
        size_t colon = pose_obj.find(":", x_pos);
        size_t comma = pose_obj.find(",", colon);
        std::string x_str = pose_obj.substr(colon + 1, comma - colon - 1);
        pose.x = std::stod(x_str);
      }

      // Parse y
      size_t y_pos = pose_obj.find("\"y\"");
      if (y_pos != std::string::npos) {
        size_t colon = pose_obj.find(":", y_pos);
        size_t comma = pose_obj.find(",", colon);
        std::string y_str = pose_obj.substr(colon + 1, comma - colon - 1);
        pose.y = std::stod(y_str);
      }

      // Parse theta
      size_t theta_pos = pose_obj.find("\"theta\"");
      if (theta_pos != std::string::npos) {
        size_t colon = pose_obj.find(":", theta_pos);
        size_t end = pose_obj.find("\n", colon);
        if (end == std::string::npos) end = pose_obj.length();
        std::string theta_str = pose_obj.substr(colon + 1, end - colon - 1);
        pose.theta = std::stod(theta_str);
      }

      poses.push_back(pose);
      pos = obj_end + 1;
    }

    return poses;
  }

  // Helper: Load map info from JSON
  MapInfo loadMapInfoJSON(const std::string& filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();

    MapInfo info;

    // Parse map_name
    size_t name_pos = content.find("\"map_name\"");
    if (name_pos != std::string::npos) {
      size_t colon = content.find(":", name_pos);
      size_t quote1 = content.find("\"", colon + 1);
      size_t quote2 = content.find("\"", quote1 + 1);
      info.map_name = content.substr(quote1 + 1, quote2 - quote1 - 1);
    }

    // Parse num_keyframes
    size_t num_pos = content.find("\"num_keyframes\"");
    if (num_pos != std::string::npos) {
      size_t colon = content.find(":", num_pos);
      size_t comma = content.find(",", colon);
      std::string num_str = content.substr(colon + 1, comma - colon - 1);
      info.num_keyframes = std::stoi(num_str);
    }

    // Parse created_at
    size_t created_pos = content.find("\"created_at\"");
    if (created_pos != std::string::npos) {
      size_t colon = content.find(":", created_pos);
      size_t quote1 = content.find("\"", colon + 1);
      size_t quote2 = content.find("\"", quote1 + 1);
      info.created_at = content.substr(quote1 + 1, quote2 - quote1 - 1);
    }

    // Parse use_vgicp
    size_t vgicp_pos = content.find("\"use_vgicp\"");
    if (vgicp_pos != std::string::npos) {
      size_t colon = content.find(":", vgicp_pos);
      size_t end = content.find("\n", colon);
      std::string vgicp_str = content.substr(colon + 1, end - colon - 1);
      info.use_vgicp = (vgicp_str.find("true") != std::string::npos);
    }

    return info;
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

    file << "{\n";
    file << "  \"poses\": [\n";

    for (size_t i = 0; i < optimized_poses_.size(); ++i) {
      const auto& pose = optimized_poses_[i];

      file << "    {\n";
      file << "      \"key\": " << i << ",\n";
      file << "      \"x\": " << pose.x() << ",\n";
      file << "      \"y\": " << pose.y() << ",\n";
      file << "      \"theta\": " << pose.theta() << "\n";
      file << "    }";

      if (i < optimized_poses_.size() - 1) {
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
    file << "  \"use_vgicp\": " << (use_vgicp_ ? "true" : "false") << ",\n";
    file << "  \"num_loop_closures\": " << loop_closures_.size() << "\n";
    file << "}\n";

    file.close();
  }

  // Loop closure structure
  struct LoopClosure {
    int from_key;
    int to_key;
    gtsam::Pose2 relative_pose;
    double match_score;
  };

private:
  // ROS parameters
  std::string map_frame_;
  bool use_vgicp_;
  double voxel_resolution_;
  double max_correspondence_distance_;
  double loop_closure_threshold_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_marker_pub_;
  rclcpp::Service<gtsam_points_2d_slam::srv::LoadMap>::SharedPtr load_map_service_;
  rclcpp::Service<gtsam_points_2d_slam::srv::SaveMap>::SharedPtr save_map_service_;
  rclcpp::Service<gtsam_points_2d_slam::srv::AddLoopClosure>::SharedPtr add_loop_closure_service_;
  rclcpp::Service<gtsam_points_2d_slam::srv::OptimizeMap>::SharedPtr optimize_map_service_;
  rclcpp::TimerBase::SharedPtr viz_timer_;

  // GTSAM
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
  std::shared_ptr<IncrementalGridMap2D> gridmap_;

  // Map data
  bool map_loaded_;
  int num_keyframes_;
  std::vector<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::vector<gtsam::Pose2> original_poses_;
  std::vector<gtsam::Pose2> optimized_poses_;
  std::vector<LoopClosure> loop_closures_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OfflineMapOptimizer>());
  rclcpp::shutdown();
  return 0;
}
