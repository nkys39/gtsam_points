#include <memory>
#include <deque>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/reintegrated_imu_factor_2d.hpp>
#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>

using namespace gtsam_points;

/**
 * @brief SLAM with IMU Integration using ReintegratedIMUFactor2D
 *
 * This node integrates IMU measurements (acceleration and gyroscope) with
 * LiDAR scan matching using gtsam_points 2D IMU preintegration.
 */
class SlamWithIMUNode : public rclcpp::Node
{
public:
  SlamWithIMUNode()
  : Node("gtsam_slam_with_imu"),
    key_counter_(0),
    initialized_(false),
    has_imu_(false)
  {
    // Declare parameters
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_footprint");
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("imu_topic", "imu");
    this->declare_parameter("use_vgicp", true);
    this->declare_parameter("keyframe_distance", 0.5);
    this->declare_parameter("keyframe_angle", 0.3);
    this->declare_parameter("voxel_resolution", 0.1);
    this->declare_parameter("max_correspondence_distance", 1.0);

    // IMU parameters
    this->declare_parameter("use_imu", true);
    this->declare_parameter("imu_acc_noise", 0.1);
    this->declare_parameter("imu_gyro_noise", 0.01);
    this->declare_parameter("imu_acc_bias_noise", 0.001);
    this->declare_parameter("imu_gyro_bias_noise", 0.0001);
    this->declare_parameter("gravity", 9.81);

    // Get parameters
    map_frame_ = this->get_parameter("map_frame").as_string();
    odom_frame_ = this->get_parameter("odom_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    use_vgicp_ = this->get_parameter("use_vgicp").as_bool();
    keyframe_distance_ = this->get_parameter("keyframe_distance").as_double();
    keyframe_angle_ = this->get_parameter("keyframe_angle").as_double();
    voxel_resolution_ = this->get_parameter("voxel_resolution").as_double();
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();

    use_imu_ = this->get_parameter("use_imu").as_bool();
    imu_acc_noise_ = this->get_parameter("imu_acc_noise").as_double();
    imu_gyro_noise_ = this->get_parameter("imu_gyro_noise").as_double();
    imu_acc_bias_noise_ = this->get_parameter("imu_acc_bias_noise").as_double();
    imu_gyro_bias_noise_ = this->get_parameter("imu_gyro_bias_noise").as_double();
    gravity_ = this->get_parameter("gravity").as_double();

    // Initialize ISAM2
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.1;
    isam2_params.relinearizeSkip = 1;
    isam2_ = std::make_shared<gtsam::ISAM2>(isam2_params);

    // Initialize gridmap for VGICP
    if (use_vgicp_) {
      gridmap_ = std::make_shared<IncrementalGridMap2D>(voxel_resolution_);
    }

    // Initialize IMU preintegration parameters
    if (use_imu_) {
      initializeIMUPreintegration();
    }

    // Create subscribers
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, 10,
      std::bind(&SlamWithIMUNode::scanCallback, this, std::placeholders::_1));

    if (use_imu_) {
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 100,
        std::bind(&SlamWithIMUNode::imuCallback, this, std::placeholders::_1));
    }

    // Create publishers
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("slam_odom", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("slam_path", 10);

    // Create TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Initialize current pose and velocity
    current_pose_ = gtsam::Pose2(0.0, 0.0, 0.0);
    current_velocity_ = Eigen::Vector2d::Zero();
    last_keyframe_pose_ = current_pose_;

    RCLCPP_INFO(this->get_logger(), "SLAM with IMU Integration Node initialized");
    RCLCPP_INFO(this->get_logger(), "Using %s for scan matching", use_vgicp_ ? "VGICP" : "GICP");
    RCLCPP_INFO(this->get_logger(), "IMU integration: %s", use_imu_ ? "Enabled" : "Disabled");
  }

private:
  void initializeIMUPreintegration()
  {
    // Create IMU preintegration parameters for 2D SLAM
    auto imu_params = std::make_shared<ReintegratedIMUParams2D>();

    // Set noise parameters
    imu_params->accelerometer_noise = imu_acc_noise_;
    imu_params->gyroscope_noise = imu_gyro_noise_;
    imu_params->accelerometer_bias_noise = imu_acc_bias_noise_;
    imu_params->gyroscope_bias_noise = imu_gyro_bias_noise_;
    imu_params->gravity = gravity_;

    imu_params_ = imu_params;

    // Initialize bias
    imu_bias_ = Eigen::Vector3d::Zero();  // [acc_x_bias, acc_y_bias, gyro_z_bias]

    RCLCPP_INFO(this->get_logger(), "IMU preintegration initialized");
    RCLCPP_INFO(this->get_logger(), "  Acc noise: %.3f, Gyro noise: %.4f", imu_acc_noise_, imu_gyro_noise_);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!initialized_) {
      return;
    }

    // Store IMU measurement
    IMUMeasurement2D meas;
    meas.timestamp = rclcpp::Time(msg->header.stamp).seconds();
    meas.linear_acceleration = Eigen::Vector2d(
      msg->linear_acceleration.x,
      msg->linear_acceleration.y);
    meas.angular_velocity = msg->angular_velocity.z;

    imu_measurements_.push_back(meas);
    has_imu_ = true;

    // Keep only recent measurements (last 10 seconds)
    const double current_time = rclcpp::Time(msg->header.stamp).seconds();
    while (!imu_measurements_.empty() &&
           current_time - imu_measurements_.front().timestamp > 10.0) {
      imu_measurements_.pop_front();
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Convert LaserScan to PointCloud2D
    auto scan = convertLaserScan(msg);
    if (!scan || scan->size() < 10) {
      RCLCPP_WARN(this->get_logger(), "Scan has too few points");
      return;
    }

    const double scan_time = rclcpp::Time(msg->header.stamp).seconds();

    if (!initialized_) {
      initializeFirstScan(scan, msg->header.stamp, scan_time);
      return;
    }

    // Check if we need a new keyframe
    if (shouldAddKeyframe()) {
      addKeyframe(scan, msg->header.stamp, scan_time);
    }

    // Publish odometry and TF
    publishOdometry(msg->header.stamp);
    publishTransform(msg->header.stamp);
    publishPath(msg->header.stamp);
  }

  std::shared_ptr<PointCloud2DCPU> convertLaserScan(
    const sensor_msgs::msg::LaserScan::SharedPtr& msg)
  {
    auto cloud = std::make_shared<PointCloud2DCPU>();

    std::vector<Eigen::Vector3d> points;
    points.reserve(msg->ranges.size());

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
    const rclcpp::Time& stamp,
    double scan_time)
  {
    // Add prior factor for pose
    auto prior_pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
      gtsam::Vector3(0.01, 0.01, 0.01));

    graph_.add(gtsam::PriorFactor<gtsam::Pose2>(
      gtsam::Symbol('x', key_counter_), current_pose_, prior_pose_noise));

    initial_estimates_.insert(gtsam::Symbol('x', key_counter_), current_pose_);

    // Add prior factor for velocity
    if (use_imu_) {
      auto prior_vel_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector2(0.01, 0.01));

      graph_.add(gtsam::PriorFactor<Eigen::Vector2d>(
        gtsam::Symbol('v', key_counter_), current_velocity_, prior_vel_noise));

      initial_estimates_.insert(gtsam::Symbol('v', key_counter_), current_velocity_);

      // Add prior factor for IMU bias
      auto prior_bias_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector3(0.001, 0.001, 0.0001));

      graph_.add(gtsam::PriorFactor<Eigen::Vector3d>(
        gtsam::Symbol('b', key_counter_), imu_bias_, prior_bias_noise));

      initial_estimates_.insert(gtsam::Symbol('b', key_counter_), imu_bias_);
    }

    // Update ISAM2
    isam2_->update(graph_, initial_estimates_);
    graph_.resize(0);
    initial_estimates_.clear();

    // Store keyframe
    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_);
    keyframe_times_.push_back(scan_time);
    last_keyframe_pose_ = current_pose_;
    last_keyframe_time_ = scan_time;

    // Add to gridmap
    if (use_vgicp_) {
      gridmap_->insert(scan);
    }

    key_counter_++;
    initialized_ = true;

    RCLCPP_INFO(this->get_logger(), "First scan initialized with IMU");
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
    const rclcpp::Time& stamp,
    double scan_time)
  {
    // Create factors
    if (!keyframes_.empty()) {
      // Add IMU factor if available
      if (use_imu_ && has_imu_) {
        auto imu_measurements = getIMUMeasurementsBetween(
          last_keyframe_time_, scan_time);

        if (!imu_measurements.empty()) {
          // Create IMU preintegration factor
          auto imu_factor = createIMUFactor(
            imu_measurements,
            gtsam::Symbol('x', key_counter_ - 1),
            gtsam::Symbol('v', key_counter_ - 1),
            gtsam::Symbol('b', key_counter_ - 1),
            gtsam::Symbol('x', key_counter_),
            gtsam::Symbol('v', key_counter_),
            gtsam::Symbol('b', key_counter_));

          if (imu_factor) {
            graph_.add(imu_factor);

            RCLCPP_DEBUG(this->get_logger(),
              "Added IMU factor with %zu measurements",
              imu_measurements.size());
          }
        }
      }

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

    // Add initial estimates
    initial_estimates_.insert(gtsam::Symbol('x', key_counter_), current_pose_);

    if (use_imu_) {
      initial_estimates_.insert(gtsam::Symbol('v', key_counter_), current_velocity_);
      initial_estimates_.insert(gtsam::Symbol('b', key_counter_), imu_bias_);
    }

    // Update ISAM2
    isam2_->update(graph_, initial_estimates_);

    // Get optimized result
    const auto result = isam2_->calculateEstimate();
    current_pose_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', key_counter_));

    if (use_imu_) {
      current_velocity_ = result.at<Eigen::Vector2d>(gtsam::Symbol('v', key_counter_));
      imu_bias_ = result.at<Eigen::Vector3d>(gtsam::Symbol('b', key_counter_));
    }

    // Update keyframe poses
    for (size_t i = 0; i < keyframe_poses_.size(); ++i) {
      keyframe_poses_[i] = result.at<gtsam::Pose2>(gtsam::Symbol('x', i));
    }

    graph_.resize(0);
    initial_estimates_.clear();

    // Store keyframe
    keyframes_.push_back(scan);
    keyframe_poses_.push_back(current_pose_);
    keyframe_times_.push_back(scan_time);
    last_keyframe_pose_ = current_pose_;
    last_keyframe_time_ = scan_time;

    key_counter_++;

    RCLCPP_INFO(this->get_logger(),
      "Keyframe %zu | Pose: (%.2f, %.2f, %.2f) | Vel: (%.2f, %.2f) | Bias: (%.3f, %.3f, %.4f)",
      key_counter_ - 1,
      current_pose_.x(), current_pose_.y(), current_pose_.theta(),
      current_velocity_.x(), current_velocity_.y(),
      imu_bias_.x(), imu_bias_.y(), imu_bias_.z());
  }

  struct IMUMeasurement2D {
    double timestamp;
    Eigen::Vector2d linear_acceleration;
    double angular_velocity;
  };

  std::vector<IMUMeasurement2D> getIMUMeasurementsBetween(
    double start_time, double end_time)
  {
    std::vector<IMUMeasurement2D> measurements;

    for (const auto& meas : imu_measurements_) {
      if (meas.timestamp >= start_time && meas.timestamp <= end_time) {
        measurements.push_back(meas);
      }
    }

    return measurements;
  }

  gtsam::NonlinearFactor::shared_ptr createIMUFactor(
    const std::vector<IMUMeasurement2D>& measurements,
    gtsam::Key pose_i, gtsam::Key vel_i, gtsam::Key bias_i,
    gtsam::Key pose_j, gtsam::Key vel_j, gtsam::Key bias_j)
  {
    if (measurements.empty()) {
      return nullptr;
    }

    // Create ReintegratedIMUFactor2D
    auto factor = gtsam::make_shared<ReintegratedIMUFactor2D>(
      pose_i, vel_i, bias_i,
      pose_j, vel_j, bias_j,
      imu_params_);

    // Integrate IMU measurements
    for (size_t i = 1; i < measurements.size(); ++i) {
      const double dt = measurements[i].timestamp - measurements[i-1].timestamp;
      if (dt > 0.0 && dt < 1.0) {  // Sanity check
        factor->integrateMeasurement(
          measurements[i].linear_acceleration,
          measurements[i].angular_velocity,
          dt);
      }
    }

    return factor;
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

    // Add velocity if IMU is used
    if (use_imu_) {
      odom_msg.twist.twist.linear.x = current_velocity_.x();
      odom_msg.twist.twist.linear.y = current_velocity_.y();
    }

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

  void publishPath(const rclcpp::Time& stamp)
  {
    path_.header.stamp = stamp;
    path_.header.frame_id = map_frame_;
    path_.poses.clear();

    for (const auto& pose : keyframe_poses_) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = path_.header;
      pose_stamped.pose.position.x = pose.x();
      pose_stamped.pose.position.y = pose.y();
      pose_stamped.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, pose.theta());
      pose_stamped.pose.orientation = tf2::toMsg(q);

      path_.poses.push_back(pose_stamped);
    }

    path_pub_->publish(path_);
  }

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_topic_;
  std::string imu_topic_;
  bool use_vgicp_;
  double keyframe_distance_;
  double keyframe_angle_;
  double voxel_resolution_;
  double max_correspondence_distance_;

  bool use_imu_;
  double imu_acc_noise_;
  double imu_gyro_noise_;
  double imu_acc_bias_noise_;
  double imu_gyro_bias_noise_;
  double gravity_;

  // SLAM state
  std::shared_ptr<gtsam::ISAM2> isam2_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimates_;
  std::shared_ptr<IncrementalGridMap2D> gridmap_;

  std::deque<std::shared_ptr<PointCloud2DCPU>> keyframes_;
  std::deque<gtsam::Pose2> keyframe_poses_;
  std::deque<double> keyframe_times_;

  gtsam::Pose2 current_pose_;
  gtsam::Pose2 last_keyframe_pose_;
  Eigen::Vector2d current_velocity_;
  Eigen::Vector3d imu_bias_;

  double last_keyframe_time_;
  nav_msgs::msg::Path path_;

  size_t key_counter_;
  bool initialized_;
  bool has_imu_;

  // IMU integration
  std::shared_ptr<ReintegratedIMUParams2D> imu_params_;
  std::deque<IMUMeasurement2D> imu_measurements_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SlamWithIMUNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
