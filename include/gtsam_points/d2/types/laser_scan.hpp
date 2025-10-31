// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>

namespace gtsam_points {

/**
 * @brief 2D Laser scan data with polar coordinates
 *
 * This class represents data from 2D laser range finders (LiDAR/LRF) which
 * output measurements in polar coordinates (range, angle). It automatically
 * converts to Cartesian coordinates and provides compatibility with ROS
 * sensor_msgs/LaserScan format.
 *
 * Coordinate system:
 * - Angle 0 is along the positive x-axis
 * - Angles increase counter-clockwise
 * - Range is the distance from the sensor origin
 */
struct LaserScan : public PointCloud2DCPU {
public:
  using Ptr = std::shared_ptr<LaserScan>;
  using ConstPtr = std::shared_ptr<const LaserScan>;

  /**
   * @brief Default constructor
   */
  LaserScan();

  /**
   * @brief Constructor from polar coordinates
   * @param ranges        Array of range measurements [m]
   * @param num_ranges    Number of range measurements
   * @param angle_min     Start angle [rad]
   * @param angle_max     End angle [rad]
   * @param angle_increment  Angular distance between measurements [rad]
   */
  LaserScan(
    const double* ranges,
    int num_ranges,
    double angle_min,
    double angle_max,
    double angle_increment);

  /**
   * @brief Constructor from polar coordinates (vector version)
   * @param ranges        Vector of range measurements [m]
   * @param angle_min     Start angle [rad]
   * @param angle_max     End angle [rad]
   * @param angle_increment  Angular distance between measurements [rad]
   */
  LaserScan(
    const std::vector<double>& ranges,
    double angle_min,
    double angle_max,
    double angle_increment);

  /**
   * @brief Constructor with explicit angles and ranges
   * @param ranges        Array of range measurements [m]
   * @param angles        Array of bearing angles [rad]
   * @param num_points    Number of measurements
   */
  LaserScan(
    const double* ranges,
    const double* angles,
    int num_points);

  /**
   * @brief Constructor with explicit angles and ranges (vector version)
   * @param ranges        Vector of range measurements [m]
   * @param angles        Vector of bearing angles [rad]
   */
  LaserScan(
    const std::vector<double>& ranges,
    const std::vector<double>& angles);

  /**
   * @brief Set scan parameters and compute Cartesian coordinates
   * @param ranges        Array of range measurements [m]
   * @param num_ranges    Number of range measurements
   * @param angle_min     Start angle [rad]
   * @param angle_max     End angle [rad]
   * @param angle_increment  Angular distance between measurements [rad]
   */
  void set_scan(
    const double* ranges,
    int num_ranges,
    double angle_min,
    double angle_max,
    double angle_increment);

  /**
   * @brief Set scan with explicit angles
   * @param ranges        Array of range measurements [m]
   * @param angles        Array of bearing angles [rad]
   * @param num_points    Number of measurements
   */
  void set_scan(
    const double* ranges,
    const double* angles,
    int num_points);

  /**
   * @brief Convert polar coordinates to Cartesian (x, y, 1)
   *
   * This method is called automatically by constructors/set_scan.
   * Can be called manually if polar data is modified directly.
   */
  void compute_cartesian();

  /**
   * @brief Filter out invalid measurements
   * @param range_min  Minimum valid range [m]
   * @param range_max  Maximum valid range [m]
   *
   * Points outside [range_min, range_max] are removed.
   * Points with NaN or Inf ranges are removed.
   */
  void filter_range(double range_min, double range_max);

  /**
   * @brief Get range at index
   * @param i  Point index
   * @return Range value [m]
   */
  double range(size_t i) const {
    return ranges_storage ? (*ranges_storage)[i] : 0.0;
  }

  /**
   * @brief Get angle at index
   * @param i  Point index
   * @return Angle value [rad]
   */
  double angle(size_t i) const {
    return angles_storage ? (*angles_storage)[i] : 0.0;
  }

public:
  // Polar coordinate storage
  std::shared_ptr<std::vector<double>> ranges_storage;  ///< Range measurements [m]
  std::shared_ptr<std::vector<double>> angles_storage;  ///< Bearing angles [rad]

  // Scan parameters (compatible with ROS sensor_msgs/LaserScan)
  double angle_min;        ///< Start angle of the scan [rad]
  double angle_max;        ///< End angle of the scan [rad]
  double angle_increment;  ///< Angular distance between measurements [rad]
  double time_increment;   ///< Time between measurements [sec] (0 if unknown)
  double scan_time;        ///< Time between scans [sec] (0 if unknown)
  double range_min;        ///< Minimum range value [m]
  double range_max;        ///< Maximum range value [m]
};

}  // namespace gtsam_points
