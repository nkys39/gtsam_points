// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/types/laser_scan.hpp>

#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>

namespace gtsam_points {

LaserScan::LaserScan()
  : PointCloud2DCPU(),
    angle_min(0.0),
    angle_max(0.0),
    angle_increment(0.0),
    time_increment(0.0),
    scan_time(0.0),
    range_min(0.0),
    range_max(std::numeric_limits<double>::max()) {}

LaserScan::LaserScan(
  const double* ranges,
  int num_ranges,
  double angle_min,
  double angle_max,
  double angle_increment)
  : LaserScan() {
  set_scan(ranges, num_ranges, angle_min, angle_max, angle_increment);
}

LaserScan::LaserScan(
  const std::vector<double>& ranges,
  double angle_min,
  double angle_max,
  double angle_increment)
  : LaserScan() {
  set_scan(ranges.data(), ranges.size(), angle_min, angle_max, angle_increment);
}

LaserScan::LaserScan(
  const double* ranges,
  const double* angles,
  int num_points)
  : LaserScan() {
  set_scan(ranges, angles, num_points);
}

LaserScan::LaserScan(
  const std::vector<double>& ranges,
  const std::vector<double>& angles)
  : LaserScan() {
  if (ranges.size() != angles.size()) {
    std::cerr << "error: ranges and angles size mismatch" << std::endl;
    std::cerr << "     : ranges.size()=" << ranges.size() << " angles.size()=" << angles.size() << std::endl;
    return;
  }
  set_scan(ranges.data(), angles.data(), ranges.size());
}

void LaserScan::set_scan(
  const double* ranges,
  int num_ranges,
  double angle_min,
  double angle_max,
  double angle_increment) {

  this->num_points = num_ranges;
  this->angle_min = angle_min;
  this->angle_max = angle_max;
  this->angle_increment = angle_increment;

  // Store ranges
  ranges_storage = std::make_shared<std::vector<double>>(ranges, ranges + num_ranges);

  // Compute angles
  angles_storage = std::make_shared<std::vector<double>>(num_ranges);
  for (int i = 0; i < num_ranges; i++) {
    (*angles_storage)[i] = angle_min + i * angle_increment;
  }

  // Convert to Cartesian coordinates
  compute_cartesian();
}

void LaserScan::set_scan(
  const double* ranges,
  const double* angles,
  int num_points) {

  this->num_points = num_points;

  // Store ranges and angles
  ranges_storage = std::make_shared<std::vector<double>>(ranges, ranges + num_points);
  angles_storage = std::make_shared<std::vector<double>>(angles, angles + num_points);

  // Compute scan parameters from angles
  if (num_points > 1) {
    this->angle_min = (*angles_storage)[0];
    this->angle_max = (*angles_storage)[num_points - 1];
    this->angle_increment = (angle_max - angle_min) / (num_points - 1);
  } else if (num_points == 1) {
    this->angle_min = (*angles_storage)[0];
    this->angle_max = (*angles_storage)[0];
    this->angle_increment = 0.0;
  }

  // Convert to Cartesian coordinates
  compute_cartesian();
}

void LaserScan::compute_cartesian() {
  if (!ranges_storage || !angles_storage) {
    std::cerr << "error: ranges or angles not set" << std::endl;
    return;
  }

  const int num = ranges_storage->size();
  if (num != static_cast<int>(angles_storage->size())) {
    std::cerr << "error: ranges and angles size mismatch" << std::endl;
    return;
  }

  // Allocate points storage
  points_storage = std::make_shared<std::vector<Eigen::Vector3d>>(num);

  // Convert polar to Cartesian: (r, θ) -> (x, y, 1)
  // x = r * cos(θ)
  // y = r * sin(θ)
  for (int i = 0; i < num; i++) {
    const double r = (*ranges_storage)[i];
    const double theta = (*angles_storage)[i];

    (*points_storage)[i] << r * std::cos(theta), r * std::sin(theta), 1.0;
  }

  // Update base class pointer
  this->points = points_storage->data();
}

void LaserScan::filter_range(double range_min, double range_max) {
  if (!ranges_storage || !angles_storage || !points_storage) {
    std::cerr << "warning: scan data not initialized" << std::endl;
    return;
  }

  std::vector<double> filtered_ranges;
  std::vector<double> filtered_angles;
  std::vector<Eigen::Vector3d> filtered_points;

  filtered_ranges.reserve(ranges_storage->size());
  filtered_angles.reserve(angles_storage->size());
  filtered_points.reserve(points_storage->size());

  for (size_t i = 0; i < ranges_storage->size(); i++) {
    const double r = (*ranges_storage)[i];

    // Check for valid range
    if (std::isnan(r) || std::isinf(r)) {
      continue;
    }

    if (r < range_min || r > range_max) {
      continue;
    }

    filtered_ranges.push_back(r);
    filtered_angles.push_back((*angles_storage)[i]);
    filtered_points.push_back((*points_storage)[i]);
  }

  // Update storage
  this->num_points = filtered_ranges.size();

  ranges_storage = std::make_shared<std::vector<double>>(std::move(filtered_ranges));
  angles_storage = std::make_shared<std::vector<double>>(std::move(filtered_angles));
  points_storage = std::make_shared<std::vector<Eigen::Vector3d>>(std::move(filtered_points));

  // Update base class pointer
  this->points = points_storage->data();

  // Update range parameters
  this->range_min = range_min;
  this->range_max = range_max;
}

}  // namespace gtsam_points
