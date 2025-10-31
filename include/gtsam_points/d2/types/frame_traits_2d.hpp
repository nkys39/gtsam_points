// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <gtsam_points/types/frame_traits.hpp>
#include <gtsam_points/d2/types/point_cloud_2d.hpp>
#include <gtsam_points/d2/types/laser_scan.hpp>

namespace gtsam_points {
namespace frame {

/**
 * @brief Frame traits specialization for PointCloud2D
 */
template <>
struct traits<PointCloud2D> {
  static int size(const PointCloud2D& frame) { return frame.size(); }

  static bool has_times(const PointCloud2D& frame) { return frame.has_times(); }
  static bool has_points(const PointCloud2D& frame) { return frame.has_points(); }
  static bool has_normals(const PointCloud2D& frame) { return frame.has_normals(); }
  static bool has_covs(const PointCloud2D& frame) { return frame.has_covs(); }
  static bool has_intensities(const PointCloud2D& frame) { return frame.has_intensities(); }

  static double time(const PointCloud2D& frame, size_t i) { return frame.times[i]; }

  // 2D points in homogeneous coordinates (x, y, 1)
  static const Eigen::Vector3d& point(const PointCloud2D& frame, size_t i) { return frame.points[i]; }

  // 2D normals (nx, ny, 0)
  static const Eigen::Vector3d& normal(const PointCloud2D& frame, size_t i) { return frame.normals[i]; }

  // 2D covariance (2x2 in upper-left corner of 3x3 matrix)
  static const Eigen::Matrix3d& cov(const PointCloud2D& frame, size_t i) { return frame.covs[i]; }

  static double intensity(const PointCloud2D& frame, size_t i) { return frame.intensities[i]; }

  static const Eigen::Vector3d* points_ptr(const PointCloud2D& frame) { return frame.points; }
};

// Note: LaserScan uses the PointCloud2D traits through inheritance
// (LaserScan -> PointCloud2DCPU -> PointCloud2D)

}  // namespace frame
}  // namespace gtsam_points
