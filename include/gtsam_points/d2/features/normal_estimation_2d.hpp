// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <vector>
#include <Eigen/Core>
#include <gtsam_points/d2/types/point_cloud_2d.hpp>

namespace gtsam_points {

/**
 * @brief Estimate 2D point normals from covariances
 *
 * For 2D points, normals are 2D vectors perpendicular to the local line segment.
 * Stored as (nx, ny, 0) in homogeneous coordinates.
 *
 * @param points      Input 2D points (homogeneous coordinates: x, y, 1)
 * @param covs        Input 2x2 covariances (stored in upper-left of 3x3)
 * @param num_points  Number of input points and covariances
 * @param num_threads Number of threads
 * @return            Estimated 2D normals (nx, ny, 0)
 */
std::vector<Eigen::Vector3d> estimate_normals_2d(const Eigen::Vector3d* points, const Eigen::Matrix3d* covs, int num_points, int num_threads = 1);

/**
 * @brief Estimate 2D point normals from neighboring points
 *
 * @param points      Input 2D points (homogeneous coordinates)
 * @param num_points  Number of input points
 * @param k_neighbors Number of neighboring points for normal estimation (typically 5-10 for 2D)
 * @param num_threads Number of threads
 * @return            Estimated 2D normals (nx, ny, 0)
 */
std::vector<Eigen::Vector3d> estimate_normals_2d(const Eigen::Vector3d* points, int num_points, int k_neighbors = 5, int num_threads = 1);

template <typename Alloc>
std::vector<Eigen::Vector3d> estimate_normals_2d(const std::vector<Eigen::Vector3d, Alloc>& points, int k_neighbors = 5, int num_threads = 1) {
  return estimate_normals_2d(points.data(), points.size(), k_neighbors, num_threads);
}

std::vector<Eigen::Vector3d> estimate_normals_2d(const PointCloud2D& points, int k_neighbors = 5, int num_threads = 1);

}  // namespace gtsam_points
