// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gtsam_points {

/// @brief  Find the 2D transformation (SE(2)) that aligns two point pairs.
/// @param  target1  First target point in 2D homogeneous coordinates (x, y, 1)
/// @param  target2  Second target point in 2D homogeneous coordinates
/// @param  source1  First source point in 2D homogeneous coordinates
/// @param  source2  Second source point in 2D homogeneous coordinates
/// @return T_target_source that minimizes the sum of squared errors.
Eigen::Isometry2d align_points_se2(
  const Eigen::Vector3d& target1,
  const Eigen::Vector3d& target2,
  const Eigen::Vector3d& source1,
  const Eigen::Vector3d& source2);

/// @brief Find the 2D transformation (SE(2)) that aligns two point sets.
/// @param target_points  Array of target points in 2D homogeneous coordinates
/// @param source_points  Array of source points in 2D homogeneous coordinates
/// @param weights        Array of weights for each point pair
/// @param num_points     Number of point pairs
/// @return T_target_source that minimizes the sum of squared errors.
Eigen::Isometry2d align_points_se2(const Eigen::Vector3d* target_points, const Eigen::Vector3d* source_points, const double* weights, size_t num_points);

}  // namespace gtsam_points
