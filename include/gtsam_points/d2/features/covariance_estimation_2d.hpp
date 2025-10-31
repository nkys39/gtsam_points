// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <vector>
#include <Eigen/Core>
#include <gtsam_points/d2/types/point_cloud_2d.hpp>

namespace gtsam_points {

/**
 * @brief 2D Covariance estimation parameters
 */
struct CovarianceEstimationParams2D {
public:
  enum RegularizationMethod { NONE, EIG };

  CovarianceEstimationParams2D()
  : num_threads(1),
    k_neighbors(5),  // Fewer neighbors needed for 2D
    regularization_method(EIG),
    eigen_values(1e-3, 1.0)  // 2D: only 2 eigenvalues
  {}

public:
  int num_threads;                             ///< Number of threads
  int k_neighbors;                             ///< Number of neighboring points used for covariance estimation
  RegularizationMethod regularization_method;  ///< Regularization method
  Eigen::Vector2d eigen_values;                ///< Eigenvalues used for EIG regularization (2D)
};

/**
 * @brief Estimate 2D point covariances from neighboring points
 *
 * Estimates 2x2 covariance matrices stored in upper-left of 3x3 matrices.
 * The third row/column (homogeneous coordinate) is set to zero.
 *
 * @param points       Input 2D points (homogeneous: x, y, 1)
 * @param num_points   Number of input points
 * @param params       Estimation params
 * @return             Estimated 2x2 covariances (in 3x3 format)
 */
std::vector<Eigen::Matrix3d> estimate_covariances_2d(const Eigen::Vector3d* points, int num_points, const CovarianceEstimationParams2D& params);

/**
 * @brief Estimate 2D point covariances from neighboring points
 * @param points       Input 2D points
 * @param num_points   Number of input points
 * @param k_neighbors  Number of neighboring points for covariance estimation (default=5 for 2D)
 * @param eigen_values Eigenvalues used for regularization (default=[1e-3, 1])
 * @param num_threads  Number of threads
 * @return             Estimated covariances (2x2 in 3x3 format)
 */
std::vector<Eigen::Matrix3d>
estimate_covariances_2d(const Eigen::Vector3d* points, int num_points, int k_neighbors, const Eigen::Vector2d& eigen_values, int num_threads);

std::vector<Eigen::Matrix3d> estimate_covariances_2d(const Eigen::Vector3d* points, int num_points, int k_neighbors = 5, int num_threads = 1);

template <typename Alloc>
std::vector<Eigen::Matrix3d> estimate_covariances_2d(const std::vector<Eigen::Vector3d, Alloc>& points, int k_neighbors = 5, int num_threads = 1) {
  return estimate_covariances_2d(points.data(), points.size(), k_neighbors, num_threads);
}

std::vector<Eigen::Matrix3d> estimate_covariances_2d(const PointCloud2D& points, int k_neighbors = 5, int num_threads = 1);

}  // namespace gtsam_points
