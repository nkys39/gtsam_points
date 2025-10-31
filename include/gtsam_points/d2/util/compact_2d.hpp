// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <Eigen/Core>

namespace gtsam_points {

/**
 * @brief Compact 2x2 covariance matrix to 3-element vector
 *
 * Stores upper-triangular elements: (m00, m01, m11)
 * This is used for memory-efficient caching of 2D covariance matrices.
 *
 * @param cov 2x2 covariance matrix (extracted from upper-left of 3x3)
 * @return Compact 3-element vector [m00, m01, m11]
 */
inline Eigen::Vector3f compact_cov_2d(const Eigen::Matrix2d& cov) {
  Eigen::Vector3f compact;
  compact << cov(0, 0), cov(0, 1), cov(1, 1);
  return compact;
}

/**
 * @brief Compact 2x2 part of 3x3 covariance matrix
 *
 * @param cov 3x3 matrix (2x2 covariance in upper-left)
 * @return Compact 3-element vector [m00, m01, m11]
 */
inline Eigen::Vector3f compact_cov_2d(const Eigen::Matrix3d& cov) {
  return compact_cov_2d(cov.block<2, 2>(0, 0));
}

/**
 * @brief Uncompact 3-element vector to 3x3 covariance matrix
 *
 * Reconstructs 2x2 covariance from upper-triangular storage and
 * stores it in upper-left of 3x3 matrix (homogeneous coordinates).
 *
 * @param compact Compact 3-element vector [m00, m01, m11]
 * @return 3x3 matrix with 2x2 covariance in upper-left
 */
inline Eigen::Matrix3d uncompact_cov_2d(const Eigen::Vector3f& compact) {
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  cov(0, 0) = compact(0);
  cov(0, 1) = cov(1, 0) = compact(1);
  cov(1, 1) = compact(2);
  // cov(2,2) and other elements remain zero (homogeneous coordinate)
  return cov;
}

}  // namespace gtsam_points
