// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)
#include <gtsam_points/d2/registration/alignment_2d.hpp>

#include <numeric>
#include <iostream>
#include <Eigen/Eigen>

namespace gtsam_points {

Eigen::Isometry2d align_points_se2(
  const Eigen::Vector3d& target1,
  const Eigen::Vector3d& target2,
  const Eigen::Vector3d& source1,
  const Eigen::Vector3d& source2) {
  // Compute centroids
  const Eigen::Vector3d mean_target = (target1 + target2) / 2.0;
  const Eigen::Vector3d mean_source = (source1 + source2) / 2.0;

  // Build the cross-covariance matrix H
  const Eigen::Matrix3d H = (target1 - mean_target) * (source1 - mean_source).transpose() +  //
                            (target2 - mean_target) * (source2 - mean_source).transpose();

  // Perform SVD on the 2x2 upper-left block (rotation part)
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(H.block<2, 2>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto& U = svd.matrixU();
  const auto& V = svd.matrixV();
  Eigen::Vector2d S = Eigen::Vector2d::Ones();

  // Ensure proper rotation (not reflection)
  const double det = U.determinant() * V.determinant();
  if (det < 0.0) {
    S(1) = -1;
  }

  // Construct the SE(2) transformation
  Eigen::Isometry2d T_target_source = Eigen::Isometry2d::Identity();
  T_target_source.linear() = U * S.asDiagonal() * V.transpose();
  T_target_source.translation() = mean_target.head<2>() - T_target_source.linear() * mean_source.head<2>();

  return T_target_source;
}

Eigen::Isometry2d
align_points_se2(const Eigen::Vector3d* target_points, const Eigen::Vector3d* source_points, const double* weights, size_t num_points) {
  // Compute weighted centroids
  double sum_weights = 0.0;
  Eigen::Vector3d mean_target = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_source = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < num_points; i++) {
    sum_weights += weights[i];
    mean_target += weights[i] * target_points[i];
    mean_source += weights[i] * source_points[i];
  }

  mean_target /= sum_weights;
  mean_source /= sum_weights;

  // Build the cross-covariance matrix H
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < num_points; i++) {
    H += weights[i] * (target_points[i] - mean_target) * (source_points[i] - mean_source).transpose();
  }

  // Perform SVD on the 2x2 upper-left block (rotation part)
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(H.block<2, 2>(0, 0), Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto& U = svd.matrixU();
  const auto& V = svd.matrixV();
  Eigen::Vector2d S = Eigen::Vector2d::Ones();

  // Ensure proper rotation (not reflection)
  const double det = U.determinant() * V.determinant();
  if (det < 0.0) {
    S(1) = -1;
  }

  // Construct the SE(2) transformation
  Eigen::Isometry2d T_target_source = Eigen::Isometry2d::Identity();
  T_target_source.linear() = U * S.asDiagonal() * V.transpose();
  T_target_source.translation() = mean_target.head<2>() - T_target_source.linear() * mean_source.head<2>();

  return T_target_source;
}

}  // namespace gtsam_points
