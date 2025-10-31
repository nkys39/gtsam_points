// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)
#pragma once

#include <random>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_points/types/point_cloud.hpp>
#include <gtsam_points/d2/ann/nearest_neighbor_search_2d.hpp>
#include <gtsam_points/d2/registration/registration_result_2d.hpp>

namespace gtsam_points {

/// @brief RANSAC parameters for 2D registration
struct RANSACParams2D {
  int max_iterations = 5000;             ///< Maximum number of iterations
  double early_stop_inlier_rate = 0.8;   ///< Maximum inlier rate for early stopping
  double poly_error_thresh = 0.5;        ///< Polynomial error threshold
  double inlier_voxel_resolution = 1.0;  ///< Inlier voxel resolution
  std::uint64_t seed = 5489u;            ///< Random seed
  int num_threads = 4;                   ///< Number of threads

  double taboo_thresh_rot = 0.5 * M_PI / 180.0;  ///< Taboo threshold in radian
  double taboo_thresh_trans = 0.25;              ///< Taboo threshold in meter
  std::vector<Eigen::Isometry2d> taboo_list;     ///< Taboo list
};

/// @brief Estimate 2D pose using RANSAC
/// @param target                 Target 2D point cloud
/// @param source                 Source 2D point cloud
/// @param target_features        Target features
/// @param source_features        Source features
/// @param target_tree            Target nearest neighbor search
/// @param target_features_tree   Target features nearest neighbor search
/// @param params                 RANSAC parameters
/// @return                       Registration result
template <typename PointCloud, typename Features>
RegistrationResult2D estimate_pose_ransac_2d_(
  const PointCloud& target,
  const PointCloud& source,
  const Features& target_features,
  const Features& source_features,
  const NearestNeighborSearch2D& target_tree,
  const NearestNeighborSearch2D& target_features_tree,
  const RANSACParams2D& params = RANSACParams2D());

// Overload for 33-dimensional features
RegistrationResult2D estimate_pose_ransac_2d(
  const PointCloud2D& target,
  const PointCloud2D& source,
  const Eigen::Matrix<double, 33, 1>* target_features,
  const Eigen::Matrix<double, 33, 1>* source_features,
  const NearestNeighborSearch2D& target_tree,
  const NearestNeighborSearch2D& target_features_tree,
  const RANSACParams2D& params = RANSACParams2D()) {
  using ConstFeaturePtr = const Eigen::Matrix<double, 33, 1>*;
  return estimate_pose_ransac_2d_<PointCloud2D, ConstFeaturePtr>(
    target,
    source,
    target_features,
    source_features,
    target_tree,
    target_features_tree,
    params);
}

// Overload for 125-dimensional features
RegistrationResult2D estimate_pose_ransac_2d(
  const PointCloud2D& target,
  const PointCloud2D& source,
  const Eigen::Matrix<double, 125, 1>* target_features,
  const Eigen::Matrix<double, 125, 1>* source_features,
  const NearestNeighborSearch2D& target_tree,
  const NearestNeighborSearch2D& target_features_tree,
  const RANSACParams2D& params = RANSACParams2D()) {
  using ConstFeaturePtr = const Eigen::Matrix<double, 125, 1>*;
  return estimate_pose_ransac_2d_<PointCloud2D, ConstFeaturePtr>(
    target,
    source,
    target_features,
    source_features,
    target_tree,
    target_features_tree,
    params);
}

// Overload for variable-dimensional features
RegistrationResult2D estimate_pose_ransac_2d(
  const PointCloud2D& target,
  const PointCloud2D& source,
  const Eigen::VectorXd* target_features,
  const Eigen::VectorXd* source_features,
  const NearestNeighborSearch2D& target_tree,
  const NearestNeighborSearch2D& target_features_tree,
  const RANSACParams2D& params = RANSACParams2D()) {
  using ConstFeaturePtr = const Eigen::VectorXd*;
  return estimate_pose_ransac_2d_<PointCloud2D, ConstFeaturePtr>(
    target,
    source,
    target_features,
    source_features,
    target_tree,
    target_features_tree,
    params);
}

}  // namespace gtsam_points
