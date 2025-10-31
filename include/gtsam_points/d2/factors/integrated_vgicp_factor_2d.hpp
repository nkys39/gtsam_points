// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <gtsam_points/d2/types/point_cloud_2d.hpp>
#include <gtsam_points/d2/types/gaussian_gridmap_2d.hpp>
#include <gtsam_points/d2/factors/integrated_matching_cost_factor_2d.hpp>
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>

namespace gtsam_points {

/**
 * @brief Voxelized GICP matching cost factor for 2D
 *
 * 2D adaptation of Voxelized GICP (Koide et al., "Voxelized GICP for Fast and Accurate 3D Point Cloud Registration", ICRA2021)
 *
 * Uses GaussianGridMap2D for efficient grid-based matching.
 * Each grid cell stores mean and covariance of contained points.
 *
 * Benefits:
 * - Faster than point-to-point GICP2D
 * - Robust to point cloud density variations
 * - Efficient memory usage
 */
template <typename SourceFrame = gtsam_points::PointCloud2D>
class IntegratedVGICPFactor2D_ : public gtsam_points::IntegratedMatchingCostFactor2D {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedVGICPFactor2D_>;

  /**
   * @brief Create a binary VGICP2D factor between target and source poses.
   * @param target_key          Target key
   * @param source_key          Source key
   * @param target_gridmap      Target grid map
   * @param source              Source point cloud frame
   */
  IntegratedVGICPFactor2D_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const GaussianGridMap2D::ConstPtr& target_gridmap,
    const std::shared_ptr<const SourceFrame>& source);

  /**
   * @brief Create a unary VGICP2D factor between a fixed target pose and an active source pose.
   * @param fixed_target_pose   Fixed target pose (2D)
   * @param source_key          Source key
   * @param target_gridmap      Target grid map
   * @param source              Source point cloud frame
   */
  IntegratedVGICPFactor2D_(
    const gtsam::Pose2& fixed_target_pose,
    gtsam::Key source_key,
    const GaussianGridMap2D::ConstPtr& target_gridmap,
    const std::shared_ptr<const SourceFrame>& source);

  virtual ~IntegratedVGICPFactor2D_() override;

  /// @brief Print the factor information.
  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

  /**
   * @brief  Calculate the memory usage of this factor
   * @return Memory usage in bytes (Approximate)
   */
  virtual size_t memory_usage() const override;

  /// @brief Set the number of threads used for linearization.
  void set_num_threads(int n) { num_threads = n; }

  /// @brief Set the cache mode for fused covariance matrices.
  void set_fused_cov_cache_mode(FusedCovCacheMode2D mode) { mahalanobis_cache_mode = mode; }

  /// @brief Get the number of inlier points.
  int num_inliers() const {
    const int outliers = std::count(correspondences.begin(), correspondences.end(), nullptr);
    return correspondences.size() - outliers;
  }

  /// @brief Compute the fraction of inlier points that have correspondences.
  double inlier_fraction() const { return num_inliers() / static_cast<double>(correspondences.size()); }

  /// @brief Get the target grid map.
  const std::shared_ptr<const GaussianGridMap2D>& get_target() const { return target_gridmap; }

  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::NonlinearFactor::shared_ptr(new IntegratedVGICPFactor2D_(*this)); }

private:
  virtual void update_correspondences(const Eigen::Isometry2d& delta) const override;

  virtual double evaluate(
    const Eigen::Isometry2d& delta,
    Eigen::Matrix<double, 3, 3>* H_target = nullptr,
    Eigen::Matrix<double, 3, 3>* H_source = nullptr,
    Eigen::Matrix<double, 3, 3>* H_target_source = nullptr,
    Eigen::Matrix<double, 3, 1>* b_target = nullptr,
    Eigen::Matrix<double, 3, 1>* b_source = nullptr) const override;

private:
  int num_threads;
  FusedCovCacheMode2D mahalanobis_cache_mode;

  mutable Eigen::Isometry2d linearization_point;
  mutable std::vector<const GaussianGridCell2D*> correspondences;  // nullptr = no correspondence
  mutable std::vector<Eigen::Matrix3d> mahalanobis_full;
  mutable std::vector<Eigen::Vector3f> mahalanobis_compact;

  std::shared_ptr<const GaussianGridMap2D> target_gridmap;
  std::shared_ptr<const SourceFrame> source;
};

using IntegratedVGICPFactor2D = IntegratedVGICPFactor2D_<>;

}  // namespace gtsam_points
