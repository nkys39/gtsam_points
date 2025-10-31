// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <gtsam_points/d2/types/point_cloud_2d.hpp>
#include <gtsam_points/d2/factors/integrated_matching_cost_factor_2d.hpp>

namespace gtsam_points {

struct NearestNeighborSearch2D;

/**
 * @brief Cache mode for fused 2D covariance matrices (i.e., mahalanobis)
 */
enum class FusedCovCacheMode2D {
  FULL,     // Full 2x2 matrix stored in 3x3 (double precision: ~72 bytes per point, fast)
  COMPACT,  // Compact upper-triangular 2x2 (3 floats: 12 bytes per point, intermediate)
  NONE      // No cache (0 bytes per point, slow - recompute every time)
};

/**
 * @brief 2D Generalized ICP matching cost factor
 *
 * 2D adaptation of Generalized ICP (Segal et al., "Generalized-ICP", RSS2005)
 * Uses 2x2 covariance matrices for probabilistic point cloud alignment.
 *
 * Key differences from 3D:
 * - Uses 2x2 covariances (stored in upper-left of 3x3)
 * - Mahalanobis distance computed in 2D space
 * - 3 DOF optimization (x, y, θ)
 */
template <typename TargetFrame = gtsam_points::PointCloud2D, typename SourceFrame = gtsam_points::PointCloud2D>
class IntegratedGICPFactor2D_ : public gtsam_points::IntegratedMatchingCostFactor2D {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedGICPFactor2D_>;

  /**
   * @brief Create a binary 2D GICP factor between target and source poses.
   * @param target_key          Target key
   * @param source_key          Source key
   * @param target              Target 2D point cloud frame
   * @param source              Source 2D point cloud frame
   * @param target_tree         Target nearest neighbor search
   */
  IntegratedGICPFactor2D_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree);

  /// Create a binary 2D GICP factor (auto-create KdTree).
  IntegratedGICPFactor2D_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source);

  /**
   * @brief Create a unary 2D GICP factor between a fixed target pose and an active source pose.
   * @param fixed_target_pose   Fixed target pose (2D)
   * @param source_key          Source key
   * @param target              Target 2D point cloud frame
   * @param source              Source 2D point cloud frame
   * @param target_tree         Target nearest neighbor search
   */
  IntegratedGICPFactor2D_(
    const gtsam::Pose2& fixed_target_pose,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree);

  /// Create a unary 2D GICP factor (auto-create KdTree).
  IntegratedGICPFactor2D_(
    const gtsam::Pose2& fixed_target_pose,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source);

  virtual ~IntegratedGICPFactor2D_() override;

  /// @brief Print the factor information.
  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

  /**
   * @brief  Calculate the memory usage of this factor
   * @return Memory usage in bytes (Approximate)
   */
  virtual size_t memory_usage() const override;

  /// @brief Set the number of threads used for linearization of this factor.
  void set_num_threads(int n) { num_threads = n; }

  /// @brief Set the maximum distance between corresponding points.
  void set_max_correspondence_distance(double dist) { max_correspondence_distance_sq = dist * dist; }

  /// @brief Correspondences are updated only when the displacement from the last update point is larger than these threshold values.
  void set_correspondence_update_tolerance(double angle, double trans) {
    correspondence_update_tolerance_rot = angle;
    correspondence_update_tolerance_trans = trans;
  }

  /// @brief Set the cache mode for fused covariance matrices (i.e., mahalanobis).
  void set_fused_cov_cache_mode(FusedCovCacheMode2D mode) { mahalanobis_cache_mode = mode; }

  /// @brief Compute the fraction of inlier points that have correspondences.
  double inlier_fraction() const {
    const int outliers = std::count(correspondences.begin(), correspondences.end(), -1);
    const int inliers = correspondences.size() - outliers;
    return static_cast<double>(inliers) / correspondences.size();
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override { return gtsam::NonlinearFactor::shared_ptr(new IntegratedGICPFactor2D_(*this)); }

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
  double max_correspondence_distance_sq;
  FusedCovCacheMode2D mahalanobis_cache_mode;

  std::shared_ptr<const NearestNeighborSearch2D> target_tree;

  // Correspondence tracking (mutable for caching)
  double correspondence_update_tolerance_rot;
  double correspondence_update_tolerance_trans;
  mutable Eigen::Isometry2d linearization_point;
  mutable Eigen::Isometry2d last_correspondence_point;
  mutable std::vector<long> correspondences;
  mutable std::vector<Eigen::Matrix3d> mahalanobis_full;       // 2x2 in upper-left of 3x3
  mutable std::vector<Eigen::Vector3f> mahalanobis_compact;    // Upper-triangular: (m00, m01, m11)

  std::shared_ptr<const TargetFrame> target;
  std::shared_ptr<const SourceFrame> source;
};

using IntegratedGICPFactor2D = IntegratedGICPFactor2D_<>;

}  // namespace gtsam_points
