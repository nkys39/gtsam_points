// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <gtsam_points/d2/factors/integrated_ct_icp_factor_2d.hpp>

namespace gtsam_points {

/**
 * @brief Continuous Time GICP Factor for 2D SLAM
 *        2D adaptation of CT-GICP combining:
 *        - Bellenbach et al., "CT-ICP: Real-time Elastic LiDAR Odometry with Loop Closure", 2021
 *        - Segal et al., "Generalized-ICP", RSS2005
 *        Uses Mahalanobis distance with per-point covariances for robust matching
 */
template <typename TargetFrame = gtsam_points::PointCloud2D, typename SourceFrame = gtsam_points::PointCloud2D>
class IntegratedCT_GICPFactor2D_ : public IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedCT_GICPFactor2D_<TargetFrame, SourceFrame>>;

  /**
   * @brief Constructor
   * @param source_t0_key   Key of the source Pose2 at the beginning of the scan
   * @param source_t1_key   Key of the source Pose2 at the end of the scan
   * @param target          Target 2D point cloud (must have covariances)
   * @param source          Source 2D point cloud (must have covariances and timestamps)
   * @param target_tree     Nearest neighbor search for the target point cloud
   */
  IntegratedCT_GICPFactor2D_(
    gtsam::Key source_t0_key,
    gtsam::Key source_t1_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree);

  /**
   * @brief Constructor (creates KdTree2D internally)
   * @param source_t0_key   Key of the source Pose2 at the beginning of the scan
   * @param source_t1_key   Key of the source Pose2 at the end of the scan
   * @param target          Target 2D point cloud (must have covariances)
   * @param source          Source 2D point cloud (must have covariances and timestamps)
   */
  IntegratedCT_GICPFactor2D_(
    gtsam::Key source_t0_key,
    gtsam::Key source_t1_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source);

  virtual ~IntegratedCT_GICPFactor2D_() override;

  /// @brief Print the factor information.
  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

  virtual size_t memory_usage() const override;

  virtual double error(const gtsam::Values& values) const override;
  virtual std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& values) const override;

protected:
  virtual void update_correspondences() const override;

  mutable std::vector<Eigen::Matrix3d> mahalanobis;  ///< Mahalanobis matrices (3x3 for 2D homogeneous coordinates)
};

using IntegratedCT_GICPFactor2D = IntegratedCT_GICPFactor2D_<>;

}  // namespace gtsam_points
