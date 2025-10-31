// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <gtsam_points/d2/types/point_cloud_2d.hpp>

namespace gtsam_points {

struct NearestNeighborSearch2D;

/**
 * @brief Continuous Time ICP Factor for 2D SLAM
 *        2D adaptation of CT-ICP: Bellenbach et al., "CT-ICP: Real-time Elastic LiDAR Odometry with Loop Closure", 2021
 *        This factor handles motion-compensated scan matching for 2D LiDAR data with per-point timestamps
 */
template <typename TargetFrame = gtsam_points::PointCloud2D, typename SourceFrame = gtsam_points::PointCloud2D>
class IntegratedCT_ICPFactor2D_ : public gtsam::NonlinearFactor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>>;

  /**
   * @brief Constructor
   * @param source_t0_key   Key of the source Pose2 at the beginning of the scan
   * @param source_t1_key   Key of the source Pose2 at the end of the scan
   * @param target          Target 2D point cloud
   * @param source          Source 2D point cloud (must have per-point timestamps)
   * @param target_tree     Nearest neighbor search for the target point cloud
   */
  IntegratedCT_ICPFactor2D_(
    gtsam::Key source_t0_key,
    gtsam::Key source_t1_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree);

  /**
   * @brief Constructor (creates KdTree2D internally)
   * @param source_t0_key   Key of the source Pose2 at the beginning of the scan
   * @param source_t1_key   Key of the source Pose2 at the end of the scan
   * @param target          Target 2D point cloud
   * @param source          Source 2D point cloud (must have per-point timestamps)
   */
  IntegratedCT_ICPFactor2D_(
    gtsam::Key source_t0_key,
    gtsam::Key source_t1_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source);

  virtual ~IntegratedCT_ICPFactor2D_() override;

  /// @brief Print the factor information.
  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

  virtual size_t memory_usage() const;

  virtual size_t dim() const override { return 3; }  // 3 DOF for Pose2 (x, y, θ)
  virtual double error(const gtsam::Values& values) const override;
  virtual std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& values) const override;

  void set_num_threads(int n) { num_threads = n; }
  void set_max_correspondence_distance(double dist) { max_correspondence_distance_sq = dist * dist; }

  const std::vector<double>& get_time_table() const { return time_table; }
  const std::vector<int>& get_time_indices() const { return time_indices; }
  const std::vector<gtsam::Pose2>& get_source_poses() const { return source_poses; }

  std::vector<Eigen::Vector3d> deskewed_source_points(const gtsam::Values& values, bool local = false);

public:
  virtual void update_poses(const gtsam::Values& values) const;

protected:
  virtual void update_correspondences() const;

protected:
  int num_threads;
  double max_correspondence_distance_sq;

  std::shared_ptr<const NearestNeighborSearch2D> target_tree;

  std::vector<double> time_table;                      ///< Normalized timestamps [0, 1]
  mutable std::vector<gtsam::Pose2> source_poses;      ///< Interpolated poses at each unique time
  mutable std::vector<gtsam::Matrix3> pose_derivatives_t0;  ///< Derivatives w.r.t. t0 pose (3x3 for Pose2)
  mutable std::vector<gtsam::Matrix3> pose_derivatives_t1;  ///< Derivatives w.r.t. t1 pose (3x3 for Pose2)

  std::vector<int> time_indices;                       ///< Time table index for each point
  mutable std::vector<long> correspondences;           ///< Correspondence indices (-1 if no match)

  std::shared_ptr<const TargetFrame> target;
  std::shared_ptr<const SourceFrame> source;
};

using IntegratedCT_ICPFactor2D = IntegratedCT_ICPFactor2D_<>;

}  // namespace gtsam_points
