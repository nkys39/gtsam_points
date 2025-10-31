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
 * @brief 2D Point-to-Point ICP matching cost factor
 *
 * 2D equivalent of IntegratedICPFactor.
 * Implements naive point-to-point distance minimization in 2D space.
 * Reference: Zhang, "Iterative Point Matching for Registration of Free-Form Curve", IJCV1994
 */
template <typename TargetFrame = gtsam_points::PointCloud2D, typename SourceFrame = gtsam_points::PointCloud2D>
class IntegratedICPFactor2D_ : public gtsam_points::IntegratedMatchingCostFactor2D {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedICPFactor2D_<PointCloud2D>>;

  /**
   * @brief Create a binary 2D ICP factor between two poses.
   * @param target_key          Target key
   * @param source_key          Source key
   * @param target              Target 2D point cloud frame
   * @param source              Source 2D point cloud frame
   * @param target_tree         Target nearest neighbor search (2D)
   * @param use_point_to_line   If true, use point-to-line distance instead of point-to-point distance
   */
  IntegratedICPFactor2D_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree,
    bool use_point_to_line = false);

  /// Create a binary 2D ICP factor between two poses (auto-create KdTree).
  IntegratedICPFactor2D_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    bool use_point_to_line = false);

  /**
   * @brief Create a unary 2D ICP factor between a fixed target pose and an active source pose.
   * @param fixed_target_pose   Fixed target pose (2D)
   * @param source_key          Source key
   * @param target              Target 2D point cloud frame
   * @param source              Source 2D point cloud frame
   * @param target_tree         Target nearest neighbor search (2D)
   * @param use_point_to_line   If true, use point-to-line distance instead of point-to-point distance
   */
  IntegratedICPFactor2D_(
    const gtsam::Pose2& fixed_target_pose,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree,
    bool use_point_to_line = false);

  /// Create a unary 2D ICP factor between a fixed target pose and an active source pose (auto-create KdTree).
  IntegratedICPFactor2D_(
    const gtsam::Pose2& fixed_target_pose,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    bool use_point_to_line = false);

  virtual ~IntegratedICPFactor2D_() override;

  /// @brief Print the factor information.
  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

  /**
   * @brief  Calculate the memory usage of this factor
   * @note   The result is approximate and does not account for objects not owned by this factor (e.g., point clouds)
   * @return Memory usage in bytes (Approximate size in bytes)
   */
  virtual size_t memory_usage() const override;

  /// @brief Set the number of thread used for linearization of this factor.
  void set_num_threads(int n) { num_threads = n; }

  /// @brief Set the maximum distance between corresponding points.
  ///        Correspondences with distances larger than this will be rejected (i.e., correspondence trimming).
  void set_max_correspondence_distance(double dist) { max_correspondence_distance_sq = dist * dist; }

  /// @brief Enable or disable point-to-line distance computation.
  void set_point_to_line_distance(bool use) { use_point_to_line = use; }

  /// @brief Correspondences are updated only when the displacement from the last update point is larger than these threshold values.
  /// @note  Default values are angle=trans=0 and correspondences are updated every linearization call.
  void set_correspondence_update_tolerance(double angle, double trans) {
    correspondence_update_tolerance_rot = angle;
    correspondence_update_tolerance_trans = trans;
  }

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
  bool use_point_to_line;

  std::shared_ptr<const NearestNeighborSearch2D> target_tree;

  // Correspondence tracking (mutable for caching)
  double correspondence_update_tolerance_rot;
  double correspondence_update_tolerance_trans;
  mutable Eigen::Isometry2d last_correspondence_point;
  mutable std::vector<long> correspondences;

  std::shared_ptr<const TargetFrame> target;
  std::shared_ptr<const SourceFrame> source;
};

/**
 * @brief 2D Point-to-Line ICP factor
 *
 * Uses point-to-line distance for 2D scan matching.
 * This is the 2D equivalent of point-to-plane ICP in 3D.
 */
template <typename TargetFrame = gtsam_points::PointCloud2D, typename SourceFrame = gtsam_points::PointCloud2D>
class IntegratedPointToLineICPFactor2D_ : public gtsam_points::IntegratedICPFactor2D_<TargetFrame, SourceFrame> {
public:
  using shared_ptr = std::shared_ptr<IntegratedPointToLineICPFactor2D_<TargetFrame, SourceFrame>>;

  IntegratedPointToLineICPFactor2D_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree)
  : IntegratedICPFactor2D_<TargetFrame, SourceFrame>(target_key, source_key, target, source, target_tree, true) {}

  IntegratedPointToLineICPFactor2D_(
    gtsam::Key target_key,
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source)
  : IntegratedICPFactor2D_<TargetFrame, SourceFrame>(target_key, source_key, target, source, true) {}
};

using IntegratedICPFactor2D = IntegratedICPFactor2D_<>;
using IntegratedPointToLineICPFactor2D = IntegratedPointToLineICPFactor2D_<>;

}  // namespace gtsam_points
