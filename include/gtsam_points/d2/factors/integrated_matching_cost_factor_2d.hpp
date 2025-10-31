// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>

namespace gtsam_points {

/**
 * @brief Abstraction of LSQ-based 2D scan matching constraints between point clouds
 *
 * This is the 2D equivalent of IntegratedMatchingCostFactor.
 * It handles scan matching in 2D space (x, y, θ) with 3 DOF.
 */
class IntegratedMatchingCostFactor2D : public gtsam::NonlinearFactor {
public:
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
  using shared_ptr = std::shared_ptr<IntegratedMatchingCostFactor2D>;

  /**
   * @brief Create a binary matching cost factor between target and source poses
   * @param target_key  Target key
   * @param source_key  Source key
   */
  IntegratedMatchingCostFactor2D(gtsam::Key target_key, gtsam::Key source_key);

  /**
   * @brief Create a unary matching cost factor between a fixed target pose and an active source pose
   * @param fixed_target_pose  Fixed target pose (2D)
   * @param source_key         Source key
   */
  IntegratedMatchingCostFactor2D(const gtsam::Pose2& fixed_target_pose, gtsam::Key source_key);

  virtual ~IntegratedMatchingCostFactor2D() override;

  /// @brief Dimension of the factor (3 DOF for 2D: x, y, θ)
  virtual size_t dim() const override { return 3; }

  /// @brief Print the factor information.
  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

  /// @note The following error and linearize methods are not thread-safe,
  ///       because we need to update correspondences (that may be mutable members) for every linearization
  virtual double error(const gtsam::Values& values) const override;
  virtual std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& values) const override;

  const Eigen::Isometry2d& get_fixed_target_pose() const { return fixed_target_pose; }

public:
  Eigen::Isometry2d calc_delta(const gtsam::Values& values) const;

  /**
   * @brief  Calculate the memory usage of this factor
   * @note   The result is approximate and does not account for objects not owned by this factor (e.g., point clouds)
   * @return Memory usage in bytes (Approximate size in bytes)
   */
  virtual size_t memory_usage() const;

  /**
   * @brief Update point correspondences
   * @param delta Transformation between target and source in 2D (T_target_source)
   */
  virtual void update_correspondences(const Eigen::Isometry2d& delta) const = 0;

  /**
   * @brief Evaluate the matching cost in 2D
   * @param delta Transformation between target and source (T_target_source)
   * @param H_target         Hessian (target x target) - 3x3 for 2D
   * @param H_source         Hessian (source x source) - 3x3 for 2D
   * @param H_target_source  Hessian (target x source) - 3x3 for 2D
   * @param b_target         Error vector (target) - 3x1 for 2D
   * @param b_source         Error vector (source) - 3x1 for 2D
   */
  virtual double evaluate(
    const Eigen::Isometry2d& delta,
    Eigen::Matrix<double, 3, 3>* H_target = nullptr,
    Eigen::Matrix<double, 3, 3>* H_source = nullptr,
    Eigen::Matrix<double, 3, 3>* H_target_source = nullptr,
    Eigen::Matrix<double, 3, 1>* b_target = nullptr,
    Eigen::Matrix<double, 3, 1>* b_source = nullptr) const = 0;

protected:
  bool is_binary;
  Eigen::Isometry2d fixed_target_pose;
};

}  // namespace gtsam_points
