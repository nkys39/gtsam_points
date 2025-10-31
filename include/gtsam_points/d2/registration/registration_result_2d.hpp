// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gtsam_points {

/// @brief 2D Registration result
struct RegistrationResult2D {
  double inlier_rate;                        ///< Inlier rate (The population differs for each method)
  Eigen::Isometry2d T_target_source;         ///< Estimated 2D transformation (SE(2))
};

}  // namespace gtsam_points
