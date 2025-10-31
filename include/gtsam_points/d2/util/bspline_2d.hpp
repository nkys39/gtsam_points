// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <gtsam/slam/expressions.h>
#include <gtsam/geometry/Pose2.h>

namespace gtsam_points {

/**
 * @brief B-Spline pose interpolation for 2D (SE(2))
 *        Rotation and translation are independently interpolated
 * @note  Requirement: t0 < t1 < t2 < t3 and t is the normalized time between t1 and t2 in [0, 1]
 *        Sec. 2.2 in https://www.robots.ox.ac.uk/~mobile/Theses/StewartThesis.pdf
 * @param pose0 Pose2 at t0
 * @param pose1 Pose2 at t1
 * @param pose2 Pose2 at t2
 * @param pose3 Pose2 at t3
 * @param t     Normalized time between t1 and t2 in [0, 1]
 * @return      Interpolated pose (SE(2))
 */
gtsam::Pose2_ bspline_2d(const gtsam::Pose2_& pose0, const gtsam::Pose2_& pose1, const gtsam::Pose2_& pose2, const gtsam::Pose2_& pose3, const gtsam::Double_& t);

/**
 * @brief B-Spline pose interpolation for 2D (SE(2))
 *        Rotation and translation are jointly interpolated on SE(2) manifold
 *        This is suitable for interpolating vehicle motion in 2D
 * @note  Requirement: t0 < t1 < t2 < t3 and t is the normalized time between t1 and t2 in [0, 1]
 *        Sec. 2.2 in https://www.robots.ox.ac.uk/~mobile/Theses/StewartThesis.pdf
 * @param pose0 Pose2 at t0
 * @param pose1 Pose2 at t1
 * @param pose2 Pose2 at t2
 * @param pose3 Pose2 at t3
 * @param t     Normalized time between t1 and t2 in [0, 1]
 * @return      Interpolated pose (SE(2))
 */
gtsam::Pose2_ bspline_se2(const gtsam::Pose2_& pose0, const gtsam::Pose2_& pose1, const gtsam::Pose2_& pose2, const gtsam::Pose2_& pose3, const gtsam::Double_& t);

/**
 * @brief B-Spline rotation interpolation for 2D (SO(2))
 * @note  Requirement: t0 < t1 < t2 < t3 and t is the normalized time between t1 and t2 in [0, 1]
 *        Sec. 2.2 in https://www.robots.ox.ac.uk/~mobile/Theses/StewartThesis.pdf
 * @param rot0  Rot2 at t0
 * @param rot1  Rot2 at t1
 * @param rot2  Rot2 at t2
 * @param rot3  Rot2 at t3
 * @param t     Normalized time between t1 and t2 in [0, 1]
 * @return      Interpolated rotation (SO(2))
 */
gtsam::Rot2_ bspline_so2(const gtsam::Rot2_& rot0, const gtsam::Rot2_& rot1, const gtsam::Rot2_& rot2, const gtsam::Rot2_& rot3, const gtsam::Double_& t);

/**
 * @brief B-Spline translation interpolation for 2D
 * @note  Requirement: t0 < t1 < t2 < t3 and t is the normalized time between t1 and t2 in [0, 1]
 *        Sec. 2.2 in https://www.robots.ox.ac.uk/~mobile/Theses/StewartThesis.pdf
 * @param trans0  Translation (Vector2) at t0
 * @param trans1  Translation (Vector2) at t1
 * @param trans2  Translation (Vector2) at t2
 * @param trans3  Translation (Vector2) at t3
 * @param t       Normalized time between t1 and t2 in [0, 1]
 * @return        Interpolated translation (Vector2)
 */
gtsam::Vector2_ bspline_trans_2d(
  const gtsam::Vector2_& trans0,
  const gtsam::Vector2_& trans1,
  const gtsam::Vector2_& trans2,
  const gtsam::Vector2_& trans3,
  const gtsam::Double_& t);

/**
 * @brief Calculate angular velocity (yaw rate) of B-spline interpolated 2D trajectory
 *        Sommer et al., "Efficient Derivative Computation for Cumulative B-Splines on Lie Groups", CVPR2020
 * @param rot0          Rot2 at t0
 * @param rot1          Rot2 at t1
 * @param rot2          Rot2 at t2
 * @param rot3          Rot2 at t3
 * @param t             Normalized time between t1 and t2 in [0, 1]
 * @param knot_interval Real time interval between spline knots
 * @return              Angular velocity (yaw rate) as a scalar expression
 */
gtsam::Double_
bspline_angular_vel_2d(const gtsam::Rot2_& rot0, const gtsam::Rot2_& rot1, const gtsam::Rot2_& rot2, const gtsam::Rot2_& rot3, const gtsam::Double_& t, const double knot_interval);

/**
 * @brief Calculate linear velocity (2D) of B-spline interpolated trajectory
 *        Sommer et al., "Efficient Derivative Computation for Cumulative B-Splines on Lie Groups", CVPR2020
 * @param trans0        Translation (Vector2) at t0
 * @param trans1        Translation (Vector2) at t1
 * @param trans2        Translation (Vector2) at t2
 * @param trans3        Translation (Vector2) at t3
 * @param t             Normalized time between t1 and t2 in [0, 1]
 * @param knot_interval Real time interval between spline knots
 * @return              Linear velocity (vx, vy) in global frame
 */
gtsam::Vector2_ bspline_linear_vel_2d(
  const gtsam::Vector2_& trans0,
  const gtsam::Vector2_& trans1,
  const gtsam::Vector2_& trans2,
  const gtsam::Vector2_& trans3,
  const gtsam::Double_& t,
  const double knot_interval);

/**
 * @brief Calculate linear acceleration (2D) of B-spline interpolated trajectory
 *        Sommer et al., "Efficient Derivative Computation for Cumulative B-Splines on Lie Groups", CVPR2020
 * @param trans0        Translation (Vector2) at t0
 * @param trans1        Translation (Vector2) at t1
 * @param trans2        Translation (Vector2) at t2
 * @param trans3        Translation (Vector2) at t3
 * @param t             Normalized time between t1 and t2 in [0, 1]
 * @param knot_interval Real time interval between spline knots
 * @return              Linear acceleration (ax, ay) in global frame
 */
gtsam::Vector2_ bspline_linear_acc_2d(
  const gtsam::Vector2_& trans0,
  const gtsam::Vector2_& trans1,
  const gtsam::Vector2_& trans2,
  const gtsam::Vector2_& trans3,
  const gtsam::Double_& t,
  const double knot_interval);

/**
 * @brief Calculate local linear acceleration and angular velocity (2D IMU measurement) of B-spline interpolated trajectory
 *        Returns [ax_local, ay_local, omega_z] where acceleration is in the local frame and angular velocity is the yaw rate
 *
 * @param pose0         Pose2 at t0
 * @param pose1         Pose2 at t1
 * @param pose2         Pose2 at t2
 * @param pose3         Pose2 at t3
 * @param t             Normalized time between t1 and t2 in [0, 1]
 * @param g             Gravity acceleration vector (2D, typically [0, 0] for planar motion or [0, -9.81] if considering vertical)
 * @param knot_interval Real time interval between spline knots
 * @return              Local linear acceleration (ax, ay) and angular velocity (ωz) as Vector3
 */
gtsam::Vector3_ bspline_imu_2d(
  const gtsam::Pose2_ pose0,
  const gtsam::Pose2_ pose1,
  const gtsam::Pose2_ pose2,
  const gtsam::Pose2_ pose3,
  const gtsam::Double_& t,
  const double knot_interval,
  const gtsam::Vector2& g);

// Utility functions for convenient key-based access
inline gtsam::Pose2_ bspline_2d(const gtsam::Key key1, const gtsam::Double_& t) {
  return bspline_2d(gtsam::Pose2_(key1 - 1), gtsam::Pose2_(key1), gtsam::Pose2_(key1 + 1), gtsam::Pose2_(key1 + 2), t);
}

inline gtsam::Pose2_ bspline_se2(const gtsam::Key key1, const gtsam::Double_& t) {
  return bspline_se2(gtsam::Pose2_(key1 - 1), gtsam::Pose2_(key1), gtsam::Pose2_(key1 + 1), gtsam::Pose2_(key1 + 2), t);
}

inline gtsam::Vector3_ bspline_imu_2d(const gtsam::Key key1, const gtsam::Double_& t, const double knot_interval, const gtsam::Vector2& g) {
  return bspline_imu_2d(gtsam::Pose2_(key1 - 1), gtsam::Pose2_(key1), gtsam::Pose2_(key1 + 1), gtsam::Pose2_(key1 + 2), t, knot_interval, g);
}

}  // namespace gtsam_points
