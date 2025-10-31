// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#include <gtsam_points/d2/util/bspline_2d.hpp>

#include <gtsam_points/util/expressions.hpp>

namespace gtsam_points {

// Helper expressions for Pose2 and Rot2
namespace {

inline gtsam::Pose2_ create_se2(const gtsam::Rot2_& rot, const gtsam::Vector2_& trans) {
  return gtsam::Pose2_(&gtsam::Pose2::Create, rot, trans);
}

inline gtsam::Pose2_ expmap_se2(const gtsam::Vector3_& x) {
  return gtsam::Pose2_(&gtsam::Pose2::Expmap, x);
}

inline gtsam::Vector3_ logmap_se2(const gtsam::Pose2_& x) {
  return gtsam::Vector3_(&gtsam::Pose2::Logmap, x);
}

inline gtsam::Rot2_ expmap_so2(const gtsam::Double_& x) {
  return gtsam::Rot2_(&gtsam::Rot2::Expmap, x);
}

inline gtsam::Double_ logmap_so2(const gtsam::Rot2_& x) {
  return gtsam::Double_(&gtsam::Rot2::Logmap, x);
}

}  // namespace

gtsam::Pose2_
bspline_2d(const gtsam::Pose2_& pose0, const gtsam::Pose2_& pose1, const gtsam::Pose2_& pose2, const gtsam::Pose2_& pose3, const gtsam::Double_& t) {
  const gtsam::Double_ t2 = t * t;
  const gtsam::Double_ t3 = t2 * t;

  // B-spline basis functions (cubic)
  const gtsam::Double_ beta1 = gtsam::Double_(5.0 / 6.0) + 3.0 / 6.0 * t - 3.0 / 6.0 * t2 + 1.0 / 6.0 * t3;
  const gtsam::Double_ beta2 = gtsam::Double_(1.0 / 6.0) + 3.0 / 6.0 * t + 3.0 / 6.0 * t2 - 2.0 / 6.0 * t3;
  const gtsam::Double_ beta3 = 1.0 / 6.0 * t3;

  // Rotation interpolation (SO(2))
  const gtsam::Rot2_ rot0 = gtsam::rotation(pose0);
  const gtsam::Rot2_ rot1 = gtsam::rotation(pose1);
  const gtsam::Rot2_ rot2 = gtsam::rotation(pose2);
  const gtsam::Rot2_ rot3 = gtsam::rotation(pose3);

  // Compute rotation deltas using exponential map on SO(2)
  const gtsam::Rot2_ r_delta1 = expmap_so2(beta1 * logmap_so2(rot0.between(rot1)));
  const gtsam::Rot2_ r_delta2 = expmap_so2(beta2 * logmap_so2(rot1.between(rot2)));
  const gtsam::Rot2_ r_delta3 = expmap_so2(beta3 * logmap_so2(rot2.between(rot3)));

  // Translation interpolation (R^2)
  const gtsam::Vector2_ trans0 = gtsam::translation(pose0);
  const gtsam::Vector2_ trans1 = gtsam::translation(pose1);
  const gtsam::Vector2_ trans2 = gtsam::translation(pose2);
  const gtsam::Vector2_ trans3 = gtsam::translation(pose3);

  const gtsam::Vector2_ t_delta1 = gtsam_points::scale<2>(beta1, gtsam::between(trans0, trans1));
  const gtsam::Vector2_ t_delta2 = gtsam_points::scale<2>(beta2, gtsam::between(trans1, trans2));
  const gtsam::Vector2_ t_delta3 = gtsam_points::scale<2>(beta3, gtsam::between(trans2, trans3));

  // Compose rotations and translations
  const gtsam::Rot2_ rot = gtsam::compose(gtsam::compose(gtsam::compose(rot0, r_delta1), r_delta2), r_delta3);
  const gtsam::Vector2_ trans = gtsam::compose(gtsam::compose(gtsam::compose(trans0, t_delta1), t_delta2), t_delta3);
  const gtsam::Pose2_ pose = create_se2(rot, trans);

  return pose;
}

gtsam::Pose2_
bspline_se2(const gtsam::Pose2_& pose0, const gtsam::Pose2_& pose1, const gtsam::Pose2_& pose2, const gtsam::Pose2_& pose3, const gtsam::Double_& t) {
  const gtsam::Double_ t2 = t * t;
  const gtsam::Double_ t3 = t2 * t;

  // B-spline basis functions (cubic)
  const gtsam::Double_ beta1 = gtsam::Double_(5.0 / 6.0) + 3.0 / 6.0 * t - 3.0 / 6.0 * t2 + 1.0 / 6.0 * t3;
  const gtsam::Double_ beta2 = gtsam::Double_(1.0 / 6.0) + 3.0 / 6.0 * t + 3.0 / 6.0 * t2 - 2.0 / 6.0 * t3;
  const gtsam::Double_ beta3 = 1.0 / 6.0 * t3;

  // Joint SE(2) interpolation using exponential map
  const gtsam::Pose2_ delta1 = expmap_se2(gtsam_points::scale<3>(beta1, logmap_se2(pose0.between(pose1))));
  const gtsam::Pose2_ delta2 = expmap_se2(gtsam_points::scale<3>(beta2, logmap_se2(pose1.between(pose2))));
  const gtsam::Pose2_ delta3 = expmap_se2(gtsam_points::scale<3>(beta3, logmap_se2(pose2.between(pose3))));

  const gtsam::Pose2_ pose = gtsam::compose(gtsam::compose(gtsam::compose(pose0, delta1), delta2), delta3);

  return pose;
}

gtsam::Rot2_
bspline_so2(const gtsam::Rot2_& rot0, const gtsam::Rot2_& rot1, const gtsam::Rot2_& rot2, const gtsam::Rot2_& rot3, const gtsam::Double_& t) {
  const gtsam::Double_ t2 = t * t;
  const gtsam::Double_ t3 = t2 * t;

  // B-spline basis functions (cubic)
  const gtsam::Double_ beta1 = gtsam::Double_(5.0 / 6.0) + 3.0 / 6.0 * t - 3.0 / 6.0 * t2 + 1.0 / 6.0 * t3;
  const gtsam::Double_ beta2 = gtsam::Double_(1.0 / 6.0) + 3.0 / 6.0 * t + 3.0 / 6.0 * t2 - 2.0 / 6.0 * t3;
  const gtsam::Double_ beta3 = 1.0 / 6.0 * t3;

  // Rotation deltas on SO(2)
  const gtsam::Rot2_ r_delta1 = expmap_so2(beta1 * logmap_so2(rot0.between(rot1)));
  const gtsam::Rot2_ r_delta2 = expmap_so2(beta2 * logmap_so2(rot1.between(rot2)));
  const gtsam::Rot2_ r_delta3 = expmap_so2(beta3 * logmap_so2(rot2.between(rot3)));

  const gtsam::Rot2_ rot = gtsam::compose(gtsam::compose(gtsam::compose(rot0, r_delta1), r_delta2), r_delta3);

  return rot;
}

gtsam::Vector2_ bspline_trans_2d(
  const gtsam::Vector2_& trans0,
  const gtsam::Vector2_& trans1,
  const gtsam::Vector2_& trans2,
  const gtsam::Vector2_& trans3,
  const gtsam::Double_& t) {
  const gtsam::Double_ t2 = t * t;
  const gtsam::Double_ t3 = t2 * t;

  // B-spline basis functions (cubic)
  const gtsam::Double_ beta1 = gtsam::Double_(5.0 / 6.0) + 3.0 / 6.0 * t - 3.0 / 6.0 * t2 + 1.0 / 6.0 * t3;
  const gtsam::Double_ beta2 = gtsam::Double_(1.0 / 6.0) + 3.0 / 6.0 * t + 3.0 / 6.0 * t2 - 2.0 / 6.0 * t3;
  const gtsam::Double_ beta3 = 1.0 / 6.0 * t3;

  const gtsam::Vector2_ t_delta1 = gtsam_points::scale<2>(beta1, gtsam::between(trans0, trans1));
  const gtsam::Vector2_ t_delta2 = gtsam_points::scale<2>(beta2, gtsam::between(trans1, trans2));
  const gtsam::Vector2_ t_delta3 = gtsam_points::scale<2>(beta3, gtsam::between(trans2, trans3));

  const gtsam::Vector2_ trans = gtsam::compose(gtsam::compose(gtsam::compose(trans0, t_delta1), t_delta2), t_delta3);

  return trans;
}

gtsam::Double_ bspline_angular_vel_2d(
  const gtsam::Rot2_& rot0,
  const gtsam::Rot2_& rot1,
  const gtsam::Rot2_& rot2,
  const gtsam::Rot2_& rot3,
  const gtsam::Double_& t,
  const double knot_interval) {
  const gtsam::Double_ t2 = t * t;
  const gtsam::Double_ t3 = t2 * t;

  // B-spline basis functions (cubic)
  const gtsam::Double_ beta1 = gtsam::Double_(5.0 / 6.0) + 3.0 / 6.0 * t - 3.0 / 6.0 * t2 + 1.0 / 6.0 * t3;
  const gtsam::Double_ beta2 = gtsam::Double_(1.0 / 6.0) + 3.0 / 6.0 * t + 3.0 / 6.0 * t2 - 2.0 / 6.0 * t3;
  const gtsam::Double_ beta3 = 1.0 / 6.0 * t3;

  // First derivatives of basis functions
  const gtsam::Double_ H_beta1_t = gtsam::Double_(3.0 / 6.0) - 3.0 / 6.0 * 2.0 * t + 1.0 / 6.0 * 3.0 * t2;
  const gtsam::Double_ H_beta2_t = gtsam::Double_(3.0 / 6.0) + 3.0 / 6.0 * 2.0 * t - 2.0 / 6.0 * 3.0 * t2;
  const gtsam::Double_ H_beta3_t = 1.0 / 6.0 * 3.0 * t2;

  // Log-map differences (angular differences)
  const gtsam::Double_ d1 = logmap_so2(rot0.between(rot1));
  const gtsam::Double_ d2 = logmap_so2(rot1.between(rot2));
  const gtsam::Double_ d3 = logmap_so2(rot2.between(rot3));

  // For SO(2), the composition is simpler (scalar addition)
  const gtsam::Double_ omega1 = H_beta1_t * d1;
  const gtsam::Double_ omega2 = omega1 + H_beta2_t * d2;
  const gtsam::Double_ omega3 = omega2 + H_beta3_t * d3;

  return (1.0 / knot_interval) * omega3;
}

gtsam::Vector2_ bspline_linear_vel_2d(
  const gtsam::Vector2_& trans0,
  const gtsam::Vector2_& trans1,
  const gtsam::Vector2_& trans2,
  const gtsam::Vector2_& trans3,
  const gtsam::Double_& t,
  const double knot_interval) {
  const gtsam::Double_ t2 = t * t;
  const gtsam::Double_ t3 = t2 * t;

  // B-spline basis functions (cubic)
  const gtsam::Double_ beta1 = gtsam::Double_(5.0 / 6.0) + 3.0 / 6.0 * t - 3.0 / 6.0 * t2 + 1.0 / 6.0 * t3;
  const gtsam::Double_ beta2 = gtsam::Double_(1.0 / 6.0) + 3.0 / 6.0 * t + 3.0 / 6.0 * t2 - 2.0 / 6.0 * t3;
  const gtsam::Double_ beta3 = 1.0 / 6.0 * t3;

  // First derivatives of basis functions
  const gtsam::Double_ H_beta1_t = gtsam::Double_(3.0 / 6.0) - 3.0 / 6.0 * 2.0 * t + 1.0 / 6.0 * 3.0 * t2;
  const gtsam::Double_ H_beta2_t = gtsam::Double_(3.0 / 6.0) + 3.0 / 6.0 * 2.0 * t - 2.0 / 6.0 * 3.0 * t2;
  const gtsam::Double_ H_beta3_t = 1.0 / 6.0 * 3.0 * t2;

  const gtsam::Vector2_ d1 = gtsam::between(trans0, trans1);
  const gtsam::Vector2_ d2 = gtsam::between(trans1, trans2);
  const gtsam::Vector2_ d3 = gtsam::between(trans2, trans3);

  const gtsam::Vector2_ omega1 = gtsam_points::scale<2>(H_beta1_t, d1);
  const gtsam::Vector2_ omega2 = gtsam::compose(omega1, gtsam_points::scale<2>(H_beta2_t, d2));
  const gtsam::Vector2_ omega3 = gtsam::compose(omega2, gtsam_points::scale<2>(H_beta3_t, d3));

  return (1.0 / knot_interval) * omega3;
}

gtsam::Vector2_ bspline_linear_acc_2d(
  const gtsam::Vector2_& trans0,
  const gtsam::Vector2_& trans1,
  const gtsam::Vector2_& trans2,
  const gtsam::Vector2_& trans3,
  const gtsam::Double_& t,
  const double knot_interval) {
  const gtsam::Double_ t2 = t * t;
  const gtsam::Double_ t3 = t2 * t;

  // B-spline basis functions (cubic)
  const gtsam::Double_ beta1 = gtsam::Double_(5.0 / 6.0) + 3.0 / 6.0 * t - 3.0 / 6.0 * t2 + 1.0 / 6.0 * t3;
  const gtsam::Double_ beta2 = gtsam::Double_(1.0 / 6.0) + 3.0 / 6.0 * t + 3.0 / 6.0 * t2 - 2.0 / 6.0 * t3;
  const gtsam::Double_ beta3 = 1.0 / 6.0 * t3;

  // First derivatives of basis functions
  const gtsam::Double_ H_beta1_t = gtsam::Double_(3.0 / 6.0) - 3.0 / 6.0 * 2.0 * t + 1.0 / 6.0 * 3.0 * t2;
  const gtsam::Double_ H_beta2_t = gtsam::Double_(3.0 / 6.0) + 3.0 / 6.0 * 2.0 * t - 2.0 / 6.0 * 3.0 * t2;
  const gtsam::Double_ H_beta3_t = 1.0 / 6.0 * 3.0 * t2;

  // Second derivatives of basis functions
  const gtsam::Double_ H2_beta1_t = gtsam::Double_(-3.0 / 6.0 * 2.0) + 1.0 / 6.0 * 3.0 * 2.0 * t;
  const gtsam::Double_ H2_beta2_t = gtsam::Double_(3.0 / 6.0 * 2.0) - 2.0 / 6.0 * 3.0 * 2.0 * t;
  const gtsam::Double_ H2_beta3_t = 1.0 / 6.0 * 3.0 * 2.0 * t;

  const gtsam::Vector2_ d1 = gtsam::between(trans0, trans1);
  const gtsam::Vector2_ d2 = gtsam::between(trans1, trans2);
  const gtsam::Vector2_ d3 = gtsam::between(trans2, trans3);

  const gtsam::Vector2_ omega1_ = gtsam_points::scale<2>(H2_beta1_t, d1);
  const gtsam::Vector2_ omega2_ = gtsam::compose(omega1_, gtsam_points::scale<2>(H2_beta2_t, d2));
  const gtsam::Vector2_ omega3_ = gtsam::compose(omega2_, gtsam_points::scale<2>(H2_beta3_t, d3));

  return (1.0 / (knot_interval * knot_interval)) * omega3_;
}

gtsam::Vector3_ bspline_imu_2d(
  const gtsam::Pose2_ pose0,
  const gtsam::Pose2_ pose1,
  const gtsam::Pose2_ pose2,
  const gtsam::Pose2_ pose3,
  const gtsam::Double_& t,
  const double knot_interval,
  const gtsam::Vector2& g) {
  const gtsam::Double_ t2 = t * t;
  const gtsam::Double_ t3 = t2 * t;

  // B-spline basis functions and their derivatives
  const gtsam::Double_ beta1 = gtsam::Double_(5.0 / 6.0) + 3.0 / 6.0 * t - 3.0 / 6.0 * t2 + 1.0 / 6.0 * t3;
  const gtsam::Double_ beta2 = gtsam::Double_(1.0 / 6.0) + 3.0 / 6.0 * t + 3.0 / 6.0 * t2 - 2.0 / 6.0 * t3;
  const gtsam::Double_ beta3 = 1.0 / 6.0 * t3;

  const gtsam::Double_ H_beta1_t = gtsam::Double_(3.0 / 6.0) - 3.0 / 6.0 * 2.0 * t + 1.0 / 6.0 * 3.0 * t2;
  const gtsam::Double_ H_beta2_t = gtsam::Double_(3.0 / 6.0) + 3.0 / 6.0 * 2.0 * t - 2.0 / 6.0 * 3.0 * t2;
  const gtsam::Double_ H_beta3_t = 1.0 / 6.0 * 3.0 * t2;

  const gtsam::Double_ H2_beta1_t = gtsam::Double_(-3.0 / 6.0 * 2.0) + 1.0 / 6.0 * 3.0 * 2.0 * t;
  const gtsam::Double_ H2_beta2_t = gtsam::Double_(3.0 / 6.0 * 2.0) - 2.0 / 6.0 * 3.0 * 2.0 * t;
  const gtsam::Double_ H2_beta3_t = 1.0 / 6.0 * 3.0 * 2.0 * t;

  // Rotation (SO(2))
  const gtsam::Rot2_ rot0 = gtsam::rotation(pose0);
  const gtsam::Rot2_ rot1 = gtsam::rotation(pose1);
  const gtsam::Rot2_ rot2 = gtsam::rotation(pose2);
  const gtsam::Rot2_ rot3 = gtsam::rotation(pose3);

  const gtsam::Double_ r_d1 = logmap_so2(rot0.between(rot1));
  const gtsam::Double_ r_d2 = logmap_so2(rot1.between(rot2));
  const gtsam::Double_ r_d3 = logmap_so2(rot2.between(rot3));

  const gtsam::Rot2_ r_A1 = expmap_so2(beta1 * r_d1);
  const gtsam::Rot2_ r_A2 = expmap_so2(beta2 * r_d2);
  const gtsam::Rot2_ r_A3 = expmap_so2(beta3 * r_d3);

  // Angular velocity (scalar for 2D)
  const gtsam::Double_ r_omega1 = H_beta1_t * r_d1;
  const gtsam::Double_ r_omega2 = r_omega1 + H_beta2_t * r_d2;
  const gtsam::Double_ r_omega3 = r_omega2 + H_beta3_t * r_d3;

  // Translation (R^2)
  const gtsam::Vector2_ trans0 = gtsam::translation(pose0);
  const gtsam::Vector2_ trans1 = gtsam::translation(pose1);
  const gtsam::Vector2_ trans2 = gtsam::translation(pose2);
  const gtsam::Vector2_ trans3 = gtsam::translation(pose3);

  const gtsam::Vector2_ t_d1 = gtsam::between(trans0, trans1);
  const gtsam::Vector2_ t_d2 = gtsam::between(trans1, trans2);
  const gtsam::Vector2_ t_d3 = gtsam::between(trans2, trans3);

  const gtsam::Vector2_ t_omega1_ = gtsam_points::scale<2>(H2_beta1_t, t_d1);
  const gtsam::Vector2_ t_omega2_ = gtsam::compose(t_omega1_, gtsam_points::scale<2>(H2_beta2_t, t_d2));
  const gtsam::Vector2_ t_omega3_ = gtsam::compose(t_omega2_, gtsam_points::scale<2>(H2_beta3_t, t_d3));

  // Transform to local coordinate frame
  const gtsam::Rot2_ rot = gtsam::compose(gtsam::compose(gtsam::compose(rot0, r_A1), r_A2), r_A3);

  const double inv_knot_interval = 1.0 / knot_interval;
  const double inv_knot_interval2 = inv_knot_interval * inv_knot_interval;

  const gtsam::Double_ angular_vel = inv_knot_interval * r_omega3;
  const gtsam::Vector2_ linear_acc = gtsam::unrotate(rot, inv_knot_interval2 * t_omega3_ + gtsam::Vector2_(g));

  // Return [ax_local, ay_local, omega_z]
  return gtsam_points::concatenate<2, 1>(linear_acc, angular_vel);
}

}  // namespace gtsam_points
