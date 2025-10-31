// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/factors/integrated_matching_cost_factor_2d.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/HessianFactor.h>

namespace gtsam_points {

IntegratedMatchingCostFactor2D::IntegratedMatchingCostFactor2D(gtsam::Key target_key, gtsam::Key source_key)
: gtsam::NonlinearFactor(gtsam::KeyVector{target_key, source_key}),
  is_binary(true),
  fixed_target_pose(Eigen::Isometry2d::Identity()) {}

IntegratedMatchingCostFactor2D::IntegratedMatchingCostFactor2D(const gtsam::Pose2& fixed_target_pose, gtsam::Key source_key)
: gtsam::NonlinearFactor(gtsam::KeyVector{source_key}),
  is_binary(false),
  fixed_target_pose(Eigen::Isometry2d::Identity()) {
  // Convert gtsam::Pose2 to Eigen::Isometry2d
  this->fixed_target_pose.linear() = fixed_target_pose.rotation().matrix();
  this->fixed_target_pose.translation() = fixed_target_pose.translation();
}

IntegratedMatchingCostFactor2D::~IntegratedMatchingCostFactor2D() {}

void IntegratedMatchingCostFactor2D::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << s << "IntegratedMatchingCostFactor2D";
  if (is_binary) {
    std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;
  } else {
    std::cout << "(fixed, " << keyFormatter(this->keys()[0]) << ")" << std::endl;
  }
}

double IntegratedMatchingCostFactor2D::error(const gtsam::Values& values) const {
  Eigen::Isometry2d delta = calc_delta(values);
  return evaluate(delta);
}

std::shared_ptr<gtsam::GaussianFactor> IntegratedMatchingCostFactor2D::linearize(const gtsam::Values& values) const {
  Eigen::Isometry2d delta = calc_delta(values);

  update_correspondences(delta);

  Eigen::Matrix<double, 3, 3> H_target, H_source, H_target_source;
  Eigen::Matrix<double, 3, 1> b_target, b_source;
  double error = evaluate(delta, &H_target, &H_source, &H_target_source, &b_target, &b_source);

  gtsam::HessianFactor::shared_ptr factor;

  if (is_binary) {
    factor.reset(new gtsam::HessianFactor(keys()[0], keys()[1], H_target, H_target_source, -b_target, H_source, -b_source, error));
  } else {
    factor.reset(new gtsam::HessianFactor(keys()[0], H_source, -b_source, error));
  }

  return factor;
}

Eigen::Isometry2d IntegratedMatchingCostFactor2D::calc_delta(const gtsam::Values& values) const {
  if (is_binary) {
    gtsam::Pose2 target_pose = values.at<gtsam::Pose2>(keys()[0]);
    gtsam::Pose2 source_pose = values.at<gtsam::Pose2>(keys()[1]);

    // Calculate relative pose: T_target_source = T_target^-1 * T_source
    gtsam::Pose2 delta_pose = target_pose.inverse() * source_pose;

    // Convert to Eigen::Isometry2d
    Eigen::Isometry2d delta = Eigen::Isometry2d::Identity();
    delta.linear() = delta_pose.rotation().matrix();
    delta.translation() = delta_pose.translation();
    return delta;
  } else {
    // Unary case: fixed target pose
    gtsam::Pose2 target_pose(gtsam::Rot2::fromCosSin(fixed_target_pose.linear()(0, 0), fixed_target_pose.linear()(1, 0)),
                             gtsam::Point2(fixed_target_pose.translation()));
    gtsam::Pose2 source_pose = values.at<gtsam::Pose2>(keys()[0]);

    gtsam::Pose2 delta_pose = target_pose.inverse() * source_pose;

    Eigen::Isometry2d delta = Eigen::Isometry2d::Identity();
    delta.linear() = delta_pose.rotation().matrix();
    delta.translation() = delta_pose.translation();
    return delta;
  }
}

size_t IntegratedMatchingCostFactor2D::memory_usage() const {
  return sizeof(*this);
}

}  // namespace gtsam_points
