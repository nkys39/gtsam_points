// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/factors/integrated_icp_factor_2d.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/config.hpp>
#include <gtsam_points/d2/ann/kdtree_2d.hpp>
#include <gtsam_points/d2/types/frame_traits_2d.hpp>
#include <gtsam_points/util/parallelism.hpp>
#include <gtsam_points/d2/factors/impl/scan_matching_reduction_2d.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/parallel_for.h>
#endif

namespace gtsam_points {

template <typename TargetFrame, typename SourceFrame>
IntegratedICPFactor2D_<TargetFrame, SourceFrame>::IntegratedICPFactor2D_(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const std::shared_ptr<const TargetFrame>& target,
  const std::shared_ptr<const SourceFrame>& source,
  const std::shared_ptr<const NearestNeighborSearch2D>& target_tree,
  bool use_point_to_line)
: gtsam_points::IntegratedMatchingCostFactor2D(target_key, source_key),
  num_threads(1),
  max_correspondence_distance_sq(1.0),
  use_point_to_line(use_point_to_line),
  correspondence_update_tolerance_rot(0.0),
  correspondence_update_tolerance_trans(0.0),
  target(target),
  source(source) {
  //
  if (!frame::has_points(*target) || (use_point_to_line && !frame::has_normals(*target))) {
    std::cerr << "error: target frame doesn't have required attributes for 2D icp" << std::endl;
    abort();
  }

  if (!frame::has_points(*source)) {
    std::cerr << "error: source frame doesn't have required attributes for 2D icp" << std::endl;
    abort();
  }

  if (target_tree) {
    this->target_tree = target_tree;
  } else {
    // Auto-create KdTree2D if not provided
    this->target_tree.reset(new KdTree2D(frame::points_ptr(*target), frame::size(*target)));
  }
}

template <typename TargetFrame, typename SourceFrame>
IntegratedICPFactor2D_<TargetFrame, SourceFrame>::IntegratedICPFactor2D_(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const std::shared_ptr<const TargetFrame>& target,
  const std::shared_ptr<const SourceFrame>& source,
  bool use_point_to_line)
: gtsam_points::IntegratedICPFactor2D_<TargetFrame, SourceFrame>(target_key, source_key, target, source, nullptr, use_point_to_line) {}

template <typename TargetFrame, typename SourceFrame>
IntegratedICPFactor2D_<TargetFrame, SourceFrame>::IntegratedICPFactor2D_(
  const gtsam::Pose2& fixed_target_pose,
  gtsam::Key source_key,
  const std::shared_ptr<const TargetFrame>& target,
  const std::shared_ptr<const SourceFrame>& source,
  const std::shared_ptr<const NearestNeighborSearch2D>& target_tree,
  bool use_point_to_line)
: gtsam_points::IntegratedMatchingCostFactor2D(fixed_target_pose, source_key),
  num_threads(1),
  max_correspondence_distance_sq(1.0),
  use_point_to_line(use_point_to_line),
  correspondence_update_tolerance_rot(0.0),
  correspondence_update_tolerance_trans(0.0),
  target(target),
  source(source) {
  //
  if (!frame::has_points(*target) || (use_point_to_line && !frame::has_normals(*target))) {
    std::cerr << "error: target frame doesn't have required attributes for 2D icp" << std::endl;
    abort();
  }

  if (!frame::has_points(*source)) {
    std::cerr << "error: source frame doesn't have required attributes for 2D icp" << std::endl;
    abort();
  }

  if (target_tree) {
    this->target_tree = target_tree;
  } else {
    this->target_tree.reset(new KdTree2D(frame::points_ptr(*target), frame::size(*target)));
  }
}

template <typename TargetFrame, typename SourceFrame>
IntegratedICPFactor2D_<TargetFrame, SourceFrame>::IntegratedICPFactor2D_(
  const gtsam::Pose2& fixed_target_pose,
  gtsam::Key source_key,
  const std::shared_ptr<const TargetFrame>& target,
  const std::shared_ptr<const SourceFrame>& source,
  bool use_point_to_line)
: gtsam_points::IntegratedICPFactor2D_<TargetFrame, SourceFrame>(fixed_target_pose, source_key, target, source, nullptr, use_point_to_line) {}

template <typename TargetFrame, typename SourceFrame>
IntegratedICPFactor2D_<TargetFrame, SourceFrame>::~IntegratedICPFactor2D_() {}

template <typename TargetFrame, typename SourceFrame>
void IntegratedICPFactor2D_<TargetFrame, SourceFrame>::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << s << "IntegratedICPFactor2D";
  if (is_binary) {
    std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;
  } else {
    std::cout << "(fixed, " << keyFormatter(this->keys()[0]) << ")" << std::endl;
  }

  std::cout << "|target|=" << frame::size(*target) << "pts, |source|=" << frame::size(*source) << "pts" << std::endl;
  std::cout << "num_threads=" << num_threads << ", max_corr_dist=" << std::sqrt(max_correspondence_distance_sq) << std::endl;
}

template <typename TargetFrame, typename SourceFrame>
size_t IntegratedICPFactor2D_<TargetFrame, SourceFrame>::memory_usage() const {
  return sizeof(*this) + sizeof(long) * correspondences.capacity();
}

template <typename TargetFrame, typename SourceFrame>
void IntegratedICPFactor2D_<TargetFrame, SourceFrame>::update_correspondences(const Eigen::Isometry2d& delta) const {
  bool do_update = true;
  if (correspondences.size() == frame::size(*source) && (correspondence_update_tolerance_trans > 0.0 || correspondence_update_tolerance_rot > 0.0)) {
    Eigen::Isometry2d diff = delta.inverse() * last_correspondence_point;
    double diff_rot = std::abs(std::atan2(diff.linear()(1, 0), diff.linear()(0, 0)));
    double diff_trans = diff.translation().norm();
    if (diff_rot < correspondence_update_tolerance_rot && diff_trans < correspondence_update_tolerance_trans) {
      do_update = false;
    }
  }

  if (!do_update) {
    return;
  }

  last_correspondence_point = delta;
  correspondences.resize(frame::size(*source));

  const auto perpoint_task = [&](int i) {
    // Transform source point to target frame
    Eigen::Vector3d pt = delta * frame::point(*source, i);

    size_t k_index = -1;
    double k_sq_dist = -1;
    // Search using only x,y coordinates
    size_t num_found = target_tree->knn_search(pt.data(), 1, &k_index, &k_sq_dist, max_correspondence_distance_sq);

    if (num_found == 0 || k_sq_dist > max_correspondence_distance_sq) {
      correspondences[i] = -1;
    } else {
      correspondences[i] = k_index;
    }
  };

  if (is_omp_default() || num_threads == 1) {
#pragma omp parallel for num_threads(num_threads) schedule(guided, 8)
    for (int i = 0; i < frame::size(*source); i++) {
      perpoint_task(i);
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, frame::size(*source), 8), [&](const tbb::blocked_range<int>& range) {
      for (int i = range.begin(); i < range.end(); i++) {
        perpoint_task(i);
      }
    });
#else
    std::cerr << "error: TBB is not available" << std::endl;
    abort();
#endif
  }
}

template <typename TargetFrame, typename SourceFrame>
double IntegratedICPFactor2D_<TargetFrame, SourceFrame>::evaluate(
  const Eigen::Isometry2d& delta,
  Eigen::Matrix<double, 3, 3>* H_target,
  Eigen::Matrix<double, 3, 3>* H_source,
  Eigen::Matrix<double, 3, 3>* H_target_source,
  Eigen::Matrix<double, 3, 1>* b_target,
  Eigen::Matrix<double, 3, 1>* b_source) const {
  //
  if (correspondences.size() != frame::size(*source)) {
    update_correspondences(delta);
  }

  const auto perpoint_task = [&](
                               int i,
                               Eigen::Matrix<double, 3, 3>* H_target,
                               Eigen::Matrix<double, 3, 3>* H_source,
                               Eigen::Matrix<double, 3, 3>* H_target_source,
                               Eigen::Matrix<double, 3, 1>* b_target,
                               Eigen::Matrix<double, 3, 1>* b_source) {
    long target_index = correspondences[i];
    if (target_index < 0) {
      return 0.0;
    }

    const auto& mean_A = frame::point(*source, i);  // Source point (homogeneous 2D: x, y, 1)
    const auto& mean_B = frame::point(*target, target_index);  // Target point

    const Eigen::Vector3d transed_mean_A = delta * mean_A;
    Eigen::Vector2d residual = mean_B.head<2>() - transed_mean_A.head<2>();  // 2D residual (x, y only)

    if (use_point_to_line) {
      const auto& normal_B = frame::normal(*target, target_index);
      // Point-to-line distance: project residual onto normal
      double dist = residual.dot(normal_B.head<2>());
      residual = dist * normal_B.head<2>();
    }

    const double error = residual.squaredNorm();
    if (H_target == nullptr) {
      return error;
    }

    // Jacobians for 2D SE(2): [θ, x, y]
    // For point p = [x, y]^T and pose [θ, tx, ty]:
    // Transformed point: R(θ) * p + t
    // where R(θ) = [cos(θ) -sin(θ); sin(θ) cos(θ)]

    const double theta = std::atan2(delta.linear()(1, 0), delta.linear()(0, 0));
    const double cos_theta = delta.linear()(0, 0);
    const double sin_theta = delta.linear()(1, 0);

    const double px = mean_A(0);
    const double py = mean_A(1);

    // Jacobian w.r.t. target pose
    Eigen::Matrix<double, 2, 3> J_target;
    // d(R * p) / dθ = [-sin(θ) -cos(θ); cos(θ) -sin(θ)] * p
    J_target(0, 0) = -(-sin_theta * px - cos_theta * py);  // d/dθ of (cos*px - sin*py)
    J_target(1, 0) = -(cos_theta * px - sin_theta * py);   // d/dθ of (sin*px + cos*py)
    J_target.block<2, 2>(0, 1) = Eigen::Matrix2d::Identity();  // Translation part

    // Jacobian w.r.t. source pose
    Eigen::Matrix<double, 2, 3> J_source;
    J_source(0, 0) = -sin_theta * px - cos_theta * py;  // d/dθ
    J_source(1, 0) = cos_theta * px - sin_theta * py;
    J_source.block<2, 2>(0, 1) = -delta.linear();  // Rotation applied

    if (use_point_to_line) {
      const auto& normal_B = frame::normal(*target, target_index);
      const Eigen::Vector2d n = normal_B.head<2>();
      // Project Jacobians onto normal direction
      J_target = (n * n.transpose()) * J_target;
      J_source = (n * n.transpose()) * J_source;
    }

    *H_target += J_target.transpose() * J_target;
    *H_source += J_source.transpose() * J_source;
    *H_target_source += J_target.transpose() * J_source;
    *b_target += J_target.transpose() * residual;
    *b_source += J_source.transpose() * residual;

    return error;
  };

  if (is_omp_default() || num_threads == 1) {
    return scan_matching_reduce_omp_2d(perpoint_task, frame::size(*source), num_threads, H_target, H_source, H_target_source, b_target, b_source);
  } else {
    return scan_matching_reduce_tbb_2d(perpoint_task, frame::size(*source), H_target, H_source, H_target_source, b_target, b_source);
  }
}

}  // namespace gtsam_points
