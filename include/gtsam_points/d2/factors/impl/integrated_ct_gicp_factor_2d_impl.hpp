// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#include <gtsam_points/d2/factors/integrated_ct_gicp_factor_2d.hpp>

#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/config.hpp>
#include <gtsam_points/d2/ann/nearest_neighbor_search_2d.hpp>
#include <gtsam_points/util/parallelism.hpp>
#include <gtsam_points/d2/factors/impl/scan_matching_reduction_2d.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/parallel_for.h>
#endif

namespace gtsam_points {

template <typename TargetFrame, typename SourceFrame>
IntegratedCT_GICPFactor2D_<TargetFrame, SourceFrame>::IntegratedCT_GICPFactor2D_(
  gtsam::Key source_t0_key,
  gtsam::Key source_t1_key,
  const std::shared_ptr<const TargetFrame>& target,
  const std::shared_ptr<const SourceFrame>& source,
  const std::shared_ptr<const NearestNeighborSearch2D>& target_tree)
: IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>(source_t0_key, source_t1_key, target, source, target_tree) {
  //
  if (!frame::has_points(*target) || !frame::has_covs(*target)) {
    std::cerr << "error: target frame doesn't have required attributes for ct_gicp_2d (needs points and covariances)" << std::endl;
    abort();
  }

  if (!frame::has_points(*source) || !frame::has_covs(*source)) {
    std::cerr << "error: source frame doesn't have required attributes for ct_gicp_2d (needs points and covariances)" << std::endl;
    abort();
  }
}

template <typename TargetFrame, typename SourceFrame>
IntegratedCT_GICPFactor2D_<TargetFrame, SourceFrame>::IntegratedCT_GICPFactor2D_(
  gtsam::Key source_t0_key,
  gtsam::Key source_t1_key,
  const std::shared_ptr<const TargetFrame>& target,
  const std::shared_ptr<const SourceFrame>& source)
: IntegratedCT_GICPFactor2D_(source_t0_key, source_t1_key, target, source, nullptr) {}

template <typename TargetFrame, typename SourceFrame>
IntegratedCT_GICPFactor2D_<TargetFrame, SourceFrame>::~IntegratedCT_GICPFactor2D_() {}

template <typename TargetFrame, typename SourceFrame>
void IntegratedCT_GICPFactor2D_<TargetFrame, SourceFrame>::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << s << "IntegratedCT_GICPFactor2D";
  std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;

  std::cout << "|target|=" << frame::size(*this->target) << "pts, |source|=" << frame::size(*this->source) << "pts" << std::endl;
  std::cout << "num_threads=" << this->num_threads << ", max_corr_dist=" << std::sqrt(this->max_correspondence_distance_sq) << std::endl;
}

template <typename TargetFrame, typename SourceFrame>
size_t IntegratedCT_GICPFactor2D_<TargetFrame, SourceFrame>::memory_usage() const {
  return IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::memory_usage() + sizeof(Eigen::Matrix3d) * mahalanobis.capacity();
}

template <typename TargetFrame, typename SourceFrame>
double IntegratedCT_GICPFactor2D_<TargetFrame, SourceFrame>::error(const gtsam::Values& values) const {
  this->update_poses(values);
  if (this->correspondences.size() != frame::size(*this->source)) {
    this->update_correspondences();
  }

  const auto perpoint_task = [&](
                               int i,
                               Eigen::Matrix<double, 3, 3>* H_target,
                               Eigen::Matrix<double, 3, 3>* H_source,
                               Eigen::Matrix<double, 3, 3>* H_target_source,
                               Eigen::Matrix<double, 3, 1>* b_target,
                               Eigen::Matrix<double, 3, 1>* b_source) {
    const long target_index = this->correspondences[i];
    if (target_index < 0) {
      return 0.0;
    }

    const int time_index = this->time_indices[i];
    const Eigen::Isometry2d pose(this->source_poses[time_index].matrix());

    const auto& source_pt = frame::point(*this->source, i);
    const auto& target_pt = frame::point(*this->target, target_index);

    // Transform source point to target frame (homogeneous coordinates)
    const Eigen::Vector3d transed_source_pt = pose * source_pt;
    const Eigen::Vector3d residual = transed_source_pt - target_pt;

    // Mahalanobis distance: error = residual^T * M * residual
    const double error = residual.transpose() * mahalanobis[i] * residual;

    return error;
  };

  if (is_omp_default() || this->num_threads == 1) {
    return scan_matching_reduce_omp_2d(perpoint_task, frame::size(*this->source), this->num_threads, nullptr, nullptr, nullptr, nullptr, nullptr);
  } else {
    return scan_matching_reduce_tbb_2d(perpoint_task, frame::size(*this->source), nullptr, nullptr, nullptr, nullptr, nullptr);
  }
}

template <typename TargetFrame, typename SourceFrame>
std::shared_ptr<gtsam::GaussianFactor> IntegratedCT_GICPFactor2D_<TargetFrame, SourceFrame>::linearize(const gtsam::Values& values) const {
  this->update_poses(values);
  this->update_correspondences();

  const auto perpoint_task = [&](
                               int i,
                               Eigen::Matrix<double, 3, 3>* H_00,
                               Eigen::Matrix<double, 3, 3>* H_11,
                               Eigen::Matrix<double, 3, 3>* H_01,
                               Eigen::Matrix<double, 3, 1>* b_0,
                               Eigen::Matrix<double, 3, 1>* b_1) {
    const long target_index = this->correspondences[i];
    if (target_index < 0) {
      return 0.0;
    }

    const int time_index = this->time_indices[i];

    const Eigen::Isometry2d pose(this->source_poses[time_index].matrix());
    const auto& H_pose_0 = this->pose_derivatives_t0[time_index];
    const auto& H_pose_1 = this->pose_derivatives_t1[time_index];

    const auto& source_pt = frame::point(*this->source, i);
    const auto& target_pt = frame::point(*this->target, target_index);

    // Jacobian of transformed point w.r.t. pose (3x3 for 2D)
    // For homogeneous 2D: transed_pt = [R * pt_2d + t; 1]
    // Jacobian w.r.t. [dx, dy, dtheta]: [I, -J*pt; 0, 0, 0] where J is 2D rotation derivative
    gtsam::Matrix33 H_transed_pose = gtsam::Matrix33::Zero();
    const double c = std::cos(pose.rotation().theta());
    const double s = std::sin(pose.rotation().theta());
    const double px = source_pt(0);
    const double py = source_pt(1);

    // Derivatives w.r.t. translation (x, y)
    H_transed_pose(0, 0) = 1.0;  // dx/dx
    H_transed_pose(1, 1) = 1.0;  // dy/dy

    // Derivatives w.r.t. rotation (theta)
    H_transed_pose(0, 2) = -s * px - c * py;  // dx/dtheta
    H_transed_pose(1, 2) = c * px - s * py;   // dy/dtheta

    const Eigen::Vector3d transed_source_pt = pose * source_pt;
    const auto& H_residual_pose = H_transed_pose;
    const Eigen::Vector3d residual = transed_source_pt - target_pt;

    // Chain rule with pose derivatives
    const gtsam::Matrix33 H_0 = H_residual_pose * H_pose_0;
    const gtsam::Matrix33 H_1 = H_residual_pose * H_pose_1;

    // Apply Mahalanobis distance
    const gtsam::Vector3 mahalanobis_residual = mahalanobis[i] * residual;
    const gtsam::Matrix33 H_0_mahalanobis = H_0.transpose() * mahalanobis[i];
    const gtsam::Matrix33 H_1_mahalanobis = H_1.transpose() * mahalanobis[i];

    const double error = residual.transpose() * mahalanobis_residual;
    *H_00 += H_0_mahalanobis * H_0;
    *H_11 += H_1_mahalanobis * H_1;
    *H_01 += H_0_mahalanobis * H_1;
    *b_0 += H_0.transpose() * mahalanobis_residual;
    *b_1 += H_1.transpose() * mahalanobis_residual;

    return error;
  };

  double error = 0.0;
  gtsam::Matrix3 H_00;
  gtsam::Matrix3 H_01;
  gtsam::Matrix3 H_11;
  gtsam::Vector3 b_0;
  gtsam::Vector3 b_1;

  if (is_omp_default() || this->num_threads == 1) {
    error = scan_matching_reduce_omp_2d(perpoint_task, frame::size(*this->source), this->num_threads, &H_00, &H_11, &H_01, &b_0, &b_1);
  } else {
    error = scan_matching_reduce_tbb_2d(perpoint_task, frame::size(*this->source), &H_00, &H_11, &H_01, &b_0, &b_1);
  }

  auto factor = gtsam::HessianFactor::shared_ptr(new gtsam::HessianFactor(this->keys_[0], this->keys_[1], H_00, H_01, -b_0, H_11, -b_1, error));
  return factor;
}

template <typename TargetFrame, typename SourceFrame>
void IntegratedCT_GICPFactor2D_<TargetFrame, SourceFrame>::update_correspondences() const {
  this->correspondences.resize(frame::size(*this->source));
  this->mahalanobis.resize(frame::size(*this->source));

  const auto perpoint_task = [&](int i) {
    const int time_index = this->time_indices[i];
    const Eigen::Matrix3d pose = this->source_poses[time_index].matrix();

    const auto& pt = frame::point(*this->source, i);
    const Eigen::Vector3d transed_pt = pose * pt;

    size_t k_index = -1;
    double k_sq_dist = std::numeric_limits<double>::max();
    size_t num_found = this->target_tree->knn_search(transed_pt.data(), 1, &k_index, &k_sq_dist, this->max_correspondence_distance_sq);

    if (num_found == 0 || k_sq_dist > this->max_correspondence_distance_sq) {
      this->correspondences[i] = -1;
      this->mahalanobis[i].setZero();
    } else {
      this->correspondences[i] = k_index;

      const long target_index = this->correspondences[i];
      const auto& cov_A = frame::cov(*this->source, i);
      const auto& cov_B = frame::cov(*this->target, target_index);

      // Compute Mahalanobis matrix: M = (cov_B + R*cov_A*R^T)^{-1}
      // For 2D, covariances are 3x3 but only the upper-left 2x2 is used
      const Eigen::Matrix3d RCR = (cov_B + pose * cov_A * pose.transpose());

      mahalanobis[i].setZero();
      // Only invert the 2x2 upper-left block (the 2D part)
      mahalanobis[i].block<2, 2>(0, 0) = RCR.block<2, 2>(0, 0).inverse();
    }
  };

  if (is_omp_default() || this->num_threads == 1) {
#pragma omp parallel for num_threads(this->num_threads) schedule(guided, 8)
    for (int i = 0; i < frame::size(*this->source); i++) {
      perpoint_task(i);
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, frame::size(*this->source), 8), [&](const tbb::blocked_range<int>& range) {
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

}  // namespace gtsam_points
