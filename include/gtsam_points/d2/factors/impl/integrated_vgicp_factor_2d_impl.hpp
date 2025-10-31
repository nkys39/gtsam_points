// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/config.hpp>
#include <gtsam_points/d2/util/compact_2d.hpp>
#include <gtsam_points/util/parallelism.hpp>
#include <gtsam_points/d2/types/frame_traits_2d.hpp>
#include <gtsam_points/d2/factors/impl/scan_matching_reduction_2d.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/parallel_for.h>
#endif

namespace gtsam_points {

template <typename SourceFrame>
IntegratedVGICPFactor2D_<SourceFrame>::IntegratedVGICPFactor2D_(
  gtsam::Key target_key,
  gtsam::Key source_key,
  const GaussianGridMap2D::ConstPtr& target_gridmap,
  const std::shared_ptr<const SourceFrame>& source)
: gtsam_points::IntegratedMatchingCostFactor2D(target_key, source_key),
  num_threads(1),
  mahalanobis_cache_mode(FusedCovCacheMode2D::FULL),
  target_gridmap(target_gridmap),
  source(source) {
  //
  if (!target_gridmap->has_points() || !target_gridmap->has_covs()) {
    std::cerr << "error: target gridmap doesn't have required attributes for 2D vgicp" << std::endl;
    abort();
  }

  if (!frame::has_points(*source) || !frame::has_covs(*source)) {
    std::cerr << "error: source frame doesn't have required attributes for 2D vgicp" << std::endl;
    abort();
  }
}

template <typename SourceFrame>
IntegratedVGICPFactor2D_<SourceFrame>::IntegratedVGICPFactor2D_(
  const gtsam::Pose2& fixed_target_pose,
  gtsam::Key source_key,
  const GaussianGridMap2D::ConstPtr& target_gridmap,
  const std::shared_ptr<const SourceFrame>& source)
: gtsam_points::IntegratedMatchingCostFactor2D(fixed_target_pose, source_key),
  num_threads(1),
  mahalanobis_cache_mode(FusedCovCacheMode2D::FULL),
  target_gridmap(target_gridmap),
  source(source) {
  //
  if (!target_gridmap->has_points() || !target_gridmap->has_covs()) {
    std::cerr << "error: target gridmap doesn't have required attributes for 2D vgicp" << std::endl;
    abort();
  }

  if (!frame::has_points(*source) || !frame::has_covs(*source)) {
    std::cerr << "error: source frame doesn't have required attributes for 2D vgicp" << std::endl;
    abort();
  }
}

template <typename SourceFrame>
IntegratedVGICPFactor2D_<SourceFrame>::~IntegratedVGICPFactor2D_() {}

template <typename SourceFrame>
void IntegratedVGICPFactor2D_<SourceFrame>::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << s << "IntegratedVGICPFactor2D";
  if (is_binary) {
    std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;
  } else {
    std::cout << "(fixed, " << keyFormatter(this->keys()[0]) << ")" << std::endl;
  }

  std::cout << "|target_cells|=" << target_gridmap->num_cells() << ", |source|=" << frame::size(*source) << "pts" << std::endl;
  std::cout << "num_threads=" << num_threads << ", cache_mode=" << static_cast<int>(mahalanobis_cache_mode) << std::endl;
}

template <typename SourceFrame>
size_t IntegratedVGICPFactor2D_<SourceFrame>::memory_usage() const {
  return sizeof(*this) + sizeof(const GaussianGridCell2D*) * correspondences.capacity() +
         sizeof(Eigen::Matrix3d) * mahalanobis_full.capacity() + sizeof(Eigen::Vector3f) * mahalanobis_compact.capacity();
}

template <typename SourceFrame>
void IntegratedVGICPFactor2D_<SourceFrame>::update_correspondences(const Eigen::Isometry2d& delta) const {
  linearization_point = delta;

  correspondences.resize(frame::size(*source));

  switch (mahalanobis_cache_mode) {
    case FusedCovCacheMode2D::FULL:
      mahalanobis_full.resize(frame::size(*source));
      break;
    case FusedCovCacheMode2D::COMPACT:
      mahalanobis_compact.resize(frame::size(*source));
      break;
    case FusedCovCacheMode2D::NONE:
      break;
  }

  const auto perpoint_task = [&](int i) {
    // Transform source point to target frame
    Eigen::Vector3d pt = delta * frame::point(*source, i);

    // Lookup grid cell
    Eigen::Vector2i coord = target_gridmap->grid_coord(pt);
    const GaussianGridCell2D* cell = target_gridmap->lookup_cell(coord);

    correspondences[i] = cell;

    if (!cell) {
      // No correspondence found
      switch (mahalanobis_cache_mode) {
        case FusedCovCacheMode2D::FULL:
          mahalanobis_full[i].setZero();
          break;
        case FusedCovCacheMode2D::COMPACT:
          mahalanobis_compact[i].setZero();
          break;
        case FusedCovCacheMode2D::NONE:
          break;
      }
      return;
    }

    // Compute fused covariance
    const auto& source_cov = frame::cov(*source, i);
    const auto& target_cov = cell->cov;

    Eigen::Matrix2d target_cov_2d = target_cov.block<2, 2>(0, 0);
    Eigen::Matrix2d source_cov_2d = source_cov.block<2, 2>(0, 0);

    Eigen::Matrix2d R = delta.linear();
    Eigen::Matrix2d RCR = target_cov_2d + R * source_cov_2d * R.transpose();

    switch (mahalanobis_cache_mode) {
      case FusedCovCacheMode2D::FULL:
        mahalanobis_full[i].setZero();
        mahalanobis_full[i].block<2, 2>(0, 0) = RCR.inverse();
        break;
      case FusedCovCacheMode2D::COMPACT:
        mahalanobis_compact[i] = compact_cov_2d(RCR.inverse());
        break;
      case FusedCovCacheMode2D::NONE:
        break;
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

template <typename SourceFrame>
double IntegratedVGICPFactor2D_<SourceFrame>::evaluate(
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
    const GaussianGridCell2D* cell = correspondences[i];
    if (!cell) {
      return 0.0;
    }

    const auto& mean_A = frame::point(*source, i);
    const auto& cov_A = frame::cov(*source, i);
    const auto& mean_B = cell->mean;
    const auto& cov_B = cell->cov;

    const Eigen::Vector3d transed_mean_A = delta * mean_A;
    const Eigen::Vector2d residual = mean_B.head<2>() - transed_mean_A.head<2>();

    // Compute mahalanobis matrix (2x2)
    Eigen::Matrix2d mahalanobis;
    switch (mahalanobis_cache_mode) {
      case FusedCovCacheMode2D::FULL:
        mahalanobis = mahalanobis_full[i].block<2, 2>(0, 0);
        break;
      case FusedCovCacheMode2D::COMPACT: {
        Eigen::Matrix3d maha_3d = uncompact_cov_2d(mahalanobis_compact[i]);
        mahalanobis = maha_3d.block<2, 2>(0, 0);
      } break;
      case FusedCovCacheMode2D::NONE: {
        const auto& delta_l = linearization_point;
        Eigen::Matrix2d target_cov_2d = cov_B.block<2, 2>(0, 0);
        Eigen::Matrix2d source_cov_2d = cov_A.block<2, 2>(0, 0);
        Eigen::Matrix2d R = delta_l.linear();
        Eigen::Matrix2d RCR = target_cov_2d + R * source_cov_2d * R.transpose();
        mahalanobis = RCR.inverse();
      } break;
    }

    const double error = residual.transpose() * mahalanobis * residual;
    if (H_target == nullptr) {
      return error;
    }

    // Jacobians for 2D SE(2): [θ, x, y]
    const double theta = std::atan2(delta.linear()(1, 0), delta.linear()(0, 0));
    const double cos_theta = delta.linear()(0, 0);
    const double sin_theta = delta.linear()(1, 0);

    const double px = mean_A(0);
    const double py = mean_A(1);

    // Jacobian w.r.t. target pose
    Eigen::Matrix<double, 2, 3> J_target;
    J_target(0, 0) = -(-sin_theta * px - cos_theta * py);
    J_target(1, 0) = -(cos_theta * px - sin_theta * py);
    J_target.block<2, 2>(0, 1) = Eigen::Matrix2d::Identity();

    // Jacobian w.r.t. source pose
    Eigen::Matrix<double, 2, 3> J_source;
    J_source(0, 0) = -sin_theta * px - cos_theta * py;
    J_source(1, 0) = cos_theta * px - sin_theta * py;
    J_source.block<2, 2>(0, 1) = -delta.linear();

    // Weight by mahalanobis
    Eigen::Matrix<double, 3, 2> J_target_mahalanobis = J_target.transpose() * mahalanobis;
    Eigen::Matrix<double, 3, 2> J_source_mahalanobis = J_source.transpose() * mahalanobis;

    *H_target += J_target_mahalanobis * J_target;
    *H_source += J_source_mahalanobis * J_source;
    *H_target_source += J_target_mahalanobis * J_source;
    *b_target += J_target_mahalanobis * residual;
    *b_source += J_source_mahalanobis * residual;

    return error;
  };

  if (is_omp_default() || num_threads == 1) {
    return scan_matching_reduce_omp_2d(perpoint_task, frame::size(*source), num_threads, H_target, H_source, H_target_source, b_target, b_source);
  } else {
    return scan_matching_reduce_tbb_2d(perpoint_task, frame::size(*source), H_target, H_source, H_target_source, b_target, b_source);
  }
}

}  // namespace gtsam_points
