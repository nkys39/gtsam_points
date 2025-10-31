// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/features/covariance_estimation_2d.hpp>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <gtsam_points/config.hpp>
#include <gtsam_points/d2/ann/kdtree_2d.hpp>
#include <gtsam_points/util/parallelism.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/parallel_for.h>
#endif

namespace gtsam_points {

std::vector<Eigen::Matrix3d> estimate_covariances_2d(const Eigen::Vector3d* points, int num_points, const CovarianceEstimationParams2D& params) {
  KdTree2D tree(points, num_points, params.num_threads);
  std::vector<Eigen::Matrix3d> covs(num_points);

  const auto perpoint_task = [&](int i) {
    std::vector<size_t> k_indices(params.k_neighbors);
    std::vector<double> k_sq_dists(params.k_neighbors);

    // Search using x,y coordinates only
    size_t num_found = tree.knn_search(points[i].data(), params.k_neighbors, &k_indices[0], &k_sq_dists[0]);

    if (num_found < params.k_neighbors) {
      std::cerr << "warning: fewer than k neighbors found for point " << i << std::endl;
      covs[i].setIdentity();
      return;
    }

    // Compute mean and covariance in 2D (x, y only)
    Eigen::Vector2d sum_points = Eigen::Vector2d::Zero();
    Eigen::Matrix2d sum_covs = Eigen::Matrix2d::Zero();

    for (int j = 0; j < num_found; j++) {
      const Eigen::Vector2d pt = points[k_indices[j]].head<2>();
      sum_points += pt;
      sum_covs += pt * pt.transpose();
    }

    Eigen::Vector2d mean = sum_points / num_found;
    Eigen::Matrix2d cov = (sum_covs - mean * sum_points.transpose()) / num_found;

    // Store in 3x3 format (upper-left 2x2 for actual covariance)
    covs[i].setZero();

    switch (params.regularization_method) {
      default:
      case CovarianceEstimationParams2D::NONE:
        covs[i].block<2, 2>(0, 0) = cov;
        break;

      case CovarianceEstimationParams2D::EIG: {
        // Eigenvalue decomposition for 2D covariance
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig;
        eig.computeDirect(cov);

        // Regularize with specified eigenvalues
        covs[i].block<2, 2>(0, 0) = eig.eigenvectors() * params.eigen_values.asDiagonal() * eig.eigenvectors().inverse();
      } break;
    }

    // Homogeneous coordinate part remains zero
    covs[i](2, 2) = 0.0;
  };

  if (is_omp_default() || params.num_threads == 1) {
#pragma omp parallel for num_threads(params.num_threads) schedule(guided, 8)
    for (int i = 0; i < num_points; i++) {
      perpoint_task(i);
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, num_points, 8), [&](const tbb::blocked_range<int>& range) {
      for (int i = range.begin(); i < range.end(); i++) {
        perpoint_task(i);
      }
    });
#else
    std::cerr << "error: TBB is not available" << std::endl;
    abort();
#endif
  }

  return covs;
}

std::vector<Eigen::Matrix3d> estimate_covariances_2d(const Eigen::Vector3d* points, int num_points, int k_neighbors, int num_threads) {
  CovarianceEstimationParams2D params;
  params.k_neighbors = k_neighbors;
  params.num_threads = num_threads;
  return estimate_covariances_2d(points, num_points, params);
}

std::vector<Eigen::Matrix3d>
estimate_covariances_2d(const Eigen::Vector3d* points, int num_points, int k_neighbors, const Eigen::Vector2d& eigen_values, int num_threads) {
  CovarianceEstimationParams2D params;
  params.k_neighbors = k_neighbors;
  params.eigen_values = eigen_values;
  params.num_threads = num_threads;
  return estimate_covariances_2d(points, num_points, params);
}

std::vector<Eigen::Matrix3d> estimate_covariances_2d(const PointCloud2D& points, int k_neighbors, int num_threads) {
  return estimate_covariances_2d(points.points, points.num_points, k_neighbors, num_threads);
}

}  // namespace gtsam_points
