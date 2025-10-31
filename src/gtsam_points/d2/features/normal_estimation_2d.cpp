// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/features/normal_estimation_2d.hpp>

#include <iostream>
#include <Eigen/Eigen>
#include <gtsam_points/config.hpp>
#include <gtsam_points/util/parallelism.hpp>
#include <gtsam_points/d2/features/covariance_estimation_2d.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/parallel_for.h>
#endif

namespace gtsam_points {

std::vector<Eigen::Vector3d> estimate_normals_2d(const Eigen::Vector3d* points, const Eigen::Matrix3d* covs, int num_points, int num_threads) {
  std::vector<Eigen::Vector3d> normals(num_points, Eigen::Vector3d::Zero());

  const auto perpoint_task = [&](int i) {
    // Extract 2x2 covariance from upper-left of 3x3
    Eigen::Matrix2d cov_2d = covs[i].block<2, 2>(0, 0);

    // Compute eigenvalues and eigenvectors for 2D covariance
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig;
    eig.computeDirect(cov_2d);

    // Normal is the eigenvector corresponding to the smallest eigenvalue
    // (perpendicular to the dominant direction in 2D)
    Eigen::Vector2d normal_2d = eig.eigenvectors().col(0);

    // Store as homogeneous coordinate (nx, ny, 0)
    normals[i] << normal_2d, 0.0;

    // Orient normal towards the sensor origin (assuming origin at 0,0)
    // Flip if pointing away from origin
    Eigen::Vector2d pt_2d = points[i].head<2>();
    if (pt_2d.dot(normal_2d) > 0.0) {
      normals[i].head<2>() = -normal_2d;
    }
  };

  if (is_omp_default() || num_threads == 1) {
#pragma omp parallel for num_threads(num_threads)
    for (int i = 0; i < num_points; i++) {
      perpoint_task(i);
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, num_points, 64), [&](const tbb::blocked_range<int>& range) {
      for (int i = range.begin(); i < range.end(); i++) {
        perpoint_task(i);
      }
    });
#else
    std::cerr << "error : TBB is not enabled" << std::endl;
    abort();
#endif
  }

  return normals;
}

std::vector<Eigen::Vector3d> estimate_normals_2d(const Eigen::Vector3d* points, int num_points, int k_neighbors, int num_threads) {
  auto covs = estimate_covariances_2d(points, num_points, k_neighbors, num_threads);
  return estimate_normals_2d(points, covs.data(), num_points, num_threads);
}

std::vector<Eigen::Vector3d> estimate_normals_2d(const PointCloud2D& points, int k_neighbors, int num_threads) {
  if (points.has_covs()) {
    return gtsam_points::estimate_normals_2d(points.points, points.covs, points.num_points, num_threads);
  }
  return gtsam_points::estimate_normals_2d(points.points, points.num_points, k_neighbors, num_threads);
}

}  // namespace gtsam_points
