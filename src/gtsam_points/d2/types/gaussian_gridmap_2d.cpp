// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/types/gaussian_gridmap_2d.hpp>
#include <gtsam_points/d2/types/frame_traits_2d.hpp>

namespace gtsam_points {

// GaussianGridCell2D implementation
void GaussianGridCell2D::add(const PointCloud2D& points, size_t i) {
  if (!points.check_points()) {
    return;
  }

  const Eigen::Vector3d& pt = points.points[i];
  mean += pt;
  cov += pt * pt.transpose();
  num_points++;
}

void GaussianGridCell2D::finalize() {
  if (num_points == 0 || finalized) {
    return;
  }

  mean /= num_points;
  cov = (cov / num_points) - mean * mean.transpose();

  // Regularize covariance (2x2 part)
  Eigen::Matrix2d cov_2d = cov.block<2, 2>(0, 0);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov_2d);

  // Regularize eigenvalues to avoid singularity
  Eigen::Vector2d eigen_values = eig.eigenvalues();
  eigen_values = eigen_values.array().max(1e-3);  // Min eigenvalue = 1e-3

  cov.setZero();
  cov.block<2, 2>(0, 0) = eig.eigenvectors() * eigen_values.asDiagonal() * eig.eigenvectors().transpose();

  finalized = true;
}

// GaussianGridMap2D implementation
GaussianGridMap2D::GaussianGridMap2D(double resolution) : resolution(resolution) {}

GaussianGridMap2D::~GaussianGridMap2D() {}

Eigen::Vector2i GaussianGridMap2D::grid_coord(const Eigen::Vector3d& x) const {
  Eigen::Vector2i coord;
  coord.x() = static_cast<int>(std::floor(x.x() / resolution));
  coord.y() = static_cast<int>(std::floor(x.y() / resolution));
  return coord;
}

const GaussianGridCell2D* GaussianGridMap2D::lookup_cell(const Eigen::Vector2i& coord) const {
  auto it = cells.find(coord);
  if (it == cells.end()) {
    return nullptr;
  }
  return &(it->second);
}

void GaussianGridMap2D::insert(const PointCloud2D& frame) {
  if (!frame.check_points()) {
    return;
  }

  // First pass: accumulate points into cells
  for (size_t i = 0; i < frame.num_points; i++) {
    Eigen::Vector2i coord = grid_coord(frame.points[i]);
    cells[coord].add(frame, i);
  }

  // Second pass: finalize all cells
  for (auto& pair : cells) {
    pair.second.finalize();
  }
}

}  // namespace gtsam_points
