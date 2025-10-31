// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <iostream>

namespace gtsam_points {

template <typename T, int D>
PointCloud2DCPU::PointCloud2DCPU(const Eigen::Matrix<T, D, 1>* points, int num_points) : PointCloud2D() {
  this->num_points = num_points;
  add_points(points, num_points);
}

template <typename T>
void PointCloud2DCPU::add_times(const T* times, int num_points) {
  if (this->num_points && this->num_points != num_points) {
    std::cerr << "warning: number of points mismatch" << std::endl;
    std::cerr << "       : num_points=" << this->num_points << " given=" << num_points << std::endl;
  }

  this->num_points = num_points;

  times_storage.reset(new std::vector<double>(num_points));
  std::copy(times, times + num_points, times_storage->begin());
  this->times = times_storage->data();
}

template <typename T, int D>
void PointCloud2DCPU::add_points(const Eigen::Matrix<T, D, 1>* points, int num_points) {
  static_assert(D == 2 || D == 3, "Point dimension must be 2 or 3");

  if (this->num_points && this->num_points != num_points) {
    std::cerr << "warning: number of points mismatch" << std::endl;
    std::cerr << "       : num_points=" << this->num_points << " given=" << num_points << std::endl;
  }

  this->num_points = num_points;

  points_storage.reset(new std::vector<Eigen::Vector3d>(num_points));

  if constexpr (D == 2) {
    // Convert 2D points to homogeneous coordinates (x, y, 1)
    for (int i = 0; i < num_points; i++) {
      (*points_storage)[i] << points[i].template cast<double>(), 1.0;
    }
  } else {
    // D == 3, already in homogeneous coordinates
    std::transform(points, points + num_points, points_storage->begin(),
      [](const Eigen::Matrix<T, 3, 1>& p) {
        return p.template cast<double>();
      });
  }

  this->points = points_storage->data();
}

template <typename T, int D>
void PointCloud2DCPU::add_normals(const Eigen::Matrix<T, D, 1>* normals, int num_points) {
  static_assert(D == 2 || D == 3, "Normal dimension must be 2 or 3");

  if (this->num_points && this->num_points != num_points) {
    std::cerr << "warning: number of points mismatch" << std::endl;
    std::cerr << "       : num_points=" << this->num_points << " given=" << num_points << std::endl;
  }

  this->num_points = num_points;

  normals_storage.reset(new std::vector<Eigen::Vector3d>(num_points));

  if constexpr (D == 2) {
    // Convert 2D normals to 3D format (nx, ny, 0)
    for (int i = 0; i < num_points; i++) {
      (*normals_storage)[i] << normals[i].template cast<double>(), 0.0;
    }
  } else {
    // D == 3, already in correct format
    std::transform(normals, normals + num_points, normals_storage->begin(),
      [](const Eigen::Matrix<T, 3, 1>& n) {
        return n.template cast<double>();
      });
  }

  this->normals = normals_storage->data();
}

template <typename T, int D>
void PointCloud2DCPU::add_covs(const Eigen::Matrix<T, D, D>* covs, int num_points) {
  static_assert(D == 2 || D == 3, "Covariance dimension must be 2 or 3");

  if (this->num_points && this->num_points != num_points) {
    std::cerr << "warning: number of points mismatch" << std::endl;
    std::cerr << "       : num_points=" << this->num_points << " given=" << num_points << std::endl;
  }

  this->num_points = num_points;

  covs_storage.reset(new std::vector<Eigen::Matrix3d>(num_points));

  if constexpr (D == 2) {
    // Expand 2x2 covariance to 3x3 with padding
    for (int i = 0; i < num_points; i++) {
      (*covs_storage)[i].setZero();
      (*covs_storage)[i].template topLeftCorner<2, 2>() = covs[i].template cast<double>();
    }
  } else {
    // D == 3, already in correct format
    std::transform(covs, covs + num_points, covs_storage->begin(),
      [](const Eigen::Matrix<T, 3, 3>& c) {
        return c.template cast<double>();
      });
  }

  this->covs = covs_storage->data();
}

template <typename T>
void PointCloud2DCPU::add_intensities(const T* intensities, int num_points) {
  if (this->num_points && this->num_points != num_points) {
    std::cerr << "warning: number of points mismatch" << std::endl;
    std::cerr << "       : num_points=" << this->num_points << " given=" << num_points << std::endl;
  }

  this->num_points = num_points;

  intensities_storage.reset(new std::vector<double>(num_points));
  std::copy(intensities, intensities + num_points, intensities_storage->begin());
  this->intensities = intensities_storage->data();
}

}  // namespace gtsam_points
