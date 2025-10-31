// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/types/point_cloud_2d.hpp>

#include <fstream>
#include <iostream>
#include <algorithm>
#include <boost/filesystem.hpp>

namespace gtsam_points {

bool PointCloud2D::has_times() const {
  return times != nullptr;
}

bool PointCloud2D::has_points() const {
  return points != nullptr;
}

bool PointCloud2D::has_normals() const {
  return normals != nullptr;
}

bool PointCloud2D::has_covs() const {
  return covs != nullptr;
}

bool PointCloud2D::has_intensities() const {
  return intensities != nullptr;
}

bool PointCloud2D::check_times() const {
  if (!times) {
    std::cerr << "warning: 2D point cloud doesn't have times" << std::endl;
  }
  return times != nullptr;
}

bool PointCloud2D::check_points() const {
  if (!points) {
    std::cerr << "warning: 2D point cloud doesn't have points" << std::endl;
  }
  return points != nullptr;
}

bool PointCloud2D::check_normals() const {
  if (!normals) {
    std::cerr << "warning: 2D point cloud doesn't have normals" << std::endl;
  }
  return normals != nullptr;
}

bool PointCloud2D::check_covs() const {
  if (!covs) {
    std::cerr << "warning: 2D point cloud doesn't have covariances" << std::endl;
  }
  return covs != nullptr;
}

bool PointCloud2D::check_intensities() const {
  if (!intensities) {
    std::cerr << "warning: 2D point cloud doesn't have intensities" << std::endl;
  }
  return intensities != nullptr;
}

namespace {
void write_binary(const std::string& filename, const void* data, size_t size) {
  std::ofstream ofs(filename, std::ios::binary);
  ofs.write(reinterpret_cast<const char*>(data), size);
}
}  // namespace

void PointCloud2D::save(const std::string& path) const {
  boost::filesystem::create_directories(path);

  if (times) {
    write_binary(path + "/times.bin", times, sizeof(double) * num_points);
  }

  if (points) {
    write_binary(path + "/points.bin", points, sizeof(Eigen::Vector3d) * num_points);
  }

  if (normals) {
    write_binary(path + "/normals.bin", normals, sizeof(Eigen::Vector3d) * num_points);
  }

  if (covs) {
    write_binary(path + "/covs.bin", covs, sizeof(Eigen::Matrix3d) * num_points);
  }

  if (intensities) {
    write_binary(path + "/intensities.bin", intensities, sizeof(double) * num_points);
  }

  for (const auto& attrib : aux_attributes) {
    const auto& name = attrib.first;
    const size_t elem_size = attrib.second.first;
    const void* data_ptr = attrib.second.second;
    write_binary(path + "/aux_" + name + ".bin", data_ptr, elem_size * num_points);
  }
}

void PointCloud2D::save_compact(const std::string& path) const {
  boost::filesystem::create_directories(path);

  if (times) {
    std::vector<float> times_f(num_points);
    std::copy(times, times + num_points, times_f.begin());
    write_binary(path + "/times_compact.bin", times_f.data(), sizeof(float) * num_points);
  }

  if (points) {
    // Save only (x, y) without homogeneous coordinate
    std::vector<Eigen::Vector2f> points_f(num_points);
    std::transform(points, points + num_points, points_f.begin(),
      [](const Eigen::Vector3d& p) { return p.head<2>().cast<float>(); });
    write_binary(path + "/points_compact.bin", points_f.data(), sizeof(Eigen::Vector2f) * num_points);
  }

  if (normals) {
    // Save only (nx, ny) without padding
    std::vector<Eigen::Vector2f> normals_f(num_points);
    std::transform(normals, normals + num_points, normals_f.begin(),
      [](const Eigen::Vector3d& n) { return n.head<2>().cast<float>(); });
    write_binary(path + "/normals_compact.bin", normals_f.data(), sizeof(Eigen::Vector2f) * num_points);
  }

  if (covs) {
    // Save only 2x2 covariance (upper-left block) as 3 floats: cov(0,0), cov(0,1), cov(1,1)
    std::vector<Eigen::Vector3f> covs_f(num_points);
    std::transform(covs, covs + num_points, covs_f.begin(),
      [](const Eigen::Matrix3d& cov) {
        return Eigen::Vector3f(
          static_cast<float>(cov(0, 0)),
          static_cast<float>(cov(0, 1)),
          static_cast<float>(cov(1, 1))
        );
      });
    write_binary(path + "/covs_compact.bin", covs_f.data(), sizeof(Eigen::Vector3f) * num_points);
  }

  if (intensities) {
    std::vector<float> intensities_f(num_points);
    std::copy(intensities, intensities + num_points, intensities_f.begin());
    write_binary(path + "/intensities_compact.bin", intensities_f.data(), sizeof(float) * num_points);
  }

  for (const auto& attrib : aux_attributes) {
    const auto& name = attrib.first;
    const size_t elem_size = attrib.second.first;
    const void* data_ptr = attrib.second.second;
    write_binary(path + "/aux_" + name + "_compact.bin", data_ptr, elem_size * num_points);
  }
}

}  // namespace gtsam_points
