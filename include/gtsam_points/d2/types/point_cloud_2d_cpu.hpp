// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <gtsam_points/d2/types/point_cloud_2d.hpp>

namespace gtsam_points {

/**
 * @brief 2D Point cloud frame on CPU memory with automatic memory management
 */
struct PointCloud2DCPU : public PointCloud2D {
public:
  using Ptr = std::shared_ptr<PointCloud2DCPU>;
  using ConstPtr = std::shared_ptr<const PointCloud2DCPU>;

  PointCloud2DCPU();
  ~PointCloud2DCPU();

  /**
   * @brief Constructor
   * @param points     Pointer to 2D point data
   * @param num_points Number of points
   */
  template <typename T, int D>
  PointCloud2DCPU(const Eigen::Matrix<T, D, 1>* points, int num_points);

  /**
   * @brief Constructor from vector
   * @param points  2D Points vector
   */
  template <typename T, int D, typename Alloc>
  PointCloud2DCPU(const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& points)
  : PointCloud2DCPU(points.data(), points.size()) {}

  // Forbid shallow copy
  PointCloud2DCPU(const PointCloud2DCPU& points) = delete;
  PointCloud2DCPU& operator=(PointCloud2DCPU const&) = delete;

  /// Deep copy
  static PointCloud2DCPU::Ptr clone(const PointCloud2D& points);

  template <typename T>
  void add_times(const T* times, int num_points);
  template <typename T>
  void add_times(const std::vector<T>& times) {
    add_times(times.data(), times.size());
  }

  template <typename T, int D>
  void add_points(const Eigen::Matrix<T, D, 1>* points, int num_points);
  template <typename T, int D, typename Alloc>
  void add_points(const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& points) {
    add_points(points.data(), points.size());
  }

  template <typename T, int D>
  void add_normals(const Eigen::Matrix<T, D, 1>* normals, int num_points);
  template <typename T, int D, typename Alloc>
  void add_normals(const std::vector<Eigen::Matrix<T, D, 1>, Alloc>& normals) {
    add_normals(normals.data(), normals.size());
  }

  template <typename T, int D>
  void add_covs(const Eigen::Matrix<T, D, D>* covs, int num_points);
  template <typename T, int D, typename Alloc>
  void add_covs(const std::vector<Eigen::Matrix<T, D, D>, Alloc>& covs) {
    add_covs(covs.data(), covs.size());
  }

  template <typename T>
  void add_intensities(const T* intensities, int num_points);
  template <typename T>
  void add_intensities(const std::vector<T>& intensities) {
    add_intensities(intensities.data(), intensities.size());
  }

  template <typename T>
  void add_aux_attribute(const std::string& attrib_name, const T* values, int num_points) {
    auto attributes = std::make_shared<std::vector<T>>(values, values + num_points);
    aux_attributes_storage[attrib_name] = attributes;
    aux_attributes[attrib_name] = std::make_pair(sizeof(T), attributes->data());
  }
  template <typename T, typename Alloc>
  void add_aux_attribute(const std::string& attrib_name, const std::vector<T, Alloc>& values) {
    add_aux_attribute(attrib_name, values.data(), values.size());
  }

  /// Load 2D point cloud from file
  static PointCloud2DCPU::Ptr load(const std::string& path);

  /// @brief Memory usage in bytes
  size_t memory_usage() const;

public:
  std::shared_ptr<std::vector<double>> times_storage;
  std::shared_ptr<std::vector<Eigen::Vector3d>> points_storage;
  std::shared_ptr<std::vector<Eigen::Vector3d>> normals_storage;
  std::shared_ptr<std::vector<Eigen::Matrix3d>> covs_storage;
  std::shared_ptr<std::vector<double>> intensities_storage;

  std::unordered_map<std::string, std::shared_ptr<void>> aux_attributes_storage;
};

}  // namespace gtsam_points

// Template implementations
#include <gtsam_points/d2/types/point_cloud_2d_cpu_impl.hpp>
