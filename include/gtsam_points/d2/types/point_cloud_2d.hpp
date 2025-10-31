// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <memory>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gtsam_points {

/**
 * @brief 2D point cloud class that holds only pointers to point attributes
 * @note  For 2D laser-based SLAM. Points are represented in homogeneous coordinates (x, y, 1).
 * @note  If you don't want to manage the lifetime of point data by yourself, use gtsam_points::PointCloud2DCPU.
 */
struct PointCloud2D {
public:
  using Ptr = std::shared_ptr<PointCloud2D>;
  using ConstPtr = std::shared_ptr<const PointCloud2D>;

  PointCloud2D()
  : num_points(0),
    times(nullptr),
    points(nullptr),
    normals(nullptr),
    covs(nullptr),
    intensities(nullptr) {}

  virtual ~PointCloud2D() {}

  // Forbid copy
  PointCloud2D(const PointCloud2D&) = delete;
  PointCloud2D& operator=(PointCloud2D const&) = delete;

  /// Number of points
  size_t size() const { return num_points; }

  bool has_times() const;        ///< Check if the point cloud has per-point timestamps
  bool has_points() const;       ///< Check if the point cloud has points
  bool has_normals() const;      ///< Check if the point cloud has point normals (2D: perpendicular to local line)
  bool has_covs() const;         ///< Check if the point cloud has point covariances (2x2)
  bool has_intensities() const;  ///< Check if the point cloud has point intensities

  bool check_times() const;        ///< Warn if the point cloud doesn't have times
  bool check_points() const;       ///< Warn if the point cloud doesn't have points
  bool check_normals() const;      ///< Warn if the point cloud doesn't have normals
  bool check_covs() const;         ///< Warn if the point cloud doesn't have covs
  bool check_intensities() const;  ///< Warn if the point cloud doesn't have intensities

  /**
   * @brief Get the pointer to an aux attribute
   * @param  attrib Attribute name
   * @return Returns the pointer to it if the specified attribute exists. Otherwise, returns nullptr.
   */
  template <typename T>
  const T* aux_attribute(const std::string& attrib) const {
    const auto found = aux_attributes.find(attrib);
    if (found == aux_attributes.end()) {
      std::cerr << "warning: attribute " << attrib << " not found!!" << std::endl;
      return nullptr;
    }

    if (sizeof(T) != found->second.first) {
      std::cerr << "warning: attribute element size mismatch!! attrib:" << attrib
                << " size:" << found->second.first << " requested:" << sizeof(T) << std::endl;
    }

    return static_cast<const T*>(found->second.second);
  }

  /**
   * @brief Save the point cloud data
   * @param path Destination path
   */
  void save(const std::string& path) const;

  /**
   * @brief Save the point cloud data with a compact representation (x, y only, without homogeneous coordinate)
   * @param path Destination path
   */
  void save_compact(const std::string& path) const;

public:
  size_t num_points;  ///< Number of points

  double* times;             ///< Per-point timestamp w.r.t. the first point (should be sorted)
  Eigen::Vector3d* points;   ///< Point coordinates (x, y, 1) in homogeneous coordinates
  Eigen::Vector3d* normals;  ///< Point normals (nx, ny, 0) - perpendicular to local line segment
  Eigen::Matrix3d* covs;     ///< Point covariances - 2x2 in upper-left, cov(2,2) = 0
  double* intensities;       ///< Point intensities (e.g., laser reflectivity)

  /// Aux attributes <attribute_name, pair<element_size, data_ptr>>
  std::unordered_map<std::string, std::pair<size_t, void*>> aux_attributes;
};

}  // namespace gtsam_points
