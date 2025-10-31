// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <unordered_map>
#include <Eigen/Core>
#include <gtsam_points/d2/types/point_cloud_2d.hpp>

namespace gtsam_points {

/// @brief 2D Gaussian grid cell that computes and stores cell mean and covariance.
struct GaussianGridCell2D {
public:
  using Ptr = std::shared_ptr<GaussianGridCell2D>;
  using ConstPtr = std::shared_ptr<const GaussianGridCell2D>;

  struct Setting {};

  /// @brief Constructor.
  GaussianGridCell2D() : finalized(false), num_points(0), mean(Eigen::Vector3d::Zero()), cov(Eigen::Matrix3d::Zero()) {}

  /// @brief Number of points in the cell (Always 1 for GaussianGridCell2D after finalization).
  size_t size() const { return 1; }

  /// @brief Add a point to the cell.
  void add(const PointCloud2D& points, size_t i);

  /// @brief Finalize the cell mean and covariance.
  void finalize();

  /// @brief Find k nearest neighbors (returns this cell's mean).
  template <typename Result>
  void knn_search(const Eigen::Vector3d& pt, Result& result) const {
    const double sq_dist = (pt.head<2>() - mean.head<2>()).squaredNorm();
    result.push(0, sq_dist);
  }

public:
  bool finalized;        ///< If true, mean and cov are finalized
  size_t num_points;     ///< Number of input points
  Eigen::Vector3d mean;  ///< Mean (x, y, 1)
  Eigen::Matrix3d cov;   ///< 2x2 Covariance (in upper-left of 3x3)
};

namespace frame {

template <>
struct traits<GaussianGridCell2D> {
  static int size(const GaussianGridCell2D& frame) { return frame.size(); }

  static bool has_points(const GaussianGridCell2D& frame) { return true; }
  static bool has_normals(const GaussianGridCell2D& frame) { return false; }
  static bool has_covs(const GaussianGridCell2D& frame) { return true; }
  static bool has_intensities(const GaussianGridCell2D& frame) { return false; }

  static const Eigen::Vector3d& point(const GaussianGridCell2D& frame, size_t i) { return frame.mean; }
  static Eigen::Vector3d normal(const GaussianGridCell2D& frame, size_t i) { return Eigen::Vector3d::Zero(); }
  static const Eigen::Matrix3d& cov(const GaussianGridCell2D& frame, size_t i) { return frame.cov; }
  static double intensity(const GaussianGridCell2D& frame, size_t i) { return 0.0; }
};

}  // namespace frame

/**
 * @brief 2D Gaussian grid map for voxelized 2D point clouds
 *
 * A 2D grid map where each cell stores the mean and covariance
 * of points within that cell. Used for VGICP2D.
 */
class GaussianGridMap2D {
public:
  using Ptr = std::shared_ptr<GaussianGridMap2D>;
  using ConstPtr = std::shared_ptr<const GaussianGridMap2D>;

  /// @brief Constructor.
  /// @param resolution   Grid resolution [m]
  GaussianGridMap2D(double resolution);
  virtual ~GaussianGridMap2D();

  /// Grid resolution
  double grid_resolution() const { return resolution; }

  /// @brief Compute the grid cell coordinate corresponding to a point.
  Eigen::Vector2i grid_coord(const Eigen::Vector3d& x) const;

  /// @brief Look up a grid cell by coordinate. Returns nullptr if not found.
  const GaussianGridCell2D* lookup_cell(const Eigen::Vector2i& coord) const;

  /// @brief Insert a point cloud frame into the grid map.
  void insert(const PointCloud2D& frame);

  /// @brief Get number of cells
  size_t num_cells() const { return cells.size(); }

  /// @brief Check if grid has points
  bool has_points() const { return !cells.empty(); }

  /// @brief Check if grid has covariances
  bool has_covs() const { return !cells.empty(); }

  /// @brief Check if grid has normals (always false)
  bool has_normals() const { return false; }

  /// @brief Check if grid has intensities (always false)
  bool has_intensities() const { return false; }

private:
  double resolution;  ///< Grid resolution [m]

  // Hash function for Eigen::Vector2i
  struct Vector2iHash {
    std::size_t operator()(const Eigen::Vector2i& v) const {
      std::size_t h1 = std::hash<int>{}(v.x());
      std::size_t h2 = std::hash<int>{}(v.y());
      return h1 ^ (h2 << 1);
    }
  };

  std::unordered_map<Eigen::Vector2i, GaussianGridCell2D, Vector2iHash> cells;  ///< Grid cells
};

}  // namespace gtsam_points
