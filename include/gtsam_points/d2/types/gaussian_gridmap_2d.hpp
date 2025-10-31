// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)
#pragma once

#include <Eigen/Core>
#include <gtsam_points/d2/types/point_cloud_2d.hpp>
#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>

namespace gtsam_points {

/// @brief Gaussian grid cell that computes and stores cell mean and covariance for 2D.
struct GaussianGridCell2D {
public:
  using Ptr = std::shared_ptr<GaussianGridCell2D>;
  using ConstPtr = std::shared_ptr<const GaussianGridCell2D>;

  struct Setting {};

  /// @brief Constructor.
  GaussianGridCell2D() : finalized(false), num_points(0), mean(Eigen::Vector3d::Zero()), cov(Eigen::Matrix3d::Zero()) {}

  /// @brief  Number of points in the cell (Always 1 for GaussianGridCell2D after finalization).
  size_t size() const { return 1; }

  /// @brief  Add a point to the cell.
  /// @param  points         Point cloud (2D)
  /// @param  i              Index of the point
  void add(const PointCloud2D& points, size_t i);

  /// @brief Finalize the cell mean and covariance.
  void finalize();

  /// @brief Find k nearest neighbors.
  /// @param pt           Query point
  /// @param result       Result
  template <typename Result>
  void knn_search(const Eigen::Vector3d& pt, Result& result) const {
    const double sq_dist = (pt.head<2>() - mean.head<2>()).squaredNorm();
    result.push(0, sq_dist);
  }

public:
  bool finalized;        ///< If true, mean and cov are finalized, otherwise they represent the sum of input points
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
 * @brief 2D Gaussian grid map that inherits from IncrementalGridMap2D
 *
 * A complete implementation of 2D Gaussian grid map with:
 * - LRU caching for dynamic cell management
 * - Efficient neighbor search using IncrementalGridMap2D
 * - Full feature parity with GaussianVoxelMapCPU (3D version)
 */
class GaussianGridMap2D : public IncrementalGridMap2D<GaussianGridCell2D> {
public:
  using Ptr = std::shared_ptr<GaussianGridMap2D>;
  using ConstPtr = std::shared_ptr<const GaussianGridMap2D>;

  /// @brief Constructor.
  /// @param resolution   Grid resolution [m]
  GaussianGridMap2D(double resolution);
  virtual ~GaussianGridMap2D();

  /// Grid resolution
  double grid_resolution() const;

  /// @brief Compute the grid cell coordinate corresponding to a point.
  Eigen::Vector2i grid_coord(const Eigen::Vector3d& x) const;

  /// @brief Look up a grid cell index. If the cell does not exist, return -1.
  int lookup_cell_index(const Eigen::Vector2i& coord) const;

  /// @brief Look up a grid cell.
  const GaussianGridCell2D& lookup_cell(int cell_id) const;

  /// @brief  Insert a point cloud frame into the grid map.
  virtual void insert(const PointCloud2D& frame) override;

  /**
   * @brief Save the grid map in compact format
   * @param path  Destination path to save the grid map
   */
  void save_compact(const std::string& path) const;

  /**
   * @brief Load a grid map from a file
   * @param path  Path to a grid map file to be loaded
   */
  static GaussianGridMap2D::Ptr load(const std::string& path);
};

namespace frame {

template <>
struct traits<GaussianGridMap2D> {
  static bool has_points(const GaussianGridMap2D& igrid) { return igrid.has_points(); }
  static bool has_normals(const GaussianGridMap2D& igrid) { return igrid.has_normals(); }
  static bool has_covs(const GaussianGridMap2D& igrid) { return igrid.has_covs(); }
  static bool has_intensities(const GaussianGridMap2D& igrid) { return igrid.has_intensities(); }

  static decltype(auto) point(const GaussianGridMap2D& igrid, size_t i) { return igrid.point(i); }
  static decltype(auto) normal(const GaussianGridMap2D& igrid, size_t i) { return igrid.normal(i); }
  static decltype(auto) cov(const GaussianGridMap2D& igrid, size_t i) { return igrid.cov(i); }
  static decltype(auto) intensity(const GaussianGridMap2D& igrid, size_t i) { return igrid.intensity(i); }
};

}  // namespace frame

}  // namespace gtsam_points
