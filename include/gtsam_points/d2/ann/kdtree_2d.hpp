// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <memory>
#include <iostream>
#include <Eigen/Core>

#include <gtsam_points/d2/ann/nearest_neighbor_search_2d.hpp>

namespace gtsam_points {

template <typename PointCloud, typename Projection>
struct UnsafeKdTree;

/**
 * @brief 2D axis-aligned projection for KdTree
 *
 * Selects either X or Y axis based on variance for 2D point splitting.
 */
struct AxisAlignedProjection2D {
public:
  /// @brief Project the 2D point to the selected axis
  /// @param pt  2D homogeneous point (x, y, 1)
  /// @return    Projected value (either x or y)
  double operator()(const Eigen::Vector3d& pt) const { return pt[axis]; }

  /// @brief Find the axis with the largest variance in 2D (X or Y)
  /// @param points     Point cloud with 2D points
  /// @param first      First point index iterator
  /// @param last       Last point index iterator
  /// @param setting    Search setting
  /// @return           Projection with the largest variance axis
  template <typename PointCloud, typename IndexConstIterator>
  static AxisAlignedProjection2D
  find_axis(const PointCloud& points, IndexConstIterator first, IndexConstIterator last, const struct ProjectionSetting& setting);

public:
  int axis;  ///< Axis index (0: X, 1: Y)
};

/**
 * @brief KdTree-based 2D nearest neighbor search
 *
 * Efficient spatial indexing for 2D points using a KdTree.
 * Points are stored as homogeneous coordinates (x, y, 1) but
 * distance calculations use only 2D Euclidean distance.
 */
struct KdTree2D : public NearestNeighborSearch2D {
public:
  using Index = UnsafeKdTree<KdTree2D, AxisAlignedProjection2D>;

  /**
   * @brief Constructor
   * @param points           Array of 2D homogeneous points (x, y, 1)
   * @param num_points       Number of points
   * @param build_num_threads Number of threads for building the tree
   */
  KdTree2D(const Eigen::Vector3d* points, int num_points, int build_num_threads = 1);
  virtual ~KdTree2D() override;

  /// @brief Find k nearest neighbors in 2D
  /// @param pt           Query point (2D data: x, y)
  /// @param k            Number of neighbors to search
  /// @param k_indices    Indices of k nearest neighbors
  /// @param k_sq_dists   Squared distances of k nearest neighbors (2D Euclidean)
  /// @param max_sq_dist  Maximum squared distance threshold
  /// @return             Number of neighbors found
  virtual size_t knn_search(
    const double* pt,
    size_t k,
    size_t* k_indices,
    double* k_sq_dists,
    double max_sq_dist = std::numeric_limits<double>::max()) const override;

  /**
   * @brief Radius search in 2D
   * @note  KdTree tends to first pick closer points when max_num_neighbors is specified
   * @param pt                 Query point (2D data: x, y)
   * @param radius             Search radius in 2D Euclidean space
   * @param indices            Indices of neighbors within the radius
   * @param sq_dists           Squared distances to the neighbors
   * @param max_num_neighbors  Maximum number of neighbors
   * @return                   Number of neighbors found
   */
  virtual size_t radius_search(
    const double* pt,
    double radius,
    std::vector<size_t>& indices,
    std::vector<double>& sq_dists,
    int max_num_neighbors = std::numeric_limits<int>::max()) const override;

public:
  const int num_points;
  const Eigen::Vector3d* points;  ///< 2D homogeneous points (x, y, 1)

  double search_eps;  ///< Search epsilon for approximate search

  std::unique_ptr<Index> index;  ///< Underlying KdTree index
};

}  // namespace gtsam_points
