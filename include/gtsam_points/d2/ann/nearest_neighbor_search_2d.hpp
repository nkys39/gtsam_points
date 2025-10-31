// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#pragma once

#include <memory>
#include <vector>
#include <limits>
#include <iostream>

namespace gtsam_points {

/**
 * @brief 2D Nearest neighbor search interface
 *
 * Base interface for 2D nearest neighbor search algorithms.
 * Points are expected to be in homogeneous coordinates (x, y, 1),
 * but distance calculations use only the 2D Euclidean distance in xy-plane.
 */
struct NearestNeighborSearch2D {
public:
  using Ptr = std::shared_ptr<NearestNeighborSearch2D>;
  using ConstPtr = std::shared_ptr<const NearestNeighborSearch2D>;

  NearestNeighborSearch2D() {}
  virtual ~NearestNeighborSearch2D() {}

  /**
   * @brief k-nearest neighbor search in 2D
   * @param pt          Query point (2D point data, typically x,y from Vector3d)
   * @param k           Number of neighbors
   * @param k_indices   Indices of k-nearest neighbors
   * @param k_sq_dists  Squared distances to the neighbors (sorted in ascending order)
   * @param max_sq_dist Maximum squared distance threshold
   * @return            Number of neighbors found
   */
  virtual size_t knn_search(
    const double* pt,
    size_t k,
    size_t* k_indices,
    double* k_sq_dists,
    double max_sq_dist = std::numeric_limits<double>::max()) const {
    std::cerr << "NearestNeighborSearch2D::knn_search() is not implemented" << std::endl;
    return 0;
  };

  /**
   * @brief Radius search in 2D
   * @note  There is no assumption and guarantee on the order of points to be selected when `max_num_neighbors` is specified.
   *        (Some algorithms like KdTree tend to first pick closer points though).
   * @param pt                 Query point (2D point data)
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
    int max_num_neighbors = std::numeric_limits<int>::max()) const {
    std::cerr << "NearestNeighborSearch2D::radius_search() is not implemented" << std::endl;
    return 0;
  };
};

}  // namespace gtsam_points
