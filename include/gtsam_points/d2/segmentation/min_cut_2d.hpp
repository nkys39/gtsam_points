// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <deque>
#include <vector>
#include <cstdint>
#include <Eigen/Core>
#include <gtsam_points/d2/types/point_cloud_2d.hpp>
#include <gtsam_points/d2/ann/nearest_neighbor_search_2d.hpp>

namespace gtsam_points {

/// @brief Parameters for min-cut segmentation in 2D
struct MinCutParams2D {
  double distance_sigma = 0.25;              ///< Distance sigma [m]
  double angle_sigma = 10.0 * M_PI / 180.0;  ///< Angle sigma [rad]

  double foreground_mask_radius = 0.2;  ///< All points within this radius from the source point are considered as foreground [m]
  double background_mask_radius = 3.0;  ///< All points out of this radius from the source point are considered as background [m]
  double foreground_weight = 0.2;       ///< Weight for the foreground points
  double background_weight = 0.2;       ///< Weight for the background points

  int k_neighbors = 20;  ///< Number of neighbors
  int num_threads = 1;   ///< Number of threads
};

/// @brief Result of min-cut segmentation in 2D
struct MinCutResult2D {
  MinCutResult2D() : source_index(-1), sink_index(-1), max_flow(0.0) {}

  size_t source_index;                  ///< Source point index
  size_t sink_index;                    ///< Sink point index
  double max_flow;                      ///< Maximum flow
  std::vector<size_t> cluster_indices;  ///< Indices of foreground points
};

/// @brief Min-cut segmentation for 2D
/// @param points           Point cloud (2D)
/// @param search           Nearest neighbor search (2D)
/// @param source_pt_index  Index of the source point
/// @param params           Parameters
/// @return                 Segmentation result
template <typename PointCloud>
MinCutResult2D min_cut_2d_(const PointCloud& points, const NearestNeighborSearch2D& search, size_t source_pt_index, const MinCutParams2D& params);

/// @brief Min-cut segmentation for 2D
/// @param points       Point cloud (2D)
/// @param search       Nearest neighbor search (2D)
/// @param source_pt    Source point in 2D homogeneous coordinates (x, y, 1) - The point nearest to this point is used as the source point
/// @param params       Parameters
/// @return             Segmentation result
template <typename PointCloud>
MinCutResult2D min_cut_2d_(const PointCloud& points, const NearestNeighborSearch2D& search, const Eigen::Vector3d& source_pt, const MinCutParams2D& params);

MinCutResult2D min_cut_2d(const PointCloud2D& points, const NearestNeighborSearch2D& search, size_t source_pt_index, const MinCutParams2D& params);

MinCutResult2D min_cut_2d(const PointCloud2D& points, const NearestNeighborSearch2D& search, const Eigen::Vector3d& source_pt, const MinCutParams2D& params);

}  // namespace gtsam_points
