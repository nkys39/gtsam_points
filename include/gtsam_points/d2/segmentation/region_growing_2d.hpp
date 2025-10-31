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

/// @brief Region growing parameters for 2D
struct RegionGrowingParams2D {
  double distance_threshold = 0.5;               ///< Distance threshold [m]
  double angle_threshold = 10.0 * M_PI / 180.0;  ///< Angle threshold [rad]
  double dilation_radius = 0.5;                  ///< Radius of dilation after region growing [m]
  int max_cluster_size = 1000000;                ///< Maximum cluster size
  int max_steps = 1000000;                       ///< Maximum number of update steps
  int num_threads = 1;                           ///< Number of threads
};

/// @brief Region growing context for 2D
struct RegionGrowingContext2D {
  std::vector<size_t> cluster_indices;  ///< Indices of points in the cluster
  std::deque<size_t> seed_points;       ///< Seed points to be evaluated
  std::vector<bool> visited_seeds;      ///< Seed points that have been visited
};

/// @brief Initialize region growing for 2D
/// @param points       Point cloud (2D)
/// @param search       Nearest neighbor search (2D)
/// @param seed_point   Seed point in 2D homogeneous coordinates (x, y, 1)
/// @param params       Region growing parameters
/// @return             Region growing context
template <typename PointCloud>
RegionGrowingContext2D region_growing_init_2d_(
  const PointCloud& points,
  const NearestNeighborSearch2D& search,
  const Eigen::Vector3d& seed_point,
  const RegionGrowingParams2D& params);

/// @brief Update the region growing context once for 2D
/// @param context    Region growing context
/// @param points     Point cloud (2D)
/// @param search     Nearest neighbor search (2D)
/// @param params     Region growing parameters
/// @return           True if the region growing is converged
template <typename PointCloud>
bool region_growing_step_2d_(
  RegionGrowingContext2D& context,
  const PointCloud& points,
  const NearestNeighborSearch2D& search,
  const RegionGrowingParams2D& params);

/// @brief Dilation step after region growing for 2D
/// @param context  Region growing context
/// @param points   Point cloud (2D)
/// @param search   Nearest neighbor search (2D)
/// @param params   Region growing parameters
template <typename PointCloud>
void region_growing_dilation_2d_(
  RegionGrowingContext2D& context,
  const PointCloud& points,
  const NearestNeighborSearch2D& search,
  const RegionGrowingParams2D& params);

/// @brief Update region growing several steps for 2D
/// @param context  Region growing context
/// @param points   Point cloud (2D)
/// @param search   Nearest neighbor search (2D)
/// @param params   Region growing parameters
/// @return         True if the region growing is converged
template <typename PointCloud>
bool region_growing_update_2d_(
  RegionGrowingContext2D& context,
  const PointCloud& points,
  const NearestNeighborSearch2D& search,
  const RegionGrowingParams2D& params);

RegionGrowingContext2D region_growing_init_2d(
  const PointCloud2D& points,
  const NearestNeighborSearch2D& search,
  const Eigen::Vector3d& seed_point,
  const RegionGrowingParams2D& params);

bool region_growing_update_2d(
  RegionGrowingContext2D& context,
  const PointCloud2D& points,
  const NearestNeighborSearch2D& search,
  const RegionGrowingParams2D& params);

}  // namespace gtsam_points
