// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <mutex>
#include <numeric>
#include <unordered_set>
#include <gtsam_points/util/easy_profiler.hpp>
#include <gtsam_points/d2/segmentation/region_growing_2d.hpp>

namespace gtsam_points {

template <typename PointCloud>
RegionGrowingContext2D region_growing_init_2d_(
  const PointCloud& points,
  const NearestNeighborSearch2D& search,
  const Eigen::Vector3d& seed_point,
  const RegionGrowingParams2D& params) {
  //
  size_t index;
  double sq_distance;
  // Find the closest point to be the seed
  if (!search.knn_search(seed_point.data(), 1, &index, &sq_distance)) {
    return RegionGrowingContext2D();
  }

  RegionGrowingContext2D context;
  context.seed_points.emplace_back(index);
  context.visited_seeds.resize(frame::size(points), 0);
  return context;
}

template <typename PointCloud>
bool region_growing_step_2d_(
  RegionGrowingContext2D& context,
  const PointCloud& points,
  const NearestNeighborSearch2D& search,
  const RegionGrowingParams2D& params) {
  //
  if (frame::has_normals(points) == false) {
    std::cerr << "warning: normals are required for region growing 2D" << std::endl;
    return true;
  }

  if (context.visited_seeds.size() != frame::size(points)) {
    std::cerr << "error: context.visited_seeds.size() != frame::size(points)" << std::endl;
    return true;
  }

  if (context.seed_points.empty()) {
    return true;
  }

  if (context.cluster_indices.size() > params.max_cluster_size) {
    context.seed_points.clear();
    return true;
  }

  const int seed_index = context.seed_points.front();
  context.seed_points.pop_front();

  if (context.visited_seeds[seed_index]) {
    // Skip if the seed has been visited
    return region_growing_step_2d_(context, points, search, params);
  }

  // Add the seed to the cluster
  context.visited_seeds[seed_index] = 1;
  context.cluster_indices.emplace_back(seed_index);

  // Find neighbors of the seed
  std::vector<size_t> neighbor_indices;
  std::vector<double> neighbor_sq_dists;
  search
    .radius_search(frame::point(points, seed_index).data(), params.distance_threshold, neighbor_indices, neighbor_sq_dists, params.max_cluster_size);

  const double sq_distance_threshold = params.distance_threshold * params.distance_threshold;
  const double cosine_threshold = std::cos(params.angle_threshold);
  for (size_t i = 0; i < neighbor_indices.size(); i++) {
    // Skip if the neighbor has been visited
    if (context.visited_seeds[neighbor_indices[i]]) {
      continue;
    }

    // Distance check
    if (neighbor_sq_dists[i] > sq_distance_threshold) {
      continue;
    }

    // Angle check (2D normals)
    const auto& seed_normal = frame::normal(points, seed_index);
    const auto& pt_normal = frame::normal(points, neighbor_indices[i]);
    // For 2D, normals are (nx, ny, 0), compute dot product of 2D part
    const double dot_product = seed_normal.head<2>().dot(pt_normal.head<2>());
    if (dot_product < cosine_threshold) {
      continue;
    }

    // Add the neighbor to the seed list
    context.seed_points.emplace_back(neighbor_indices[i]);
  }

  return false;
}

template <typename PointCloud>
void region_growing_dilation_2d_(
  RegionGrowingContext2D& context,
  const PointCloud& points,
  const NearestNeighborSearch2D& search,
  const RegionGrowingParams2D& params) {
  //
  std::vector<std::unordered_set<size_t>> new_indices(params.num_threads);

  // Find points within the dilation radius of the cluster
#pragma omp parallel for num_threads(params.num_threads) schedule(guided, 4)
  for (size_t i = 0; i < context.cluster_indices.size(); i++) {
    std::vector<size_t> indices;
    std::vector<double> sq_dists;
    search.radius_search(frame::point(points, context.cluster_indices[i]).data(), params.dilation_radius, indices, sq_dists);

#ifdef _OPENMP
    const int thread_num = omp_get_thread_num();
#else
    const int thread_num = 0;  // Single-threaded execution
#endif

    new_indices[thread_num].insert(indices.begin(), indices.end());
  }

  // Merge and sort the indices
  for (size_t i = 1; i < new_indices.size(); i++) {
    new_indices[0].insert(new_indices[i].begin(), new_indices[i].end());
  }

  context.cluster_indices.assign(new_indices[0].begin(), new_indices[0].end());
  std::sort(context.cluster_indices.begin(), context.cluster_indices.end());
}

template <typename PointCloud>
bool region_growing_update_2d_(
  RegionGrowingContext2D& context,
  const PointCloud& points,
  const NearestNeighborSearch2D& search,
  const RegionGrowingParams2D& params) {
  //
  if (context.seed_points.empty()) {
    return true;
  }

  for (int i = 0; i < params.max_steps; i++) {
    if (context.seed_points.empty()) {
      break;
    }

    region_growing_step_2d_(context, points, search, params);
  }

  if (context.seed_points.empty() && params.dilation_radius > 0.0) {
    region_growing_dilation_2d_(context, points, search, params);
  }

  return context.seed_points.empty();
}

}  // namespace gtsam_points
