// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)
#pragma once

#include <gtsam_points/d2/ann/fast_occupancy_grid_2d.hpp>

#include <gtsam_points/d2/types/frame_traits_2d.hpp>
#include <gtsam_points/util/fast_floor.hpp>
#include <gtsam_points/d2/util/vector2i_hash.hpp>
#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>

namespace gtsam_points {

template <typename PointCloud>
void FastOccupancyGrid2D::insert(const PointCloud& points, const Eigen::Isometry2d& pose) {
  for (int i = 0; i < frame::size(points); i++) {
    const auto& pt = frame::point(points, i);
    // Transform 2D point to global coordinates
    const Eigen::Vector3d transformed = pose * pt;
    const Eigen::Vector3i global_coord = fast_floor(transformed * inv_resolution) + Eigen::Vector3i(coord_offset, coord_offset, 0);
    const Eigen::Vector3i block_coord = global_coord / FastOccupancyBlock2D::stride;
    const Eigen::Vector3i cell_coord = global_coord - block_coord * FastOccupancyBlock2D::stride;

    const std::uint64_t block_index = calc_index(block_coord);
    const std::uint64_t block_loc = find_or_insert_block(block_index);
    blocks[block_loc].second.set_occupied(cell_coord.head<2>());
  }
}

template <typename PointCloud>
int FastOccupancyGrid2D::calc_overlap(const PointCloud& points, const Eigen::Isometry2d& pose) const {
  int num_overlap = 0;
  for (int i = 0; i < frame::size(points); i++) {
    const auto& pt = frame::point(points, i);
    // Transform 2D point to global coordinates
    const Eigen::Vector3d transformed = pose * pt;
    const Eigen::Vector3i global_coord = fast_floor(transformed * inv_resolution) + Eigen::Vector3i(coord_offset, coord_offset, 0);
    const Eigen::Vector3i block_coord = global_coord / FastOccupancyBlock2D::stride;

    const std::uint64_t block_index = calc_index(block_coord);
    const std::uint64_t block_loc = find_block(block_index);
    if (block_loc == INVALID_INDEX) {
      continue;
    }

    const Eigen::Vector3i cell_coord = global_coord - block_coord * FastOccupancyBlock2D::stride;
    num_overlap += blocks[block_loc].second.occupied(cell_coord.head<2>());
  }

  return num_overlap;
}

template <typename PointCloud>
double FastOccupancyGrid2D::calc_overlap_rate(const PointCloud& points, const Eigen::Isometry2d& pose) const {
  return calc_overlap(points, pose) / static_cast<double>(frame::size(points));
}

template <typename PointCloud>
std::vector<unsigned char> FastOccupancyGrid2D::get_overlaps(const PointCloud& points, const Eigen::Isometry2d& pose) const {
  std::vector<unsigned char> overlaps(frame::size(points), 0);

  for (int i = 0; i < frame::size(points); i++) {
    const auto& pt = frame::point(points, i);
    // Transform 2D point to global coordinates
    const Eigen::Vector3d transformed = pose * pt;
    const Eigen::Vector3i global_coord = fast_floor(transformed * inv_resolution) + Eigen::Vector3i(coord_offset, coord_offset, 0);
    const Eigen::Vector3i block_coord = global_coord / FastOccupancyBlock2D::stride;

    const std::uint64_t block_index = calc_index(block_coord);
    const std::uint64_t block_loc = find_block(block_index);
    if (block_loc == INVALID_INDEX) {
      continue;
    }

    const Eigen::Vector3i cell_coord = global_coord - block_coord * FastOccupancyBlock2D::stride;
    overlaps[i] = blocks[block_loc].second.occupied(cell_coord.head<2>());
  }

  return overlaps;
}

}  // namespace gtsam_points
