// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)
#pragma once

#include <vector>
#include <bitset>
#include <numeric>
#include <gtsam_points/types/point_cloud.hpp>

namespace gtsam_points {

/// @brief Fast occupancy grid block for 2D.
struct FastOccupancyBlock2D {
public:
  FastOccupancyBlock2D() : cells(0) {}

  static constexpr int stride = 8;                         ///< Size of the block in each dimension (8x8 grid)
  static constexpr int num_cells = stride * stride;        ///< Number of cells in the block (64)

  /// @brief Calculate the index of the cell in the block.
  /// @param coord 2D coordinate of the cell within the block.
  int cell_index(const Eigen::Vector2i& coord) const { return coord.x() + coord.y() * stride; }

  /// @brief Check if the cell is occupied.
  /// @param coord 2D coordinate of the cell.
  /// @return True if the cell is occupied.
  bool occupied(const Eigen::Vector2i& coord) const { return cells[cell_index(coord)]; }

  /// @brief Check if the cell is free.
  /// @param coord 2D coordinate of the cell.
  /// @return True if the cell is free.
  bool free(const Eigen::Vector2i& coord) const { return !occupied(coord); }

  /// @brief Set the cell as occupied.
  /// @param coord 2D coordinate of the cell.
  void set_occupied(const Eigen::Vector2i& coord) { cells.set(cell_index(coord)); }

  /// @brief Set the cell as free.
  /// @param coord 2D coordinate of the cell.
  void set_free(const Eigen::Vector2i& coord) { cells.reset(cell_index(coord)); }

  /// @brief Count the number of occupied cells.
  /// @return Number of occupied cells.
  int count() const { return cells.count(); }

public:
  std::bitset<num_cells> cells;  ///< Occupancy status of each cell in the block.
};

/// @brief Fast occupancy grid for 2D with occupancy blocks and flat hashing for efficient point cloud overlap evaluation.
class FastOccupancyGrid2D {
public:
  using Ptr = std::shared_ptr<FastOccupancyGrid2D>;
  using ConstPtr = std::shared_ptr<const FastOccupancyGrid2D>;

  /// @brief Constructor.
  /// @param resolution Voxel resolution.
  FastOccupancyGrid2D(double resolution);
  ~FastOccupancyGrid2D();

  /// @brief Insert points into the grid.
  /// @param points Point cloud (2D).
  /// @param pose   Pose of the points (SE(2)).
  template <typename PointCloud>
  void insert(const PointCloud& points, const Eigen::Isometry2d& pose = Eigen::Isometry2d::Identity());

  /// @brief Calculate the number of points overlapping with the grid.
  /// @param points Point cloud (2D).
  /// @param pose   Pose of the points (SE(2)).
  /// @return       Number of overlapping points.
  template <typename PointCloud>
  int calc_overlap(const PointCloud& points, const Eigen::Isometry2d& pose = Eigen::Isometry2d::Identity()) const;

  /// @brief Calculate the overlap ratio of the points with the grid.
  /// @param points Point cloud (2D).
  /// @param pose   Pose of the points (SE(2)).
  /// @return       Overlap ratio.
  template <typename PointCloud>
  double calc_overlap_rate(const PointCloud& points, const Eigen::Isometry2d& pose = Eigen::Isometry2d::Identity()) const;

  /// @brief Get the overlap status of each point in the point cloud.
  /// @param points Point cloud (2D).
  /// @param pose   Pose of the points (SE(2)).
  /// @return       Overlap status of each point (0=free, 1=occupied).
  template <typename PointCloud>
  std::vector<unsigned char> get_overlaps(const PointCloud& points, const Eigen::Isometry2d& pose = Eigen::Isometry2d::Identity()) const;

  /// @brief Get the number of occupied cells in the grid.
  int num_occupied_cells() const;

private:
  std::uint64_t calc_index(const Eigen::Vector3i& coord) const;

  Eigen::Vector2i calc_coord(std::uint64_t index) const;

  std::uint64_t calc_hash(std::uint64_t index) const;

  std::uint64_t find_block(std::uint64_t block_index) const;

  std::uint64_t find_or_insert_block(std::uint64_t block_index);

  void rehash(size_t hash_size);

private:
  static constexpr std::uint64_t INVALID_INDEX = std::numeric_limits<std::uint64_t>::max();
  static constexpr int coord_bit_size = 21;                          ///< Bits to represent each voxel coordinate (pack 21x2=42bits in 64bit int)
  static constexpr std::uint64_t coord_bit_mask = (1ull << 21) - 1;  ///< Bit mask
  static constexpr std::uint64_t coord_offset = 1ull << (coord_bit_size - 1);  ///< Coordinate offset to make values positive

  double inv_resolution;                                                  ///< Inverse of the voxel resolution
  int max_seek_count;                                                     ///< Maximum number of seek attempts
  std::vector<std::pair<std::uint64_t, FastOccupancyBlock2D>> blocks;    ///< Hash table of occupancy blocks
};

}  // namespace gtsam_points
