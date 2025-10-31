// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)
#include <gtsam_points/d2/ann/fast_occupancy_grid_2d.hpp>
#include <gtsam_points/d2/ann/impl/fast_occupancy_grid_2d_impl.hpp>

#include <gtsam_points/d2/types/frame_traits_2d.hpp>
#include <gtsam_points/util/fast_floor.hpp>
#include <gtsam_points/d2/util/vector2i_hash.hpp>

namespace gtsam_points {

FastOccupancyGrid2D::FastOccupancyGrid2D(double resolution)
: inv_resolution(1.0 / resolution),
  max_seek_count(10),
  blocks(512, std::make_pair(INVALID_INDEX, FastOccupancyBlock2D())) {}

FastOccupancyGrid2D::~FastOccupancyGrid2D() {}

int FastOccupancyGrid2D::num_occupied_cells() const {
  return std::accumulate(blocks.begin(), blocks.end(), 0, [](int sum, const auto& block) { return sum + block.second.count(); });
}

std::uint64_t FastOccupancyGrid2D::calc_index(const Eigen::Vector3i& coord) const {
  // Pack 2D coordinates into 64-bit index (21 bits each for X and Y)
  return (static_cast<std::uint64_t>((coord[0]) & coord_bit_mask) << (coord_bit_size * 0)) |  //
         (static_cast<std::uint64_t>((coord[1]) & coord_bit_mask) << (coord_bit_size * 1));
}

Eigen::Vector2i FastOccupancyGrid2D::calc_coord(std::uint64_t index) const {
  return Eigen::Vector2i(                                                               //
    static_cast<int>((index >> (coord_bit_size * 0)) & coord_bit_mask) - coord_offset,  //
    static_cast<int>((index >> (coord_bit_size * 1)) & coord_bit_mask) - coord_offset);
}

std::uint64_t FastOccupancyGrid2D::calc_hash(std::uint64_t index) const {
  return XORVector2iHash()(calc_coord(index));
}

std::uint64_t FastOccupancyGrid2D::find_block(std::uint64_t block_index) const {
  const std::uint64_t hash = calc_hash(block_index);
  for (int i = 0; i < max_seek_count; i++) {
    const int loc = (hash + i) & (blocks.size() - 1);
    if (blocks[loc].first == block_index) {
      return loc;
    }
  }

  return INVALID_INDEX;
}

std::uint64_t FastOccupancyGrid2D::find_or_insert_block(std::uint64_t block_index) {
  const std::uint64_t hash = calc_hash(block_index);
  for (int i = 0; i < max_seek_count; i++) {
    const int loc = (hash + i) & (blocks.size() - 1);
    if (blocks[loc].first == INVALID_INDEX || blocks[loc].first == block_index) {
      blocks[loc].first = block_index;
      return loc;
    }
  }

  rehash(blocks.size() * 2);
  return find_or_insert_block(block_index);
}

void FastOccupancyGrid2D::rehash(size_t hash_size) {
  std::vector<std::pair<std::uint64_t, FastOccupancyBlock2D>> new_blocks(hash_size, std::make_pair(INVALID_INDEX, FastOccupancyBlock2D()));

  for (const auto& block : blocks) {
    if (block.first == INVALID_INDEX) {
      continue;
    }

    bool inserted = false;
    const std::uint64_t hash = calc_hash(block.first);
    for (int i = 0; i < max_seek_count; i++) {
      const int loc = (hash + i) % hash_size;
      if (new_blocks[loc].first == INVALID_INDEX) {
        new_blocks[loc] = block;
        inserted = true;
        break;
      }
    }

    if (!inserted) {
      std::cerr << "failed to rehash (hash_size=" << hash_size << ")" << std::endl;
      return rehash(hash_size * 2);
    }
  }

  blocks = std::move(new_blocks);
}

// Explicit template instantiations
template void FastOccupancyGrid2D::insert<PointCloud2D>(const PointCloud2D& points, const Eigen::Isometry2d& pose);
template int FastOccupancyGrid2D::calc_overlap<PointCloud2D>(const PointCloud2D& points, const Eigen::Isometry2d& pose) const;
template double FastOccupancyGrid2D::calc_overlap_rate<PointCloud2D>(const PointCloud2D& points, const Eigen::Isometry2d& pose) const;
template std::vector<unsigned char> FastOccupancyGrid2D::get_overlaps<PointCloud2D>(const PointCloud2D& points, const Eigen::Isometry2d& pose) const;

}  // namespace gtsam_points
