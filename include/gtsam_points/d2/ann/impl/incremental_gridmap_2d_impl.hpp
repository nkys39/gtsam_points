// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)
#pragma once

#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>

#include <gtsam_points/d2/ann/knn_result_2d.hpp>
#include <gtsam_points/util/fast_floor.hpp>

namespace gtsam_points {

template <typename CellContents>
IncrementalGridMap2D<CellContents>::IncrementalGridMap2D(double resolution)
: inv_resolution(1.0 / resolution),
  lru_horizon(10),
  lru_clear_cycle(10),
  lru_counter(0),
  offsets(neighbor_offsets(5)) {}

template <typename CellContents>
IncrementalGridMap2D<CellContents>::~IncrementalGridMap2D() {}

template <typename CellContents>
void IncrementalGridMap2D<CellContents>::clear() {
  lru_counter = 0;
  flat_cells.clear();
  cells.clear();
}

template <typename CellContents>
void IncrementalGridMap2D<CellContents>::insert(const PointCloud2D& points) {
  // Insert points to the grid map
  for (size_t i = 0; i < points.size(); i++) {
    const Eigen::Vector2i coord = fast_floor(points.points[i] * inv_resolution).template head<2>();

    auto found = cells.find(coord);
    if (found == cells.end()) {
      auto cell = std::make_shared<std::pair<GridCellInfo2D, CellContents>>(GridCellInfo2D(coord, lru_counter), CellContents());

      found = cells.emplace_hint(found, coord, flat_cells.size());
      flat_cells.emplace_back(cell);
    }

    auto& [info, cell] = *flat_cells[found->second];
    info.lru = lru_counter;
    cell.add(points, i);
  }

  if ((++lru_counter) % lru_clear_cycle == 0) {
    // Remove least recently used cells
    auto remove_counter =
      std::remove_if(flat_cells.begin(), flat_cells.end(), [&](const std::shared_ptr<std::pair<GridCellInfo2D, CellContents>>& cell) {
        return cell->first.lru + lru_horizon < lru_counter;
      });
    flat_cells.erase(remove_counter, flat_cells.end());

    // Rehash
    cells.clear();
    for (size_t i = 0; i < flat_cells.size(); i++) {
      cells[flat_cells[i]->first.coord] = i;
    }
  }

  // Finalize cell means and covs
  for (auto& cell : flat_cells) {
    cell->second.finalize();
  }
}

template <typename CellContents>
size_t IncrementalGridMap2D<CellContents>::knn_search(const double* pt, size_t k, size_t* k_indices, double* k_sq_dists, double max_sq_dist) const {
  const Eigen::Vector3d query = (Eigen::Vector3d() << pt[0], pt[1], 1.0).finished();
  const Eigen::Vector2i center = fast_floor(query * inv_resolution).template head<2>();

  size_t cell_index = 0;
  const auto index_transform = [&](const size_t point_index) { return calc_index(cell_index, point_index); };

  KnnResult2D<-1, decltype(index_transform)> result(k_indices, k_sq_dists, k, index_transform, max_sq_dist);
  for (const auto& offset : offsets) {
    const Eigen::Vector2i coord = center + offset;
    const auto found = cells.find(coord);
    if (found == cells.end()) {
      continue;
    }

    cell_index = found->second;
    const auto& cell = flat_cells[cell_index]->second;
    cell.knn_search(query, result);
  }

  return result.num_found();
}

template <typename CellContents>
std::vector<Eigen::Vector2i> IncrementalGridMap2D<CellContents>::neighbor_offsets(const int neighbor_cell_mode) const {
  switch (neighbor_cell_mode) {
    case 1:
      // Only center cell
      return std::vector<Eigen::Vector2i>{Eigen::Vector2i(0, 0)};
    case 5:
      // Center + 4 neighbors (up, down, left, right)
      return std::vector<Eigen::Vector2i>{
        Eigen::Vector2i(0, 0),
        Eigen::Vector2i(1, 0),
        Eigen::Vector2i(-1, 0),
        Eigen::Vector2i(0, 1),
        Eigen::Vector2i(0, -1)};
    case 9: {
      // All 9 cells in a 3x3 grid
      std::vector<Eigen::Vector2i> offsets;
      for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
          offsets.push_back(Eigen::Vector2i(i, j));
        }
      }
      return offsets;
    }

    default:
      std::cerr << "error: invalid neighbor cell mode " << neighbor_cell_mode << std::endl;
      std::cerr << "     : neighbor cell mode must be 1, 5, or 9" << std::endl;
      return std::vector<Eigen::Vector2i>();
  }
}

template <typename CellContents>
bool IncrementalGridMap2D<CellContents>::has_points() const {
  return flat_cells.empty() ? false : frame::has_points(flat_cells.front()->second);
}

template <typename CellContents>
bool IncrementalGridMap2D<CellContents>::has_normals() const {
  return flat_cells.empty() ? false : frame::has_normals(flat_cells.front()->second);
}

template <typename CellContents>
bool IncrementalGridMap2D<CellContents>::has_covs() const {
  return flat_cells.empty() ? false : frame::has_covs(flat_cells.front()->second);
}

template <typename CellContents>
bool IncrementalGridMap2D<CellContents>::has_intensities() const {
  return flat_cells.empty() ? false : frame::has_intensities(flat_cells.front()->second);
}

template <typename CellContents>
std::vector<Eigen::Vector3d> IncrementalGridMap2D<CellContents>::cell_points() const {
  std::vector<Eigen::Vector3d> points;
  points.reserve(flat_cells.size() * 10);
  visit_points([&](const auto& cell, const int i) { points.emplace_back(frame::point(cell, i)); });
  return points;
}

template <typename CellContents>
std::vector<Eigen::Vector3d> IncrementalGridMap2D<CellContents>::cell_normals() const {
  std::vector<Eigen::Vector3d> normals;
  normals.reserve(flat_cells.size() * 10);
  visit_points([&](const auto& cell, const int i) { normals.emplace_back(frame::normal(cell, i)); });
  return normals;
}

template <typename CellContents>
std::vector<Eigen::Matrix3d> IncrementalGridMap2D<CellContents>::cell_covs() const {
  std::vector<Eigen::Matrix3d> covs;
  covs.reserve(flat_cells.size() * 10);
  visit_points([&](const auto& cell, const int i) { covs.emplace_back(frame::cov(cell, i)); });
  return covs;
}

template <typename CellContents>
std::vector<double> IncrementalGridMap2D<CellContents>::cell_intensities() const {
  std::vector<double> intensities;
  intensities.reserve(flat_cells.size() * 10);
  visit_points([&](const auto& cell, const int i) { intensities.emplace_back(frame::intensity(cell, i)); });
  return intensities;
}

template <typename CellContents>
PointCloud2DCPU::Ptr IncrementalGridMap2D<CellContents>::cell_data() const {
  auto frame = std::make_shared<PointCloud2DCPU>();
  frame->points_storage.reserve(flat_cells.size() * 10);
  if (has_normals()) {
    frame->normals_storage.reserve(flat_cells.size() * 10);
  }
  if (has_covs()) {
    frame->covs_storage.reserve(flat_cells.size() * 10);
  }
  if (has_intensities()) {
    frame->intensities_storage.reserve(flat_cells.size() * 10);
  }

  visit_points([&](const auto& cell, const int i) {
    frame->points_storage.emplace_back(frame::point(cell, i));
    if (has_normals()) {
      frame->normals_storage.emplace_back(frame::normal(cell, i));
    }
    if (has_covs()) {
      frame->covs_storage.emplace_back(frame::cov(cell, i));
    }
    if (has_intensities()) {
      frame->intensities_storage.emplace_back(frame::intensity(cell, i));
    }
  });

  frame->num_points = frame->points_storage.size();
  frame->points = frame->points_storage.data();
  frame->normals = frame->normals_storage.empty() ? nullptr : frame->normals_storage.data();
  frame->covs = frame->covs_storage.empty() ? nullptr : frame->covs_storage.data();
  frame->intensities = frame->intensities_storage.empty() ? nullptr : frame->intensities_storage.data();

  return frame;
}

}  // namespace gtsam_points
