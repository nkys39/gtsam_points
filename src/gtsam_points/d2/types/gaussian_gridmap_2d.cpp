// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#include <gtsam_points/d2/types/gaussian_gridmap_2d.hpp>

#include <memory>
#include <fstream>
#include <iostream>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam_points/util/fast_floor.hpp>
#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>
#include <gtsam_points/d2/ann/impl/incremental_gridmap_2d_impl.hpp>

namespace gtsam_points {

// Explicit template instantiation
template class IncrementalGridMap2D<GaussianGridCell2D>;

// GaussianGridCell2D implementation
void GaussianGridCell2D::add(const PointCloud2D& points, size_t i) {
  if (finalized) {
    this->finalized = false;
    this->mean *= num_points;
    this->cov *= num_points;
  }

  num_points++;
  this->mean += points.points[i];
  this->cov += points.covs ? points.covs[i] : Eigen::Matrix3d::Zero();
}

void GaussianGridCell2D::finalize() {
  if (finalized) {
    return;
  }

  mean /= num_points;
  cov /= num_points;
  finalized = true;
}

// GaussianGridMap2D implementation
GaussianGridMap2D::GaussianGridMap2D(double resolution) : IncrementalGridMap2D<GaussianGridCell2D>(resolution) {
  offsets = neighbor_offsets(1);
}

GaussianGridMap2D::~GaussianGridMap2D() {}

double GaussianGridMap2D::grid_resolution() const {
  return resolution();
}

Eigen::Vector2i GaussianGridMap2D::grid_coord(const Eigen::Vector3d& x) const {
  return fast_floor(x * inv_resolution).head<2>();
}

int GaussianGridMap2D::lookup_cell_index(const Eigen::Vector2i& coord) const {
  auto found = cells.find(coord);
  if (found == cells.end()) {
    return -1;
  }
  return found->second;
}

const GaussianGridCell2D& GaussianGridMap2D::lookup_cell(int cell_id) const {
  return flat_cells[cell_id]->second;
}

void GaussianGridMap2D::insert(const PointCloud2D& frame) {
  IncrementalGridMap2D<GaussianGridCell2D>::insert(frame);
}

void GaussianGridMap2D::save_compact(const std::string& path) const {
  std::ofstream ofs(path);
  ofs << "compact " << 1 << std::endl;
  ofs << "resolution " << grid_resolution() << std::endl;
  ofs << "lru_count " << lru_counter << std::endl;
  ofs << "lru_cycle " << lru_clear_cycle << std::endl;
  ofs << "lru_thresh " << lru_horizon << std::endl;
  ofs << "num_cells " << flat_cells.size() << std::endl;

  for (const auto& cell : flat_cells) {
    ofs << cell->first.coord.x() << " " << cell->first.coord.y() << " ";
    ofs << cell->second.num_points << " ";
    ofs << cell->second.finalized << " ";
    ofs << cell->second.mean.x() << " " << cell->second.mean.y() << " " << cell->second.mean.z() << " ";

    const Eigen::Matrix2d cov_2d = cell->second.cov.block<2, 2>(0, 0);
    ofs << cov_2d(0, 0) << " " << cov_2d(0, 1) << " " << cov_2d(1, 0) << " " << cov_2d(1, 1) << std::endl;
  }
}

GaussianGridMap2D::Ptr GaussianGridMap2D::load(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) {
    std::cerr << "error: failed to open " << path << std::endl;
    return nullptr;
  }

  auto gridmap = std::make_shared<GaussianGridMap2D>(1.0);

  std::string token;
  double resolution_val;
  bool compact;
  int num_cells;

  ifs >> token >> compact;
  ifs >> token >> resolution_val;
  ifs >> token >> gridmap->lru_counter;
  ifs >> token >> gridmap->lru_clear_cycle;
  ifs >> token >> gridmap->lru_horizon;
  ifs >> token >> num_cells;

  gridmap->set_grid_resolution(resolution_val);
  gridmap->flat_cells.reserve(num_cells * 2);

  for (int i = 0; i < num_cells; i++) {
    Eigen::Vector2i coord;
    size_t num_pts;
    bool fin;
    Eigen::Vector3d mean;
    Eigen::Matrix2d cov_2d;

    ifs >> coord.x() >> coord.y();
    ifs >> num_pts >> fin;
    ifs >> mean.x() >> mean.y() >> mean.z();
    ifs >> cov_2d(0, 0) >> cov_2d(0, 1) >> cov_2d(1, 0) >> cov_2d(1, 1);

    auto cell = std::make_shared<std::pair<GridCellInfo2D, GaussianGridCell2D>>(
      GridCellInfo2D(coord, gridmap->lru_counter),
      GaussianGridCell2D());

    cell->second.num_points = num_pts;
    cell->second.finalized = fin;
    cell->second.mean = mean;
    cell->second.cov.setZero();
    cell->second.cov.block<2, 2>(0, 0) = cov_2d;

    gridmap->flat_cells.emplace_back(cell);
    gridmap->cells[coord] = gridmap->flat_cells.size() - 1;
  }

  return gridmap;
}

}  // namespace gtsam_points
