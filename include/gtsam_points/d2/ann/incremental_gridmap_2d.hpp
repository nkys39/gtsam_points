// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)
#pragma once

#include <Eigen/Core>
#include <gtsam_points/d2/util/vector2i_hash.hpp>
#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/ann/nearest_neighbor_search_2d.hpp>

namespace gtsam_points {

/// @brief Grid cell meta information for 2D.
struct GridCellInfo2D {
public:
  /// @brief Default constructor.
  GridCellInfo2D() : lru(0), coord(-1, -1) {}

  /// @brief Constructor.
  /// @param coord Integer grid coordinates
  /// @param lru   LRU counter for caching
  GridCellInfo2D(const Eigen::Vector2i& coord, size_t lru) : lru(lru), coord(coord) {}

public:
  size_t lru;             ///< Last used time
  Eigen::Vector2i coord;  ///< Grid cell coordinate
};

/// @brief Incremental grid map for 2D.
///        This class supports incremental point cloud insertion and LRU-based cell deletion.
/// @note  This class can be used as a point cloud as well as a neighbor search structure.
/// @note  For the compatibility with other nearest neighbor search methods, this implementation returns indices that encode the cell and point IDs.
///        The first ```point_id_bits``` (e.g., 32) bits of a point index represent the point ID, and the rest ```cell_id_bits``` (e.g., 32) bits
///        represent the cell ID that contains the point. The specified point can be looked up by `gridmap.point(index)`;
template <typename CellContents>
struct IncrementalGridMap2D : public NearestNeighborSearch2D {
public:
  using Ptr = std::shared_ptr<IncrementalGridMap2D>;
  using ConstPtr = std::shared_ptr<const IncrementalGridMap2D>;

  /// @brief Constructor.
  /// @param resolution  Cell size
  explicit IncrementalGridMap2D(double resolution);
  virtual ~IncrementalGridMap2D();

  /// @brief Cell resolution.
  void set_grid_resolution(const double resolution) { inv_resolution = 1.0 / resolution; }
  /// @brief LRU cache clearing cycle.
  void set_lru_clear_cycle(const int lru_clear_cycle) { this->lru_clear_cycle = lru_clear_cycle; }
  /// @brief LRU cache horizon.
  void set_lru_horizon(const int lru_horizon) { this->lru_horizon = lru_horizon; }
  /// @brief Neighboring cell search mode (1, 5, or 9).
  void set_neighbor_cell_mode(const int mode) { offsets = neighbor_offsets(mode); }
  /// @brief Cell setting.
  typename CellContents::Setting& cell_insertion_setting() { return cell_setting; }

  /// @brief Cell size.
  double resolution() const { return 1.0 / inv_resolution; }

  /// @brief Number of cells in the grid map.
  size_t num_cells() const { return flat_cells.size(); }

  /// @brief Clear the grid map.
  virtual void clear();

  /// @brief Insert points to the grid map.
  /// @param points Point cloud
  virtual void insert(const PointCloud2D& points);

  /// @brief  Find k nearest neighbors.
  /// @param pt           Query point
  /// @param k            Number of neighbors to search
  /// @param k_indices    Indices of the k nearest neighbors
  /// @param k_sq_dists   Squared distances of the k nearest neighbors
  /// @return             Number of found neighbors
  virtual size_t knn_search(
    const double* pt,
    size_t k,
    size_t* k_indices,
    double* k_sq_dists,
    double max_sq_dist = std::numeric_limits<double>::max()) const override;

  /// @brief Calculate the global point index from the cell index and the point index.
  inline size_t calc_index(const size_t cell_id, const size_t point_id) const { return (cell_id << point_id_bits) | point_id; }
  inline size_t cell_id(const size_t i) const { return i >> point_id_bits; }                 ///< Extract the cell ID from a global index.
  inline size_t point_id(const size_t i) const { return i & ((1ull << point_id_bits) - 1); }  ///< Extract the point ID from a global index.

  bool has_points() const;
  bool has_normals() const;
  bool has_covs() const;
  bool has_intensities() const;

  decltype(auto) point(const size_t i) const { return frame::point(flat_cells[cell_id(i)]->second, point_id(i)); }
  decltype(auto) normal(const size_t i) const { return frame::normal(flat_cells[cell_id(i)]->second, point_id(i)); }
  decltype(auto) cov(const size_t i) const { return frame::cov(flat_cells[cell_id(i)]->second, point_id(i)); }
  decltype(auto) intensity(const size_t i) const { return frame::intensity(flat_cells[cell_id(i)]->second, point_id(i)); }

  virtual std::vector<Eigen::Vector3d> cell_points() const;
  virtual std::vector<Eigen::Vector3d> cell_normals() const;
  virtual std::vector<Eigen::Matrix3d> cell_covs() const;
  virtual std::vector<double> cell_intensities() const;

  virtual PointCloud2DCPU::Ptr cell_data() const;

protected:
  std::vector<Eigen::Vector2i> neighbor_offsets(const int neighbor_cell_mode) const;

  template <typename Func>
  void visit_points(const Func& f) const {
    for (const auto& cell : flat_cells) {
      for (int i = 0; i < frame::size(cell->second); i++) {
        f(cell->second, i);
      }
    }
  }

protected:
  static_assert(sizeof(size_t) == 8, "size_t must be 64-bit");
  static constexpr int point_id_bits = 32;                  ///< Use the first 32 bits for point id
  static constexpr int cell_id_bits = 64 - point_id_bits;  ///< Use the remaining bits for cell id
  double inv_resolution;                                    ///< Inverse of the cell size
  std::vector<Eigen::Vector2i> offsets;                     ///< Neighbor cell offsets

  size_t lru_horizon;      ///< LRU horizon size. Cells that have not been accessed for lru_horizon steps are deleted.
  size_t lru_clear_cycle;  ///< LRU clear cycle. Cell deletion is performed every lru_clear_cycle steps.
  size_t lru_counter;      ///< LRU counter. Incremented every step.

  typename CellContents::Setting cell_setting;                                  ///< Cell setting.
  std::vector<std::shared_ptr<std::pair<GridCellInfo2D, CellContents>>> flat_cells;  ///< Cell contents.
  std::unordered_map<Eigen::Vector2i, size_t, XORVector2iHash> cells;            ///< Cell index map.
};

namespace frame {

template <typename CellContents>
struct traits<IncrementalGridMap2D<CellContents>> {
  static bool has_points(const IncrementalGridMap2D<CellContents>& igrid) { return igrid.has_points(); }
  static bool has_normals(const IncrementalGridMap2D<CellContents>& igrid) { return igrid.has_normals(); }
  static bool has_covs(const IncrementalGridMap2D<CellContents>& igrid) { return igrid.has_covs(); }
  static bool has_intensities(const IncrementalGridMap2D<CellContents>& igrid) { return igrid.has_intensities(); }

  static const Eigen::Vector3d& point(const IncrementalGridMap2D<CellContents>& igrid, size_t i) { return igrid.point(i); }
  static const Eigen::Vector3d& normal(const IncrementalGridMap2D<CellContents>& igrid, size_t i) { return igrid.normal(i); }
  static const Eigen::Matrix3d& cov(const IncrementalGridMap2D<CellContents>& igrid, size_t i) { return igrid.cov(i); }
  static double intensity(const IncrementalGridMap2D<CellContents>& igrid, size_t i) { return igrid.intensity(i); }
};

}  // namespace frame

}  // namespace gtsam_points
