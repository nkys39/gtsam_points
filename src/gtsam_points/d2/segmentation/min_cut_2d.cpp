// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#include <gtsam_points/d2/segmentation/min_cut_2d.hpp>
#include <gtsam_points/d2/segmentation/impl/min_cut_2d_impl.hpp>

namespace gtsam_points {

MinCutResult2D min_cut_2d(const PointCloud2D& points, const NearestNeighborSearch2D& search, size_t source_pt_index, const MinCutParams2D& params) {
  return min_cut_2d_(points, search, source_pt_index, params);
}

MinCutResult2D min_cut_2d(const PointCloud2D& points, const NearestNeighborSearch2D& search, const Eigen::Vector3d& source_pt, const MinCutParams2D& params) {
  return min_cut_2d_(points, search, source_pt, params);
}

}  // namespace gtsam_points
