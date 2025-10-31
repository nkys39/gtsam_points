// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#include <gtsam_points/d2/segmentation/region_growing_2d.hpp>
#include <gtsam_points/d2/segmentation/impl/region_growing_2d_impl.hpp>

namespace gtsam_points {

RegionGrowingContext2D region_growing_init_2d(
  const PointCloud2D& points,
  const NearestNeighborSearch2D& search,
  const Eigen::Vector3d& seed_point,
  const RegionGrowingParams2D& params) {
  return region_growing_init_2d_(points, search, seed_point, params);
}

bool region_growing_update_2d(
  RegionGrowingContext2D& context,
  const PointCloud2D& points,
  const NearestNeighborSearch2D& search,
  const RegionGrowingParams2D& params) {
  return region_growing_update_2d_(context, points, search, params);
}

}  // namespace gtsam_points
