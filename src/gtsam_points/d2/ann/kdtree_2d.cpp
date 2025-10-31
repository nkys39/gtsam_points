// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/ann/kdtree_2d.hpp>

#include <gtsam_points/config.hpp>
#include <gtsam_points/d2/types/frame_traits_2d.hpp>
#include <gtsam_points/ann/small_kdtree.hpp>
#include <gtsam_points/util/parallelism.hpp>

namespace gtsam_points {

// Frame traits for KdTree2D to work with UnsafeKdTree template
namespace frame {

template <>
struct traits<KdTree2D> {
  static int size(const KdTree2D& tree) { return tree.num_points; }
  static bool has_points(const KdTree2D& tree) { return tree.num_points; }
  static const Eigen::Vector3d& point(const KdTree2D& tree, size_t i) { return tree.points[i]; }
};

}  // namespace frame

// AxisAlignedProjection2D implementation
template <typename PointCloud, typename IndexConstIterator>
AxisAlignedProjection2D AxisAlignedProjection2D::find_axis(
  const PointCloud& points,
  IndexConstIterator first,
  IndexConstIterator last,
  const ProjectionSetting& setting) {

  const size_t N = std::distance(first, last);
  Eigen::Vector3d sum_pt = Eigen::Vector3d::Zero();
  Eigen::Vector3d sum_sq = Eigen::Vector3d::Zero();

  const size_t step = N < setting.max_scan_count ? 1 : N / setting.max_scan_count;
  const size_t num_steps = N / step;

  for (int i = 0; i < num_steps; i++) {
    const auto itr = first + step * i;
    const Eigen::Vector3d pt = frame::point(points, *itr);
    sum_pt += pt;
    sum_sq += pt.cwiseProduct(pt);
  }

  // For 2D points in homogeneous coordinates (x, y, 1)
  // We only consider X and Y axes for splitting
  const Eigen::Vector3d mean = sum_pt / sum_pt.z();  // Normalize by homogeneous coordinate
  const Eigen::Vector3d var = (sum_sq - mean.cwiseProduct(sum_pt));

  // Select axis with largest variance (0: X, 1: Y)
  // Ignore Z axis (index 2) as it's always 1 for homogeneous coordinates
  return AxisAlignedProjection2D{var[0] > var[1] ? 0 : 1};
}

// KdTree2D implementation
KdTree2D::KdTree2D(const Eigen::Vector3d* points, int num_points, int build_num_threads)
: num_points(num_points),
  points(points),
  search_eps(-1.0),
  index(
    is_omp_default() || build_num_threads == 1 ?             //
      new Index(*this, KdTreeBuilderOMP(build_num_threads))  //
                                               :             //
#ifdef GTSAM_POINTS_USE_TBB                                  //
      new Index(*this, KdTreeBuilderTBB())                   //
#else                                                        //
      new Index(*this, KdTreeBuilder())
#endif
  ) {
}

KdTree2D::~KdTree2D() {}

size_t KdTree2D::knn_search(
  const double* pt,
  size_t k,
  size_t* k_indices,
  double* k_sq_dists,
  double max_sq_dist) const {

  KnnSetting setting;
  setting.max_sq_dist = max_sq_dist;

  // Map to 2D vector (only x, y coordinates, ignore homogeneous coordinate)
  if (k == 1) {
    return index->nearest_neighbor_search(Eigen::Map<const Eigen::Vector2d>(pt), k_indices, k_sq_dists, setting);
  } else {
    return index->knn_search(Eigen::Map<const Eigen::Vector2d>(pt), k, k_indices, k_sq_dists, setting);
  }
}

size_t KdTree2D::radius_search(
  const double* pt,
  double radius,
  std::vector<size_t>& indices,
  std::vector<double>& sq_dists,
  int max_num_neighbors) const {

  KnnSetting setting;
  setting.max_nn = max_num_neighbors;

  // Map to 2D vector (only x, y coordinates)
  return index->radius_search(Eigen::Map<const Eigen::Vector2d>(pt), radius, indices, sq_dists, setting);
}

}  // namespace gtsam_points
