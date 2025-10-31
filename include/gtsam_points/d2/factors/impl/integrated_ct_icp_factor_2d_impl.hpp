// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#include <gtsam_points/d2/factors/integrated_ct_icp_factor_2d.hpp>

#include <gtsam/linear/HessianFactor.h>
#include <gtsam_points/config.hpp>
#include <gtsam_points/d2/ann/kdtree_2d.hpp>
#include <gtsam_points/util/parallelism.hpp>
#include <gtsam_points/d2/factors/impl/scan_matching_reduction_2d.hpp>

#ifdef GTSAM_POINTS_USE_TBB
#include <tbb/parallel_for.h>
#endif

namespace gtsam_points {

template <typename TargetFrame, typename SourceFrame>
IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::IntegratedCT_ICPFactor2D_(
  gtsam::Key source_t0_key,
  gtsam::Key source_t1_key,
  const std::shared_ptr<const TargetFrame>& target,
  const std::shared_ptr<const SourceFrame>& source,
  const std::shared_ptr<const NearestNeighborSearch2D>& target_tree)
: gtsam::NonlinearFactor(gtsam::KeyVector{source_t0_key, source_t1_key}),
  num_threads(1),
  max_correspondence_distance_sq(1.0),
  target(target),
  source(source) {
  //
  if (!frame::has_points(*target)) {
    std::cerr << "error: target frame doesn't have required attributes for ct_icp_2d" << std::endl;
    abort();
  }

  if (!frame::has_points(*source) || !frame::has_times(*source)) {
    std::cerr << "error: source frame doesn't have required attributes for ct_icp_2d (needs points and timestamps)" << std::endl;
    abort();
  }

  // Build time table: group timestamps and normalize to [0, 1]
  time_table.reserve(frame::size(*source) / 10);
  time_indices.reserve(frame::size(*source));

  const double time_eps = 1e-3;
  for (int i = 0; i < frame::size(*source); i++) {
    const double t = frame::time(*source, i);
    if (time_table.empty() || t - time_table.back() > time_eps) {
      time_table.push_back(t);
    }
    time_indices.push_back(time_table.size() - 1);
  }

  // Normalize time table to [0, 1]
  for (auto& t : time_table) {
    t = t / std::max(1e-9, time_table.back());
  }

  if (target_tree) {
    this->target_tree = target_tree;
  } else {
    this->target_tree.reset(new KdTree2D<TargetFrame>(target));
  }
}

template <typename TargetFrame, typename SourceFrame>
IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::IntegratedCT_ICPFactor2D_(
  gtsam::Key source_t0_key,
  gtsam::Key source_t1_key,
  const std::shared_ptr<const TargetFrame>& target,
  const std::shared_ptr<const SourceFrame>& source)
: IntegratedCT_ICPFactor2D_(source_t0_key, source_t1_key, target, source, nullptr) {}

template <typename TargetFrame, typename SourceFrame>
IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::~IntegratedCT_ICPFactor2D_() {}

template <typename TargetFrame, typename SourceFrame>
void IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::print(const std::string& s, const gtsam::KeyFormatter& keyFormatter) const {
  std::cout << s << "IntegratedCT_ICPFactor2D";
  std::cout << "(" << keyFormatter(this->keys()[0]) << ", " << keyFormatter(this->keys()[1]) << ")" << std::endl;

  std::cout << "|target|=" << frame::size(*target) << "pts, |source|=" << frame::size(*source) << "pts" << std::endl;
  std::cout << "num_threads=" << num_threads << ", max_corr_dist=" << std::sqrt(max_correspondence_distance_sq) << std::endl;
}

template <typename TargetFrame, typename SourceFrame>
size_t IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::memory_usage() const {
  return sizeof(*this) + sizeof(double) * time_table.capacity() + sizeof(gtsam::Pose2) * source_poses.capacity() +
         sizeof(gtsam::Matrix3) * pose_derivatives_t0.capacity() + sizeof(gtsam::Matrix3) * pose_derivatives_t1.capacity() +
         sizeof(long) * correspondences.capacity() + sizeof(int) * time_indices.capacity();
}

template <typename TargetFrame, typename SourceFrame>
double IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::error(const gtsam::Values& values) const {
  update_poses(values);
  if (correspondences.size() != frame::size(*source)) {
    update_correspondences();
  }

  const auto perpoint_task = [&](
                               int i,
                               Eigen::Matrix<double, 3, 3>* H_target,
                               Eigen::Matrix<double, 3, 3>* H_source,
                               Eigen::Matrix<double, 3, 3>* H_target_source,
                               Eigen::Matrix<double, 3, 1>* b_target,
                               Eigen::Matrix<double, 3, 1>* b_source) {
    const long target_index = correspondences[i];
    if (target_index < 0) {
      return 0.0;
    }

    const int time_index = time_indices[i];
    const auto& pose = source_poses[time_index];

    const auto& source_pt = frame::point(*source, i);
    const auto& target_pt = frame::point(*target, target_index);
    const auto& target_normal = frame::normal(*target, target_index);

    // Transform source point using interpolated pose
    gtsam::Point2 transed_source_pt = pose.transformFrom(source_pt.head<2>().eval());
    gtsam::Point2 residual = transed_source_pt - target_pt.head<2>();

    // Point-to-line error (2D equivalent of point-to-plane)
    double error = gtsam::dot(residual, target_normal.head<2>());

    return error * error;
  };

  if (is_omp_default() || num_threads == 1) {
    return scan_matching_reduce_omp_2d(perpoint_task, frame::size(*this->source), this->num_threads, nullptr, nullptr, nullptr, nullptr, nullptr);
  } else {
    return scan_matching_reduce_tbb_2d(perpoint_task, frame::size(*this->source), nullptr, nullptr, nullptr, nullptr, nullptr);
  }
}

template <typename TargetFrame, typename SourceFrame>
std::shared_ptr<gtsam::GaussianFactor> IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::linearize(const gtsam::Values& values) const {
  if (!frame::has_normals(*target)) {
    std::cerr << "error: target cloud doesn't have normals!!" << std::endl;
    abort();
  }

  update_poses(values);
  update_correspondences();

  const auto perpoint_task = [&](
                               int i,
                               Eigen::Matrix<double, 3, 3>* H_00,
                               Eigen::Matrix<double, 3, 3>* H_11,
                               Eigen::Matrix<double, 3, 3>* H_01,
                               Eigen::Matrix<double, 3, 1>* b_0,
                               Eigen::Matrix<double, 3, 1>* b_1) {
    const long target_index = correspondences[i];
    if (target_index < 0) {
      return 0.0;
    }

    const int time_index = time_indices[i];

    const auto& pose = source_poses[time_index];
    const auto& H_pose_0 = pose_derivatives_t0[time_index];
    const auto& H_pose_1 = pose_derivatives_t1[time_index];

    const auto& source_pt = frame::point(*source, i);
    const auto& target_pt = frame::point(*target, target_index);
    const auto& target_normal = frame::normal(*target, target_index);

    // Transform source point with Jacobian w.r.t. pose
    gtsam::Matrix23 H_transed_pose;
    gtsam::Point2 transed_source_pt = pose.transformFrom(source_pt.head<2>(), H_transed_pose);

    gtsam::Point2 residual = transed_source_pt - target_pt.head<2>();

    // Point-to-line error with Jacobian
    gtsam::Matrix12 H_error_transed;
    double error = gtsam::dot(residual, target_normal.head<2>(), H_error_transed);

    // Chain rule: error w.r.t. pose and then w.r.t. t0/t1
    gtsam::Matrix13 H_error_pose = H_error_transed * H_transed_pose;
    gtsam::Matrix13 H_0 = H_error_pose * H_pose_0;
    gtsam::Matrix13 H_1 = H_error_pose * H_pose_1;

    *H_00 += H_0.transpose() * H_0;
    *H_11 += H_1.transpose() * H_1;
    *H_01 += H_0.transpose() * H_1;
    *b_0 += H_0.transpose() * error;
    *b_1 += H_1.transpose() * error;

    return error * error;
  };

  double error = 0.0;
  gtsam::Matrix3 H_00;
  gtsam::Matrix3 H_01;
  gtsam::Matrix3 H_11;
  gtsam::Vector3 b_0;
  gtsam::Vector3 b_1;

  if (is_omp_default() || num_threads == 1) {
    error = scan_matching_reduce_omp_2d(perpoint_task, frame::size(*this->source), this->num_threads, &H_00, &H_11, &H_01, &b_0, &b_1);
  } else {
    error = scan_matching_reduce_tbb_2d(perpoint_task, frame::size(*this->source), &H_00, &H_11, &H_01, &b_0, &b_1);
  }

  auto factor = gtsam::HessianFactor::shared_ptr(new gtsam::HessianFactor(this->keys_[0], this->keys_[1], H_00, H_01, -b_0, H_11, -b_1, error));
  return factor;
}

template <typename TargetFrame, typename SourceFrame>
void IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::update_poses(const gtsam::Values& values) const {
  gtsam::Pose2 pose0 = values.at<gtsam::Pose2>(keys_[0]);
  gtsam::Pose2 pose1 = values.at<gtsam::Pose2>(keys_[1]);

  // Compute relative motion between t0 and t1
  gtsam::Matrix3 H_delta_0, H_delta_1;
  gtsam::Pose2 delta = pose0.between(pose1, H_delta_0, H_delta_1);

  // Convert to velocity (tangent space) - 3 DOF for Pose2
  gtsam::Matrix3 H_vel_delta;
  gtsam::Vector3 vel = gtsam::Pose2::Logmap(delta, H_vel_delta);

  source_poses.resize(time_table.size());
  pose_derivatives_t0.resize(time_table.size());
  pose_derivatives_t1.resize(time_table.size());

  const auto task = [&](int i) {
    const double t = time_table[i];

    // Interpolate pose using exponential map: pose(t) = pose0 * exp(t * vel)
    gtsam::Matrix3 H_inc_vel;
    gtsam::Pose2 inc = gtsam::Pose2::Expmap(t * vel, H_inc_vel);

    gtsam::Matrix3 H_pose_0_a, H_pose_inc;
    source_poses[i] = pose0.compose(inc, H_pose_0_a, H_pose_inc);

    // Chain rule for derivatives
    gtsam::Matrix3 H_pose_delta = H_pose_inc * H_inc_vel * t * H_vel_delta;

    pose_derivatives_t0[i] = H_pose_0_a + H_pose_delta * H_delta_0;
    pose_derivatives_t1[i] = H_pose_delta * H_delta_1;
  };

  if (is_omp_default() || num_threads == 1) {
#pragma omp parallel for num_threads(this->num_threads) schedule(guided, 32)
    for (int i = 0; i < time_table.size(); i++) {
      task(i);
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, time_table.size(), 32), [&](const tbb::blocked_range<int>& range) {
      for (int i = range.begin(); i < range.end(); i++) {
        task(i);
      }
    });
#else
    std::cerr << "error: TBB is not available" << std::endl;
    abort();
#endif
  }
}

template <typename TargetFrame, typename SourceFrame>
void IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::update_correspondences() const {
  correspondences.resize(frame::size(*source));

  const auto perpoint_task = [&](int i) {
    const int time_index = time_indices[i];

    const auto& pt = frame::point(*source, i);
    // Transform point using interpolated pose
    gtsam::Point2 transed_pt = source_poses[time_index] * pt.head<2>();

    size_t k_index = -1;
    double k_sq_dist = -1;
    // Convert Point2 to Vector3d with homogeneous coordinates for NN search
    Eigen::Vector3d search_pt(transed_pt.x(), transed_pt.y(), 1.0);
    size_t num_found = target_tree->knn_search(search_pt.data(), 1, &k_index, &k_sq_dist, max_correspondence_distance_sq);

    if (num_found == 0 || k_sq_dist > max_correspondence_distance_sq) {
      correspondences[i] = -1;
    } else {
      correspondences[i] = k_index;
    }
  };

  if (is_omp_default() || num_threads == 1) {
#pragma omp parallel for num_threads(this->num_threads) schedule(guided, 8)
    for (int i = 0; i < frame::size(*this->source); i++) {
      perpoint_task(i);
    }
  } else {
#ifdef GTSAM_POINTS_USE_TBB
    tbb::parallel_for(tbb::blocked_range<int>(0, frame::size(*this->source), 8), [&](const tbb::blocked_range<int>& range) {
      for (int i = range.begin(); i < range.end(); i++) {
        perpoint_task(i);
      }
    });
#else
    std::cerr << "error: TBB is not available" << std::endl;
    abort();
#endif
  }
}

template <typename TargetFrame, typename SourceFrame>
std::vector<Eigen::Vector3d> IntegratedCT_ICPFactor2D_<TargetFrame, SourceFrame>::deskewed_source_points(const gtsam::Values& values, bool local) {
  update_poses(values);

  if (local) {
    for (auto& pose : source_poses) {
      pose = values.at<gtsam::Pose2>(keys_[0]).inverse() * pose;
    }
  }

  std::vector<Eigen::Vector3d> deskewed(frame::size(*source));
  for (int i = 0; i < frame::size(*source); i++) {
    const int time_index = time_indices[i];
    const auto& pose = source_poses[time_index];
    // Transform and store in homogeneous coordinates
    gtsam::Point2 pt_2d = pose.matrix() * frame::point(*source, i).head<2>();
    deskewed[i] = Eigen::Vector3d(pt_2d.x(), pt_2d.y(), 1.0);
  }

  return deskewed;
}

}  // namespace gtsam_points
