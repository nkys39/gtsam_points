// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>
#include <gtsam_points/d2/types/point_cloud_2d.hpp>
#include <gtsam_points/d2/types/gaussian_gridmap_2d.hpp>
#include <gtsam_points/d2/factors/integrated_ct_icp_factor_2d.hpp>
#include <gtsam_points/d2/factors/impl/integrated_ct_icp_factor_2d_impl.hpp>

// Explicit template instantiations for common 2D frame types
template class gtsam_points::IntegratedCT_ICPFactor2D_<gtsam_points::GaussianGridMap2D, gtsam_points::PointCloud2D>;
template class gtsam_points::IntegratedCT_ICPFactor2D_<gtsam_points::PointCloud2D, gtsam_points::PointCloud2D>;
