// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/types/point_cloud_2d.hpp>
#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/types/laser_scan.hpp>
#include <gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/impl/integrated_vgicp_factor_2d_impl.hpp>

// Explicit template instantiations for 2D VGICP factors
template class gtsam_points::IntegratedVGICPFactor2D_<gtsam_points::PointCloud2D>;
template class gtsam_points::IntegratedVGICPFactor2D_<gtsam_points::PointCloud2DCPU>;
template class gtsam_points::IntegratedVGICPFactor2D_<gtsam_points::LaserScan>;
