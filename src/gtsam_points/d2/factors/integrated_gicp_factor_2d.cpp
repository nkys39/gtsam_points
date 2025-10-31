// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/types/point_cloud_2d.hpp>
#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/types/laser_scan.hpp>
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/impl/integrated_gicp_factor_2d_impl.hpp>

// Explicit template instantiations for 2D GICP factors
template class gtsam_points::IntegratedGICPFactor2D_<gtsam_points::PointCloud2D, gtsam_points::PointCloud2D>;
template class gtsam_points::IntegratedGICPFactor2D_<gtsam_points::PointCloud2DCPU, gtsam_points::PointCloud2DCPU>;
template class gtsam_points::IntegratedGICPFactor2D_<gtsam_points::LaserScan, gtsam_points::LaserScan>;
template class gtsam_points::IntegratedGICPFactor2D_<gtsam_points::PointCloud2D, gtsam_points::LaserScan>;
