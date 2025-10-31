# 2D SLAM Migration Plan - gtsam_points

## Project Overview
Complete migration plan for creating a 2D SLAM library based on gtsam_points architecture using Git branch strategy.

## Branch Strategy

```
main (stable 3D SLAM)
  │
  └── develop
        │
        ├── feature/2d-slam-core        (Week 1-4)
        ├── feature/2d-slam-factors     (Week 5-8)
        ├── feature/2d-slam-advanced    (Week 9-12)
        └── feature/2d-slam-optimize    (Week 13-16)
              │
              └── release/2d-slam-v1.0
```

## Directory Structure

```
gtsam_points/
├── include/gtsam_points/
│   ├── d2/                          # 2D SLAM components
│   │   ├── types/
│   │   │   ├── point_cloud_2d.hpp
│   │   │   ├── laser_scan.hpp
│   │   │   └── frame_traits_2d.hpp
│   │   ├── factors/
│   │   │   ├── integrated_matching_cost_factor_2d.hpp
│   │   │   ├── integrated_icp_factor_2d.hpp
│   │   │   ├── integrated_point_to_line_factor.hpp
│   │   │   ├── integrated_gicp_factor_2d.hpp
│   │   │   └── integrated_vgicp_factor_2d.hpp
│   │   ├── ann/
│   │   │   ├── nearest_neighbor_search_2d.hpp
│   │   │   ├── kdtree_2d.hpp
│   │   │   ├── incremental_gridmap.hpp
│   │   │   └── igrid.hpp
│   │   ├── features/
│   │   │   ├── normal_estimation_2d.hpp
│   │   │   └── covariance_estimation_2d.hpp
│   │   ├── registration/
│   │   │   ├── ransac_2d.hpp
│   │   │   ├── graduated_non_convexity_2d.hpp
│   │   │   └── alignment_2d.hpp
│   │   ├── segmentation/
│   │   │   ├── region_growing_2d.hpp
│   │   │   └── min_cut_2d.hpp
│   │   └── util/
│   │       ├── bspline_2d.hpp
│   │       └── pose2_utils.hpp
│   └── (existing 3D components)
└── src/gtsam_points/
    ├── d2/                          # 2D implementations
    │   ├── types/
    │   ├── factors/
    │   ├── ann/
    │   ├── features/
    │   ├── registration/
    │   └── segmentation/
    └── (existing 3D implementations)
```

## Component Migration List

### Priority S - Critical (Week 1-2)
| Component | 3D Source | 2D Target | Status |
|-----------|-----------|-----------|--------|
| PointCloud | types/point_cloud.hpp | d2/types/point_cloud_2d.hpp | ⏳ TODO |
| PointCloudCPU | types/point_cloud_cpu.hpp | d2/types/point_cloud_2d_cpu.hpp | ⏳ TODO |
| LaserScan | N/A (new) | d2/types/laser_scan.hpp | ⏳ TODO |
| FrameTraits | types/frame_traits.hpp | d2/types/frame_traits_2d.hpp | ⏳ TODO |
| MatchingCostFactor | factors/integrated_matching_cost_factor.hpp | d2/factors/integrated_matching_cost_factor_2d.hpp | ⏳ TODO |
| ICPFactor | factors/integrated_icp_factor.hpp | d2/factors/integrated_icp_factor_2d.hpp | ⏳ TODO |
| PointToLineFactor | N/A (new) | d2/factors/integrated_point_to_line_factor.hpp | ⏳ TODO |
| NearestNeighborSearch | ann/nearest_neighbor_search.hpp | d2/ann/nearest_neighbor_search_2d.hpp | ⏳ TODO |
| KdTree | ann/kdtree.hpp | d2/ann/kdtree_2d.hpp | ⏳ TODO |

### Priority A - Core Features (Week 3-6)
| Component | 3D Source | 2D Target | Status |
|-----------|-----------|-----------|--------|
| GICPFactor | factors/integrated_gicp_factor.hpp | d2/factors/integrated_gicp_factor_2d.hpp | ⏳ TODO |
| VGICPFactor | factors/integrated_vgicp_factor.hpp | d2/factors/integrated_vgicp_factor_2d.hpp | ⏳ TODO |
| IncrementalVoxelMap | ann/incremental_voxelmap.hpp | d2/ann/incremental_gridmap.hpp | ⏳ TODO |
| iVox | ann/ivox.hpp | d2/ann/igrid.hpp | ⏳ TODO |
| NormalEstimation | features/normal_estimation.hpp | d2/features/normal_estimation_2d.hpp | ⏳ TODO |
| CovarianceEstimation | features/covariance_estimation.hpp | d2/features/covariance_estimation_2d.hpp | ⏳ TODO |
| RANSAC | registration/ransac.hpp | d2/registration/ransac_2d.hpp | ⏳ TODO |
| Alignment | registration/alignment.hpp | d2/registration/alignment_2d.hpp | ⏳ TODO |

### Priority B - Advanced Features (Week 7-10)
| Component | 3D Source | 2D Target | Status |
|-----------|-----------|-----------|--------|
| RegionGrowing | segmentation/region_growing.hpp | d2/segmentation/region_growing_2d.hpp | ⏳ TODO |
| MinCut | segmentation/min_cut.hpp | d2/segmentation/min_cut_2d.hpp | ⏳ TODO |
| GNC | registration/graduated_non_convexity.hpp | d2/registration/graduated_non_convexity_2d.hpp | ⏳ TODO |
| FastOccupancyGrid | ann/fast_occupancy_grid.hpp | Use as-is (2D compatible) | ⏳ TODO |

### Priority C - Future Enhancements (Week 11-16)
| Component | 3D Source | 2D Target | Status |
|-----------|-----------|-----------|--------|
| CT-ICP Factor | factors/integrated_ct_icp_factor.hpp | d2/factors/integrated_ct_icp_factor_2d.hpp | ⏳ TODO |
| CT-GICP Factor | factors/integrated_ct_gicp_factor.hpp | d2/factors/integrated_ct_gicp_factor_2d.hpp | ⏳ TODO |
| B-Spline | util/bspline.hpp | d2/util/bspline_2d.hpp | ⏳ TODO |
| ContinuousTrajectory | util/continuous_trajectory.hpp | d2/util/continuous_trajectory_2d.hpp | ⏳ TODO |
| **IMU Factor (2D)** | factors/reintegrated_imu_factor.hpp | d2/factors/reintegrated_imu_factor_2d.hpp | ⏳ TODO |

**Note on IMU Factor 2D**: For planar motion, the 2D IMU factor will integrate:
- Z-axis angular velocity (yaw rate)
- XY-plane linear acceleration
- Useful for ground robots with IMU sensors

### Reusable Components (No Migration Needed)
- optimizers/levenberg_marquardt_ext.* (Pose2 compatible)
- optimizers/isam2_ext.* (Pose2 compatible)
- optimizers/linear_system_builder.*
- optimizers/gaussian_factor_graph_solver.*
- optimizers/incremental_fixed_lag_smoother_ext.*
- optimizers/dogleg_optimizer_ext.*
- optimizers/fast_scatter.*
- ann/knn_result.hpp
- ann/flat_container.hpp
- util/parallelism.cpp
- factors/linear_damping_factor.hpp

### Not Applicable for 2D
- Colored ICP factors (color-based)
- LOAM factor (3D edge/plane specific)
- FPFH estimation (3D descriptor)
- Bundle adjustment factors (typically 3D)
- GPU implementations (future consideration)

## Implementation Timeline

### Week 1-2: Foundation (Milestone 1)
**Goal**: Branch structure + Core data types

**Tasks**:
1. ✅ Create branch structure (develop, feature/2d-slam-core)
2. ⏳ Setup CMakeLists.txt with BUILD_2D_SLAM option
3. ⏳ Implement PointCloud2D
4. ⏳ Implement LaserScan
5. ⏳ Implement FrameTraits2D
6. ⏳ Basic unit tests

**Deliverables**:
- `include/gtsam_points/d2/types/point_cloud_2d.hpp`
- `src/gtsam_points/d2/types/point_cloud_2d.cpp`
- `include/gtsam_points/d2/types/laser_scan.hpp`
- `src/gtsam_points/d2/types/laser_scan.cpp`
- `tests/d2/test_point_cloud_2d.cpp`

### Week 3-4: Basic Factors (Milestone 2)
**Goal**: ICP and Point-to-Line factors

**Tasks**:
1. ⏳ Implement IntegratedMatchingCostFactor2D (base class)
2. ⏳ Implement IntegratedICPFactor2D
3. ⏳ Implement IntegratedPointToLineFactor
4. ⏳ Implement NearestNeighborSearch2D interface
5. ⏳ Implement KdTree2D
6. ⏳ Factor unit tests

**Deliverables**:
- `include/gtsam_points/d2/factors/integrated_matching_cost_factor_2d.hpp`
- `include/gtsam_points/d2/factors/integrated_icp_factor_2d.hpp`
- `include/gtsam_points/d2/factors/integrated_point_to_line_factor.hpp`
- `include/gtsam_points/d2/ann/kdtree_2d.hpp`
- `tests/d2/test_icp_factor_2d.cpp`

### Week 5-6: GICP Factors (Milestone 3)
**Goal**: Generalized ICP for 2D

**Tasks**:
1. ⏳ Implement IntegratedGICPFactor2D
2. ⏳ Implement IntegratedVGICPFactor2D
3. ⏳ GICP unit tests
4. ⏳ Benchmark ICP vs GICP

**Deliverables**:
- `include/gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp`
- `include/gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp`
- `tests/d2/test_gicp_factor_2d.cpp`

### Week 7-8: Grid Maps (Milestone 4)
**Goal**: Incremental 2D grid mapping

**Tasks**:
1. ⏳ Implement IncrementalGridMap (2D version of iVox)
2. ⏳ Implement iGrid
3. ⏳ Grid map unit tests
4. ⏳ Integration with VGICP factor

**Deliverables**:
- `include/gtsam_points/d2/ann/incremental_gridmap.hpp`
- `include/gtsam_points/d2/ann/igrid.hpp`
- `tests/d2/test_incremental_gridmap.cpp`

### Week 9-10: Features & Registration (Milestone 5)
**Goal**: Feature extraction and global registration

**Tasks**:
1. ⏳ Implement NormalEstimation2D
2. ⏳ Implement CovarianceEstimation2D
3. ⏳ Implement RANSAC2D (3DoF)
4. ⏳ Implement GNC2D
5. ⏳ Global registration tests

**Deliverables**:
- `include/gtsam_points/d2/features/normal_estimation_2d.hpp`
- `include/gtsam_points/d2/registration/ransac_2d.hpp`
- `include/gtsam_points/d2/registration/graduated_non_convexity_2d.hpp`
- `tests/d2/test_global_registration_2d.cpp`

### Week 11-12: Segmentation (Milestone 6)
**Goal**: Point cloud segmentation

**Tasks**:
1. ⏳ Implement RegionGrowing2D
2. ⏳ Implement MinCut2D
3. ⏳ Segmentation tests

**Deliverables**:
- `include/gtsam_points/d2/segmentation/region_growing_2d.hpp`
- `include/gtsam_points/d2/segmentation/min_cut_2d.hpp`
- `tests/d2/test_segmentation_2d.cpp`

### Week 13-14: Continuous Time (Milestone 7)
**Goal**: CT-ICP for 2D

**Tasks**:
1. ⏳ Implement BSpline2D (Pose2 interpolation)
2. ⏳ Implement IntegratedCT_ICPFactor2D
3. ⏳ Implement IntegratedCT_GICPFactor2D
4. ⏳ Continuous time tests

**Deliverables**:
- `include/gtsam_points/d2/util/bspline_2d.hpp`
- `include/gtsam_points/d2/factors/integrated_ct_icp_factor_2d.hpp`
- `tests/d2/test_continuous_time_2d.cpp`

### Week 15-16: Integration & Documentation (Milestone 8)
**Goal**: Polish and release preparation

**Tasks**:
1. ⏳ Performance optimization
2. ⏳ Complete test coverage
3. ⏳ Example programs (demo_2d_slam.cpp)
4. ⏳ Documentation (Doxygen)
5. ⏳ README for 2D SLAM
6. ⏳ Prepare release/2d-slam-v1.0 branch

**Deliverables**:
- `src/demo/demo_2d_slam.cpp`
- `docs/2D_SLAM_API.md`
- `release/2d-slam-v1.0` branch
- Git tag `v1.0.0-2d`

## Key Design Decisions

### Naming Convention
- 2D components use `_2d` suffix (e.g., `IntegratedICPFactor2D`)
- Namespace: `gtsam_points::d2::` or keep `gtsam_points::` with class suffix

### Type Mappings
- `Eigen::Vector4d` → `Eigen::Vector3d` (x, y, 1 homogeneous)
- `Eigen::Matrix4d` → `Eigen::Matrix3d` (3x3 transformation)
- `Eigen::Isometry3d` → `Eigen::Isometry2d`
- `gtsam::Pose3` → `gtsam::Pose2`
- `gtsam::Rot3` → `gtsam::Rot2`

### Dimension Changes
- DOF: 6 → 3 (x, y, θ)
- Hessian: 6x6 → 3x3
- Error vector: 6x1 → 3x1
- Normals: 3D vector → 2D vector (perpendicular to line)
- Covariance: 3x3 → 2x2 (in point space)

### CMake Build Options
```cmake
option(BUILD_2D_SLAM "Build 2D SLAM components" ON)
option(BUILD_2D_TESTS "Build 2D SLAM tests" ON)
option(BUILD_2D_DEMOS "Build 2D SLAM demos" ON)
```

## Testing Strategy

### Unit Tests (per component)
- Basic functionality tests
- Edge case handling
- Memory leak checks

### Integration Tests
- Multi-scan alignment
- Loop closure detection
- Large-scale mapping

### Benchmarks
- ICP vs GICP performance
- Grid map insertion speed
- Memory usage profiling

## CI/CD Configuration

GitHub Actions workflow:
- `.github/workflows/ci-2d-slam.yml`
- Build matrix: Ubuntu 20.04, 22.04, 24.04
- Test coverage reporting
- Memory sanitizers (ASan, UBSan)

## Success Criteria

- ✅ All Priority S components implemented
- ✅ All Priority A components implemented
- ✅ Unit test coverage > 80%
- ✅ At least 1 working demo program
- ✅ Documentation complete
- ✅ CI passing on all platforms

## Future Work (Post v1.0)

- GPU acceleration for grid maps
- ROS2 integration (separate branch)
- Python bindings
- Loop closure detection
- Place recognition

## References

- Base repository: https://github.com/koide3/gtsam_points
- GTSAM documentation: https://gtsam.org/
- Related papers listed in main README.md

---

**Last Updated**: 2025-10-30
**Status**: Planning Phase
**Next Action**: Create develop and feature/2d-slam-core branches
