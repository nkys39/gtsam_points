# gtsam_points - 2D SLAM Library

## Overview

This is the 2D SLAM extension of gtsam_points, providing factors, optimizers, and utilities for 2D laser-based SLAM.

**Status**: 🚧 Under Active Development (feature/2d-slam-core branch)

## Features

### Implemented
- 🚧 Coming soon...

### Planned (See [2D_SLAM_MIGRATION_PLAN.md](./2D_SLAM_MIGRATION_PLAN.md))

#### Priority S - Core Components
- [ ] `PointCloud2D` - 2D point cloud data structure
- [ ] `LaserScan` - Laser scan with polar coordinates
- [ ] `IntegratedICPFactor2D` - 2D Point-to-Point ICP
- [ ] `IntegratedPointToLineFactor` - 2D Point-to-Line ICP
- [ ] `KdTree2D` - 2D nearest neighbor search

#### Priority A - Advanced Features
- [ ] `IntegratedGICPFactor2D` - 2D Generalized ICP
- [ ] `IntegratedVGICPFactor2D` - Grid-based GICP
- [ ] `IncrementalGridMap` - iVox-style 2D grid map
- [ ] `NormalEstimation2D` - Line fitting and normal estimation
- [ ] `RANSAC2D` - 3DoF global registration

#### Priority B - Segmentation & Registration
- [ ] `RegionGrowing2D` - Point cloud clustering
- [ ] `MinCut2D` - Graph-cut segmentation
- [ ] `GraduatedNonConvexity2D` - Robust global registration

#### Priority C - Continuous Time
- [ ] `IntegratedCT_ICPFactor2D` - Continuous-time ICP
- [ ] `BSpline2D` - Pose2 trajectory interpolation
- [ ] `ReintegratedIMUFactor2D` - 2D IMU integration (z-axis rotation + xy acceleration)

## Build Instructions

### Requirements
- CMake >= 3.22
- C++17 compiler
- Eigen3
- GTSAM 4.3a0
- Boost (graph, filesystem)

### Building from Source

```bash
# Clone repository
git clone https://github.com/koide3/gtsam_points
cd gtsam_points

# Checkout 2D SLAM development branch
git checkout feature/2d-slam-core

# Install GTSAM 4.3a0 first (if not already installed)
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.3a0
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF
make -j$(nproc)
sudo make install
cd ../..

# Build 2D SLAM library
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_2D_SLAM=ON \
  -DBUILD_2D_TESTS=ON \
  -DBUILD_2D_DEMOS=ON
make -j$(nproc)
sudo make install
```

### Build Options

```cmake
-DBUILD_2D_SLAM=ON          # Enable 2D SLAM library (default: ON)
-DBUILD_2D_TESTS=ON         # Build 2D unit tests (default: ON)
-DBUILD_2D_DEMOS=ON         # Build 2D demo programs (default: ON)
-DBUILD_WITH_OPENMP=ON      # Enable OpenMP (default: ON)
```

## Usage Example

```cpp
#include <gtsam_points/d2/types/laser_scan.hpp>
#include <gtsam_points/d2/factors/integrated_icp_factor_2d.hpp>
#include <gtsam_points/d2/ann/kdtree_2d.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace gtsam_points;

int main() {
  // Load laser scans
  auto scan1 = std::make_shared<LaserScan>();
  auto scan2 = std::make_shared<LaserScan>();
  // ... load data ...

  // Build KdTree for target scan
  auto tree = std::make_shared<KdTree2D>(scan1->points, scan1->num_points);

  // Create factor graph
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;

  // Add poses
  initial.insert(0, gtsam::Pose2(0, 0, 0));
  initial.insert(1, gtsam::Pose2(1, 0, 0.1));  // Initial guess

  // Add ICP factor
  auto icp_factor = std::make_shared<IntegratedICPFactor2D>(
    0, 1, scan1, scan2, tree);
  graph.add(icp_factor);

  // Add prior
  graph.addPrior(0, gtsam::Pose2(0, 0, 0),
    gtsam::noiseModel::Isotropic::Sigma(3, 0.01));

  // Optimize
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
  gtsam::Values result = optimizer.optimize();

  // Get result
  gtsam::Pose2 pose2 = result.at<gtsam::Pose2>(1);
  std::cout << "Optimized pose: " << pose2 << std::endl;

  return 0;
}
```

## Development Roadmap

See [2D_SLAM_MIGRATION_PLAN.md](./2D_SLAM_MIGRATION_PLAN.md) for detailed implementation plan.

### Milestones

- **Week 1-2**: Foundation (PointCloud2D, LaserScan, Basic types)
- **Week 3-4**: Basic Factors (ICP, Point-to-Line)
- **Week 5-6**: GICP Factors
- **Week 7-8**: Grid Maps (IncrementalGridMap)
- **Week 9-10**: Features & Global Registration
- **Week 11-12**: Segmentation
- **Week 13-14**: Continuous Time
- **Week 15-16**: Polish & Documentation

## Architecture

### Type Mappings (3D → 2D)

| 3D | 2D | Notes |
|----|-----|-------|
| `Eigen::Vector4d` | `Eigen::Vector3d` | (x, y, 1) homogeneous |
| `Eigen::Matrix4d` | `Eigen::Matrix3d` | 3x3 transformation |
| `Eigen::Isometry3d` | `Eigen::Isometry2d` | 2D rigid transform |
| `gtsam::Pose3` | `gtsam::Pose2` | (x, y, θ) |
| `gtsam::Rot3` | `gtsam::Rot2` | SO(2) rotation |
| 6 DOF | 3 DOF | x, y, θ |
| 6x6 Hessian | 3x3 Hessian | Reduced dimensionality |
| 3D voxel | 2D grid cell | Spatial indexing |
| 3D normal | 2D normal | Line perpendicular |

### Directory Structure

```
gtsam_points/
├── include/gtsam_points/d2/
│   ├── types/              # Data structures (PointCloud2D, LaserScan)
│   ├── factors/            # Matching factors (ICP, GICP, etc.)
│   ├── ann/                # Nearest neighbor search
│   ├── features/           # Feature extraction (normals, covariance)
│   ├── registration/       # Global registration (RANSAC, GNC)
│   ├── segmentation/       # Point cloud segmentation
│   └── util/               # Utilities (B-Spline, etc.)
├── src/gtsam_points/d2/    # Implementation files
├── tests/d2/               # Unit tests
└── src/demo/d2/            # Demo programs
```

## Testing

```bash
cd build
ctest -R "2d_" --output-on-failure
```

## Contributing

This is an active development branch. Contributions are welcome!

1. Check [2D_SLAM_MIGRATION_PLAN.md](./2D_SLAM_MIGRATION_PLAN.md) for current status
2. Pick an unimplemented component
3. Follow the existing 3D code structure
4. Add unit tests
5. Submit PR to `feature/2d-slam-core` branch

## Branch Strategy

```
main (stable 3D SLAM)
  │
  └── develop
        │
        └── feature/2d-slam-core (current)
              │
              └── release/2d-slam-v1.0 (future)
```

## License

MIT License (same as gtsam_points)

## References

- Base 3D library: https://github.com/koide3/gtsam_points
- GTSAM: https://gtsam.org/
- Related papers: See main [README.md](./README.md)

## Contact

For 2D SLAM specific questions, please open an issue with `[2D SLAM]` prefix.

---

**Last Updated**: 2025-10-30
**Branch**: feature/2d-slam-core
**Status**: Initial setup complete, implementation in progress
