// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <Eigen/Core>
#include <boost/functional/hash/hash.hpp>

namespace gtsam_points {

/**
 * @brief 2D spatial hashing function using boost::hash_combine
 */
class Vector2iHash {
public:
  size_t operator()(const Eigen::Vector2i& x) const {
    size_t seed = 0;
    boost::hash_combine(seed, x[0]);
    boost::hash_combine(seed, x[1]);
    return seed;
  }
};

/**
 * @brief 2D spatial hashing function
 *        Adapted from Teschner et al., "Optimized Spatial Hashing for Collision Detection of Deformable Objects", VMV2003
 */
class XORVector2iHash {
public:
  size_t operator()(const Eigen::Vector2i& x) const {
    const size_t p1 = 9132043225175502913;
    const size_t p2 = 7277549399757405689;
    return static_cast<size_t>((x[0] * p1) ^ (x[1] * p2));
  }
};

}  // namespace gtsam_points
