// SPDX-License-Identifier: MIT
// Copyright (c) 2021  Kenji Koide (k.koide@aist.go.jp)
// Copyright (c) 2025  2D SLAM Extension

#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>

#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>

namespace gtsam_points {

PointCloud2DCPU::PointCloud2DCPU() : PointCloud2D() {}

PointCloud2DCPU::~PointCloud2DCPU() {}

PointCloud2DCPU::Ptr PointCloud2DCPU::clone(const PointCloud2D& points) {
  auto cloned = std::make_shared<PointCloud2DCPU>();
  cloned->num_points = points.num_points;

  if (points.times) {
    cloned->add_times(points.times, points.num_points);
  }

  if (points.points) {
    cloned->add_points(points.points, points.num_points);
  }

  if (points.normals) {
    cloned->add_normals(points.normals, points.num_points);
  }

  if (points.covs) {
    cloned->add_covs(points.covs, points.num_points);
  }

  if (points.intensities) {
    cloned->add_intensities(points.intensities, points.num_points);
  }

  // Copy aux attributes
  for (const auto& attrib : points.aux_attributes) {
    const std::string& name = attrib.first;
    const size_t elem_size = attrib.second.first;
    const void* data_ptr = attrib.second.second;

    std::vector<char> buffer(elem_size * points.num_points);
    std::memcpy(buffer.data(), data_ptr, elem_size * points.num_points);

    auto storage = std::make_shared<std::vector<char>>(std::move(buffer));
    cloned->aux_attributes_storage[name] = storage;
    cloned->aux_attributes[name] = std::make_pair(elem_size, storage->data());
  }

  return cloned;
}

namespace {
template <typename T>
bool read_binary(const std::string& filename, std::vector<T>& data) {
  std::ifstream ifs(filename, std::ios::binary | std::ios::ate);
  if (!ifs) {
    return false;
  }

  const size_t size = ifs.tellg();
  data.resize(size / sizeof(T));
  ifs.seekg(0);
  ifs.read(reinterpret_cast<char*>(data.data()), size);

  return true;
}
}  // namespace

PointCloud2DCPU::Ptr PointCloud2DCPU::load(const std::string& path) {
  if (!boost::filesystem::exists(path)) {
    std::cerr << "error: point cloud directory does not exist: " << path << std::endl;
    return nullptr;
  }

  auto cloud = std::make_shared<PointCloud2DCPU>();

  // Load times
  std::vector<double> times;
  if (read_binary(path + "/times.bin", times)) {
    cloud->add_times(times);
  }

  // Load points
  std::vector<Eigen::Vector3d> points;
  if (read_binary(path + "/points.bin", points)) {
    cloud->add_points(points);
  }

  // Load normals
  std::vector<Eigen::Vector3d> normals;
  if (read_binary(path + "/normals.bin", normals)) {
    cloud->add_normals(normals);
  }

  // Load covariances
  std::vector<Eigen::Matrix3d> covs;
  if (read_binary(path + "/covs.bin", covs)) {
    cloud->add_covs(covs);
  }

  // Load intensities
  std::vector<double> intensities;
  if (read_binary(path + "/intensities.bin", intensities)) {
    cloud->add_intensities(intensities);
  }

  // Load aux attributes
  for (const auto& entry : boost::filesystem::directory_iterator(path)) {
    const std::string filename = entry.path().filename().string();
    if (filename.find("aux_") == 0 && filename.find(".bin") != std::string::npos) {
      const std::string attrib_name = filename.substr(4, filename.size() - 8);

      std::vector<char> buffer;
      if (read_binary(entry.path().string(), buffer)) {
        auto storage = std::make_shared<std::vector<char>>(std::move(buffer));
        cloud->aux_attributes_storage[attrib_name] = storage;
        cloud->aux_attributes[attrib_name] = std::make_pair(1, storage->data());
      }
    }
  }

  if (cloud->num_points == 0) {
    std::cerr << "warning: loaded point cloud is empty" << std::endl;
  }

  return cloud;
}

size_t PointCloud2DCPU::memory_usage() const {
  size_t usage = 0;

  if (times_storage) {
    usage += sizeof(double) * times_storage->size();
  }

  if (points_storage) {
    usage += sizeof(Eigen::Vector3d) * points_storage->size();
  }

  if (normals_storage) {
    usage += sizeof(Eigen::Vector3d) * normals_storage->size();
  }

  if (covs_storage) {
    usage += sizeof(Eigen::Matrix3d) * covs_storage->size();
  }

  if (intensities_storage) {
    usage += sizeof(double) * intensities_storage->size();
  }

  for (const auto& attrib : aux_attributes) {
    usage += attrib.second.first * num_points;
  }

  return usage;
}

}  // namespace gtsam_points
