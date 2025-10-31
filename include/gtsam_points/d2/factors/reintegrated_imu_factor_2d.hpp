// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <Eigen/Core>
#include <vector>
#include <memory>

namespace gtsam_points {

/**
 * @brief Parameters for 2D IMU preintegration
 *        Adapted from GTSAM's PreintegrationParams for planar motion
 */
struct PreintegrationParams2D {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PreintegrationParams2D();

  // Gravity in the global frame (2D: typically [0, -9.81] or [0, 0] for planar motion)
  Eigen::Vector2d n_gravity;

  // Noise standard deviations
  double accelerometer_noise_sigma;  // Accelerometer white noise (m/s²)
  double gyroscope_noise_sigma;      // Gyroscope white noise (rad/s)
  double accelerometer_bias_sigma;   // Accelerometer random walk (m/s²√Hz)
  double gyroscope_bias_sigma;       // Gyroscope random walk (rad/s√Hz)
  double integration_error_cov;      // Integration uncertainty (unused in simple implementation)

  // Use 2nd order integration (Runge-Kutta)
  bool use_2nd_order_integration;
};

/**
 * @brief 2D IMU bias representation: [bias_ax, bias_ay, bias_ωz]
 */
class ImuBias2D {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuBias2D() : accelerometer_(Eigen::Vector2d::Zero()), gyroscope_(0.0) {}
  ImuBias2D(const Eigen::Vector2d& acc_bias, double gyro_bias) : accelerometer_(acc_bias), gyroscope_(gyro_bias) {}

  const Eigen::Vector2d& accelerometer() const { return accelerometer_; }
  double gyroscope() const { return gyroscope_; }

  Eigen::Vector3d vector() const {
    Eigen::Vector3d v;
    v << accelerometer_, gyroscope_;
    return v;
  }

  static ImuBias2D Zero() { return ImuBias2D(Eigen::Vector2d::Zero(), 0.0); }

  ImuBias2D operator+(const Eigen::Vector3d& delta) const {
    return ImuBias2D(accelerometer_ + delta.head<2>(), gyroscope_ + delta(2));
  }

  Eigen::Vector3d operator-(const ImuBias2D& other) const {
    Eigen::Vector3d v;
    v << accelerometer_ - other.accelerometer_, gyroscope_ - other.gyroscope_;
    return v;
  }

private:
  Eigen::Vector2d accelerometer_;  // Accelerometer bias (ax, ay)
  double gyroscope_;               // Gyroscope bias (ωz)
};

/**
 * @brief Preintegrated IMU measurements for 2D motion
 *        Stores the result of integrating IMU measurements between two poses
 */
class PreintegratedImuMeasurements2D {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PreintegratedImuMeasurements2D(const std::shared_ptr<PreintegrationParams2D>& params, const ImuBias2D& bias_hat = ImuBias2D::Zero());

  // Reset integration
  void resetIntegration();

  // Integrate a single IMU measurement
  void integrateMeasurement(const Eigen::Vector2d& measured_acc, double measured_omega, double dt);

  // Getters
  const Eigen::Vector2d& deltaPij() const { return delta_p_ij_; }
  const Eigen::Vector2d& deltaVij() const { return delta_v_ij_; }
  double deltaRij() const { return delta_R_ij_; }  // Angle in radians
  double deltaTij() const { return delta_t_ij_; }

  const std::shared_ptr<PreintegrationParams2D>& params() const { return params_; }
  const ImuBias2D& biasHat() const { return bias_hat_; }

  // Noise covariance
  const Eigen::Matrix<double, 5, 5>& preintMeasCov() const { return preint_meas_cov_; }

private:
  std::shared_ptr<PreintegrationParams2D> params_;
  ImuBias2D bias_hat_;  // Linearization point for bias

  // Preintegrated quantities
  double delta_R_ij_;       // Integrated rotation (angle in radians)
  Eigen::Vector2d delta_v_ij_;  // Integrated velocity
  Eigen::Vector2d delta_p_ij_;  // Integrated position
  double delta_t_ij_;       // Integrated time

  // Noise covariance (5x5: 2 for position, 2 for velocity, 1 for rotation)
  Eigen::Matrix<double, 5, 5> preint_meas_cov_;
};

/**
 * @brief "Re"-integrated IMU factor for 2D (IMU factor without pre-integration)
 *        This factor re-integrates IMU measurements every linearization to better estimate the IMU bias
 */
class ReintegratedImuMeasurements2D : public PreintegratedImuMeasurements2D {
public:
  friend class ReintegratedImuFactor2D;

  ReintegratedImuMeasurements2D(const std::shared_ptr<PreintegrationParams2D>& params, const ImuBias2D& bias_hat = ImuBias2D::Zero());

  void resetIntegration();
  void integrateMeasurement(const Eigen::Vector2d& measured_acc, double measured_omega, double dt);

  const Eigen::Vector2d mean_acc() const;
  double mean_gyro() const;

private:
  std::vector<Eigen::Matrix<double, 4, 1>> imu_data;  // [ax, ay, ωz, dt]
};

/**
 * @brief 2D IMU Factor connecting two poses and velocities through IMU preintegration
 *        State variables: Pose2_i, Velocity2_i, Pose2_j, Velocity2_j, Bias2D
 *        Error dimension: 5 (2 position + 2 velocity + 1 rotation)
 */
class ReintegratedImuFactor2D : public gtsam::NonlinearFactor {
public:
  ReintegratedImuFactor2D(
    gtsam::Key pose_i,
    gtsam::Key vel_i,
    gtsam::Key pose_j,
    gtsam::Key vel_j,
    gtsam::Key bias,
    const ReintegratedImuMeasurements2D& imu_measurements);

  virtual ~ReintegratedImuFactor2D() override;

  size_t dim() const override { return 5; }  // 2D position + 2D velocity + 1D rotation

  virtual void print(const std::string& s = "", const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter) const override;

  std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& values) const override;
  double error(const gtsam::Values& values) const override;

  const ReintegratedImuMeasurements2D& measurements() const { return imu_measurements; }

private:
  Eigen::Matrix<double, 5, 1> evaluateError(
    const gtsam::Pose2& pose_i,
    const Eigen::Vector2d& vel_i,
    const gtsam::Pose2& pose_j,
    const Eigen::Vector2d& vel_j,
    const ImuBias2D& bias,
    Eigen::Matrix<double, 5, 3>* H_pose_i = nullptr,
    Eigen::Matrix<double, 5, 2>* H_vel_i = nullptr,
    Eigen::Matrix<double, 5, 3>* H_pose_j = nullptr,
    Eigen::Matrix<double, 5, 2>* H_vel_j = nullptr,
    Eigen::Matrix<double, 5, 3>* H_bias = nullptr) const;

  std::shared_ptr<PreintegratedImuMeasurements2D> reintegrate(const ImuBias2D& bias) const;

private:
  const ReintegratedImuMeasurements2D imu_measurements;
  mutable std::shared_ptr<PreintegratedImuMeasurements2D> cached_pim;
  mutable ImuBias2D cached_bias;
};

}  // namespace gtsam_points
