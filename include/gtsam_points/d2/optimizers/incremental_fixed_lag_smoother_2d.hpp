// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam/geometry/Pose2.h>
#include <memory>

namespace gtsam_points {

/**
 * @brief 2D SLAM用Fixed-Lag Smoother
 *
 * IncrementalFixedLagSmootherExtの2D特化版。Pose2ベースのSLAMシステムで
 * メモリと計算量を一定に保ちながらリアルタイム最適化を実現。
 *
 * 主な特徴:
 * - 一定時間ウィンドウ内の状態のみを保持（古い状態は周辺化）
 * - ISAM2ベースのインクリメンタル更新
 * - メモリ効率的（長時間動作可能）
 * - Pose2、Vector2、その他の2D状態変数に対応
 *
 * 用途:
 * - リアルタイムロボットナビゲーション
 * - オンライン2D SLAM
 * - 移動ロボットの位置推定
 *
 * 使用例:
 * @code
 * // 5秒のラグで初期化
 * ISAM2Params params;
 * params.relinearizeThreshold = 0.1;
 * IncrementalFixedLagSmoother2D smoother(5.0, params);
 *
 * // 新しいファクターと値を追加
 * gtsam::NonlinearFactorGraph new_factors;
 * gtsam::Values new_values;
 * gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
 *
 * // ポーズとタイムスタンプを追加
 * gtsam::Key pose_key = gtsam::Symbol('x', frame_id);
 * new_values.insert(pose_key, gtsam::Pose2(x, y, theta));
 * timestamps[pose_key] = current_time;
 *
 * // ファクターを追加（ICP、IMUなど）
 * new_factors.add(...);
 *
 * // 更新実行
 * smoother.update(new_factors, new_values, timestamps);
 *
 * // 推定値を取得
 * gtsam::Pose2 estimated_pose = smoother.calculateEstimate<gtsam::Pose2>(pose_key);
 * @endcode
 */
class IncrementalFixedLagSmoother2D : public IncrementalFixedLagSmootherExt {
public:
  using shared_ptr = std::shared_ptr<IncrementalFixedLagSmoother2D>;

  /**
   * @brief コンストラクタ
   * @param smoother_lag 保持する時間ウィンドウの長さ（秒）。この時間より古い状態は周辺化される
   * @param parameters ISAM2パラメータ
   */
  IncrementalFixedLagSmoother2D(
    double smoother_lag = 5.0,
    const gtsam::ISAM2Params& parameters = DefaultISAM2Params())
  : IncrementalFixedLagSmootherExt(smoother_lag, parameters) {}

  /**
   * @brief デストラクタ
   */
  virtual ~IncrementalFixedLagSmoother2D() {}

  /**
   * @brief Pose2の推定値を取得（型安全版）
   * @param key ポーズのキー
   * @return 推定されたPose2
   */
  gtsam::Pose2 getPose2(gtsam::Key key) const {
    return calculateEstimate<gtsam::Pose2>(key);
  }

  /**
   * @brief Vector2の推定値を取得（速度など）
   * @param key ベクトルのキー
   * @return 推定されたVector2
   */
  Eigen::Vector2d getVector2(gtsam::Key key) const {
    return calculateEstimate<Eigen::Vector2d>(key);
  }

  /**
   * @brief Pose2の共分散行列を取得（3x3）
   * @param key ポーズのキー
   * @return 共分散行列 [x, y, θ]
   */
  gtsam::Matrix3 getPose2Covariance(gtsam::Key key) const {
    return marginalCovariance(key);
  }

  /**
   * @brief 現在のスムーサーラグを取得
   * @return ラグ時間（秒）
   */
  double getSmootherLag() const {
    return smootherLag_;
  }

  /**
   * @brief 現在保持している状態変数の数を取得
   * @return 状態変数の数
   */
  size_t getNumVariables() const {
    return calculateEstimate().size();
  }

  /**
   * @brief 現在のファクター数を取得
   * @return ファクター数
   */
  size_t getNumFactors() const {
    return getFactors().size();
  }

  /**
   * @brief 最新のタイムスタンプを取得
   * @return 最新タイムスタンプ（秒）
   */
  double getLatestTimestamp() const {
    return getCurrentTimestamp();
  }

  /**
   * @brief 指定キーのタイムスタンプを取得
   * @param key キー
   * @return タイムスタンプ（秒）、存在しない場合は-1
   */
  double getKeyTimestamp(gtsam::Key key) const {
    const auto& ts_map = timestamps();
    auto it = ts_map.find(key);
    return (it != ts_map.end()) ? it->second : -1.0;
  }

  /**
   * @brief 2D軌跡を取得（Symbol 'x'の全Pose2）
   * @return タイムスタンプでソートされたPose2のベクトル
   */
  std::vector<std::pair<double, gtsam::Pose2>> getTrajectory2D() const {
    std::vector<std::pair<double, gtsam::Pose2>> trajectory;

    const auto& values = calculateEstimate();
    const auto& ts_map = timestamps();

    for (const auto& key_value : values) {
      gtsam::Key key = key_value.key;

      // Symbol 'x'のキーのみを取得
      if (gtsam::Symbol(key).chr() == 'x') {
        auto ts_it = ts_map.find(key);
        if (ts_it != ts_map.end()) {
          try {
            gtsam::Pose2 pose = values.at<gtsam::Pose2>(key);
            trajectory.emplace_back(ts_it->second, pose);
          } catch (...) {
            // Pose2でない場合はスキップ
          }
        }
      }
    }

    // タイムスタンプでソート
    std::sort(trajectory.begin(), trajectory.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    return trajectory;
  }

  /**
   * @brief 最適化の統計情報を取得
   * @return ISAM2の最新結果
   */
  const gtsam::ISAM2Result& getOptimizationStats() const {
    return getISAM2Result();
  }

  /**
   * @brief スムーサーの状態を出力（デバッグ用）
   * @param s 出力プレフィックス
   */
  void printStatus(const std::string& s = "") const {
    std::cout << s << "IncrementalFixedLagSmoother2D Status:" << std::endl;
    std::cout << "  Smoother Lag: " << smootherLag_ << " seconds" << std::endl;
    std::cout << "  Num Variables: " << getNumVariables() << std::endl;
    std::cout << "  Num Factors: " << getNumFactors() << std::endl;
    std::cout << "  Latest Timestamp: " << getLatestTimestamp() << std::endl;

    const auto& result = getISAM2Result();
    std::cout << "  Last Update:" << std::endl;
    std::cout << "    Variables Reeliminated: " << result.variablesReeliminated << std::endl;
    std::cout << "    Variables Relinearized: " << result.variablesRelinearized << std::endl;
    std::cout << "    Cliques: " << result.cliques << std::endl;
  }

private:
  /**
   * @brief デフォルトISAM2パラメータ（2D最適化用）
   */
  static gtsam::ISAM2Params DefaultISAM2Params() {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;     // 2Dでは小さめの閾値
    params.relinearizeSkip = 1;
    params.enableRelinearization = true;
    params.evaluateNonlinearError = false;  // パフォーマンス重視
    params.factorization = gtsam::ISAM2Params::CHOLESKY;
    params.findUnusedFactorSlots = true;
    return params;
  }
};

}  // namespace gtsam_points
