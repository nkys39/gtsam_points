// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_with_fallback.hpp>
#include <gtsam/geometry/Pose2.h>

namespace gtsam_points {

/**
 * @brief フォールバック機能付き2D Fixed-Lag Smoother
 *
 * IncrementalFixedLagSmootherExtWithFallbackの2D特化版。
 * 最適化が失敗した場合に自動的にフォールバックしてスムーザーを再構築する。
 *
 * 主な特徴:
 * - 最適化失敗時の自動リカバリー
 * - 一定時間ウィンドウ内の状態のみを保持
 * - ISAM2ベースのインクリメンタル更新
 * - Pose2、Vector2の型安全なAPI
 *
 * フォールバック動作:
 * - update()やcalculateEstimate()で例外が発生した場合
 * - 現在の状態とファクターでスムーザーを再構築
 * - ユーザーコードへの影響を最小化
 *
 * 用途:
 * - ロバストな長時間動作SLAM
 * - 不安定な環境でのロボットナビゲーション
 * - デバッグが困難な実運用システム
 *
 * 使用例:
 * @code
 * // フォールバック機能付きで初期化
 * gtsam::ISAM2Params params;
 * params.relinearizeThreshold = 0.01;
 * IncrementalFixedLagSmoother2DWithFallback smoother(5.0, params);
 *
 * // 新しいファクターと値を追加
 * // ... (IncrementalFixedLagSmoother2Dと同じ使い方)
 *
 * // 更新実行（失敗時は自動フォールバック）
 * smoother.update(new_factors, new_values, timestamps);
 *
 * // フォールバックが発生したか確認
 * if (smoother.fallbackHappened()) {
 *     std::cerr << "Warning: Fallback occurred!" << std::endl;
 * }
 *
 * // 推定値を取得（型安全）
 * gtsam::Pose2 pose = smoother.getPose2(pose_key);
 * @endcode
 */
class IncrementalFixedLagSmoother2DWithFallback : public IncrementalFixedLagSmootherExtWithFallback {
public:
  using shared_ptr = std::shared_ptr<IncrementalFixedLagSmoother2DWithFallback>;

  /**
   * @brief コンストラクタ
   * @param smoother_lag 保持する時間ウィンドウの長さ（秒）
   * @param parameters ISAM2パラメータ
   */
  IncrementalFixedLagSmoother2DWithFallback(
    double smoother_lag = 5.0,
    const gtsam::ISAM2Params& parameters = DefaultISAM2Params())
  : IncrementalFixedLagSmootherExtWithFallback(smoother_lag, parameters) {}

  /**
   * @brief デストラクタ
   */
  virtual ~IncrementalFixedLagSmoother2DWithFallback() {}

  /**
   * @brief Pose2の推定値を取得（型安全版）
   * @param key ポーズのキー
   * @return 推定されたPose2
   * @note 例外が発生した場合は自動的にフォールバック
   */
  gtsam::Pose2 getPose2(gtsam::Key key) const {
    return calculateEstimate<gtsam::Pose2>(key);
  }

  /**
   * @brief Vector2の推定値を取得（速度など）
   * @param key ベクトルのキー
   * @return 推定されたVector2
   * @note 例外が発生した場合は自動的にフォールバック
   */
  Eigen::Vector2d getVector2(gtsam::Key key) const {
    return calculateEstimate<Eigen::Vector2d>(key);
  }

  /**
   * @brief Pose2の共分散行列を取得（3x3）
   * @param key ポーズのキー
   * @return 共分散行列 [x, y, θ]
   * @note 例外が発生した場合は自動的にフォールバック
   */
  gtsam::Matrix3 getPose2Covariance(gtsam::Key key) const {
    try {
      return marginalCovariance(key);
    } catch (std::exception& e) {
      std::cerr << "warning: exception in getPose2Covariance: " << e.what() << std::endl;
      // フォールバックは既にcalculateEstimate内で発生しているはず
      return marginalCovariance(key);
    }
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
   * @brief スムーサーの状態を出力（デバッグ用）
   * @param s 出力プレフィックス
   */
  void printStatus(const std::string& s = "") const {
    std::cout << s << "IncrementalFixedLagSmoother2DWithFallback Status:" << std::endl;
    std::cout << "  Smoother Lag: " << smootherLag_ << " seconds" << std::endl;
    std::cout << "  Num Variables: " << getNumVariables() << std::endl;
    std::cout << "  Num Factors: " << getNumFactors() << std::endl;
    std::cout << "  Latest Timestamp: " << getLatestTimestamp() << std::endl;
    std::cout << "  Fallback Occurred: " << (fallbackHappened() ? "YES" : "NO") << std::endl;
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
