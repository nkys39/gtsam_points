// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <gtsam/geometry/Pose2.h>

namespace gtsam_points {

/**
 * @brief 2D SLAM用ISAM2インクリメンタル最適化器
 *
 * ISAM2Extの2D特化版。Pose2ベースのSLAMシステムでインクリメンタルな
 * 非線形最適化を実現。
 *
 * ISAM2 (Incremental Smoothing and Mapping 2)の主な特徴:
 * - インクリメンタル更新: 新しい観測が到着するたびに効率的に更新
 * - ベイズ木による疎行列の表現: 高速な更新と推論
 * - 選択的再線形化: 必要な変数のみを再線形化して計算量を削減
 * - 周辺化: 古い変数を削除してメモリ効率化
 *
 * 用途:
 * - バッチ最適化が不要なリアルタイムSLAM
 * - 逐次的にデータが到着するオンライン推定
 * - ループクロージャ付きグラフSLAM
 *
 * 使用例:
 * @code
 * // ISAM2パラメータ
 * gtsam::ISAM2Params params;
 * params.relinearizeThreshold = 0.01;  // 2D用
 * params.relinearizeSkip = 1;
 *
 * ISAM2_2D isam2(params);
 *
 * // 初期ポーズの事前分布
 * gtsam::NonlinearFactorGraph graph;
 * gtsam::Values initial;
 *
 * gtsam::Key pose0 = gtsam::Symbol('x', 0);
 * initial.insert(pose0, gtsam::Pose2(0, 0, 0));
 *
 * auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
 *     gtsam::Vector3(0.01, 0.01, 0.01)
 * );
 * graph.add(gtsam::PriorFactor<gtsam::Pose2>(pose0, gtsam::Pose2(), prior_noise));
 *
 * // 初期更新
 * isam2.update(graph, initial);
 *
 * // メインループ
 * for (int i = 1; i < num_frames; ++i) {
 *     gtsam::NonlinearFactorGraph new_factors;
 *     gtsam::Values new_values;
 *
 *     gtsam::Key pose_i = gtsam::Symbol('x', i);
 *     new_values.insert(pose_i, initial_guess);
 *
 *     // ICPファクターを追加
 *     new_factors.add(...);
 *
 *     // 更新
 *     isam2.update(new_factors, new_values);
 *
 *     // 推定値を取得
 *     gtsam::Pose2 pose = isam2.getPose2(pose_i);
 * }
 * @endcode
 */
class ISAM2_2D : public ISAM2Ext {
public:
  using shared_ptr = std::shared_ptr<ISAM2_2D>;

  /**
   * @brief コンストラクタ（パラメータ指定）
   * @param params ISAM2パラメータ
   */
  explicit ISAM2_2D(const gtsam::ISAM2Params& params = DefaultISAM2Params())
  : ISAM2Ext(params) {}

  /**
   * @brief デストラクタ
   */
  virtual ~ISAM2_2D() {}

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
   * @brief 2D軌跡を取得（Symbol 'x'の全Pose2）
   * @return Pose2のベクトル（インデックス順）
   */
  std::vector<gtsam::Pose2> getTrajectory2D() const {
    std::vector<std::pair<size_t, gtsam::Pose2>> indexed_poses;
    const auto& values = calculateEstimate();

    for (const auto& key_value : values) {
      gtsam::Key key = key_value.key;

      // Symbol 'x'のキーのみを取得
      if (gtsam::Symbol(key).chr() == 'x') {
        try {
          gtsam::Pose2 pose = values.at<gtsam::Pose2>(key);
          size_t index = gtsam::Symbol(key).index();
          indexed_poses.emplace_back(index, pose);
        } catch (...) {
          // Pose2でない場合はスキップ
        }
      }
    }

    // インデックスでソート
    std::sort(indexed_poses.begin(), indexed_poses.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    // Pose2のみを抽出
    std::vector<gtsam::Pose2> trajectory;
    trajectory.reserve(indexed_poses.size());
    for (const auto& [idx, pose] : indexed_poses) {
      trajectory.push_back(pose);
    }

    return trajectory;
  }

  /**
   * @brief 現在保持している状態変数の数を取得
   * @return 状態変数の数
   */
  size_t getNumVariables() const {
    return size();
  }

  /**
   * @brief 現在のファクター数を取得
   * @return ファクター数
   */
  size_t getNumFactors() const {
    return getFactorsUnsafe().size();
  }

  /**
   * @brief ISAM2の状態を出力（デバッグ用）
   * @param s 出力プレフィックス
   */
  void printStatus(const std::string& s = "") const {
    std::cout << s << "ISAM2_2D Status:" << std::endl;
    std::cout << "  Num Variables: " << getNumVariables() << std::endl;
    std::cout << "  Num Factors: " << getNumFactors() << std::endl;
    std::cout << "  Update Count: " << update_count_ << std::endl;
  }

  /**
   * @brief 最適化の統計情報を取得
   * @return 最新のISAM2Result
   */
  const ISAM2ResultExt& getOptimizationStats() const {
    return last_result_;
  }

  /**
   * @brief 更新の実行（統計情報を保存）
   */
  ISAM2ResultExt update(
    const gtsam::NonlinearFactorGraph& newFactors = gtsam::NonlinearFactorGraph(),
    const gtsam::Values& newTheta = gtsam::Values(),
    const gtsam::FactorIndices& removeFactorIndices = gtsam::FactorIndices(),
    const std::optional<gtsam::FastMap<gtsam::Key, int>>& constrainedKeys = {},
    const std::optional<gtsam::FastList<gtsam::Key>>& noRelinKeys = {},
    const std::optional<gtsam::FastList<gtsam::Key>>& extraReelimKeys = {},
    bool force_relinearize = false) override {

    last_result_ = ISAM2Ext::update(
      newFactors, newTheta, removeFactorIndices,
      constrainedKeys, noRelinKeys, extraReelimKeys,
      force_relinearize
    );
    return last_result_;
  }

  /**
   * @brief 更新の実行（パラメータ版、統計情報を保存）
   */
  ISAM2ResultExt update(
    const gtsam::NonlinearFactorGraph& newFactors,
    const gtsam::Values& newTheta,
    const gtsam::ISAM2UpdateParams& updateParams) override {

    last_result_ = ISAM2Ext::update(newFactors, newTheta, updateParams);
    return last_result_;
  }

private:
  mutable ISAM2ResultExt last_result_;  ///< 最新の更新結果

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
