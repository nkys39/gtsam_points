// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <gtsam_points/optimizers/levenberg_marquardt_ext.hpp>
#include <gtsam/geometry/Pose2.h>

namespace gtsam_points {

/**
 * @brief 2D SLAM用Levenberg-Marquardt最適化器
 *
 * LevenbergMarquardtOptimizerExtの2D特化版。Pose2ベースのSLAMシステムで
 * バッチ非線形最適化を実現。
 *
 * Levenberg-Marquardt法の特徴:
 * - Trust-region法とGauss-Newton法のハイブリッド
 * - 初期値が悪くても収束しやすい
 * - バッチ最適化（全データを一括処理）
 * - ループクロージャ後のグローバル最適化に最適
 *
 * 用途:
 * - ループクロージャ検出後のバッチ最適化
 * - オフライン軌跡補正
 * - 初期推定値の改善
 * - 小規模から中規模のSLAM問題
 *
 * 使用例:
 * @code
 * // ファクターグラフを構築
 * gtsam::NonlinearFactorGraph graph;
 * gtsam::Values initial_values;
 *
 * // 事前分布
 * gtsam::Key pose0 = gtsam::Symbol('x', 0);
 * initial_values.insert(pose0, gtsam::Pose2(0, 0, 0));
 *
 * auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
 *     gtsam::Vector3(0.01, 0.01, 0.01)
 * );
 * graph.add(gtsam::PriorFactor<gtsam::Pose2>(pose0, gtsam::Pose2(), prior_noise));
 *
 * // ICPファクターを追加
 * for (int i = 1; i < num_frames; ++i) {
 *     gtsam::Key pose_i = gtsam::Symbol('x', i);
 *     initial_values.insert(pose_i, initial_guess);
 *
 *     auto icp_factor = gtsam::make_shared<IntegratedICPFactor2D>(...);
 *     graph.add(icp_factor);
 * }
 *
 * // ループクロージャ
 * auto loop_factor = gtsam::make_shared<IntegratedICPFactor2D>(...);
 * graph.add(loop_factor);
 *
 * // 最適化
 * LevenbergMarquardtExtParams params;
 * params.setVerbosity("ERROR");
 * params.setMaxIterations(100);
 *
 * LevenbergMarquardtOptimizer2D optimizer(graph, initial_values, params);
 * gtsam::Values result = optimizer.optimize();
 *
 * // 結果を取得
 * for (int i = 0; i < num_frames; ++i) {
 *     gtsam::Pose2 pose = optimizer.getPose2(gtsam::Symbol('x', i));
 *     std::cout << "Pose " << i << ": " << pose << std::endl;
 * }
 * @endcode
 */
class LevenbergMarquardtOptimizer2D : public LevenbergMarquardtOptimizerExt {
public:
  using shared_ptr = std::shared_ptr<LevenbergMarquardtOptimizer2D>;

  /**
   * @brief コンストラクタ
   * @param graph 非線形ファクターグラフ
   * @param initialValues 初期値
   * @param params 最適化パラメータ
   */
  LevenbergMarquardtOptimizer2D(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& initialValues,
    const LevenbergMarquardtExtParams& params = DefaultLMParams())
  : LevenbergMarquardtOptimizerExt(graph, initialValues, params) {}

  /**
   * @brief コンストラクタ（変数順序指定版）
   * @param graph 非線形ファクターグラフ
   * @param initialValues 初期値
   * @param ordering 変数の順序
   * @param params 最適化パラメータ
   */
  LevenbergMarquardtOptimizer2D(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& initialValues,
    const gtsam::Ordering& ordering,
    const LevenbergMarquardtExtParams& params = DefaultLMParams())
  : LevenbergMarquardtOptimizerExt(graph, initialValues, ordering, params) {}

  /**
   * @brief デストラクタ
   */
  virtual ~LevenbergMarquardtOptimizer2D() {}

  /**
   * @brief Pose2の推定値を取得（型安全版）
   * @param key ポーズのキー
   * @return 推定されたPose2
   */
  gtsam::Pose2 getPose2(gtsam::Key key) const {
    return values().at<gtsam::Pose2>(key);
  }

  /**
   * @brief Vector2の推定値を取得（速度など）
   * @param key ベクトルのキー
   * @return 推定されたVector2
   */
  Eigen::Vector2d getVector2(gtsam::Key key) const {
    return values().at<Eigen::Vector2d>(key);
  }

  /**
   * @brief 2D軌跡を取得（Symbol 'x'の全Pose2）
   * @return Pose2のベクトル（インデックス順）
   */
  std::vector<gtsam::Pose2> getTrajectory2D() const {
    std::vector<std::pair<size_t, gtsam::Pose2>> indexed_poses;
    const auto& vals = values();

    for (const auto& key_value : vals) {
      gtsam::Key key = key_value.key;

      // Symbol 'x'のキーのみを取得
      if (gtsam::Symbol(key).chr() == 'x') {
        try {
          gtsam::Pose2 pose = vals.at<gtsam::Pose2>(key);
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
   * @brief 現在の誤差（コスト）を取得
   * @return 現在の誤差
   */
  double getCurrentError() const {
    return error();
  }

  /**
   * @brief 現在の反復回数を取得
   * @return 反復回数
   */
  int getCurrentIterations() const {
    return iterations();
  }

  /**
   * @brief オプティマイザの状態を出力（デバッグ用）
   * @param s 出力プレフィックス
   */
  void printStatus(const std::string& s = "") const {
    std::cout << s << "LevenbergMarquardtOptimizer2D Status:" << std::endl;
    std::cout << "  Current Error: " << error() << std::endl;
    std::cout << "  Current Lambda: " << lambda() << std::endl;
    std::cout << "  Iterations: " << iterations() << std::endl;
    std::cout << "  Inner Iterations: " << getInnerIterations() << std::endl;
  }

private:
  /**
   * @brief デフォルトLMパラメータ（2D最適化用）
   */
  static LevenbergMarquardtExtParams DefaultLMParams() {
    LevenbergMarquardtExtParams params;
    params.setVerbosity("ERROR");
    params.setMaxIterations(100);
    params.setRelativeErrorTol(1e-5);
    params.setAbsoluteErrorTol(1e-5);
    return params;
  }
};

}  // namespace gtsam_points
