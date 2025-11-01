// SPDX-License-Identifier: MIT
// Copyright (c) 2025  Kenji Koide (k.koide@aist.go.jp)

#pragma once

#include <gtsam_points/optimizers/dogleg_optimizer_ext.hpp>
#include <gtsam/geometry/Pose2.h>

namespace gtsam_points {

/**
 * @brief 2D SLAM用Dogleg最適化器
 *
 * DoglegOptimizerExtの2D特化版。Pose2ベースのSLAMシステムで
 * Trust-region法によるバッチ非線形最適化を実現。
 *
 * Dogleg法の特徴:
 * - Trust-region法: 信頼領域内で最適化
 * - Gauss-Newton方向とSteepest Descent方向の補間
 * - Levenberg-Marquardtより計算効率が良い場合がある
 * - 大規模問題に対してロバスト
 *
 * アルゴリズム:
 * 1. Trust-region半径Δを設定
 * 2. Gauss-Newton step p_gn を計算
 * 3. Steepest descent step p_sd を計算
 * 4. |p_gn| ≤ Δ なら p_gn を採用
 * 5. そうでなければ、Dogleg曲線上で最適なステップを選択
 * 6. 更新の品質に応じてΔを調整
 *
 * 用途:
 * - 大規模SLAMのバッチ最適化
 * - Levenberg-Marquardtが遅い場合の代替
 * - Trust-region制約が必要な問題
 * - 数値的に不安定な問題
 *
 * 使用例:
 * @code
 * // ファクターグラフを構築
 * gtsam::NonlinearFactorGraph graph;
 * gtsam::Values initial_values;
 *
 * // ... ファクターと初期値を追加 ...
 *
 * // Doglegパラメータ
 * gtsam::DoglegParams params;
 * params.setVerbosity("ERROR");
 * params.setMaxIterations(100);
 * params.setRelativeErrorTol(1e-5);
 * params.setAbsoluteErrorTol(1e-5);
 *
 * // 最適化
 * DoglegOptimizer2D optimizer(graph, initial_values, params);
 * gtsam::Values result = optimizer.optimize();
 *
 * // 結果を取得
 * for (int i = 0; i < num_frames; ++i) {
 *     gtsam::Pose2 pose = optimizer.getPose2(gtsam::Symbol('x', i));
 * }
 *
 * // Trust-region半径を確認
 * double delta = optimizer.getDelta();
 * std::cout << "Final trust-region radius: " << delta << std::endl;
 * @endcode
 */
class DoglegOptimizer2D : public DoglegOptimizerExt {
public:
  using shared_ptr = std::shared_ptr<DoglegOptimizer2D>;

  /**
   * @brief コンストラクタ
   * @param graph 非線形ファクターグラフ
   * @param initialValues 初期値
   * @param params Dogleg最適化パラメータ
   */
  DoglegOptimizer2D(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& initialValues,
    const gtsam::DoglegParams& params = DefaultDoglegParams())
  : DoglegOptimizerExt(graph, initialValues, params) {}

  /**
   * @brief コンストラクタ（変数順序指定版）
   * @param graph 非線形ファクターグラフ
   * @param initialValues 初期値
   * @param ordering 変数の順序
   */
  DoglegOptimizer2D(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& initialValues,
    const gtsam::Ordering& ordering)
  : DoglegOptimizerExt(graph, initialValues, ordering) {}

  /**
   * @brief デストラクタ
   */
  virtual ~DoglegOptimizer2D() {}

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
    std::cout << s << "DoglegOptimizer2D Status:" << std::endl;
    std::cout << "  Current Error: " << error() << std::endl;
    std::cout << "  Current Delta (trust-region radius): " << getDelta() << std::endl;
    std::cout << "  Iterations: " << iterations() << std::endl;
  }

private:
  /**
   * @brief デフォルトDoglegパラメータ（2D最適化用）
   */
  static gtsam::DoglegParams DefaultDoglegParams() {
    gtsam::DoglegParams params;
    params.setVerbosity("ERROR");
    params.setMaxIterations(100);
    params.setRelativeErrorTol(1e-5);
    params.setAbsoluteErrorTol(1e-5);
    return params;
  }
};

}  // namespace gtsam_points
