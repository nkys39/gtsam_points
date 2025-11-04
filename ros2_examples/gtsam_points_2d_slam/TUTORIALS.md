# gtsam_points 2D SLAM チュートリアル完全ガイド

このドキュメントは、gtsam_points を用いた2D SLAMの15個のチュートリアルについて、アルゴリズムの詳細から実装のポイントまでを包括的に解説します。

---

## 目次

### Phase 1: 基礎
- [Tutorial 1: 基本SLAM](#tutorial-1-基本slam)
- [Tutorial 2: LiDARオドメトリ](#tutorial-2-lidarオドメトリ)

### Phase 2: 高度な機能
- [Tutorial 3: ループクロージャ](#tutorial-3-ループクロージャ)
- [Tutorial 4: ホイールオドメトリ統合](#tutorial-4-ホイールオドメトリ統合)
- [Tutorial 5: IMU統合](#tutorial-5-imu統合)
- [Tutorial 6: 固定ラグ平滑化](#tutorial-6-固定ラグ平滑化)

### Phase 3: スキャンマッチング
- [Tutorial 7: CT-ICP](#tutorial-7-ct-icp)
- [Tutorial 8: CT-GICP](#tutorial-8-ct-gicp)

### Phase 4: ロバスト推定
- [Tutorial 9: RANSAC](#tutorial-9-ransac)
- [Tutorial 10: GNC (Graduated Non-Convexity)](#tutorial-10-gnc-graduated-non-convexity)
- [Tutorial 11: セグメンテーションSLAM](#tutorial-11-セグメンテーションslam)
- [Tutorial 12: マップ保存・読込](#tutorial-12-マップ保存読込)
- [Tutorial 13: オフラインマップ最適化](#tutorial-13-オフラインマップ最適化)

### Phase 5-6: 統合・管理
- [Tutorial 14: 統合SLAM](#tutorial-14-統合slam)
- [Tutorial 15: キーフレーム管理](#tutorial-15-キーフレーム管理)

---

## Tutorial 1: 基本SLAM

### 概要
最も基本的な2D SLAMの実装。LiDARスキャンマッチングとISAM2による増分最適化を組み合わせた、SLAMの基礎となる実装です。

### アルゴリズム

#### スキャンマッチング
- **手法**: GICP (Generalized ICP) または VGICP (Voxelized GICP)
- **マッチング対象**: Scan-to-Scan（最新のキーフレームに対して現在のスキャンを位置合わせ）
- **初期推定**: 単位変換（移動なしを仮定）

#### ファクターグラフ
```
x0 --- x1 --- x2 --- x3 ...
|      |      |      |
prior  GICP   GICP   GICP
```

- **Prior Factor**: 最初のポーズを原点に固定
- **GICP Factor**: 連続するキーフレーム間のスキャンマッチング制約

#### 最適化
- **手法**: ISAM2 (Incremental Smoothing and Mapping)
- **更新**: キーフレーム追加時に増分的に最適化

### 使用するgtsam_points API

```cpp
#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp>
#include <gtsam_points/d2/registration/registration_2d.hpp>
#include <gtsam_points/d2/ann/incremental_gridmap_2d.hpp>
```

**主要API:**
1. **`PointCloud2DCPU`**: 2D点群データ構造
   ```cpp
   auto cloud = std::make_shared<PointCloud2DCPU>();
   cloud->add_point(Eigen::Vector2d(x, y));
   ```

2. **`align_scans_2d()`**: スキャンマッチング関数
   ```cpp
   RegistrationSetting2D setting;
   setting.type = RegistrationType2D::GICP;  // または VGICP
   setting.max_correspondence_distance = 1.0;
   setting.max_iterations = 64;

   auto result = align_scans_2d(target, source, initial_guess, setting);
   gtsam::Pose2 relative_pose = result.T_target_source;
   ```

3. **`IntegratedGICPFactor2D`**: GICPファクター
   ```cpp
   auto gicp_factor = gtsam::make_shared<IntegratedGICPFactor2D>(
     gtsam::Symbol('x', i),      // 前のポーズ
     gtsam::Symbol('x', i+1),    // 現在のポーズ
     target_cloud,               // ターゲット点群
     source_cloud                // ソース点群
   );
   graph.add(gicp_factor);
   ```

4. **`IncrementalGridMap2D`**: VGICPのためのボクセルグリッド
   ```cpp
   auto gridmap = std::make_shared<IncrementalGridMap2D>(voxel_resolution);
   ```

### パラメータ

```yaml
# フレーム設定
map_frame: "map"
odom_frame: "odom"
base_frame: "base_footprint"
scan_topic: "scan"

# スキャンマッチング
use_vgicp: true                      # VGICP使用（false = GICP）
voxel_resolution: 0.1                # VGICPボクセルサイズ [m]
max_correspondence_distance: 1.0     # 対応点探索の最大距離 [m]

# キーフレーム判定
keyframe_distance: 0.5               # キーフレーム追加の距離閾値 [m]
keyframe_angle: 0.3                  # キーフレーム追加の角度閾値 [rad]
```

**パラメータ調整のポイント:**
- `keyframe_distance`: 小さいほど高密度（計算コスト増）、大きいほど低密度
- `max_correspondence_distance`: 環境の広さに応じて調整（広い環境では大きく）
- `voxel_resolution`: 細かいほど精度向上（計算コスト増）

### 起動方法

```bash
# Gazeboシミュレーション起動
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# SLAM起動
ros2 launch gtsam_points_2d_slam slam.launch.py

# RViz起動（別ターミナル）
rviz2 -d $(ros2 pkg prefix gtsam_points_2d_slam)/share/gtsam_points_2d_slam/config/slam.rviz

# ロボット操作（別ターミナル）
ros2 run turtlebot3_teleop teleop_keyboard
```

### 実装のポイント

#### 1. スキャン変換
LaserScanメッセージから2D点群への変換:
```cpp
auto cloud = std::make_shared<PointCloud2DCPU>();
for (size_t i = 0; i < msg->ranges.size(); ++i) {
  float range = msg->ranges[i];
  if (range >= msg->range_min && range <= msg->range_max) {
    float angle = msg->angle_min + i * msg->angle_increment;
    cloud->add_point(Eigen::Vector2d(
      range * std::cos(angle),
      range * std::sin(angle)
    ));
  }
}
```

#### 2. キーフレーム判定
移動距離と回転角度でキーフレームを判定:
```cpp
bool shouldAddKeyframe() {
  double dx = current_pose_.x() - last_keyframe_pose_.x();
  double dy = current_pose_.y() - last_keyframe_pose_.y();
  double distance = std::sqrt(dx*dx + dy*dy);
  double dtheta = std::abs(current_pose_.theta() - last_keyframe_pose_.theta());

  return (distance > keyframe_distance_) || (dtheta > keyframe_angle_);
}
```

#### 3. ISAM2更新
```cpp
// ファクター追加
graph_.add(gicp_factor);
initial_estimates_.insert(gtsam::Symbol('x', key), current_pose_);

// 増分最適化
isam2_->update(graph_, initial_estimates_);
graph_.resize(0);
initial_estimates_.clear();

// 最適化結果取得
gtsam::Values result = isam2_->calculateEstimate();
current_pose_ = result.at<gtsam::Pose2>(gtsam::Symbol('x', key));
```

---

## Tutorial 2: LiDARオドメトリ

### 概要
グラフ最適化を使わないシンプルなオドメトリ。Scan-to-ScanまたはScan-to-Mapのマッチングで相対移動を推定し、デッドレコニングで累積します。

### アルゴリズム

#### Scan-to-Scan モード
```
prev_scan -> current_scan
    |            |
  T_prev      T_current
```
前スキャンに対して現在スキャンをマッチング

#### Scan-to-Map モード
```
global_map -> current_scan
    |              |
accumulated    T_current
```
累積マップに対してマッチング（ドリフトが少ない）

### 使用するgtsam_points API

**Scan-to-Scan:**
```cpp
RegistrationSetting2D setting;
setting.type = use_vgicp_ ? RegistrationType2D::VGICP : RegistrationType2D::GICP;
auto result = align_scans_2d(prev_scan_, scan, gtsam::Pose2(0,0,0), setting);
```

**Scan-to-Map:**
```cpp
// ファクターグラフによる最適化
gtsam::NonlinearFactorGraph graph;
gtsam::Values initial_estimate;

auto vgicp_factor = gtsam::make_shared<IntegratedVGICPFactor2D>(
  pose_key, scan, gridmap_
);
graph.add(vgicp_factor);
initial_estimate.insert(pose_key, current_pose_);

gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, lm_params);
gtsam::Values result = optimizer.optimize();
```

### パラメータ

```yaml
# オドメトリモード
use_scan_to_map: false               # true = scan-to-map, false = scan-to-scan

# レジストレーション
use_vgicp: true
voxel_resolution: 0.1
max_correspondence_distance: 1.0
registration_max_iterations: 64
registration_transformation_epsilon: 1.0e-3
```

### 起動方法

```bash
ros2 launch gtsam_points_2d_slam lidar_odometry.launch.py use_scan_to_map:=false
```

### 実装のポイント

#### Scan-to-Map実装
Levenberg-Marquardt最適化を使用:
```cpp
gtsam::LevenbergMarquardtParams lm_params;
lm_params.setMaxIterations(registration_max_iterations_);
lm_params.setRelativeErrorTol(registration_transformation_epsilon_);

gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_estimate, lm_params);
gtsam::Values result = optimizer.optimize();
gtsam::Pose2 optimized_pose = result.at<gtsam::Pose2>(pose_key);
```

#### ドリフト特性
- **Scan-to-Scan**: ドリフトが蓄積しやすい（ループクロージャなし）
- **Scan-to-Map**: 大域的整合性があるためドリフトが少ない

---

## Tutorial 3: ループクロージャ

### 概要
ロボットが以前訪れた場所に戻った時を検出し、大域的な整合性を保つ。長期間の走行でもドリフトを抑制できます。

### アルゴリズム

#### ループ検出
1. **候補探索**: 現在位置から一定半径内の過去のキーフレームを探索
2. **チェイン長制約**: 最近N個のキーフレームは除外（短いループを防ぐ）
3. **スキャンマッチング**: 候補とのマッチングを試行
4. **検証**: 収束判定とインライア数でループを確認

#### ファクターグラフ
```
x0 --- x1 --- x2 --- x3 --- x4 --- x5
|      |      |      |      |      |
prior  GICP   GICP   GICP   GICP   GICP
       |             |
       +--Loop-------+
```

### 使用するgtsam_points API

```cpp
// ループクロージャのスキャンマッチング
RegistrationSetting2D setting;
setting.type = RegistrationType2D::GICP;
setting.max_correspondence_distance = 0.5;  // ループは厳しめ

auto result = align_scans_2d(candidate_scan, current_scan, initial_guess, setting);

if (result.converged && result.num_inliers > 50) {
  // ループクロージャ制約追加
  auto loop_noise = gtsam::noiseModel::Diagonal::Sigmas(
    gtsam::Vector3(0.1, 0.1, 0.05)  // ループは信頼度高め
  );
  graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
    gtsam::Symbol('x', candidate_idx),
    gtsam::Symbol('x', current_idx),
    result.T_target_source,
    loop_noise
  ));
}
```

### パラメータ

```yaml
# ループクロージャ
enable_loop_closure: true
loop_search_radius: 10.0             # ループ候補探索半径 [m]
loop_min_chain_length: 10            # 最小チェイン長（最近10個は除外）
loop_fitness_threshold: 0.3          # フィットネス閾値
```

### 実装のポイント

#### 候補探索の最適化
```cpp
for (size_t i = 0; i < keyframe_poses_.size() - loop_min_chain_length_; ++i) {
  double dx = current_pose_.x() - keyframe_poses_[i].x();
  double dy = current_pose_.y() - keyframe_poses_[i].y();
  double distance = std::sqrt(dx*dx + dy*dy);

  if (distance < loop_search_radius_) {
    candidates.push_back(i);
  }
}
```

#### ループ検証
- **収束判定**: `result.converged == true`
- **インライア数**: `result.num_inliers > threshold`
- **フィットネススコア**: マッチング品質の評価

#### 可視化
```cpp
visualization_msgs::msg::Marker marker;
marker.type = visualization_msgs::msg::Marker::LINE_LIST;
// ループエッジを線で表示
marker.points.push_back(start_point);
marker.points.push_back(end_point);
```

---

## Tutorial 4: ホイールオドメトリ統合

### 概要
ホイールエンコーダーとLiDARを融合。ホイールオドメトリは高周波で取得でき、LiDARより短期的には正確な場合があります。

### アルゴリズム

#### センサフュージョン
```
Wheel Odom:  高周波、短期正確、長期ドリフト
LiDAR:       低周波、大域的整合性

→ 両方をBetweenFactorとして追加
```

#### ファクターグラフ
```
x0 --------- x1 --------- x2
|            |            |
prior   GICP + Wheel  GICP + Wheel
```

### 使用するgtsam_points API

```cpp
// LiDAR制約
auto gicp_factor = gtsam::make_shared<IntegratedGICPFactor2D>(
  gtsam::Symbol('x', i), gtsam::Symbol('x', i+1),
  target, source
);

// ホイールオドメトリ制約
gtsam::Pose2 wheel_delta = last_wheel_pose_.between(current_wheel_pose_);
auto wheel_noise = gtsam::noiseModel::Diagonal::Sigmas(
  gtsam::Vector3(0.2 / wheel_weight_, 0.2 / wheel_weight_, 0.1 / wheel_weight_)
);
graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
  gtsam::Symbol('x', i), gtsam::Symbol('x', i+1),
  wheel_delta, wheel_noise
));
```

### パラメータ

```yaml
# ホイールオドメトリ
use_wheel_odom: true
wheel_odom_topic: "/odom"
wheel_odom_weight: 0.5               # ホイールオドメトリの重み（0.0-1.0）
```

**重み調整:**
- `wheel_odom_weight = 1.0`: ホイールとLiDARを同等に信頼
- `wheel_odom_weight = 0.5`: LiDARを2倍信頼
- `wheel_odom_weight = 0.1`: ほぼLiDARのみ使用

### 実装のポイント

#### Odometryメッセージの処理
```cpp
void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Quaternionからyaw抽出
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  current_wheel_odom_pose_ = gtsam::Pose2(
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    yaw
  );
}
```

#### ノイズモデルの設定
重みが小さいほどノイズ共分散が大きい（信頼度低い）:
```cpp
double sigma_x = 0.2 / wheel_odom_weight_;
double sigma_y = 0.2 / wheel_odom_weight_;
double sigma_theta = 0.1 / wheel_odom_weight_;
```

---

## Tutorial 5: IMU統合

### 概要
IMU（加速度・角速度）を統合し、高速移動時の推定精度を向上。IMU事前積分により、速度とバイアスも状態変数として推定します。

### アルゴリズム

#### 状態変数
- **x**: ポーズ (x, y, θ)
- **v**: 速度 (vx, vy)
- **b**: バイアス (acc_x_bias, acc_y_bias, gyro_z_bias)

#### IMU事前積分
キーフレーム間のIMU測定を積分:
```
Δp = ∫∫ (a - b_a) dt²
Δv = ∫ (a - b_a) dt
Δθ = ∫ (ω - b_g) dt
```

#### ファクターグラフ
```
x0   v0   b0       x1   v1   b1       x2   v2   b2
|    |    |        |    |    |        |    |    |
+----+----+--------+----+----+--------+----+----+
     |                  |                  |
  prior          IMU Factor         IMU Factor
                    GICP              GICP
```

### 使用するgtsam_points API

```cpp
#include <gtsam_points/factors/reintegrated_imu_factor_2d.hpp>

// IMUパラメータ設定
auto imu_params = std::make_shared<ReintegratedIMUParams2D>();
imu_params->accelerometer_noise = 0.1;
imu_params->gyroscope_noise = 0.01;
imu_params->accelerometer_bias_noise = 0.001;
imu_params->gyroscope_bias_noise = 0.0001;
imu_params->gravity = 9.81;

// IMUファクター作成
auto imu_factor = gtsam::make_shared<ReintegratedIMUFactor2D>(
  X(i), V(i), B(i),   // 前の状態
  X(j), V(j), B(j),   // 現在の状態
  imu_params
);

// IMU測定を積分
for (const auto& meas : measurements) {
  imu_factor->integrateMeasurement(
    meas.linear_acceleration,  // Vector2
    meas.angular_velocity,     // double (z軸のみ)
    dt
  );
}
```

### パラメータ

```yaml
# IMU
use_imu: true
imu_topic: "/imu"
imu_acc_noise: 0.1                   # 加速度計ノイズ [m/s²]
imu_gyro_noise: 0.01                 # ジャイロノイズ [rad/s]
imu_acc_bias_noise: 0.001            # 加速度バイアスノイズ
imu_gyro_bias_noise: 0.0001          # ジャイロバイアスノイズ
gravity: 9.81                        # 重力加速度 [m/s²]
```

### 実装のポイント

#### IMU測定のバッファリング
```cpp
struct IMUMeasurement2D {
  Eigen::Vector2d linear_acceleration;
  double angular_velocity;
  double timestamp;
};

std::deque<IMUMeasurement2D> imu_buffer_;

void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  IMUMeasurement2D meas;
  meas.linear_acceleration = Eigen::Vector2d(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y
  );
  meas.angular_velocity = msg->angular_velocity.z;
  meas.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  imu_buffer_.push_back(meas);
}
```

#### 初期バイアス推定
```cpp
// 静止時のIMU測定から初期バイアスを推定
Eigen::Vector3d initial_bias = Eigen::Vector3d::Zero();

// Priorファクター
graph_.add(gtsam::PriorFactor<gtsam::Pose2>(...));
graph_.add(gtsam::PriorFactor<gtsam::Vector2>(...));  // 速度
graph_.add(gtsam::PriorFactor<gtsam::Vector3>(...));  // バイアス
```

#### 速度のオドメトリ出力
```cpp
odom_msg.twist.twist.linear.x = velocity.x();
odom_msg.twist.twist.linear.y = velocity.y();
odom_msg.twist.twist.angular.z = /* 角速度 */;
```

---

## Tutorial 6: 固定ラグ平滑化

### 概要
固定時間窓（例: 30秒）のみを最適化し、古い状態を周辺化。長時間走行時のメモリ使用量と計算時間を一定に保ちます。

### アルゴリズム

#### 固定ラグスムーザー
```
時刻 t=0   t=30  t=60  t=90
     |---lag---|
          |---lag---|
               |---lag---|

古い状態は周辺化（marginalize）
```

#### タイムスタンプ管理
各変数にタイムスタンプを紐付け:
```cpp
gtsam::KeyTimestampMap timestamps;
timestamps[Symbol('x', i)] = time_i;
```

### 使用するgtsam_points API

```cpp
#include <gtsam_points/util/incremental_fixed_lag_smoother_ext.hpp>

// 固定ラグスムーサー初期化
gtsam::ISAM2Params isam2_params;
smoother_ = std::make_shared<IncrementalFixedLagSmootherExt>(
  smoother_lag_,    // 30.0秒
  isam2_params
);

// 更新（タイムスタンプ付き）
gtsam::KeyTimestampMap timestamps;
timestamps[Symbol('x', i)] = current_time;
smoother_->update(graph_, initial_estimates_, timestamps);
```

### パラメータ

```yaml
# 固定ラグ平滑化
smoother_lag: 30.0                   # ラグ窓サイズ [秒]
max_keyframes: 100                   # 最大キーフレーム数
```

### 起動方法

```bash
ros2 launch gtsam_points_2d_slam slam_with_fixed_lag.launch.py
```

### 実装のポイント

#### 古いキーフレームの削除
```cpp
void pruneOldKeyframes(double current_time) {
  const double cutoff_time = current_time - smoother_lag_;

  while (!keyframe_timestamps_.empty() &&
         keyframe_timestamps_.front() < cutoff_time &&
         keyframes_.size() > max_keyframes_) {
    keyframes_.pop_front();
    keyframe_poses_.pop_front();
    keyframe_ids_.pop_front();
    keyframe_timestamps_.pop_front();
  }
}
```

#### メモリ使用量
- **ISAM2**: キーフレーム数に比例して増加
- **固定ラグ**: ほぼ一定（`smoother_lag`に依存）

---

## Tutorial 7: CT-ICP

### 概要
Continuous-Time ICP。スキャン取得中のロボットの動きを考慮し、1スキャンを2つのポーズ（開始・終了）で表現します。

### アルゴリズム

#### 連続時間表現
```
スキャン開始 (t=0.0) -------- スキャン終了 (t=1.0)
    pose_start                    pose_end
         |                            |
         各点は時刻tに応じて補間される
```

#### 点ごとのタイムスタンプ
```cpp
for (size_t i = 0; i < ranges.size(); ++i) {
  double normalized_time = (time_increment * i) / scan_duration;  // [0, 1]
  timestamps.push_back(normalized_time);
}
cloud->add_times(timestamps);
```

#### ポーズの補間
各点の位置は開始・終了ポーズを線形補間:
```
T(t) = T_start ⊕ (t × (T_start^-1 ⊕ T_end))
```

### 使用するgtsam_points API

```cpp
#include <gtsam_points/factors/integrated_ct_icp_factor_2d.hpp>

// CT-ICPファクター
auto ct_icp_factor = gtsam::make_shared<IntegratedCT_ICPFactor2D>(
  Symbol('x', key * 2),       // スキャン開始ポーズ
  Symbol('x', key * 2 + 1),   // スキャン終了ポーズ
  target,                     // ターゲット点群
  source                      // ソース点群（タイムスタンプ付き）
);
ct_icp_factor->set_max_correspondence_distance(1.0);
ct_icp_factor->set_num_threads(4);
graph_.add(ct_icp_factor);

// 運動連続性制約
graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
  Symbol('x', prev_key * 2 + 1),   // 前スキャン終了
  Symbol('x', curr_key * 2),       // 現スキャン開始
  gtsam::Pose2(0, 0, 0),           // ほぼ同じはず
  noise
));
```

### パラメータ

```yaml
# CT-ICP
max_correspondence_distance: 1.0
k_neighbors: 10                      # 法線推定のK近傍数
```

### 実装のポイント

#### タイムスタンプ付き点群
```cpp
std::vector<Eigen::Vector3d> points;
std::vector<double> timestamps;

const double scan_duration = msg->time_increment * msg->ranges.size();
for (size_t i = 0; i < msg->ranges.size(); ++i) {
  // 点座標
  points.push_back(Eigen::Vector3d(x, y, 1.0));

  // 正規化タイムスタンプ [0, 1]
  const double t = (msg->time_increment * i) / scan_duration;
  timestamps.push_back(t);
}

cloud->add_points(points);
cloud->add_times(timestamps);
```

#### ポーズキー管理
```cpp
// キー i のスキャン:
//   開始ポーズ: Symbol('x', i * 2)
//   終了ポーズ: Symbol('x', i * 2 + 1)
```

---

## Tutorial 8: CT-GICP

### 概要
CT-ICPに幾何的制約（法線・共分散）を追加。より高精度なマッチングが可能です。

### アルゴリズム

#### GICP vs ICP
- **ICP**: 点間距離を最小化
- **GICP**: Mahalanobis距離を最小化（共分散を考慮）

```
ICP:  E = Σ ||p_i - q_i||²
GICP: E = Σ (p_i - q_i)ᵀ C_i^-1 (p_i - q_i)
```

#### 法線・共分散推定
```cpp
estimate_normals_2d(*cloud, k_neighbors);
estimate_covariances_2d(*cloud);
```

### 使用するgtsam_points API

```cpp
#include <gtsam_points/factors/integrated_ct_gicp_factor_2d.hpp>
#include <gtsam_points/d2/features/normal_estimation_2d.hpp>
#include <gtsam_points/d2/features/covariance_estimation_2d.hpp>

// 法線・共分散推定
estimate_normals_2d(*scan, k_neighbors_);
estimate_covariances_2d(*scan);

// CT-GICPファクター
auto ct_gicp_factor = gtsam::make_shared<IntegratedCT_GICPFactor2D>(
  Symbol('x', key * 2),
  Symbol('x', key * 2 + 1),
  target,  // 法線・共分散付き
  source   // タイムスタンプ・法線・共分散付き
);
graph_.add(ct_gicp_factor);
```

### パラメータ

```yaml
# CT-GICP
max_correspondence_distance: 1.0
k_neighbors: 10                      # 法線・共分散推定のK近傍数
```

### 実装のポイント

#### 法線推定
K近傍点でPCA（主成分分析）:
```cpp
estimate_normals_2d(*cloud, k);
// 各点にnormalベクトルが追加される
```

#### 共分散推定
局所的な点分布から共分散行列を計算:
```cpp
estimate_covariances_2d(*cloud);
// 各点に2x2共分散行列が追加される
```

#### CT-GICP vs CT-ICP
- **CT-GICP**: ノイズに強い、エッジやコーナーで高精度
- **CT-ICP**: シンプル、計算コスト低

---

## Tutorial 9: RANSAC

### 概要
RANSAC（Random Sample Consensus）でアウトライアに対処。ランダムサンプリングとコンセンサス検証で外れ値を除外します。

### アルゴリズム

#### RANSAC手順
1. **ランダムサンプリング**: 最小点数（3点）をランダム選択
2. **モデル推定**: 選択点からポーズを計算
3. **インライア判定**: 全点を変換してターゲットと比較
4. **コンセンサス**: インライア数が最大のモデルを採用

```
反復 1: サンプル{1,5,9}  → インライア 45個
反復 2: サンプル{3,7,12} → インライア 67個 ← 採用
反復 3: サンプル{2,8,15} → インライア 52個
...
```

### 使用するgtsam_points API

```cpp
#include <gtsam_points/d2/ransac/ransac_2d.hpp>

// RANSACパラメータ
RANSAC2DParams params;
params.ransac_iterations = 100;
params.inlier_threshold = 0.1;          // インライア判定距離 [m]
params.min_inliers = 50;                // 最小インライア数
params.confidence = 0.99;               // 信頼度

// RANSAC実行
RANSAC2D ransac(params);
Eigen::Isometry2d T_estimate;
std::vector<int> inliers;

bool success = ransac.estimate(
  target,           // ターゲット点群
  source,           // ソース点群
  T_estimate,       // 推定変換
  inliers           // インライアインデックス
);

if (success) {
  double inlier_ratio = static_cast<double>(inliers.size()) / source->size();
  RCLCPP_INFO("Inlier ratio: %.2f%%", inlier_ratio * 100.0);
}
```

### パラメータ

```yaml
# RANSAC
ransac_iterations: 100               # RANSAC反復回数
ransac_inlier_threshold: 0.1         # インライア判定距離 [m]
ransac_min_inliers: 50               # 最小インライア数
ransac_confidence: 0.99              # 信頼度
enable_relocalization: true          # 再ローカライゼーション
reloc_search_window: 20              # 再ローカライゼーション探索窓
```

### 実装のポイント

#### 再ローカライゼーション
マッチング失敗時、過去のキーフレーム全体から探索:
```cpp
if (!success || inlier_ratio < min_inlier_ratio_) {
  for (size_t i = 0; i < keyframes_.size(); ++i) {
    bool match_success = ransac.estimate(keyframes_[i], cloud, T, inliers);
    if (match_success && inliers.size() > best_inliers) {
      best_match = i;
      best_inliers = inliers.size();
    }
  }
}
```

#### RANSACの強み
- **ロバスト性**: 50%までのアウトライアに耐性
- **再ローカライゼーション**: 誘拐問題（kidnapped robot）に対処

---

## Tutorial 10: GNC (Graduated Non-Convexity)

### 概要
Graduated Non-Convexity（段階的非凸性）でロバストマッチング。ロバストコスト関数のパラメータμを徐々に変化させて、大域解に収束させます。

### アルゴリズム

#### Geman-McClureロバストコスト
```
ρ(r) = r² / (μ + r²)
w(r) = μ / (μ + r²)²
```

- **μ小**: 非凸、アウトライアに強い
- **μ大**: 凸、精密フィッティング

#### GNCアルゴリズム
```
1. μ = μ_init (小さい値、ロバスト)
2. 対応点探索
3. Geman-McClure重みを計算: w_i = μ/(μ + r_i²)²
4. 重み付きICP: align_points_se2(target, source, weights)
5. μ *= μ_step (μを増やす)
6. 収束まで2-5を反復
```

### 使用するgtsam_points API

```cpp
#include <gtsam_points/d2/ann/kdtree2d_tbb.hpp>
#include <gtsam_points/d2/registration/alignment_2d.hpp>

// KD-tree構築
auto target_tree = std::make_shared<KdTree2dTBB>(target);

double mu = gnc_mu_init_;  // 例: 1.0

for (int iter = 0; iter < gnc_max_iterations_; ++iter) {
  // 対応点探索
  for (size_t i = 0; i < source->size(); ++i) {
    Eigen::Vector2d src_transformed = T * source->points[i];
    target_tree->knn_search(src_transformed.data(), 1, &idx, &sq_dist);

    // Geman-McClure重み
    double weight = mu / std::pow(mu + sq_dist, 2.0);
    weights.push_back(weight);
  }

  // 重み付きアライメント
  T = align_points_se2(
    target_points.data(),
    source_points.data(),
    weights.data(),
    num_points
  );

  // μを増やす（graduated non-convexity）
  mu *= gnc_mu_step_;  // 例: 1.4
}
```

### パラメータ

```yaml
# GNC
gnc_mu_init: 1.0                     # 初期μ（小さいほどロバスト）
gnc_mu_step: 1.4                     # μ増加率
gnc_max_iterations: 50               # 最大反復回数
gnc_convergence_threshold: 1.0e-6    # 収束判定閾値
```

**パラメータ調整:**
- `gnc_mu_init`: 小さい（0.1-1.0）とロバスト、大きいとL2に近い
- `gnc_mu_step`: 1.2-1.5が一般的（小さいほど慎重）

### 実装のポイント

#### Geman-McClure重み関数
```cpp
double computeGemanMcClureWeight(double squared_residual, double mu) const {
  return mu / std::pow(mu + squared_residual, 2.0);
}
```

#### 収束判定
```cpp
double cost = 0.0;
for (size_t i = 0; i < correspondences.size(); ++i) {
  cost += weights[i] * residuals[i];
}
cost /= correspondences.size();

if (std::abs(prev_cost - cost) < gnc_convergence_threshold_) {
  break;  // 収束
}
```

#### GNC vs RANSAC
- **GNC**: 決定論的、滑らかな収束
- **RANSAC**: 確率的、より多くのアウトライアに対処可能

---

## Tutorial 11: セグメンテーションSLAM

### 概要
点群をセグメント化し、動的物体を除去。静的な環境のみでSLAMを行い、動的環境でのロバスト性を向上させます。

### アルゴリズム

#### Region Growing セグメンテーション
1. **法線推定**: K近傍でPCA
2. **シード選択**: 未割り当て点
3. **領域成長**: 距離・法線角度が閾値内の点を追加
4. **反復**: すべての点が割り当てられるまで

#### 動的物体検出
各セグメントをマップと比較:
```
static_ratio = 一致点数 / セグメント総点数

if static_ratio > threshold:
  → 静的セグメント
else:
  → 動的セグメント（除外）
```

### 使用するgtsam_points API

```cpp
#include <gtsam_points/d2/features/normal_estimation_2d.hpp>
#include <gtsam_points/d2/segmentation/region_growing_2d.hpp>
#include <gtsam_points/d2/ann/kdtree2d_tbb.hpp>

// 1. 法線推定
estimate_normals_2d(*cloud, normal_estimation_k_);

// 2. KD-tree構築
auto kdtree = std::make_shared<KdTree2dTBB>(cloud);

// 3. Region Growingパラメータ
RegionGrowingParams2D params;
params.distance_threshold = 0.2;     // 隣接判定距離 [m]
params.angle_threshold = 0.5;        // 法線角度差 [rad]
params.max_cluster_size = 1000;

// 4. 複数シードでRegion Growing
std::vector<std::vector<size_t>> segments;
std::vector<bool> assigned(cloud->size(), false);

for (size_t i = 0; i < cloud->size(); ++i) {
  if (assigned[i]) continue;

  // シード点から初期化
  Eigen::Vector3d seed(cloud->points[i].x(), cloud->points[i].y(), 1.0);
  auto context = region_growing_init_2d(*cloud, *kdtree, seed, params);

  // 領域を成長
  while (!region_growing_update_2d(context, *cloud, *kdtree, params)) {}

  // セグメント保存
  if (context.cluster_indices.size() >= min_segment_size_) {
    for (size_t idx : context.cluster_indices) {
      assigned[idx] = true;
    }
    segments.push_back(context.cluster_indices);
  }
}
```

### パラメータ

```yaml
# セグメンテーション
region_growing_distance_threshold: 0.2   # 隣接判定距離 [m]
region_growing_angle_threshold: 0.5      # 法線角度差 [rad]
min_segment_size: 15                     # 最小セグメントサイズ
max_segment_size: 1000                   # 最大セグメントサイズ
normal_estimation_k: 10                  # 法線推定K近傍数

# 動的物体検出
consistency_check_distance: 0.5          # 一致判定距離 [m]
min_static_ratio: 0.3                    # 静的判定閾値
```

### 実装のポイント

#### セグメントの静的判定
```cpp
bool isSegmentStatic(
  const std::shared_ptr<PointCloud2DCPU>& cloud,
  const std::vector<size_t>& segment,
  const gtsam::Pose2& current_pose)
{
  // セグメントをマップ座標系に変換
  auto segment_in_map = transformSegmentToMap(cloud, segment, current_pose);

  // マップとの一致点を数える
  auto reference_tree = std::make_shared<KdTree2dTBB>(keyframes_.back());
  int consistent_count = 0;

  for (size_t i = 0; i < segment_in_map->size(); ++i) {
    size_t nearest_idx;
    double sq_dist;
    reference_tree->knn_search(segment_in_map->points[i].data(), 1, &nearest_idx, &sq_dist);

    if (sq_dist < consistency_check_distance_ * consistency_check_distance_) {
      consistent_count++;
    }
  }

  double static_ratio = static_cast<double>(consistent_count) / segment_in_map->size();
  return static_ratio > min_static_ratio_;
}
```

#### セグメント可視化
```cpp
visualization_msgs::msg::MarkerArray marker_array;
for (size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx) {
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::POINTS;

  // 静的=緑、動的=赤
  if (segment_labels[seg_idx]) {
    marker.color.g = 1.0;  // 緑
  } else {
    marker.color.r = 1.0;  // 赤
  }

  marker_array.markers.push_back(marker);
}
```

---

## Tutorial 12: マップ保存・読込

### 概要
SLAMマップをディスクに保存・読込。オフライン最適化や再利用が可能になります。

### アルゴリズム

#### 保存データ
- **キーフレーム**: PCD形式（Point Cloud Data）
- **ポーズ**: JSON形式
- **メタデータ**: JSON形式（キーフレーム数、タイムスタンプなど）

#### ディレクトリ構造
```
slam_map_YYYYMMDD_HHMMSS/
  ├── map_info.json          # メタデータ
  ├── poses.json             # ポーズリスト
  ├── keyframe_000000.pcd
  ├── keyframe_000001.pcd
  └── ...
```

### 使用するgtsam_points API

```cpp
// 基本的にはファイルI/O
// gtsam_points APIはIntegratedGICPFactor2Dなどを使用
```

### 実装のポイント

#### マップ保存
```cpp
void saveMapCallback(const std_srvs::srv::Trigger::Request::SharedPtr req,
                     std_srvs::srv::Trigger::Response::SharedPtr res)
{
  // ディレクトリ作成
  std::string map_dir = createMapDirectory();

  // キーフレーム保存
  for (size_t i = 0; i < keyframes_.size(); ++i) {
    std::string pcd_path = map_dir + "/keyframe_" + std::to_string(i) + ".pcd";
    savePCD(pcd_path, keyframes_[i]);
  }

  // ポーズ保存
  nlohmann::json poses_json;
  for (size_t i = 0; i < keyframe_poses_.size(); ++i) {
    poses_json[i] = {
      {"x", keyframe_poses_[i].x()},
      {"y", keyframe_poses_[i].y()},
      {"theta", keyframe_poses_[i].theta()}
    };
  }
  std::ofstream(map_dir + "/poses.json") << poses_json.dump(2);

  // メタデータ保存
  nlohmann::json info_json;
  info_json["num_keyframes"] = keyframes_.size();
  info_json["timestamp"] = getCurrentTimestamp();
  std::ofstream(map_dir + "/map_info.json") << info_json.dump(2);
}
```

#### マップ読込
```cpp
void loadMapCallback(...)
{
  // メタデータ読込
  nlohmann::json info_json;
  std::ifstream(map_path_ + "/map_info.json") >> info_json;
  size_t num_keyframes = info_json["num_keyframes"];

  // ポーズ読込
  nlohmann::json poses_json;
  std::ifstream(map_path_ + "/poses.json") >> poses_json;

  // キーフレーム読込
  for (size_t i = 0; i < num_keyframes; ++i) {
    std::string pcd_path = map_path_ + "/keyframe_" + std::to_string(i) + ".pcd";
    auto cloud = loadPCD(pcd_path);
    keyframes_.push_back(cloud);

    gtsam::Pose2 pose(
      poses_json[i]["x"],
      poses_json[i]["y"],
      poses_json[i]["theta"]
    );
    keyframe_poses_.push_back(pose);
  }

  // ファクターグラフ再構築
  rebuildFactorGraph();
}
```

#### PCD形式
```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y
SIZE 4 4
TYPE F F
COUNT 1 1
WIDTH 512
HEIGHT 1
POINTS 512
DATA ascii
0.125 0.342
0.234 0.456
...
```

### 起動方法

```bash
# SLAM起動
ros2 launch gtsam_points_2d_slam slam_with_map_save.launch.py

# マップ保存
ros2 service call /save_map std_srvs/srv/Trigger

# マップ読込
ros2 service call /load_map std_srvs/srv/Trigger
```

---

## Tutorial 13: オフラインマップ最適化

### 概要
保存したマップをオフラインで最適化。ループクロージャを手動追加し、バッチ最適化で大域的な整合性を向上させます。

### アルゴリズム

#### バッチ最適化
ISAM2の代わりにLevenberg-Marquardt:
```cpp
gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values);
gtsam::Values result = optimizer.optimize();
```

#### ループクロージャ追加
```cpp
// 手動指定
addLoopClosure(from_idx=10, to_idx=50);

// または自動マッチング
addLoopClosureAuto(from_idx=10, to_idx=50, auto_match=true);
```

### 使用するgtsam_points API

```cpp
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// ループクロージャファクター
auto gicp_factor = gtsam::make_shared<IntegratedGICPFactor2D>(
  gtsam::Symbol('x', from_idx),
  gtsam::Symbol('x', to_idx),
  keyframes_[from_idx],
  keyframes_[to_idx]
);
graph_.add(gicp_factor);

// バッチ最適化
gtsam::LevenbergMarquardtParams lm_params;
lm_params.setVerbosity("ERROR");
gtsam::LevenbergMarquardtOptimizer optimizer(graph_, initial_values_, lm_params);
gtsam::Values optimized_values = optimizer.optimize();
```

### 実装のポイント

#### サービスベース操作
```cpp
// マップ読込
ros2 service call /load_map std_srvs/srv/SetBool "{data: true}"

// ループクロージャ追加
ros2 service call /add_loop_closure gtsam_points_2d_slam/srv/AddLoopClosure \
  "{from_keyframe: 10, to_keyframe: 50, auto_match: true}"

// 最適化実行
ros2 service call /optimize_map std_srvs/srv/Trigger

// マップ保存
ros2 service call /save_map std_srvs/srv/SetBool "{data: true}"
```

#### 最適化前後の誤差
```cpp
double initial_error = graph_.error(initial_values_);
double final_error = graph_.error(optimized_values);

RCLCPP_INFO("Error reduced: %.2f -> %.2f (%.1f%% reduction)",
  initial_error, final_error,
  (initial_error - final_error) / initial_error * 100.0);
```

### 起動方法

```bash
ros2 launch gtsam_points_2d_slam offline_map_optimizer.launch.py \
  map_path:=/path/to/slam_map_YYYYMMDD_HHMMSS
```

---

## Tutorial 14: 統合SLAM

### 概要
ループクロージャ + IMU + セグメンテーションを統合した最も高度な実装。動的環境での高速移動に対応します。

### アルゴリズム

#### 3つの技術統合
1. **セグメンテーション**: 動的物体除去
2. **IMU統合**: 高速移動対応
3. **ループクロージャ**: 大域的整合性

#### 処理フロー
```
LiDAR Scan
    ↓
セグメンテーション（動的除去）
    ↓
静的点のみでGICP Factor
    ↓
IMU Factorも追加
    ↓
ループ検出・追加
    ↓
ISAM2最適化
```

### 使用するgtsam_points API

すべてのAPIを統合:
- `region_growing_init_2d` / `update_2d`
- `ReintegratedIMUFactor2D`
- `IntegratedGICPFactor2D`
- `KdTree2dTBB`

### パラメータ

```yaml
# ループクロージャ
enable_loop_closure: true
loop_search_radius: 10.0
loop_min_chain_length: 10

# IMU
use_imu: true
imu_acc_noise: 0.1
imu_gyro_noise: 0.01
imu_acc_bias_noise: 0.001
imu_gyro_bias_noise: 0.0001

# セグメンテーション
enable_segmentation: true
region_growing_distance_threshold: 0.2
region_growing_angle_threshold: 0.5
min_segment_size: 15
consistency_check_distance: 0.5
min_static_ratio: 0.3
```

### 実装のポイント

#### スキャンコールバック
```cpp
void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  auto cloud = convertToPointCloud2D(msg);

  // 1. セグメンテーション
  auto segments = segmentPointCloud(cloud);

  // 2. 動的物体フィルタリング
  auto static_cloud = filterStaticPoints(cloud, segments, current_pose_);

  if (shouldCreateKeyframe(current_pose_)) {
    // 3. GICPファクター（静的点のみ）
    auto gicp_factor = gtsam::make_shared<IntegratedGICPFactor2D>(...);
    graph_.add(gicp_factor);

    // 4. IMUファクター
    if (use_imu_ && !imu_measurements_.empty()) {
      auto imu_factor = gtsam::make_shared<ReintegratedIMUFactor2D>(
        X(prev), V(prev), X(curr), V(curr), B(prev), B(curr), *imu_preintegration_
      );
      graph_.add(imu_factor);
      imu_preintegration_->resetIntegration();
    }

    // 5. ループクロージャ検出
    detectLoopClosures();

    // 6. 最適化
    isam2_->update(graph_, initial_estimates_);
  }
}
```

#### 機能のON/OFF
各機能は独立してON/OFFできる:
```bash
# ループのみ
ros2 launch gtsam_points_2d_slam slam_integrated.launch.py \
  enable_loop_closure:=true use_imu:=false enable_segmentation:=false

# IMUのみ
ros2 launch gtsam_points_2d_slam slam_integrated.launch.py \
  enable_loop_closure:=false use_imu:=true enable_segmentation:=false

# 全機能ON
ros2 launch gtsam_points_2d_slam slam_integrated.launch.py
```

---

## Tutorial 15: キーフレーム管理

### 概要
マップのメンテナンス用ユーティリティ。冗長なキーフレームの統合や間引きでマップを最適化します。

### アルゴリズム

#### マップのマージ
近いキーフレームを統合:
```
距離 < threshold かつ 角度差 < threshold
→ 点群をマージ、ポーズを平均
```

#### マップの間引き
一定間隔でキーフレームを削減:
```
元: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
↓ (thinning_factor=3)
後: 0, 3, 6, 9, 10  (最初と最後は保持)
```

### 使用するgtsam_points API

基本的な点群操作のみ:
```cpp
// 点群マージ
merged_cloud->add_points_from(cloud);

// ポーズ平均
gtsam::Pose2 avg_pose(
  (pose1.x() + pose2.x()) / 2,
  (pose1.y() + pose2.y()) / 2,
  (pose1.theta() + pose2.theta()) / 2
);
```

### 実装のポイント

#### マージサービス
```cpp
void mergeKeyframesCallback(...) {
  for (size_t i = 0; i < keyframes_.size(); ++i) {
    for (size_t j = i+1; j < keyframes_.size(); ++j) {
      double dist = distance(poses_[i], poses_[j]);
      double angle_diff = angleDiff(poses_[i], poses_[j]);

      if (dist < merge_distance_threshold_ &&
          angle_diff < merge_angle_threshold_) {
        // マージ
        merged_cloud->add_points_from(keyframes_[j]);
        avg_pose = averagePose(poses_[i], poses_[j]);
      }
    }
  }
}
```

#### 間引きサービス
```cpp
void thinKeyframesCallback(...) {
  std::vector<std::shared_ptr<PointCloud2DCPU>> thinned;

  // 最初は必ず保持
  thinned.push_back(keyframes_[0]);

  // thinning_factor間隔で保持
  for (size_t i = thinning_factor_; i < keyframes_.size()-1; i += thinning_factor_) {
    thinned.push_back(keyframes_[i]);
  }

  // 最後は必ず保持
  thinned.push_back(keyframes_.back());

  keyframes_ = thinned;
}
```

### 起動方法

```bash
# キーフレームマネージャー起動
ros2 launch gtsam_points_2d_slam keyframe_manager.launch.py \
  map_path:=/path/to/slam_map

# マップ読込
ros2 service call /load_map std_srvs/srv/SetBool "{data: true}"

# マージ
ros2 service call /merge_keyframes std_srvs/srv/Trigger

# 間引き
ros2 service call /thin_keyframes std_srvs/srv/Trigger

# 保存
ros2 service call /save_map std_srvs/srv/SetBool "{data: true}"
```

---

## まとめ

### 学習パス推奨

#### 初心者
1. Tutorial 1: 基本SLAM
2. Tutorial 2: LiDARオドメトリ
3. Tutorial 3: ループクロージャ

#### 中級者
4. Tutorial 4: ホイールオドメトリ
5. Tutorial 5: IMU統合
6. Tutorial 9: RANSAC
7. Tutorial 10: GNC

#### 上級者
8. Tutorial 7: CT-ICP
9. Tutorial 8: CT-GICP
10. Tutorial 11: セグメンテーション
11. Tutorial 14: 統合SLAM

#### 運用・保守
12. Tutorial 12: マップ保存・読込
13. Tutorial 13: オフライン最適化
14. Tutorial 15: キーフレーム管理

### 各技術の適用場面

| 技術 | 適用場面 |
|------|---------|
| 基本SLAM | 静的環境、低速移動 |
| ループクロージャ | 長距離走行、再訪問あり |
| ホイールオドメトリ | ホイールエンコーダー利用可能 |
| IMU | 高速移動、振動環境 |
| 固定ラグ | 長時間走行、メモリ制約 |
| CT-ICP/GICP | 高速移動（モーション補償） |
| RANSAC | アウトライア多（屋外、混雑） |
| GNC | 中程度のアウトライア |
| セグメンテーション | 動的環境（人・車が多い） |
| 統合SLAM | 最も困難な環境 |

### gtsam_points APIサマリー

#### 点群処理
- `PointCloud2DCPU`: 点群データ構造
- `estimate_normals_2d()`: 法線推定
- `estimate_covariances_2d()`: 共分散推定

#### スキャンマッチング
- `align_scans_2d()`: GICP/VGICPレジストレーション
- `align_points_se2()`: 重み付きSE(2)アライメント

#### ファクター
- `IntegratedGICPFactor2D`: GICPファクター
- `IntegratedVGICPFactor2D`: VGICPファクター
- `IntegratedCT_ICPFactor2D`: 連続時間ICPファクター
- `IntegratedCT_GICPFactor2D`: 連続時間GICPファクター
- `ReintegratedIMUFactor2D`: IMU事前積分ファクター

#### データ構造
- `IncrementalGridMap2D`: ボクセルグリッド
- `KdTree2dTBB`: K-d木（TBB並列化）

#### アルゴリズム
- `RANSAC2D`: RANSACマッチング
- `region_growing_init_2d()` / `update_2d()`: 領域成長

#### 最適化
- `ISAM2`: 増分平滑化
- `IncrementalFixedLagSmootherExt`: 固定ラグスムーザー
- `LevenbergMarquardtOptimizer`: バッチ最適化

---

## トラブルシューティング

### マッチング失敗
**症状**: `Scan matching did not converge`

**原因**:
- 初期推定が悪い
- 対応点が少ない
- アウトライアが多い

**対処**:
- `max_correspondence_distance`を大きく
- RANSACやGNCを使用
- キーフレーム間隔を短く

### メモリ不足
**症状**: プロセス強制終了

**対処**:
- 固定ラグスムーザー（Tutorial 6）を使用
- `smoother_lag`を短く（例: 30秒 → 15秒）
- `max_keyframes`を制限

### ドリフト蓄積
**症状**: マップが歪む

**対処**:
- ループクロージャを有効化
- `loop_search_radius`を広げる
- オフライン最適化で手動ループ追加

### IMUノイズ
**症状**: IMU使用時に推定が不安定

**対処**:
- `imu_acc_noise`, `imu_gyro_noise`を大きく
- キャリブレーション実施
- IMUバイアス推定を確認

### 動的物体による誤マッチ
**症状**: 人や車でマップが乱れる

**対処**:
- セグメンテーションSLAM（Tutorial 11）を使用
- `min_static_ratio`を調整（0.3 → 0.5など）

---

## 参考文献

1. **GTSAM**: Georgia Tech Smoothing and Mapping library
   - https://gtsam.org/

2. **gtsam_points**: Point cloud processing library for GTSAM
   - https://github.com/koide3/gtsam_points

3. **GICP**: Generalized ICP
   - Segal, A., et al. "Generalized-ICP." RSS 2009

4. **CT-ICP**: Continuous-Time ICP
   - Dellenbach, P., et al. "CT-ICP." ICRA 2022

5. **GNC**: Graduated Non-Convexity
   - Yang, H., et al. "Polynomial-time Certifiable Estimation." CVPR 2020

6. **RANSAC**: Random Sample Consensus
   - Fischler, M.A., and Bolles, R.C. "Random Sample Consensus." Comm. ACM 1981

---

このドキュメントは gtsam_points 2D SLAM チュートリアルの完全ガイドです。
各チュートリアルの詳細、APIリファレンス、実装のポイントを網羅しています。

実装に関する質問や問題がある場合は、GitHub Issuesでお問い合わせください。
