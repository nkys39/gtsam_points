# GTSAM Points 2D SLAM for TurtleBot3

ROS2実装例：gtsam_pointsライブラリを使用したTurtleBot3シミュレーション用の2D SLAMノード

## 概要

このパッケージは、gtsam_pointsの2D SLAM機能を使用して、TurtleBot3シミュレーション環境で動作する**4種類の異なる2D SLAM実装例**を提供します。

### 4つの実装例

| 実装 | ノード名 | 特徴 | 用途 |
|------|----------|------|------|
| **1. LiDARオドメトリ** | `lidar_odometry_node` | スキャンマッチングのみ、グラフ最適化なし | 高速な軌跡推定、デッドレコニング |
| **2. 基本SLAM** | `slam_node` | ISAM2によるグラフSLAM、ループクロージャーなし | 小規模環境での高精度マッピング |
| **3. ループクロージャー付きSLAM** | `slam_with_loop_closure_node` | ループクロージャー検出と因子追加 | 大規模環境、長時間運用 |
| **4. Wheel Odometry統合SLAM** | `slam_with_wheel_odom_node` | ホイールオドメトリとLiDARの融合 | 高精度かつロバストな位置推定 |

---

## 1. LiDARオドメトリ（lidar_odometry_node）

### 特徴
- ✅ **シンプル**: スキャンマッチングのみで軽量
- ✅ **高速**: グラフ最適化なしでリアルタイム処理
- ❌ **累積誤差**: 長時間運用で誤差が蓄積
- ❌ **ループクロージャーなし**

### アーキテクチャ
```
LaserScan → Scan-to-Scan or Scan-to-Map Matching → Odometry (Dead Reckoning)
                        ↓
                   GICP / VGICP
```

### 使用方法
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch gtsam_points_2d_slam lidar_odometry.launch.py

# キーボードで操作
ros2 run turtlebot3_teleop teleop_keyboard
```

### パラメータ（`config/lidar_odometry_params.yaml`）
- `use_scan_to_map`: スキャン対マップ（true）またはスキャン対スキャン（false）
- `use_vgicp`: VGICP（true）またはGICP（false）

---

## 2. 基本SLAM（slam_node）

### 特徴
- ✅ **グラフSLAM**: ISAM2による増分最適化
- ✅ **高精度**: スキャンマッチングとグラフ最適化の組み合わせ
- ❌ **ループクロージャーなし**: 大規模環境では累積誤差あり

### アーキテクチャ
```
LaserScan → Keyframe Selection → Factor Graph Optimization (ISAM2)
                 ↓                         ↓
            GICP/VGICP Factor    Between Factors (Odometry)
```

### ファクターグラフ構造
```
x0 --[Prior]
x0 --[Between]-- x1 --[Between]-- x2 --[Between]-- x3
 |               |                |                |
[GICP]         [GICP]           [GICP]          [GICP]
```

### 使用方法
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch gtsam_points_2d_slam turtlebot3_slam.launch.py

# キーボードで操作
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## 3. ループクロージャー付きSLAM（slam_with_loop_closure_node）

### 特徴
- ✅ **ループクロージャー検出**: 距離ベース + スキャンマッチング検証
- ✅ **グローバル一貫性**: 累積誤差の自動修正
- ✅ **大規模環境対応**: 長時間運用でも高精度維持
- ⚠️ **計算コスト**: ループ検出時に再最適化

### アーキテクチャ
```
LaserScan → Keyframe Selection → Loop Closure Detection → ISAM2 Optimization
                 ↓                         ↓                      ↓
            GICP/VGICP             Distance Check       全軌跡の再最適化
                                   + Scan Matching
```

### ファクターグラフ構造（ループクロージャー後）
```
x0 --[Between]-- x1 --[Between]-- x2 --[Between]-- x3 --[Between]-- x10
 |               |                |                |                 |
[GICP]         [GICP]           [GICP]          [GICP]           [GICP]
 |                                                                   |
 +--------------------[Loop Closure BetweenFactor]-----------------+
```

### ループクロージャーの仕組み

1. **候補検出**: 現在位置から一定距離内の過去のキーフレームを検索
2. **検証**: スキャンマッチングで相対姿勢を推定
3. **制約追加**: **BetweenFactorとして因子グラフに追加**
4. **最適化**: ISAM2が全軌跡を再最適化し、累積誤差を修正

### 使用方法
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch gtsam_points_2d_slam slam_with_loop_closure.launch.py

# キーボードで操作し、同じ場所に戻るとループクロージャーが発動
ros2 run turtlebot3_teleop teleop_keyboard
```

### パラメータ（`config/slam_with_loop_closure_params.yaml`）
- `enable_loop_closure`: ループクロージャーの有効化
- `loop_search_radius`: ループ候補の検索半径 [m]
- `loop_min_chain_length`: ループクロージャーを開始する最小キーフレーム数

### RVizでの確認
- **緑の線**: ループクロージャー制約（`/loop_closure_markers`）
- **パス**: 最適化後の軌跡（`/slam_path`）

---

## 4. Wheel Odometry統合SLAM（slam_with_wheel_odom_node）

### 特徴
- ✅ **センサーフュージョン**: ホイールオドメトリ + LiDARスキャンマッチング
- ✅ **ロバスト**: どちらかのセンサーが一時的に使えなくても継続
- ✅ **スムーズな軌跡**: ホイールオドメトリで補間
- ⚠️ **重み調整が必要**: 環境に応じてパラメータ調整

### アーキテクチャ
```
LaserScan + Wheel Odometry → Factor Graph with Dual Constraints → ISAM2
       ↓             ↓                    ↓
   GICP/VGICP  BetweenFactor    Weighted Fusion
   (高重み)     (低重み)
```

### ファクターグラフ構造
```
x0 --[Between(Wheel)]-- x1 --[Between(Wheel)]-- x2
 |                       |                        |
[GICP(高重み)]        [GICP(高重み)]          [GICP(高重み)]
```

**重み設定の意味:**
- **LiDAR重み（高）**: 絶対的な位置推定、信頼性が高い
- **Wheel重み（低）**: 相対的な移動量、スリップやドリフトの影響を受ける

### 使用方法
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch gtsam_points_2d_slam slam_with_wheel_odom.launch.py

# キーボードで操作
ros2 run turtlebot3_teleop teleop_keyboard
```

### パラメータ（`config/slam_with_wheel_odom_params.yaml`）
- `use_wheel_odom`: ホイールオドメトリ統合の有効化
- `wheel_odom_weight`: ホイールオドメトリの重み（小さいほど不確実性が高い）
- `lidar_odom_weight`: LiDARオドメトリの重み（大きいほど信頼性が高い）

---

## 必要な依存関係

### システム依存

```bash
# GTSAM 4.3a0
# gtsam_points (2D SLAM機能付き)
# Eigen3
```

### ROS2パッケージ

```bash
sudo apt install ros-${ROS_DISTRO}-turtlebot3-gazebo \
                 ros-${ROS_DISTRO}-turtlebot3-description \
                 ros-${ROS_DISTRO}-rviz2
```

## インストール

### 1. gtsam_pointsのビルドとインストール

まず、gtsam_pointsライブラリ（2D SLAM機能付き）をビルドしてインストールします：

```bash
cd /path/to/gtsam_points
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### 2. ROS2ワークスペースのセットアップ

```bash
# ワークスペースの作成
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# このパッケージをシンボリックリンクまたはコピー
ln -s /path/to/gtsam_points/ros2_examples/gtsam_points_2d_slam .

# ビルド
cd ~/ros2_ws
colcon build --packages-select gtsam_points_2d_slam
source install/setup.bash
```

## 実装の比較

| 機能 | LiDAR Odom | 基本SLAM | ループクロージャー | Wheel統合 |
|------|------------|----------|-------------------|-----------|
| グラフ最適化 | ❌ | ✅ | ✅ | ✅ |
| ループクロージャー | ❌ | ❌ | ✅ | ❌ |
| Wheel Odom統合 | ❌ | ❌ | ❌ | ✅ |
| 計算コスト | 低 | 中 | 高 | 中 |
| 累積誤差 | 大 | 中 | 小 | 小 |
| 適用環境 | 小規模 | 中規模 | 大規模 | 全般 |

## パラメータ調整ガイド

### 共通パラメータ

- `keyframe_distance`: 小さい→密な地図、大きい→疎な地図
- `voxel_resolution`: 小さい→高精度、大きい→高速
- `use_vgicp`: VGICP（高速）vs GICP（高精度）

### ループクロージャー固有

- `loop_search_radius`: 環境サイズに応じて調整（小規模: 3m、大規模: 10m）
- `loop_min_chain_length`: 短いほど早くループ検出、長いほど誤検出が減る

### Wheel Odometry固有

- `wheel_odom_weight` / `lidar_odom_weight`: 比率が重要
  - 滑りやすい床: LiDAR重みを上げる
  - LiDAR遮蔽が多い: Wheel重みを上げる

## トラブルシューティング

### ビルドエラー

```bash
# gtsam_pointsが見つからない場合
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH

# または、明示的にパスを指定
colcon build --cmake-args -Dgtsam_points_DIR=/usr/local/lib/cmake/gtsam_points
```

### 実行時エラー

```bash
# TurtleBot3モデルが設定されていない場合
export TURTLEBOT3_MODEL=waffle_pi

# Gazeboが起動しない場合
source /usr/share/gazebo/setup.sh
```

## ベンチマークと評価

TurtleBot3 Gazebo環境での性能比較（参考値）：

| 実装 | 処理時間/frame | 最終誤差 | メモリ使用量 |
|------|---------------|----------|-------------|
| LiDAR Odom | ~10ms | 大 | 低 |
| 基本SLAM | ~50ms | 中 | 中 |
| ループクロージャー | ~100ms* | 小 | 高 |
| Wheel統合 | ~50ms | 小 | 中 |

*ループクロージャー検出時はさらに増加

## 今後の改善予定

- [ ] マップの保存・読み込み機能
- [ ] RViz用の設定ファイル追加
- [ ] IMU統合（ReintegratedIMUFactor2D使用）
- [ ] Continuous-Time SLAM (CT-ICP/CT-GICP)の統合
- [ ] パフォーマンスの最適化
- [ ] ベンチマークデータセットでの評価

## ライセンス

MIT License

## 参考資料

- [gtsam_points 2D SLAM ドキュメント](../../docs/2D_SLAM_README_JA.md)
- [TurtleBot3 マニュアル](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [GTSAM ドキュメント](https://gtsam.org/)
- [ISAM2 論文](https://ieeexplore.ieee.org/document/5979641)
