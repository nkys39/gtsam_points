# GTSAM Points 2D SLAM for TurtleBot3

ROS2実装例：gtsam_pointsライブラリを使用したTurtleBot3シミュレーション用の2D SLAMノード

## 概要

このパッケージは、gtsam_pointsの2D SLAM機能を使用して、TurtleBot3シミュレーション環境で動作する2D SLAMシステムを提供します。

### 主な機能

- **GTSAM-based SLAM**: GTSAMファクターグラフを使用した高精度な最適化
- **GICP/VGICP**: Generalized ICP と Voxelized GICP によるスキャンマッチング
- **キーフレーム管理**: 効率的なメモリ使用と計算負荷の低減
- **リアルタイム処理**: ISAM2による増分最適化

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

## 使用方法

### TurtleBot3モデルの設定

```bash
export TURTLEBOT3_MODEL=waffle_pi
```

### シミュレーションの起動

```bash
# すべてを一度に起動（Gazebo + SLAM + RViz）
ros2 launch gtsam_points_2d_slam turtlebot3_slam.launch.py
```

### 個別にノードを起動する場合

```bash
# ターミナル1: Gazeboシミュレーション
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# ターミナル2: SLAMノード
ros2 run gtsam_points_2d_slam slam_node --ros-args --params-file config/slam_params.yaml

# ターミナル3: RViz
ros2 run rviz2 rviz2
```

### ロボットの操作

```bash
# キーボードで操作
ros2 run turtlebot3_teleop teleop_keyboard
```

## パラメータ設定

`config/slam_params.yaml` で以下のパラメータを調整できます：

- `map_frame`: マップフレームID (デフォルト: "map")
- `base_frame`: ベースフレームID (デフォルト: "base_footprint")
- `use_vgicp`: VGICP使用フラグ (デフォルト: true)
- `keyframe_distance`: キーフレーム追加の距離閾値 [m] (デフォルト: 0.5)
- `keyframe_angle`: キーフレーム追加の角度閾値 [rad] (デフォルト: 0.3)
- `voxel_resolution`: ボクセル解像度 [m] (デフォルト: 0.1)
- `max_correspondence_distance`: 対応点探索の最大距離 [m] (デフォルト: 1.0)

## トピック

### Subscribe

- `/scan` (sensor_msgs/LaserScan): 2D LiDARスキャンデータ

### Publish

- `slam_odom` (nav_msgs/Odometry): SLAM推定の位置姿勢
- `map` (nav_msgs/OccupancyGrid): 占有格子地図
- `/tf` (tf2_msgs/TFMessage): マップ→ベースリンクの変換

## アーキテクチャ

```
LaserScan → LaserScan変換 → スキャンマッチング → GTSAM最適化 → Odometry + TF
                 ↓                    ↓                    ↓
            PointCloud2D         GICP/VGICP          ISAM2 Factor Graph
                                                           ↓
                                                     Optimized Poses
```

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

## 今後の改善予定

- [ ] マップの保存・読み込み機能
- [ ] ループクロージャの実装
- [ ] IMU統合（ReintegratedIMUFactor2D使用）
- [ ] Continuous-Time SLAM (CT-ICP/CT-GICP)の統合
- [ ] パフォーマンスの最適化
- [ ] RViz用の設定ファイル追加

## ライセンス

MIT License

## 参考資料

- [gtsam_points 2D SLAM ドキュメント](../../docs/2D_SLAM_README_JA.md)
- [TurtleBot3 マニュアル](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [GTSAM ドキュメント](https://gtsam.org/)
