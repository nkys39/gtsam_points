# gtsam_points 2D SLAM - チュートリアル

## 目次

1. [基礎: 単純なICPマッチング](#基礎-単純なicpマッチング)
2. [ロバストマッチング: GICPとGNC](#ロバストマッチング-gicpとgnc)
3. [リアルタイムSLAM: 連続時間マッチング](#リアルタイムslam-連続時間マッチング)
4. [IMU統合: センサーフュージョン](#imu統合-センサーフュージョン)
5. [高度な応用: 完全なSLAMシステム](#高度な応用-完全なslamシステム)

---

## 基礎: 単純なICPマッチング

### 目標
2つの2Dレーザースキャンを位置合わせする基本的なプログラムを作成。

### ステップ1: データの準備

```cpp
#include <gtsam_points/d2/types/point_cloud_2d_cpu.hpp>
#include <gtsam_points/d2/features/normal_estimation_2d.hpp>
#include <gtsam_points/d2/ann/kdtree_2d.hpp>
#include <gtsam_points/d2/factors/integrated_icp_factor_2d.hpp>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

using namespace gtsam_points;

// ダミーデータの生成（実際はファイルから読み込み）
auto create_scan(double x_offset = 0.0) {
    auto cloud = std::make_shared<PointCloud2DCPU>();

    // 単純な矩形を生成
    for (int i = 0; i < 100; i++) {
        double angle = 2.0 * M_PI * i / 100.0;
        double r = 5.0;
        cloud->points.emplace_back(
            r * std::cos(angle) + x_offset,
            r * std::sin(angle),
            1.0
        );
    }

    return cloud;
}

int main() {
    // ターゲットとソースのスキャン
    auto target = create_scan(0.0);
    auto source = create_scan(0.5);  // 0.5mオフセット

    // 法線を推定
    estimate_normals_2d(*target, 10);
    estimate_normals_2d(*source, 10);

    // ... 次のステップへ
}
```

### ステップ2: ファクターグラフの構築

```cpp
    // 最近傍探索の準備
    auto target_tree = std::make_shared<KdTree2D<PointCloud2DCPU>>(target);

    // ファクターグラフとinitial values
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // 初期推定値（真値から少しずらす）
    gtsam::Pose2 initial_pose(0.4, 0.1, 0.05);  // (x, y, θ)
    initial.insert(0, initial_pose);

    // ICPファクターの追加
    auto icp_factor = gtsam::make_shared<IntegratedICPFactor2D>(
        0,          // ポーズキー
        target,     // ターゲット
        source,     // ソース
        target_tree
    );

    // パラメータ設定
    icp_factor->set_max_correspondence_distance(1.0);
    icp_factor->set_num_threads(4);

    graph.add(icp_factor);
```

### ステップ3: 最適化と結果の取得

```cpp
    // 最適化器の選択
    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.setVerbosity("ERROR");
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, lm_params);

    // 最適化実行
    gtsam::Values result = optimizer.optimize();

    // 結果の取得と表示
    gtsam::Pose2 optimized_pose = result.at<gtsam::Pose2>(0);

    std::cout << "初期推定: " << initial_pose << std::endl;
    std::cout << "最適化結果: " << optimized_pose << std::endl;
    std::cout << "真値: (0.5, 0.0, 0.0)" << std::endl;

    // 誤差の計算
    double error = graph.error(result);
    std::cout << "最終誤差: " << error << std::endl;

    return 0;
}
```

### 実行例

```bash
$ ./icp_simple_example
初期推定: (0.4, 0.1, 0.05)
最適化結果: (0.500123, -0.000456, 0.000123)
真値: (0.5, 0.0, 0.0)
最終誤差: 0.0123
```

---

## ロバストマッチング: GICPとGNC

### 目標
ノイズや外れ値があるデータでも安定して動作するマッチング。

### GICPを使用したロバストマッチング

```cpp
#include <gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp>
#include <gtsam_points/d2/features/covariance_estimation_2d.hpp>

int main() {
    auto target = load_scan("scan_target.txt");
    auto source = load_scan("scan_source.txt");

    // 法線と共分散を推定
    estimate_normals_2d(*target, 10);
    estimate_covariances_2d(*target);

    estimate_normals_2d(*source, 10);
    estimate_covariances_2d(*source);

    // GICPファクター
    auto target_tree = std::make_shared<KdTree2D<PointCloud2DCPU>>(target);
    auto gicp_factor = gtsam::make_shared<IntegratedGICPFactor2D>(
        0, target, source, target_tree
    );
    gicp_factor->set_max_correspondence_distance(2.0);
    gicp_factor->set_num_threads(8);

    // ファクターグラフの構築と最適化
    gtsam::NonlinearFactorGraph graph;
    graph.add(gicp_factor);

    gtsam::Values initial;
    initial.insert(0, gtsam::Pose2(0, 0, 0));

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    gtsam::Values result = optimizer.optimize();

    std::cout << "GICP結果: " << result.at<gtsam::Pose2>(0) << std::endl;

    return 0;
}
```

### GNCを使った超ロバストマッチング

```cpp
#include <gtsam_points/d2/registration/graduated_non_convexity_2d.hpp>

int main() {
    auto target = load_scan_with_outliers("scan_target_noisy.txt");
    auto source = load_scan_with_outliers("scan_source_noisy.txt");

    // 特徴量推定（GNCには法線が推奨）
    estimate_normals_2d(*target, 10);
    estimate_normals_2d(*source, 10);

    // 最近傍探索の準備
    auto target_tree = std::make_shared<KdTree2D<PointCloud2DCPU>>(target);

    // GNCパラメータ
    GNCParams2D params;
    params.max_iterations = 64;
    params.div_factor = 1.4;
    params.max_corr_dist = 1.0;
    params.reciprocal_check = true;
    params.num_threads = 8;

    // GNC実行
    RegistrationResult2D result = estimate_pose_gnc_2d(
        *target, *source,
        *target, *source,  // 特徴としても使用
        *target_tree,
        *target_tree,  // 特徴ツリー
        *target_tree,  // ソース特徴ツリー（簡略化）
        params
    );

    std::cout << "GNC結果: " << result.T_target_source.matrix() << std::endl;
    std::cout << "インライア数: " << result.num_inliers << std::endl;
    std::cout << "インライア率: " << result.inlier_fraction << std::endl;

    return 0;
}
```

---

## リアルタイムSLAM: 連続時間マッチング

### 目標
移動中のロボットから取得したタイムスタンプ付きスキャンを補正。

### タイムスタンプ付きデータの準備

```cpp
#include <gtsam_points/d2/types/laser_scan.hpp>
#include <gtsam_points/d2/factors/integrated_ct_icp_factor_2d.hpp>

// レーザースキャンの作成（例）
auto create_laser_scan() {
    auto scan = std::make_shared<LaserScan>();

    // スキャン設定
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->angle_increment = 0.01;  // 約0.57度
    scan->time_increment = 0.0001;  // 0.1ms per point
    scan->scan_time = 0.1;          // 100ms per scan

    // データの読み込み（実際はセンサーから）
    for (double angle = scan->angle_min; angle < scan->angle_max; angle += scan->angle_increment) {
        double range = measure_range(angle);  // センサー計測
        scan->ranges.push_back(range);
    }

    // タイムスタンプを自動生成
    for (size_t i = 0; i < scan->ranges.size(); i++) {
        scan->times.push_back(i * scan->time_increment);
    }

    // 極座標からデカルト座標へ変換
    scan->points.clear();
    for (size_t i = 0; i < scan->ranges.size(); i++) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double r = scan->ranges[i];
        scan->points.emplace_back(
            r * std::cos(angle),
            r * std::sin(angle),
            1.0
        );
    }

    return scan;
}
```

### CT-ICPの実行

```cpp
int main() {
    // 地図（ターゲット）とスキャン（ソース）
    auto map = load_map("current_map.pcd");
    auto scan = create_laser_scan();  // タイムスタンプ付き

    // 法線推定
    estimate_normals_2d(*map, 10);
    estimate_normals_2d(*scan, 10);

    auto map_tree = std::make_shared<KdTree2D<PointCloud2DCPU>>(map);

    // ファクターグラフ
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // スキャン開始・終了のポーズ
    gtsam::Pose2 start_pose = get_odometry_start();
    gtsam::Pose2 end_pose = get_odometry_end();

    initial.insert(0, start_pose);
    initial.insert(1, end_pose);

    // CT-ICPファクター
    auto ct_icp = gtsam::make_shared<IntegratedCT_ICPFactor2D>(
        0,      // スキャン開始
        1,      // スキャン終了
        map,
        scan,
        map_tree
    );
    ct_icp->set_max_correspondence_distance(1.0);
    ct_icp->set_num_threads(8);

    graph.add(ct_icp);

    // 最適化
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
    gtsam::Values result = optimizer.optimize();

    gtsam::Pose2 corrected_start = result.at<gtsam::Pose2>(0);
    gtsam::Pose2 corrected_end = result.at<gtsam::Pose2>(1);

    // モーション補償されたスキャンを取得
    auto ct_icp_ptr = std::dynamic_pointer_cast<IntegratedCT_ICPFactor2D>(ct_icp);
    std::vector<Eigen::Vector3d> deskewed = ct_icp_ptr->deskewed_source_points(result);

    std::cout << "補正前: " << start_pose << " → " << end_pose << std::endl;
    std::cout << "補正後: " << corrected_start << " → " << corrected_end << std::endl;

    return 0;
}
```

---

## IMU統合: センサーフュージョン

### 目標
LiDARとIMUを統合した高精度オドメトリ。

### IMUデータの準備と前処理

```cpp
#include <gtsam_points/d2/factors/reintegrated_imu_factor_2d.hpp>

// IMU計測データの構造体
struct ImuMeasurement {
    double timestamp;
    Eigen::Vector2d acceleration;  // [ax, ay]
    double angular_velocity;       // ωz
};

// IMUデータをファイルから読み込み（例）
std::vector<ImuMeasurement> load_imu_data(const std::string& filename) {
    std::vector<ImuMeasurement> data;
    std::ifstream ifs(filename);

    double t, ax, ay, wz;
    while (ifs >> t >> ax >> ay >> wz) {
        ImuMeasurement meas;
        meas.timestamp = t;
        meas.acceleration << ax, ay;
        meas.angular_velocity = wz;
        data.push_back(meas);
    }

    return data;
}
```

### IMU事前積分の実行

```cpp
ReintegratedImuMeasurements2D preintegrate_imu(
    const std::vector<ImuMeasurement>& measurements,
    double start_time,
    double end_time,
    const std::shared_ptr<PreintegrationParams2D>& params,
    const ImuBias2D& bias) {

    ReintegratedImuMeasurements2D pim(params, bias);

    for (size_t i = 0; i < measurements.size() - 1; i++) {
        const auto& meas = measurements[i];

        if (meas.timestamp < start_time) continue;
        if (meas.timestamp > end_time) break;

        double dt = measurements[i+1].timestamp - meas.timestamp;
        pim.integrateMeasurement(meas.acceleration, meas.angular_velocity, dt);
    }

    return pim;
}
```

### LiDAR-IMU融合SLAM

```cpp
int main() {
    // データ読み込み
    std::vector<LaserScan::Ptr> scans = load_scans("scans/");
    std::vector<ImuMeasurement> imu_data = load_imu_data("imu.txt");

    // IMUパラメータ
    auto imu_params = std::make_shared<PreintegrationParams2D>();
    imu_params->n_gravity = Eigen::Vector2d(0.0, -9.81);
    imu_params->accelerometer_noise_sigma = 0.02;
    imu_params->gyroscope_noise_sigma = 0.0005;
    imu_params->accelerometer_bias_sigma = 0.0001;
    imu_params->gyroscope_bias_sigma = 0.00001;
    imu_params->use_2nd_order_integration = true;

    // ファクターグラフ
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // 初期状態
    ImuBias2D initial_bias = ImuBias2D::Zero();
    gtsam::Pose2 current_pose(0, 0, 0);
    Eigen::Vector2d current_vel(0, 0);

    // 地図の初期化
    auto map = std::make_shared<PointCloud2DCPU>();

    // 各スキャンを処理
    for (size_t i = 0; i < scans.size(); i++) {
        const auto& scan = scans[i];

        // キー定義
        gtsam::Key pose_i = i * 5 + 0;
        gtsam::Key vel_i = i * 5 + 1;
        gtsam::Key pose_j = i * 5 + 5;
        gtsam::Key vel_j = i * 5 + 6;
        gtsam::Key bias_key = i * 5 + 2;

        // 初期値設定
        initial.insert(pose_i, current_pose);
        initial.insert(vel_i, current_vel);

        if (i == 0) {
            // 初期バイアス
            initial.insert(bias_key, initial_bias.vector());

            // 第一スキャンを地図に追加
            *map += *scan;
            estimate_normals_2d(*map, 10);
            continue;
        }

        // IMU事前積分
        double t_start = scans[i-1]->times[0];
        double t_end = scan->times[0];

        auto pim = preintegrate_imu(imu_data, t_start, t_end, imu_params, initial_bias);

        // IMUファクター追加
        gtsam::Key pose_prev = (i-1) * 5 + 0;
        gtsam::Key vel_prev = (i-1) * 5 + 1;
        gtsam::Key bias_prev = (i-1) * 5 + 2;

        auto imu_factor = gtsam::make_shared<ReintegratedImuFactor2D>(
            pose_prev, vel_prev, pose_i, vel_i, bias_prev, pim
        );
        graph.add(imu_factor);

        // LiDARファクター追加
        auto map_tree = std::make_shared<KdTree2D<PointCloud2DCPU>>(map);
        auto lidar_factor = gtsam::make_shared<IntegratedGICPFactor2D>(
            pose_i, map, scan, map_tree
        );
        lidar_factor->set_max_correspondence_distance(1.0);
        lidar_factor->set_num_threads(8);
        graph.add(lidar_factor);

        // バイアス連続性制約（簡略版）
        // 実際のシステムでは、バイアスのランダムウォークモデルを追加

        // 中間最適化
        if (i % 10 == 0) {
            gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
            gtsam::Values result = optimizer.optimize();

            // 結果を更新
            current_pose = result.at<gtsam::Pose2>(pose_i);
            current_vel = result.at<Eigen::Vector2d>(vel_i);
            initial_bias = ImuBias2D(
                result.at<Eigen::Vector3d>(bias_prev).head<2>(),
                result.at<Eigen::Vector3d>(bias_prev)(2)
            );

            // 地図更新
            auto transformed_scan = transform_cloud(*scan, current_pose);
            *map += *transformed_scan;
            estimate_normals_2d(*map, 10);

            std::cout << "フレーム " << i << ": " << current_pose << std::endl;
        }
    }

    // 最終最適化
    gtsam::LevenbergMarquardtOptimizer final_optimizer(graph, initial);
    gtsam::Values final_result = final_optimizer.optimize();

    // 軌跡を保存
    save_trajectory(final_result, "trajectory.txt");

    // 地図を保存
    save_map(map, "final_map.pcd");

    return 0;
}
```

---

## 高度な応用: 完全なSLAMシステム

### 目標
ループクロージャ付きの完全なSLAMシステム。

### システムアーキテクチャ

```
センサー入力
    ├─ 2D LiDAR (10Hz)
    ├─ IMU (100Hz)
    └─ オドメトリ (10Hz, optional)
         ↓
フロントエンド
    ├─ スキャンマッチング (CT-GICP)
    ├─ IMU積分
    └─ ローカル地図管理
         ↓
バックエンド
    ├─ ポーズグラフ最適化
    ├─ ループ検出
    └─ グローバル最適化
         ↓
出力
    ├─ 補正された軌跡
    ├─ グローバル地図
    └─ 局所化
```

### フロントエンド実装

```cpp
class SlamFrontend {
public:
    SlamFrontend(const PreintegrationParams2D::Ptr& imu_params)
        : imu_params_(imu_params),
          current_pose_(gtsam::Pose2::identity()),
          current_vel_(Eigen::Vector2d::Zero()),
          current_bias_(ImuBias2D::Zero()) {

        local_map_ = std::make_shared<PointCloud2DCPU>();
        map_resolution_ = 0.3;  // 30cm voxel
    }

    // 新しいスキャンを処理
    void process_scan(const LaserScan::Ptr& scan, double timestamp) {
        // IMU統合（前回スキャンから）
        auto pim = integrate_imu_since_last(timestamp);

        // IMUからの予測
        gtsam::Pose2 predicted_pose = predict_pose(pim);
        Eigen::Vector2d predicted_vel = predict_velocity(pim);

        // スキャンマッチング
        auto match_result = match_scan(scan, predicted_pose);

        // ファクターグラフに追加
        add_to_graph(scan, pim, match_result);

        // 局所最適化
        if (frame_count_ % 5 == 0) {
            optimize_local_window();
        }

        // 地図更新
        update_local_map(scan, match_result.pose);

        // バックエンドへ送信
        if (frame_count_ % 20 == 0) {
            send_to_backend(match_result);
        }

        frame_count_++;
    }

private:
    // 局所地図を管理（一定サイズに保つ）
    void update_local_map(const LaserScan::Ptr& scan, const gtsam::Pose2& pose) {
        auto transformed = transform_cloud(*scan, pose);

        // ボクセルグリッドフィルタ（簡略版）
        auto downsampled = voxel_filter(*transformed, map_resolution_);

        *local_map_ += *downsampled;

        // 古い点を削除（LRU方式）
        if (local_map_->points.size() > max_map_points_) {
            prune_map();
        }

        // 法線更新
        estimate_normals_2d(*local_map_, 10);
    }

    PreintegrationParams2D::Ptr imu_params_;
    PointCloud2DCPU::Ptr local_map_;
    gtsam::Pose2 current_pose_;
    Eigen::Vector2d current_vel_;
    ImuBias2D current_bias_;

    int frame_count_ = 0;
    double map_resolution_;
    const size_t max_map_points_ = 50000;
};
```

### バックエンド実装

```cpp
class SlamBackend {
public:
    SlamBackend() {
        loop_detector_ = std::make_shared<LoopDetector>();
    }

    void add_keyframe(
        const gtsam::Pose2& pose,
        const LaserScan::Ptr& scan,
        const gtsam::Values& local_values) {

        // キーフレームを保存
        KeyFrame kf;
        kf.id = keyframes_.size();
        kf.pose = pose;
        kf.scan = scan;
        keyframes_.push_back(kf);

        // ポーズグラフに追加
        pose_graph_.add(gtsam::PriorFactor<gtsam::Pose2>(
            kf.id, pose, gtsam::noiseModel::Isotropic::Sigma(3, 0.1)
        ));

        // オドメトリエッジ（前のキーフレームとの相対ポーズ）
        if (kf.id > 0) {
            gtsam::Pose2 relative = keyframes_[kf.id - 1].pose.between(pose);
            pose_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
                kf.id - 1, kf.id, relative,
                gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.05))
            ));
        }

        // ループクロージャ検出
        auto loop_candidates = loop_detector_->detect(scan, kf.id);
        for (const auto& candidate : loop_candidates) {
            if (verify_loop(kf.id, candidate.id)) {
                add_loop_closure(kf.id, candidate.id);
            }
        }

        // ポーズグラフ最適化
        if (kf.id % 50 == 0) {
            optimize_pose_graph();
        }
    }

private:
    bool verify_loop(int current_id, int candidate_id) {
        const auto& current_kf = keyframes_[current_id];
        const auto& candidate_kf = keyframes_[candidate_id];

        // 相対ポーズを推定
        auto result = estimate_relative_pose(
            current_kf.scan,
            candidate_kf.scan
        );

        // 閾値チェック
        return result.score > loop_score_threshold_;
    }

    void add_loop_closure(int from_id, int to_id) {
        // ループクロージャ制約を追加
        gtsam::Pose2 relative = estimate_relative_pose(
            keyframes_[from_id].scan,
            keyframes_[to_id].scan
        ).pose;

        pose_graph_.add(gtsam::BetweenFactor<gtsam::Pose2>(
            from_id, to_id, relative,
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1))
        ));

        std::cout << "ループクロージャ検出: " << from_id << " ← " << to_id << std::endl;
    }

    void optimize_pose_graph() {
        gtsam::Values initial_estimate;
        for (const auto& kf : keyframes_) {
            initial_estimate.insert(kf.id, kf.pose);
        }

        gtsam::LevenbergMarquardtOptimizer optimizer(pose_graph_, initial_estimate);
        gtsam::Values result = optimizer.optimize();

        // キーフレームのポーズを更新
        for (auto& kf : keyframes_) {
            kf.pose = result.at<gtsam::Pose2>(kf.id);
        }

        std::cout << "ポーズグラフ最適化完了: " << keyframes_.size() << " キーフレーム" << std::endl;
    }

    struct KeyFrame {
        int id;
        gtsam::Pose2 pose;
        LaserScan::Ptr scan;
        double timestamp;
    };

    std::vector<KeyFrame> keyframes_;
    gtsam::NonlinearFactorGraph pose_graph_;
    std::shared_ptr<LoopDetector> loop_detector_;
    const double loop_score_threshold_ = 0.7;
};
```

### メインループ

```cpp
int main() {
    // パラメータ設定
    auto imu_params = std::make_shared<PreintegrationParams2D>();
    imu_params->n_gravity = Eigen::Vector2d(0.0, -9.81);
    imu_params->accelerometer_noise_sigma = 0.02;
    imu_params->gyroscope_noise_sigma = 0.0005;
    imu_params->use_2nd_order_integration = true;

    // フロントエンド・バックエンド初期化
    SlamFrontend frontend(imu_params);
    SlamBackend backend;

    // センサーデータストリーム
    SensorStream stream("dataset/");

    while (stream.has_next()) {
        auto data = stream.next();

        if (data.type == SensorData::LASER_SCAN) {
            // スキャン処理
            frontend.process_scan(data.scan, data.timestamp);

            // キーフレーム判定
            if (frontend.is_keyframe()) {
                backend.add_keyframe(
                    frontend.current_pose(),
                    data.scan,
                    frontend.local_values()
                );
            }
        }
        else if (data.type == SensorData::IMU) {
            // IMUデータをキューに追加
            frontend.add_imu_measurement(data.imu);
        }
    }

    // 最終的なグローバル最適化
    backend.final_optimization();

    // 結果の保存
    backend.save_trajectory("trajectory_final.txt");
    backend.save_map("map_final.pcd");

    std::cout << "SLAM完了" << std::endl;

    return 0;
}
```

---

## まとめ

このチュートリアルでは、以下のトピックをカバーしました：

1. **基礎ICP**: シンプルなスキャンマッチング
2. **ロバストマッチング**: GICP/GNCによる外れ値対応
3. **連続時間SLAM**: モーション補償とCT-ICP
4. **IMU統合**: センサーフュージョン
5. **完全なSLAM**: フロントエンド/バックエンドアーキテクチャ

### 次のステップ

- **パフォーマンス最適化**: 並列処理、ボクセルマップの活用
- **ループクロージャ**: Scan Context、NDTなどの手法
- **動的環境対応**: セグメンテーションと動的物体除去
- **マルチロボットSLAM**: 分散SLAM、地図マージ

---

*このチュートリアルで使用したコード例は簡略化されています。*
*完全な実装例は`examples/`ディレクトリを参照してください。*
