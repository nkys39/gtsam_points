# gtsam_points 2D SLAM - APIリファレンス

## 目次

1. [ファクター](#ファクター)
2. [ポイントクラウド](#ポイントクラウド)
3. [特徴量推定](#特徴量推定)
4. [最近傍探索](#最近傍探索)
5. [レジストレーション](#レジストレーション)
6. [セグメンテーション](#セグメンテーション)
7. [オプティマイザー](#オプティマイザー)
8. [ユーティリティ](#ユーティリティ)

---

## ファクター

### IntegratedICPFactor2D

**ヘッダー**: `gtsam_points/d2/factors/integrated_icp_factor_2d.hpp`

**説明**: Point-to-Line ICPファクター。2D環境での基本的なスキャンマッチング。

#### コンストラクタ

```cpp
IntegratedICPFactor2D(
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree = nullptr
);
```

**パラメータ**:
- `source_key`: ソースポーズのGTSAMキー
- `target`: ターゲットポイントクラウド（法線が必要）
- `source`: ソースポイントクラウド
- `target_tree`: 最近傍探索構造（nullptrの場合はKdTree2Dを自動生成）

#### メソッド

```cpp
void set_max_correspondence_distance(double dist);
```
対応点探索の最大距離を設定。

```cpp
void set_num_threads(int n);
```
並列処理のスレッド数を設定。

```cpp
double error(const gtsam::Values& values) const override;
```
現在の推定値での誤差を計算。

```cpp
std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values& values) const override;
```
線形化してガウシアンファクターを生成。

#### 使用例

```cpp
auto factor = gtsam::make_shared<IntegratedICPFactor2D>(
    0, target, source
);
factor->set_max_correspondence_distance(1.0);
factor->set_num_threads(4);
graph.add(factor);
```

---

### IntegratedGICPFactor2D

**ヘッダー**: `gtsam_points/d2/factors/integrated_gicp_factor_2d.hpp`

**説明**: Generalized ICP with Mahalanobis距離。共分散を考慮したロバストマッチング。

#### コンストラクタ

```cpp
IntegratedGICPFactor2D(
    gtsam::Key source_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree = nullptr
);
```

**要件**:
- ターゲットとソースの両方に`normals`と`covs`が必要
- `estimate_normals_2d()`と`estimate_covariances_2d()`で事前計算

#### 誤差モデル

```
e = (T*p_s - p_t)^T * M * (T*p_s - p_t)
M = (C_t + R*C_s*R^T)^{-1}
```

ここで:
- `C_t`, `C_s`: ターゲットとソースの共分散行列
- `R`: 回転行列 (2x2)
- `M`: Mahalanobis行列

#### 使用例

```cpp
// 事前処理
estimate_normals_2d(*target, 10);
estimate_covariances_2d(*target);
estimate_normals_2d(*source, 10);
estimate_covariances_2d(*source);

// ファクター追加
auto factor = gtsam::make_shared<IntegratedGICPFactor2D>(
    0, target, source
);
graph.add(factor);
```

---

### IntegratedVGICPFactor2D

**ヘッダー**: `gtsam_points/d2/factors/integrated_vgicp_factor_2d.hpp`

**説明**: Voxelized GICP。大規模地図に対してメモリ効率的。

#### コンストラクタ

```cpp
IntegratedVGICPFactor2D(
    gtsam::Key source_key,
    const std::shared_ptr<const GaussianGridMap2D>& target,
    const std::shared_ptr<const SourceFrame>& source
);
```

**パラメータ**:
- `target`: GaussianGridMap2D（ボクセル化されたターゲット）
- `source`: ソースポイントクラウド（共分散が必要）

#### GaussianGridMap2Dの作成

```cpp
double resolution = 0.5;  // 0.5mボクセル
auto voxel_map = std::make_shared<GaussianGridMap2D>(resolution);

// ポイントクラウドをボクセルマップに変換
voxel_map->insert(*target_cloud);
```

#### 使用例

```cpp
auto voxel_map = std::make_shared<GaussianGridMap2D>(0.5);
voxel_map->insert(*target_cloud);

auto factor = gtsam::make_shared<IntegratedVGICPFactor2D>(
    0, voxel_map, source
);
graph.add(factor);
```

---

### IntegratedCT_ICPFactor2D

**ヘッダー**: `gtsam_points/d2/factors/integrated_ct_icp_factor_2d.hpp`

**説明**: 連続時間ICP。タイムスタンプ付きスキャンのモーション補償。

#### コンストラクタ

```cpp
IntegratedCT_ICPFactor2D(
    gtsam::Key source_t0_key,
    gtsam::Key source_t1_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree = nullptr
);
```

**パラメータ**:
- `source_t0_key`: スキャン開始時のポーズキー
- `source_t1_key`: スキャン終了時のポーズキー
- `source`: タイムスタンプ付きポイントクラウド（`times`属性が必要）

#### タイムスタンプの要件

- 各点に`time`属性が必要
- タイムスタンプは0から始まる相対時間
- 自動的に[0, 1]に正規化される

#### 補間方式

SE(2)上での指数写像補間:
```
T(t) = T_0 * exp(t * log(T_0^{-1} * T_1))
```

#### 使用例

```cpp
auto scan = std::make_shared<LaserScan>();
// ... タイムスタンプ付きデータを読み込み

auto factor = gtsam::make_shared<IntegratedCT_ICPFactor2D>(
    0,  // スキャン開始
    1,  // スキャン終了
    target,
    scan
);
graph.add(factor);

initial_values.insert(0, gtsam::Pose2(0, 0, 0));
initial_values.insert(1, gtsam::Pose2(0.1, 0, 0.01));
```

---

### IntegratedCT_GICPFactor2D

**ヘッダー**: `gtsam_points/d2/factors/integrated_ct_gicp_factor_2d.hpp`

**説明**: 連続時間GICP。CT-ICPとGICPの組み合わせ。

#### コンストラクタ

```cpp
IntegratedCT_GICPFactor2D(
    gtsam::Key source_t0_key,
    gtsam::Key source_t1_key,
    const std::shared_ptr<const TargetFrame>& target,
    const std::shared_ptr<const SourceFrame>& source,
    const std::shared_ptr<const NearestNeighborSearch2D>& target_tree = nullptr
);
```

**要件**:
- ターゲットとソース: `normals`, `covs`, `times`

#### 使用例

```cpp
// 事前処理
estimate_normals_2d(*target, 10);
estimate_covariances_2d(*target);
estimate_normals_2d(*scan, 10);
estimate_covariances_2d(*scan);

auto factor = gtsam::make_shared<IntegratedCT_GICPFactor2D>(
    0, 1, target, scan
);
graph.add(factor);
```

---

### ReintegratedImuFactor2D

**ヘッダー**: `gtsam_points/d2/factors/reintegrated_imu_factor_2d.hpp`

**説明**: 2D IMU事前積分ファクター。平面運動に特化。

#### IMUパラメータ

```cpp
struct PreintegrationParams2D {
    Eigen::Vector2d n_gravity;           // 重力 [gx, gy]
    double accelerometer_noise_sigma;    // 加速度計ノイズ [m/s²]
    double gyroscope_noise_sigma;        // ジャイロノイズ [rad/s]
    double accelerometer_bias_sigma;     // 加速度計バイアスランダムウォーク
    double gyroscope_bias_sigma;         // ジャイロバイアスランダムウォーク
    bool use_2nd_order_integration;      // 2次積分を使用
};
```

#### IMUバイアス

```cpp
class ImuBias2D {
    ImuBias2D(const Eigen::Vector2d& acc_bias, double gyro_bias);

    const Eigen::Vector2d& accelerometer() const;
    double gyroscope() const;
    Eigen::Vector3d vector() const;  // [bias_ax, bias_ay, bias_ωz]

    static ImuBias2D Zero();
};
```

#### IMU計測の積分

```cpp
auto params = std::make_shared<PreintegrationParams2D>();
params->n_gravity = Eigen::Vector2d(0.0, -9.81);
params->accelerometer_noise_sigma = 0.01;
params->gyroscope_noise_sigma = 0.0001;
params->use_2nd_order_integration = true;

ImuBias2D bias(Eigen::Vector2d::Zero(), 0.0);
ReintegratedImuMeasurements2D pim(params, bias);

for (const auto& meas : measurements) {
    Eigen::Vector2d acc(meas.ax, meas.ay);
    double omega_z = meas.omega_z;
    double dt = meas.dt;
    pim.integrateMeasurement(acc, omega_z, dt);
}
```

#### ファクター構築

```cpp
ReintegratedImuFactor2D(
    gtsam::Key pose_i,      // 初期ポーズ (Pose2)
    gtsam::Key vel_i,       // 初期速度 (Vector2d)
    gtsam::Key pose_j,      // 終了ポーズ (Pose2)
    gtsam::Key vel_j,       // 終了速度 (Vector2d)
    gtsam::Key bias,        // IMUバイアス (Vector3d)
    const ReintegratedImuMeasurements2D& imu_meas
);
```

#### 完全な使用例

```cpp
// パラメータ設定
auto params = std::make_shared<PreintegrationParams2D>();
params->n_gravity = Eigen::Vector2d(0.0, -9.81);
params->accelerometer_noise_sigma = 0.01;
params->gyroscope_noise_sigma = 0.0001;

// IMU計測を積分
ImuBias2D bias = ImuBias2D::Zero();
ReintegratedImuMeasurements2D imu_meas(params, bias);

for (double t = 0.0; t < 1.0; t += 0.01) {
    Eigen::Vector2d acc = get_imu_acceleration(t);
    double omega_z = get_imu_angular_velocity(t);
    imu_meas.integrateMeasurement(acc, omega_z, 0.01);
}

// ファクター追加
auto imu_factor = gtsam::make_shared<ReintegratedImuFactor2D>(
    0,  // pose_i
    1,  // vel_i
    2,  // pose_j
    3,  // vel_j
    4,  // bias
    imu_meas
);
graph.add(imu_factor);

// 初期値
initial_values.insert(0, gtsam::Pose2(0, 0, 0));
initial_values.insert(1, Eigen::Vector2d(0, 0));
initial_values.insert(2, gtsam::Pose2(1, 0, 0));
initial_values.insert(3, Eigen::Vector2d(1, 0));
initial_values.insert(4, Eigen::Vector3d::Zero());  // [bias_ax, bias_ay, bias_ωz]
```

---

## ポイントクラウド

### PointCloud2D

**ヘッダー**: `gtsam_points/d2/types/point_cloud_2d.hpp`

**説明**: 2Dポイントクラウドの基底クラス。

#### フレームトレイト

```cpp
namespace frame {
    size_t size(const PointCloud2D& points);
    const Eigen::Vector3d& point(const PointCloud2D& points, size_t i);
    const Eigen::Vector3d& normal(const PointCloud2D& points, size_t i);
    const Eigen::Matrix3d& cov(const PointCloud2D& points, size_t i);
    double time(const PointCloud2D& points, size_t i);
    double intensity(const PointCloud2D& points, size_t i);

    bool has_points(const PointCloud2D& points);
    bool has_normals(const PointCloud2D& points);
    bool has_covs(const PointCloud2D& points);
    bool has_times(const PointCloud2D& points);
    bool has_intensities(const PointCloud2D& points);
}
```

### PointCloud2DCPU

**ヘッダー**: `gtsam_points/d2/types/point_cloud_2d_cpu.hpp`

**説明**: CPU上の具体的な実装。

#### 構築

```cpp
auto cloud = std::make_shared<PointCloud2DCPU>();

// 点の追加
cloud->points.emplace_back(x, y, 1.0);  // 同次座標

// 法線の追加
cloud->normals.emplace_back(nx, ny, 0.0);

// 共分散の追加
Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
cov.block<2, 2>(0, 0) = /* 2x2共分散行列 */;
cloud->covs.emplace_back(cov);
```

### LaserScan

**ヘッダー**: `gtsam_points/d2/types/laser_scan.hpp`

**説明**: 2D LiDARスキャンデータ。

#### 構築

```cpp
auto scan = std::make_shared<LaserScan>();
scan->angle_min = -M_PI;
scan->angle_max = M_PI;
scan->angle_increment = 0.01;
scan->time_increment = 0.0001;
scan->scan_time = 0.1;
scan->range_min = 0.1;
scan->range_max = 30.0;

// 距離データ
scan->ranges = {1.0, 1.1, 1.2, ...};

// タイムスタンプ（自動生成可能）
scan->times = {0.0, 0.0001, 0.0002, ...};
```

---

## 特徴量推定

### 法線推定

**ヘッダー**: `gtsam_points/d2/features/normal_estimation_2d.hpp`

#### k近傍法

```cpp
void estimate_normals_2d(
    PointCloud2D& cloud,
    int k_neighbors = 10,
    int num_threads = 1
);
```

**パラメータ**:
- `cloud`: 入出力ポイントクラウド（`normals`が設定される）
- `k_neighbors`: 使用する近傍点数
- `num_threads`: 並列スレッド数

#### 半径探索法

```cpp
void estimate_normals_radius_2d(
    PointCloud2D& cloud,
    double search_radius = 0.5,
    int num_threads = 1
);
```

#### アルゴリズム

1. 各点のk近傍を探索
2. 近傍点に対してPCA実行
3. 最小固有値の固有ベクトル → 法線
4. 視点方向への向き調整

### 共分散推定

**ヘッダー**: `gtsam_points/d2/features/covariance_estimation_2d.hpp`

```cpp
void estimate_covariances_2d(
    PointCloud2D& cloud,
    int k_neighbors = 10,
    int num_threads = 1
);
```

**出力**: 各点の2x2共分散行列（3x3の左上ブロック）

```
C = [ σ_x²   σ_xy ]
    [ σ_xy   σ_y² ]
```

---

## 最近傍探索

### KdTree2D

**ヘッダー**: `gtsam_points/d2/ann/kdtree_2d.hpp`

```cpp
template<typename PointCloud>
class KdTree2D : public NearestNeighborSearch2D {
public:
    KdTree2D(const std::shared_ptr<const PointCloud>& cloud);

    size_t knn_search(
        const double* query,      // [x, y, 1]
        size_t k,
        size_t* k_indices,
        double* k_sq_dists,
        double max_sq_dist = std::numeric_limits<double>::max()
    ) const override;

    size_t radius_search(
        const double* query,
        double radius,
        std::vector<size_t>& indices,
        std::vector<double>& sq_dists
    ) const override;
};
```

### GaussianGridMap2D

**ヘッダー**: `gtsam_points/d2/types/gaussian_gridmap_2d.hpp`

```cpp
class GaussianGridMap2D : public IncrementalGridMap2D<GaussianGridCell2D> {
public:
    GaussianGridMap2D(double resolution);

    void insert(const PointCloud2D& cloud) override;

    // グリッド座標変換
    Eigen::Vector2i grid_coord(const Eigen::Vector3d& x) const;

    // セル取得
    const GaussianGridCell2D& lookup_cell(int cell_id) const;

    // 保存/読み込み
    void save_compact(const std::string& path) const;
    static GaussianGridMap2D::Ptr load(const std::string& path);
};
```

**GaussianGridCell2D**:
```cpp
struct GaussianGridCell2D {
    Eigen::Vector3d mean;       // 平均位置 [x, y, 1]
    Eigen::Matrix3d cov;        // 共分散 (2x2有効)
    int num_points;             // 点数
};
```

---

## レジストレーション

### Alignment2D

**ヘッダー**: `gtsam_points/d2/registration/alignment_2d.hpp`

#### 2点ペアからの位置合わせ

```cpp
Eigen::Isometry2d align_points_se2(
    const Eigen::Vector3d& target1,
    const Eigen::Vector3d& target2,
    const Eigen::Vector3d& source1,
    const Eigen::Vector3d& source2
);
```

#### 重み付き多点位置合わせ

```cpp
Eigen::Isometry2d align_points_se2(
    const Eigen::Vector3d* target_points,
    const Eigen::Vector3d* source_points,
    const double* weights,
    size_t num_points
);
```

### RANSAC2D

**ヘッダー**: `gtsam_points/d2/registration/ransac_2d.hpp`

```cpp
struct RANSAC2DParams {
    int ransac_iterations = 1024;
    double inlier_threshold = 0.1;
    int min_inliers = 10;
    double polygonal_error_thresh = 0.5;
    int num_threads = 1;
    std::uint64_t seed = 0;
};

RegistrationResult2D estimate_pose_ransac_2d(
    const PointCloud& target,
    const PointCloud& source,
    const NearestNeighborSearch2D& target_tree,
    const RANSAC2DParams& params = RANSAC2DParams()
);
```

---

## セグメンテーション

### RegionGrowing2D

**ヘッダー**: `gtsam_points/d2/segmentation/region_growing_2d.hpp`

```cpp
struct RegionGrowingParams2D {
    double distance_threshold = 0.5;
    double angle_threshold = 10.0 * M_PI / 180.0;
    double dilation_radius = 0.5;
    int max_cluster_size = 1000000;
    int max_steps = 1000000;
    int num_threads = 1;
};

std::vector<size_t> region_growing_2d(
    const PointCloud& points,
    const NearestNeighborSearch2D& search,
    size_t seed_index,
    const RegionGrowingParams2D& params = RegionGrowingParams2D()
);
```

### MinCut2D

**ヘッダー**: `gtsam_points/d2/segmentation/min_cut_2d.hpp`

```cpp
struct MinCutParams2D {
    double distance_sigma = 0.25;
    double angle_sigma = 10.0 * M_PI / 180.0;
    double foreground_mask_radius = 0.2;
    double background_mask_radius = 3.0;
    double foreground_weight = 0.2;
    double background_weight = 0.2;
    int k_neighbors = 20;
    int num_threads = 1;
};

MinCutResult2D min_cut_2d(
    const PointCloud& points,
    const NearestNeighborSearch2D& search,
    size_t source_index,
    const MinCutParams2D& params = MinCutParams2D()
);
```

---

## オプティマイザー

### IncrementalFixedLagSmootherExt

**ヘッダー**: `gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp`

**重要**: このクラスは**2D/3D汎用**です。Pose2でもPose3でもそのまま使用できます。2D専用の実装は不要です。

**説明**: ISAM2ベースのFixed-Lag Smoother。一定時間ウィンドウ内の状態のみを保持し、古い状態を周辺化することでメモリと計算量を一定に保ちながらリアルタイム最適化を実現。

#### 主な特徴

- **2D/3D汎用**: Pose2、Pose3、Vector2、Vector3など任意の状態変数に対応
- **一定メモリ使用量**: 時間ウィンドウ外の状態は自動的に周辺化
- **ISAM2ベース**: インクリメンタルな更新で高速動作
- **GTSAMネイティブ**: 標準的なFixedLagSmootherインターフェース

#### コンストラクタ

```cpp
IncrementalFixedLagSmootherExt(
    double smoother_lag = 5.0,
    const gtsam::ISAM2Params& parameters = gtsam::ISAM2Params()
);
```

**パラメータ**:
- `smoother_lag`: 保持する時間ウィンドウの長さ（秒）。この時間より古い状態は周辺化される
- `parameters`: ISAM2パラメータ

#### 基本メソッド

```cpp
// 更新（ファクターと値を追加）
gtsam::ISAM2Result update(
    const gtsam::NonlinearFactorGraph& new_factors,
    const gtsam::Values& new_values,
    const KeyTimestampMap& timestamps
);

// 推定値を取得
template<typename T>
T calculateEstimate(gtsam::Key key) const;

// 共分散を取得
gtsam::Matrix marginalCovariance(gtsam::Key key) const;

// 全ての推定値を取得
gtsam::Values calculateEstimate() const;
```

#### 2D SLAMでの使用例

```cpp
#include <gtsam_points/optimizers/incremental_fixed_lag_smoother_ext.hpp>
#include <gtsam/geometry/Pose2.h>

using namespace gtsam_points;

// 5秒のラグで初期化（2D用パラメータ）
gtsam::ISAM2Params params;
params.relinearizeThreshold = 0.01;  // 2Dでは小さめの閾値
params.relinearizeSkip = 1;
params.enableRelinearization = true;
params.evaluateNonlinearError = false;
params.factorization = gtsam::ISAM2Params::CHOLESKY;

IncrementalFixedLagSmootherExt smoother(5.0, params);

// 新しいファクターと値を追加
gtsam::NonlinearFactorGraph new_factors;
gtsam::Values new_values;
gtsam::FixedLagSmoother::KeyTimestampMap timestamps;

// Pose2を追加
gtsam::Key pose_key = gtsam::Symbol('x', frame_id);
new_values.insert(pose_key, gtsam::Pose2(x, y, theta));
timestamps[pose_key] = current_time;

// Vector2 (速度) を追加
gtsam::Key vel_key = gtsam::Symbol('v', frame_id);
new_values.insert(vel_key, Eigen::Vector2d(vx, vy));
timestamps[vel_key] = current_time;

// ファクターを追加（ICP、IMUなど）
new_factors.add(gtsam::make_shared<IntegratedICPFactor2D>(...));
new_factors.add(gtsam::make_shared<ReintegratedImuFactor2D>(...));

// 更新実行
auto result = smoother.update(new_factors, new_values, timestamps);

// 推定値を取得（テンプレートで型指定）
gtsam::Pose2 estimated_pose = smoother.calculateEstimate<gtsam::Pose2>(pose_key);
Eigen::Vector2d estimated_vel = smoother.calculateEstimate<Eigen::Vector2d>(vel_key);

// 共分散を取得（Pose2の場合は3x3）
gtsam::Matrix3 covariance = smoother.marginalCovariance(pose_key);

// 全軌跡を取得（Symbol 'x'のみ）
gtsam::Values all_values = smoother.calculateEstimate();
for (const auto& key_value : all_values) {
    gtsam::Key key = key_value.key;
    if (gtsam::Symbol(key).chr() == 'x') {
        gtsam::Pose2 pose = all_values.at<gtsam::Pose2>(key);
        std::cout << "Pose " << gtsam::Symbol(key).index()
                  << ": " << pose.translation().transpose()
                  << " theta=" << pose.theta() << std::endl;
    }
}
```

#### 推奨パラメータ（2D SLAM用）

```cpp
gtsam::ISAM2Params params;
params.relinearizeThreshold = 0.01;    // 2Dでは小さめ（3Dは0.1推奨）
params.relinearizeSkip = 1;
params.enableRelinearization = true;
params.evaluateNonlinearError = false; // パフォーマンス重視
params.factorization = gtsam::ISAM2Params::CHOLESKY;
params.findUnusedFactorSlots = true;
```

#### 用途

- **リアルタイムロボットナビゲーション**: 移動ロボットのオンライン位置推定（2D/3D）
- **オンライン SLAM**: スキャンマッチング + IMU統合
- **長時間動作**: メモリ効率的で無制限に動作可能

#### 3D SLAMでの使用例

```cpp
// 同じクラスをPose3でも使用可能
gtsam::Key pose_key_3d = gtsam::Symbol('x', frame_id);
new_values.insert(pose_key_3d, gtsam::Pose3(...));  // Pose3
timestamps[pose_key_3d] = current_time;

// 後で取得
gtsam::Pose3 pose_3d = smoother.calculateEstimate<gtsam::Pose3>(pose_key_3d);
```

---

## ユーティリティ

### BSpline2D

**ヘッダー**: `gtsam_points/d2/util/bspline_2d.hpp`

#### 基本的なB-スプライン補間

```cpp
// 独立補間 (rotation + translation)
gtsam::Pose2_ bspline_2d(
    const gtsam::Pose2_& pose0,
    const gtsam::Pose2_& pose1,
    const gtsam::Pose2_& pose2,
    const gtsam::Pose2_& pose3,
    const gtsam::Double_& t  // [0, 1]
);

// SE(2)上の関節補間
gtsam::Pose2_ bspline_se2(
    const gtsam::Pose2_& pose0,
    const gtsam::Pose2_& pose1,
    const gtsam::Pose2_& pose2,
    const gtsam::Pose2_& pose3,
    const gtsam::Double_& t
);
```

#### 速度・加速度計算

```cpp
// 角速度 (rad/s)
gtsam::Double_ bspline_angular_vel_2d(
    const gtsam::Rot2_& rot0,
    const gtsam::Rot2_& rot1,
    const gtsam::Rot2_& rot2,
    const gtsam::Rot2_& rot3,
    const gtsam::Double_& t,
    double knot_interval
);

// 線速度 (m/s)
gtsam::Vector2_ bspline_linear_vel_2d(
    const gtsam::Vector2_& trans0,
    const gtsam::Vector2_& trans1,
    const gtsam::Vector2_& trans2,
    const gtsam::Vector2_& trans3,
    const gtsam::Double_& t,
    double knot_interval
);

// 線形加速度 (m/s²)
gtsam::Vector2_ bspline_linear_acc_2d(
    const gtsam::Vector2_& trans0,
    const gtsam::Vector2_& trans1,
    const gtsam::Vector2_& trans2,
    const gtsam::Vector2_& trans3,
    const gtsam::Double_& t,
    double knot_interval
);
```

#### IMU計測予測

```cpp
// [ax_local, ay_local, ωz]
gtsam::Vector3_ bspline_imu_2d(
    const gtsam::Pose2_ pose0,
    const gtsam::Pose2_ pose1,
    const gtsam::Pose2_ pose2,
    const gtsam::Pose2_ pose3,
    const gtsam::Double_& t,
    double knot_interval,
    const gtsam::Vector2& g  // 重力
);
```

---

## データ型サマリー

### 基本型

| 型 | 説明 | 次元 |
|----|------|------|
| `gtsam::Pose2` | SE(2)ポーズ | 3 DOF (x, y, θ) |
| `gtsam::Rot2` | SO(2)回転 | 1 DOF (θ) |
| `Eigen::Vector2d` | 2D速度 | 2 |
| `Eigen::Vector3d` | 同次座標/法線 | 3 (点: [x,y,1], 法線: [nx,ny,0]) |
| `Eigen::Matrix3d` | 共分散行列 | 3x3 (有効: 2x2) |

### 状態変数

| 変数 | 型 | GTSAMキー型 |
|------|-----|------------|
| ポーズ | `gtsam::Pose2` | `gtsam::Key` |
| 速度 | `Eigen::Vector2d` | `gtsam::Key` |
| IMUバイアス | `Eigen::Vector3d` | `gtsam::Key` |

---

*このAPIリファレンスは実装に基づいて作成されました。*
*最新情報はヘッダーファイルを参照してください。*
