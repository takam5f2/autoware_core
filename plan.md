# autoware_ndt_scan_matcher: ROS Node 層とコアロジック層の分離

## Context

`NDTScanMatcher`（`rclcpp::Node`）は 1154 行の単一クラスで、パラメータ取得・pub/sub/service/timer・TF・診断出力と、スキャンマッチングの判定ロジックが完全に交錯している。中でも `callback_sensor_points_main` は 390 行の一枚岩で、検証・NDT 実行・スコア判定・共分散推定・14 トピックの publish・20 箇所以上の診断キー追加が 1 関数に同居している。この状態ではロジック単体をテストする手段がなく、実際にテストは全て ROS ノードを起動する統合テストになっている。

先行して仕様化テスト（`test/test_ndt_scan_matcher_characteristics.cpp`、44 ケース / 2348 行）を追加済みで、`/diagnostics` のキー集合と順序、publish されるトピックの有無と内容、パーティクルごとの cloud 数、地図更新の差分ロード挙動が外部から pin されている。**この安全網が有効なうちに** ロジックを ROS 非依存のコア層へ移す。

到達点は「コア層に残る rclcpp 要素はメッセージ型の参照だけ」。コールバック・pub/sub・service・timer・TF・clock・logger はコア層に一切入れない。外部から観測できる振る舞いは 1 ビットも変えない。

参照実装: `takam5f2/refactor/ndt-map-update-module-cleanup`。`MapUpdateModule` を ROS-free 化した先行事例で、本計画はそこで確立したイディオムをパッケージ全体に横展開する。

> ローカルに `refactor-ndt-scan-matcher-phase-a`（2026-07-31、未 push）という同種の先行試行があるが、仕様化テストも map_update クリーンアップも存在しない時点の分岐であり、**本計画では参照しない**。

---

## 確立済みのイディオム（参照ブランチ由来・全モジュールで踏襲）

0. **注入する前に、呼び出し側が回せる形にできないか確かめる**（ステップ 5 で得た原則）。モジュールが「途中で外を呼ぶ」必要があるように見えても、多くは「途中経過を返して呼び出し側が続きを頼む」に置き換えられる。制御が反転せず、`std::function` も増えず、publish がそれの属するループの見えるところに残る。`PoseInitializationSearch` の `next()` / `finish()` がその形。逆に `PcdLoaderFunction` は当てはまらない — 呼び出しがモジュールの計算の途中で必要で、結果によって続きが変わるため。

0a. **Node が 3 つのコア部品を並べて持つ**（2026-08-31 決定）。`MapUpdateModule` / `ScanMatchingModule` / `PoseInitializationSearch` は兄弟で、`Guarded<NdtPtrType> ndt_ptr_` と直近スキャンは Node が持ち続ける。

> 検討して見送った案: コアに `NdtScanMatcher` クラスを置き、3 つを has-a で所有させる。`MapUpdateModule` が `Guarded<NdtPtrType> &` で Node の状態を参照している歪みは解消し、`out_of_map_range` も内部呼び出しになるが、align 探索が「ロックを保持したまま Node に publish させる」ため、ロックを外へ出す型かスコープ付きアクセサのどちらかが必要になる。現状の `ndt_ptr_.with([&]{ ... })` はその問題を持たない。構造の純度より、いま動いている形を保つことを優先する。
>
> **2026-09-01 撤回。** 見送りの理由はただ一つ「align 探索がロックの内側で Node に publish させる」ことで、その publish 自体が仕様として不要だと判明した。進捗 publish をやめれば理由が消える。最終節「コア root `NdtScanMatcher` への移行」を参照。

0b. **module にするかは、呼び出しをまたぐ状態を持つかで決める。** `MapUpdateModule`（ロード済み地図）と `PoseInterpolationBuffer`（姿勢バッファ）は長命な module。`PoseInitializationSearch` は状態を持たないので要求ごとに構築するただのオブジェクト。「3 モジュール構成」という枠に合わせて状態のない処理を module にすると、本体をその中に押し込むことになりクラス内クラスが生まれる。

1. **ROS I/O は `std::function` で注入する。** `MapUpdateModule` は `pcd_loader_service` の呼び出しを `PcdLoaderFunction` として受け取り、モジュール自身は `rclcpp::Client` を知らない。ただし 0 を先に試すこと。
2. **パラメータ構造体はモジュールが所有する。** `HyperParameters`（`rclcpp::Node *` を取る唯一のアダプタ）が各モジュールの `Params` を埋める。`MapUpdateModule::Params` が既にこの形。
3. **診断は ROS-free な `DiagnosticsReport` として返し、Node が `DiagnosticsInterface` へ転送する。** `level` / `message` / `key_values`（`std::variant<bool,int64_t,double,std::string>`）を持ち、`update_level_and_message` が `DiagnosticsInterface` と同じ蓄積規則を再現する。
4. **状態は戻り値で渡す。** `MapUpdateModule::update()` は新しい NDT を返し、呼び出し側が入れ替える。モジュールは Node が持つ状態に手を伸ばさない。
5. **共有状態は `Guarded<T>`。** ロック順序をコメントで明示する。

---

## 最終的な分離結果

### ファイル構成

```
include/autoware/ndt_scan_matcher/
  guarded.hpp                     既存・ROS-free。変更なし
  diagnostics_report.hpp          [新] DiagnosticLevel / DiagnosticKeyValue / LogRequest
                                      / DiagnosticsReport。MapUpdateModule から昇格
  pose_interpolation_buffer.hpp   [新] SmartPoseBuffer の ROS-free 版（vendoring）
  map_update_module.hpp           参照ブランチ由来・ROS-free
  scan_matching_module.hpp        [新] ホットパス
  pose_initialization_module.hpp  [新] ndt_align の TPE 推定
  particle.hpp                    既存。msg 型のみに（rclcpp::Duration を排除）
  hyper_parameters.hpp            rclcpp::Node* → 各モジュールの Params を埋めるだけのアダプタ
  ndt_scan_matcher_node.hpp       [改名] ← ndt_scan_matcher_core.hpp。rclcpp::Node
src/
  pose_interpolation_buffer.cpp   [新]
  scan_matching_module.cpp        [新]
  pose_initialization_module.cpp  [新]
  map_update_module.cpp           参照ブランチ由来
  ndt_scan_matcher_helper.{hpp,cpp}  既存の純粋関数。コア lib に所属
  particle.cpp                    既存
  ndt_scan_matcher_node.cpp       [改名] ← ndt_scan_matcher_core.cpp
```

現在の `ndt_scan_matcher_core.hpp/cpp` は名前に反して ROS ノードそのものなので、`_node` へ改名する。クラス名 `NDTScanMatcher` と PLUGIN 文字列 `"autoware::ndt_scan_matcher::NDTScanMatcher"` は据え置く（agnocast 登録・launch・テストフィクスチャが依存）。

### CMake によるライブラリ分割

```cmake
# コア層。rclcpp には依存しない。
add_library(${PROJECT_NAME}_core SHARED
  src/map_update_module.cpp
  src/scan_matching_module.cpp
  src/pose_initialization_module.cpp
  src/pose_interpolation_buffer.cpp
  src/ndt_scan_matcher_helper.cpp
  src/particle.cpp
)
ament_target_dependencies(${PROJECT_NAME}_core
  autoware_map_msgs builtin_interfaces geometry_msgs sensor_msgs std_msgs
  visualization_msgs pcl_conversions autoware_utils_geometry autoware_utils_pcl
  autoware_utils_visualization Eigen3)   # rclcpp / tf2_ros は含めない
target_link_libraries(${PROJECT_NAME}_core ${PCL_LIBRARIES} multigrid_ndt_omp)

# ROS 層。ライブラリ名は ${PROJECT_NAME} のまま（PLUGIN 名・既存テストのリンク先を壊さない）。
ament_auto_add_library(${PROJECT_NAME} SHARED src/ndt_scan_matcher_node.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ndt_scan_matcher::NDTScanMatcher"   # 不変
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor)
```

SIMD フラグ（`-msse ... -msse4.2`）はファイル冒頭の `add_compile_options` でディレクトリ全体に効くので、分割後も両ライブラリに同一に適用される。ここを崩すと Eigen のアラインメント違反で初期化時にクラッシュするため、ターゲット別フラグには**しない**。

**機械的な担保**: `autoware_localization_util` は `pose_to_matrix4f` などのために必要だが rclcpp を推移的に引き込むため、リンクだけでは「rclcpp を使っていない」を保証できない。テストターゲット `test_core_is_ros_free` を追加し、コア公開ヘッダを全て include するだけの TU を rclcpp の include ディレクトリ**なし**でコンパイルする。ヘッダに `<rclcpp/...>` が混入した瞬間にビルドが落ちる。加えて pre-commit で `src/{map_update_module,scan_matching_module,pose_initialization_module,pose_interpolation_buffer,ndt_scan_matcher_helper,particle}.cpp` に対する `rclcpp::` / `tf2_ros` の grep を掛ける。

---

## コアロジックのインタフェース（本計画の中心）

### 共通: `diagnostics_report.hpp`

`MapUpdateModule` 内で定義済みのものを独立ヘッダへ昇格させ、全モジュールで共有する。加えてログ要求を持たせる。

```cpp
namespace autoware::ndt_scan_matcher
{

// diagnostic_msgs::msg::DiagnosticStatus のレベルを写したもの。コアが ROS の診断に依存しないため。
enum class DiagnosticLevel : int8_t { OK = 0, WARN = 1, ERROR = 2, STALE = 3 };

struct DiagnosticKeyValue
{
  std::string key;
  // 型を保つのは、Node 側が DiagnosticsInterface と同じ整形（bool を "True"/"False" 等）を
  // 再現できるようにするため。
  std::variant<bool, int64_t, double, std::string> value;
};

// コアが「ここでログを出していた」ことを Node に伝えるための記録。コアはログを出さない。
//
// 重大度と throttle 幅は site から一意に決まる（Node 側の case が使うマクロで決まる）ので、
// level は持たせない。持たせると真実が 2 つになって必ずズレる。
enum class LogSite : uint8_t {
  ScanTransformFailed,         // ERROR, throttle 1000ms
  ScanOutOfMapRange,           // WARN,  throttle 1000ms
  ScanScoreBelowThreshold,     // WARN,  throttle 1000ms
  AlignTransformFailed,        // ERROR, throttle 1000ms
  AlignNoInputTarget,          // WARN,  throttle 1000ms
  AlignNoInputSource,          // WARN,  throttle 1000ms
  AlignUnstableScore,          // WARN,  throttle なし
  PoseBufferTooFewSamples,     // INFO,  throttle なし（vendoring した buffer 由来）
  PoseBufferStampMismatch,     // INFO,  throttle なし
  PoseBufferTimeoutViolation,  // WARN,  throttle なし
  PoseBufferDistanceViolation  // WARN,  throttle なし
};
struct LogRequest
{
  LogSite site;
  std::string message;
};

struct DiagnosticsReport
{
  DiagnosticLevel level{DiagnosticLevel::OK};
  std::string message;
  std::vector<DiagnosticKeyValue> key_values;
  std::vector<LogRequest> logs;

  void add_key_value(DiagnosticKeyValue key_value);
  // DiagnosticsInterface と同じ蓄積: レベルは引き上げ、メッセージは "; " で連結。
  void update_level_and_message(DiagnosticLevel new_level, const std::string & new_message);
  void log(DiagnosticLevel level, std::string throttle_key, std::string message);
};

}  // namespace autoware::ndt_scan_matcher
```

`DiagnosticsInterface::add_key_value` は算術型を `std::to_string`、bool を `"True"`/`"False"`、文字列をそのまま格納する（`autoware_utils_diagnostics/diagnostics_interface.hpp:70-91`）。variant を `std::visit` して同じオーバーロードへ渡せば整形はバイト単位で一致する。ただし**元の呼び出しと同じ `std::to_string` オーバーロード分類に落とすこと** — 現状 `size_t` / `float` / `int` / `int64_t` が混在しており、`float`→`double` と `size_t`→`int64_t` は本パッケージの値域では文字列表現が変わらないが、`unsigned` を `double` に落とすと変わる。

Node 側の転送は 2 本の小さな関数で済む。`apply_diagnostics_update` は参照ブランチにあるものをそのまま使う。ログは **`switch` の `case` ごとに別のマクロ展開**にすること — `RCLCPP_*_THROTTLE` の throttle 状態はマクロ展開位置のローカル static なので、1 箇所に集約して引数で重大度を切り替えると 3 つの独立した時計が 1 つに縮退し、「TF エラーが出ている間はスコア警告が出ない」という新しいバグが生まれる。

```cpp
// ndt_scan_matcher_node.cpp
void NDTScanMatcher::apply_report(DiagnosticsInterface & diag, const DiagnosticsReport & report);

void NDTScanMatcher::replay_logs(const std::vector<LogRequest> & logs)
{
  for (const auto & log : logs) {
    switch (log.site) {
      case LogSite::ScanTransformFailed:
        RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, log.message);
        break;
      case LogSite::ScanOutOfMapRange:
        RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, log.message);
        break;
      // ... site ごとに別の case（= 別のマクロ展開）を必ず書く
    }
  }
}
```

### `pose_interpolation_buffer.hpp`（`SmartPoseBuffer` の vendoring）

`autoware::localization_util::SmartPoseBuffer` は ctor が `rclcpp::Logger`、`interpolate`/`pop_old` が `rclcpp::Time` を取るため、そのままではコアに置けない。本パッケージ内に ROS-free 版を持ち込む。実装は `smart_pose_buffer.cpp`（158 行）と `util_func.cpp` の `interpolate_pose` / `calc_twist` の移植で、`rclcpp::Time` は `builtin_interfaces::msg::Time` からナノ秒 `int64_t` に落として扱う（`rclcpp::Time` の比較・減算は元々ナノ秒整数演算なので数値的に等価）。`RCLCPP_INFO` / `RCLCPP_WARN` は `LogRequest` に置き換える。

```cpp
class PoseInterpolationBuffer
{
public:
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

  struct InterpolateResult
  {
    PoseWithCovarianceStamped old_pose;
    PoseWithCovarianceStamped new_pose;
    PoseWithCovarianceStamped interpolated_pose;
  };

  PoseInterpolationBuffer(double pose_timeout_sec, double pose_distance_tolerance_meters);

  void push_back(PoseWithCovarianceStamped::ConstSharedPtr pose_msg_ptr);

  // 失敗理由は diagnostics.logs に積まれる（元の RCLCPP_INFO / RCLCPP_WARN に対応）。
  [[nodiscard]] std::optional<InterpolateResult> interpolate(
    const builtin_interfaces::msg::Time & target_time, DiagnosticsReport & diagnostics);

  void pop_old(const builtin_interfaces::msg::Time & target_time);
  void clear();
};
```

> 注: `interpolate_pose` は「いずれかの stamp が 0 秒なら空 Pose を返す」という分岐を持つ。仕様化テスト `PublishedInitialPoseIsTheInterpolatedMidpoint` と `InitialPoseDistanceToleranceReachesTheInterpolationBuffer` がこの経路を通るので、移植時に落とさない。

### `scan_matching_module.hpp`（ホットパス）

`callback_sensor_points` と `callback_sensor_points_main` の全体が移る。19 キー・8 ゲート・14 publish が絡む最大の区間。

#### 注入 3 つの洗い直し（ステップ 5 の原則を適用した結果）

当初案は `std::function` を 3 つ注入する形だったが、**2 つは消せる**。

| 当初案 | 結論 | 理由 |
|---|---|---|
| `TransformLookupFunction` | **入力にする** | 変換は `sensor_frame`（メッセージのヘッダ）→ `base_frame`（パラメータ）で、呼び出し前に両方分かる。Node が引いた結果を渡せばよい |
| `HasSubscriberFunction` | **2 つ目のメソッドにする** | 「購読者がいるか」は純粋に ROS の関心事。Node が判断して、必要なときだけ追加の計算を頼む |
| `OutOfMapRangeFunction` | **残す（ただし参照渡し）** | 補間後にしか位置が決まらず、`MapUpdateModule` への問い合わせが要る。同じコア層同士の協調なので `std::function` で隠す必要はない |

**TF を入力にしても診断は割れない。** 失敗の情報を構造体で渡せば、キー `is_succeed_transform_sensor_points` と ERROR メッセージと throttle ログはコアが出せる。

```cpp
// Node が tf2_buffer_ を引いた結果。失敗時は transform を空にし、error に例外文言を入れる
//（診断メッセージが ex.what() を前置きするため、bool ではこの形にならない）。
struct TransformLookup
{
  std::optional<geometry_msgs::msg::TransformStamped> transform;
  std::string error;
};
```

Node が常に TF を引くことになる（現状は空スキャンなら引かない）が、`lookupTransform(TimePointZero)` はキャッシュ読みで無視できる。キーの位置は変わらない — コアは 4 番目に出し、空スキャンは 2 番目で早期 return するのでキーは出ないままになる。

**`out_of_map_range` を Node 側に出すのは不可。** この WARN は何もゲートせず助言だけなので、コアが返した補間姿勢を使って Node が後から問い合わせる案が成立しそうに見える。だが現状この確認は `is_set_map_points` ゲートの**前**にあり、地図がない場合にも WARN が出る。後に回すとその場合に WARN が消える。`OutOfMapRangeIsAWarnOnTheScanAndAnErrorOnTheTimer` は地図ありの経路しか通らないので、**この差は網が捕まえない**。

#### なぜ `Search` のような段階分けにしないか

`PoseInitializationSearch` は「同じ処理を N 回繰り返し、その都度呼び出し側が publish する」形だったので段階分けが自然だった。ホットパスは 1 スキャンにつき 1 回で、外部が必要な地点が 3 箇所（TF / 範囲外照会 / 購読者数）に散っている。段階に割ると `DiagnosticsReport` を段階間で渡し歩くことになり、「レポートの組み立てが 1 箇所」という性質が壊れる。**1 回の呼び出し + 条件付きの 2 つ目のメソッド**が下限。

#### インタフェース案

```cpp
class ScanMatchingModule
{
public:
  using PointSource = pcl::PointXYZ;
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointSource>;
  using CloudPtr = pcl::shared_ptr<pcl::PointCloud<PointSource>>;

  struct Params { /* frame, sensor_points, score_estimation, covariance, validation, regularization */ };

  struct TransformLookup
  {
    std::optional<geometry_msgs::msg::TransformStamped> transform;
    std::string error;
  };

  struct ScanInput
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr scan;  // センサフレームのまま
    // this->now()。コアは時計を持たないので、遅延計算のために受け取る。
    builtin_interfaces::msg::Time now;
    bool is_activated{};
    // Node が引いた sensor_frame -> base_frame。
    TransformLookup base_from_sensor;
  };

  // align まで到達したときだけ埋まる、Node が publish する材料。すべてメッセージ型か POD。
  struct AlignedOutput
  {
    builtin_interfaces::msg::Time stamp;   // すべての stamp はこれ 1 つ

    // ★ 現状 estimate_covariance の内側から、他の 14 本より先に publish される。
    std::optional<geometry_msgs::msg::PoseArray> multi_ndt_pose;
    std::optional<geometry_msgs::msg::PoseArray> multi_initial_pose;

    geometry_msgs::msg::PoseWithCovarianceStamped interpolated_pose;
    geometry_msgs::msg::TransformStamped tf;   // ★ 無条件。pose と同じ分岐に巻き込まない

    // 収束時のみ engaged。2 つは必ず同時。is_converged を bool で外に出さないのは、
    // Node に分岐を書かせないため。
    std::optional<geometry_msgs::msg::PoseStamped> ndt_pose;
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> ndt_pose_with_covariance;

    float exe_time_ms{};
    float transform_probability{};
    float nearest_voxel_transformation_likelihood{};
    int32_t iteration_num{};

    visualization_msgs::msg::MarkerArray ndt_marker;
    geometry_msgs::msg::PoseStamped initial_to_result_relative_pose;
    float initial_to_result_distance{};
    float initial_to_result_distance_old{};
    float initial_to_result_distance_new{};
    sensor_msgs::msg::PointCloud2 sensor_points_in_map;

    // no_ground_points.enable のときだけ。3 つは必ず揃う／揃わない。
    struct NoGroundScore
    {
      sensor_msgs::msg::PointCloud2 points;
      float transform_probability{};
      float nearest_voxel_transformation_likelihood{};
    };
    std::optional<NoGroundScore> no_ground;
  };

  struct Result
  {
    DiagnosticsReport diagnostics;         // キー 1〜18。19 は Node が append する
    std::optional<AlignedOutput> aligned;  // align まで到達しなければ空

    // 距離ゲートを通過したら non-null。Node が受け取って自分のメンバに代入し、
    // align サービスへ渡す（所有は Node 側）。
    CloudPtr scan_in_baselink_frame;

    // ★ skipping_publish_num の計算 *専用*。publish の可否をこれで判断してはならない。
    //   可否はすべて AlignedOutput 内の optional で表現済み。
    bool succeeded{};
  };

  ScanMatchingModule(Params params);

  // ホットパス全体。`ndt` と `map_update` は呼び出し側が所有し、NDT のロックは
  // この呼び出しの間ずっと保持されている前提。
  [[nodiscard]] Result scan_match(const ScanInput & input, NdtType & ndt, MapUpdateModule & map_update);

  // voxel_score_points を購読している者がいるときだけ Node が呼ぶ。align 済みの結果に対して
  // 追加で計算するので、同じ NDT ロックの内側で呼ぶこと。
  [[nodiscard]] sensor_msgs::msg::PointCloud2 colorize_voxel_scores(
    NdtType & ndt, const sensor_msgs::msg::PointCloud2 & sensor_points_in_map) const;

  // 購読で受けた姿勢をバッファへ（frame_id / is_activated の検証も含む）。
  void push_initial_pose(
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose, bool is_activated,
    DiagnosticsReport & diagnostics);
  void push_regularization_pose(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose);
  void clear_initial_pose_buffer();
  [[nodiscard]] std::optional<geometry_msgs::msg::Point> latest_ekf_position();
};
```

Node 側:

```cpp
bool NDTScanMatcher::callback_sensor_points_main(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  ScanMatchingModule::ScanInput input{msg, this->now(), is_activated_, lookup_base_from_sensor(msg)};

  ScanMatchingModule::Result out;
  ndt_ptr_.with([&](const auto & ndt_ptr) {
    out = scan_matching_->scan_match(input, *ndt_ptr, *map_update_module_);
    if (out.aligned && voxel_score_points_pub_->get_subscription_count() > 0) {
      voxel_score_points_pub_->publish(
        scan_matching_->colorize_voxel_scores(*ndt_ptr, out.aligned->sensor_points_in_map));
    }
  });

  if (out.scan_in_baselink_frame) { sensor_points_in_baselink_frame_ = out.scan_in_baselink_frame; }
  replay_logs(out.diagnostics.logs);
  if (out.aligned) { publish_scan_matching_output(*out.aligned); }
  apply_diagnostics_update(*diagnostics_scan_points_, out.diagnostics);
  return out.succeeded;
}
```

**`publish_scan_matching_output` に書いてよい制御構文は `if (opt)` だけ**を不変条件にする。`bool` を見て分岐させると `NonConvergedScanSuppressesPoseButStillBroadcastsTf` が名指ししている「TF は無条件、pose は収束時のみ」の非対称性が必ず壊れる。

#### 決定と未決事項

**決定: `MapUpdateModule &` を `scan_match()` の引数で渡す。** 同じコア層同士の協調を `std::function` で隠す理由はなく、依存が呼び出し側で見える。単体テストで `MapUpdateModule` を組む手間は `test_map_update_module.cpp` の一式で足りる。ホットパスの単体テストで `MapUpdateModule` の準備がテスト本体より長くなるようなら、狭いインタフェース（`class MapCoverage { virtual bool out_of_map_range(const Point &) = 0; }`）に切り替える。

1. **補間バッファ 2 本をモジュールに移すか。** 補間は判定なのでモジュール側が筋だが、`clear_initial_pose_buffer()` と `latest_ekf_position()` のアクセサが要る（trigger サービスとタイマーが触るため）。
3. **`AlignedOutput` の 20 フィールドをこのまま持つか。** publish するものを全部返す設計の帰結。分けるとしたら「常に publish するもの」と「条件付きのもの」だが、条件は既に optional で表現されている。

### `pose_initialization_module.hpp`（`ndt_align` の TPE 推定）

`align_pose` はパーティクルごとに `points_aligned` を publish し、20 回に分けて marker を publish する。これは「進捗を見せるため」の意図的な設計で、仕様化テスト `SuccessfulAlignEmitsTheseKeysAndOneCloudPerParticle` が publish 回数を pin している。したがって「全部計算してから返す」形にはできず、**1 パーティクルごとのコールバックを注入する**。marker の 20 分割は publish の都合なので Node 側が持つ。

```cpp
class PoseInitializationModule
{
public:
  struct Params
  {
    int64_t particles_num{};
    int64_t n_startup_trials{};
    std::string map_frame;
    double converged_param_nearest_voxel_transformation_likelihood{};
  };

  // 1 パーティクル分の途中経過。Node はこれを受けて marker を溜め、cloud を publish する。
  struct Progress
  {
    int64_t index{};
    Particle particle;
    // ndt 結果の姿勢で変換済みのセンサ点群（points_aligned 用）。
    sensor_msgs::msg::PointCloud2 sensor_points_in_map;
  };
  using ProgressCallback = std::function<void(const Progress &)>;

  struct Request
  {
    // Node が TF で map フレームへ変換済みのもの。
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_in_map_frame;
  };

  struct Result
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance;
    double score{};
    bool reliable{};   // converged_param_nvtl < score
  };

  PoseInitializationModule(Params params, ProgressCallback on_progress);

  // align 対象の NDT とセンサ点群は ScanMatchingModule が所有しているので参照で受け取る。
  // is_set_map_points / is_set_sensor_points の判定と診断もここで行う。
  [[nodiscard]] std::optional<Result> estimate(
    NdtType & ndt, const CloudPtr & sensor_points_in_baselink_frame, const Request & request,
    DiagnosticsReport & diagnostics);
};
```

`service_ndt_align_main` に残る Node 側の仕事は、TF ルックアップ（失敗時の診断とエラーログ）、`map_frame` への変換、`MapUpdateModule` による地図読み込み（参照ブランチの `install_map_update`）、`estimate` の呼び出し、レスポンスの組み立てだけになる。

### `map_update_module.hpp`

参照ブランチのものをそのまま採用する。`DiagnosticLevel` / `DiagnosticKeyValue` / `DiagnosticsReport` の定義は `diagnostics_report.hpp` へ移し、`MapUpdateModule` はそれを使う側になる。

### 分離後の Node に残るもの

pub/sub/service/timer/callback group の生成、`tf2_ros` の buffer / listener / broadcaster、`this->now()`、`DiagnosticsInterface` 6 本、`LoggerLevelConfigure`、throttle 付きログの再生、`HyperParameters`（`declare_parameter`）、`is_activated_`、そして 3 モジュールの所有と結線。判定は 1 つも残らない。

---

## 崩してはいけない不変条件

- **診断メッセージの連結順**。`AligningOutsideMapRangeFailsWithThreeJoinedMessages` は `ndt_align_service_status` のメッセージを `"update_ndt failed. ...; No InputTarget. ...; ndt_align_service is failed."` と完全一致で assert している。この 3 本は `MapUpdateModule` → `PoseInitializationModule` → `service_ndt_align` のラッパという**3 つの異なる層**から積まれる。したがって 1 本の `DiagnosticsReport` を呼び出し順に渡し歩く形を崩してはいけない（層ごとに別のレポートを作って後で結合する設計は不可）。
- **サービス名・トピック名は相対名のまま**（`ndt_align_srv` / `trigger_node_srv` / `points_raw` / `pcd_loader_service`）。ノード名や namespace を与えるとテストのスタブが黙ってハングする。
- **callback group 構成**。`callback_sensor_points` / `service_ndt_align` / `service_trigger_node` が同一 MutuallyExclusive、タイマーが専用グループ。`MultiThreadedExecutor` 前提で、align サービスが地図取得サービスを同期呼び出しする経路が成立している。
- **`config/ndt_scan_matcher.param.yaml` のパラメータ名**。テストは実 yaml をそのまま `parameter_overrides` に流し込むので、`declare_parameter` の名前を 1 つ変えただけで全ケースが落ちる。
- **SIMD コンパイルフラグ**をライブラリ間で揃えること（前述）。

## ロックスコープについて

現状 `callback_sensor_points_main` は `ndt_ptr_.with(...)` の内側で publish まで行っている。分離後は publish が Node 側に移るのでロック外になる。これは安全: `callback_sensor_points` と `service_ndt_align` / `service_trigger_node` は**同一の MutuallyExclusive callback group** にあり同時実行されない。`ndt_ptr_` に対するもう一方の競合相手は地図更新タイマー（別 group）だけで、算出済みのメッセージを publish している最中に地図が差し替わっても観測可能な差は生じない。

`sensor_points_in_baselink_frame_` は宣言上 mutex を持たない。**これを守っているのは `ndt_ptr_` の mutex ではなく callback group である** — 書き手 `callback_sensor_points` と読み手 `service_ndt_align` はどちらも `sensor_callback_group`（MutuallyExclusive）にいるので executor が直列化している。`ndt_ptr_.with()` の内側にあるのは偶然で、保護の根拠ではない。抽出時にこの根拠をコメントとして書き残すこと（現状どこにも書かれていない）。

```cpp
// Written by callback_sensor_points, read by service_ndt_align. Both are registered in
// sensor_callback_group, which is MutuallyExclusive, so the executor -- not a mutex --
// is what keeps these two from overlapping. Moving either callback to another group
// makes this a data race.
pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame_;
```

所有者は **Node**。`ScanMatchingModule` の出力に載せて Node が代入し、align サービスへは `const CloudPtr &` で渡す。2 モジュールが共有する状態はどちらの内部状態でもなく、片方に持たせると getter で内部を漏らすかモジュール間の直接依存が生まれる（`MapUpdateModule` が「Node の持つ NDT には一切触れない」という規律と同じ）。`Guarded` に包むのは無競合な追加ロックで振る舞い上は no-op だが、「保護の根拠が mutex になった」と読めてしまうので包まない。

格納タイミングは `SensorPointsAreStoredEvenWhileDeactivated` が pin している — 最大距離ゲートを通過した**後**、活性化ゲートで `return false` する**前**。出力経由にすると代入はコールバック全体の後になるが、途中で読む主体が存在しない（align は同一グループで直列）ため観測不能。

### lock narrowing は 2 段階に分ける

- **段階 A（本リファクタの範囲）**: ロック保持範囲は現状のまま（コアの呼び出し全体がロック内）。変わるのは publish 14 本と TF broadcast がロック外に出ることだけ。NDT に触る計算（marker の `getMaximumIterations()`、`calculateNearestVoxelScoreEachPoint`、no_ground の 2 スコア）はすべてコア内 = ロック内に残るので、ロック外のコードは NDT に一切触れない。
- **段階 B（別 PR）**: ロック内で `shared_ptr` のスナップショットを取り、align 前にロックを離す。これは参照ブランチの `MapUpdateModule` が「設置済み NDT を変異させず、`std::make_shared<NdtType>(*cached_map.ndt)` で別オブジェクトを渡す」ようになったことで初めて成立する（現 `main` の `MapUpdateModule` は設置済み NDT をその場で書き換えるので、スナップショットは真のデータ競合になる）。マップロードが align と並行に進むようになり `map_update_status` のタイミングが変わるため、切り離す。

---

## 進め方（スタックする PR 列）

各ステップ単独で仕様化テストが緑であること。順序には理由がある。

| # | 内容 | 狙い |
|---|---|---|
| 0 | `test/ndt-scan-matcher-characterization-optional-paths` から分岐する | 安全網の確定 |
| 1 | ~~参照ブランチの載せ替え~~ → **完了。実際には upstream `#1322` を 3 分割した fork PR #6/#7/#8 になった**（`refactor/ndt-map-update-ros-free-v2` / `fix/ndt-map-update-retry` / `fix/ndt-map-update-rebuild-decision`）。参照ブランチは `#1322` の revert 前を土台にしており 142 行ずれていたため、`#1322` head から組み直した | イディオムの確立。以降のステップはこれを真似るだけになる |

**抽出は fork #6（`refactor/ndt-map-update-ros-free-v2`）の上に積む。** #7 は独立して正しい振る舞い修正、#8 は B1+B2 の回帰を含むので、どちらも抽出の土台にはしない。抽出が「振る舞いを 1 ビットも変えない」ことを、振る舞い修正を含まない段の上で示す。
| 2 | `DiagnosticsReport` を `diagnostics_report.hpp` へ昇格し、`LogRequest` を追加。Node に throttle 付き再生ヘルパを置く | ロジック移動なし。全モジュールが使う土台 |
| 3 | ファイル改名（`ndt_scan_matcher_core.*` → `ndt_scan_matcher_node.*`）と CMake のライブラリ分割、`test_core_is_ros_free` の追加 | 純粋な移動。以降どこにコードを置けばよいかが自明になる |
| 4 | `SmartPoseBuffer` を `PoseInterpolationBuffer` として vendoring | ホットパス抽出の前提。バッファ単体テストをここで書ける |
| 5 | `PoseInitializationModule` の抽出（`align_pose` + `service_ndt_align_main` の判定部） | ホットパスより小さく、align サービスの仕様化テストが厚い。注入コールバックの形をここで検証する |

ホットパスは **モジュールを作る前に、ファイルを動かさない 4 コミットで形を整える**。これが最も危険な区間なので、抽出と混ぜない。ファイルが変わらないので diff がそのまま読め、テストの実行条件も変わらない。

| # | 内容 | 網 |
|---|---|---|
| 6 | `estimate_covariance` を純関数化し `CovarianceEstimate{covariance, multi_ndt_pose, multi_initial_pose}` を返す。2 本の publish は呼び出し側へ | `MultiNdtCovarianceEstimationPublishesOneResultPerOffset`, `MultiNdtScoreCovarianceEstimationPublishesOnlyTheInitialPoses`, `EstimatedCovarianceOverwritesOnlyFourOfThirtySixEntries` |
| 7 | `ndt_ptr_.with` のラムダ内で `PublishPayload` を組み立て、**ロック解放後**に publish。ラムダ内には NDT に触る計算だけ残す。`exe_time` の計測開始・停止点を動かさない | `ConvergedScanPublishesTheseTopicsAndNotThose`, `NonConvergedScanSuppressesPoseButStillBroadcastsTf`, `NoGroundScoringPublishesTheFilteredCloudAndItsTwoScores` |
| 8 | 3 つの throttle ログを `std::vector<LogRequest>` に置き換え、`replay_logs` で再生 | `ScanWithoutATransformIsAnError`, `OutOfMapRangeIsAWarnOnTheScanAndAnErrorOnTheTimer` |
| 9 | `diagnostics_scan_points_->add_key_value` をローカル `DiagnosticsReport` に置き換え、末尾で `apply_report` | `ScanMatchingStatusEmitsExactlyTheseNineteenKeys` + 早期 return 6 ケース |
| 10 | 9 の関数本体を**そのまま** `ScanMatchingModule::scan_match` へ切り出し、4 つの ROS 依存を `ScanInput` の 3 フィールド + `MapUpdateModule &` 引数に置換。**注入はゼロ** | 全ケース |
| 11 | 補間バッファ 2 本と `latest_ekf_position_` をモジュールへ。Node の購読コールバックは `push_*` を呼ぶだけに | `InitialPoseIsRejectedBeforeTheFrameCheckWhenNotActivated` 他 |

**実績（2026-08-31 完了）: ステップ 10 と 11 は 1 コミットにまとめた。** ホットパスが補間バッファを 2 回参照しており、分けると `PoseInterpolationBuffer &` を引数で渡す中途半端なシグネチャを一度経由することになるため。

抽出中に踏んだ取りこぼし 3 件（いずれもコンパイルは通った）:

| 症状 | 原因 | 検出したもの |
|---|---|---|
| `pcl_conversions` が見つからない | 公開ヘッダにテンプレート実装を置いた | `test_core_is_ros_free` |
| ROS テスト全件が `missing_result` | `scan_matching_module_` を宣言のみで未生成 | 仕様化テストの全件実行 |
| 全ケースで補間失敗 | バッファを移したが push 側のコールバックを繋ぎ替えず | 仕様化テストの失敗内容 |

3 件目は「静かな機能停止」で、個別実行では 1 件目のケースしか走らず気づけなかった。`missing_result` はアサーション失敗より重い信号として読むこと。

ステップ 9 が終わった時点で `callback_sensor_points_main` は「入力を取る → 純粋に計算する → データを返す」形になり、rclcpp への参照が 4 つに減っている。ステップ 10 はその 4 つを差し替えるだけの機械的な移動になる。

**publish 関数に書いてよい制御構文は `if (opt)` だけ**を不変条件にすること。`bool` を見て分岐させると `NonConvergedScanSuppressesPoseButStillBroadcastsTf` が名指ししている「TF は無条件、pose は収束時のみ」の非対称性が必ず壊れる。`is_converged` は `ndt_pose` / `ndt_pose_with_covariance` の `optional` に畳み込み、Node には渡さない。

---

## 検証

- **仕様化テスト**（各ステップ必須。1 ケースでも落ちたらそのステップは間違い）
  ```bash
  source /opt/ros/humble/setup.bash && source install/setup.bash
  colcon build --packages-select autoware_ndt_scan_matcher --cmake-args -DCMAKE_BUILD_TYPE=Release
  colcon test --packages-select autoware_ndt_scan_matcher --event-handlers console_direct+
  colcon test-result --verbose
  ```
  順序依存を炙り出すため、少なくとも 1 度は `GTEST_SHUFFLE=1 GTEST_RANDOM_SEED=<n>` を付けて回す。
- **既存の統合テスト 3 本と launch test** も同じ実行に含まれる。`ndt_align_srv` / `trigger_node_srv` / `points_raw` の相対名、`MultiThreadedExecutor` 前提の callback group 構成、`config/ndt_scan_matcher.param.yaml` のパラメータ名を変えていないことがここで担保される。
- **コア層の ROS-free 性**: `test_core_is_ros_free` のビルドが通ること。加えて
  ```bash
  grep -nE 'rclcpp|tf2_ros' include/autoware/ndt_scan_matcher/{diagnostics_report,pose_interpolation_buffer,map_update_module,scan_matching_module,pose_initialization_module,particle,guarded}.hpp src/{map_update_module,scan_matching_module,pose_initialization_module,pose_interpolation_buffer,ndt_scan_matcher_helper,particle}.cpp
  ```
  が無出力であること。
- **コア単体テスト**（ステップ 4 以降、モジュールごとに追加）: 参照ブランチの `test/test_map_update_module.cpp` と同じ形で、注入した `std::function` をスタブに差し替え、ノードを立てずにモジュールを直接駆動する。仕様化テストは寿命の限られた足場なので、モジュールが 1 つ ROS-free になるたびに、その範囲の pin をコア単体テストへ移していく。
- **pre-commit**: `source /opt/ros/humble/setup.bash && pre-commit run --all-files`

---

## 仕様化テストが捕まえない落とし穴（実装時に個別に守る）

網が緑のまま壊せる箇所。整頓の衝動が最も危険なところなので、触らないと決めておく。

1. **共分散の 4 要素上書き**。36 要素のうち `[0]`, `[7]`, `[1]`, `[6]` の 4 つだけを書き、しかも off-diagonal は転置されている（`[1] = adj(1,0)`, `[6] = adj(0,1)`）。バグだが**直さない**。ループに畳んで「きれいに」書き直すと必ず変わる。4 行をそのまま移す。
2. **`no_ground` 点群の `width`/`height` が 0**。`push_back` だけで設定せず `toROSMsg` が両方 0 から導出している。`height = 1` を足すと `width = 0` が publish される。
3. **フレーム一致時のエイリアス**。`transform_sensor_measurement` は `source_frame == target_frame` のとき TF を一切引かず入力ポインタをそのまま出力に代入する（コピーしない）。この短絡を保たないと TF 注入関数が余計に呼ばれ、点群が 1 回余分にコピーされ、`execution_time` が伸びる。
4. **`voxel_score_points` の購読者判定は align の *後***。`bool` で事前捕捉すると「align 中に購読が張られた」ケースの挙動が変わる。テストはこのトピックを意図的に捕捉していない（捕捉すると条件そのものを作ってしまう）ので、**この差分を捕まえる網は存在しない**。必ず `std::function<bool()>` で注入する。
5. **`multi_ndt_pose` / `multi_initial_pose` は他の 14 本より先に publish される**（`estimate_covariance` の内側から出るため）。DDS 上は観測不能だが、Node 側でも先頭に置く。
6. **秒への変換は除算**。`rclcpp::Duration::seconds()` は `static_cast<double>(ns) / 1e9`。移植時に `ns * 1e-9` と書くと最下位 ulp が変わる。vendoring する補間バッファの `dt` も同じで、**int64 ns で差を取ってから `/ 1e9`**。double 秒に変換してから引き算してはいけない。
7. **`std::to_string(float)` と `std::to_string(double)` はバイト同一**（`%f` への可変長引数で float が double に昇格するため）。よって variant に `float` を足す必要はない。足すと `%f` の経路が 2 つできて逆に危険。
8. **`unsetRegularizationPose()` は補間の成否によらず無条件に呼ばれる**（前スキャンの残留を消すため）。`setRegularizationPose()` は成功時のみ。この非対称性を落とさない。`RegularizationSubscriberRecordsOneKeyAndTheEnabledPathConverges` は購読側の 1 キーしか見ておらず、網が薄い。
9. **`ndt_align_srv` が `sensor_callback_group` にいること**が本リファクタ全体の安全性の土台（前掲）。どこにも書かれていないので、コメントを足す。可能なら「両者が同一グループであること」を検査する小さなテストを添える。

## 別チケットに切り出すもの（本リファクタでは触らない）

- **`skipping_publish_num` のメンバ化**。プロセス横断の関数ローカル static。直すべきだが、網が捕まえない振る舞い変更（プロセス毎 → ノード毎）なので、単独・ラベル付きのコミットで。
- ~~**lock narrowing（段階 B）**。前掲。~~ → **2026-09-01 完了**（`refactor/ndt-map-update-owns-its-map`）。参照ブランチの `MapUpdateModule` を移植し、モジュールが自前の地図を持ち `update()` が新しい NDT を返す形になったため、参照 node と同じ条件付きロック（`install_map_update`: 範囲外のときだけロード全体を保持、それ以外は swap のみ）を採用した。B1（derived need_rebuild）+ B2（Failed は位置を記録しない）+ adopt-on-success の 3 点が揃うと座礁しない。SUSPICIOUS 2 件はこの修正を予告していたので書き換えた。
- `NDTScanMatcher::count_oscillation` は `ndt_scan_matcher_helper.cpp` の自由関数へ委譲するだけの空の private static。削除できる。
- `publish_point_cloud` は `frame_id` を引数に取りながら、常に `sensor_aligned_pose_pub_` へ固定 publish している。名前と実体が合っていない。
- `out_of_map_range` の不等式がホットパスとタイマーの 2 箇所に書かれている。注入で実装は 1 本化されるので、タイマー側の記述を消すだけ。
- `ConvergedParamType` の値検証が align の**後**にしかない（`UnknownConvergedParamTypeIsAnErrorAfterAligning` が示す潜在バグ）。
- 地図更新タイマーの周期 1.0 秒が `constexpr` でパラメータ化されていない。
- `ndt_scan_matcher_node.cpp` の using 宣言 4 本（`TreeStructuredParzenEstimator` / `exchange_color_crc` / `matrix4f_to_pose` / `pose_to_matrix4f`）と `<deque>` `<map>` `<tuple>` `<thread>` `fmt` の include が未使用。いずれも PR #18 より前から死んでいる（計測済み）。

---

# コア root `NdtScanMatcher` への移行（2026-09-01 追加。fork PR #18〜#20）

> **実績（2026-09-03）: fork PR #19〜#28 として 10 本のドラフト PR を作成済み**（#18 は欠番）。
> #19 align 1 件化 / #20 root / #21 コア単体テスト / #22 out-parameter 除去 / #23 Params をアダプタへ /
> #24 活性状態 / #25 voxel_score_points パラメータ化 / #26 ScanInput 廃止 / #27 friend 除去 /
> #28 MapUpdateModule 移植（lock narrowing 段階 B を含む）。振る舞いを変えるのは #19・#25・#28 の 3 本。

## Context

分離は済んだが、3 つのコア部品は Node が並べて持つ兄弟のままで、Node は `Guarded<NdtPtrType> ndt_ptr_`・スキャンバッファ・スキップカウンタという**コアの状態**を保持し続けている。`MapUpdateModule` は Node の `ndt_ptr_` を参照で借りており、所有関係が逆立ちしている。

has-a を見送った理由は原則 0a に記録したとおり**ただ一つ**、align 探索がパーティクルごとに publish するため Node がロックの内側でループを回さねばならない、という点だった。その進捗 publish は「進捗を見せ、データを落とさないため」という debug 目的の設計で、本番設定（`particles_num` 既定 100）では depth 10 の `points_aligned` に重い点群を 100 回流している。ここに意図的な仕様変更を入れれば、`PoseInitializationSearch` は**呼び出しをまたぐ状態を持たなくなり**（原則 0b）、class ですらなくなって自由関数 1 本に潰れる。同時に has-a を阻んでいた唯一の理由が消える。

到達点は「Node は I/O だけ、判定と状態はすべてコア root が持つ」。root は**外部イベント 1 つにつきメソッド 1 本**を公開し、各メソッドは「Node が引かねばならなかった事実を受け取り、Node が出さねばならないものを返す」。

現状の裏取り:

- Node が `ndt_ptr_` に触るのは 3 箇所だけ — `ndt_scan_matcher_node.cpp:187`（`setParams`）、`:434`（`scan_match`）、`:620`（align 探索）。進捗 publish が消えれば 3 つとも root の内側に入り、Node から `ndt_ptr_` が丸ごと消える。
- 仕様化テストが pin しているのは `points_aligned` の**件数だけ**（`test_ndt_scan_matcher_characteristics.cpp:1552`）。`monte_carlo_initial_pose_marker` は 1 ケースも捕捉していないので、20 分割 flush は網の外。
- `AligningOutsideMapRangeFailsWithThreeJoinedMessages` はキー順を直接 assert せず、値の一致・`is_set_sensor_points` の不在・連結メッセージの完全一致を見ている。下記の設計はいずれも保つ。

---

## PR #18 — 仕様変更: align が publish する点群を 1 件にする

**振る舞いが変わる。単独・ラベル付きの PR にし、本文で明示的に宣言する。**

| トピック | 変更前 | 変更後 |
|---|---|---|
| `points_aligned` | 1 align につき `particles_num` 件（既定 100、depth 10 なので大半は落ちる） | 1 align につき 1 件。最良パーティクルの結果姿勢で変換した点群 |
| `monte_carlo_initial_pose_marker` | 1 align につき約 20 件（`particles_num/20` ごとに flush） | 1 align につき 1 件。全パーティクル分の marker を 1 つの `MarkerArray` に |

探索の可視化は marker 側（初期姿勢・結果姿勢を score / iteration / index で色付け、6 namespace）がそのまま担い続ける。重い点群だけを 1 回に減らす。変換計算も N 回から 1 回になる。

### `pose_initialization_search.hpp` — class を廃し自由関数へ

```cpp
struct PoseInitializationParams { /* 現 PoseInitializationSearch::Params と同じ 4 フィールド */ };
struct PoseInitializationEstimate { /* 現 Estimate と同じ 3 フィールド */ };

struct PoseInitializationResult
{
  std::optional<PoseInitializationEstimate> estimate;
  DiagnosticsReport diagnostics;
  // 前提条件で弾かれたときは両方とも空。呼び出し側に許す制御構文は `if (opt)` だけ。
  std::optional<sensor_msgs::msg::PointCloud2> best_points_aligned;
  std::optional<visualization_msgs::msg::MarkerArray> search_markers;
};

[[nodiscard]] PoseInitializationResult search_initial_pose(
  const PoseInitializationParams & param, NdtType & ndt, const CloudPtr & scan_in_baselink_frame,
  const PoseWithCovarianceStamped & initial_pose_in_map_frame,
  const builtin_interfaces::msg::Time & now);
```

- `next()` / `finish()` / ctor 分割 / `output_cloud_` / `particle_index_` / `pose_sampler_` の寿命管理がすべて 1 本のループに戻る。デストラクタの out-of-line 宣言も不要になる。
- **TPE のシードは現状どおり `initial_pose_in_map_frame.header.stamp`**（PR #16 の判断）。ここは触らない。
- `now` は marker の stamp 用。現状 `get_clock()->now()` をパーティクルごとに渡しているが実質同一瞬間なので、1 つ受け取れば足りる。`ScanInput::now` と同じイディオム。
- **最良パーティクルの `Eigen::Matrix4f` を particles と並走して保持する。** `matrix4f_to_pose` した結果を `pose_to_matrix4f` で戻すと float の往復誤差が入る。`std::max_element` で選んだ添字でそのまま引けるよう、同じ順序の `std::vector<Eigen::Matrix4f>` を持つ（選択基準は 1 つのまま）。
- marker 生成（`push_debug_markers`）はすでに `particle.cpp` にあり ROS-free。ここから呼ぶだけ。

### Node 側

- `publish_pose_initialization_progress` を削除。`monte_carlo_marker_array_` メンバも削除。
- `service_ndt_align_main` の `ndt_ptr_.with` 内は `search_initial_pose(...)` 1 行になり、publish はロック解放後に `if (opt)` 2 本。

### テスト

`SuccessfulAlignEmitsTheseKeysAndOneCloudPerParticle` → `SuccessfulAlignEmitsTheseKeysAndOneAlignedCloud`。`particles_num` の override は残す（10 回 align で済み、実行が速い）が、期待値は `EXPECT_EQ(points_aligned->count(), 1U)` にする。docstring の「cloud count is `particles_num`」も書き換える。

---

## PR #19 — コア root `NdtScanMatcher`（純粋な移動、振る舞い不変）

### 新規 `include/autoware/ndt_scan_matcher/ndt_scan_matcher.hpp` / `src/ndt_scan_matcher.cpp`

コア lib に所属。`test/test_core_is_ros_free.cpp` の include 一覧に追加する（追加しないヘッダは検査対象外）。

```cpp
class NdtScanMatcher
{
public:
  using NdtType = pclomp::MultiGridNormalDistributionsTransform<PointSource, PointTarget>;
  using CloudPtr = pcl::shared_ptr<pcl::PointCloud<PointSource>>;
  // 定義は 1 つに保つ。ScanMatchingModule のものをそのまま公開する。
  using ScanInput = ScanMatchingModule::ScanInput;
  using ScanMatchingOutput = ScanMatchingModule::ScanMatchingOutput;

  struct Params
  {
    pclomp::NdtParams ndt{};
    ScanMatchingModule::Params scan_matching;
    MapUpdateModule::Params map_update;
    PoseInitializationParams pose_initialization;
    // 連続スキップがこの回数に達したら WARN。判定なので Node ではなくここ。
    int64_t skipping_publish_num{};
  };

  NdtScanMatcher(Params param, MapUpdateModule::PcdLoaderFunction pcd_loader);

  // --- 外部イベント 1 つにつきメソッド 1 本 ---

  // `points_raw`
  struct ScanResult
  {
    // キー 1〜19。skipping_publish_num までここで積む。
    DiagnosticsReport diagnostics;
    // publish するものがあるときだけ。Node に許す制御構文は `if (opt)` だけ。
    std::optional<ScanMatchingOutput> output;
  };
  [[nodiscard]] ScanResult match_scan(const ScanInput & input);

  // 1 Hz の地図更新タイマー。参照位置が未設定なら `is_set_last_update_position` を報告して
  // 地図に触らず返す。Node 側の `is_activated` ゲートはノードのライフサイクルなので Node に残る。
  [[nodiscard]] MapUpdateModule::UpdateResult update_map_periodically();

  // `ndt_align_srv`
  struct AlignInput
  {
    // Node が map_frame へ変換済みのもの。TF は Node の仕事。
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_in_map_frame;
    builtin_interfaces::msg::Time now;  // marker の stamp。このクラスは時計を読まない。
  };
  struct AlignResult
  {
    std::optional<PoseInitializationEstimate> estimate;
    // 地図更新 → 探索 の順で 1 本のレポートに積む。連結メッセージの順序はここで保証される。
    DiagnosticsReport diagnostics;
    std::optional<sensor_msgs::msg::PointCloud2> best_points_aligned;
    std::optional<visualization_msgs::msg::MarkerArray> search_markers;
    std::optional<sensor_msgs::msg::PointCloud2> loaded_pcd_map;
  };
  [[nodiscard]] AlignResult align(const AlignInput & input);

  // 購読・trigger サービス
  void push_initial_pose(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose, bool is_activated,
    DiagnosticsReport & report);
  void push_regularization_pose(const PoseWithCovarianceStamped::ConstSharedPtr & pose);
  void clear_initial_pose_buffer();

private:
  Params param_;
  Guarded<std::shared_ptr<NdtType>> ndt_ptr_;
  Guarded<CloudPtr> scan_in_baselink_frame_;
  int64_t skipping_publish_num_{0};
  MapUpdateModule map_update_;        // ndt_ptr_ より後に宣言する（ctor で参照を受ける）
  ScanMatchingModule scan_matching_;
};
```

### 吸収するもの / 消えるもの

| 現 Node のもの | 移動先 | 効果 |
|---|---|---|
| `Guarded<...> ndt_ptr_` と ctor の `setParams` | root | Node から NDT が完全に消える。`MapUpdateModule` が借りる参照が root 自身のメンバになり、所有の逆立ちが解消 |
| `sensor_points_in_baselink_frame_` | root（`Guarded<CloudPtr>`） | 下記参照 |
| `skipping_publish_num_` と `param_.validation.skipping_publish_num` | root | **`ScanMatchingModule::Result::succeeded` が公開インタフェースから消える**。「publish の可否に使うな」という注意書き付きのフィールドが内部に隠れる。フィールド名も `converged` に改める |
| タイマーの `is_set_last_update_position` ゲート | root（`update_map_periodically`） | **`latest_ekf_position()` が公開インタフェースから消える**。Node が読んで地図モジュールに渡し直すだけのアクセサだった |
| `pose_initialization_params_`, `map_update_module_`, `scan_matching_module_` | root | Node のメンバが `matcher_` 1 本に |

### スキャンバッファを `Guarded` で包む理由

現状これを守っているのは mutex ではなく callback group（`callback_sensor_points` と `service_ndt_align` が同一 MutuallyExclusive）で、プランの当該節はそのために「包むな」と書いていた。root に移った時点でその論拠は使えない — **クラスは呼び出し側の executor 構成を仮定できない**（仮定できないことが、ノードなしで単体テストできるということでもある）。現構成では無競合なので追加ロックは振る舞い上 no-op、競合が起きる構成なら現コードはデータ競合なので、包むほうが厳密に正しい。当該節のコメントは「callback group が根拠」から「このクラスは呼び出し側の直列化を仮定しない」へ書き換える。

### ロック順序

新しい mutex を**葉**に保つ。既存の順序（`builder_state_` → `ndt_ptr_`、`builder_state_` → `last_update_position_`、ホットパスの `ndt_ptr_` → `last_update_position_`）に循環はなく、root がその上に何も重ねなければ増えない。

- `match_scan`: `ndt_ptr_` のロックを解放して**から** `scan_in_baselink_frame_` に代入する。
- `align`: `scan_in_baselink_frame_` を**先に**コピー（`shared_ptr` の複製）してから `ndt_ptr_` のロックを取る。
- `align` は `map_update_.update_map()` を**ロックを取る前に**呼ぶ（`map_update_module.hpp` の "Do not call this function while holding the lock for ndt_ptr_"）。現 Node の順序と同じ。コメントを添える。

### 分離後の Node

pub/sub/service/timer/callback group、`tf2_ros` 一式、`this->now()`、`DiagnosticsInterface` 6 本、`LoggerLevelConfigure`、`replay_logs`、`apply_diagnostics_update`、`HyperParameters`、`is_activated_`、そして `matcher_` 1 本。各コールバックは「入力を組む → root のメソッドを 1 本呼ぶ → ログ再生 → `if (opt)` で publish → 診断転送」に揃う。

```cpp
void NdtScanMatcherNode::callback_sensor_points(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  diagnostics_scan_points_->clear();

  NdtScanMatcher::ScanInput input;
  input.scan = msg;
  input.now = this->now();
  input.is_activated = is_activated_;
  input.base_from_sensor = lookup_base_from_sensor(msg->header.frame_id);
  input.voxel_score_points_wanted = voxel_score_points_pub_->get_subscription_count() > 0;

  auto match = matcher_->match_scan(input);
  replay_logs(match.diagnostics.logs);
  if (match.output) {
    publish_scan_matching_output(*match.output);
  }
  apply_diagnostics_update(*diagnostics_scan_points_, match.diagnostics);
  diagnostics_scan_points_->publish(msg->header.stamp);
}
```

### 崩してはいけないもの（この PR 固有）

- **align の診断連結**。Node が `service_call_time_stamp` と `is_succeed_transform_initial_pose` を積み、root が地図更新→探索を 1 本のレポートに積み、Node が最後に `is_succeed_service` を積む。`AligningOutsideMapRangeFailsWithThreeJoinedMessages` の完全一致メッセージはこれで保たれる（むしろ連結が root 内部の性質になる分、現状より強くなる）。
- **`skipping_publish_num` はキー 19 番目**。root が `scan_matching_.scan_match()` の返したレポートに追記する順序を変えない。
- 既存の不変条件（相対サービス名、callback group 構成、param.yaml のキー名、SIMD フラグ）はすべてそのまま。

---

## PR #20 — ノードを立てないコア単体テスト

`test/test_ndt_scan_matcher_core.cpp`。`test_map_update_module.cpp` と同じ形で `stub_pcd_loader` を `PcdLoaderFunction` に差し込み、`NdtScanMatcher` を直接駆動する。仕様化テストは寿命の限られた足場なので、ここから pin を移していく最初の一歩。移す先の候補は、rclcpp なしで再現できる範囲 — 8 つのゲートの early return、19 キーの集合と順序、収束/非収束での optional の有無、align の前提条件 2 つと連結メッセージ。

**移動と振る舞い変更を混ぜないため、#19 とは分ける。** #19 は 42 ケース全緑で「1 ビットも変えていない」ことを示す段、#20 は網を張り替える段。

**実績: 6 ケース。** うち 5 件は 1 ms 前後、`AScanKeptWhileDeactivatedIsTheOneAlignSearchesFrom` だけが 5 パーティクルの探索を回して 2.4 秒。移せた 2 件（上記と `AligningOutsideMapRangeJoinsTheMapAndSearchMessages`）は仕様化テストより素直に読める。**まだ移していない**のは 19 キーと収束/非収束の optional 非対称性で、どちらも「スキャンを収束する align まで通すフィクスチャ」が要る（6 件目がそこまで到達できることは示した）。

スタブ loader には ROS スタブサービスと同じ範囲判定を入れること。入れないと地図中心から遠い要求でもロードが成功してしまい、out-of-range のケースが書けない（一度これで落ちた）。

---

## 検証（#18〜#20 共通、各段で必須）

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
if colcon build --packages-select autoware_ndt_scan_matcher --cmake-args -DCMAKE_BUILD_TYPE=Release; then
  colcon test --packages-select autoware_ndt_scan_matcher --event-handlers console_direct+
  colcon test-result --verbose
else
  echo "BUILD FAILED"
fi
```

**`set -o pipefail` を必ず付けること。** `if colcon build ... | tail; then` はパイプラインの終了ステータスが `tail` のものになるので、ビルドが落ちても常に成功する。これで 3 回目の誤報を出した（前回バイナリの結果を緑と読んだ）。ビルド成功を確認せずにテスト要約を読まないこと、および要約に出ているテスト名が今回のものかを確かめること。少なくとも 1 度は `GTEST_SHUFFLE=1 GTEST_RANDOM_SEED=<n>` を付けて回す。`test_core_is_ros_free` のビルドが通ること、および `grep -nE 'rclcpp|tf2_ros'` がコアの .hpp/.cpp に対して無出力であること。最後に `pre-commit run --all-files`。

---

# 直線化（2026-09-04 完了）

`line/00-characterization-on-main`（仕様化 47 コミットを main `9ee2a5a4` に rebase + ローダ文言の 1 行修正）を出発点に、
1 ブランチ 1 コミットで 12 段。**`line/12-core-unit-tests` は `refactor/ndt-final-version-on-main` と tree 完全一致。**

| 段 | ブランチ | 振る舞い |
|---|---|---|
| 01 | `line/01-skip-counter-per-node` | 変更 |
| 02 | `line/02-tpe-per-instance-rng` | 変更 |
| 03 | `line/03-validate-enum-params` | 変更 |
| 04 | `line/04-align-single-aligned-cloud` | 変更 |
| 05 | `line/05-voxel-score-points-is-a-parameter` | 変更 |
| 06 | `line/06-core-layer-foundation` | 移動（fork #9 + #12 を squash） |
| 07 | `line/07-map-update-owns-its-map` | 変更（#28 の 3 点） |
| 08 | `line/08-vendor-pose-buffer` | 移動 |
| 09 | `line/09-pose-initialization-search` | 移動（自由関数を最終形で直接） |
| 10 | `line/10-scan-matching-module` | 移動（最終インタフェースで直接） |
| 11 | `line/11-core-root` | 移動 |
| 12 | `line/12-core-unit-tests` | テスト追加 |

各段: 両パッケージビルド → `env -u CYCLONEDDS_URI colcon test`（再起動後の `rmem_max` 問題の回避）→ 仕様化 44 / map-update 9 / core 6 / helper 8 / 共分散 8 / 統合 3、全 0 failure。
main 上の merge 版 `#1322` は fork #6 と 118 行ずれるが、必要だった仕様化テストの修正は文字列 1 件のみ。

**PR: 直線は fork ドラフト #29〜#40（01→#29 … 12→#40）。旧 #6〜#17・#19〜#28 は 2026-09-04 に close（各 PR に新番号へのポインタをコメント）。#1〜#5 は残置。**

**2026-09-04 追記: `AlignInput` を廃止**（2 引数 `align(initial_pose_in_map_frame, now)` に。`match_scan` と作法を揃えた）。
直線に往復を持ち込まないため `line/11-core-root`（#39）に amend、`line/12`（#40）を `--onto` で載せ直し。
到達点 `refactor/ndt-final-version-on-main` は新 `line/12` tip（43344a61）へ移動、tree 一致を再確認済み。

