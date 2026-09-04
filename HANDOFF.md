# autoware_ndt_scan_matcher リファクタ — 会話の引き継ぎ

このブランチは会話を別のマシンで再開するためのもので、コードは含まない。読む順は この文書 → `memory/` → `plan.md`。
`memory/` は元の会話が蓄積した作業ルールと環境の落とし穴、`plan.md` は設計計画の全文。

## 何をしているか

`autoware_ndt_scan_matcher` を、メッセージ型しか参照しないコア（root `NdtScanMatcher` が `MapUpdateModule`・`ScanMatchingModule`・自由関数 `search_initial_pose` を持つ）と薄い `rclcpp::Node` に分ける。
安全網は仕様化テスト 42 件（`test/test_ndt_scan_matcher_characteristics.cpp`）。成果は fork `takam5f2/autoware_core` 上の 1 ブランチ 1 コミットの直線 `line/01`〜`line/12`（draft PR #29〜#40）。upstream `origin` には push しない。

## ブランチ地図（fork `takam5f2`）

| ブランチ | 内容 | PR |
|---|---|---|
| `line/00-characterization-on-main` | 仕様化テスト 47 コミット + ローダ文言修正 + ケースの並び替え（tip 9468dd3d、2026-09-04） | なし |
| `line/01`〜`line/05` | 振る舞い変更 5 段（skip counter ノード毎化、TPE の RNG、enum 検証、align 点群 1 件化、voxel_score_points パラメータ化） | #29〜#33 |
| `line/06` | コア層の土台（移動のみ） | #34 |
| `line/07` | MapUpdateModule が地図を所有（振る舞い変更） | #35 |
| `line/08`〜`line/12` | 移動のみ + コア単体テスト | #36〜#40 |
| `refactor/ndt-final-version-on-main` | 到達点。line/12 と tree 一致（43344a61） | — |
| `test/ndt-scan-matcher-characterization-*` | 旧ベース上の仕様化テスト | #1〜#5（open） |
| `refactor/*`, `fix/*` の残り | 旧系列（#6〜#28、closed） | 履歴 |

## 現在地（2026-09-04）

- line/00 に「ケースの並びをノードの流れに揃える」コミットを足して push 済み。**`line/01`〜`12` はまだ旧 line/00（f68d86aa）の上にある。** 載せ直しは未実施。
- 並び替え後の章: A ゲート 7 / B 初期姿勢と活性化 6 / C 収束経路 12 / D `ndt_align_srv` 6 / E 出荷設定 2 / F 地図 5 / G 既定オフ 4。カウンタを進めたまま終わる 6 ケースに `ScopeExit` を足し、順序をプロセス共有の `static` カウンタから独立させた。宣言順と `--gtest_shuffle`（seed 7 / 12345）で 42/42。
- ユーザーは設計説明資料（Confluence）を執筆中。仕様化テストの節は出発点 42 件の表を主にし、到達点 44 件との差分（改名 4、追加 2）を最後にまとめる方針。追加 2 件は元の振る舞いが観測不能で pin できなかった箇所: `UnknownCovarianceEstimationTypeIsRejectedAtConstruction`（line/03）、`VoxelScorePointsFollowItsParameter`（line/05）。

## 次にやること

1. 仕様化テストを 1 件ずつ確認する（読みやすさ優先）。docstring が参照する旧シンボル名は line/00 のコードでは正しいので、line/00 上で読む。
2. `line/01`〜`12` を新しい line/00 に載せ直す。各段 `git rebase --onto <ひとつ下の新 tip> <ひとつ下の旧 tip> line/<i>`。01 は `--onto line/00-characterization-on-main f68d86aa line/01-skip-counter-per-node`。ひとつ下の旧 tip: 01 edb2f75f、02 da8c35be、03 6be1a0b1、04 0ea76175、05 c175bd8d、06 bb62d703、07 c5b24cb1、08 ea7c9666、09 2393df69、10 0a05e536、11 25103c1d、12 43344a61。
   衝突が見込まれる所: 01（`ScopeExit` 機構の撤去。並び替えで足した 7 箇所も一緒に消す）、01 が書き換えるファイル先頭 docstring、移動した上で後段が編集する `ActivatingClearsTheInitialPoseBuffer` / `InitialPoseDistanceTolerance…` / `ReliableIgnoresTheTransformProbabilityThreshold`。
   各段で両パッケージをビルドして全テストを回し、最後に line/12 と `refactor/ndt-final-version-on-main` の差がテストファイルの並びだけであることを確認してから `--force-with-lease` で push し、`refactor/ndt-final-version-on-main` を新 line/12 に移す。
3. line/12 側では `Unknown*` 2 件が構築時の検証になっているので、C に置かず「構築時の検証」の群として独立させる。

## ルール

- fork のみ。PR 本文で人を `@` しない。upstream の PR 番号はコードスパンで書く（`#1322`）。
- 1 ブランチ 1 コミット。後からのインタフェース修正は、それを導入した段に amend する。末尾に足さない。
- 移動だけの段（06、08〜12）はテストファイルを触らない。触る必要が出たら振る舞いが変わった証拠。
- 全コミットに `-s`（DCO）。
- 文章は短く。コードを言い直す文は削る。

## 検証の作法

```bash
cd ~/work/autoware && source /opt/ros/humble/setup.bash && source install/setup.bash
set -o pipefail
colcon build --packages-select autoware_localization_util autoware_ndt_scan_matcher --cmake-args -DCMAKE_BUILD_TYPE=Release
rm -rf build/autoware_ndt_scan_matcher/test_results
env -u CYCLONEDDS_URI colcon test --packages-select autoware_ndt_scan_matcher
colcon test-result --test-result-base build/autoware_ndt_scan_matcher
```

- `| tail` を付けるなら `pipefail` が必須。gtest xml のケース名が今回のバイナリのものか確認する。
- 両パッケージを必ずビルドする（02 で TPE の .so が変わる。片方だけだとリンク不整合）。
- 再起動後は `net.core.rmem_max` が 212992 に戻り、ノードを立てるテストが全て落ちる。`sudo sysctl -w net.core.rmem_max=2147483647` か、root なしなら `env -u CYCLONEDDS_URI`。
- 順序依存は `GTEST_SHUFFLE=1 GTEST_RANDOM_SEED=<n>` で 1 回は回す。

## 別のマシンで始める

```bash
git clone git@github.com:takam5f2/autoware_core.git   # 既存 clone なら remote takam5f2 を追加
git fetch takam5f2
git switch line/00-characterization-on-main
git show takam5f2/notes/ndt-characterization-handoff:HANDOFF.md
```

Claude Code への最初の一文: 「`takam5f2/notes/ndt-characterization-handoff` の HANDOFF.md と memory/ を読んで、そこから続けてください。」
