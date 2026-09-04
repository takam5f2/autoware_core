---
name: ndt-characterization-series-state
description: State of the ndt_scan_matcher fork PR stack — characterization #1–#5, refactor #6–#17 and #19–#28 (as of 2026-09-03); pending items and separate-ticket findings
metadata: 
  node_type: memory
  type: project
  originSessionId: aecf1ef5-5418-4f9f-9d7b-0286124a38ee
  modified: 2026-08-28T12:56:53.279Z
---

Four stacked PRs, all pushed to the fork `takam5f2` (never to upstream `origin`), each
branch based on the previous one:

| Step | Branch (`test/ndt-scan-matcher-characterization-*`) | PR | Cases |
|---|---|---|---|
| PR2 | `harness` | upstream #1348 (open, in review) | 10 gate cases |
| PR3 | `hot-path` | upstream #1405 (draft) + fork #1 | 13 hot-path cases (7 + 6 review-driven) |
| PR4 | `align-service` | fork #2 (draft) | 6 `ndt_align_srv` cases (3 + 3 review-driven) |
| PR5 | `typical-operation` | fork #3 (draft) | 2 shipped-config cases |
| PR6 | `map-update` | fork #4 (draft) | 5 `MapUpdateModule` cases (H); stub gets a 2nd cell at x=300, harness gets `with_map_loader`. One SUSPICIOUS: a far align request removes the loaded cell via the service path's differential query (not the :156 reset), and the next tick raises a false "not keeping up" ERROR |

| PR7 | `optional-paths` | fork #5 (draft) | 4 default-off-path cases (D): no_ground, MULTI_NDT, MULTI_NDT_SCORE, regularization; harness gets `publish_regularization_pose` |

**Review round 2 (2026-08-28 evening, external Claude review of the whole 42-case suite) — two
fixes, amended into the owning drafts rather than a PR8 (six-PR cap; drafts are free to amend):**
PR3 — `drive_one_scan` now emits `ADD_FAILURE() << last TF message << "(after N attempts)"`
when the `/tf_static` race outlasts its retries (verified by breaking the node's `base_link`
lookup). PR6 — the two "not retrying" cases replaced `EXPECT_FALSE(wait_until(query >= 2, 1.5 s))`
(passes with a dead timer) with the positive witness `distance_last_update_position_to_current_position
== 0.0` and `absent("is_need_rebuild")`; loader-absent 3.5 → 2.0 s. Mutation note: removing the
*incremental* branch's position recording (`map_update_module.cpp:195`, the no-op query) also fails
the Walk — its pacing relies on a no-op query recording the position. Declined from that review:
a `publish_loaded_map` case (debug topic off in every shipped yaml, rviz-only consumers; the cell
set is already guarded by `maps_size_*`, `loaded_map_` is only its debug mirror), the timer's deactivated-gate WARN (activation stays in the Node), the
`initial_pose_timeout_sec` rejection side (SmartPoseBuffer stays in the Node) — ledger only.
**Review round 3 (2026-08-28 night, 12-finder panel on the PR7 diff) — 8 of 9 findings applied,
all into PR7 (plus one PR3 re-amend):** the regularization case's pair is now stamped ±100 s
(a retry's scan stamp lags wall time by ~attempts·timeout and `interpolate` rejects a target
older than the buffer's first entry, so the old ±1 s bracket went quietly unset on any retry)
and the hook waits for both poses' diag records — published after `push_back` — before the scan
goes out; the no-ground scores gained relative pins against the full scan's (measured gaps
0.116 TP / 0.022 NVTL vs a 1e-6 floor; identical values = unfiltered cloud reached the scorer);
the two PoseArrays gained a rotation-invariant identity pin (`max_pairwise_distance`: initials
exactly 2.0, results 0.003 vs a 1.0 bound — sizes/frame could not tell a swap); SCORE's absence
assert now sits behind a second scan; `wait_for_capture_discovery` added to no_ground/MULTI_NDT;
hand-rolled waits replaced by `wait_for_diagnostics_ready(6)` / `wait_for_diag_stamp`; offsets
shared; ASSERT→EXPECT in the void hook; stimulus.hpp's stale "(100,100)" docstring reworded.
PR3: the ADD_FAILURE is tagged "(attempt N of M)" — untagged it could misattribute a later,
different failure. Mutations, each failing only its case: unfiltered cloud scored, PoseArray
publishes swapped, SCORE publishing `multi_ndt_pose`, reg key dropped. Declined/ledger: the
in-flight false-pass is bounded not eliminated (no cross-writer ordering exists); FarAlign's
timer-race (timer group ≠ align's sensor group, µs window at 1 Hz); `ready_to_align` throw
skipping counter cleanup (failure-cascade noise only); stub `covers()` square-vs-circle (all
geometry is collinear at y=100 — no divergence).
Tips: harness dd1a67da → hot-path 04ac7bf6 → align-service 243d8057 →
typical-operation e3086be7 → map-update 2125cfa2 → optional-paths 5268228f. PR6 tip 3+3 runs
clean at 39.2 s; PR7 tip 3+3 clean at ~43.4 s. Fork #1/#4/#5 bodies updated; #1405's body
(upstream) still lacks the one-line harness note — Takayuki's hand.

**Coverage table** (`extraction_coverage.md` in this session's scratchpad; also printed in the
transcript): "extraction target → guarding cases → not pinned here", 20 rows. The external review
recommends posting it as an **upstream tracking issue** (linked from each PR body, updated as a
checklist during extraction) rather than a #1348 comment — creation needs Takayuki's hand or an
explicit instruction, being an upstream write. Review submission order agreed: #1348 → merge →
rebase #1405 onto main → next; fork #2–#5 stay as drafts meanwhile.

42 cases, ~45 s (38 before PR7, ~41 s), one binary `test_ndt_scan_matcher_characteristics` (PR2 alone: 12 cases,
1.5 s). Review items A, B (both TF ERRORs), C, E (align "No InputSource", 12-key align record
+ per-particle `points_aligned`, ScopeExit on `ReliableIgnores`), F, I are done and
mutation-verified — all six ERROR sites now have a case. A non-obvious fact E2 pins: the align
path's `update_map(Point)` calls `update_map_internal` directly with no `should_update_map`, so
every align queries the loader. Still open, for PR6/H:
`out_of_map_range` reporting true with no position on record (`map_update_module.cpp:131`),
the second stub cell for the incremental clone/swap path, the failed-load retry policy, the
`update_distance` boundary, and loader-absent. Takayuki intends to extract `MapUpdateModule`
into a position → loaded-NDT class and move its double buffering to the NDT core or the Node,
which makes PR6 the safety net for that move. Every case was
reviewed line by line with Takayuki on 2026-08-27/28; the suite is considered done and the
extraction (see plan file `ndt-scan-matcher-core-cpp-ros-2-node-sparkling-manatee.md`)
starts from here. Per that plan, the test file must not change from now until Step 5 —
a needed change is evidence the refactor altered behavior.

**Pending:** #1405 stays draft until #1348 merges (upstream cannot express a stacked base;
#1405 shows #1348's commits too). #1405's DCO check will clear itself when #1348 merges.

**Findings for separate tickets (not fixed, deliberately):**
1. Off-diagonal covariance writes are transposed (`ndt_scan_matcher_core.cpp:585-586`,
   row-major index 1 gets `adj(1,0)`); harmless while every estimator is symmetric.
2. `ndt_align_srv` copies the request covariance into the response one line after assigning
   the result; the only caller (`pose_initializer`) discards it. The client still carries a
   `// Overwrite the covariance.` comment with no code. Both should go.
3. `align_pose` optimizes the TPE on `transform_probability` but picks the best particle by
   NVTL — two different objectives, no stated reason.
4. NVTL averages only over points that found a voxel, so a 90° rotation of the corner scene
   scores like the true pose; `align_pose`'s yaw is unboundable on this scene, and the same
   property holds on the vehicle.
5. The rebuild path resets `ndt_ptr` before `update_ndt` can fail (`map_update_module.cpp:156`);
   reachable only via `need_rebuild` on the timer path (map loaded → EKF jumps >50 m → "not
   keeping up" → rebuild → load fails). PR6 measured that a *far align request* loses the map by a
   different route — the service path's own differential query returns `ids_to_remove` — and
   pinned that as SUSPICIOUS instead.

**Void finding:** the plan file's "yaml comment contradicts the no_ground code" is wrong. yaml:
"`z - base_z <= threshold` → removed"; code keeps when `point_z - result_pose_z > margin`. Same
statement. Verified 2026-08-28 while designing PR7.

**PR7 (`optional-paths`, D):** no_ground enabled path, MULTI_NDT and MULTI_NDT_SCORE covariance
paths (both publish PoseArrays; SCORE publishes only `multi_initial_pose`), regularization
enabled path (sixth `/diagnostics` publisher, `regularization_pose_subscriber_status` with a single
`topic_time_stamp` key — no activation/frame validation, unlike the initial-pose subscriber).
Takayuki confirmed regularization goes into the core, so all three are extraction targets.

**External review (2026-08-28, another Claude session) — verified, mostly right.** Two errors
it found were fixed the same day: `StubMapLoader` ignored `cached_ids` (now differential, in
PR2), and two docstrings claimed the timer retries a failed load (it does not — a failed load
records the position, `map_update_module.cpp:176`). PR5's steady-state case now asserts the
realistic empty second query (`is_updated_map: False`, `maps_to_add_size: 0`) instead of a
second load. **Still open, Takayuki to decide before/while refactoring:** ~10 cheap cases the
review proposed — (A) the three WARN-and-continue checks at core.cpp:594/608/429 have no pin;
(B) 4 of 6 ERROR sites unpinned (scan TF, align TF, unknown `converged_param_type`, "not
keeping up"); (C) hot path never runs `converged_param_type=0`; (E) align "No InputSource",
per-particle `points_aligned`, align key set; (F) `skipping_publish_num` "exceed limit" WARN
asserted nowhere; (I) `delta_x > 10` rejection never driven. Declined for now: re-pinning the
covariance echo (nobody consumes it), D (MULTI_NDT/no_ground) and H (MapUpdateModule) unless
the extraction scope grows to include them. B (scan-TF ERROR) and F (skip-counter WARN) were
added to PR2 on 2026-08-28 (`ScanWithoutATransformIsAnError`,
`SkipCounterWarnsWhenItReachesTheThreshold`), mutation-verified. Same review, second pass,
noted the differential stub leaves `map_update_module.cpp:205-217` (incremental success →
clone/swap) and `removeTarget` unexercised; a second stub cell placed ~255–270 m out (so it
enters the 150 m query circle only after the first incremental update) would restore that —
part of H, pending the scope decision.

**Rebase mechanics:** when a lower branch is rewritten, each branch above it must be moved with
`git rebase --onto <new BASE tip> <old BASE tip> <branch>` — the second argument is the old tip
of the branch it sits on, NOT the branch's own old tip (`<branch-old-tip>^` is a safe way to
name it). Passing the branch's own tip moves zero commits and silently collapses it onto the
base; this happened on 2026-08-28 and was pushed before being caught (recovered from reflog).
Always check `git log --oneline <base>..<branch>` shows the expected commits before pushing.
A plain `git rebase` replays the rewritten commits and conflicts. Add `-s` to every commit —
DCO is enforced.

**PR7 review (2026-08-28, /code-review, 12 finders + verify) — resolved the same day.** 9 in-diff
findings reported and all addressed at the new tips (dd1a67da → 04ac7bf6 → 243d8057 → e3086be7 →
2125cfa2 → 5268228f; five fork branches synced, fork #1/#5 bodies updated): regularization bracket
widened to ±100 s AND both poses' diag records awaited inside the hook before the scan goes out
(SmartPoseBuffer::push_back clears-then-inserts on a backward stamp, so record ⇒ buffered holds
strictly; worst realistic retry lag ~60 s < 100 s); no-ground scores pinned relative to the
full-scan topics (measured gap TP 0.116 / NVTL 0.022, assert > 1e-6) plus the nvtl stamp; the two
MULTI_NDT arrays distinguished via max_pairwise_distance (initials exactly 2.0 — rotation-invariant
— results < 1.0, measured 0.003); SCORE's absence re-checked behind a second scan (bounds the
in-flight window; NOT eliminated — DDS gives no cross-writer ordering); discovery gates on every
capture; hand-rolled waits replaced by wait_for_diagnostics_ready(6)/wait_for_diag_stamp (the final
older_stamp check is a plain find_by_stamp — needs no wait since the hook confirmed receipt);
offsets shared as file-scope multi_offsets_x/y; hook ASSERT→EXPECT; stimulus.hpp "(100,100)"
docstring corrected. PR3 tip's ADD_FAILURE now tags "(attempt N of M)" (O1 fixed, verified by
breaking base_link lookup). Mutation-verified ×4, each failing only its own case, production
restored; PR3 25/25 (16.1 s), PR7 42/42 ×3 + 3 shuffle seeds (42.9–43.6 s).
**Still open (ledger/ticket candidates):** FarAlignRequest 1 Hz-timer race (timer can rebuild
between the align's update_map and hasTarget()); make_harness_ready_to_align throws past the
skip-counter ScopeExit; stub covers() square-on-anchor vs real loader circle-vs-box (diverges
x∈(250,270] — moot while the second-cell-at-x≥300 rule holds); wait_for_diagnostics_ready
default 5 stale for future regularization-enabled cases; docstring contradictions at
1142/1652/1695/1736; TypicalScanUnderShippedConfig's shipped 1 s/100 ms bounds are load-flaky
with no retry path.

**2026-09-03 — refactor stack extended to fork #28.** Ten more 1-commit draft PRs on `takam5f2`,
each based on the previous branch, continuing from #17 (`fix/ndt-validate-enum-params`); **#18
is an unused number** (neither issue nor PR resolves). #19 align publishes one cloud + one
marker array (search became the free function `search_initial_pose`); #20 core root
`NdtScanMatcher` owns NDT/lock/scan buffer/skip counter; #21 six node-free core unit tests;
#22 `push_*_pose` return their report; #23 `HyperParameters::to_core_params()`; #24 activation
is matcher state (`set_activated`, replaces `clear_initial_pose_buffer`); #25
`score_estimation.publish_voxel_score_points` parameter (made the topic testable — both arms
pinned); #26 `ScanInput` retired for three parameters; #27 `MapUpdateModule` friendship
dropped (`callback_timer` → `update_map_if_moved`); #28 `MapUpdateModule` ported from the
reference branch — owns its cached map, `update()` returns the NDT, `needs_update()` /
`out_of_map_range()` / `distance_from_last_update()`, conditional lock in
`NdtScanMatcher::install_map_update` (= plan 段階 B, done). **Behaviour changes only in #19,
#25, #28.** #28 resolved separate-ticket finding 5 and rewrote the two SUSPICIOUS cases that
predicted it (`FarAlignRequestLeavesTheLoadedMapIntact`, `WithoutAMapLoaderTheTimerKeepsRetrying`);
remaining flaw pinned: a loader that *answers with nothing* (NoChange) still records the
position, so a far align still causes one false "not keeping up" tick. Suite at #28 tip:
44 characterization / 8 map-update / 6 core, 0 failures. Reference branch
`refactor/ndt-map-update-module-cleanup` is now fully ported. Not moved yet to core unit
tests: the 19 scan keys and the converged/non-converged optional asymmetry (need a converging
fixture; #21's sixth case shows it is reachable).

**Residuals at close (2026-09-03):** #25 declares `score_estimation.publish_voxel_score_points`
with no default — autoware_launch's yaml must gain the key or the declaration needs a default
before upstreaming. #7/#8 are a side branch off #6, superseded by #28 (which delivers their intent
with the NoChange/Failed distinction + adopt-on-success); closing them is Takayuki's call. The
reference branch `refactor/ndt-map-update-module-cleanup` is fully ported and can go. Lock
narrowing (plan 段階 B) landed inside #28 rather than as its own PR — conditional form, but a
concurrency change the suite cannot verify, so a review focus. Separate-ticket findings 1–4 still
open; 5 resolved. Not yet in node-free tests: the 19 scan keys and the tf/pose publish asymmetry.
Offered, not done: create the `voxel_score_points` publisher only when the parameter is on.

**2026-09-04 — the straight line (`line/00`…`line/12`), on today's main.** Requested by Takayuki
because the #6–#28 history had reversals; the final version is preserved as
`refactor/ndt-final-version` (old base) and `refactor/ndt-final-version-on-main` (rebased onto
main `9ee2a5a4`, which already contains merged `#1322`; 31 commits, three conflicts resolved).
`line/00-characterization-on-main` = the 47 characterization commits rebased onto main + one
test fix (only the loader-message text changed under merged #1322). Then twelve 1-commit
branches, each green on 44 characterization + module/core/helper/covariance + 3 integration:
01 skip counter per node · 02 TPE per-instance RNG · 03 enum validation · 04 align publishes
one cloud · 05 voxel_score_points parameter · 06 core-layer foundation (#9+#12, squashed) ·
07 MapUpdateModule owns its map · 08 vendored pose buffer · 09 `search_initial_pose` ·
10 ScanMatchingModule (final interface from the start) · 11 NdtScanMatcher root ·
12 core unit tests. **`line/12` is tree-identical to `refactor/ndt-final-version-on-main`.**
Behaviour changes sit in 01–05 and 07 only; 06 and 08–12 are pure moves. Mechanics that
mattered: `git cherry-pick <range> <single>` picks the single's ancestors too — pick them
separately; main's #1322 ≠ fork #6 by ~118 lines in map_update_module.cpp (keeps
`should_update_map`, records the position on a failed load); build **both**
autoware_localization_util and autoware_ndt_scan_matcher on this line (step 02 changes the
TPE .so); run tests with `env -u CYCLONEDDS_URI` (see [[cyclonedds-rmem-after-reboot]]); clear
`build/.../test_results` before reading counts. No PRs opened for the line yet.

**2026-09-04 — PRs rebuilt.** Fork #6–#17 and #19–#28 are **closed** (comment on each points to
the new series; branches kept as history). The straight line is open as draft PRs **#29–#40**
(`line/01`…`line/12`, each based on the previous line branch; #29 on
`line/00-characterization-on-main`, which has no PR of its own). #1–#5 (characterization on the
old base) remain open and untouched. Bodies: commit message + a first line saying "Pure move"
or "Behaviour changes" + the stack note; `Signed-off-by` trailers stripped; `#1322` in code
spans. Open items unchanged: autoware_launch yaml for #33's parameter, sysctl persistence,
19-key core unit test, separate-ticket findings 1–4.

**2026-09-04 (later) — `AlignInput` retired, folded into line/11.** Takayuki: a 2-field struct
of two different types for `align()` was inconsistent with `match_scan`'s loose parameters.
Removed by amending `line/11-core-root` (#39, now 25103c1d) so the line stays reversal-free;
`line/12` (#40) rebased with `--onto <new 11> <old 11 = 3ddd0b0e>` and its three test calls
unbraced (43344a61). Both force-pushed; `refactor/ndt-final-version-on-main` moved to 43344a61
(still tree-identical to line/12). `refactor/ndt-final-version` (old base) is historical and
still has `AlignInput`. Rule confirmed: a late interface tidy goes into the step that
introduced the thing, not onto the end of the line.

