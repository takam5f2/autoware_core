---
name: ndt-1322-split-stack
description: "The three fork PRs that split upstream #1322, what the characterization suite measured about it, and the B1+B2 regression that only shows in combination"
metadata: 
  node_type: memory
  type: project
  originSessionId: f8ba2cbf-0b89-40b9-a758-b8fee3a2e9ec
  modified: 2026-08-31T05:19:15.617Z
---

Built 2026-08-31. Upstream #1322 (sasakisasaki, "make map update module ROS-node free") was
measured against the characterization suite and **fails 3 of its 42 cases**. Controlled: the
suite is 42/42 on the same main (2026-08-19) without #1322, so the failures are #1322's, not
main's.

#1322 carries four behaviour changes; its "Effects on system behavior" declares only two.

| | Change | Declared |
|---|---|---|
| B1 | `need_rebuild` latched flag → `out_of_map_range(position)` | no |
| B2 | failed rebuild no longer records its position | no |
| B3 | throttled error log on map-update failure dropped | yes |
| B4 | two pcd-loader WARNs collapsed into one | yes |

**The finding worth carrying forward: B1 and B2 are each harmless alone and strand the map
together.** A far `ndt_align` request takes the rebuild path (B1), which resets `ndt_ptr_` before
loading; the load fails and records nothing (B2), so `last_update_position_` still points at the
load that succeeded. Every later tick measures 0 m against a map that is gone and skips — a
stationary vehicle runs with an empty NDT until it moves `update_distance`. Before either change
the timer measured 283 m and rebuilt. `FarAlignRequestRemovesTheLoadedCellUntilTheTimerReloadsIt`
had already named the real fix in its docstring ("load into a scratch NDT and swap only on
success"); #1322 does not implement it, and `takam5f2/refactor/ndt-map-update-module-cleanup` does.

Three stacked draft PRs on the takam5f2 fork, based on
`test/ndt-scan-matcher-characterization-optional-paths`, each 42/42 (full suite 116 tests,
0 failures):

| PR | branch | content |
|---|---|---|
| fork #6 | `refactor/ndt-map-update-ros-free-v2` | #1322's structure with B1/B2 left out; keeps B3/B4 |
| fork #7 | `fix/ndt-map-update-retry` | B2, with 2 cases renamed around the retry |
| fork #8 | `fix/ndt-map-update-rebuild-decision` | B1, with the SUSPICIOUS case rewritten around the regression |

fork #8's tree is byte-identical to #1322's head apart from one comment — that equality is the
proof the split is complete, and worth re-checking with
`git diff pr1322-head <branch> -- .../src .../include` after any rebase.

Also measured: upstream #1403 (path_generator) touches no file in this package; only main moving
forward matters there.

Next: Takayuki writes a Confluence page on the refactoring approach and takes it to sasaki.
See [[ndt-core-extraction-design-decisions]] for the extraction design these PRs clear the way for,
and [[ndt-characterization-series-state]] for the suite itself.
