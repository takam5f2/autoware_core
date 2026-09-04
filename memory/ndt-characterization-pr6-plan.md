---
name: ndt-characterization-pr6-plan
description: "PR6 (MapUpdateModule cases, 'H') of the ndt_scan_matcher characterization stack: what it covers, the geometric constraint on the second stub cell, and what was deliberately left to unit tests"
metadata: 
  node_type: memory
  type: project
  originSessionId: d08719cc-5cdb-4e98-8987-0266772aca33
  modified: 2026-08-28T09:06:43.505Z
---

On 2026-08-28 Takayuki decided to build PR6 of the characterization stack (PR2 12 cases /
PR3 +13 / PR4 +6 / PR5 +2 = 33 at ec30ab04, 24 s) *before* sending the stack for review,
against my recommendation to pause. PR6 = my table's item **H**: MapUpdateModule as the
safety net for its extraction and the double-buffer handover. Item **D** (MULTI_NDT /
no_ground / regularization) waits on the extraction scope.

**Constraints agreed for PR6:**
- A second stub cell must sit at **x >= 300**: `SteadyState` and `OutOfMapRange` query from
  x=125 with radius 150, so anything at x <= 275 is returned to them and breaks
  `maps_to_add_size == "0"`.
- Walk steps must be > `update_distance` (20) and < `map_radius - lidar_radius` (50) to stay
  on the incremental path; 25 m steps 100 -> 275 add cell "1" at 150 and remove "0" at 275.
- `out_of_map_range` with position unset (= true) is **deliberately not pinned**: only
  observable before the first 1 Hz tick, races the timer; needs `/clock` control. Left to a
  unit test on the extracted module.
- Loader-absent case needs the first harness knob (`with_map_loader=false`).

**Why:** the stub's world is shared by every case; a cell inside an existing request circle
silently changes the map (and NVTL) for all converged cases.

**How to apply:** when reviewing PR6, check the cell position and step sizes first, then the
retry-policy case (failed load memorizes position -> no retry until 20 m), the rebuild-wipe
SUSPICIOUS case, and the `>` boundary at exactly 20.0. Related:
[[characterization-tests-are-temporary]], [[pr-1405-description-gaps]].
