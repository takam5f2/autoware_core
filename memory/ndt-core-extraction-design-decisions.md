---
name: ndt-core-extraction-design-decisions
description: "Decisions Takayuki settled for extracting ROS-free core logic from the ndt_scan_matcher node, including two he reversed, and the things the characterization suite cannot catch"
metadata: 
  node_type: memory
  type: project
  originSessionId: f8ba2cbf-0b89-40b9-a758-b8fee3a2e9ec
  modified: 2026-08-31T11:07:12.440Z
---

Design settled over 2026-08-31 for splitting `autoware_ndt_scan_matcher` into a ROS layer and a
core that references **message types only**. Plan file:
`~/.claude/plans/ros-node-squishy-lightning.md`. Fork PR stack: see [[ndt-1322-split-stack]].

## Structure

- **The node holds the three core pieces as siblings** — `MapUpdateModule`,
  `ScanMatchingModule`, `PoseInitializationSearch` — and keeps `Guarded<NdtPtrType> ndt_ptr_` and
  the last scan. Takayuki considered and **set aside** a `NdtScanMatcher` core owning all three
  (has-a): it would dissolve the "how does the hot path reach the map module" question, but the
  align search holds the NDT lock while the node publishes per particle, so the core would have to
  hand out a lock-holding object or a scoped accessor. Keeping `ndt_ptr_.with([&]{...})` on the
  node avoids that entirely.
- `ScanMatchingModule` gets `MapUpdateModule &` as a **parameter** to `scan_match()`. Fall back to
  a narrow `MapCoverage` interface only if unit-test setup for `MapUpdateModule` outgrows the test.
- Node class renamed `NDTScanMatcher` → **`NdtScanMatcherNode`** (fork #12). The plan had said not
  to: measured, all 43 references are in-package. The launch file names the *executable*, and
  `autoware_launch` only includes that launch file — neither composes by plugin name.

## Two principles that came out of review, both from Takayuki's questions

- **Try letting the caller drive before injecting.** A module that seems to need a callback
  mid-algorithm often just needs to hand back a step and be asked for the next.
  `PoseInitializationSearch::next()` / `finish()` replaced a `ProgressCallback`. `PcdLoaderFunction`
  is the counter-case: its result changes what the module does next, so it stays injected.
- **Module-or-plain-object follows whether it holds state across calls.** Forcing a stateless
  operation into the "module" shape is what produced a class nested in a class.

Also: `DiagnosticsReport::check(key, value, level, message, site)` collapses the eight-line gate
pattern to a condition. Deliberately *not* done: hoisting diagnostics into a `describe(outcome)` —
six cases pin which keys are **absent** after each early return, so it would encode the control
flow a second time.

## What the characterization suite does not catch (verified by mutation)

- **Per-particle publish timing.** Collect-then-publish passes
  `SuccessfulAlignEmitsTheseKeysAndOneCloudPerParticle`, which only counts. The reason to publish
  as the search runs is the original comment ("to avoid dropping data") and `points_aligned`'s
  depth of ten.
- **Diagnostics key order.** Only the set and count are pinned. The joined *message* order is
  pinned, though, by `AligningOutsideMapRangeFailsWithThreeJoinedMessages`.
- Moving the "Lidar has gone out of the map range" WARN after the map gate would silently drop it
  in the no-map case; that case is not exercised.

## Shell hazard hit twice

`colcon build ... | tail && colcon test` lets a build failure through — the pipeline's status is
`tail`'s — and `colcon test-result` then reports the *previous* binary's results. Always
`if colcon build ...; then ...; else ...; fi`, and never read a test summary without a preceding
"BUILD OK".

See [[ndt-characterization-series-state]] for the suite this all rests on, and
[[verify-before-claiming]].
