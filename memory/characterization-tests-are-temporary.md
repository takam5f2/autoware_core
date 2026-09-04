---
name: characterization-tests-are-temporary
description: "The ndt_scan_matcher characterization suite is treated as bounded-life scaffolding — don't invest in cosmetic consistency there"
metadata: 
  node_type: memory
  type: project
  originSessionId: aecf1ef5-5418-4f9f-9d7b-0286124a38ee
  modified: 2026-08-27T09:25:27.794Z
---

Takayuki treats the `autoware_ndt_scan_matcher` characterization suite
(`test/test_ndt_scan_matcher_characteristics.cpp` and `test/harness/`) as **temporary
scaffolding**. It exists to prove the ROS-free-core extraction preserves behavior; several
cases are meant to be replaced by unit tests on the extracted pure functions once that
lands.

**Why:** stated on 2026-08-27 when declining to normalize `/// @brief` usage across the
harness — 55 Doxygen tags that generate nothing (the repo has no `Doxyfile`, and
`mkdocs.yaml` reads only markdown; the package's own production headers use `/** */`).
The tags are inconsistent and buy nothing mechanically, and he still chose to leave them.

**How to apply:** for this suite, spend effort on what affects the tests' working life —
whether an assertion actually pins something, whether a failure is diagnosable, whether a
comment misleads. Do not spend it on cosmetic or stylistic uniformity, house-style
alignment, or refactoring the harness for elegance. Propose such changes only if they fall
out of work already being done. Note this is about *polish*, not rigor: he reviews the
assertions themselves closely, and wrong claims in comments get fixed. Related:
[[prefers-terse-prose]].
