---
name: colcon-build-pipefail
description: colcon build piped to tail/grep hides failures — always set -o pipefail before the guard
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f8ba2cbf-0b89-40b9-a758-b8fee3a2e9ec
  modified: 2026-09-01T04:35:44.010Z
---

`if colcon build ... 2>&1 | tail -20; then` **always succeeds**: a shell pipeline's exit status is the last command's, so `tail`/`grep` masks a failed build. `colcon test-result` then reports the *previous* binary's results.

**Why:** this produced three separate false "all green" reports across the NDT refactor series. The first two were blamed on missing a build guard; adding the guard did not help, because the pipe was the actual defect.

**How to apply:** write `set -o pipefail && if colcon build ... | tail; then ... else echo FAILED; fi`. Then cross-check the test summary against the test *names* that ran (`grep -o 'name="..." status="run"' on the gtest xml) — a renamed or newly added case appearing under its old name is proof you are reading a stale binary. See [[verify-before-claiming]] and [[ros-env-not-sourced-in-shell]].
