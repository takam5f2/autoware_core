---
name: verify-checkout-before-branching
description: "the working tree can be switched to another branch between turns; always print HEAD before `git switch -c` or reading files"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: f8ba2cbf-0b89-40b9-a758-b8fee3a2e9ec
  modified: 2026-09-03T02:16:24.406Z
---

The checkout moved from the NDT refactor stack to `refact_gyrodo_step7_covariance_and_frame_check` twice in one session without any git command from me — presumably the user's IDE or another session. Once it made `git switch -c` create a branch off the wrong base; once it made a file "vanish" mid-analysis.

**Why:** the user works several branches in the same clone (NDT stack, gyro_odometer, EKF WIP in a stash). Nothing pins the checkout between turns.

**How to apply:** before `git switch -c`, before reading a file to answer a question, and before any build, run `git branch --show-current` (or `git log --oneline -1`) and compare against the branch the work belongs to. If it moved, say so to the user and switch back explicitly rather than guessing. Related: [[colcon-build-pipefail]] — a stale checkout also makes a green build meaningless.
