---
name: ros-env-not-sourced-in-shell
description: "Claude's Bash shell on this machine has no ROS environment; colcon build/test and pre-commit need `source /opt/ros/humble/setup.bash` (and the workspace `install/setup.bash`) prefixed in the same command"
metadata: 
  node_type: memory
  type: project
  originSessionId: aecf1ef5-5418-4f9f-9d7b-0286124a38ee
  modified: 2026-08-30T23:58:37.207Z
---

`~/.bashrc` sources nothing ROS-related, so every Bash call starts without `ROS_DISTRO`,
`AMENT_PREFIX_PATH` or the ROS Python path. Symptom seen 2026-08-28: `colcon build` fails in
~1 s at CMake configure with `ModuleNotFoundError: No module named 'ament_package'` — only when
CMake has to re-run (e.g. after a branch switch changes `CMakeLists.txt`); an incremental
compile-only build can appear to work without it.

**Why:** shell state does not persist between Bash calls, so the source lines have to be part of
each command.

**How to apply:** prefix workspace commands with
`cd /home/takamine/work/autoware && source /opt/ros/humble/setup.bash && source install/setup.bash && …`.
Run `pre-commit` with the ROS setup sourced too. Shuffled gtest runs through colcon:
`GTEST_SHUFFLE=1 GTEST_RANDOM_SEED=<n> colcon test …` (the xml then carries `random_seed`).
Related: [[verify-before-claiming]] (check the build Summary before trusting a gtest xml).

Trap when building a package in a **scratch workspace** (e.g. reviewing a PR worktree) while the
main `install/setup.bash` is sourced: the main install space's stale `.so` shadows the fresh
scratch build via `LD_LIBRARY_PATH` (beats RUNPATH), so gtest binaries silently test old code.
Seen 2026-08-31 reviewing PR #1410. Prefix test runs with
`LD_LIBRARY_PATH=<scratch>/build/<pkg>:$LD_LIBRARY_PATH` and confirm with `ldd`.
