---
name: cyclonedds-rmem-after-reboot
description: After a reboot every node-creating test fails with "rcl node's rmw handle is invalid" — net.core.rmem_max resets and ~/.cyclonedds/cyclonedds.xml demands 20MB
metadata:
  type: project
---

On this machine `~/.cyclonedds/cyclonedds.xml` sets `<SocketReceiveBufferSize min="20MB"/>` and
`CYCLONEDDS_URI` points at it. `net.core.rmem_max` is **not persisted** in `/etc/sysctl*`, so after
a reboot it is back at 212992 and Cyclone refuses to create the domain:
`rmw_create_node: failed to create domain` → every test that constructs a node throws in
`SetUp()` with `rcl node's rmw handle is invalid`. Pure unit tests keep passing, so it looks
like a code regression in whatever was just changed. (Hit 2026-09-04 after a 12:38 reboot.)

**Why:** lost an hour to it once; the symptom pattern (unit tests green, all node tests dead,
cases that `EXPECT_THROW` at construction pass) is distinctive.

**How to apply:** check `sysctl -n net.core.rmem_max` and `uptime -s` first. The proper fix needs
root and is the user's: `sudo sysctl -w net.core.rmem_max=2147483647` (Autoware docs also set
`net.ipv4.ipfrag_time=3 net.ipv4.ipfrag_high_thresh=134217728`), or persist in `/etc/sysctl.d/`.
For running tests without root: `env -u CYCLONEDDS_URI colcon test ...` uses Cyclone's defaults
and works. See [[ros-env-not-sourced-in-shell]].
