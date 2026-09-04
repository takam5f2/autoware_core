---
name: verify-before-claiming
description: "Before calling production behavior suspicious or saying a test pins something, trace the consumers across the meta-repo and run the mutation — reading the code was wrong four times in one day"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: aecf1ef5-5418-4f9f-9d7b-0286124a38ee
  modified: 2026-08-28T06:39:52.892Z
---

On 2026-08-27/28 I marked four ndt_scan_matcher behaviors SUSPICIOUS from reading the code.
Three were wrong: the response covariance nobody reads (only caller overwrites it), the
`reliable` flag that "ignores `converged_param_type`" (the align path is NVTL end to end
and the parameter is never read there), and a case named for "destroying the loaded map"
that never loaded one — deleting the destructive line kept it green. A fifth claim, "align
position stays within 1.1 m", came from seven samples in one RNG slice and failed at 3.14 m.

**Why:** Takayuki caught each by asking one question ("who reads this?", "isn't it a
threshold setting issue?", "does this test really check that?"). A wrong SUSPICIOUS marker
freezes a line that should go, and a docstring that overstates a hazard misleads the
reviewer about risk.

**How to apply:** before writing that something is suspicious, dangerous, or pinned:
(1) grep the whole meta-repo (`/home/takamine/work/autoware/src`, ~490 packages) for who
consumes the value — including launch remaps, since `ndt_scan_matcher.launch.xml` remaps
only two outputs and the rest have no subscriber; (2) mutate the production line and
confirm the case fails; (3) if quoting a measured bound, sample across RNG positions, not
one run. State only what survived. Related: [[prefers-terse-prose]].
