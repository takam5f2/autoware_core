---
name: pr-1405-description-gaps
description: "Four things deliberately left out of PR #1405's description on 2026-08-27, to add only if a reviewer trips on them"
metadata: 
  node_type: memory
  type: project
  originSessionId: aecf1ef5-5418-4f9f-9d7b-0286124a38ee
  modified: 2026-08-27T00:30:02.848Z
---

PR #1405 (autowarefoundation/autoware_core, draft, "pin the converged scan-matching hot
path") had its description cut from ~172 to 121 lines by hand on 2026-08-27. Four items were
identified as costing something but **deliberately not restored** — hold unless a reviewer
actually trips on one.

Ranked by value, with the reason each might matter:

1. **Why the diff is large.** The PR shows +2301/−3 over 33 commits because upstream cannot
   express a stacked base — #1348's branch lives only in the fork `takam5f2`, so #1405 targets
   `main` and carries #1348's commits too. Risk: a reviewer reads it as an oversized PR.
2. **Run the binary through ctest, not directly.** `ament_add_ros_isolated_gtest` supplies an
   unused `ROS_DOMAIN_ID`; run directly it shares a domain and up to sixteen assertions fail
   per process, each looking like a node defect. Risk: a reviewer verifying "`--gtest_shuffle`
   passes" hits this.
3. **The mutation sweep.** 18 mutations, each failing the case named for it and only that one;
   three coverage holes were found this way and closed. Risk: "how do you know these tests
   catch a regression?" has no answer in the body.
4. **`scale_factor = 1.0e6`, six orders above shipped.** `adjust_diagonal_covariance` floors
   the estimated diagonal at the parameter variance, so at the shipped scale "written" and
   "never written" are indistinguishable from outside the node. Note the surviving phrase
   "about −0.04 after the scale" now dangles without this.

What **was** kept and must stay: the transpose finding and its verification ask
(`ndt_scan_matcher_core.cpp:585-586` writes `adj(1,0)` into row-major index 1 = element (0,1),
and `adj(0,1)` into index 6 = (1,0)). That is the only item in the PR needing another
person's eyes.

Cutting was the right call given [[prefers-terse-prose]] — this is a hold list, not a regret.
