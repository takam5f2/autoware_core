---
name: prefers-terse-prose
description: "Takayuki consistently asks for shorter prose — code comments, docstrings, gtest failure messages, and PR descriptions alike"
metadata: 
  node_type: memory
  type: feedback
  originSessionId: aecf1ef5-5418-4f9f-9d7b-0286124a38ee
  modified: 2026-08-28T02:26:39.597Z
---

Takayuki repeatedly asks to compress written text: code comments, test docstrings, gtest
failure messages, and PR descriptions. Over one review session this came up roughly ten
times, always in the same direction — never "explain more".

**Why:** the objection is duplication, not length as such. Text that restates the code next
to it, the function it calls, or the test name adds nothing for the reader and rots
independently of the code. Their phrasing for the rule that emerged: *the source explains,
the failure message reports.*

**How to apply:** write the short version first. Before adding an explanatory sentence, check
whether a nearby docstring, helper name, or assertion already carries it — if so, drop it or
cross-reference instead of repeating. Keep only what the reader cannot derive: measured
numbers, data the code does not contain (an index-to-field mapping, an actual diagnostic
string), and warnings that a failure means a broken precondition rather than the behavior
under test. Applies to PR descriptions too — a ~160-line body was cut down by hand.

**The tendency to watch for is generating prose, not just failing to cut it.** Late in that
same session — one spent almost entirely on compression — I proposed replacing an adequate
two-line comment with a three-line one, framed as a precision fix. Takayuki declined with
"Claude はそうやってコードに文章を増やす傾向がある". The existing text was approximately
right and misled nobody; the rewrite served my own sense of rigor. Before proposing any
comment change, check whether it makes the comment *shorter*. If it does not, the bar is that
the current text would actually mislead a reader into doing something wrong — marginal
imprecision does not clear it.

See also [[characterization-tests-are-temporary]].
