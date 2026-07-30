---
issue: 80
---

# Issue #80 — remove raster background-chart plumbing repo-wide (CAMP OSM base map)

## Local Review (Pre-Push)
**Status**: complete
**When**: 2026-07-30 14:21 -04:00
**By**: Claude Code Agent (Claude Fable 5)
**Verdict**: approved

**Branch**: feature/issue-80 at `4913652`
**Mode**: pre-push
**Depth**: Light (reason: 5 files, 32 pure deletions, launch-config only)
**Must-fix**: 0 | **Suggestions**: 0
**Round**: 1 | **Ship**: recommended — pure deletion verified complete; no dangling refs, no orphaned imports

### Findings
- [ ] No issues found. LGTM.

Static analysis: F401 AnyLaunchDescriptionSource in vrx operator_launch.py is pre-existing (unused before this change). Local Adversarial: off (user request). Note: sim_operator_launch no longer hard-depends on camp share dir at description time.
