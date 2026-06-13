---
issue: 66
---

# Issue #66 — Sim: BizzyBoat at Lake Massabesic (non-Gazebo) — reproduce local-costmap-only planning

## Plan Authored
**Status**: complete
**When**: 2026-06-13 11:40 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))

**Plan**: `.agent/work-plans/issue-66/plan.md` at `92e9e68`
**PR**: https://github.com/rolker/unh_marine_simulation/pull/67 (`[PLAN]` prefix)
**Phases**: 2 (PR-A scaffolding; PR-B obstacle emulator + reproduction write-up)

### Open questions
- [ ] Bizzy autonomy bringup in sim: (a) sim-repo-only nav2_bringup at bizzy params [lean], or (b) cross-repo is_simulator path in bizzyboat_project11
- [ ] `asv_sim` `start_heading` convention (compass-true 353.6 vs ENU-yaw -6.5)
- [ ] Hydro model for bizzy: reuse `cw4` now vs add echoboat-240 model

## Local Review
**Status**: complete
**When**: 2026-06-13 14:05 -0400
**By**: Claude Code Agent (Claude Opus 4.8 (1M context))
**Verdict**: approved (after fixes)

**PR**: #67 at `5c9c48a`
**Mode**: post-PR
**Depth**: Standard (reason: shared launch wiring + new model/config)
**Must-fix**: 2 | **Suggestions**: 1

### Findings
- [x] (must-fix) `{'platforms': [namespace]}` collapses to a scalar string → asv_sim STRING_ARRAY crash for ALL platforms incl. ben regression; fixed via OpaqueFunction resolving namespace to a plain string — `marine_simulation/launch/sim_robot_launch.py`
- [x] (must-fix) echoboat240 `clutch_engagement_rpm: 0.0` fed idle thrust → ~1.2 m/s uncommanded creep; engage above idle (800) — `asv_sim/config/echoboat240.yaml`
- [x] (doc) default MBES grid is Portsmouth (~100 km away) → zero ground truth at Massabesic; clarified — `marine_simulation/launch/bizzyboat_massabesic_launch.py`
- [ ] (suggestion, follow-up) `mbes_sim/bathy_grid.py` out-of-bounds guard relies on IndexError; negative indices silently wrap → fabricated depth. Pre-existing in mbes_sim (not this diff); harden with explicit bounds check.

### Cross-pass note
Lens A and Lens B disagreed on the platforms-param finding; resolved by empirical test (launch_ros `normalize_parameters`) — Lens A correct.
