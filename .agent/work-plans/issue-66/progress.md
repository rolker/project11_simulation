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
