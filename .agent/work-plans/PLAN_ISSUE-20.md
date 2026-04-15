# Plan: Add tidal water level, waves, and sensor noise to asv_sim and mbes_sim

## Issue

https://github.com/rolker/unh_marine_simulation/issues/20

## Context

The simulator is purely 2D — `NavSatFix.altitude` is always 0, roll/pitch are
always 0, and `mbes_sim` reads chart depths with no tide offset. This means
`sea_surface_estimator` in `mru_transform` has no signal to work with, and we
can't validate the RTK-based tide detection pipeline that BizzyBoat needs.

The issue review recommended phased PRs. This plan covers **Phase 1: tide model
and altitude propagation** — the minimum needed to get a tide signal flowing
through the full pipeline.

## Phased Approach

| Phase | Scope | PR |
|-------|-------|----|
| **1 (this plan)** | Tide model + altitude propagation + mbes_sim tide offset | First |
| 2 | Wave-driven roll/pitch/heave | Second |
| 3 | Sensor noise layer (GPS, IMU, velocity measurement noise) | Third |

Phases 2-3 will be planned after Phase 1 merges.

## Approach

### 1. Add tide model to `environment.py`

Add a `getTide(timestamp)` method that computes tide height above MLLW using
harmonic constituents. Default to Portsmouth Harbor top-3 (M2, N2, S2).

New ROS parameters:
- `environment.tide.constituents.amplitudes` (double array, meters)
- `environment.tide.constituents.speeds` (double array, degrees/hour)
- `environment.tide.constituents.phases` (double array, degrees)
- `environment.tide.ellipsoid_to_mllw` (double, default: -28.104)
- `environment.tide.speed_factor` (double, default: 1.0 — for accelerated testing)

The three arrays are parallel — index 0 of each defines one constituent. This
avoids nested parameter structures which are awkward in ROS 2.

Also add a `getEllipsoidalAltitude(timestamp)` method that returns
`ellipsoid_to_mllw + getTide(timestamp)` — this is what gets assigned to
`NavSatFix.altitude`.

### 2. Add tide topic publisher to `asv_sim_node.py`

Publish `environment/tide_level` (Float64) at the nav update rate (5 Hz).
This is the tide height above MLLW (not ellipsoidal altitude), for use by
`mbes_sim` and any other consumer that works in chart-datum space.

### 3. Propagate altitude through `dynamics.py`

Add `altitude` state variable, initialized to `ellipsoid_to_mllw`.
In `update()`, set `self.altitude = environment.getEllipsoidalAltitude(timestamp)`.
No heave yet (Phase 2).

### 4. Publish altitude in `platform.py`

In `updateNav()`, set `nsf.altitude = self.dynamics.altitude` before publishing
NavSatFix. This feeds `mru_transform` → `sea_surface_estimator`.

### 5. Subscribe to tide in `mbes_sim.py`

In `on_activate()`, subscribe to `environment/tide_level` (Float64). Store
latest value (default 0.0 if no message received — graceful degradation).

In `ping_callback()`, adjust depths: `depth_actual = depth_chart + tide_level`.
When tide is high, water is deeper; when low, shallower.

### 6. Add config defaults

Add tide parameters to an existing config or create
`asv_sim/config/environment.yaml` with Portsmouth Harbor defaults. Update
launch files to load it.

### 7. Unit test for tide model

Add `asv_sim/test/test_tide_model.py`:
- Verify `getTide()` produces expected amplitude range (~±1.3m for Portsmouth)
- Verify `speed_factor` accelerates the signal
- Verify `getEllipsoidalAltitude()` includes the ellipsoid offset

## Files to Change

| File | Change |
|------|--------|
| `asv_sim/asv_sim/environment.py` | Add tide model (getTide, getEllipsoidalAltitude, parameters) |
| `asv_sim/asv_sim/dynamics.py` | Add `altitude` state variable, set from environment |
| `asv_sim/asv_sim/platform.py` | Publish `nsf.altitude` from dynamics |
| `asv_sim/asv_sim/asv_sim_node.py` | Add tide_level publisher on nav timer |
| `mbes_sim/mbes_sim/mbes_sim.py` | Subscribe to tide_level, apply offset to depths |
| `asv_sim/config/environment.yaml` | New: Portsmouth Harbor tide defaults |
| `asv_sim/test/test_tide_model.py` | New: unit test for tide computation |
| Launch files (if needed) | Load environment.yaml |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Improve incrementally | Phase 1 only — tide model. Waves and sensor noise follow. |
| A change includes its consequences | Unit test included. Config defaults included. mbes_sim updated in same PR. |
| Test what breaks | Testing the tide math directly. Integration testing via sim launch is manual (validation section in issue). |
| Only what's needed | No wave model, no sensor noise yet — just what's needed for tide signal. |
| Workspace vs. project separation | All changes in project repo (unh_marine_simulation). No workspace changes. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Working in `feature/issue-20` worktree |
| 0008 — ROS 2 conventions | Yes | Parameters follow ROS 2 naming. Float64 topic for inter-node communication. Lifecycle pattern in mbes_sim preserved. |
| 0009 — Python packages | No | No new pip dependencies |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `environment.py` interface | `dynamics.py` caller | Yes |
| `dynamics.py` state | `platform.py` publisher | Yes |
| New topic `environment/tide_level` | `mbes_sim` subscriber | Yes |
| New parameters | Config YAML defaults | Yes |
| `.agents/README.md` | Architecture overview (tide model) | Yes — will update after implementation |

## Open Questions

1. **Topic namespace**: Should `environment/tide_level` be published under
   the node namespace (`~/environment/tide_level`) or as an absolute topic?
   The node currently publishes diagnostics under `~/`, but tide level is
   more of a shared environment state. Leaning toward `~/` for consistency
   with existing patterns — `mbes_sim` can remap.

2. **Config file organization**: New `environment.yaml` or add tide params
   to existing platform configs? Tide is environment-level (shared across
   platforms), so a separate file seems cleaner.

## Estimated Scope

Single PR. ~150-200 lines of new code across 5 files + 1 new test + 1 new config.
