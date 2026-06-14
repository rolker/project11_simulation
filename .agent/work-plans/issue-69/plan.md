# Plan: Wire operator station (command_bridge_sender + CAMP) into bizzyboat_massabesic_launch

## Issue

https://github.com/rolker/unh_marine_simulation/issues/69

## Context

`bizzyboat_massabesic_launch.py` brings up the boat (autonomy + nav2 + asv_sim +
mbes) but not the operator side. The boat-side command chain runs
(`command_bridge_receiver`, `helm_manager`), but the **operator-side
`command_bridge_sender` is missing**, so CAMP's `send_command` is never converted
to `command` → the boat won't change piloting mode. `simulator_launch.py`
(`:95-123`) solves this for Ben by including `sim_operator_launch.py`, which is
already generic (parameterized by `robot_namespace`) and brings up
`operator_core_launch` (the sender, in the robot ns) + `operator_ui_launch` (CAMP).

## Approach

1. **Add a `sim_operator_launch` include** to `bizzyboat_massabesic_launch.py`,
   mirroring `simulator_launch.py:103-123` but for `bizzy`, with no chart / no RViz:
   - `robot_namespace='bizzy'`, `operator_namespace='operator'`, `enable_bridge='false'`,
     `background_chart=''`, `rviz='false'`.
   - Plain `IncludeLaunchDescription` (no `GroupAction`/`ROS_DOMAIN_ID` env — that
     block in `simulator_launch` only applies when `enable_bridge` is true).
2. **Update the docstring** — drop the "operator station … not included here yet"
   note; state that it now brings up the operator station (CAMP + command sender)
   on one ROS graph, so CAMP should not also be run separately.
3. **Verify** `--show-args` loads (operator includes resolve) and the Ben path
   (`simulator_launch`) is untouched.

## Files to Change

| File | Change |
|------|--------|
| `marine_simulation/launch/bizzyboat_massabesic_launch.py` | Add `sim_operator_launch` include for `bizzy` (no chart, no RViz); update docstring |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Reuse over duplication | Reuses the existing generic `sim_operator_launch.py`; no new config or forked operator launch. |
| Mirror surrounding code | Follows `simulator_launch.py`'s operator-include pattern exactly (the established convention). |
| Do it right / complete | Closes the full command path end-to-end; acceptance includes an actual CAMP→mode-switch test. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| (none) | No | Sim launch wiring; no ADR-governed area (store/datum/etc.) touched. |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| Sim now launches CAMP | Operator should stop running CAMP separately (avoid two instances) | Yes — documented in docstring |
| `background_chart=''` passed to operator_ui/CAMP | Confirm CAMP tolerates an empty chart (else adjust) | Open Question 1 |
| Ben path | Must stay unaffected (only the bizzy entry changes) | Yes (separate file) |

## Open Questions

1. Does `operator_ui_launch` / CAMP tolerate `background_chart=''` (no chart)?
   If it errors on empty, fall back (omit the arg to use CAMP's own default, or
   point at `~/data/test_charts/massabesic_rgba.tif`). Confirm at run.

## Estimated Scope

Single small PR on `unh_marine_simulation` (`jazzy`). One file changed.
