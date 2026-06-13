# Plan: Sim — BizzyBoat at Lake Massabesic (non-Gazebo), reproduce local-costmap-only planning

## Issue

https://github.com/rolker/unh_marine_simulation/issues/66

## Context

The `simulator_launch.py` → `sim_robot_launch.py` stack (`asv_sim` dynamics + `mbes_sim`
reading a bathy GeoTIFF + the boat autonomy/Nav2 stack; **no Gazebo**) is hardcoded to Ben
(`sim_robot_launch.py:88-102` includes `ben_core_launch.py`; `:180` sets `platforms: ['ben']`;
`:150-161` loads `cw4.yaml` + `ben.yaml`). We want a **BizzyBoat at Lake Massabesic** variant
that mirrors the boat's *current* field setup so Friday's "planner won't plan beyond the local
costmap" bug reproduces (hypothesis: at a chart-less lake the global costmap has no spatial data
beyond the ~100 m segmentation bubble). Then an obstacle emulator feeds the real
`SeaSurfaceRelayLayer` to recreate local-only perception.

The `drix` flag is the existing alternate-platform precedent; the MBES + cube groups
(`:218-357`) are already namespace-driven, so they work for `bizzy` unchanged.

## Approach

1. **Add a `platform` arg** (`ben` default, `bizzy`) to `sim_robot_launch.py` + pass-through from
   `simulator_launch.py` (which currently hardcodes `namespace='ben'` at its include). Drive the
   conditional swaps below off it, mirroring the `drix` pattern.
2. **Autonomy bringup swap** — for `bizzy`, bring up BizzyBoat's Nav2 stack (model 240 +
   `bizzyboat_project11/config/nav2_overlay.yaml`) with `use_sim_time` instead of
   `ben_core_launch.py`. Exact mechanism = **Open Question 1**.
3. **`asv_sim/config/bizzyboat.yaml`** — mirror `ben.yaml`: `platforms.bizzy.model`,
   `mru_frame: bizzy/motion_sensor`, `start_lat: 42.990559`, `start_lon: -71.392958`,
   `start_heading: <353.6 or ENU-equiv>` (06-12 last pose). Confirm `start_heading` convention
   against `asv_sim` (compass vs ENU-yaw) — **Open Question 2**.
4. **`platforms` + config selection** — replace hardcoded `['ben']` (`:180`) with the namespace;
   load `bizzyboat.yaml` (+ model config) under `IfCondition(platform==bizzy)`.
5. **No current** — set `asv_sim` environment current to 0 for the bizzy path (verify param name
   in `asv_sim` environment config).
6. **MBES ground truth** — `SetParameter('grid_file', <massabesic_contour.tiff>)` in the MBES
   group (alongside the existing `sonar_frame_id`/`ping_interval` SetParameters at `:220-229`).
   Use the contour GeoTIFF we already have; full-truth composite is a later swap (cube#43).
7. **Obstacle emulator node** (`mbes_sim` or a new small `marine_simulation` node): scatter `N`
   seeded random obstacles over a configurable lake bbox, track boat pose via TF
   (`<ns>/map_tide`→`<ns>/base_link`), publish `nav_msgs/OccupancyGrid` on
   `<ns>/sea_surface/lethal_grid` (frame `<ns>/map_tide`, lethal=100) containing **only**
   obstacles within `reveal_radius` (default 25 m). Params: `seed`, `obstacle_count` (15),
   `reveal_radius` (25.0), `bbox`. Feeds the real `SeaSurfaceRelayLayer` unchanged.
8. **Bizzy launch entry** — `bizzyboat_massabesic_launch.py` (thin wrapper over
   `simulator_launch`/`sim_robot_launch` with `platform:=bizzy`, obstacle emulator on) for one-line
   invocation.
9. **Reproduce** — command a goal beyond the local window; confirm the planner cannot route to it.
   Document the observed behavior in progress.md as the baseline.

## Files to Change

| File | Change |
|------|--------|
| `marine_simulation/launch/sim_robot_launch.py` | Add `platform` arg; conditional core-launch + config + `platforms` namespace; MBES `grid_file` SetParameter; zero current |
| `marine_simulation/launch/simulator_launch.py` | Add/forward `platform` (+ namespace) arg |
| `marine_simulation/launch/bizzyboat_massabesic_launch.py` (new) | One-line BizzyBoat-Massabesic entry |
| `asv_sim/config/bizzyboat.yaml` (new) | Platform model + 06-12 spawn pose, no current |
| `<obstacle emulator>.py` (new) + launch | Seeded random obstacles → `<ns>/sea_surface/lethal_grid` within 25 m |
| `*/setup.py` / `CMakeLists.txt` | Register new node/launch/config |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Robustness / do it right | Emulator is **seeded/deterministic** (reproducible runs); reuses the *real* relay layer so the data path matches the boat — not a parallel fake costmap path. |
| Reproduce before fixing | This issue deliberately recreates the field bug as a baseline before the Phase-4 fix; no premature fix bundled in. |
| Verify, don't assume | The root-cause is a *hypothesis* to confirm in sim, not asserted as fact. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 (bathy store) | No | Bathy→costmap layer (the fix) is deferred; this issue is sim infra only. |
| (sim repo has no local ADR set) | — | Follows workspace Quality Standard + existing `drix` launch convention. |

## Consequences

| If we change... | Also update... | Included? |
|---|---|---|
| `sim_robot_launch.py` platform branching | Ben path must stay default + unbroken | Yes (regression-check Ben launch) |
| New asv_sim platform | needs a hydro model (reuse `cw4` initially) | Yes (OQ3) |
| Obstacle emulator topic/frame | must match `SeaSurfaceRelayLayer` contract (`OccupancyGrid`, `<ns>/sea_surface/lethal_grid`) | Yes |

## Open Questions

1. **Bizzy autonomy bringup in sim** — cleanest path to run BizzyBoat's Nav2 (model 240 +
   `nav2_overlay.yaml`) under sim_time: (a) new sim-aware bringup *in the sim repo* pointing
   `nav2_bringup` at bizzy's params (keeps changes one-repo, avoids real-hardware drivers), or
   (b) add an `is_simulator` path to `bizzyboat_project11` (cross-repo). Lean (a) — the bug is
   pure Nav2, no hardware needed. Confirm with Roland.
2. **`start_heading` convention** in `asv_sim` (compass-true vs ENU-yaw) — set 353.6 or -6.5 accordingly.
3. **Hydro model for bizzy** — reuse `cw4` for now (planner bug is model-independent) or add an
   echoboat-240 model config? Proposed: reuse `cw4`, follow-up for a real model.

## Estimated Scope

Two PRs on a stacked branch: **PR-A** scaffolding (platform arg + bizzy config + MBES grid +
launch entry, Ben path unbroken); **PR-B** obstacle emulator + the reproduction write-up. Both
target `jazzy`. The Phase-4 `bathymetry_geotiff_layer` fix is a separate issue/repo.
