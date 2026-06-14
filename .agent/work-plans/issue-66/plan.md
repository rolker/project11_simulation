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
2. **Autonomy bringup swap (anti-drift)** — for `bizzy`, include a **sim-aware BizzyBoat core
   launch** that reuses the boat's REAL config (`nav2_overlay.yaml` + base) and layers a small new
   `bizzyboat_sim.yaml` overlay only under `is_simulator` — mirroring `ben_core_launch.py:103-110`
   (`ben.yaml` + `ben_sim.yaml`). The sim repo includes it with `is_simulator:=true`, exactly as it
   includes `ben_core_launch.py` today. **No config duplication: sim and field read one source.**
   Bizzy's current `core_launch.py` is hardware-coupled (real FCU `/dev/fcu` via mavros + sensor
   drivers), so add a thin **`bizzyboat_sim_core_launch.py`** (in `bizzyboat_project11`) that brings
   up only the sim-shareable stack (autonomy + nav2 + mru_transform + sea_surface_estimator),
   reusing the same config files. (Cross-repo change — see Scope.) Cleaner long-term: split bizzy's
   bringup into autonomy-vs-hardware like Ben's.
3. **`asv_sim/config/bizzyboat.yaml`** — mirror `ben.yaml`: `platforms.bizzy.model: echoboat240`,
   `mru_frame: bizzy/motion_sensor`, `start_lat: 42.990559`, `start_lon: -71.392958`,
   `start_heading: 353.6` (06-12 last pose; **compass-true degrees** confirmed via `platform.py:227`
   `yaw=90-heading` + `geodesic.py:24`).
3b. **`asv_sim/config/echoboat240.yaml`** (new) — `models.echoboat240.*` mirroring `cw4.yaml` with
   240 mass/length/speed + `propulsion_type: jet`. **No `dynamics.py` change needed:** the jet
   branch (`dynamics.py:229-254`) already vectors yaw — `yaw_thrust = thrust·sin(rudder)` with
   `thrust ∝ rpm` (not hull speed), so it pivots at zero forward speed (requires throttle, like the
   real vectored thrusters). Tune jet params to the empirical targets: `go_straight_coefficient`
   (yaw damping) → ~0.9-1.0 rad/s cap; `mass`+`max_power` → 0.6 m/s² launch + 1.9 m/s top;
   `drag_coefficient` → ~10 s coast-down. (The prop branch is flow-dependent and would NOT pivot —
   jet is the correct proxy, confirming Roland's call.)
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

**Repo `unh_marine_simulation`:**

| File | Change |
|------|--------|
| `marine_simulation/launch/sim_robot_launch.py` | Add `platform` arg; conditional core-launch (ben vs bizzy sim-core) + config + `platforms` namespace; MBES `grid_file` SetParameter; zero current |
| `marine_simulation/launch/simulator_launch.py` | Add/forward `platform` (+ namespace) arg |
| `marine_simulation/launch/bizzyboat_massabesic_launch.py` (new) | One-line BizzyBoat-Massabesic entry |
| `asv_sim/config/bizzyboat.yaml` (new) | `echoboat240` model + 06-12 spawn pose (353.6° compass), no current |
| `asv_sim/config/echoboat240.yaml` (new) | `models.echoboat240.*` — 240 dynamics, `propulsion_type: jet` (jet branch already vectors yaw; tune to empirical targets) |
| `<obstacle emulator>.py` (new) + launch | Seeded random obstacles → `<ns>/sea_surface/lethal_grid` within 25 m |
| `*/setup.py` | Register new node/launch/config |

**Repo `unh_echoboats_project11` (cross-repo):**

| File | Change |
|------|--------|
| `bizzyboat_project11/launch/bizzyboat_sim_core_launch.py` (new) | Sim-shareable bringup (autonomy + nav2 + transforms), reuses real `bizzyboat.yaml`/`nav2_overlay.yaml`, layers `bizzyboat_sim.yaml` under `is_simulator` — mirrors `ben_core_launch.py` |
| `bizzyboat_project11/config/bizzyboat_sim.yaml` (new) | Sim deltas only (nav from asv_sim, sim tide threshold) — mirrors `ben_sim.yaml`. **No copy of nav2_overlay.** |

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

1. **Bizzy autonomy bringup in sim** — RESOLVED (anti-drift): mirror Ben — a sim-aware bringup in
   `bizzyboat_project11` reusing the boat's real config + a small `bizzyboat_sim.yaml` overlay; the
   sim includes it with `is_simulator:=true`. Cross-repo, but the only no-duplication path. *Pending
   Roland's final OK on the cross-repo change + the thin-sim-core (ii) vs full-split (i) choice.*
2. **`start_heading` convention** — RESOLVED: compass-true degrees (`platform.py:227`,
   `geodesic.py:24`); use 353.6.
3. **Hydro model for bizzy** — RESOLVED: new `echoboat240` model, `propulsion_type: jet`. The jet
   branch already vectors yaw (pivots at zero speed) — **no `dynamics.py` extension needed**. Tune
   jet params to empirical targets derived from Friday's odom + `bizzyboat_performance.md`.

## echoboat240 model spec (empirical, for asv_sim)

Derived from Friday's odom + the boat's current-corrected dynamics doc + datasheet
(Roland: derive from bag + look up mass). Targets the asv_sim model params to reproduce:

| Quantity | Value | Source |
|---|---|---|
| Hull L × W | 2.4 m × 0.9 m | manual §1.3 (`platform.yaml`) |
| Hull mass | ~159 kg (no battery/payload; **operating mass higher** with M3/batteries/sensors — refine) | Seafloor datasheet |
| Max forward speed | **1.9 m/s** STW (current-corrected) | `bizzyboat_performance.md` (Friday SOG max 2.14 incl. current) |
| Cruise | 1.52 m/s | `bizzyboat_performance.md` |
| Hard-launch accel | ~0.6 m/s² peak, τ≈2.5 s | `bizzyboat_performance.md` |
| Coast-down decel | ~0.15 m/s², τ≈9–10 s (~10–15 m to stop) | `bizzyboat_performance.md` |
| Yaw-rate cap / pivot | 1.0 rad/s cap; **~0.9 rad/s pivot at full throttle (vectored → yaw at v≈0)** | `bizzyboat_performance.md`; Friday odom peak 1.11 rad/s |

Model build (`propulsion_type: jet` — the jet branch already vectors yaw, pivots at zero speed):
set `max_speed≈1.9`, `mass≈` operating estimate, tune `max_power`/`drag_coefficient` to hit
~0.6 m/s² launch + τ≈10 s coast, and `go_straight_coefficient` (yaw damping) so full-throttle
full-steer settles at ~0.9-1.0 rad/s. Reproduces real 240 feel, not datasheet guesses. No
`dynamics.py` change.

## Estimated Scope

Cross-repo, stacked: **PR-A** (`unh_echoboats_project11`) sim-aware bizzy core launch +
`bizzyboat_sim.yaml`; **PR-B** (`unh_marine_simulation`) platform arg + `bizzyboat.yaml` +
`echoboat240` model (jet, no dynamics.py change) + MBES grid + launch entry (Ben path unbroken);
**PR-C** (`unh_marine_simulation`) obstacle emulator + reproduction write-up. All target `jazzy`.
The Phase-4 `bathymetry_geotiff_layer` fix is a separate issue/repo.
