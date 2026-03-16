# Plan: Add wave/buoyancy support to generated world SDFs

## Issue

https://github.com/rolker/unh_marine_simulation/issues/48

## Context

Generated world SDFs (harbor, pier, isles of shoals, etc.) lack VRX wave visuals
and wavefield parameters. BEN's `PolyhedraBuoyancyDrag` plugin subscribes to
`/vrx/wavefield/parameters`, but nothing publishes that topic in the generated
worlds. Users currently must manually patch the installed SDF after build.

VRX packages (`vrx_gz`, `vrx_gazebo`) are already in the underlay. The
`coast_waves` model provides Gerstner wave visuals via `libWaveVisual.so`, and
`libPublisherPlugin.so` publishes wavefield parameters (direction, gain, period,
steepness) on a configurable topic.

The world generation pipeline is:
1. YAML config (`portsmouth_nh_gazebo/config/*.yaml`)
2. `build_world.py` reads YAML → builds CLI args
3. `generate_world.py` orchestrates terrain/features/SDF generation
4. `world_builder.py` fills `world_template.sdf.xml` placeholders

## Approach

1. **Add `{wave_models}` placeholder to `world_template.sdf.xml`** — Insert a new
   placeholder between the water plane and feature models sections. This follows the
   existing pattern (`{terrain_includes}`, `{s57_feature_models}`, etc.).

2. **Add wave SDF generation to `world_builder.py`** — New function
   `_build_wave_sdf(wave_config)` that generates:
   - A `coast_waves` model `<include>` for VRX Gerstner wave visuals
   - A `vrx::PublisherPlugin` world plugin publishing wavefield parameters

   Parameters (from `wave_config` dict, with defaults):
   - `gain` (default: `0.3`) — wave amplitude multiplier
   - `period` (default: `5.0`) — wave period in seconds
   - `direction` (default: `0.0`) — wave direction in degrees
   - `steepness` (default: `0.0`) — Gerstner wave steepness
   - `topic` (default: `/vrx/wavefield/parameters`) — parameter topic

   Update `generate_world_sdf()` to accept a `wave_config` parameter and pass the
   generated SDF to the template.

3. **Add `--waves` CLI flag to `generate_world.py`** — A simple boolean flag that
   enables wave generation with defaults. Individual wave parameters are not exposed
   as CLI args (YAML config is the primary interface for per-world customization).
   Pass `wave_config` through to `generate_world_sdf()`.

4. **Add `waves` YAML config support to `build_world.py`** — Read optional `waves`
   key from the YAML config. When present (either `waves: true` for defaults or a
   mapping with parameter overrides), pass `--waves` to generate_world and propagate
   parameter values.

   YAML examples:
   ```yaml
   # Enable with defaults
   waves: true

   # Enable with custom parameters
   waves:
     gain: 0.5
     period: 8.0
     direction: 240.0
   ```

5. **Enable waves in all four Portsmouth NH configs** — Add `waves: true` to
   `portsmouth.yaml`, `isles_of_shoals.yaml`, `portsmouth_to_isles.yaml`, and
   `unh_pier.yaml`. Default parameters are appropriate for coastal New Hampshire.

6. **Add test for wave SDF generation** — Extend `test_world_builder.py`:
   - Test that wave config produces `coast_waves` include and PublisherPlugin
   - Test that no wave content appears when wave_config is None (default)
   - Test that custom parameters are reflected in the generated SDF

## Files to Change

| File | Change |
|------|--------|
| `marine_charts_to_gazebo_world/marine_charts_to_gazebo_world/world_template.sdf.xml` | Add `{wave_models}` placeholder |
| `marine_charts_to_gazebo_world/marine_charts_to_gazebo_world/world_builder.py` | Add `_build_wave_sdf()`, update `generate_world_sdf()` signature |
| `marine_charts_to_gazebo_world/marine_charts_to_gazebo_world/generate_world.py` | Add `--waves` flag, pass wave_config to builder |
| `portsmouth_nh_gazebo/scripts/build_world.py` | Read `waves` YAML config, pass `--waves` flag |
| `portsmouth_nh_gazebo/config/portsmouth.yaml` | Add `waves: true` |
| `portsmouth_nh_gazebo/config/isles_of_shoals.yaml` | Add `waves: true` |
| `portsmouth_nh_gazebo/config/portsmouth_to_isles.yaml` | Add `waves: true` |
| `portsmouth_nh_gazebo/config/unh_pier.yaml` | Add `waves: true` |
| `marine_charts_to_gazebo_world/test/test_world_builder.py` | Add wave SDF tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| A change includes its consequences | Tests added for wave SDF generation. YAML configs updated alongside template changes. |
| Only what's needed | Minimal addition: one template placeholder, one builder function, one CLI flag, one YAML key. No over-engineering of wave parameter CLI exposure. |
| Improve incrementally | Single PR adds wave support without restructuring the generation pipeline. |
| Test what breaks | Tests verify wave content presence/absence and parameter propagation — the regression-prone parts. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| ADR-0002 (Worktree isolation) | Yes | Working in `feature/issue-48` worktree |
| ADR-0008 (ROS 2 conventions) | Watch | YAML config keys follow existing patterns (lowercase, underscores). No new ROS interfaces. |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| `world_template.sdf.xml` | `world_builder.py` (new placeholder) | Yes |
| `generate_world_sdf()` signature | All callers (`generate_world.py`) | Yes |
| `generate_world.py` CLI args | `build_world.py` (YAML-to-CLI bridge) | Yes |
| YAML config schema | `.agents/README.md` (if it documents config format) | No — minor, not currently documented there |

## Open Questions

- **Replace or augment the flat water plane?** The `coast_waves` model provides its
  own wave mesh visual. Should the existing `water_plane` model be removed when waves
  are enabled, or kept as a fallback? The VRX wave mesh may not cover the full world
  extent for large worlds. Recommend: keep both; the wave mesh overlays the flat plane.

## Estimated Scope

Single PR. All changes are in two packages within the same repo.
