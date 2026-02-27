# Plan: S57 ENC to Gazebo Harmonic world generator (s57_world_gen)

## Issue

https://github.com/rolker/unh_marine_simulation/issues/21

## Context

The current `vrx_project11` package launches VRX's stock `sydney_regatta` world, which
bears no relation to the UNH operating area. This issue proposes a Python tool that
generates Gazebo Harmonic SDF world files from S57 Electronic Navigational Chart data,
enabling simulation in realistic local environments.

The issue is large (7 milestones) and the review recommends breaking it into sub-issues.
**This plan covers milestones 1–2 only** (terrain heightmap + minimal world SDF) as the
MVP that can be merged and tested independently. Later milestones (nav aids, structures,
launch integration, raster support, interpolation) will be planned in follow-up issues.

### Owner feedback to address

1. **Online topo/bathy data** — The owner asked whether terrain data can be queried
   online. Yes: GEBCO (global bathymetry, 15 arc-sec) and USGS 3DEP (US land elevation,
   10m) are freely available via Python libraries (`pygmt`, `seamless-3dep`). For the MVP
   we will use **GEBCO via PyGMT** for bathymetry — it covers both ocean and land relief
   globally, requires no authentication, and avoids needing local raster files. USGS 3DEP
   can supplement land elevation in a later milestone if needed.

2. **Soundings (SOUNDG)** — S57 SOUNDG features are point data with precise depths. The
   existing C++ `marine_charts` library already identifies SOUNDG (OBJL 129). For the MVP
   interpolation, we will use SOUNDG points alongside DEPARE polygon boundaries to build
   a denser depth surface. This is a natural fit for `scipy.interpolate`.

3. **Existing Gazebo models** — VRX provides buoy models (`mb_marker_buoy_*`,
   `mb_round_buoy_*`, `surmark950400`, `polyform_a3/a7`). The general Gazebo Fuel catalog
   has limited maritime models. Model mapping is deferred to milestone 3 (nav aids).

4. **Not using `US5NH02M.tiff`** — Per owner direction, we will not rely on the local
   NOAA raster. GEBCO online data replaces this.

## Approach

### Phase 1: Package scaffolding and S57 parsing (milestone 1 prep)

1. **Create `s57_world_gen` ament_python package** — Follow `asv_sim`/`mbes_sim` patterns
   for `setup.py`, `setup.cfg`, `package.xml`, resource marker, and directory structure.

2. **Implement S57 region extraction (`s57_reader.py`)** — Use GDAL/OGR Python bindings
   (`from osgeo import ogr`) to read S57 `.000` files. Extract:
   - DEPARE polygons with DRVAL1/DRVAL2 depth band values
   - SOUNDG point features with depth values (VALSOU)
   - LNDARE polygons (land areas)
   - COALNE line features (coastlines)
   Accept bounding box (lat/lon) to clip features to the region of interest.

3. **Implement online bathymetry fetching (`bathy_fetcher.py`)** — Use PyGMT
   `load_earth_relief()` to download GEBCO data for the bounding box. Returns a numpy
   array with elevation/depth values. Cache downloaded data locally. This replaces the
   need for local GeoTIFF files.

### Phase 2: Heightmap generation (milestone 1)

4. **Implement depth surface interpolation (`terrain.py`)** — Combine data sources:
   - Start with GEBCO raster as the base surface (covers both land and sea)
   - Overlay S57 SOUNDG points where available (higher precision)
   - Use S57 DEPARE boundaries to constrain interpolation within depth bands
   - `scipy.interpolate.CloughTocher2DInterpolator` for smooth surfaces
   - Apply land mask from LNDARE polygons

5. **Implement heightmap output (`heightmap.py`)** — Convert the interpolated surface to
   a Gazebo-compatible heightmap:
   - Coordinate transform: WGS84 → local ENU frame (centered on region)
   - Resample to `(2^n + 1) × (2^n + 1)` grid (Gazebo OGRE requirement)
   - Normalize depth/elevation range to 16-bit grayscale PNG
   - Record the `<size>` (meters) and `<pos>` (offset) values for the SDF

### Phase 3: Minimal world SDF (milestone 2)

6. **Implement SDF world generation (`world_builder.py`)** — Produce a self-contained
   world file using Python string formatting (following VRX pattern):
   - `<spherical_coordinates>` set to region center (WGS84, ENU)
   - DART physics engine, 4ms step (matching VRX)
   - VRX wave system (`coast_waves` model, `WaveVisual` plugin, wave `PublisherPlugin`)
   - VRX wind plugin (`USVWind`)
   - Standard Gazebo system plugins (physics, sensors, navsat, scene broadcaster, etc.)
   - Terrain model include (referencing the generated heightmap)
   - Scene configuration (sky, lighting)

7. **Implement CLI entry point (`generate_world.py`)** — Console script registered in
   `setup.py` as `ros2 run s57_world_gen generate_world`:
   ```
   generate_world --enc-root <path> --bounds <s,w,n,e> --output-dir <path> --world-name <name>
   ```
   Optional: `--resolution <meters>` (default 1.0), `--heightmap-size <n>` (default 9 for
   513×513). The `--bathy-raster` flag is deferred to milestone 6.

8. **Generate terrain model structure** — Output directory:
   ```
   output_dir/
   ├── <world_name>.sdf
   ├── models/terrain/
   │   ├── model.sdf
   │   ├── model.config
   │   └── heightmap.png
   └── README.md  (generation metadata)
   ```

### Phase 4: Validation and tests

9. **Add unit tests** — In `test/`:
   - `test_s57_reader.py` — S57 feature extraction with mock OGR data
   - `test_terrain.py` — Interpolation produces correct grid dimensions and value range
   - `test_heightmap.py` — Output PNG is correct dimensions, 16-bit, values in range
   - `test_world_builder.py` — Generated SDF is well-formed XML with required elements

10. **Validate with `gz sdf --check`** — If available in the environment, run the
    generated SDF through Gazebo's validator.

11. **Manual test** — Generate a world for Portsmouth Harbor (bounds
    `43.065,-70.72,43.085,-70.70`) and verify terrain looks reasonable.

## Files to Change

| File | Change |
|------|--------|
| `s57_world_gen/package.xml` | **New** — ament_python package with deps: rclpy, python3-gdal, python3-numpy, python3-scipy, python3-pygmt |
| `s57_world_gen/setup.py` | **New** — setuptools config with console_scripts entry point |
| `s57_world_gen/setup.cfg` | **New** — script directories |
| `s57_world_gen/resource/s57_world_gen` | **New** — empty ament index marker |
| `s57_world_gen/s57_world_gen/__init__.py` | **New** — package init |
| `s57_world_gen/s57_world_gen/s57_reader.py` | **New** — S57 ENC parsing via GDAL/OGR |
| `s57_world_gen/s57_world_gen/bathy_fetcher.py` | **New** — GEBCO data fetching via PyGMT |
| `s57_world_gen/s57_world_gen/terrain.py` | **New** — Depth surface interpolation |
| `s57_world_gen/s57_world_gen/heightmap.py` | **New** — Heightmap PNG generation |
| `s57_world_gen/s57_world_gen/world_builder.py` | **New** — SDF world file generation |
| `s57_world_gen/s57_world_gen/generate_world.py` | **New** — CLI entry point |
| `s57_world_gen/test/test_s57_reader.py` | **New** — Unit tests |
| `s57_world_gen/test/test_terrain.py` | **New** — Unit tests |
| `s57_world_gen/test/test_heightmap.py` | **New** — Unit tests |
| `s57_world_gen/test/test_world_builder.py` | **New** — Unit tests |

## Principles Self-Check

| Principle | Consideration |
|---|---|
| Only what's needed | Plan covers milestones 1–2 only — the MVP terrain + world. Nav aids, structures, and launch integration are deferred to sub-issues. GEBCO fetching replaces manual raster management. |
| Improve incrementally | Single PR produces a usable standalone tool. Generated world can be tested with `gz sim` immediately. Integration with `vrx_project11` launch is a separate follow-up. |
| Test what breaks | Unit tests for interpolation, heightmap dimensions, and SDF validity. These catch the regressions that matter (corrupt heightmaps, invalid SDF, wrong coordinate transforms). |
| A change includes its consequences | New package includes its own tests and a README with generation metadata. No existing packages are modified in this MVP. |
| Capture decisions, not just implementations | Key design decisions (heightmap vs. mesh, GEBCO over local rasters, CloughTocher interpolation) are documented in this plan and will be recorded in code comments and the output README. |
| Workspace vs. project separation | All changes are in the project repo (`unh_marine_simulation`). No workspace changes needed. |

## ADR Compliance

| ADR | Triggered | How addressed |
|---|---|---|
| 0002 — Worktree isolation | Yes | Working in `feature/issue-21` worktree |
| 0003 — Project-agnostic workspace | No | This is project repo work |
| 0001 — Adopt ADRs | Watch | If `unh_marine_simulation` adopts ADRs, the heightmap format choice could warrant one. Not blocking for MVP. |
| 0004–0006 | No | Not triggered |

## Consequences

| If we change... | Also update... | Included in plan? |
|---|---|---|
| Add new package to repo | `README.md` at repo root (package list) | Yes — brief mention of new package |
| Add `python3-pygmt` dependency | Verify `pygmt` is installable via pip/apt in CI | Yes — test in dev environment first |
| Add `python3-scipy` dependency | Already available via `python3-scipy` apt package | Yes — declared in `package.xml` |

## Open Questions

1. **PyGMT availability** — Is `pygmt` installable in the current ROS 2 environment?
   If not, we could fall back to direct GEBCO NetCDF download via `xarray` + `rioxarray`,
   or use the Open Topo Data REST API (no library needed). **Recommend testing before
   committing to the dependency.**

2. **Sub-issues** — Should milestones 3–7 each get their own GitHub issue now, or should
   we wait until the MVP (milestones 1–2) is merged? The review recommended breaking
   the issue up — this plan already scopes to 1–2 only.

3. **ENC data for testing** — Do we have S57 `.000` files for Portsmouth Harbor
   available locally, or should the test use GEBCO-only mode without S57 overlay?

4. **VRX wave/wind plugin compatibility** — The VRX plugins (`WaveVisual`,
   `PublisherPlugin`, `USVWind`) need to be verified as available in the current
   Gazebo Harmonic installation. If not, the minimal world can omit them initially.

## Estimated Scope

Single PR for milestones 1–2. Approximately 6–8 source files + 4 test files.
Follow-up PRs for milestones 3–7 via separate sub-issues.

---
**Authored-By**: `Claude Code Agent`
**Model**: `Claude Opus 4.6`
