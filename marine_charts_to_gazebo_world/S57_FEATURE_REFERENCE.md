# S57 Feature Reference for Gazebo World Generation

How each IHO S-57 feature type from NOAA Electronic Navigational Charts (ENCs) is
used by `marine_charts_to_gazebo_world` to generate Gazebo Harmonic SDF worlds.

Data source: NOAA ENC files in `$ROS_S57_ENC_ROOT`, read via GDAL/OGR.

Region analyzed: Portsmouth, NH area (~43.0N, -70.7W), covering charts US4NH01M,
US4NH1BD, US5PSMCC, US5PSMCD, and others at scales 1:18000 to 1:700000.

## Terrain Features (used for heightmap generation)

### DEPARE (OBJL=42) — Depth Area

| | |
|---|---|
| Count | ~1,226 polygons |
| S57 fields used | `DRVAL1` (min depth), `DRVAL2` (max depth) |
| S57 fields available but unused | `INFORM` (1 feature) |
| Gazebo representation | Interpolated into terrain heightmap (bathymetry below sea level) |
| Notes | Depth values are positive-down in S57 convention. These are the primary source for underwater terrain. |

### SOUNDG (OBJL=129) — Soundings

| | |
|---|---|
| Count | ~110 multi-point features |
| S57 fields used | Point Z values (depth) |
| S57 fields available but unused | `SCAMIN` (all), `EXPSOU` (21), `TECSOU` (1), `QUASOU` (1) |
| Gazebo representation | Additional depth samples interpolated into terrain heightmap |
| Potential improvement | `EXPSOU` (exposure of sounding) could indicate whether a sounding represents a shoal danger and should be preserved at all resolutions. |

### LNDARE (OBJL=71) — Land Area

| | |
|---|---|
| Count | ~346 features (164 polygon, 173 point, 9 line) |
| S57 fields used | Polygon geometry (constrains terrain to be above sea level) |
| S57 fields available but unused | `OBJNAM` (71 — island/land names), `INFORM` (3) |
| Gazebo representation | Used in terrain generation to constrain land elevation |

### COALNE (OBJL=30) — Coastline

| | |
|---|---|
| Count | ~454 line features |
| S57 fields used | Line geometry |
| S57 fields available but unused | `CATCOA` (75 — coastline category: 8=manmade, 3=sandy) |
| Gazebo representation | Used in terrain generation as land/water boundary |
| Potential improvement | `CATCOA` could inform terrain texture selection (sandy vs rocky vs manmade shoreline). |

## Building Features (extruded polygons)

### BUISGL (OBJL=12) — Building, Single

| | |
|---|---|
| Count | ~785 features (190 polygon, 595 point) |
| S57 fields used | Polygon geometry, `OBJNAM` (in model name) |
| S57 fields available but unused | `SCAMIN` (all — display scale) |
| Gazebo representation | Extruded polygon (`<polyline>` geometry), default height 5.0m |
| Height source | OSM height (if matched) > hardcoded 5.0m default |
| Color source | OSM material/colour (if matched) > default grey |
| Skip category | `buildings` |
| Notes | S57 provides no HEIGHT, COLOUR, NATCON, FUNCTN, or BUISHP for this region. Point features are silently dropped (no polygon to extrude). 595 of 785 features are points and produce no geometry. |
| Decision | Hardcoded height is the only option without OSM data. **Review**: should point buildings generate simple box models at a default size? Currently 76% of building features are silently ignored. |

### LNDMRK (OBJL=73) — Landmark

| | |
|---|---|
| Count | ~140 features (134 polygon, 6 point) |
| S57 fields used | Polygon geometry, `OBJNAM` |
| S57 fields available but unused | `SCAMIN` (all), `CATLND` (82 — category: 2=cemetery) |
| Gazebo representation | Extruded polygon, default height 12.0m |
| Skip category | `buildings` (grouped with BUISGL) |
| Notes | Filtered by area (<5000 m^2) to exclude large land parcels. `CATLND=2` (cemetery) appears on 82 of 140 features — these are being rendered as 12m-tall buildings, which is incorrect. |
| Decision | **Review**: Cemetery landmarks should not be extruded as tall buildings. Consider filtering by `CATLND` or using a much lower height for cemeteries. |

### SILTNK (OBJL=119) — Silo/Tank

| | |
|---|---|
| Count | ~86 features (all polygon) |
| S57 fields used | Polygon geometry, `OBJNAM` |
| S57 fields available but unused | `SCAMIN` (all), `CATSEA` (25 — sea area category) |
| Gazebo representation | Extruded polygon, default height 8.0m |
| Skip category | `buildings` (grouped with BUISGL) |
| Notes | Despite the object class name, in this dataset many SILTNK features are actually named sea areas (e.g. "Bigelow Bight", "Gulf of Maine") with `CATSEA` set. These are likely being incorrectly rendered as buildings. Only features passing the area filter (<5000 m^2) are included. |
| Decision | **Review**: SILTNK features with `CATSEA` set are sea area labels, not structures. They should be excluded from building extrusion. Verify that the area filter catches all of them. |

## Water Features

### PONTON (OBJL=95) — Pontoon

| | |
|---|---|
| Count | ~371 features (367 line, 4 polygon) |
| S57 fields used | Polygon geometry only |
| S57 fields available but unused | `SCAMIN` (367) |
| Gazebo representation | Extruded polygon at z=0 (water surface), height 1.0m |
| Skip category | `pontoons` |
| Notes | 367 of 371 features are LineStrings, which are silently dropped — only the 4 polygon features produce geometry. Similar to the BUISGL point issue: the majority of features generate nothing. |
| Decision | **Review**: Should LineString pontoons be rendered as wall segments (like SLCONS) or converted to narrow extruded rectangles? Currently 99% of pontoon features are ignored. |

### BRIDGE (OBJL=11) — Bridge

| | |
|---|---|
| Count | ~51 features (24 polygon, 27 line) |
| S57 fields used | Polygon geometry, `VERCLR` (vertical clearance, 9 features) |
| S57 fields available but unused | `CATBRG` (all — bridge category), `HORCLR` (11 — horizontal clearance), `SCAMIN` (all), `OBJNAM` (6), `CONDTN` (7 — condition: 2=ruined), `VERCCL`/`VERCOP` (closed/open clearance) |
| Gazebo representation | Extruded polygon, height from `VERCLR` or default 10.0m |
| Skip category | `bridges` |
| Notes | `CONDTN=2` (ruined) appears on 7 features — these may not need rendering. Line features (27 of 51) are dropped. `CATBRG=1` (fixed) dominates. |
| Decision | Heights from `VERCLR` are used when available. **Review**: `CONDTN=2` features could be skipped or rendered differently. Line bridges could be rendered as narrow deck spans. |

## Shore Constructions (SLCONS, OBJL=122)

| | |
|---|---|
| Count | ~457 features (435 line, 17 polygon, 5 point) |
| Total segments | ~4,869 (each becomes a separate Gazebo model) |
| S57 fields used | `CATSLC` (category), `WATLEV` (water level) |
| S57 fields available but unused | `CONDTN` (4 — condition), `OBJNAM` (1) |
| S57 fields defined but never populated | `HEIGHT`, `HORLEN`, `HORWID`, `NATCON`, `COLOUR`, `SCAMIN`, `SCAMAX` |
| Collision | Disabled for land features (`WATLEV=2`, 87% of features) |

### SLCONS Sub-types by CATSLC

| CATSLC | Name | Features | Segments | Gazebo color | Height | Width | Skip category |
|--------|------|----------|----------|-------------|--------|-------|---------------|
| 1 | Breakwater | 17 | 586 | Grey | 3.0m | 1.0m | `breakwaters` |
| 2 | Groyne | 16 | 29 | Green | 1.5m | 1.0m | `groynes` |
| 4 | Pier/Jetty | 239 | 664 | Brown | 2.0m | 1.0m | `piers` |
| 6 | Wharf/Quay | 9 | 139 | Blue | 2.5m | 1.0m | `wharves` |
| 8 | Rip Rap | 60 | 2,293 | Tan | 1.0m | 0.5m | `rip_rap` |
| 9 | Landing Steps | (0 in area) | — | Yellow | 1.5m | 0.5m | `landing_steps` |
| 10 | Sea Wall | 72 | 787 | Red | 3.0m | 1.0m | `sea_walls` |
| 12 | Ramp | 12 | 38 | Purple | 1.0m | 1.0m | `ramps` |
| 13 | Slipway | 14 | 29 | Cyan | 1.0m | 1.0m | `slipways` |
| 14 | Fender | 2 | 10 | Orange | 1.5m | 0.5m | `fenders` |
| 0 | Unknown | 16 | 121 | Default grey | 2.0m | 1.0m | — |

### SLCONS Rendering Notes

- **LineString features** (95%): Each segment between consecutive vertices becomes a
  separate oriented box model. A single feature with 26 vertices creates 25 Gazebo models.
- **Polygon features** (4%): Rendered as filled extruded polylines (solid shape).
- **Point features** (1%): Rendered as small bollard cylinders.
- **Rip rap is 47% of all segments** (2,293 of 4,869). These are irregular rocky
  shoreline features with many vertices. Skipping them via `rip_rap` category is the
  single most effective complexity reduction.
- **Closed LineStrings**: Some features (e.g. the UNH pier outline) are stored as open
  LineStrings tracing a structure's perimeter. These render as wall outlines rather than
  solid filled shapes, even when visually they represent a solid structure.
- **Decision**: Heights and widths are hardcoded per category because NOAA does not
  populate the S57 `HEIGHT`, `HORLEN`, or `HORWID` fields for this region. The values
  are reasonable approximations.

## Navigation Aid Features (point models)

### Buoys (OBJL 14-19) — BOYCAR, BOYISD, BOYLAT, BOYSAW, BOYSPP

| | |
|---|---|
| Count | ~87 point features |
| S57 fields used | Position, `COLOUR` |
| S57 fields available but unused | `BOYSHP` (all — buoy shape: 1=conical, 2=can, 4=pillar), `CATLAM` (lateral category), `OBJNAM` (all), `STATUS`, `SCAMIN`, `COLPAT` (color pattern), `CATSPM` (special purpose) |
| Gazebo representation | Cylinder body + cone top, colored by `COLOUR` attribute |
| Skip category | `buoys` |
| Decision | `BOYSHP` could drive different 3D shapes (cone vs cylinder vs pillar). Currently all buoys use the same shape. Low priority — correct color is more important than shape for simulation. |

### Beacons (OBJL 5-9) — BCNCAR, BCNISD, BCNLAT, BCNSAW, BCNSPP

| | |
|---|---|
| Count | ~17 point features |
| S57 fields used | Position, `COLOUR` |
| S57 fields available but unused | `BCNSHP` (all — beacon shape), `OBJNAM` (all), `STATUS`, `SCAMIN`, `CATLAM`/`CATSPM` |
| Gazebo representation | Tall thin cylinder, colored by `COLOUR` |
| Skip category | `beacons` |

### LIGHTS (OBJL=75) — Light

| | |
|---|---|
| Count | ~43 point features |
| S57 fields used | Position only |
| S57 fields available but unused | `COLOUR` (all — 3=red, 1=white, 4=green), `HEIGHT` (27 — tower height!), `LITCHR` (character), `SIGPER` (period), `VALNMR` (nominal range), `OBJNAM`, `SCAMIN`, `CATLIT`, sector info |
| Gazebo representation | Generic light tower model |
| Skip category | `lights` |
| Decision | **Review**: `HEIGHT` is populated for 63% of light features (values like 25.0m, 18.0m, 7.3m). This is the one feature type where NOAA provides real height data and we are not using it. `COLOUR` could also color the light. |

### PILPNT (OBJL=90) — Pile/Post

| | |
|---|---|
| Count | ~35 point features |
| S57 fields used | Position only |
| S57 fields available but unused | `SCAMIN` (all), `CATPLE` (15 — category: 3=post) |
| Gazebo representation | Small cylinder at water level |
| Skip category | `piles` |

### MORFAC (OBJL=84) — Mooring Facility

| | |
|---|---|
| Count | ~1 point feature |
| S57 fields used | Position, `CATMOR` (category: 7=mooring buoy) |
| S57 fields available but unused | `INFORM`, `SCAMIN` |
| Gazebo representation | Model varies by `CATMOR` (bollard, dolphin, buoy) |
| Skip category | `mooring_facilities` |

### CRANES (OBJL=35) — Crane

| | |
|---|---|
| Count | 0 in this region |
| S57 fields used | Position, `CATCRN`, `HEIGHT` |
| Skip category | `cranes` |

### PYLONS (OBJL=98) — Bridge Pylon

| | |
|---|---|
| Count | 0 in this region |
| S57 fields used | Position, `CATPYL`, `HEIGHT` |
| Skip category | `pylons` |

## Summary of Review Items

### High Priority

1. **LNDMRK cemeteries rendered as 12m buildings**: 82 of 140 landmarks have
   `CATLND=2` (cemetery). These should not be extruded as tall buildings.

2. **SILTNK sea areas rendered as buildings**: Features with `CATSEA` set are
   named sea areas (e.g. "Gulf of Maine"), not silos. They should be filtered out.

3. **LIGHTS height data unused**: 63% of light features have real `HEIGHT` values
   from NOAA. This is the only feature type with populated height data and we
   ignore it.

### Medium Priority

4. **Point buildings silently dropped**: 595 of 785 BUISGL features are points,
   producing no geometry. Consider generating default-size box models for them.

5. **LineString pontoons silently dropped**: 367 of 371 pontoon features are
   lines, producing no geometry. Consider rendering as wall segments or narrow
   rectangles.

6. **LineString bridges silently dropped**: 27 of 51 bridge features are lines.
   Consider rendering as narrow deck spans.

7. **LIGHTS colour unused**: All 43 lights have `COLOUR` set but it's not read.

### Low Priority

8. **Buoy shapes**: `BOYSHP` could differentiate conical, can, and pillar buoys.

9. **COALNE categories**: `CATCOA` could inform terrain texturing.

10. **Closed LineString SLCONS**: Could be detected and rendered as filled polygons
    instead of wall outlines.

11. **SCAMIN display scale**: Available on most feature types. Could be used to
    implement scale-dependent feature inclusion for different world sizes.

## Configuration Reference

Features can be excluded via the `skip_categories` list in world config YAML files.
Available skip category names:

**Whole feature types**: `buildings`, `pontoons`, `bridges`, `buoys`, `beacons`,
`lights`, `shore_constructions`, `piles`, `mooring_facilities`, `cranes`, `pylons`

**SLCONS sub-types**: `breakwaters`, `groynes`, `piers`, `wharves`, `rip_rap`,
`landing_steps`, `sea_walls`, `ramps`, `slipways`, `fenders`

Additional parameters:
- `max_wall_segments`: Cap total SLCONS line segments (default: unlimited)
- `simplify_tolerance`: Douglas-Peucker tolerance for coastline simplification
- `grid_power`: Terrain heightmap resolution (2^N + 1 per side)
