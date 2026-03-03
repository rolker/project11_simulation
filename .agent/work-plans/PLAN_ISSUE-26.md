# Plan: S57 Feature Placement in World Generator (#26)

## Context

Issue #26 adds S57 chart feature placement to `marine_charts_to_gazebo_world` so generated
Gazebo worlds include buildings, bridges, pontoons, buoys, beacons, and lights extracted
from ENC data. PR #22 (MVP terrain + water surface) is now merged. This is the next major
enhancement — milestones 3–4 from the original #21 scope.

All models will be parametric SDF (self-contained, no external mesh/Fuel dependencies).
Fuel model support can be added later as an optional enhancement.

## Key Design Decisions

1. **Polygon features → SDF `<polyline>` extrusion** — SDF 1.9 natively supports `<polyline>`
   geometry with `<height>` for vertical extrusion. Perfect for building footprints, pontoons,
   and bridges without triangulation or mesh generation.

2. **Point features → parametric SDF primitives** — Buoys as colored cylinders + cones,
   beacons as cylinders + boxes, lights as thin cylinders (poles). All inline SDF, no files.

3. **Coordinate placement → local ENU offsets** — Gazebo uses Cartesian coordinates internally.
   Convert feature lat/lon to meters offset from the world center using the existing
   `_METERS_PER_DEG_LAT` / `_meters_per_deg_lon()` pattern from `terrain.py`.

4. **Feature models inserted into SDF template** — Add a `{feature_models}` placeholder in
   `world_template.sdf.xml`. The world builder populates it with generated `<model>` elements.

## Files to Modify

| File | Change |
|------|--------|
| `s57_reader.py` | Add new dataclasses + extract 9 OBJL feature types |
| `feature_models.py` (new) | Convert S57 features → SDF `<model>` XML strings |
| `world_builder.py` | Accept features, pass to template |
| `world_template.sdf.xml` | Add `{feature_models}` placeholder |
| `generate_world.py` | Add `--no-features` flag, pass features through pipeline |
| `test/test_s57_reader.py` | Tests for new feature extraction |
| `test/test_feature_models.py` (new) | Tests for SDF model generation |
| `test/test_world_builder.py` | Update for feature_models parameter |

All paths under `marine_charts_to_gazebo_world/marine_charts_to_gazebo_world/`.

## Implementation Steps

### 1. Extend S57 reader with new feature types

**New dataclasses:**

```python
@dataclass
class Building:
    geometry: ogr.Geometry  # Polygon footprint
    # S57 doesn't reliably provide height; use default

@dataclass
class Pontoon:
    geometry: ogr.Geometry  # Polygon footprint

@dataclass
class Bridge:
    geometry: ogr.Geometry  # Polygon footprint
    clearance: float = 0.0  # VERCLR vertical clearance (meters)

@dataclass
class Buoy:
    lat: float
    lon: float
    colour: int = 0  # S57 COLOUR attribute (1=white, 3=red, 4=green, 6=yellow)
    category: int = 0  # CATCAM/CATLAM for shape selection

@dataclass
class Beacon:
    lat: float
    lon: float

@dataclass
class Light:
    lat: float
    lon: float
```

**Update `S57Features`** — add fields: `buildings`, `pontoons`, `bridges`, `buoys`,
`beacons`, `lights`.

**Add OBJL extraction** in `read_s57_file()`:
- OBJL 12 (BUISGL) → `buildings`
- OBJL 95 (PONTON) → `pontoons`
- OBJL 11 (BRIDGE) → `bridges` (extract VERCLR attribute)
- OBJL 73 (LNDMRK) → `buildings` (treat as tall buildings)
- OBJL 119 (SILTNK) → `buildings` (treat as buildings)
- OBJL 17 (BOYLAT) → `buoys` (extract COLOUR attribute)
- OBJL 75 (LIGHTS) → `lights`
- OBJL 9 (BCNSPP) → `beacons`

Landmarks and silos go into the `buildings` list since they share the same polygon
extrusion approach — just with different default heights set in `feature_models.py`.

### 2. Create `feature_models.py` — SDF model generation

**Core function:** `generate_feature_models(features: S57Features, center_lat, center_lon) -> str`

Returns a string of SDF `<model>` elements ready to insert into the template.

**Coordinate conversion** (reuse `terrain.py` pattern):
```python
def _latlon_to_enu(lat, lon, center_lat, center_lon):
    """Convert WGS84 lat/lon to local ENU meters offset from world center."""
    dx = (lon - center_lon) * _meters_per_deg_lon(center_lat)
    dy = (lat - center_lat) * _METERS_PER_DEG_LAT
    return dx, dy
```

**Polygon extrusion** (buildings, pontoons, bridges):
```xml
<model name="building_0001">
  <static>true</static>
  <pose>123.4 -56.7 0 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <polyline>
          <height>5.0</height>
          <point>0.0 0.0</point>
          <point>10.0 0.0</point>
          <point>10.0 8.0</point>
          <point>0.0 8.0</point>
        </polyline>
      </geometry>
      <material>
        <ambient>0.6 0.6 0.55 1.0</ambient>
        <diffuse>0.7 0.7 0.65 1.0</diffuse>
      </material>
    </visual>
  </link>
</model>
```

Polygon vertex coordinates are converted from WGS84 to local offsets relative to the
model's pose (which is the polygon centroid in ENU). The model's pose places the centroid
in the world; the polyline points are relative to that pose.

**Default heights:**
- Buildings (BUISGL): 5.0m
- Pontoons (PONTON): 1.0m (at water level)
- Bridges (BRIDGE): use VERCLR if available, else 10.0m
- Landmarks (LNDMRK): 12.0m
- Silos/tanks (SILTNK): 8.0m

**Point models** (buoys, beacons, lights):
- Buoy: cylinder (r=0.4, h=0.8) + cone (r=0.2, h=0.4) on top, colored by S57 COLOUR
- Beacon: cylinder (r=0.15, h=3.0), gray
- Light: thin cylinder (r=0.1, h=5.0), dark gray with small yellow sphere on top

**S57 COLOUR mapping:**
```python
_S57_COLOURS = {
    1: (1.0, 1.0, 1.0),   # white
    2: (0.1, 0.1, 0.1),   # black
    3: (1.0, 0.0, 0.0),   # red
    4: (0.0, 0.8, 0.0),   # green
    5: (0.0, 0.0, 1.0),   # blue
    6: (1.0, 1.0, 0.0),   # yellow
}
```

### 3. Update world builder and template

**`world_template.sdf.xml`** — add `{feature_models}` placeholder before closing `</world>`:
```xml
    <!-- Water surface at z=0 -->
    ...

{feature_models}

  </world>
</sdf>
```

**`world_builder.py`** — add `feature_models` parameter (default empty string):
```python
def generate_world_sdf(
    world_name, center_lat, center_lon, output_dir, heightmap_info,
    feature_models="",
):
```

Pass `feature_models=feature_models` to `template.format(...)`.

### 4. Update CLI pipeline

**`generate_world.py`:**
- Add `--no-features` flag (default: features enabled when ENC data provided)
- After Step 1 (read S57 charts), if features enabled and s57_features is not None:
  ```python
  from .feature_models import generate_feature_models
  feature_sdf = generate_feature_models(s57_features, bbox.center_lat, bbox.center_lon)
  ```
- Pass `feature_models=feature_sdf` to `generate_world_sdf()`

### 5. Tests

**`test/test_s57_reader.py`** — add tests for new feature extraction:
- Test BUISGL polygon extraction with mock OGR data
- Test BOYLAT point extraction with COLOUR attribute
- Test BRIDGE extraction with VERCLR attribute
- Test that new features are merged in `read_enc_directory()`

**`test/test_feature_models.py`** (new):
- Test `generate_feature_models()` with empty features → empty string
- Test building polygon → valid SDF `<model>` with `<polyline>`
- Test buoy point → valid SDF `<model>` with cylinder + cone
- Test COLOUR mapping → correct material colors
- Test coordinate conversion → correct ENU offsets
- Parse generated SDF as XML to verify validity

**`test/test_world_builder.py`** — update:
- Test that `feature_models` parameter is included in output SDF
- Existing tests should continue passing (feature_models defaults to "")

### 6. Copyright headers

Add Apache 2.0 headers to new files (`feature_models.py`, `test/test_feature_models.py`).

## Commit Strategy

1. **feat: extend S57 reader to extract buildings, buoys, and nav aids**
2. **feat: add feature_models module for SDF model generation**
3. **feat: integrate feature placement into world builder and CLI**
4. **test: add tests for feature extraction and model generation**

## Verification

1. `colcon build --packages-select marine_charts_to_gazebo_world portsmouth_nh_gazebo`
2. `colcon test --packages-select marine_charts_to_gazebo_world` — all tests pass
3. End-to-end with ENC data (if available):
   ```bash
   ros2 run marine_charts_to_gazebo_world generate_world \
     --enc-root /path/to/ENC \
     --bounds 43.065,-70.72,43.085,-70.70 \
     --output-dir /tmp/test_features \
     --world-name portsmouth_features
   ```
   Verify SDF contains `<model name="building_...">` and `<model name="buoy_...">` elements.
4. End-to-end without features:
   ```bash
   ros2 run marine_charts_to_gazebo_world generate_world \
     --bounds 43.065,-70.72,43.085,-70.70 \
     --output-dir /tmp/test_nofeatures \
     --world-name test --no-features
   ```
   Verify SDF contains no feature models.
5. Verify no flake8 violations in new code.
