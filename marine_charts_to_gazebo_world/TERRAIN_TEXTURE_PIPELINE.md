# Terrain Texture Pipeline

How OSM features are rasterized onto a texture image and applied to a
Gazebo OGRE2 heightmap.  This document exists because the coordinate
mapping between the rasterizer and Gazebo's UV system is subtle and
has been a recurring source of bugs.

## Pipeline Overview

```
OSM data (lat/lon)
    │
    ▼
osm_texture.py ── rasterize_osm_texture()
    │  Converts lat/lon → ENU → pixel coordinates
    │  Draws features onto a PIL Image (grid_size × grid_size)
    ▼
land_diffuse.png  (square image, 1025×1025 for grid_power=10)
    │
    ▼
heightmap.py ── _write_model_sdf()
    │  Embeds texture in SDF <heightmap> with <texture><size>
    ▼
Gazebo OGRE2 renderer
    │  Maps texture onto heightmap using UV coordinates
    ▼
Visible terrain in simulation
```

## Coordinate Systems

### ENU (East-North-Up)

All 3D positioning uses a local ENU frame centered at `(ref_lat, ref_lon)`,
which is the bounding box center.  East is +X, North is +Y.

- `terrain.py` builds the elevation grid in ENU.
- `feature_models.py` places 3D models in ENU.
- `osm_texture.py` converts features to ENU before rasterizing.

### Heightmap Image

The heightmap PNG is written north-up:

- **Row 0** = north edge of the bounding box
- **Last row** = south edge
- **Column 0** = west edge
- **Last column** = east edge

### Texture Image

The texture PNG uses the same orientation as the heightmap:

- **Row 0** = north edge
- **Last row** = south edge (or unused padding for non-square extents)

## Gazebo OGRE2 UV Mapping

This is the critical part that caused repeated bugs.

### How `<texture><size>` Works

The SDF heightmap element accepts a `<size>` parameter per texture layer:

```xml
<texture>
  <diffuse>land_diffuse.png</diffuse>
  <size>656.6</size>          <!-- single value: meters per tile -->
</texture>
<size>656.6 337.9 19.6</size>  <!-- heightmap X Y Z extent -->
```

The texture `<size>` is a **single scalar** — it controls how many world
meters one full texture tile covers in **both** U and V.  It does NOT
accept separate U/V values.

### UV Coordinate Range

For a heightmap of size `(size_x, size_y)` with texture tiling at
`tex_size` meters:

| Axis | UV range | Meaning |
|------|----------|---------|
| U | `[0, size_x / tex_size]` | West to east |
| V | `[0, size_y / tex_size]` | North to south |

**OGRE2 maps image row 0 to V=0 (north), with V increasing southward.**
This matches the heightmap image orientation (row 0 = north).

### Non-Square Heightmaps

When `size_x ≠ size_y`, only a portion of the texture image is used:

- `tex_size = max(size_x, size_y)` to prevent any axis from tiling/wrapping.
- U spans `[0, size_x / tex_size]` — full width if `size_x = tex_size`.
- V spans `[0, size_y / tex_size]` — partial height if `size_y < tex_size`.

**Example**: A 656.6m × 337.9m heightmap with `tex_size = 656.6`:
- U range: `[0, 1.0]` — full image width
- V range: `[0, 0.515]` — top 51.5% of image height
- Features must be rasterized into the **top-left** portion of the image.
- The bottom ~48.5% of the image is unused (base land color).

## Pixel Mapping Formula

The rasterizer converts ENU coordinates to pixel positions:

```python
# East ∈ [-size_x/2, +size_x/2]  →  pixel column [0, grid_size-1 * size_x/tex_size]
px = (east + size_x / 2) / tex_size * (grid_size - 1)

# North ∈ [-size_y/2, +size_y/2]  →  pixel row [0, grid_size-1 * size_y/tex_size]
# Row 0 = north edge, row increases southward (matching OGRE2 V direction)
py = (size_y / 2 - north) / tex_size * (grid_size - 1)
```

### Why Not Simply `(grid_size-1) - north_normalized`?

The naive formula `py = (grid_size-1) - (north + size_y/2) / size_y * (grid_size-1)`
spreads features across the full image height.  But Gazebo only samples the
top `size_y / tex_size` fraction.  Dividing by `tex_size` instead of `size_y`
compresses features into that fraction.

## Common Pitfalls

### 1. Dividing Y by `size_y` Instead of `tex_size`

**Symptom**: Texture appears displaced ~100–200 m south of the 3D models.

The pixel Y formula must divide by `tex_size`, not `size_y`.  When
`size_x > size_y` (wide heightmap), dividing by `size_y` stretches features
across the full image height, but Gazebo only reads the top portion.
Features that should appear at the heightmap center end up in the bottom
half of the image where Gazebo doesn't look.

### 2. Assuming V=0 is at the South (OpenGL Convention)

**Symptom**: Texture is invisible or vertically flipped.

OpenGL's texture convention has V=0 at the bottom, but OGRE2's heightmap
texture mapping has V=0 at image row 0 (north/top).  Do NOT flip the image
or invert the Y formula to put south at row 0.

### 3. Flipping the Image with `FLIP_TOP_BOTTOM`

**Symptom**: Texture appears but is vertically mirrored.

The terrain grid and texture rasterizer both use north-at-row-0.  Adding
a vertical flip makes them inconsistent.

### 4. Using `size_x` for Texture `<size>` on Tall Heightmaps

**Symptom**: Texture wraps/tiles in the V direction.

If `size_y > size_x`, then `V_max = size_y / size_x > 1.0`, causing the
texture to tile.  Always use `tex_size = max(size_x, size_y)`.

## Debugging Texture Alignment

### Quick Visual Check

1. Build the world: `./simulation_ws/build.sh portsmouth_nh_gazebo`
2. Open the texture PNG directly — features should be in the **top-left**
   corner for wide heightmaps (size_x > size_y).
3. Launch in Gazebo and compare feature positions (roads, landuse boundaries)
   against the 3D building models.
4. If the texture is displaced but 3D models are correct, the bug is in the
   texture pipeline (not the OSM data or ENU conversion).

### Comparing Texture vs 3D Models

The 3D feature models (`feature_models.py`) and the texture rasterizer
(`osm_texture.py`) both consume the same OSM data and use the same ENU
conversion.  If 3D models are correctly positioned but the texture is
offset, the problem is always in:

1. The pixel mapping formula in `_latlon_to_pixel()`
2. The `tex_size` value passed to the rasterizer
3. The `<texture><size>` value in the SDF

### Inspecting SDF Values

Check the generated `model.sdf` in the build output:

```bash
cat simulation_ws/build/portsmouth_nh_gazebo/worlds/<world>/<world>_terrain/model.sdf
```

Verify:
- `<size>X Y Z</size>` matches the expected heightmap extent in meters.
- `<texture><size>` equals `max(X, Y)` (not just `X`).
- `<pos>` is `0 0 min_elev` for single-tile worlds.

### Clearing Caches

OSM data is cached.  After changing the Overpass query, bump the cache
key version in `osm_fetcher.py` (`_cache_key()` prefix) and delete stale
cache files:

```bash
rm ~/.cache/marine_charts_to_gazebo_world/osm_terrain_*
```

Also delete the build artifacts to force world regeneration:

```bash
rm -rf simulation_ws/build/portsmouth_nh_gazebo/worlds/<world>
rm -rf simulation_ws/install/portsmouth_nh_gazebo/share/portsmouth_nh_gazebo/worlds/<world>
```

## File Reference

| File | Role |
|------|------|
| `osm_texture.py` | Rasterizes OSM features to texture image |
| `terrain.py` | Builds elevation grid in ENU |
| `heightmap.py` | Writes heightmap PNG + SDF (including texture `<size>`) |
| `generate_world.py` | Orchestrates pipeline, passes `tex_size` between stages |
| `feature_models.py` | Generates 3D SDF models (independent of texture) |
| `osm_fetcher.py` | Fetches/caches OSM data from Overpass API |
