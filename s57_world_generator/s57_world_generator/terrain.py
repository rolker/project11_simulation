"""Build a continuous terrain surface from bathymetry and S57 data."""

from typing import Optional, Tuple

import numpy as np
from scipy.interpolate import LinearNDInterpolator

from .s57_reader import BoundingBox, S57Features

# WGS84 meters per degree (approximate, varies with latitude)
_METERS_PER_DEG_LAT = 111_320.0


def _meters_per_deg_lon(lat_deg: float) -> float:
    """Approximate meters per degree of longitude at a given latitude."""
    return _METERS_PER_DEG_LAT * np.cos(np.radians(lat_deg))


def build_terrain(
    bbox: BoundingBox,
    base_elevation: Optional[np.ndarray] = None,
    base_geotransform: Optional[tuple] = None,
    s57_features: Optional[S57Features] = None,
    grid_size: int = 513,
) -> Tuple[np.ndarray, dict]:
    """Build a terrain grid by combining base elevation with S57 data.

    The output grid is in a local ENU frame centered on the bbox center.
    If base_elevation is None, terrain is synthesized from S57 data alone.

    Args:
        bbox: Geographic bounding box.
        base_elevation: Optional 2D array of elevation from ETOPO/GEBCO
            (meters, positive up). If None, a flat surface is used and
            S57 soundings provide the depth variation.
        base_geotransform: GDAL GeoTransform of the base elevation raster.
            Required if base_elevation is provided.
        s57_features: Optional S57 features (soundings, depth areas, land).
        grid_size: Output grid dimension (should be 2^n + 1 for Gazebo).

    Returns:
        Tuple of (terrain_grid, terrain_info).
        terrain_grid: 2D numpy array of elevation values (meters, positive up),
            shape (grid_size, grid_size), north-up orientation.
        terrain_info: dict with 'size_x', 'size_y', 'min_elevation',
            'max_elevation', 'center_lat', 'center_lon'.
    """
    center_lat = bbox.center_lat
    center_lon = bbox.center_lon
    m_per_deg_lon = _meters_per_deg_lon(center_lat)

    # Compute physical extent in meters
    size_x = (bbox.east - bbox.west) * m_per_deg_lon
    size_y = (bbox.north - bbox.south) * _METERS_PER_DEG_LAT

    # Build output grid coordinates in degrees
    lons = np.linspace(bbox.west, bbox.east, grid_size)
    lats = np.linspace(bbox.north, bbox.south, grid_size)  # north-up
    grid_lons, grid_lats = np.meshgrid(lons, lats)

    if base_elevation is not None and base_geotransform is not None:
        # Sample base elevation onto output grid using bilinear interpolation
        terrain = _sample_raster(
            base_elevation, base_geotransform, grid_lons, grid_lats
        )
    else:
        # No base raster — synthesize from S57 data
        terrain = _synthesize_from_s57(
            grid_lons, grid_lats, s57_features, grid_size
        )

    # Overlay S57 soundings if available
    if s57_features is not None and s57_features.soundings:
        terrain = _overlay_soundings(
            terrain, grid_lons, grid_lats, s57_features, bbox
        )

    terrain_info = {
        "size_x": size_x,
        "size_y": size_y,
        "min_elevation": float(np.nanmin(terrain)),
        "max_elevation": float(np.nanmax(terrain)),
        "center_lat": center_lat,
        "center_lon": center_lon,
        "grid_size": grid_size,
    }

    return terrain, terrain_info


def _synthesize_from_s57(
    grid_lons: np.ndarray,
    grid_lats: np.ndarray,
    s57_features: Optional[S57Features],
    grid_size: int,
) -> np.ndarray:
    """Synthesize terrain from S57 sounding and depth area data.

    When no base raster is available, we interpolate a surface directly
    from S57 soundings. If no soundings exist either, returns a flat
    surface at 0m elevation.
    """
    if s57_features is None or not s57_features.soundings:
        return np.zeros((grid_size, grid_size), dtype=np.float64)

    # Build interpolation from all soundings
    snd_lons = np.array([s.lon for s in s57_features.soundings])
    snd_lats = np.array([s.lat for s in s57_features.soundings])
    snd_elevations = np.array([-s.depth for s in s57_features.soundings])

    try:
        interp = LinearNDInterpolator(
            np.column_stack([snd_lons, snd_lats]),
            snd_elevations,
        )
        terrain = interp(grid_lons, grid_lats)
        # Fill NaN (outside convex hull of soundings) with nearest value
        from scipy.interpolate import NearestNDInterpolator
        nan_mask = np.isnan(terrain)
        if np.any(nan_mask):
            nearest = NearestNDInterpolator(
                np.column_stack([snd_lons, snd_lats]),
                snd_elevations,
            )
            terrain[nan_mask] = nearest(
                grid_lons[nan_mask], grid_lats[nan_mask]
            )
        return terrain
    except Exception:
        return np.zeros((grid_size, grid_size), dtype=np.float64)


def _sample_raster(
    data: np.ndarray,
    geotransform: tuple,
    query_lons: np.ndarray,
    query_lats: np.ndarray,
) -> np.ndarray:
    """Bilinear interpolation of raster data at query lat/lon points."""
    gt = geotransform
    # Convert geographic coords to pixel coords
    # pixel_x = (lon - gt[0]) / gt[1]
    # pixel_y = (lat - gt[3]) / gt[5]
    pixel_x = (query_lons - gt[0]) / gt[1]
    pixel_y = (query_lats - gt[3]) / gt[5]

    rows, cols = data.shape
    result = np.full(query_lons.shape, np.nan, dtype=np.float64)

    # Bilinear interpolation
    px = np.clip(pixel_x, 0, cols - 1)
    py = np.clip(pixel_y, 0, rows - 1)

    x0 = np.floor(px).astype(int)
    y0 = np.floor(py).astype(int)
    x1 = np.clip(x0 + 1, 0, cols - 1)
    y1 = np.clip(y0 + 1, 0, rows - 1)

    fx = px - x0
    fy = py - y0

    result = (
        data[y0, x0] * (1 - fx) * (1 - fy)
        + data[y0, x1] * fx * (1 - fy)
        + data[y1, x0] * (1 - fx) * fy
        + data[y1, x1] * fx * fy
    )

    return result


def _overlay_soundings(
    terrain: np.ndarray,
    grid_lons: np.ndarray,
    grid_lats: np.ndarray,
    s57_features: S57Features,
    bbox: BoundingBox,
) -> np.ndarray:
    """Refine terrain using S57 sounding data.

    Soundings are point measurements with precise depths. We use them to
    create a correction surface that adjusts the base elevation where
    sounding data is available.
    """
    if not s57_features.soundings:
        return terrain

    # Collect sounding points
    snd_lons = np.array([s.lon for s in s57_features.soundings])
    snd_lats = np.array([s.lat for s in s57_features.soundings])
    # S57 depths are positive down; convert to elevation (positive up)
    snd_elevations = np.array([-s.depth for s in s57_features.soundings])

    # Sample base elevation at sounding locations
    # (we'd need the base raster for this, but we can approximate by
    # interpolating the already-sampled terrain grid)
    from scipy.interpolate import RegularGridInterpolator

    lats_1d = grid_lats[:, 0]  # north to south
    lons_1d = grid_lons[0, :]  # west to east

    base_interp = RegularGridInterpolator(
        (lats_1d, lons_1d), terrain,
        method="linear", bounds_error=False, fill_value=None
    )
    base_at_soundings = base_interp(np.column_stack([snd_lats, snd_lons]))

    # Compute corrections (sounding truth minus base estimate)
    corrections = snd_elevations - base_at_soundings

    # Only apply corrections where they're meaningful (> 0.5m difference)
    significant = np.abs(corrections) > 0.5
    if not np.any(significant):
        return terrain

    sig_lons = snd_lons[significant]
    sig_lats = snd_lats[significant]
    sig_corrections = corrections[significant]

    # Interpolate corrections onto the grid
    try:
        correction_interp = LinearNDInterpolator(
            np.column_stack([sig_lons, sig_lats]),
            sig_corrections,
        )
        correction_grid = correction_interp(grid_lons, grid_lats)
        # Only apply where interpolation succeeded (not NaN)
        valid = ~np.isnan(correction_grid)
        terrain[valid] += correction_grid[valid]
    except Exception:
        # If interpolation fails (e.g., too few points), skip correction
        pass

    return terrain
