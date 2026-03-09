# Copyright 2025 Roland Arsenault
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Build a continuous terrain surface from bathymetry and S57 data."""

import logging
from typing import Optional, Tuple

import numpy as np
from osgeo import gdal, ogr, osr
from scipy.interpolate import LinearNDInterpolator
from scipy.ndimage import distance_transform_edt

from .s57_reader import BoundingBox, DepthArea, S57Features

logger = logging.getLogger(__name__)

# WGS84 meters per degree (approximate, varies with latitude)
_METERS_PER_DEG_LAT = 111_320.0

# Shoreline ramp parameters
_RAMP_DISTANCE_M = 25.0  # distance over which land ramps up
_RAMP_MAX_ELEVATION = 5.0  # maximum land elevation in meters


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
    If base_elevation is None, terrain is built from S57 land areas and
    soundings: land cells get a synthetic ramp from the shoreline, water
    cells get interpolated sounding depths.

    Args:
        bbox: Geographic bounding box.
        base_elevation: Optional 2D array of elevation from ETOPO/GEBCO
            (meters, positive up). If None, uses S57-only terrain with
            shoreline ramp.
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
        # Overlay S57 soundings if available
        if s57_features is not None and s57_features.soundings:
            terrain = _overlay_soundings(
                terrain, grid_lons, grid_lats, s57_features, bbox
            )
    else:
        # S57-only terrain: land ramp + sounding interpolation
        terrain = _build_s57_terrain(
            grid_lons, grid_lats, s57_features, grid_size, bbox
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


def _build_s57_terrain(
    grid_lons: np.ndarray,
    grid_lats: np.ndarray,
    s57_features: Optional[S57Features],
    grid_size: int,
    bbox: BoundingBox,
) -> np.ndarray:
    """Build terrain from S57 land areas and soundings.

    Land cells get a synthetic elevation ramp from the shoreline (0-5m
    over 25m distance). Water cells get interpolated sounding depths,
    or 0.0 if no soundings are available.
    """
    if s57_features is None:
        return np.zeros((grid_size, grid_size), dtype=np.float64)

    # Rasterize land areas and coastlines onto a binary mask
    land_mask = _rasterize_land(
        s57_features.land_areas, s57_features.coastlines,
        bbox, grid_size,
    )

    # Compute distance from shore for land cells
    center_lat = bbox.center_lat
    m_per_deg_lon = _meters_per_deg_lon(center_lat)

    # Cell sizes in meters (anisotropic)
    cell_size_y = (bbox.north - bbox.south) / (grid_size - 1) * _METERS_PER_DEG_LAT
    cell_size_x = (bbox.east - bbox.west) / (grid_size - 1) * m_per_deg_lon

    # Distance transform on land mask (distance from shore into land interior)
    # land_mask: land=1, water=0 — EDT measures distance from nearest 0-cell
    distance_cells = distance_transform_edt(
        land_mask, sampling=[cell_size_y, cell_size_x]
    )

    # Ramp land elevation: linear 0→5m over 25m, capped at 5m
    land_elevation = np.minimum(
        distance_cells / _RAMP_DISTANCE_M * _RAMP_MAX_ELEVATION,
        _RAMP_MAX_ELEVATION,
    )

    # Start with water depths from soundings
    water_terrain = _synthesize_from_s57(
        grid_lons, grid_lats, s57_features, grid_size
    )

    # Clamp interpolated depths to depth area ranges
    if s57_features.depth_areas:
        da_min, da_max = _rasterize_depth_areas(
            s57_features.depth_areas, bbox, grid_size,
        )
        da_mask = ~np.isnan(da_min)
        if np.any(da_mask):
            max_elev = -da_min[da_mask]  # shallowest allowed (elevation)
            min_elev = -da_max[da_mask]  # deepest allowed (elevation)
            water_terrain[da_mask] = np.clip(
                water_terrain[da_mask], min_elev, max_elev,
            )

    # Combine: land gets ramp elevation, water gets sounding depths
    terrain = np.where(land_mask, land_elevation, water_terrain)

    return terrain


def _rasterize_land(
    land_areas, coastlines, bbox: BoundingBox, grid_size: int,
) -> np.ndarray:
    """Rasterize LNDARE polygons and COALNE lines onto a binary grid.

    Returns a boolean mask where True = land.
    """
    if not land_areas and not coastlines:
        return np.zeros((grid_size, grid_size), dtype=bool)

    # Create in-memory OGR dataset with all land geometries
    mem_driver = ogr.GetDriverByName('Memory')
    mem_ds = mem_driver.CreateDataSource('land')
    srs = osr.SpatialReference()
    srs.ImportFromEPSG(4326)
    mem_layer = mem_ds.CreateLayer('land', srs, ogr.wkbPolygon)

    # Add land area polygons
    for geom in land_areas:
        feat = ogr.Feature(mem_layer.GetLayerDefn())
        feat.SetGeometry(geom)
        mem_layer.CreateFeature(feat)

    # Rasterize using GDAL
    # GeoTransform: (west, pixel_width, 0, north, 0, -pixel_height)
    pixel_width = (bbox.east - bbox.west) / grid_size
    pixel_height = (bbox.north - bbox.south) / grid_size
    gt = (bbox.west, pixel_width, 0.0, bbox.north, 0.0, -pixel_height)

    # Create in-memory raster
    rast_driver = gdal.GetDriverByName('MEM')
    rast_ds = rast_driver.Create('', grid_size, grid_size, 1, gdal.GDT_Byte)
    rast_ds.SetGeoTransform(gt)
    rast_ds.SetProjection(srs.ExportToWkt())
    band = rast_ds.GetRasterBand(1)
    band.Fill(0)

    # Rasterize polygons — burn value 1 for land
    gdal.RasterizeLayer(rast_ds, [1], mem_layer, burn_values=[1])

    # Read result
    land_mask = band.ReadAsArray().astype(bool)

    # Also rasterize coastlines as land boundary pixels
    if coastlines:
        line_layer = mem_ds.CreateLayer('coast', srs, ogr.wkbLineString)
        for geom in coastlines:
            feat = ogr.Feature(line_layer.GetLayerDefn())
            feat.SetGeometry(geom)
            line_layer.CreateFeature(feat)

        # Burn coastline pixels onto the same mask
        coast_ds = rast_driver.Create('', grid_size, grid_size, 1, gdal.GDT_Byte)
        coast_ds.SetGeoTransform(gt)
        coast_ds.SetProjection(srs.ExportToWkt())
        coast_band = coast_ds.GetRasterBand(1)
        coast_band.Fill(0)
        gdal.RasterizeLayer(coast_ds, [1], line_layer, burn_values=[1])
        coast_mask = coast_band.ReadAsArray().astype(bool)
        land_mask |= coast_mask

    # Clean up
    mem_ds = None
    rast_ds = None

    return land_mask


def _rasterize_depth_areas(
    depth_areas: list, bbox: BoundingBox, grid_size: int,
) -> Tuple[np.ndarray, np.ndarray]:
    """Rasterize depth areas with scale ordering onto min/max depth grids.

    Depth areas are sorted by compilation_scale descending (least detailed
    first) so that more detailed charts overwrite less detailed ones.

    Returns:
        Tuple of (min_depth_grid, max_depth_grid) with NaN where no
        depth area covers the cell.  Values are in meters, positive down.
    """
    min_depth = np.full((grid_size, grid_size), np.nan, dtype=np.float64)
    max_depth = np.full((grid_size, grid_size), np.nan, dtype=np.float64)

    if not depth_areas:
        return min_depth, max_depth

    # Sort by compilation_scale descending (coarsest first, detailed last)
    sorted_areas = sorted(
        depth_areas, key=lambda da: da.compilation_scale, reverse=True,
    )

    # GeoTransform matching _rasterize_land
    pixel_width = (bbox.east - bbox.west) / grid_size
    pixel_height = (bbox.north - bbox.south) / grid_size
    gt = (bbox.west, pixel_width, 0.0, bbox.north, 0.0, -pixel_height)

    srs = osr.SpatialReference()
    srs.ImportFromEPSG(4326)
    wkt = srs.ExportToWkt()
    rast_driver = gdal.GetDriverByName('MEM')

    for da in sorted_areas:
        # Rasterize this polygon to a mask
        mem_driver = ogr.GetDriverByName('Memory')
        mem_ds = mem_driver.CreateDataSource('da')
        mem_layer = mem_ds.CreateLayer('da', srs, ogr.wkbPolygon)
        feat = ogr.Feature(mem_layer.GetLayerDefn())
        feat.SetGeometry(da.geometry)
        mem_layer.CreateFeature(feat)

        rast_ds = rast_driver.Create('', grid_size, grid_size, 1, gdal.GDT_Byte)
        rast_ds.SetGeoTransform(gt)
        rast_ds.SetProjection(wkt)
        band = rast_ds.GetRasterBand(1)
        band.Fill(0)
        gdal.RasterizeLayer(rast_ds, [1], mem_layer, burn_values=[1])

        mask = band.ReadAsArray().astype(bool)
        min_depth[mask] = da.min_depth
        max_depth[mask] = da.max_depth

        mem_ds = None
        rast_ds = None

    return min_depth, max_depth


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
        logger.warning(
            "S57 terrain interpolation failed; falling back to flat surface",
            exc_info=True,
        )
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
