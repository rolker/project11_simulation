"""Fetch bathymetry/topography data from online sources."""

import hashlib
import os
import tempfile
from typing import Optional, Tuple

import numpy as np
import requests
from osgeo import gdal

from .s57_reader import BoundingBox

gdal.UseExceptions()

# NOAA ERDDAP endpoint for ETOPO 2022 (15 arc-second global relief)
ERDDAP_URL = (
    "https://www.ncei.noaa.gov/erddap/griddap/ETOPO_2022_v1_15s.geotif"
)

# Default cache directory
_DEFAULT_CACHE_DIR = os.path.join(
    os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache")),
    "s57_world_generator",
)


def _cache_key(bbox: BoundingBox) -> str:
    """Generate a deterministic cache key from bbox."""
    s = f"{bbox.south:.6f},{bbox.west:.6f},{bbox.north:.6f},{bbox.east:.6f}"
    return hashlib.md5(s.encode()).hexdigest()


def fetch_etopo(
    bbox: BoundingBox,
    cache_dir: Optional[str] = None,
) -> Tuple[np.ndarray, dict]:
    """Fetch ETOPO 2022 elevation data for a bounding box from NOAA ERDDAP.

    Returns elevation in meters (positive up, negative for ocean depth).

    Args:
        bbox: Geographic bounding box in WGS84 degrees.
        cache_dir: Directory to cache downloaded GeoTIFFs. Defaults to
            ~/.cache/s57_world_generator/.

    Returns:
        Tuple of (elevation_array, metadata_dict).
        elevation_array: 2D numpy array, shape (rows, cols), north-up.
        metadata_dict: dict with keys 'geotransform', 'projection',
            'south', 'west', 'north', 'east'.
    """
    if cache_dir is None:
        cache_dir = _DEFAULT_CACHE_DIR
    os.makedirs(cache_dir, exist_ok=True)

    cache_file = os.path.join(cache_dir, f"etopo_{_cache_key(bbox)}.tif")

    if not os.path.exists(cache_file):
        _download_etopo(bbox, cache_file)

    return _read_geotiff(cache_file, bbox)


def _download_etopo(bbox: BoundingBox, output_path: str):
    """Download ETOPO GeoTIFF from NOAA ERDDAP."""
    # ERDDAP griddap constraint: latitude and longitude ranges
    # Format: dataset.fileType?variable[(lat_start):(lat_end)][(lon_start):(lon_end)]
    # Add a small buffer to ensure full coverage
    buf = 0.01
    url = (
        f"{ERDDAP_URL}"
        f"?z[({bbox.south - buf}):({bbox.north + buf})]"
        f"[({bbox.west - buf}):({bbox.east + buf})]"
    )

    with requests.get(url, stream=True, timeout=(30, 300)) as response:
        response.raise_for_status()

        # Write to temp file first, then rename for atomicity
        fd, tmp_path = tempfile.mkstemp(
            suffix=".tif", dir=os.path.dirname(output_path)
        )
        try:
            with os.fdopen(fd, "wb") as tmp_file:
                for chunk in response.iter_content(chunk_size=8192):
                    if chunk:
                        tmp_file.write(chunk)
            os.rename(tmp_path, output_path)
        except Exception:
            try:
                os.unlink(tmp_path)
            except FileNotFoundError:
                pass
            raise


def _read_geotiff(
    filepath: str, bbox: BoundingBox
) -> Tuple[np.ndarray, dict]:
    """Read a GeoTIFF and extract data within the bounding box."""
    ds = gdal.Open(filepath, gdal.GA_ReadOnly)
    if ds is None:
        raise FileNotFoundError(f"Cannot open GeoTIFF: {filepath}")

    try:
        gt = ds.GetGeoTransform()
        band = ds.GetRasterBand(1)
        data = band.ReadAsArray()

        # GeoTransform: (x_origin, x_pixel_size, 0, y_origin, 0, y_pixel_size)
        # y_pixel_size is typically negative (north-up)
        metadata = {
            "geotransform": gt,
            "projection": ds.GetProjection(),
            "south": bbox.south,
            "west": bbox.west,
            "north": bbox.north,
            "east": bbox.east,
        }

        return data.astype(np.float64), metadata
    finally:
        ds = None
