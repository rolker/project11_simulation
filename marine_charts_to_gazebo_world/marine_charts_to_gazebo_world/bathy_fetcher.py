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

# NOAA ERDDAP endpoints for ETOPO 2022 (15 arc-second global relief).
# Uses NetCDF format (.nc) for full float precision — the .geotif format
# on some mirrors returns 8-bit data, losing elevation accuracy.
# Tried in order; CoastWatch uses -180/180 longitude (works for western
# hemisphere), NCEI was the original host (currently 404, may return).
_ERDDAP_URLS = [
    "https://coastwatch.pfeg.noaa.gov/erddap/griddap/ETOPO_2022_v1_15s.nc",
    "https://www.ncei.noaa.gov/erddap/griddap/ETOPO_2022_v1_15s.nc",
]

# Default cache directory
_DEFAULT_CACHE_DIR = os.path.join(
    os.environ.get("XDG_CACHE_HOME", os.path.expanduser("~/.cache")),
    "marine_charts_to_gazebo_world",
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
        cache_dir: Directory to cache downloaded data. Defaults to
            ~/.cache/marine_charts_to_gazebo_world/.

    Returns:
        Tuple of (elevation_array, metadata_dict).
        elevation_array: 2D numpy array, shape (rows, cols), north-up.
        metadata_dict: dict with keys 'geotransform', 'projection',
            'south', 'west', 'north', 'east'.
    """
    if cache_dir is None:
        cache_dir = _DEFAULT_CACHE_DIR
    os.makedirs(cache_dir, exist_ok=True)

    cache_file = os.path.join(cache_dir, f"etopo_{_cache_key(bbox)}.nc")

    if not os.path.exists(cache_file):
        _download_etopo(bbox, cache_file)

    return _read_raster(cache_file, bbox)


def _download_etopo(bbox: BoundingBox, output_path: str):
    """Download ETOPO NetCDF from NOAA ERDDAP, trying mirrors in order."""
    buf = 0.01
    constraint = (
        f"?z[({bbox.south - buf}):({bbox.north + buf})]"
        f"[({bbox.west - buf}):({bbox.east + buf})]"
    )

    last_error = None
    for base_url in _ERDDAP_URLS:
        url = base_url + constraint
        try:
            _download_file(url, output_path)
            return
        except requests.HTTPError as e:
            last_error = e
            continue

    raise last_error


def _download_file(url: str, output_path: str):
    """Download a URL to a file atomically."""
    with requests.get(url, stream=True, timeout=(30, 300)) as response:
        response.raise_for_status()

        fd, tmp_path = tempfile.mkstemp(
            suffix=".nc", dir=os.path.dirname(output_path)
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


def _read_raster(
    filepath: str, bbox: BoundingBox
) -> Tuple[np.ndarray, dict]:
    """Read a raster file (GeoTIFF or NetCDF) via GDAL."""
    ds = gdal.Open(filepath, gdal.GA_ReadOnly)
    if ds is None:
        raise FileNotFoundError(f"Cannot open raster: {filepath}")

    # NetCDF files may expose data as subdatasets rather than direct bands
    subdatasets = ds.GetSubDatasets()
    if subdatasets:
        ds = None
        ds = gdal.Open(subdatasets[0][0], gdal.GA_ReadOnly)
        if ds is None:
            raise FileNotFoundError(
                f"Cannot open NetCDF subdataset: {subdatasets[0][0]}"
            )

    try:
        gt = ds.GetGeoTransform()
        band = ds.GetRasterBand(1)
        data = band.ReadAsArray()

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
