# Shared GPS -> local x/y (metres) projection primitives.
# Pure Python/numpy. No Qt imports.
# Equirectangular approximation anchored at a chosen origin -- fine at
# track scale (a few km), no need for a proper geodesic projection.

import numpy as np

METERS_PER_DEG_LAT = 111320.0


def project_latlon_to_xy(lat, lon, origin_lat, origin_lon):
    meters_per_deg_lon = METERS_PER_DEG_LAT * np.cos(np.radians(origin_lat))
    x = (lon - origin_lon) * meters_per_deg_lon
    y = (lat - origin_lat) * METERS_PER_DEG_LAT
    return x, y


def compute_gps_origin(gps_lat_channel, gps_lon_channel):
    # Origin = the GPS channel's own first sample. Used where only raw
    # channels are available (no resampled vehicle state yet) -- e.g.
    # corner apex positions computed immediately after parsing, before any
    # stability analysis has run. This can differ by a fraction of a
    # second (and so a small distance) from prepare_vehicle_state's origin
    # in modules/stability_analysis.py, which anchors on the GPS value
    # interpolated onto ecu_speed's first sample time. Both are valid
    # local origins for the SAME projection formula above; only the
    # projection math is shared between the two call sites, not the
    # origin's exact anchor instant.
    if gps_lat_channel is None or gps_lon_channel is None:
        return None, None
    if (gps_lat_channel.get("quality") in ("missing", "failed")
            or gps_lon_channel.get("quality") in ("missing", "failed")):
        return None, None
    lat_data = gps_lat_channel.get("data")
    lon_data = gps_lon_channel.get("data")
    if lat_data is None or lon_data is None or len(lat_data) == 0 or len(lon_data) == 0:
        return None, None
    return float(lat_data[0]), float(lon_data[0])
