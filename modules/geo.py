# GPS -> local x/y in metres. Equirectangular about an origin -- fine at
# track scale (a few km).

import numpy as np

METERS_PER_DEG_LAT = 111320.0


def project_latlon_to_xy(lat, lon, origin_lat, origin_lon):
    meters_per_deg_lon = METERS_PER_DEG_LAT * np.cos(np.radians(origin_lat))
    x = (lon - origin_lon) * meters_per_deg_lon
    y = (lat - origin_lat) * METERS_PER_DEG_LAT
    return x, y


def compute_gps_origin(gps_lat_channel, gps_lon_channel):
    # origin = first raw GPS sample, for use before any resampled state exists.
    # Can sit slightly off prepare_vehicle_state's origin (anchored on
    # ecu_speed's first time) -- same projection, different anchor instant.
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
