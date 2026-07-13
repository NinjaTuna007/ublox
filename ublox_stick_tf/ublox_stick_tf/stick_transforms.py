#!/usr/bin/env python3
"""Pure GPS/UTM transform helpers using transforms3d."""

import math

import numpy as np
from transforms3d.euler import euler2quat


def _t3d_to_ros(q_wxyz):
    w, x, y, z = q_wxyz
    return (x, y, z, w)


def latlon_to_utm(latitude_deg, longitude_deg):
    """Convert WGS84 lat/lon (deg) to UTM easting/northing and zone."""
    import utm
    easting, northing, zone_number, zone_letter = utm.from_latlon(
        latitude_deg, longitude_deg)
    return (float(easting), float(northing), int(zone_number), str(zone_letter))


def utm_zone_frame_id(zone_number, zone_letter):
    """TF frame name for a UTM zone, e.g. utm_33_V."""
    return 'utm_{}_{}'.format(zone_number, zone_letter)


def yaw_to_quat_xyzw(yaw_rad):
    """Yaw-only ENU quaternion in ROS (x, y, z, w) order."""
    q_wxyz = euler2quat(0.0, 0.0, yaw_rad, axes='sxyz')
    n = np.linalg.norm(q_wxyz)
    if n > 0.0:
        q_wxyz = q_wxyz / n
    return _t3d_to_ros(q_wxyz)


def heading_rad_to_yaw_enu(heading_rad):
    """Map ground-track heading (atan2(E,N)) to ENU yaw about Z."""
    return math.pi / 2.0 - heading_rad
