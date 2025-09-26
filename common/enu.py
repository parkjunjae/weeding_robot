import math


def wgs84_to_enu(lat, lon, lat0, lon0, yaw0_deg=0.0):
    R = 6378137.0
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x_e = R * dlon * math.cos(math.radians(lat0))
    y_n = R * dlat
    c, s = math.cos(math.radians(yaw0_deg)), math.sin(math.radians(yaw0_deg))
    x = c*x_e + s*y_n
    y = -s*x_e + c*y_n
    return x, y