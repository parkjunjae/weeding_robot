import numpy as np
from typing import Tuple


def dwa_lite(v_ref: float, w_ref: float, lidar_xy: np.ndarray, v_max=0.8, w_max=1.2) -> Tuple[float,float]:
    if lidar_xy is None or lidar_xy.size==0:
        return min(v_ref,v_max), max(-w_max, min(w_ref, w_max))
    r = np.hypot(lidar_xy[:,0], lidar_xy[:,1])
    front = r[(lidar_xy[:,0]>0) & (np.abs(lidar_xy[:,1])<0.5)]
    dmin = np.min(front) if front.size>0 else 999.0
    if dmin < 0.8:
        return 0.0, (0.8 if np.mean(lidar_xy[:,1])>0 else -0.8)
    return min(v_ref,v_max), max(-w_max, min(w_ref, w_max))
