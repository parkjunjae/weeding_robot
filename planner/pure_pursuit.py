import math
from typing import List, Tuple


class PurePursuit:
    def __init__(self, lookahead: float=0.8):
        self.Ld = lookahead
    def compute(self, pose: Tuple[float,float,float], path: List[Tuple[float,float]], idx: int, v_des=0.6):
        if idx >= len(path):
            return 0.0, 0.0, idx
        x,y,th = pose
        gx, gy = path[idx]
        dx = math.cos(-th)*(gx - x) - math.sin(-th)*(gy - y)
        dy = math.sin(-th)*(gx - x) + math.cos(-th)*(gy - y)
        if dx < 0:
            return 0.2, (1.0 if dy>0 else -1.0), idx
        curvature = 2.0*dy/max(1e-3, self.Ld*self.Ld)
        w = curvature * v_des
        if ((gx-x)**2 + (gy-y)**2)**0.5 < self.Ld*0.7:
            idx = min(idx+1, len(path)-1)
        return v_des, max(-1.5, min(1.5, w)), idx
