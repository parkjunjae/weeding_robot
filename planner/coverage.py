from typing import List, Tuple
from weedbot.common.geom import rotate, bbox


Point = Tuple[float,float]


def parallel_sweep(polygon: List[Point], track: float, heading_deg: float=0.0) -> List[Point]:
# 단순 사각형/볼록 다각형 가정. 복잡 폴리곤은 shapely 사용 권장.
    rot = [rotate(p, -heading_deg) for p in polygon]
    xmin,ymin,xmax,ymax = bbox(rot)
    path: List[Point] = []
    y = ymin; rev=False
    while y <= ymax + 1e-6:
        a=(xmin,y); b=(xmax,y)
        if rev: a,b=b,a
        path.append(rotate(a, heading_deg))
        path.append(rotate(b, heading_deg))
        y += track; rev = not rev
    return path
