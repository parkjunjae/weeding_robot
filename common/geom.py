import math
from typing import Iterable, Tuple


Point = Tuple[float,float]


def rotate(p: Point, deg: float) -> Point:
    r = math.radians(deg); c,s = math.cos(r), math.sin(r)
    return (c*p[0] - s*p[1], s*p[0] + c*p[1])


def bbox(poly: Iterable[Point]):
    xs = [p[0] for p in poly]; ys = [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)