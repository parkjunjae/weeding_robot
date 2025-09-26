import numpy as np
class OccGrid:
    def __init__(self, res=0.2, w=120, h=120):
        self.res=res; self.w=w; self.h=h
        self.grid = np.zeros((h,w), np.uint8)
    def world_to_grid(self, x,y):
        cx, cy = self.w//2, self.h//2
        return int(cx + x/self.res), int(cy - y/self.res)
    def set_occupied(self, x,y):
        ix,iy = self.world_to_grid(x,y)
        if 0<=ix<self.w and 0<=iy<self.h:
            self.grid[iy,ix]=255