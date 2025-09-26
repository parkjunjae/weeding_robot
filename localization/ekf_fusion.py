import math
from typing import Optional, List, Tuple
from weedbot.common.time_sync import now


class EKFFusion:
    def __init__(self):
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self._last_ts = None
    def step(self, lidar_cloud: Tuple[float,object], imu_samples: List[object], gps_enu: Optional[Tuple[float,float,int,float]]):
        # 1) IMU yaw 적분(러프)
        if imu_samples:
            imu_samples.sort(key=lambda s: s.ts)
            for s in imu_samples:
                if self._last_ts is None: self._last_ts = s.ts
                dt = max(1e-3, s.ts - self._last_ts)
                self.yaw += s.gz * dt
                self._last_ts = s.ts
        # 2) GPS가 있으면 위치 보정(실사용에선 가중/공분산 필요)
        if gps_enu and gps_enu[2]>0:
            self.x, self.y = gps_enu[0], gps_enu[1]
    def pose(self):
        return (self.x, self.y, self.yaw, now())