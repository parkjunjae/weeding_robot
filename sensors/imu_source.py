from serial import Serial
from dataclasses import dataclass
from typing import Optional
from weedbot.common.time_sync import now


@dataclass
class ImuSample:
    ax: float; ay: float; az: float; gx: float; gy: float; gz: float; ts: float


class SimpleIMU:
    def __init__(self, port: str, baud: int=115200):
        self.ser = Serial(port, baudrate=baud, timeout=0.02)
        self.bias = (0,0,0,0,0,0)
    def calib(self, sec=8.0):
        import time
        ax=ay=az=gx=gy=gz=n=0.0
        t_end = now()+sec
        while now()<t_end:
            s = self.ser.readline().decode(errors='ignore').strip()
            if not s: continue
            try:
                a=[float(x) for x in s.split(',')]
                ax+=a[0]; ay+=a[1]; az+=a[2]; gx+=a[3]; gy+=a[4]; gz+=a[5]; n+=1
            except: pass
        if n>0: self.bias=(ax/n,ay/n,az/n,gx/n,gy/n,gz/n)
    def read(self) -> Optional[ImuSample]:
        s = self.ser.readline().decode(errors='ignore').strip()
        if not s: return None
        try:
            a=[float(x) for x in s.split(',')]
            a=[a[i]-self.bias[i] for i in range(6)]
            return ImuSample(a[0],a[1],a[2],a[3],a[4],a[5], now())
        except: return None