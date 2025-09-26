from dataclasses import dataclass

@dataclass
class PID:
    kp: float=0.6; ki: float=0.1; kd: float=0.0; dt: float=0.02
    _e: float=0.0; _i: float=0.0
    def update(self, target: float, meas: float) -> float:
        e = target - meas
        self._i += e*self.dt
        d = (e - self._e)/self.dt
        self._e = e
        u = self.kp*e + self.ki*self._i + self.kd*d
        return max(0.0, min(1.0, u)) # PWM 0..1
    
    
class DiffDrive:
    def __init__(self, tread: float):
        self.B = tread
    def to_wheels(self, v: float, w: float):
        return (v - w*self.B*0.5), (v + w*self.B*0.5)