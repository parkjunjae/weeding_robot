from serial import Serial
from typing import Optional, Tuple
from weedbot.common.enu import wgs84_to_enu
from weedbot.common.time_sync import now


class EG25G:
    def __init__(self, port_at: str, port_nmea: str, baud: int=115200, lat0=0.0, lon0=0.0, yaw0=0.0):
        self.at = Serial(port_at, baudrate=baud, timeout=1.0)
        self.nmea = Serial(port_nmea, baudrate=baud, timeout=0.1)
        self.lat0, self.lon0, self.yaw0 = lat0, lon0, yaw0
        self.last = None
    def atcmd(self, s: str):
        self.at.write((s+'\r\n').encode()); return self.at.read_until(b'OK')
    def start(self):
        self.atcmd('AT'); self.atcmd('AT+QGPS=1'); self.atcmd('AT+QGPSCFG="outport","usbnmea"')
    def stop(self):
        self.atcmd('AT+QGPSEND')
    def _dm2deg(self, dm: str):
        if not dm: return None
        v=float(dm); d=int(v/100); m=v-d*100; return d+m/60.0
    def read_fix(self) -> Optional[Tuple[float,float,int,float]]:
        line = self.nmea.readline().decode(errors='ignore').strip()
        if not line: return self.last
        if line.startswith('$GNRMC') or line.startswith('$GPRMC'):
            f = line.split(','); st=f[2] if len(f)>2 else 'V'
            lat = self._dm2deg(f[3]) * (1 if f[4]=='N' else -1) if len(f)>5 and f[3] else None
            lon = self._dm2deg(f[5]) * (1 if f[6]=='E' else -1) if len(f)>6 and f[5] else None
            fix = 1 if st=='A' else 0
            if lat and lon:
                x,y = wgs84_to_enu(lat,lon,self.lat0,self.lon0,self.yaw0)
                self.last=(x,y,fix,now())
        return self.last