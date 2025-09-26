import json, math, time, numpy as np
from pathlib import Path
import yaml
from weedbot.controller.diff_drive import DiffDrive, PID
from weedbot.planner.coverage import parallel_sweep
from weedbot.planner.pure_pursuit import PurePursuit
from weedbot.planner.local_dwa import dwa_lite
# from weedbot.sensors.livox_source import LivoxSource
# import lidar_adapter
# from weedbot.sensors.eg25g_gps import EG25G
# from weedbot.sensors.imu_source import SimpleIMU




def load_params(path: str) -> dict:
    with open(path,'r') as f: return yaml.safe_load(f)


if __name__=="__main__":
    base = Path(__file__).resolve().parents[2]
    p = load_params(str(base/"weedbot/common/params.yaml"))


# 1) 커버리지 경로 생성 (지금은 safety.geofence를 작업영역으로 사용)
    poly = p['safety']['geofence']
    track = p['planner']['blade_width']*(1.0 - p['planner']['overlap'])
    path = parallel_sweep(poly, track, heading_deg=0.0)
    pp = PurePursuit(lookahead=p['planner']['pp_lookahead'])


# 2) 제어기 (실사용시 MCU 시리얼 모듈로 교체)
    drive = DiffDrive(p['motor']['tread'])
    pidL = PID(**p['motor']['pid']); pidR = PID(**p['motor']['pid'])


# 3) 임시 상태(로컬라이제이션 붙기 전): 엔코더 대신 가상 적분
    x=y=yaw=0.5 # 시작 위치 예시
    idx=0; dt=0.02


for step in range(800):
    v_ref, w_ref, idx = pp.compute((x,y,yaw), path, idx, v_des=0.6)
    # LiDAR 붙으면 여기서 xy=cloud→로봇좌표로 변환 전달
    xy = np.zeros((0,2), np.float32)
    v_cmd, w_cmd = dwa_lite(v_ref, w_ref, xy, p['planner']['dwa']['v_max'], p['planner']['dwa']['w_max'])
    # 차동 변환 + (임시) 가상 PID → 바로 상태 갱신(실차는 엔코더로 측정)
    vl, vr = drive.to_wheels(v_cmd, w_cmd)
    x += ((vl+vr)*0.5)*math.cos(yaw)*dt
    y += ((vl+vr)*0.5)*math.sin(yaw)*dt
    yaw += (vr-vl)/max(1e-3, p['motor']['tread'])*dt
    print(f"step={step:03d} idx={idx} pose=({x:.2f},{y:.2f},{math.degrees(yaw):.1f}) v={v_cmd:.2f} w={w_cmd:.2f}")
    time.sleep(dt)
