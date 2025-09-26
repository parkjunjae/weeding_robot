#!/usr/bin/env python3
import sys, time, numpy as np
from pathlib import Path

# build 폴더 모듈 강제 사용
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "build"))
import lidar_odom_ndt as ndt

print("loaded:", ndt.__file__)
print("build_tag:", getattr(ndt, "build_tag", "NO_TAG"))

odom = ndt.LidarOdomNDT()
odom.set_params(voxel=0.0, ndt_res=0.35, max_iter=80, step_size=0.05)

# 격자 평면 + 소노이즈
xs = np.linspace(-5, 5, 120)
ys = np.linspace(-3, 3, 80)
X, Y = np.meshgrid(xs, ys)
Z = np.zeros_like(X)
p0 = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1).astype(np.float32)
p0 += np.random.randn(*p0.shape).astype(np.float32) * 0.003

def apply_T(points, T):
    N = points.shape[0]
    homo = np.c_[points, np.ones((N,1), np.float32)]
    return (homo @ T.T)[:, :3].astype(np.float32)

def T_sensor_move(dx, dy, dyaw):
    """
    센서가 +dx,+dy,+dyaw 이동했다고 가정할 때,
    포인트에는 '반대' 변환을 적용해야 동일한 관측이 됩니다.
    """
    c, s = np.cos(dyaw), np.sin(dyaw)
    # 포인트 프레임에서는 역회전·역평행이동
    Rinv = np.array([[ c, s, 0],
                     [-s, c, 0],
                     [ 0, 0, 1]], np.float32)
    T = np.eye(4, dtype=np.float32)
    T[:3,:3] = Rinv
    T[0,3] = -(dx*c - dy*s)
    T[1,3] = -(dx*s + dy*c)
    return T

# 기준 프레임 푸시
r0 = odom.push_frame(p0, time.monotonic())
print("r0.ok:", r0.ok)

# === [A] 로봇 +0.25m 전진 (→ 포인트는 -0.25m로 이동시켜야 함) ===
pA = apply_T(p0, T_sensor_move(+0.25, 0.0, 0.0))
rA = odom.push_frame(pA, time.monotonic())
print(f"[A] robot +0.25m → expect dx ~ +0.25 | got dx={rA.dx:.3f}, dy={rA.dy:.3f}, dyaw={rA.dyaw:.4f}, fit={rA.fitness:.4f}")
print("odom pose A:", odom.odom_pose())

# === [B] 포인트를 +0.25m로 이동 (→ 로봇은 -0.25m 후진과 동일) ===
odom.reset()
r0b = odom.push_frame(p0, time.monotonic())
pB = p0.copy(); pB[:,0] += 0.25
rB = odom.push_frame(pB, time.monotonic())
print(f"[B] points +0.25m → expect dx ~ -0.25 | got dx={rB.dx:.3f}, dy={rB.dy:.3f}, dyaw={rB.dyaw:.4f}, fit={rB.fitness:.4f}")
print("odom pose B:", odom.odom_pose())
