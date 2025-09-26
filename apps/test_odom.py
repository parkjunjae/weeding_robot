import numpy as np, time
import lidar_odom_ndt as ndt

odom = ndt.LidarOdomNDT()
odom.set_params(voxel=0.0, ndt_res=0.4, max_iter=60, step_size=0.1)  # 튜닝 ↓

# x∈[-5,5], y∈[-3,3] 격자 평면 + 아주 작은 노이즈
xs = np.linspace(-5, 5, 120)
ys = np.linspace(-3, 3, 80)
X, Y = np.meshgrid(xs, ys)
Z = np.zeros_like(X)
p0 = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1).astype(np.float32)
p0 += np.random.randn(*p0.shape).astype(np.float32) * 0.005  # 5mm 노이즈

r0 = odom.push_frame(p0, time.monotonic())
print("r0.ok:", r0.ok)

# +0.25 m 이동
p1 = p0.copy()
p1[:,0] += 0.25
r1 = odom.push_frame(p1, time.monotonic())
print(f"r1: ok={r1.ok} dx={r1.dx:.3f} dy={r1.dy:.3f} dyaw={r1.dyaw:.4f} fit={r1.fitness:.4f}")
print("odom pose:", odom.odom_pose())
