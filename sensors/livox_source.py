import threading, numpy as np
from collections import deque


class LivoxSource:
    def __init__(self, adapter_module, max_keep=1):
        self.ad = adapter_module
        self._q = deque(maxlen=max_keep)
        self._lock = threading.Lock()
        self._running = False
    def _cb(self, pts: np.ndarray):
        if pts is None or pts.size==0: return
        with self._lock:
            self._q.append((float(self.ad.now()), pts.astype(np.float32, copy=False)))
    def start(self):
        if self._running: return
        if not self.ad.LivoxLidarSdkInit(): raise RuntimeError('Livox init failed')
        if not self.ad.SetLivoxLidarPointCloudCallBack(self._cb): raise RuntimeError('CB set failed')
        if not self.ad.LivoxLidarSdkStart(): raise RuntimeError('Livox start failed')
        self._running=True
    def latest(self):
        with self._lock:
            return self._q[-1] if self._q else (0.0, np.zeros((0,3), np.float32))
    def stop(self):
        if self._running:
            try: self.ad.LivoxLidarSdkUninit()
            finally: self._running=False