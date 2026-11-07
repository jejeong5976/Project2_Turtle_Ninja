"""export.py 의 policy_<반복>.npz 를 numpy 로 돌림 (torch 없음, Jetson 용). TorchScript 판과 같은 식:
  x = (obs − mean) / (std + eps) → [W x + b → ELU] × (층 − 1) → W x + b (= 평균 행동, [-1, 1] 자르기는 노드가 함)."""
import numpy as np


class NumpyPolicy:
    def __init__(self, path):
        z = np.load(path)
        self.mean, self.std, self.eps = z['mean'].astype(np.float64), z['std'].astype(np.float64), float(z['eps'])
        n = int(z['n_layers'])
        self.W = [z[f'W{i}'].astype(np.float64) for i in range(n)]
        self.b = [z[f'b{i}'].astype(np.float64) for i in range(n)]
        assert str(z['activation']) == 'elu', z['activation']

    def __call__(self, obs):
        """obs (D,) 또는 (B, D) → 행동 (3,) 또는 (B, 3)"""
        x = (np.asarray(obs, np.float64) - self.mean) / (self.std + self.eps)
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            x = x @ W.T + b
            if i < len(self.W) - 1:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))      # ELU (alpha 1)
        return x
