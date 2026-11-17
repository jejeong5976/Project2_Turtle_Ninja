#!/usr/bin/env python3
"""test/nudge_ref.hpp 만들기: numpy_policy.py 와 같은 식으로 기대값
  (1) nudge_n1b.npz 에 고정 관측 5 개 → 행동 6 (C++ MlpPolicy 와 1e-5 비교)
  (2) 작은 합성 npz 두 벌 (stored · deflate) 바이트 + 같은 입력의 기대 출력 (zip · npy 읽기 시험)"""
import io, sys, zipfile
import numpy as np

def pol(z, obs):
    mean, std, eps = z['mean'].astype(np.float64), z['std'].astype(np.float64), float(z['eps'])
    n = int(z['n_layers'])
    x = (np.asarray(obs, np.float64) - mean) / (std + eps)
    for i in range(n):
        x = x @ z[f'W{i}'].astype(np.float64).T + z[f'b{i}'].astype(np.float64)
        if i < n - 1:
            x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
    return x

npz = sys.argv[1]
z = np.load(npz)
rng = np.random.default_rng(13)
m, s = z['mean'].astype(np.float64), z['std'].astype(np.float64)
obs = [np.zeros(86), m.copy(), m + s * rng.standard_normal(86), m + 2 * s * rng.standard_normal(86), rng.uniform(-1, 1, 86)]
out = [pol(z, o) for o in obs]

# 작은 합성 정책 (4 → 5 → 3, n_layers 2) — stored · deflate
W0 = rng.standard_normal((5, 4)).astype(np.float32); b0 = rng.standard_normal(5).astype(np.float32)
W1 = rng.standard_normal((3, 5)).astype(np.float32); b1 = rng.standard_normal(3).astype(np.float32)
small = dict(mean=rng.standard_normal(4).astype(np.float32), std=rng.uniform(0.5, 2, 4).astype(np.float32), eps=np.float32(0.01),
             n_layers=np.int32(2), activation=np.array('elu'), W0=W0, b0=b0, W1=W1, b1=b1)
blobs = {}
for name, fn in (('stored', np.savez), ('deflate', np.savez_compressed)):
    bio = io.BytesIO(); fn(bio, **small); blobs[name] = bio.getvalue()
    zz = zipfile.ZipFile(io.BytesIO(blobs[name])); assert all(i.compress_type == (0 if name == 'stored' else 8) for i in zz.infolist())
sobs = [np.array([0.3, -1.2, 2.0, 0.0]), np.array([-2.0, 0.5, 0.1, 1.5])]
zs = np.load(io.BytesIO(blobs['stored']))
sout = [pol(zs, o) for o in sobs]

def arr(v): return '{' + ', '.join(f'{x:.17g}' for x in np.ravel(v)) + '}'
def byts(b): return '{' + ', '.join(str(x) for x in b) + '}'
h = ['// 자동 생성 (gen_nudge_ref.py, numpy numpy_policy.py 와 같은 식) — 손으로 고치지 말 것', '#pragma once', '#include <vector>', '#include <cstdint>', 'namespace nudge_ref {']
h.append(f'inline const std::vector<std::vector<double>> kObs = {{{", ".join(arr(o) for o in obs)}}};')
h.append(f'inline const std::vector<std::vector<double>> kAct = {{{", ".join(arr(o) for o in out)}}};')
for k, b in blobs.items():
    h.append(f'inline const std::vector<uint8_t> kSmall_{k} = {byts(b)};')
h.append(f'inline const std::vector<std::vector<double>> kSmallObs = {{{", ".join(arr(o) for o in sobs)}}};')
h.append(f'inline const std::vector<std::vector<double>> kSmallAct = {{{", ".join(arr(o) for o in sout)}}};')
h.append('}  // namespace nudge_ref')
open(sys.argv[2], 'w').write('\n'.join(h) + '\n')
print('ok', [np.round(o, 4).tolist() for o in out[:2]])
