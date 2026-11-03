#!/usr/bin/env python3
"""관찰 자세 LOOK · LOOK_BARCODE: 받침 → 자세 (end_here), 자세 → 받침 (start_here, HOME 용) 을 pose_safety 로 검사해 만든다."""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, os.environ.get('NT_REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
import pose_safety as ps  # noqa: E402

LOOKS = dict(look=[4030, 1965, 1728, 2233, 1974], look_barcode=[4174, 2249, 1493, 1284, 1675])
OUT = HERE / 'out'
OUT.mkdir(exist_ok=True)

C = ps.Checker()
for name, t in LOOKS.items():
    pose = [dict(name=name, ticks=t, stop=True)]
    ok1 = ps.check_file(C, pose, str(OUT / f'{name}.checked.json'), start_here=False, end_here=True)
    ok2 = ps.check_file(C, pose, str(OUT / f'home_from_{name}.checked.json'), start_here=True, end_here=False)
    print(f'### {name}: 받침 → 자세', '통과' if ok1 else '✗', '· 자세 → 받침', '통과' if ok2 else '✗', flush=True)
