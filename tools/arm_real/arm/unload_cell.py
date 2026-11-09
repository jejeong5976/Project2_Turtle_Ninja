#!/usr/bin/env python3
"""적재함 칸에서 퍽 꺼내 박스에 내려놓기 (계산만) — unload_cycle.sh 가 부름. 넣기(rack_load.plan_rack)의 길을 거꾸로.

  1 단계  $ARM_PY unload_cell.py L2 --stage hover          → lc_unhover.checked.json
          받침 → 박스 위(들기) → 칸 위(over_cell, 그리퍼 '놓기' 간격으로 열고 멈춤)  · 끝 = 칸 위 (end_here)
  2 단계  ... unload_cell.py L2 --stage pick --run runs/runner_<1 단계> [--jog lc_jog_out.json]  → lc_unpick.checked.json
          (미리: --stage pick --planned lc_unhover.json --out lc_unpick0 — 1 단계 계획의 칸 위 자세에서)
          칸 위(키보드로 맞춘 자리) → 칸 안(손끝 = 퍽 윗면 − 25/20/15 mm, 닿는 가장 깊은 것) 쥐기 → 칸 위 → 넣을 때 길 거꾸로 → 박스 위 → 내려놓기 → 받침

칸 자리 = rack_cells.json xy + place_offset (넣을 때 맞춘 보정 = 팔 오차 + 그때 쥔 자리. 꺼낼 때는 칸 위에서 키보드로 다시 맞춤).
내려놓는 자리 = --drop "x,y,yaw" [mm, mm, °] (기본: 마지막 적재 때 퍽을 집은 박스 자리).
"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import grasp_test as gt           # noqa: E402
import pose_safety as ps          # noqa: E402
import rack_load                  # noqa: E402
import sag_model                  # noqa: E402

PY = os.environ.get('ARM_PY', sys.executable)
PICK_DEPTHS = (25, 20, 15)        # [mm] 칸 안 쥐기 깊이 후보: 손끝 = 퍽 윗면 아래 (모델. 나사 조인 뒤 실제 팔은 약 5 mm 높아 실제는 5 mm 얕음)
DROP_DEPTH = 23                   # [mm] 박스에 내려놓기: 같은 쥔 자리면 퍽 바닥이 박스 윗면 약 2 mm 위
HOVER_UP = (0.015, 0.010, 0.006)  # [m] 빈손 칸 위 멈춤 = 넣기 칸 위보다 이만큼 위 (닿는 것 중 가장 높게) (손끝이 꺼낼 퍽 윗면 약 30 mm 위 — L2: 넣기 높이면 [J3−2] 손가락 ↔ 퍽 1 mm)
DROP_DEFAULT = '210.2,38.0,-4.0'
# 칸 위로 다가가는 앞 점 [m] (칸 위 높이, 칸 가운데 기준) — 차례로 시험. L2: 박스 쪽 · 안쪽 비스듬히만 통과
APPROACH = ((0.02, -0.02, 0), (0.02, 0.02, 0), (0.04, -0.04, 0), (0.04, 0.04, 0), (-0.02, -0.02, 0), (-0.02, 0.02, 0), (0.03, 0, 0), (-0.03, 0, 0), (0, 0.03, 0), (0, -0.03, 0))  #  L2 적재 때 퍽 자리 (puck_locate runner_20261101T064426)


def enc(C, t, kg):
    q, fq = C._qf(t)
    return gt.q_to_ticks(sag_model.encoder_goal_q(C.M, q, fq, held_kg=kg))


def drop_poses(a):
    x, y, yaw = (float(v) for v in a.drop.split(','))
    tmp = HERE / f'{a.out}_drop_ik.json'
    r = subprocess.run([PY, str(HERE / 'grasp_test.py'), '--x', f'{x / 1000:.4f}', '--y', f'{y / 1000:.4f}', '--heights', '40', '10', f'-{DROP_DEPTH}',
                        '--yaw', str(yaw), '--note', '꺼내기 내려놓는 자리', '--out', str(tmp)], text=True, capture_output=True)
    if r.returncode != 0:
        sys.exit(f'내려놓는 자리 ({x}, {y}) IK 안 됨: {r.stderr.strip()[-200:]}')
    ik = json.loads(tmp.read_text())
    P = {p['name']: p for p in ik['poses']}
    return P['h40'], P['h10'], P[f'h-{DROP_DEPTH}'], [o for o in ik['obstacles'] if o['name'] != 'puck']


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('cell')
    ap.add_argument('--stage', choices=('hover', 'pick'), required=True)
    ap.add_argument('--run', type=Path, help='pick: 1 단계 실행 (칸 위 멈춤 자세)')
    ap.add_argument('--jog', type=Path, help='pick: jog_hover.py 결과 — 키보드로 맞춘 자리 · 양')
    ap.add_argument('--planned', type=Path, help='pick: 1 단계 계획 파일(lc_unhover.json)의 칸 위 자세에서 시작 — 실행 전에 미리 계산 (키보드로 안 옮겼을 때 씀)')
    ap.add_argument('--drop', default=DROP_DEFAULT)
    ap.add_argument('--out', default=None)
    a = ap.parse_args()
    cell = a.cell.upper()
    a.out = a.out or ('lc_unhover' if a.stage == 'hover' else 'lc_unpick')
    C = ps.Checker()
    # 꺼낼 때는 놓지 않으므로 칸 위 높이를 넣기 놓는 높이(PLACE_GAP 10 mm) 에 묶지 않음 — 10 이면 칸 위가 4 mm 높아져 L5 · L6 길이 막힘
    rack_load.PLACE_GAP = 0.006
    rc = rack_load.load_cells()
    if not rc['cells'][cell]['state'].startswith('full'):
        sys.exit(f'칸 {cell} 은 빈 칸 ({rc["cells"][cell]["state"]}) — 꺼낼 퍽 없음')
    up, mid, low, box_obs = drop_poses(a)
    lift = up['ticks']
    rce = json.loads(json.dumps(rc))
    rce['cells'][cell]['state'] = 'empty'                              # 계획 · 장애물에서 꺼낼 퍽은 뺌 (손가락이 감쌈)
    # 박스 ↔ 칸 위 길은 1 단계와 같은 보정(키보드 전)으로 — 키보드로 옮긴 자리로 길을 새로 찾으면 다른 길이 나오거나 막힘 (B: +4 mm 로 A 컵에 1 mm)
    rp, desc = rack_load.plan_rack(C, cell, lift, rce)
    k = next(i for i, p in enumerate(rp) if p['name'] == 'over_cell')
    vias = [p['ticks'] for p in rp[:k]]
    ta0 = ta = rp[k]['ticks']
    jog = json.loads(a.jog.read_text()) if a.jog else None
    if jog:
        if jog['cell'] != cell:
            sys.exit(f"키보드 맞추기 결과가 칸 {jog['cell']} 것 — {cell} 아님")
        off = rce['cells'][cell].get('place_offset', [0.0, 0.0])
        rce['cells'][cell]['place_offset'] = [off[0] + jog['shift'][0] / 1000, off[1] + jog['shift'][1] / 1000]
        if jog.get('yaw'):                                                # 키보드로 손목을 돌린 만큼
            rce['cells'][cell]['yaw_deg'] = rack_load.cell_yaw(rce, rce['cells'][cell]) + jog['yaw']
    sl = dict(speed=15.0)
    held = dict(held_kg=0.017)
    rs = rack_load.shifted(rce, cell)
    x, y = rs['cells'][cell]['xy']
    yaw = rack_load.cell_yaw(rs, rs['cells'][cell])

    def near_ik(z, ref):                                                 # 같은 손목 · 몸통 갈래에서 ref 에 가장 가까운 수직 손 IK (없으면 None)
        sols = [s_[0] for s_ in rack_load.ik_vertical(C, (x, y, z), yaw) if abs(s_[0][4] - ref[4]) < 200 and abs(s_[0][0] - ref[0]) < 200
                and rack_load.in_range(C, enc(C, s_[0], 0.0))]
        return min(sols, key=lambda t: np.abs(np.array(t) - np.array(ref)).max()) if sols else None
    z_above = rc['floor_z'] + rack_load.PLACE_GAP + rack_load.PUCK_H - rack_load.GRIP_DEPTH + rack_load.APPROACH_DZ
    if jog:                                                              # 키보드로 옮긴 칸 위 (같은 갈래)
        ta = near_ik(z_above, ta0)
        if ta is None:
            sys.exit(f'칸 {cell}: 키보드로 옮긴 칸 위 IK 없음 (팔은 칸 위 그대로 — --return 으로 받침)')
    th = next((t for t in (near_ik(z_above + dz, ta) for dz in HOVER_UP) if t), ta)
    if a.stage == 'hover':
        poses = [dict(name='lift', ticks=lift, enc=enc(C, lift, 0.0), stop=False)]
        # 마지막 경유점 → 칸 위: 관절 직선이면 꺼낼 퍽 위에서 손가락이 처짐 가설로 1 mm 까지 내려감 (L2) →
        #   칸 위 높이의 비스듬한 앞 점을 거쳐 손끝 곧은 선 (1 cm 마다 IK). 꺼낼 퍽까지 장애물로 넣고 통과하는 첫 방향
        last = vias[-1] if vias else lift
        ps.set_obstacles(box_obs + rack_load.cup_obstacles(rc, cell))
        tip = np.asarray(C.M.grip(C._qf(th)[0])[0])
        clear = lambda ch: not any(C.path(u, v, grip=ps.GRIP_JM['closed'], rack_zone=False) for u, v in zip(ch, ch[1:]))
        line = [th] if clear([last, th]) else None
        for dz in (0.03, 0.05, 0.07) if line is None else ():           # 칸 바로 위 높은 곳 → 수직으로 내려옴 (팔 가까운 칸, A: 곧장 오면 손가락이 A 퍽 위 0 mm)
            tu = near_ik(z_above + dz, th)
            if tu and clear([last, tu, th]):
                line = [tu, th]; break
        for off in APPROACH if line is None else ():
            # 옆 앞 점까지는 관절 직선(그 점의 IK), 거기서 칸 위까지만 손끝 곧은 선 — 박스에서부터 곧은 선은 IK 가 안 이어짐 (A)
            pre = [t_ for t_, _ in rack_load.ik_vertical(C, tip + np.array(off), yaw)]
            pre = min(pre, key=lambda t_: np.abs(np.array(t_) - np.array(th)).max()) if pre else None
            if pre is None or not clear([last, pre]):
                continue
            ln = rack_load._chain_back(C, pre, th, [tip], yaw, step=0.01)
            if ln is not None and clear([pre] + ln):
                line = [pre] + ln; break
        if line is None:
            sys.exit(f'칸 {cell}: 칸 위로 다가가는 길 없음 (꺼낼 퍽 · 옆 칸 퍽)')
        poses += [dict(name=f'carry{i + 1}', ticks=v, enc=enc(C, v, 0.0), stop=False, rack=True, speed=25.0) for i, v in enumerate(vias + line[:-1])]
        poses += [dict(name='over_cell', ticks=th, enc=enc(C, th, 0.0), stop=True, rack=True, grip='release', speed=25.0)]
        obstacles = box_obs + rack_load.cup_obstacles(rc, cell)         # 꺼낼 퍽도 장애물 (칸 위 손끝은 퍽 윗면 약 15 mm 위)
        extra = dict(end_here=True)
    else:
        if a.planned:
            t0 = json.loads(a.planned.read_text())['poses'][-1]['ticks']
        elif a.run:
            rec = [json.loads(l) for l in (a.run / 'log.jsonl').read_text().splitlines() if l.strip()]
            t0 = [round(v) for v in next(x for x in rec[::-1] if x.get('event') == 'stop' and x.get('name') == 'over_cell')['measured']]
        else:
            sys.exit('--stage pick 은 --run (1 단계 실행) 또는 --planned 필요')
        t0 = jog['ticks'] if jog else t0
        # 쥐는 깊이: 깊게부터 — 바깥 칸(L2 · L6 · B) 은 팔이 덜 닿고 손가락이 컵 테두리에 붙어 얕게 (L2: 25 mm 면 테두리 2~5 mm)
        ps.set_obstacles(box_obs + rack_load.cup_obstacles(rce, cell))
        tp = dm = None
        for dm in PICK_DEPTHS:
            t = near_ik(rc['floor_z'] + rack_load.PUCK_H - dm / 1000, ta)
            if t and not any(C.pose(t, grip=ps.GRIP_JM[g], rack_zone=False)[0] for g in ('release', 'closed')) \
                    and not C.path(ta, t, grip=ps.GRIP_JM['release'], rack_zone=False):
                tp = t; break
        if tp is None:
            sys.exit(f'칸 {cell}: 쥘 수 있는 깊이 없음 {PICK_DEPTHS} mm (팔은 칸 위 그대로 — --return 으로 받침)')
        print(f'칸 {cell}: 퍽 윗면 아래 {dm} mm 를 쥠')
        poses = [dict(name='over_cell', ticks=t0, enc=t0, stop=False, rack=True, grip='release'),
                 dict(name='over_adj', ticks=th, enc=enc(C, th, 0.0), stop=False, rack=True, **sl),
                 dict(name='over_low', ticks=ta, enc=enc(C, ta, 0.0), stop=False, rack=True, **sl),
                 dict(name='pick_in', ticks=tp, enc=enc(C, tp, 0.0), stop=False, rack=True, grip='close', speed=10.0),
                 dict(name='pick_up', ticks=ta, enc=enc(C, ta, 0.017), stop=False, rack=True, **held, speed=10.0)]
        if ta != ta0:                                                    # 1 단계 칸 위로 (키보드로 옮긴 몇 mm 를 되돌림) → 넣을 때 길 거꾸로
            h = rack_load.path_hits(C, ta, ta0, rack_load.obstacles(rce, cell), skip=('plate',))
            if h:
                sys.exit(f'칸 {cell}: 쥔 퍽을 칸 위 원래 자리로 옮기는 길 ✗ {h} (팔은 칸 위 그대로 — --return 으로 받침)')
            poses.append(dict(name='over_back', ticks=ta0, enc=enc(C, ta0, 0.017), stop=False, rack=True, **held, speed=10.0))
        # 빠져나오기 (A: 칸 위에서 박스로 곧장 가다 쥔 퍽이 자기 컵 박스 쪽 모서리를 긁음 — 계획기는 목표 컵을 장애물에서 뺌):
        #   같은 높이로 다음 경유점(없으면 박스) 쪽 수평 직선 3 · 5 cm 를 먼저, 그 칸 컵까지 넣고 쥔 퍽 검사
        obs_all = rack_load.obstacles(rce, None)
        nxt = (list(reversed(vias)) + [lift])[0]
        t_out = ta0
        tip0 = np.asarray(C.M.grip(C._qf(ta0)[0])[0]); tipn = np.asarray(C.M.grip(C._qf(nxt)[0])[0])
        dirv = np.r_[(tipn - tip0)[:2], 0.0]; dirv /= max(np.linalg.norm(dirv), 1e-9)
        exit_line = None
        for dist in (0.03, 0.05, 0.07):
            ln, seed, last_t = [], C._qf(ta0)[0], list(ta0)
            for k in range(1, int(round(dist / 0.01)) + 1):              # 1 cm 마다 수직 손 IK (앞 해 씨앗 · 250 틱 안 이어짐)
                nx_ = rack_load._next(C, tip0 + dirv * 0.01 * k, yaw, seed, last_t)
                if nx_ is None:
                    ln = None; break
                ln.append(nx_[0]); seed, last_t = nx_[1], nx_[0]
            if not ln:
                continue
            ch = [ta0] + ln
            if any(rack_load.path_hits(C, u, v, obs_all, skip=('plate',)) for u, v in zip(ch, ch[1:])) \
                    or rack_load.path_hits(C, ln[-1], nxt, obs_all, skip=('box',)):
                continue
            exit_line = ln; break
        if exit_line is None:
            sys.exit(f'칸 {cell}: 쥔 퍽이 자기 컵 테두리를 넘어 빠져나올 길 없음 (팔은 칸 위 그대로 — --return 으로 받침)')
        poses += [dict(name=f'exit{i + 1}', ticks=v, enc=enc(C, v, 0.017), stop=False, rack=True, **held, speed=10.0) for i, v in enumerate(exit_line)]
        poses += [dict(name=f'back{i + 1}', ticks=v, enc=enc(C, v, 0.017), stop=False, rack=True, **held, speed=25.0) for i, v in enumerate(reversed(vias))]
        poses += [dict(name='lift', ticks=lift, enc=enc(C, lift, 0.017), stop=False, **held, speed=25.0),
                  dict(name='down2', ticks=mid['ticks'], enc=mid['enc'], stop=False, **held, **sl),
                  dict(name='drop', ticks=low['ticks'], enc=low['enc'], stop=False, grip='open', **held, **sl),
                  dict(name='up2', ticks=mid['ticks'], enc=mid['enc'], stop=False, **sl),
                  dict(name='clear', ticks=up['ticks'], enc=up['enc'], stop=False, grip='close', **sl)]
        obstacles = box_obs + rack_load.cup_obstacles(rce, cell)
        extra = dict(start_here=True)
    out = HERE / f'{a.out}.json'
    out.write_text(json.dumps(dict(note=f'칸 {cell} 꺼내기 {a.stage} · 내려놓기 {a.drop} · {desc}', obstacles=obstacles, poses=poses, **extra),
                              ensure_ascii=False, indent=1))
    print(f'칸 {cell} 꺼내기 {a.stage}: {desc}' + (f" · 키보드 {jog['shift']} mm" if jog else ''))
    res = subprocess.run([PY, str(HERE / 'pose_safety.py'), str(out)], text=True, capture_output=True)
    print('\n'.join(l for l in res.stdout.splitlines() if '✗' in l or '전체' in l or '저장' in l))
    if res.returncode != 0:
        sys.exit('안전 검사 실패 — 실행 금지' + (' (팔은 칸 위 그대로 — --return 으로 받침)' if a.stage == 'pick' else ''))


if __name__ == '__main__':
    main()
