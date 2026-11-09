"""실물 집기 계산기(grasp_test · reach_model · sag_model)의 팔 기구학 상수 + 비교용 정답 → JSON"""
import os  # NT_* 경로 환경변수 (tools/arm_real/README_KO.md)
import json, math, sys, subprocess
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE)); sys.path.insert(0, os.environ.get('REACH_DIR', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'reach')))
import pose_safety as ps, arm_pose_fk as fk, sag_model, grasp_test as gt
C = ps.Checker(); M = C.M
f = M.fk
js = [dict(T=j['T'].tolist(), axis=(np.asarray(j['axis'], float) / np.linalg.norm(j['axis'])).tolist()) for j in f.js]
rng = [[max(C.tick_rng[k][0], C.ros_rng[k][0]), min(C.tick_rng[k][1], C.ros_rng[k][1])] for k in range(5)]
q5_rest = fk.ticks_to_q({'11': 4084, '12': 1102, '13': 2668, '14': 1971, '16': 2063, '15': 3504})[0][4]
kin = dict(T_base_armbase=f.T_base_armbase.tolist(), joints=js, tip5=M.tip5.tolist(), app5=M.app5.tolist(), close5=M.close5.tolist(),
           j1_xy=M.j1_xy.tolist(), lo=M.lo.tolist(), hi=M.hi.tolist(), template_q=np.asarray(M.template['q']).tolist(), q5_rest=q5_rest,
           joint_map=[dict(ref_tick=a['ref_tick'], q_deg_at_ref=a['q_deg_at_ref'], sign=a['sign']) for a in fk.JM['arm']],
           tick_range=rng, sag_c_rad_per_nm=sag_model.c_rad(), base_z_floor=0.072)
(HERE / 'out').mkdir(exist_ok=True)
(HERE / 'out' / 'arm_kinematics.json').write_text(json.dumps(kin, indent=1))
# 정답: grasp_test 로 여러 퍽 자리 (h40 · h10 · h-30)
gold = []
for x, y, yaw in [(0.235, 0.0, 0.0), (0.2102, 0.038, -4.0), (0.25, -0.03, 10.0), (0.22, 0.05, 30.0), (0.27, 0.0, -20.0), (0.24, 0.07, 45.0)]:
    out = HERE / 'out' / 'gold_tmp.json'
    r = subprocess.run([sys.executable, str(HERE / 'grasp_test.py'), '--x', str(x), '--y', str(y), '--heights', '40', '10', '-30', '--yaw', str(yaw), '--out', str(out)],
                       capture_output=True, text=True, cwd=HERE)
    if r.returncode:
        gold.append(dict(x=x, y=y, yaw=yaw, ok=False)); continue
    P = json.loads(out.read_text())['poses']
    for p in P:
        q = [math.radians(a['q_deg_at_ref'] + a['sign'] * (t - a['ref_tick']) * 360 / 4096) for a, t in zip(fk.JM['arm'], p['ticks'])]
        tip, ax, cl, _ = M.grip(np.array(q))
        tau = sag_model.torques(M, np.array(q), 0.0)
    gold.append(dict(x=x, y=y, yaw=yaw, ok=True, poses=P))
# FK 정답 몇 개
fkg = []
for t in ([4084, 1102, 2668, 1971, 2063], [4208, 2359, 1637, 2170, 2184], [3471, 1838, 2082, 2246, 2415], [4030, 1965, 1728, 2233, 1974]):
    q = np.array([math.radians(a['q_deg_at_ref'] + a['sign'] * (tt - a['ref_tick']) * 360 / 4096) for a, tt in zip(fk.JM['arm'], t)])
    tip, ax, cl, _ = M.grip(q)
    tau = sag_model.torques(M, q, 0.0, 0.017)
    fkg.append(dict(ticks=t, q=q.tolist(), tip=tip.tolist(), app=ax.tolist(), close=cl.tolist(), tau=[tau[2], tau[3], tau[4]],
                    enc=gt.q_to_ticks(sag_model.encoder_goal_q(M, q, 0.0, held_kg=0.017))))
(HERE / 'out' / 'arm_kin_gold.json').write_text(json.dumps(dict(ik=gold, fk=fkg), indent=1))
print('ok', sum(g['ok'] for g in gold), '/', len(gold))
