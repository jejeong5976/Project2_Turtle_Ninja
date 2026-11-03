"""arm_kinematics.json → config/arm_kinematics.yaml (control_master 파라미터) + test/arm_kin_data.hpp (시험 정답)"""
import json, sys
from pathlib import Path
HERE = Path(__file__).resolve().parent
k = json.loads((HERE / 'out' / 'arm_kinematics.json').read_text())
g = json.loads((HERE / 'out' / 'arm_kin_gold.json').read_text())
repo = Path(sys.argv[1])
def num(x):
    t = f'{float(x):.12g}'
    return t if ('.' in t or 'e' in t or 'n' in t) else t + '.0'      # ROS 파라미터 배열은 한 가지 형 (실수) 이어야 함


f = lambda xs: '[' + ', '.join(num(x) for x in xs) + ']'
flat = lambda M: [x for r in M for x in r]
y = ['# 팔 기구학 — 실물 집기 계산기(arm_tests grasp_test · reach_model · sag_model)와 같은 모델. 손으로 고치지 말 것',
     '# 모델 base = 10-27 배포 URDF base_link (바닥 위 base_z_floor). 로봇 base_link 와 x · y 같고 z 만 다름',
     '# 관절 사슬: T_base_armbase(4×4) · joint_T(관절마다 4×4) · axis, 손끝 · 접근축 · 닫힘축 = link5 좌표',
     '# 틱 ↔ 모델 관절: q = q_deg_at_ref + sign · (틱 − ref_tick) · 360/4096. 처짐: 명령 = 실제 − sag_c · τ (J2 · J3 · J4)',
     '/**:', '  ros__parameters:', '    arm_model:',
     f"      T_base_armbase: {f(flat(k['T_base_armbase']))}",
     f"      joint_T: {f([x for j in k['joints'] for x in flat(j['T'])])}",
     f"      axis: {f([x for j in k['joints'] for x in j['axis']])}",
     f"      tip5: {f(k['tip5'])}", f"      app5: {f(k['app5'])}", f"      close5: {f(k['close5'])}",
     f"      j1_xy: {f(k['j1_xy'])}", f"      lo: {f(k['lo'])}", f"      hi: {f(k['hi'])}",
     f"      template_q: {f(k['template_q'])}", f"      q5_rest: {num(k['q5_rest'])}",
     f"      ref_tick: {f([a['ref_tick'] for a in k['joint_map']])}",
     f"      q_deg_at_ref: {f([a['q_deg_at_ref'] for a in k['joint_map']])}",
     f"      sign: {f([a['sign'] for a in k['joint_map']])}",
     f"      tick_lo: {f([r[0] for r in k['tick_range']])}", f"      tick_hi: {f([r[1] for r in k['tick_range']])}",
     f"      sag_c: {num(k['sag_c_rad_per_nm'])}", f"      base_z_floor: {num(k['base_z_floor'])}"]
(repo / 'config' / 'arm_kinematics.yaml').write_text('\n'.join(y) + '\n')
h = ['// arm_kin_data.hpp — 시험 정답 (export: 실물 집기 계산기 grasp_test · sag_model 결과). 손으로 고치지 말 것', '#pragma once',
     '#include "manipulation/arm_kinematics.hpp"', 'namespace kin_data {', 'inline manipulation::ArmModel model() {', '  manipulation::ArmModel m;']
T = k['T_base_armbase']
h.append('  m.T_base_armbase << ' + ', '.join(f'{x:.17g}' for x in flat(T)) + ';')
for i, j in enumerate(k['joints']):
    h.append(f'  m.joint_T[{i}] << ' + ', '.join(f'{x:.17g}' for x in flat(j['T'])) + ';')
    h.append(f'  m.axis[{i}] = Eigen::Vector3d(' + ', '.join(f'{x:.17g}' for x in j['axis']) + ');')
for n in ('tip5', 'app5', 'close5'):
    h.append(f'  m.{n} = Eigen::Vector3d(' + ', '.join(f'{x:.17g}' for x in k[n]) + ');')
h.append('  m.j1_xy = Eigen::Vector2d(' + ', '.join(f'{x:.17g}' for x in k['j1_xy']) + ');')
for n in ('lo', 'hi', 'template_q'):
    h.append(f'  m.{n} = {{{{' + ', '.join(f'{x:.17g}' for x in k[n]) + '}};')
h.append(f"  m.q5_rest = {k['q5_rest']:.17g};")
for i, a in enumerate(k['joint_map']):
    h.append(f"  m.jm[{i}] = {{{a['ref_tick']:.17g}, {a['q_deg_at_ref']:.17g}, {float(a['sign']):.1f}}};")
for i, r in enumerate(k['tick_range']):
    h.append(f'  m.tick_range[{i}] = {{{{{r[0]:.17g}, {r[1]:.17g}}}}};')
h.append(f"  m.sag_c = {k['sag_c_rad_per_nm']:.17g};")
h += ['  return m;', '}']
h.append('struct FkCase { manipulation::Ticks ticks; double tip[3], app[3], close[3], tau[3]; manipulation::Ticks enc_held; };')
h.append('inline const FkCase fk_cases[] = {')
for c in g['fk']:
    h.append('  {{{' + ', '.join(map(str, c['ticks'])) + '}}, {' + ', '.join(f'{x:.12g}' for x in c['tip']) + '}, {' + ', '.join(f'{x:.12g}' for x in c['app']) +
             '}, {' + ', '.join(f'{x:.12g}' for x in c['close']) + '}, {' + ', '.join(f'{x:.12g}' for x in c['tau']) + '}, {{' + ', '.join(map(str, c['enc'])) + '}}},')
h.append('};')
h.append('struct PickCase { double x, y, yaw; bool ok; manipulation::Ticks ticks[3], enc[3]; };')
h.append('inline const PickCase pick_cases[] = {')
for c in g['ik']:
    if c['ok']:
        P = c['poses']
        h.append(f"  {{{c['x']}, {c['y']}, {c['yaw']}, true, {{" + ', '.join('{{' + ', '.join(map(str, p['ticks'])) + '}}' for p in P) + '}, {' +
                 ', '.join('{{' + ', '.join(map(str, p['enc'])) + '}}' for p in P) + '}},')
    else:
        h.append(f"  {{{c['x']}, {c['y']}, {c['yaw']}, false, {{}}, {{}}}},")
h += ['};', '}  // namespace kin_data']
(repo / 'ros2_ws' / 'src' / 'manipulation' / 'test' / 'arm_kin_data.hpp').write_text('\n'.join(h) + '\n')
print('ok')
