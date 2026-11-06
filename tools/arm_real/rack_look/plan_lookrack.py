import subprocess, yaml, sys, json
S='/tmp/claude-1000/-home-pa25-Documents-Codex-2026-10-29/e762aa0d-25d3-4611-981d-a9674b06b3f9/scratchpad'
REPO='/home/pa25/Documents/Codex/2026-10-29/p2_main_1029'
d=yaml.safe_load(open(f'{REPO}/config/arm_paths.yaml')); P=d[list(d)[0]]['ros__parameters']['arm_paths']
rc=yaml.safe_load(open(f'{REPO}/config/rack_cells.yaml')); cells=rc['rack_cells']['cells']
scan=P['rack_scan']; sk=len(scan['q'])//len(scan['nodes'])
best_scan={'S1':19,'S2':19,'S3':21,'S4':17,'S5':22,'S6':20,'S7':18,'S8':21,'S9':17,'S10':22,'S11':20,'S12':18}
def q_of(path,i):
    k=len(path['q'])//len(path['nodes']); return path['q'][i*k:(i+1)*k]
def fmt(q): return ' '.join(f'{v:.6f}' for v in q)
out={}
cmds=[]
for c in range(1,13):
    cs=f'S{c}'; st=P[f'store_{cs}']; rel=[i for i,g in enumerate(st['grip']) if g=='release'][0]
    qs=scan['q'][best_scan[cs]*sk:(best_scan[cs]+1)*sk]
    if cs=='S10':   # 스캔 22 번에서 J1 −4°: 수직 5.7°, 쥔 퍽에 안 가림 (22 그대로는 태그 모서리가 가림)
        qs=[float(v) for v in open(f'{S}/s10_scan_override.txt').read().split()]
    cmds.append('CLEAR'); cmds.append('HELD 1 50'); cmds.append('HELDPAD 0.003')
    # 앞 박스 (정차 0.2575, 205 mm, 윗면 base_link 0.037) + 박스 안 퍽 2 개 (중심 x ±40 mm)
    cmds.append('BOX pickbox 0.2575 0.0 0.002 0.205 0.205 0.070')
    cmds.append('PUCK bp1 0.2175 0.0 0.067 0'); cmds.append('PUCK bp2 0.2975 0.0 0.067 0')
    for o in range(1,13):
        if o==c: continue
        x,y=cells[f'S{o}']['xy']; cmds.append(f"PUCK p{o} {x} {y} 0.1625 {cells[f'S{o}']['yaw_deg']}")
    cmds.append(f'STATE {cs}_scan {fmt(qs)}')
    for i in range(1, rel):   # 기록 경로 앞부분 (구간마다)
        cmds.append(f'SEG {cs}_path_{i} {fmt(q_of(st,i))} {fmt(q_of(st,i+1))}')
    for i in range(max(1,rel-120), rel):
        cmds.append(f'SEG {cs}_hover_{i} {fmt(q_of(st,i))} {fmt(qs)}')
    cmds.append(f'SEG {cs}_direct {fmt(q_of(st,1))} {fmt(qs)}')
open(f'{S}/lookrack_cmds.txt','w').write('\n'.join(cmds)+'\n')
print(len(cmds),'cmds')
