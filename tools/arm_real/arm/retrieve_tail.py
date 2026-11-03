"""빼기 재생 경로 끝(실물 들기 · lift_out) → 박스 위 기준점(staging, PICK 끝과 같은 자세): PLACE 가 그 자리에서 시작하게 이어 붙임
검사: 쥔 퍽 ↔ 박스 · 컵 · 찬 칸 퍽 (rack_load.path_hits) + 팔 경로 (pose_safety C.path, 컵 장애물)"""
import json
import shutil
import sys
from pathlib import Path

import replay_paths as rp
from replay_paths import ARM, OUT, HELD, SPEED, ps, rack_load
from gen_paths import ORDER, S_TO_OLD, staging


def main():
    cells = sys.argv[1:] or ORDER
    summ = json.loads((OUT / 'replay_summary_retrieve.json').read_text())
    C = ps.Checker()
    st = staging('staging_ik.json')
    for s in cells:
        f = OUT / f'retrieve_{s}.checked.json'
        bak = OUT / f'retrieve_{s}.pre_tail.json'
        if not bak.exists():
            shutil.copy(f, bak)
        d = json.loads(bak.read_text())
        seq = d['sequence']
        last = seq[-1]
        full = {S_TO_OLD[x] for x in summ[f'retrieve_{s}']['full_cells']}
        rc = json.loads((ARM / 'rack_cells.json').read_text())
        for n, c in rc['cells'].items():
            c['state'] = 'full' if n in full else 'empty'
        h = rack_load.path_hits(C, last['ticks'], st['ticks'], rack_load.obstacles(rc, None))
        saved = list(ps.EXTRA_OBS)
        ps.set_obstacles(rack_load.cup_obstacles(rc, None))
        try:
            bad_end, _ = C.pose(st['ticks'], grip=ps.GRIP_JM['held'])
            bad_path = C.path(last['ticks'], st['ticks'], grip=ps.GRIP_JM['held'])
        finally:
            ps.EXTRA_OBS[:] = saved
        warn = [w for w in (h and f'퍽 {h}', bad_path and f'팔 {bad_path}', bad_end and '기준점 ' + '; '.join(bad_end[:3])) if w]
        seq.append(dict(name='staging', ticks=st['ticks'], stand=False, stop=False, pgap=[0] * 5, enc=st['enc'], held_kg=HELD, speed=SPEED))
        d['sequence'] = seq
        if warn:
            d.setdefault('safety_warnings', []).extend('[기준점 이음] ' + w for w in warn)
        d['tail'] = 'lift_out → staging (retrieve_tail.py)'
        f.write_text(json.dumps(d, ensure_ascii=False, indent=1))
        print(f'{s}: lift_out {last["ticks"]} → staging {st["ticks"]}', '✓' if not warn else '✗ ' + ' | '.join(warn), flush=True)


if __name__ == '__main__':
    main()
