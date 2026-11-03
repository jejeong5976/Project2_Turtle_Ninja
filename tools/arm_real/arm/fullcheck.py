"""최악 조건 검사: 대상 칸 말고 11 칸이 모두 찬 상태로 재생 경로를 다시 pose_safety 검사 (원래 경로 파일은 안 건드림)
원래(그 실물 실행 때 찬 칸) 검사에 없던 경고 = 칸이 더 차서 생긴 문제"""
import json, re, sys
import replay_paths as rp
from replay_paths import OUT, ps
from gen_paths import ORDER, S_TO_OLD

def warn_set(log):
    out = set()
    for l in log.splitlines():
        if '✗' in l and '전체' not in l:
            out.update(m.group(0) for m in re.finditer(r'[a-z_0-9]+ ↔ cup \w+ puck', l))
    return out

def main():
    names = sys.argv[1:] or [f'{k}_S{i}' for k in ('store', 'retrieve') for i in range(1, 13)]
    C = ps.Checker()
    res = {}
    for n in names:
        s = n.split('_')[1]; old = S_TO_OLD[s]
        seq = json.loads((OUT / f'{n}.checked.json').read_text())['sequence']
        poses = [w for w in seq if not w['stand']]
        full = {S_TO_OLD[x] for x in ORDER if x != s}
        st = n.startswith('store')
        ok, _ = rp.check(C, poses, f'full_{n}', full, old, start_here=st, end_here=not st)
        new = warn_set((OUT / f'full_{n}.safety.log').read_text())
        base = warn_set((OUT / f'{n}.safety.log').read_text()) if (OUT / f'{n}.safety.log').exists() else set()
        added = sorted(new - base)
        mins = {}
        for l in (OUT / f'full_{n}.safety.log').read_text().splitlines():
            for m in re.finditer(r'([a-z_0-9]+ ↔ cup \w+ puck) (-?\d+) mm', l):
                if m.group(1) in added: mins[m.group(1)] = min(mins.get(m.group(1), 99), int(m.group(2)))
        res[n] = dict(ok=ok, added={k: mins.get(k) for k in added})
        lab = ', '.join(f"{re.sub(r'cup (\\w+) puck', lambda c: rp.OLD_TO_S[c.group(1)] + ' 퍽', k)} {v} mm" for k, v in mins.items())
        print(f'{n:13s}', '새 경고 없음' if not added else f'⚠ 칸이 더 차면 생기는 경고: {lab}', flush=True)
    (OUT / 'fullcheck.json').write_text(json.dumps(res, ensure_ascii=False, indent=1))

if __name__ == '__main__':
    main()
