#!/usr/bin/env python3
"""라벨링한 실제 데이터를 모두 모아, 비슷한 프레임이 train/val/test 에 섞이지 않게 다시 나눈다.

왜: 같은 영상에서 몇 프레임 떨어진 사진은 거의 똑같다. 이런 사진이 train 과 val/test 에
    나뉘어 들어가면 모델이 '본 사진'으로 시험을 보는 셈이라 점수가 부풀려진다.
    → 영상 하나를 통째로 한 split 에만 넣는다 (영상 단위 분리).

입력 (전부 YOLO 형식, 클래스 순서 = 팀 6클래스)
  --team_zip : 팀 실제 데이터 zip (real/images/{train,val,test}). 기존 split 은 무시하고 다시 나눔
  --tl_dir   : 신호등 CVAT 내보내기를 푼 폴더 (obj_train_data/)
  --cvat_zip : CVAT 에서 받은 zip들 (YOLO 1.1, Save images 켬)
출력: datasets/<out> (기본 real), split_report.csv (사진별 영상·split 기록)

나누는 기준: 파일 이름 <영상이름>_<프레임번호>.jpg
  1) 대부분의 영상: 영상 하나를 통째로 한 split 에 (SPLIT 표). 표에 없는 영상이 나오면 멈춤 (--split 로 추가)
  2) --block 영상 (기본: 새 영상 new_01~05 + 어두운 신호등 영상 traffic_light_2):
     시간 순서로 같은 길이 구간 6개로 잘라 train, test, train, train, val, train 순서로 배정 (약 2/3 train)
     split 이 바뀌는 경계 뒤 --gap 프레임(기본 90 = 30 fps 에서 3초) 안의 사진은 버림 → 바로 옆 프레임이 다른 split 에 들어가지 않음
     이유: 어두운 영상(new_04, traffic_light_2)이 통째로 test·train 한쪽에만 가면 모델이 어두운 조명을 못 배우거나 평가가 치우침
     구간을 여러 개로 나누는 이유: 영상 앞뒤로 물체 구성이 달라서, 한 덩어리로 자르면 클래스가 한 split 에 몰림
  split_report.csv 와 터미널에 split 별 평균 밝기·어두운 사진 수(밝기 < 60)를 보여 줌
신호등 영상은 화면 아래 18% 를 회색으로 가림 (라벨 없는 임시 바코드가 배경으로 학습되는 것 방지)

예)
  cd yolo/wide/scripts
  python build_real_split.py \
      --team_zip <선반 영상 zip> \
      --tl_dir <신호등 CVAT 폴더> \
      --cvat_zip <퍽 영상 zip 1> <퍽 영상 zip 2>
"""
import argparse
import csv
import re
import shutil
import tempfile
import zipfile
from collections import Counter, defaultdict
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / 'datasets'
CLASSES = ['red_cyl', 'green_cyl', 'red_box', 'green_box', 'barcode', 'traffic_light']

# 영상이름: split. shelf 영상은 2개뿐이라 val/test 로 쪼개면 같은 영상이 섞임 → 통째로 train
SPLIT = {
    'wide_puck_shelf_01': 'train', 'wide_puck_shelf_02': 'train',
    'wide_traffic_light_2': 'train', 'wide_traffic_light_4': 'train',
    'wide_traffic_light_0': 'val', 'wide_traffic_light_1': 'test',
    'wide_puck_new_01': 'train', 'wide_puck_new_02': 'train', 'wide_puck_new_05': 'train',
    'wide_puck_new_03': 'val', 'wide_puck_new_04': 'test',
}
MASK = {'wide_traffic_light_': 0.82}
BLOCK = 'wide_puck_new_01,wide_puck_new_02,wide_puck_new_03,wide_puck_new_04,wide_puck_new_05,wide_traffic_light_2'   # 시간 구간으로 나눌 영상
DARK = 60   # 이 밝기(0~255) 보다 어두우면 '어두운 사진'으로 셈   # 영상이름 앞부분: 가릴 높이 비율


def video_of(stem: str) -> str:
    m = re.match(r'(.+)_\d+$', stem)
    return m.group(1) if m else stem


def collect(root: Path, source: str):
    """root 아래의 jpg 와 같은 이름 txt 를 (이미지, 라벨, 출처) 로 모은다."""
    out = []
    for img in sorted(root.rglob('*.jpg')):
        lab = img.with_suffix('.txt')
        if not lab.exists():                       # real/images/x.jpg ↔ real/labels/x.txt 구조
            lab = Path(str(img).replace('/images/', '/labels/')).with_suffix('.txt')
        out.append((img, lab if lab.exists() else None, source))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--team_zip', required=True)
    ap.add_argument('--tl_dir', required=True)
    ap.add_argument('--cvat_zip', nargs='*', default=[])
    ap.add_argument('--out', default='real')
    ap.add_argument('--block', default=BLOCK, help='시간 구간으로 나눌 영상 이름들 (쉼표). 빈 문자열이면 전부 영상 단위')
    ap.add_argument('--pattern', default='train,test,train,train,val,train', help='--block 영상을 같은 길이 구간으로 잘라 차례로 배정')
    ap.add_argument('--gap', type=int, default=90, help='split 경계 뒤 버릴 프레임 수')
    ap.add_argument('--split', default='', help='표 바꾸기/추가: 영상이름:split,영상이름:split')
    a = ap.parse_args()

    split = dict(SPLIT)
    if a.split:
        split.update(dict(s.split(':') for s in a.split.split(',')))
    out = DATA / a.out
    if out.exists():
        raise SystemExit(f'{out} 이 이미 있음 -> 지우거나 --out 을 바꿀 것 (덮어쓰지 않음)')

    tmp = Path(tempfile.mkdtemp(prefix='real_split_'))
    items = []
    for i, z in enumerate([a.team_zip] + a.cvat_zip):
        z = Path(z).expanduser()
        d = tmp / f'{i}_{z.stem}'
        with zipfile.ZipFile(z) as zf:
            zf.extractall(d)
        names = next(d.rglob('obj.names'), None)
        if names and names.read_text().split() != CLASSES:
            raise SystemExit(f'{z.name}: 클래스 순서가 다름 {names.read_text().split()}')
        items += collect(d, z.name)
    tl = Path(a.tl_dir).expanduser()
    if (tl / 'obj.names').read_text().split() != CLASSES:
        raise SystemExit(f'{tl}: 클래스 순서가 다름')
    items += collect(tl, tl.name)

    # 영상 이름 확인
    block = {b for b in a.block.split(',') if b}
    unknown = sorted({video_of(img.stem) for img, _, _ in items} - split.keys() - block)
    if unknown:
        raise SystemExit(f'SPLIT 표에 없는 영상: {unknown} -> --split 영상이름:train 처럼 추가')

    # --block 영상: 시간 순서로 같은 길이 구간 N개로 자르고, 구간마다 --pattern 순서대로 split 배정
    pattern = a.pattern.split(',')
    block_split, gapped = {}, 0
    for v in sorted(block):
        fr = sorted({int(img.stem.rsplit('_', 1)[1]) for img, _, _ in items if video_of(img.stem) == v})
        if not fr:
            continue
        lo, hi = fr[0], fr[-1] + 1
        step = (hi - lo) / len(pattern)
        for f in fr:
            i = min(int((f - lo) / step), len(pattern) - 1)
            start = lo + i * step
            # 구간이 바뀌는 곳 바로 뒤(앞 구간과 split 이 다를 때)의 --gap 프레임은 버림
            if i > 0 and pattern[i] != pattern[i - 1] and f < start + a.gap:
                block_split[(v, f)] = None
                gapped += 1
            else:
                block_split[(v, f)] = pattern[i]
        print(f'{v}: 프레임 {lo}~{hi - 1}, 구간 {len(pattern)}개 × {step:.0f} 프레임 ({"/".join(pattern)})')

    seen, rows = set(), []
    bright = defaultdict(list)
    n, nv, cls, dup, nolab = Counter(), defaultdict(Counter), Counter(), 0, 0
    for img, lab, src in items:
        if img.name in seen:
            dup += 1
            continue
        seen.add(img.name)
        v = video_of(img.stem)
        if v in block:
            sp = block_split[(v, int(img.stem.rsplit('_', 1)[1]))]
            if sp is None:
                continue
        else:
            sp = split[v]
        (out / 'images' / sp).mkdir(parents=True, exist_ok=True)
        (out / 'labels' / sp).mkdir(parents=True, exist_ok=True)
        mask = next((r for k, r in MASK.items() if v.startswith(k)), 0)
        dst = out / 'images' / sp / img.name
        im = cv2.imread(str(img))
        br = float(cv2.cvtColor(im[:int(im.shape[0] * 0.82)], cv2.COLOR_BGR2GRAY).mean())
        bright[sp].append(br)
        if mask:
            im[int(im.shape[0] * mask):, :] = 114
            cv2.imwrite(str(dst), im, [cv2.IMWRITE_JPEG_QUALITY, 95])
        else:
            shutil.copy2(img, dst)
        txt = lab.read_text() if lab else ''
        nolab += lab is None
        (out / 'labels' / sp / (img.stem + '.txt')).write_text(txt)
        n[sp] += 1
        nv[sp][v] += 1
        for line in txt.splitlines():
            if line.strip():
                cls[(sp, CLASSES[int(line.split()[0])])] += 1
        rows.append([img.name, v, sp, src, mask, round(br, 1)])
    shutil.rmtree(tmp)

    with open(out / 'split_report.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['image', 'video', 'split', 'source', 'mask_y', 'brightness'])
        w.writerows(rows)

    print('결과:', out)
    print(f'겹치는 이름 건너뜀 {dup}, 라벨 파일 없음(빈 라벨로 저장) {nolab}, 구간 경계라 버린 사진 {gapped}')
    for sp in ('train', 'val', 'test'):
        b = bright[sp]
        print(f'\n[{sp}] {n[sp]}장  평균 밝기 {sum(b) / max(len(b), 1):.0f}, 어두운 사진(<{DARK}) {sum(x < DARK for x in b)}장')
        print('  영상:', dict(nv[sp]))
        print('  박스:', {c: cls[(sp, c)] for c in CLASSES})
    print(f'\n같은 영상이 두 split 에 들어간 경우: --block 영상만 ({", ".join(sorted(block)) or "없음"}), 경계 뒤 {a.gap} 프레임은 버림')
    print('다음: python prepare_data.py --real', out)


if __name__ == '__main__':
    main()
