#!/usr/bin/env python3
"""학습용 data.yaml 만들기 (각자 PC 에서 실행. 절대경로가 들어가서 yaml 은 커밋하지 않음)

출력: configs/data_sim.yaml, configs/data_real.yaml
  data_sim.yaml  : 1단계 (합성으로 처음부터 학습). train = 합성 train, val = 실제 val (실제 점수로 best epoch 을 고름)
  data_real.yaml : 2단계 (실제 데이터만으로 미세조정). train/val = 실제
  test           : 둘 다 실제 test. 모델 고르기·경량화 판단은 실제 셋으로만

폴더 구조 (YOLO 기본): <root>/images/{train,val,test}/*.jpg, <root>/labels/{train,val,test}/*.txt
  합성: 시뮬레이터 출력 (기본 yolo/wide/datasets/sim)
  실제: build_real_split.py 출력 (기본 yolo/wide/datasets/real)
클래스 순서는 시뮬 생성기와 같아야 함 (CLASSES). 다르면 라벨 번호가 어긋나서 학습이 망가짐

예) cd yolo/wide/scripts && python prepare_data.py
    python prepare_data.py --sim <합성 폴더> --real <실제 폴더>
"""
import argparse
from pathlib import Path

CLASSES = ['red_cyl', 'green_cyl', 'red_box', 'green_box', 'barcode', 'traffic_light']
HERE = Path(__file__).resolve().parent
YOLO_DIR = HERE.parent


def count(root: Path, split: str) -> int:
    d = root / 'images' / split
    return sum(1 for p in d.glob('*') if p.suffix.lower() in ('.jpg', '.jpeg', '.png')) if d.exists() else 0


def check_names(root: Path):
    """시뮬 data.yaml 의 클래스 순서가 같은지 확인."""
    y = root / 'data.yaml'
    if y.exists():
        txt = y.read_text()
        if all(c in txt for c in CLASSES):
            i = [txt.index(c) for c in CLASSES]
            if i != sorted(i):
                raise SystemExit(f'{y} 의 클래스 순서가 {CLASSES} 와 다름')


def write_yaml(path: Path, train: Path, val: Path, test: Path):
    lines = ['# prepare_data.py 가 만듦. 직접 고치지 말고 다시 만들 것 (커밋하지 않음)',
             f'train: {train}', f'val: {val}', f'test: {test}', f'nc: {len(CLASSES)}', f'names: {CLASSES}']
    path.write_text('\n'.join(lines) + '\n')
    print(f'{path.name}: train {train}\n  val {val}\n  test {test}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim', default=str(YOLO_DIR / 'datasets' / 'sim'), help='합성 데이터셋 루트')
    ap.add_argument('--real', default=str(YOLO_DIR / 'datasets' / 'real'), help='실제 데이터셋 루트 (build_real_split.py 출력)')
    ap.add_argument('--out', default=str(YOLO_DIR / 'configs'), help='data_*.yaml 저장 위치')
    a = ap.parse_args()
    sim, real, out = (Path(p).expanduser().resolve() for p in (a.sim, a.real, a.out))
    check_names(sim)
    n = {f'sim_{s}': count(sim, s) for s in ('train', 'val', 'test')}
    n.update({f'real_{s}': count(real, s) for s in ('train', 'val', 'test')})
    print('이미지 수:', n)
    if n['sim_train'] == 0:
        raise SystemExit(f'합성 train 이미지가 없음: {sim}/images/train')
    if not (n['real_train'] and n['real_val'] and n['real_test']):
        raise SystemExit(f'실제 train/val/test 가 모두 있어야 함: {real} (build_real_split.py 먼저)')

    out.mkdir(parents=True, exist_ok=True)
    rv, rt = real / 'images' / 'val', real / 'images' / 'test'
    write_yaml(out / 'data_sim.yaml', sim / 'images' / 'train', rv, rt)
    write_yaml(out / 'data_real.yaml', real / 'images' / 'train', rv, rt)


if __name__ == '__main__':
    main()
