#!/usr/bin/env python3
"""손목 D435 퍽 분할 YOLO26-seg 학습 (광각 yolo/wide/scripts/train.py 와 같은 구조, 결과 runs/wrist)
  - 설정은 configs/hyp.yaml, 데이터는 datasets/sim_seg/data.yaml (sim_to_seg.py 가 만듦)

  stage sim: 합성(시뮬) 데이터만. 결과 runs/wrist/seg8_sim
명령줄 인자를 주면 hyp.yaml 보다 우선.
사진 증강 (화이트밸런스·노출·감마·흐림·비네팅): hyp.yaml 의 albu, albumentations 필요 (없으면 멈춤. --no_albu 로 끔)
  albumentations 시드: ultralytics 가 프로세스마다 torch.initial_seed() 로 넣어 줌 (= hyp.yaml common.seed 에서 나옴, 워커마다 다름)
    → 같은 seed 면 증강도 같게 반복됨 (GPU 연산 · AMP 차이는 남음). 따로 고정 코드 필요 없음 (ultralytics 8.4.185 확인)
TensorBoard: <결과 폴더>/tensorboard 에 에폭마다 train·val 손실, 지표, 학습률 (tensorboard 필요, 없으면 멈춤. --no_tb 로 끔)
  ultralytics 전역 설정(tensorboard)은 안 바꾸고 이 파일의 콜백으로 씀. 노트북: yolo/wrist/experiments/train_seg8.ipynb

예) cd yolo/wrist/scripts
    ~/Desktop/physical_ai/.venvs/yolo/bin/python train.py --stage sim
    ~/Desktop/physical_ai/.venvs/yolo/bin/python train.py --resume ../runs/wrist/<이름>/weights/last.pt   (끊긴 학습 이어서만)
"""
import argparse
from pathlib import Path

import yaml
from ultralytics import YOLO

WRIST_DIR = Path(__file__).resolve().parent.parent   # yolo/wrist
CONFIGS = WRIST_DIR / 'configs'


def build_albu(c: dict) -> list:
    """hyp.yaml 의 albu -> albumentations 변환 목록 (ultralytics train(augmentations=...) 로 넘김). p 가 0 인 항목은 뺌."""
    try:
        import albumentations as A
    except ImportError:
        raise SystemExit('albumentations 없음 -> pip install albumentations (또는 --no_albu). '
                         'ultralytics 는 없으면 경고 없이 사진 증강을 건너뜀')
    T = []
    if (w := c.get('white_balance', {})).get('p', 0):
        T.append(A.PlanckianJitter(mode='blackbody', temperature_limit=tuple(w['kelvin']), p=w['p']))
    if (e := c.get('exposure', {})).get('p', 0):
        T.append(A.RandomBrightnessContrast(brightness_limit=e['brightness'], contrast_limit=e['contrast'], p=e['p']))
    if (g := c.get('gamma', {})).get('p', 0):
        T.append(A.RandomGamma(gamma_limit=tuple(g['range']), p=g['p']))
    if (b := c.get('blur', {})).get('p', 0):
        T.append(A.OneOf([A.MotionBlur(blur_limit=tuple(b['motion'])), A.Defocus(radius=tuple(b['defocus'])),
                          A.GaussianBlur(sigma_limit=(0.3, 1.5))], p=b['p']))
    if (v := c.get('vignette', {})).get('p', 0):
        T.append(A.Illumination(mode='gaussian', effect_type='brighten', intensity_range=tuple(v['intensity']),
                                center_range=(0.4, 0.6), sigma_range=(0.6, 1.0), p=v['p']))
    if (sh := c.get('shadow', {})).get('p', 0):          # 한쪽만 어두움
        T.append(A.OneOf([A.Illumination(mode='linear', effect_type='darken', intensity_range=tuple(sh['intensity'])),
                          A.Illumination(mode='gaussian', effect_type='darken', intensity_range=tuple(sh['intensity']),
                                         center_range=(0.1, 0.9), sigma_range=(0.2, 0.5))], p=sh['p']))
    if (t := c.get('tint', {})).get('p', 0):             # 신호등 빛: 빨강 또는 초록 쪽만 더함
        lo, hi = t['shift']
        T.append(A.OneOf([A.RGBShift(r_shift_limit=(lo, hi), g_shift_limit=0, b_shift_limit=0, p=1.0),
                          A.RGBShift(r_shift_limit=0, g_shift_limit=(lo, hi), b_shift_limit=0, p=1.0)], p=t['p']))
    if (L := c.get('lighting', {})).get('p', 0):         # 조명 단계 하나 (어둡게 + 센서 노이즈). 맨 마지막 = 노이즈가 흐림 뒤에 옴
        noise = lambda r: A.GaussNoise(std_range=tuple(r), p=1.0)
        modes = []
        if (m := L.get('dim')):
            modes.append(A.Sequential([A.RandomGamma(gamma_limit=tuple(m['gamma']), p=1.0), noise(m['noise'])], p=m['w']))
        if (m := L.get('dark')):
            modes.append(A.Sequential([A.RandomGamma(gamma_limit=tuple(m['gamma']), p=1.0), noise(m['noise']),
                                       A.ISONoise(intensity=(0.1, 0.3), p=1.0)], p=m['w']))
        if (m := L.get('dark_ae')):
            modes.append(A.Sequential([A.RandomBrightnessContrast(brightness_limit=(-0.05, 0.05), contrast_limit=tuple(m['contrast']), p=1.0),
                                       A.ColorJitter(brightness=0, contrast=0, saturation=tuple(m['saturation']), hue=0, p=1.0),
                                       noise(m['noise']), A.ISONoise(intensity=(0.2, 0.5), p=1.0)], p=m['w']))
        if modes:
            T.append(A.OneOf(modes, p=L['p']))
    return T


def add_tensorboard(model):
    """에폭 끝마다 results.csv 와 같은 값(train 손실, val 손실·지표, 학습률)을 <결과 폴더>/tensorboard 에 씀."""
    try:
        from torch.utils.tensorboard import SummaryWriter
    except ImportError:
        raise SystemExit('tensorboard 없음 -> pip install tensorboard (또는 --no_tb)')
    w = {}

    def on_train_start(trainer):
        w['tb'] = SummaryWriter(str(Path(trainer.save_dir) / 'tensorboard'))
        print(f'[train] TensorBoard: {Path(trainer.save_dir) / "tensorboard"}', flush=True)

    def on_fit_epoch_end(trainer):
        vals = {**trainer.label_loss_items(trainer.tloss, prefix='train'), **(trainer.metrics or {}), **trainer.lr}
        for k, v in vals.items():
            w['tb'].add_scalar(k, float(v), trainer.epoch + 1)
        w['tb'].flush()

    def on_train_end(trainer):
        if 'tb' in w:
            w['tb'].close()

    model.add_callback('on_train_start', on_train_start)
    model.add_callback('on_fit_epoch_end', on_fit_epoch_end)
    model.add_callback('on_train_end', on_train_end)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--stage', choices=['sim'], default='sim')
    ap.add_argument('--hyp', default=str(CONFIGS / 'hyp.yaml'))
    ap.add_argument('--data', default=None, help='data.yaml (기본: hyp.yaml stage 의 data, yolo/wrist 기준 경로)')
    ap.add_argument('--imgsz', type=int, default=None)
    ap.add_argument('--epochs', type=int, default=None)
    ap.add_argument('--batch', type=int, default=None)
    ap.add_argument('--device', default='0')
    ap.add_argument('--name', default=None)
    ap.add_argument('--resume', default=None, metavar='LAST_PT', help='끊긴 같은 학습만 이어서 (그 학습의 weights/last.pt). 다른 학습 결과에서 시작하는 용도 아님')
    ap.add_argument('--no_albu', action='store_true', help='사진 증강(hyp.yaml albu) 끄기')
    ap.add_argument('--no_tb', action='store_true', help='TensorBoard 기록 끄기')
    a = ap.parse_args()

    hyp = yaml.safe_load(Path(a.hyp).read_text())
    cfg = {**hyp['common'], **hyp['stages'][a.stage]}
    model_name = cfg.pop('model')
    data_name = cfg.pop('data')
    weights = a.resume or model_name             # 시작은 항상 기본 모델 (이어 학습 없음)
    data = Path(a.data) if a.data else WRIST_DIR / data_name
    for k in ('imgsz', 'epochs', 'batch', 'name'):
        if getattr(a, k) is not None:
            cfg[k] = getattr(a, k)
    if not data.exists():
        raise SystemExit(f'{data} 없음 -> sim_to_seg.py 먼저')
    if a.resume is None and Path(weights).parent.name == 'weights':
        raise SystemExit(f'시작 가중치가 학습 결과({weights})임 -> hyp.yaml common.model 은 yolo26n-seg.pt 여야 함 (항상 처음부터)')

    model = YOLO(weights)
    if not a.no_tb:
        add_tensorboard(model)
    if a.resume:
        model.train(resume=True)
        return
    if hyp.get('albu') and not a.no_albu:
        cfg['augmentations'] = build_albu(hyp['albu'])
    print(f'[train] stage {a.stage}, weights {weights}, data {data}\n[train] 설정 {cfg}')
    model.train(data=str(data), device=a.device, project=str(WRIST_DIR / 'runs' / 'wrist'), exist_ok=False, plots=True, **cfg)
    print('끝. 결과:', model.trainer.save_dir, flush=True)   # 노트북이 이 줄로 결과 폴더를 찾음


if __name__ == '__main__':
    main()
