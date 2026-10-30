#!/usr/bin/env python3
"""YOLO26 학습
  - 설정은 configs/hyp.yaml, 데이터는 configs/data_<stage>.yaml

  stage 1: yolo26n.pt 에서 시작, 시뮬로 사전 학습
  stage 2 (real): 1단계 last.pt 에서 시작, 실제 데이터만으로 미세조정 (학습률 낮춤)
명령줄 인자를 주면 hyp.yaml 보다 우선.
사진 증강 (화이트밸런스·노출·감마·흐림·비네팅): hyp.yaml 의 albu, albumentations 필요 (없으면 멈춤. --no_albu 로 끔)
TensorBoard: <결과 폴더>/tensorboard 에 에폭마다 train·val 손실, 지표, 학습률 (tensorboard 필요, 없으면 멈춤. --no_tb 로 끔)
  ultralytics 전역 설정(tensorboard)은 안 바꾸고 이 파일의 콜백으로 씀. 노트북: yolo/wide/notebooks/train.ipynb

예) cd yolo/wide/scripts
    python train.py --stage sim
    python train.py --stage real --weights ../runs/wide/sim_pre/weights/last.pt
"""
import argparse
from pathlib import Path

import yaml
from ultralytics import YOLO

YOLO_DIR = Path(__file__).resolve().parent.parent
CONFIGS = YOLO_DIR / 'configs'


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
    ap.add_argument('--stage', choices=['sim', 'real'], default='sim')
    ap.add_argument('--hyp', default=str(CONFIGS / 'hyp.yaml'))
    ap.add_argument('--weights', default=None, help='시작 가중치 (기본: hyp.yaml 의 model, real 단계는 sim 단계 last.pt 를 줄 것)')
    ap.add_argument('--data', default=None, help='data.yaml (기본: configs/<hyp.yaml stage 의 data>)')
    ap.add_argument('--imgsz', type=int, default=None)
    ap.add_argument('--epochs', type=int, default=None)
    ap.add_argument('--batch', type=int, default=None)
    ap.add_argument('--device', default='0')
    ap.add_argument('--name', default=None)
    ap.add_argument('--resume', action='store_true', help='끊긴 학습 이어서 (--weights 에 last.pt)')
    ap.add_argument('--no_albu', action='store_true', help='사진 증강(hyp.yaml albu) 끄기')
    ap.add_argument('--no_tb', action='store_true', help='TensorBoard 기록 끄기')
    a = ap.parse_args()

    hyp = yaml.safe_load(Path(a.hyp).read_text())
    cfg = {**hyp['common'], **hyp['stages'][a.stage]}
    model_name = cfg.pop('model')
    data_name = cfg.pop('data')
    weights = a.weights or model_name
    data = Path(a.data) if a.data else CONFIGS / data_name
    for k in ('imgsz', 'epochs', 'batch', 'name'):
        if getattr(a, k) is not None:
            cfg[k] = getattr(a, k)
    if not data.exists():
        raise SystemExit(f'{data} 없음 -> prepare_data.py 먼저')
    if a.stage == 'real' and not a.weights:
        print('⚠ real 단계인데 --weights 가 없어 기본 모델에서 시작함 -> sim 단계 last.pt 를 주는 게 가이드 순서')

    model = YOLO(weights)
    if not a.no_tb:
        add_tensorboard(model)
    if a.resume:
        model.train(resume=True)
        return
    if hyp.get('albu') and not a.no_albu:
        cfg['augmentations'] = build_albu(hyp['albu'])
    print(f'[train] stage {a.stage}, weights {weights}, data {data}\n[train] 설정 {cfg}')
    model.train(data=str(data), device=a.device, project=str(YOLO_DIR / 'runs' / 'wide'), exist_ok=False, plots=True, **cfg)
    print('끝. 결과:', model.trainer.save_dir, flush=True)   # 노트북이 이 줄로 결과 폴더를 찾음


if __name__ == '__main__':
    main()
