"""generate.py 출력(BasicWriter)을 YOLO detection 형식으로 변환한다.

Isaac Sim 없이 numpy만 있으면 실행된다.

    python3 yolo/dataset_gen/convert_to_yolo.py yolo/datasets/sim_raw/wide_000 yolo/datasets/turtle_ninja \
        --split train --prefix wide_000

결과:
    <dst>/images/<split>/<prefix>_XXXX.png
    <dst>/labels/<split>/<prefix>_XXXX.txt   (cls cx cy w h, 0~1 정규화)
"""

import argparse
import json
import shutil
from pathlib import Path

import numpy as np

# sim 라벨 → YOLO 클래스. 모양 구분 여부가 정해지면 이 표만 바꿔서 다시 변환한다.
CLASS_MAPS = {
    "merged": {  # 모양 구분 안 함 (기본)
        "qr": 0,
        "puck_cyl": 1, "puck_box": 1,
        "puck_cyl_fallen": 2, "puck_box_fallen": 2,
        "signal_tower": 3,
    },
    "shape": {   # 모양 구분
        "qr": 0,
        "puck_cyl": 1, "puck_box": 2,
        "puck_cyl_fallen": 3, "puck_box_fallen": 4,
        "signal_tower": 5,
    },
}
# 벽 연결 부품(파란색)은 기본적으로 라벨에서 빼서 배경(hard negative)으로 학습한다.
# --with-joint를 주면 마지막 클래스로 추가한다.
JOINT = "wall_joint"
CLASS_NAMES = {
    "merged": ["qr", "puck", "puck_fallen", "signal_tower"],
    "shape": ["qr", "puck_cyl", "puck_box", "puck_cyl_fallen", "puck_box_fallen", "signal_tower"],
}

MIN_PX = 4  # 이보다 작은 박스는 라벨에서 제외 (학습 노이즈)


def image_size(png: Path):
    with png.open("rb") as f:
        head = f.read(24)
    return int.from_bytes(head[16:20], "big"), int.from_bytes(head[20:24], "big")


def convert_frame(src: Path, idx: str, class_map: dict):
    rgb = src / f"rgb_{idx}.png"
    w, h = image_size(rgb)
    boxes = np.load(src / f"bounding_box_2d_tight_{idx}.npy")
    id2label = json.loads((src / f"bounding_box_2d_tight_labels_{idx}.json").read_text())
    paths = json.loads((src / f"bounding_box_2d_tight_prim_paths_{idx}.json").read_text())
    meta_file = src / f"meta_{idx}.json"
    fallen = {}
    if meta_file.exists():
        fallen = {p["prim"]: p["fallen"] for p in json.loads(meta_file.read_text())["pucks"]}
    lines = []
    for b, path in zip(boxes, paths):
        name = id2label[str(int(b["semanticId"]))]["class"]
        if name.startswith("puck_") and fallen.get(path, False):
            name += "_fallen"   # sim 라벨은 모양만 고정, 넘어짐은 meta 기준
        if name not in class_map:
            continue
        x0, y0 = max(0, int(b["x_min"])), max(0, int(b["y_min"]))
        x1, y1 = min(w, int(b["x_max"]) + 1), min(h, int(b["y_max"]) + 1)
        if min(x1 - x0, y1 - y0) < MIN_PX:
            continue
        cx, cy = (x0 + x1) / 2 / w, (y0 + y1) / 2 / h
        lines.append(f"{class_map[name]} {cx:.6f} {cy:.6f} {(x1 - x0) / w:.6f} {(y1 - y0) / h:.6f}")
    return rgb, lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path)
    ap.add_argument("dst", type=Path)
    ap.add_argument("--split", default="train")
    ap.add_argument("--prefix", default=None)
    ap.add_argument("--classes", choices=list(CLASS_MAPS), default="merged")
    ap.add_argument("--with-joint", action="store_true", help="벽 연결 부품을 wall_joint 클래스로 포함")
    a = ap.parse_args()
    class_map = dict(CLASS_MAPS[a.classes])
    names = list(CLASS_NAMES[a.classes])
    if a.with_joint:
        class_map[JOINT] = len(names)
        names.append(JOINT)

    prefix = a.prefix or a.src.name
    img_dir = a.dst / "images" / a.split
    lbl_dir = a.dst / "labels" / a.split
    img_dir.mkdir(parents=True, exist_ok=True)
    lbl_dir.mkdir(parents=True, exist_ok=True)

    n_frames = n_boxes = 0
    for npy in sorted(a.src.glob("bounding_box_2d_tight_[0-9]*.npy")):
        idx = npy.stem.rsplit("_", 1)[1]
        rgb, lines = convert_frame(a.src, idx, class_map)
        shutil.copy2(rgb, img_dir / f"{prefix}_{idx}.png")
        (lbl_dir / f"{prefix}_{idx}.txt").write_text("\n".join(lines) + ("\n" if lines else ""))
        n_frames += 1
        n_boxes += len(lines)

    data_yaml = a.dst / "data.yaml"
    if not data_yaml.exists():
        data_yaml.write_text(
            f"path: {a.dst.resolve()}\ntrain: images/train\nval: images/val_real\n"
            f"names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(names)))
    print(f"{a.src} → {a.dst} [{a.split}]: {n_frames} frames, {n_boxes} boxes")


if __name__ == "__main__":
    main()
