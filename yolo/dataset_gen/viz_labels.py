"""generate.py 출력에 bbox와 정답(색·신호등 상태)을 그려서 눈으로 확인한다.

    python3 yolo/dataset_gen/viz_labels.py yolo/datasets/sim_raw/wide_000 [--out viz/]
"""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

COLORS = {"red": (0, 0, 255), "green": (0, 200, 0), "blue": (255, 80, 0),
          "yellow": (0, 200, 255), "off": (200, 200, 200)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args()
    out = a.out or a.src / "viz"
    out.mkdir(parents=True, exist_ok=True)

    for npy in sorted(a.src.glob("bounding_box_2d_tight_[0-9]*.npy")):
        idx = npy.stem.rsplit("_", 1)[1]
        img = cv2.imread(str(a.src / f"rgb_{idx}.png"))
        boxes = np.load(npy)
        labels = json.loads((a.src / f"bounding_box_2d_tight_labels_{idx}.json").read_text())
        paths = json.loads((a.src / f"bounding_box_2d_tight_prim_paths_{idx}.json").read_text())
        meta = json.loads((a.src / f"meta_{idx}.json").read_text())
        info = {p["prim"]: p["color"] + (" fallen" if p["fallen"] else "") for p in meta["pucks"]}
        if meta.get("tower"):
            info[meta["tower"]["prim"]] = meta["tower"]["state"]

        for b, path in zip(boxes, paths):
            name = labels[str(int(b["semanticId"]))]["class"]
            attr = info.get(path, "")
            c = COLORS.get(attr.split()[0] if attr else "", (255, 255, 255))
            p0, p1 = (int(b["x_min"]), int(b["y_min"])), (int(b["x_max"]), int(b["y_max"]))
            cv2.rectangle(img, p0, p1, c, 1)
            cv2.putText(img, f"{name} {attr}", (p0[0], max(10, p0[1] - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, c, 1, cv2.LINE_AA)
        cv2.imwrite(str(out / f"viz_{idx}.png"), img)
    print(f"saved to {out}")


if __name__ == "__main__":
    main()
