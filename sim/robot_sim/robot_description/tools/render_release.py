#!/usr/bin/env python3
"""배포 URDF 보기용 렌더: 실제 메시(visual) + 조명·그림자. 이미지 3장 + 실물 실행(A9F5) 재생 영상.
실행: MUJOCO_GL=egl /home/pa25/env_isaacsim62/bin/python tools/render_release.py [images|video RUN]"""
import glob
import json
import math
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

OUT = Path(__file__).resolve().parents[1]
IMG = OUT / "images"
JM = json.loads((OUT / "config/joint_map.json").read_text())
POSES = json.loads((OUT / "config/initial_pose.json").read_text())["poses"]
RUNS = "/home/pa25/Documents/Codex/2026-10-20/task-3/arm_staged_trial_claude_20261021/runs"
FB = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 30)
FR = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 22)
FS = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 18)
NAVY, FOOT = (18, 36, 58), (238, 242, 246)


def load():
    s = (OUT / "urdf/robot.urdf").read_text().replace("package://nt_release_description/", str(OUT) + "/")
    s = re.sub(r'(<robot name="[^"]+">)', r'\1<mujoco><compiler discardvisual="false" fusestatic="false" strippath="false"/></mujoco>', s, count=1)
    with tempfile.TemporaryDirectory() as t:
        f = Path(t) / "r.urdf"
        f.write_text(s)
        m = mujoco.MjModel.from_xml_path(str(f))
        x = Path(t) / "o.xml"
        mujoco.mj_saveLastXML(str(x), m)
        txt = x.read_text()
        vis = ('<visual><global offwidth="1600" offheight="1200"/><quality shadowsize="4096"/>'
               '<headlight ambient=".30 .30 .30" diffuse=".30 .30 .30" specular=".03 .03 .03"/></visual>')
        asset = '<asset><texture name="sky" type="skybox" builtin="gradient" rgb1=".90 .90 .91" rgb2=".88 .88 .89" width="256" height="256"/></asset>'
        light = ('<light directional="true" pos="0.4 -0.6 2" dir="-0.25 0.35 -1" diffuse=".48 .48 .48" castshadow="true"/>'
                 '<light directional="true" pos="-1 1 2" dir="0.5 -0.5 -1" diffuse=".2 .2 .2" castshadow="false"/>')
        txt = re.sub(r"<visual>.*?</visual>", "", txt, flags=re.S).replace("<worldbody>", "<worldbody>" + light, 1)
        txt = re.sub(r"(<mujoco[^>]*>)", r"\1" + vis + asset, txt, count=1)
        x.write_text(txt)
        m = mujoco.MjModel.from_xml_path(str(x))
    m.vis.global_.offwidth, m.vis.global_.offheight = 1600, 1200
    return m


M = load()
D = mujoco.MjData(M)
R = mujoco.Renderer(M, 900, 1200)


def q_from_ticks(tk):
    q = {}
    for j in JM["arm"]:
        t = tk.get(str(j["id"]), tk.get(j["id"]))
        if t is not None:
            q[j["joint"]] = math.radians(j["q_deg_at_ref"]) + j["sign"] * (t - j["ref_tick"]) * 2 * math.pi / 4096
    g = tk.get("15", tk.get(15))
    if g is not None:
        gp = JM["gripper"]["gap_table_tick_mm"]
        gap = float(np.interp(g, [a for a, _ in gp], [b for _, b in gp]))
        lo = M.jnt_range[M.joint("gripper_finger_a_joint").id][0]
        q["gripper_finger_a_joint"] = q["gripper_finger_b_joint"] = max(lo, (gap - JM["gripper"]["gap0_mm"]) / 2000)
    return q


def shot(q, az=225, el=-24, dist=0.92, look=(0.02, -0.06, 0.19)):
    D.qpos[:] = 0
    for n, v in q.items():
        D.qpos[M.jnt_qposadr[M.joint(n).id]] = v
    mujoco.mj_forward(M, D)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = look
    cam.distance, cam.azimuth, cam.elevation = dist, az, el
    R.update_scene(D, cam)
    return Image.fromarray(R.render())


def card(img, title, foot1, foot2=""):
    W = img.width
    c = Image.new("RGB", (W, img.height + 76 + 110), FOOT)
    d = ImageDraw.Draw(c)
    d.rectangle([0, 0, W, 76], fill=NAVY)
    d.text((28, 18), title, fill=(255, 255, 255), font=FB)
    c.paste(img, (0, 76))
    d.text((28, img.height + 76 + 18), foot1, fill=(30, 30, 30), font=FR)
    if foot2:
        d.text((28, img.height + 76 + 60), foot2, fill=(90, 90, 90), font=FR)
    return c


def images():
    IMG.mkdir(exist_ok=True)
    p0 = dict(POSES["C1_P0"]["q_rad"])
    p0.update({"gripper_finger_a_joint": -0.021253618998087663, "gripper_finger_b_joint": -0.021253618998087663})
    a = shot(p0).resize((800, 600))
    b = shot(p0, az=300, el=-12, dist=0.85).resize((800, 600))
    img = Image.new("RGB", (1600, 600))
    img.paste(a, (0, 0))
    img.paste(b, (800, 0))
    card(img, "닌자터틀 배포 URDF · 선택 작업 대기 자세 P0 (기본 표시: parked)", "실제 메시 · 총질량 3.9 kg(실측) · 관절 범위·속도·힘 한계 실측/데이터시트 반영",
         "nt_release_description / urdf/robot.urdf").save(IMG / "release_p0.jpg", quality=92)
    pk = dict(json.loads((OUT / "config/initial_pose.json").read_text())["positions"])
    pk["gripper_finger_b_joint"] = pk["gripper_finger_a_joint"]
    a = shot(pk, az=225, el=-22, dist=0.6, look=(0.08, 0.0, 0.14)).resize((800, 600))
    b = shot(pk, az=270, el=-8, dist=0.5, look=(0.08, 0.0, 0.13)).resize((800, 600))
    img = Image.new("RGB", (1600, 600))
    img.paste(a, (0, 0))
    img.paste(b, (800, 0))
    card(img, "받침 자세 · 실측 받침대 50×53×33 mm", "실물: 집게가 상자 안으로 들어가 로봇 쪽 벽 테두리에 기댐",
         "모델: 장착 수정·16mm 표시 / 잔여 벽 교차와 물리 초기화 미검증").save(IMG / "release_park.jpg", quality=92)
    # 관절 범위: 관절마다 양 끝 (나머지는 P0)
    tiles = []
    for name, lab in [("arm_joint1", "J1 몸통"), ("arm_joint2", "J2 어깨"), ("arm_joint3", "J3 팔꿈치"), ("arm_joint4", "J4 손목"), ("arm_joint5", "J5 손목 회전")]:
        lo, hi = M.jnt_range[M.joint(name).id]
        row = []
        for v in (lo, hi):
            q = dict(p0)
            q[name] = v
            if name in ("arm_joint2", "arm_joint3"):   # 접기 끝은 P0 에서 몸체에 닿으므로 다른 관절을 세운 자세에서
                q.update({"arm_joint2": q.get("arm_joint2"), "arm_joint1": p0["arm_joint1"]})
            t = shot(q, az=225, el=-20, dist=1.0, look=(0.0, -0.02, 0.22)).resize((400, 300))
            dd = ImageDraw.Draw(t)
            dd.rectangle([0, 0, 400, 30], fill=(255, 255, 255))
            dd.text((10, 4), f"{lab} {math.degrees(v):+.1f}°", fill=(30, 30, 30), font=FS)
            row.append(t)
        tiles.append(row)
    grid = Image.new("RGB", (400 * 5, 600), (255, 255, 255))
    for i, (t_lo, t_hi) in enumerate(tiles):
        grid.paste(t_lo, (400 * i, 0))
        grid.paste(t_hi, (400 * i, 300))
    card(grid, "관절 범위 (배포판) · 위: 아래쪽 끝 / 아래: 위쪽 끝", "손 측정(프로파일 장착) + 실제 실행 + 위치 모드 엔코더 창 · 나머지 관절은 P0",
         "자세에 따라 몸체에 닿는 범위는 관절 범위가 아니라 충돌 형상이 막음").save(IMG / "release_ranges.jpg", quality=92)
    print("images:", sorted(p.name for p in IMG.glob("release_*.jpg")))


def video(run, fps=10):
    rows, last = [], -1e9
    for l in open(f"{RUNS}/{run}/03_session.jsonl"):
        if '"event":"snapshot"' not in l or '"state":"REPLAY' not in l:
            continue
        x = json.loads(l)
        if x["monotonic_ms"] - last >= 1000 / fps:
            last = x["monotonic_ms"]
            rows.append((x["monotonic_ms"], {k: v["pos"] for k, v in x["ids"].items()}))
    t0 = rows[0][0]
    fr = OUT / "images" / f"{run}_frames"
    fr.mkdir(parents=True, exist_ok=True)
    for k, (t, tk) in enumerate(rows):
        q = q_from_ticks(tk)
        a = shot(q).resize((800, 600))
        b = shot(q, az=300, el=-10, dist=0.85, look=(0.04, -0.04, 0.2)).resize((800, 600))
        img = Image.new("RGB", (1600, 600))
        img.paste(a, (0, 0))
        img.paste(b, (800, 0))
        card(img, f"실물 실행 재생 · A9F5 (상자 왼쪽 끝 퍽 → 적재함) · {(t - t0) / 1000:5.1f} s",
             f"실제 모터 기록을 배포 URDF 에 그대로 재생 · J1~J5 {tk['11']}/{tk['12']}/{tk['13']}/{tk['14']}/{tk['16']} · 그리퍼 {tk['15']}",
             "실물 전체 27.3초(1배속) · 관절 각도 = config/joint_map.json").save(fr / f"f{k:05d}.jpg", quality=88)
    out = OUT / "images" / f"release_motion_{run}.mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(fps), "-i", str(fr / "f%05d.jpg"), "-vf", "scale=1600:-2",
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", str(out)], check=True)
    print("video:", out, len(rows), "frames")


if __name__ == "__main__":
    if sys.argv[1] == "images":
        images()
    else:
        video(sys.argv[2])
