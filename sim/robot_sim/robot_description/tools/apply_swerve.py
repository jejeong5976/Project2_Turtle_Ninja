#!/usr/bin/env python3
"""준비본 URDF(e8bd…, 고정 퍽 제거·J1 축 수정) → 스워브 배포본: 바퀴 관절 한계만 바꿈. 다른 링크·관절·메시는 그대로.
  - 조향 4개: limit lower/upper = 엔코더 창 0~4095 전체(끝 여유 0), velocity = 조향 모터 Velocity Limit, effort = W350 정지 토크
  - 구동 4개: velocity = 구동 모터 Velocity Limit, effort = W210 정지 토크
값은 config/swerve.yaml 에서 읽음. 실행: python3 tools/apply_swerve.py (입력 해시가 다르면 거부)"""
import hashlib, re, sys
from pathlib import Path
import yaml
HERE = Path(__file__).resolve().parents[1]
U = HERE / "urdf/robot.urdf"
SRC_SHA = "e8bd147277a021fbe9990ca36935b80c0596f4711293d05ba679bc6a15e46c91"
src = U.read_text()
sha = hashlib.sha256(src.encode()).hexdigest()
if sha != SRC_SHA:
    if "swerve_release_20261027" in src:
        sys.exit("이미 적용됨")
    sys.exit(f"입력 URDF 해시가 준비본과 다름: {sha}")
c = yaml.safe_load((HERE / "config/swerve.yaml").read_text())
hw = c["hardware_max"]; jn = c["joints"]; rg = c["steer_range_rad"]

def set_limit(text, joint, **kw):
    m = re.search(rf'(<joint name="{joint}"[^>]*>)(.*?)(</joint>)', text, re.S)
    body = m.group(2)
    lim = re.search(r"<limit[^>]*/>", body).group(0)
    new = lim
    for k, v in kw.items():
        new = re.sub(rf'{k}="[^"]*"', f'{k}="{v}"', new)
    return text[:m.start(2)] + body.replace(lim, new) + text[m.end(2):]

out = src
for i, j in enumerate(jn["steer"]):
    out = set_limit(out, j, lower=repr(rg["min"][i]), upper=repr(rg["max"][i]), velocity=repr(hw["steer_speed"]), effort=repr(hw["steer_torque"]))
for j in jn["drive"]:
    out = set_limit(out, j, velocity=repr(hw["wheel_speed"]), effort=repr(hw["wheel_torque"]))
out = out.replace("<robot name=\"", "<!-- swerve_release_20261027: 바퀴 관절 한계 = config/swerve.yaml hardware_max, 조향 범위 = 엔코더 창 전체 -->\n<robot name=\"", 1)
U.write_text(out)
print("적용:", hashlib.sha256(out.encode()).hexdigest())
