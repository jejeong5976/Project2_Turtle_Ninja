#!/usr/bin/env python3
"""배포 URDF → Isaac USD (usd/robot/robot.usda). 주행 학습용: 차체 고정 안 함(config/isaac_import_swerve.json, fix_base false).
  보기 렌더용 isaac_render.py 는 차체 고정(isaac_import_fixed.json) 그대로.
실행: /home/pa25/env_isaacsim62/bin/python tools/isaac_import.py"""
import json, shutil
from pathlib import Path
OUT = Path(__file__).resolve().parents[1]
rt = Path("/tmp") / "nt_release_isaac_runtime"; rt.mkdir(exist_ok=True)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "multi_gpu": False, "limit_cpu_threads": 4,
                     "extra_args": ["--/app/extensions/registryEnabled=false", "--/app/enableTelemetry=false", "--/app/settings/persistent=false", "--portable-root=" + str(rt)]})
try:
    from isaacsim.core.utils.extensions import enable_extension
    enable_extension("isaacsim.asset.importer.urdf")
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    usd_dir = OUT / "usd"
    if usd_dir.exists(): shutil.rmtree(usd_dir)
    usd_dir.mkdir()
    st = json.loads((OUT / "config/isaac_import_swerve.json").read_text())
    st.update(urdf_path=str(OUT / "urdf/robot.urdf"), usd_path=str(usd_dir), ros_package_paths=[{"name": "nt_release_description", "path": str(OUT)}])
    print("USD", URDFImporter(URDFImporterConfig(**st)).import_urdf(), flush=True)
finally:
    app.close()
