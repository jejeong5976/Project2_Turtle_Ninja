# 강화학습 (Isaac Lab)

`sim_core/` (로봇·스워브 제어·미로·소품) 위에서 학습하는 코드를 두는 곳. **아직 코드 없음** — 아래는 만들 때의 권장 구조.

```
rl/
├── tasks/
│   └── <태스크>/       # 태스크 하나 = 폴더 하나
│       ├── env_cfg.py  # 관측·보상·종료·커리큘럼
│       └── ppo_cfg.py  # PPO 설정 (rsl_rl)
├── train.py            # 학습   (--task <태스크>)
├── play.py             # 평가·재생 (--task <태스크> --checkpoint ...)
└── logs/               # 로그·체크포인트 (GitHub 에 안 올림)
```

## sim_core 불러오기

실행 스크립트(`rl/train.py`, `rl/play.py`) 맨 위에 `sim/robot_sim` 을 경로에 추가한다.

```python
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # rl/ 의 한 단계 위 = sim/robot_sim
```

그러면 어디서든 (태스크 폴더 안 파일 포함) 이렇게 불러올 수 있다.

```python
from sim_core import turtleninja_maze_env as tme
```

## 실행 (만든 뒤)

```bash
cd Project2_Turtle_Ninja/sim/robot_sim
$ISAACLAB_PY rl/train.py --task <태스크> --num_envs 256
$ISAACLAB_PY rl/play.py  --task <태스크> --checkpoint rl/logs/<태스크>/<실행>/model_<번호>.pt
```
