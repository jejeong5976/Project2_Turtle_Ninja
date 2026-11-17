"""실물 명령을 가짜 실행기로 바꿔 한 번에 켜는 순서와 늦은 서비스 재시도를 확인함."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'run_all.sh'


class RunAllTests(unittest.TestCase):
    def test_delayed_services_and_no_interactive_prompt(self):
        with tempfile.TemporaryDirectory(prefix='startup_run_all_') as dirname:
            root = Path(dirname)
            (root / 'ros2_ws/install').mkdir(parents=True)
            (root / 'bin').mkdir()
            (root / 'tools').mkdir()
            (root / 'ros2_ws/install/setup.bash').write_text(
                'export PATH="$RUN_ALL_TEST_ROOT/bin:$PATH"\n')
            ros = root / 'bin/ros2'
            ros.write_text('''#!/usr/bin/env python3
import os,sys
from pathlib import Path
r=Path(os.environ['RUN_ALL_TEST_ROOT'])
a=sys.argv[1:]
with (r/'commands').open('a') as f: f.write(' '.join(a)+'\\n')
if a[:2]==['param','set']:
 p=r/('attempt_'+a[2].replace('/',''))
 n=int(p.read_text())+1 if p.exists() else 1
 p.write_text(str(n))
 sys.exit(1 if n<2 else 0)
if a[:3]==['run','navigation','startup_report.py']:
 print('지도 맞음 97.0% · 미션 STANDBY 준비 완료')
''')
            ros.chmod(0o755)
            fake_pkill = root / 'bin/pkill'
            fake_pkill.write_text('#!/bin/sh\nprintf "%s\\n" "$*" > "$RUN_ALL_TEST_ROOT/pkill_args"\n')
            fake_pkill.chmod(0o755)
            (root / 'tools/stop_robot.sh').write_text('exit 0\n')
            (root / 'tools/start_robot_amcl.sh').write_text('''echo "$$" > "$RUN_ALL_TEST_ROOT/launch_pid"
printf '%s\\n' "$SKIP_CONFIRM $*" > "$RUN_ALL_TEST_ROOT/launch_args"
sleep 30
''')
            env = dict(os.environ, RUN_ALL_TEST_ROOT=str(root), ROBOT_PROJECT=str(root),
                       ROBOT_TOOLS=str(root / 'tools'), RUN_ALL_LOG_DIR=str(root / 'logs'),
                       RUN_ALL_BAG_DIR=str(root / 'bags'))
            try:
                result = subprocess.run(['bash', str(SCRIPT)], env=env, input='',
                                        capture_output=True, text=True, timeout=20)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                calls = (root / 'commands').read_text().splitlines()
                self.assertEqual(sum(c.startswith('param set /control_master ') for c in calls), 2)
                self.assertEqual(sum(c.startswith('param set /perception ') for c in calls), 2)
                self.assertLess(calls.index('param set /perception wrist.held.enable true'),
                                next(i for i,c in enumerate(calls) if c.startswith('run navigation')))
                self.assertIn('run robot_cli robot_cli', calls)
                self.assertIn('STANDBY 준비 완료', result.stdout)
                launch = (root / 'launch_args').read_text()
                for arg in ('startup_recovery:=true', 'auto_initial_pose:=true', 'wrist_respawn:=true'):
                    self.assertIn(arg, launch)
                self.assertTrue(launch.startswith('1 '))
                self.assertEqual((root / 'pkill_args').read_text().strip(), '-INT -f ros2 bag record')
            finally:
                pidfile = root / 'launch_pid'
                if pidfile.exists():
                    try:
                        os.killpg(int(pidfile.read_text()), signal.SIGTERM)
                    except ProcessLookupError:
                        pass


if __name__ == '__main__':
    unittest.main()
