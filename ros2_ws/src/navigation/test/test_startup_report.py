"""켜기 스크립트가 지난 준비 상태나 위치 발행만으로 성공을 출력하지 않는지 확인함."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from startup_report import readiness


class ReadyTests(unittest.TestCase):
    def samples(self):
        return {'mission': ({'state': 'STANDBY', 'ready': True}, 10),
                'lifecycle': ({'ready': True}, 10),
                'pose': ({'ready': True, 'state': 'aligned'}, 10),
                'camera': ({'received': True}, 10)}

    def test_only_fresh_complete_state_is_ready(self):
        self.assertTrue(readiness(self.samples(), 11)['ready'])
        self.assertFalse(readiness(self.samples(), 16)['ready'])

    def test_published_pose_is_not_convergence(self):
        samples = self.samples()
        samples['pose'] = ({'ready': False, 'state': 'published'}, 10)
        self.assertFalse(readiness(samples, 11)['ready'])

    def test_current_mission_state_and_camera_are_required(self):
        for key, value in [('mission', {'state': 'FAULT', 'ready': True}),
                           ('mission', {'state': 'STANDBY', 'ready': False}),
                           ('lifecycle', {'ready': False}), ('camera', {})]:
            with self.subTest(key=key, value=value):
                samples = self.samples()
                samples[key] = (value, 10)
                self.assertFalse(readiness(samples, 11)['ready'])


if __name__ == '__main__':
    unittest.main()
