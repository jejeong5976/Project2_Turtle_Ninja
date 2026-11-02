"""No ROS graph or motor: deterministic completion/telemetry-loss tests."""
import unittest
from unittest.mock import patch
import drive_sequence as ds

class FakeProbe:
    def __init__(self, clock, settled=True):
        self.clock, self.settled = clock, settled
        self.sent = 0
        self.refresh()
    def refresh(self):
        self.rows=[(self.clock[0], {j: (0 if self.settled else 1, 0) for j in ds.STEER+ds.DRIVE})]
        self.cmds=[(self.clock[0], {j: 0 for j in ds.STEER+ds.DRIVE})]
    def run(self, duration, cmd):
        assert cmd==(0,0,0)
        self.sent+=1; self.clock[0]+=duration; self.refresh()

class FinishParkTest(unittest.TestCase):
    def test_requires_stable_measurement_and_command(self):
        clock=[1.0]; n=FakeProbe(clock)
        with patch.object(ds.time,'monotonic',lambda:clock[0]):
            r=ds.finish_park(n)
        self.assertTrue(r['ok']); self.assertGreaterEqual(r['elapsed_s'],.3)
    def test_stale_telemetry_does_not_send_new_zero(self):
        clock=[1.0]; n=FakeProbe(clock); clock[0]=2
        with patch.object(ds.time,'monotonic',lambda:clock[0]):
            r=ds.finish_park(n)
        self.assertFalse(r['ok']); self.assertEqual(n.sent,0)
    def test_loss_during_wait_cannot_confirm_completion(self):
        clock=[1.0]; n=FakeProbe(clock)
        def lost(duration, cmd):
            n.sent+=1; clock[0]+=.4
        n.run=lost
        with patch.object(ds.time,'monotonic',lambda:clock[0]):
            r=ds.finish_park(n)
        self.assertFalse(r['ok']); self.assertEqual(n.sent,1)
    def test_unsettled_pose_times_out(self):
        clock=[1.0]; n=FakeProbe(clock,False)
        with patch.object(ds.time,'monotonic',lambda:clock[0]):
            r=ds.finish_park(n,timeout=1)
        self.assertFalse(r['ok']); self.assertIn('시간 초과',r['reason'])


class OutcomeTest(unittest.TestCase):
    def test_successful_subset_is_incomplete(self):
        self.assertEqual(ds.run_outcome([{'ok':True}],12,False,True)['status'],'incomplete')
    def test_interrupt_after_motion_before_cleanup_is_incomplete(self):
        self.assertEqual(ds.run_outcome([{'ok':True}]*12,12,True,False,'KeyboardInterrupt')['status'],'incomplete')
    def test_cleanup_failure_is_failed(self):
        r=ds.run_outcome([{'ok':True}]*12+[{'case':'정상 완료 정리','ok':False}],12,True,True)
        self.assertEqual(r['status'],'failed');self.assertEqual(r['failed_cases'],['정상 완료 정리'])
    def test_complete_success(self):
        self.assertEqual(ds.run_outcome([{'ok':True}]*13,12,True,True)['status'],'passed')
    def test_empty_or_exception_is_incomplete(self):
        self.assertEqual(ds.run_outcome([],12,False,False,'RuntimeError')['status'],'incomplete')

class RunModeIntegrationTest(unittest.TestCase):
    def exercise(self, rows, cleanup):
        import tempfile, json
        from types import SimpleNamespace
        from unittest.mock import MagicMock
        n=MagicMock();n.rows=[1];n.cmds=[1]
        with tempfile.TemporaryDirectory() as d:
            a=SimpleNamespace(mode='distance',reps=3,run_seconds=4,distance=.3,seconds=6,
                              stop_seconds=2,skip_shutdown=False,finish_park=True,yes=True,
                              raw=False,beep=False,hardware_map='fake-map',out=d+'/result.json')
            def mode(c): c.res.extend([{'ok':True} for _ in range(rows)])
            with patch.object(ds,'ros_param',return_value='True'), patch.object(ds,'Probe',return_value=n), \
                 patch.object(ds.rclpy,'init'),patch.object(ds.rclpy,'try_shutdown'), \
                 patch.object(ds,'mode_distance',side_effect=mode), \
                 patch.object(ds,'finish_park',side_effect=cleanup) as finish:
                with self.assertRaises(SystemExit) as ex:ds.run_mode(a,{},[],[],.033)
            with open(a.out) as stream: saved=json.load(stream)
            n.send.assert_not_called()
            return ex.exception.code,saved['outcome'],finish.call_count
    def test_failed_cleanup_written_and_exit_nonzero(self):
        code,status,calls=self.exercise(12,lambda n:{'case':'정상 완료 정리','ok':False,'reason':'timeout'})
        self.assertEqual((code,status['status'],calls),(1,'failed',1))
    def test_partial_run_does_not_cleanup_or_pass(self):
        code,status,calls=self.exercise(1,lambda n:{'ok':True})
        self.assertEqual((code,status['status'],calls),(1,'incomplete',0))
    def test_interrupt_cleanup_cannot_pass(self):
        code,status,calls=self.exercise(12,KeyboardInterrupt())
        self.assertEqual((code,status['status'],calls),(1,'incomplete',1))
        self.assertEqual(status['abort_reason'],'KeyboardInterrupt')


class SelectionAndAbortTest(unittest.TestCase):
    def test_unknown_selector_rejected(self):
        with self.assertRaises(ValueError): ds.select_sequence('typo')
    def test_empty_selector_rejected(self):
        with self.assertRaises(ValueError): ds.select_sequence('')
    def test_valid_selector(self):
        self.assertEqual([x[0] for x in ds.select_sequence('전진,후진')], ['전진','후진'])
    def test_distance_failure_cannot_start_next_segment(self):
        from types import SimpleNamespace
        from unittest.mock import MagicMock
        c=MagicMock();c.a=SimpleNamespace(distance=.3,reps=3,stop_seconds=2);c.res=[]
        c.ask.return_value=''
        bad={'ok':False,'name':'전진','odom_enc':None}
        with patch.object(ds,'segment',return_value=bad) as segment:
            ds.mode_distance(c)
        self.assertEqual(segment.call_count,1);self.assertEqual(len(c.res),1)


class AlignmentFailureTest(unittest.TestCase):
    def test_timeout_does_not_send_cleanup_zero(self):
        from types import SimpleNamespace
        from unittest.mock import MagicMock
        clock=[1.0];n=MagicMock();n.rows=[]
        c=ds.Ctx(n,SimpleNamespace(beep=False),[(0,0,0)]*4,.033)
        def spin(*args,**kwargs):clock[0]+=.05
        with patch.object(ds.time,'monotonic',lambda:clock[0]),patch.object(ds.rclpy,'spin_once',side_effect=spin):
            with self.assertRaises(RuntimeError): c.align((.05,0,0),timeout=.2)
        n.run.assert_not_called()

if __name__=='__main__': unittest.main()
