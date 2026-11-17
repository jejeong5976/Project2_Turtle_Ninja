"""ROS 없이 시작 순서·부분 ACTIVE·응답 지연·복구 횟수를 검증함."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from lifecycle_recovery_core import (ACTIVE, CLEANUP, DEACTIVATE, INACTIVE,
                                     RESET, STARTUP, UNCONFIGURED,
                                     LifecycleRecovery, Observation, Snapshot)


def snapshot(now, values=(UNCONFIGURED, UNCONFIGURED), *, active=False,
             inputs=True, services=True, requested=None):
    requested = now if requested is None else requested
    return Snapshot(inputs, services, 'scan/TF 기다림', Observation(active, requested, now),
                    {n: Observation(v, requested, now) for n, v in zip(('map', 'amcl'), values)})


class LifecycleTests(unittest.TestCase):
    def core(self, **kwargs):
        options = dict(stable_s=1, backoff_s=0, command_timeout=2, node_timeout=2,
                       drain_timeout=3, verify_timeout=4, cleanup_timeout=8)
        options.update(kwargs)
        return LifecycleRecovery('localization', ['map', 'amcl'], **options)

    def test_waits_for_scan_tf_services_and_real_state(self):
        c = self.core()
        for t in range(10):
            self.assertIsNone(c.tick(t, snapshot(t, inputs=False)))
        self.assertIsNone(c.tick(10, snapshot(10, services=False)))
        self.assertIsNone(c.tick(11, Snapshot(True, True)))
        self.assertEqual(c.recoveries, 0)
        r = c.tick(12, snapshot(12))
        self.assertEqual(r.command, STARTUP)

    def test_startup_is_not_ready_until_manager_nodes_and_bond_stable(self):
        c = self.core()
        r = c.tick(0, snapshot(0))
        c.complete(r.token, True, .1)
        c.tick(.2, snapshot(.2, (ACTIVE, INACTIVE), active=True))
        self.assertNotEqual(c.phase, 'ready')
        c.tick(.3, snapshot(.3, (ACTIVE, ACTIVE), active=True))
        c.tick(1.29, snapshot(1.29, (ACTIVE, ACTIVE), active=True))
        self.assertNotEqual(c.phase, 'ready')
        c.tick(1.3, snapshot(1.3, (ACTIVE, ACTIVE), active=True))
        self.assertEqual(c.phase, 'ready')
        # 시작 완료 뒤 운영 중 lifecycle을 새로 건드리지 않음.
        self.assertIsNone(c.tick(9, snapshot(9)))
        self.assertEqual(c.phase, 'ready')

    def test_partial_active_reset_false_deactivates_and_cleans_reverse_order(self):
        c = self.core()
        r = c.tick(0, snapshot(0))
        c.complete(r.token, False, .1)
        r = c.tick(.2, snapshot(.2, (ACTIVE, INACTIVE)))
        self.assertEqual(r.command, RESET)
        c.complete(r.token, False, .3)  # manager가 inactive amcl을 deactivate하다 실패한 장면임.
        c.tick(.4, snapshot(.4, (ACTIVE, INACTIVE), requested=.1))
        self.assertIsNone(c.pending)  # reset 전 오래된 상태로 바로 전환하지 않음.
        r = c.tick(.5, snapshot(.5, (ACTIVE, INACTIVE)))
        self.assertEqual((r.target, r.command), ('amcl', CLEANUP))
        c.complete(r.token, True, .6)
        c.tick(.7, snapshot(.7, (ACTIVE, UNCONFIGURED)))
        r = c.tick(.8, snapshot(.8, (ACTIVE, UNCONFIGURED)))
        self.assertEqual((r.target, r.command), ('map', DEACTIVATE))
        c.complete(r.token, True, .9)
        r = c.tick(1.0, snapshot(1.0, (INACTIVE, UNCONFIGURED)))
        self.assertEqual((r.target, r.command), ('map', CLEANUP))
        c.complete(r.token, True, 1.1)
        c.tick(1.2, snapshot(1.2)); c.tick(1.3, snapshot(1.3))
        r = c.tick(1.4, snapshot(1.4))
        self.assertEqual(r.command, STARTUP)
        self.assertEqual(c.recoveries, 1)

    def test_initial_partial_state_uses_reset_not_invalid_configure(self):
        c = self.core()
        self.assertIsNone(c.tick(0, snapshot(0, (ACTIVE, UNCONFIGURED))))
        r = c.tick(.1, snapshot(.1, (ACTIVE, UNCONFIGURED)))
        self.assertEqual(r.command, RESET)

    def test_timeout_keeps_pending_until_remote_completion(self):
        c = self.core()
        r = c.tick(0, snapshot(0))
        self.assertIsNone(c.tick(2.1, snapshot(2.1, requested=.5)))
        self.assertEqual(c.pending.token, r.token)
        self.assertIsNone(c.tick(2.2, snapshot(2.2, requested=1.9)))
        self.assertEqual(c.recoveries, 0)
        # 원래 응답이 늦게 성공하면 RESET을 겹쳐 보내지 않고 검증함.
        c.complete(r.token, True, 2.3)
        self.assertEqual(c.phase, 'verifying')

    def test_timeout_barrier_then_old_callback_does_not_override_reset(self):
        c = self.core()
        old = c.tick(0, snapshot(0))
        c.tick(2.1, snapshot(2.1, requested=1.0))
        c.tick(2.2, snapshot(2.2, requested=2.15))  # timeout 뒤 is_active 응답
        r = c.tick(2.3, snapshot(2.3))
        self.assertEqual(r.command, RESET)
        self.assertFalse(c.complete(old.token, True, 2.4))
        self.assertEqual(c.pending.token, r.token)
        self.assertEqual(c.phase, 'resetting')

    def test_no_response_drain_is_bounded_and_does_not_issue_parallel_reset(self):
        c = self.core()
        r = c.tick(0, snapshot(0))
        c.tick(2.1, snapshot(2.1, requested=0))
        self.assertIsNone(c.tick(5.2, Snapshot(True, True)))
        self.assertEqual(c.phase, 'exhausted')
        self.assertIsNone(c.tick(9, snapshot(9)))
        self.assertFalse(c.complete(r.token, True, 10))
        self.assertEqual(c.phase, 'exhausted')

    def test_three_recovery_cycles_maximum(self):
        c = self.core(); now = 0; starts = resets = 0
        for _ in range(200):
            r = c.tick(now, snapshot(now))
            if r:
                if r.command == STARTUP:
                    starts += 1
                    c.complete(r.token, False, now + .01)
                elif r.command == RESET:
                    resets += 1
                    c.complete(r.token, False, now + .01)
                else:
                    self.fail('전부 unconfigured라 직접 transition 불필요')
            now += .1
            if c.phase == 'exhausted':
                break
        self.assertEqual((starts, resets, c.recoveries), (4, 3, 3))
        self.assertEqual(c.phase, 'exhausted')

    def test_bond_drops_during_verification_retries(self):
        c = self.core(drain_timeout=30)
        r = c.tick(0, snapshot(0)); c.complete(r.token, True, .1)
        c.tick(.2, snapshot(.2, (ACTIVE, ACTIVE), active=True))
        c.tick(.4, snapshot(.4, (ACTIVE, ACTIVE), active=False))
        self.assertEqual(c.phase, 'waiting_native')
        c.tick(.5, snapshot(.5))
        self.assertIsNone(c.tick(11.3, snapshot(11.3)))
        self.assertIsNone(c.tick(11.5, snapshot(11.5, requested=11.3)))
        self.assertEqual(c.phase, 'waiting_native')
        c.tick(11.6, snapshot(11.6))
        r = c.tick(11.7, snapshot(11.7))
        self.assertEqual(r.command, RESET)

    def test_false_node_response_rechecks_actual_state(self):
        c = self.core()
        c.tick(0, snapshot(0, (ACTIVE, INACTIVE)))
        r = c.tick(.1, snapshot(.1)); c.complete(r.token, False, .2)
        r = c.tick(.3, snapshot(.3, (ACTIVE, INACTIVE)))
        self.assertEqual(r.command, CLEANUP)
        c.complete(r.token, False, .4)
        # 응답 false여도 실제 unconfigured이면 중복cleanup 없이 다음 노드로 감.
        c.tick(.5, snapshot(.5, (ACTIVE, UNCONFIGURED)))
        r = c.tick(.6, snapshot(.6, (ACTIVE, UNCONFIGURED)))
        self.assertEqual((r.target, r.command), ('map', DEACTIVATE))

    def test_node_transition_failure_is_bounded(self):
        c = self.core()
        now = 0; changes = 0
        for _ in range(400):
            r = c.tick(now, snapshot(now, (ACTIVE, INACTIVE)))
            if r:
                changes += r.kind == 'node'
                c.complete(r.token, False, now + .01)
            now += .1
            if c.phase == 'exhausted':
                break
        self.assertEqual(c.phase, 'exhausted')
        self.assertEqual(c.recoveries, 3)
        self.assertEqual(changes, 6)

    def test_sensor_loss_does_not_reset_a_healthy_manager(self):
        c = self.core()
        r = c.tick(0, snapshot(0)); c.complete(r.token, True, .1)
        c.tick(.2, snapshot(.2, (ACTIVE, ACTIVE), active=True))
        for now in (1., 3., 8.):
            self.assertIsNone(c.tick(now, snapshot(now, (ACTIVE, ACTIVE), active=True, inputs=False)))
        self.assertEqual(c.phase, 'verifying')
        self.assertEqual(c.recoveries, 0)

    def test_earlier_group_can_recover_before_global_startup_handoff(self):
        c = self.core(stable_s=0, drain_timeout=30)
        c.tick(0, snapshot(0, (ACTIVE, ACTIVE), active=True))
        c.tick(.1, snapshot(.1, (ACTIVE, ACTIVE), active=True))
        self.assertEqual(c.phase, 'ready')
        c.recheck_before_handoff(.2, snapshot(.2, (ACTIVE, INACTIVE), active=False))
        self.assertEqual(c.phase, 'waiting_native')
        c.tick(.25, snapshot(.25, (ACTIVE, INACTIVE)))
        c.tick(11.3, snapshot(11.3, (ACTIVE, INACTIVE)))
        r = c.tick(11.4, snapshot(11.4, (ACTIVE, INACTIVE)))
        self.assertEqual(r.command, RESET)

    def test_native_reconnect_success_needs_no_watchdog_reset(self):
        c = self.core()
        r = c.tick(0, snapshot(0)); c.complete(r.token, True, .1)
        c.tick(.2, snapshot(.2, (ACTIVE, ACTIVE), active=True))
        c.tick(.4, snapshot(.4, (ACTIVE, ACTIVE), active=False))
        self.assertEqual(c.phase, 'waiting_native')
        self.assertIsNone(c.tick(1, snapshot(1)))
        self.assertIsNone(c.tick(2, snapshot(2, (ACTIVE, ACTIVE), active=True)))
        c.tick(2.1, snapshot(2.1, (ACTIVE, ACTIVE), active=True))
        c.tick(3.1, snapshot(3.1, (ACTIVE, ACTIVE), active=True))
        self.assertEqual(c.phase, 'ready')
        self.assertEqual(c.recoveries, 0)

    def test_native_reconnect_unresponsive_manager_never_overlaps_reset(self):
        c = self.core()
        r = c.tick(0, snapshot(0)); c.complete(r.token, True, .1)
        c.tick(.2, snapshot(.2, (ACTIVE, ACTIVE), active=True))
        c.tick(.4, snapshot(.4, (ACTIVE, ACTIVE), active=False))
        self.assertIsNone(c.tick(15, Snapshot(True, True)))
        self.assertEqual(c.phase, 'exhausted')
        self.assertEqual(c.recoveries, 0)

    def test_slow_native_hard_reset_starts_reconnect_window_after_response(self):
        c = self.core(drain_timeout=120)
        r = c.tick(0, snapshot(0)); c.complete(r.token, True, .1)
        c.tick(.2, snapshot(.2, (ACTIVE, ACTIVE), active=True))
        c.tick(.4, Snapshot(True, True))
        self.assertEqual(c.phase, 'waiting_native')
        self.assertIsNone(c.tick(20, Snapshot(True, True)))
        self.assertIsNone(c.tick(30, snapshot(30)))
        self.assertIsNone(c.tick(40, snapshot(40)))
        self.assertEqual(c.phase, 'waiting_native')
        c.tick(41.1, snapshot(41.1))
        self.assertEqual(c.tick(41.2, snapshot(41.2)).command, RESET)

    def test_native_recovered_with_stale_scan_is_not_reset(self):
        c = self.core()
        r = c.tick(0, snapshot(0)); c.complete(r.token, True, .1)
        c.tick(.2, snapshot(.2, (ACTIVE, ACTIVE), active=True))
        c.tick(.4, snapshot(.4, active=False))
        c.tick(1, snapshot(1, (ACTIVE, ACTIVE), active=True, inputs=False))
        self.assertEqual(c.phase, 'verifying')
        self.assertEqual(c.recoveries, 0)

    def test_finalized_or_transitioning_node_waits_bounded_without_invalid_transition(self):
        c = self.core(cleanup_timeout=.5)
        c.tick(0, snapshot(0, (4, 10)))
        r = c.tick(.1, snapshot(.1, (4, 10))); c.complete(r.token, False, .2)
        self.assertIsNone(c.tick(.3, snapshot(.3, (4, 10))))
        self.assertIsNone(c.tick(.8, snapshot(.8, (4, 10))))
        self.assertEqual(c.phase, 'waiting_reset')


if __name__ == '__main__':
    unittest.main()
