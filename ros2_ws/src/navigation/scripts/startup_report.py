#!/usr/bin/env python3
"""켜기 결과를 실제 토픽으로 확인해 출력함. 제어·복구·미션 명령은 보내지 않음."""
import argparse
import json
import time


def readiness(samples, now, max_age=5.0):
    """지난 준비 완료 행을 현재 준비로 오해하지 않도록 최근 수신만 씀."""
    def fresh(name):
        value, stamp = samples.get(name, ({}, float('-inf')))
        return value if now - stamp <= max_age else {}
    mission, lifecycle, pose = (fresh(k) for k in ('mission', 'lifecycle', 'pose'))
    camera, health = fresh('camera'), fresh('health')
    waiting = []
    if not lifecycle.get('ready'):
        waiting.append('lifecycle: ' + str(lifecycle.get('phase', '수신 대기')))
    if not pose.get('ready'):
        waiting.append('위치 자동 맞춤: ' + str(pose.get('state', '수신 대기')))
    if not camera.get('received'):
        waiting.append('D435 영상 수신 대기')
    if mission.get('state') != 'STANDBY' or not mission.get('ready'):
        waiting.append('미션: ' + str(mission.get('detail', 'STANDBY 수신 대기')))
    return {'ready': not waiting, 'waiting': waiting, 'mission': mission,
            'lifecycle': lifecycle, 'pose': pose, 'health': health}


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    from diagnostic_msgs.msg import DiagnosticStatus
    from interfaces.msg import MissionStatus

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=240.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node('startup_report')
    samples = {}
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE)

    def save(name, value):
        samples[name] = (value, time.monotonic())

    def status(name, msg):
        try:
            value = json.loads(msg.data)
            if isinstance(value, dict):
                save(name, value)
        except (ValueError, TypeError):
            pass

    node.create_subscription(String, '/startup/lifecycle_status', lambda m: status('lifecycle', m), transient)
    node.create_subscription(String, '/startup/pose_match_status', lambda m: status('pose', m), transient)
    node.create_subscription(Image, '/camera/camera/color/image_raw',
                             lambda _: save('camera', {'received': True}), qos_profile_sensor_data)
    node.create_subscription(DiagnosticStatus, '/localization_health',
                             lambda m: save('health', {v.key: v.value for v in m.values}), 10)
    node.create_subscription(MissionStatus, '/mission/status',
                             lambda m: save('mission', {'state': m.state, 'ready': m.ready,
                                                       'detail': m.detail, 'wait_reasons': list(m.wait_reasons)}), 10)
    started = time.monotonic()
    previous = None
    last_print = 0.0
    code = 2
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            now = time.monotonic()
            result = readiness(samples, now)
            waiting = tuple(result['waiting'])
            if waiting != previous or now - last_print >= 10.0 or result['ready']:
                ratio = result['health'].get('match', '')
                try:
                    match_text = f'{float(ratio) * 100:.1f}%'
                except (ValueError, TypeError):
                    match_text = '수신 대기'
                mission = result['mission']
                print(f"지도 맞음 {match_text} · 미션 {mission.get('state', '수신 대기')} "
                      f"{'준비 완료' if mission.get('ready') else '준비 중'}", flush=True)
                if waiting:
                    print('  ' + ' / '.join(waiting), flush=True)
                previous, last_print = waiting, now
            if result['ready']:
                print('✓ 위치 자동 맞춤 · lifecycle · D435 · STANDBY 준비 완료', flush=True)
                code = 0
                break
            if now - started >= args.timeout:
                print('준비 상태 출력 대기 종료. 실행 중인 자동 맞춤·복구 노드는 계속 동작함.', flush=True)
                break
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
