#!/usr/bin/env python3
"""用假 Nav2 action server 验证服务端任务执行器。

覆盖：
  - 正常到达全部点位 → succeeded
  - 暂停 → paused，继续 → succeeded
  - 取消 → cancelled
  - 点位失败 → failed
  - reset → idle

运行（不需要真车，也不要同时开真的 Nav2）：
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 scripts/test_mission_executor.py

脚本使用 ROS_DOMAIN_ID=97，避免和正在运行的导航栈混在一起。
"""

from __future__ import annotations

import os
import sys
import threading
import time
from pathlib import Path

os.environ.setdefault('ROS_DOMAIN_ID', '97')
sys.path.insert(0, str(Path(__file__).resolve().parent))

import web_backend as wb  # noqa: E402

if not wb.ROS_AVAILABLE:
    print('FAIL: rclpy 不可用，请先 source /opt/ros/humble/setup.bash 和 install/setup.bash')
    print(wb.ROS_ERROR)
    sys.exit(1)

import rclpy  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402


class MockNavServer(Node):
    def __init__(self) -> None:
        super().__init__('mock_nav_server')
        self.server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute,
            handle_accepted_callback=self.handle_accepted,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )

    @staticmethod
    def handle_accepted(goal_handle) -> None:
        goal_handle.execute()

    @staticmethod
    def execute(goal_handle):
        x = float(goal_handle.request.pose.pose.position.x)
        if x < 0:
            goal_handle.abort()
            return NavigateToPose.Result()
        for i in range(8):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            feedback = NavigateToPose.Feedback()
            feedback.distance_remaining = max(0.0, 5.0 - i)
            goal_handle.publish_feedback(feedback)
            time.sleep(0.12)
        goal_handle.succeed()
        return NavigateToPose.Result()


def wait_state(executor, wanted, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        state = executor.snapshot()
        if state['state'] == wanted:
            return state
        time.sleep(0.1)
    return executor.snapshot()


def main() -> int:
    runtime = Path('/tmp/test_runtime_mission.json')
    runtime.unlink(missing_ok=True)
    wb.RUNTIME_FILE = runtime
    history_db = Path('/tmp/test_mission_history.db')
    history_db.unlink(missing_ok=True)
    wb.store = wb.MissionStore(history_db)

    rclpy.init()
    server_node = MockNavServer()
    server_executor = MultiThreadedExecutor(num_threads=4)
    server_executor.add_node(server_node)
    server_thread = threading.Thread(target=server_executor.spin, daemon=True)
    server_thread.start()

    client_node = wb.BackendNode()
    mission = wb.MissionExecutor(client_node)
    client_executor = MultiThreadedExecutor(num_threads=4)
    client_executor.add_node(client_node)
    client_thread = threading.Thread(target=client_executor.spin, daemon=True)
    client_thread.start()

    results = []
    try:
        if not client_node.action_client.wait_for_server(timeout_sec=5.0):
            print('FAIL: mock navigate_to_pose action server 未发现')
            return 1

        points = [
            {'id': 1, 'name': 'p1', 'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'dwell': 0.6},
            {'id': 2, 'name': 'p2', 'x': 2.0, 'y': 0.0, 'yaw': 0.0, 'dwell': 0.0},
        ]
        mission.start('success', points, False)
        saw_dwell = False
        deadline = time.monotonic() + 8.0
        state = mission.snapshot()
        while time.monotonic() < deadline:
            state = mission.snapshot()
            if '停留' in (state.get('message') or ''):
                saw_dwell = True
            if state['state'] == 'succeeded':
                break
            time.sleep(0.05)
        results.append(('全部点位到达 → succeeded', state['state'] == 'succeeded' and state['index'] == 2))
        results.append(('到达点位后执行停留', saw_dwell))

        mission.start('pause', points + [{'id': 3, 'name': 'p3', 'x': 3.0, 'y': 0.0, 'yaw': 0.0}], False)
        time.sleep(0.25)
        mission.pause()
        time.sleep(0.4)
        paused = mission.snapshot()
        results.append(('暂停 → paused', paused['state'] == 'paused'))
        mission.resume()
        state = wait_state(mission, 'succeeded', 10.0)
        results.append(('继续 → 全部到达', state['state'] == 'succeeded' and state['index'] == 3))

        mission.start('cancel', points, False)
        time.sleep(0.25)
        mission.cancel()
        time.sleep(0.4)
        results.append(('取消 → cancelled', mission.snapshot()['state'] == 'cancelled'))

        mission.start('fail', [{'id': 1, 'name': 'bad', 'x': -1.0, 'y': 0.0, 'yaw': 0.0}], False)
        state = wait_state(mission, 'failed', 5.0)
        results.append(('点位失败 → failed', state['state'] == 'failed'))

        mission.reset()
        results.append(('reset → idle', mission.snapshot()['state'] == 'idle'))

        runs = wb.store.list_runs()
        results.append(('历史记录 4 条（成功/继续成功/取消/失败）',
                        len(runs) == 4 and {r['state'] for r in runs} == {'succeeded', 'cancelled', 'failed'}))
        results.append(('历史记录包含点位数与耗时',
                        all(r['total_points'] == len(r['points']) and r['duration_sec'] is not None
                            for r in runs)))
    finally:
        try:
            mission.reset()
        except Exception:
            pass
        client_executor.shutdown()
        server_executor.shutdown()
        client_node.destroy_node()
        server_node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        runtime.unlink(missing_ok=True)
        history_db.unlink(missing_ok=True)

    print()
    failed = [name for name, ok in results if not ok]
    for name, ok in results:
        print(f'{"PASS" if ok else "FAIL"}  {name}')
    if failed:
        return 1
    print(f'\n=== 全部 {len(results)} 项通过 ===')
    return 0


if __name__ == '__main__':
    sys.exit(main())
