#!/usr/bin/env python3
"""Route runner: drives the robot through a testing_env's goal list with Nav2 (NavigateToPose), on command.

Started by navigation.launch.py (idle until told to go). Control:
  ros2 service call /route_runner/start  std_srvs/srv/Trigger {}     # begin the route
  ros2 service call /route_runner/cancel std_srvs/srv/Trigger {}     # cancel the current goal and stop
Status (also what the GUI shows): topic /route_runner/status (std_msgs/String, JSON) and, if `status_file` is
set, the same JSON in that file (rewritten twice a second).

Parameters
  env               name in testing_envs.yaml (e.g. testing_env2); its `goals` are used
  registry_file     path of testing_envs.yaml (default: the installed esdf_sim_bringup/config/testing_envs.yaml)
  goals_str         optional "x1,y1;x2,y2;..." that overrides the registry
  laps              how many times to drive the whole list (default 1)
  auto_start        start by itself once Nav2 is ready (default false)
  auto_start_delay  seconds (sim time) to wait after Nav2 is ready before auto-starting (default 8)
  goal_timeout      seconds (sim time) before one goal is cancelled and marked 'timeout' (default 240)
  stop_on_failure   abort the whole run when a goal fails (default false: log it and go on)
  status_file       JSON status path ('' = none)
The heading of each goal points at the next goal (the last one keeps the previous heading).
"""
import json
import math
import os

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

STATUS_NAMES = {GoalStatus.STATUS_SUCCEEDED: 'succeeded', GoalStatus.STATUS_CANCELED: 'canceled',
                GoalStatus.STATUS_ABORTED: 'aborted'}


class RouteRunner(Node):
    def __init__(self):
        super().__init__('route_runner')
        self.declare_parameter('env', '')
        self.declare_parameter('registry_file', '')
        self.declare_parameter('goals_str', '')
        self.declare_parameter('laps', 1)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('auto_start_delay', 8.0)
        self.declare_parameter('goal_timeout', 240.0)
        self.declare_parameter('stop_on_failure', False)
        self.declare_parameter('status_file', '')
        g = self.get_parameter
        self.env = g('env').value
        self.laps = max(1, int(g('laps').value))
        self.auto_start = bool(g('auto_start').value)
        self.auto_delay = float(g('auto_start_delay').value)
        self.goal_timeout = float(g('goal_timeout').value)
        self.stop_on_failure = bool(g('stop_on_failure').value)
        self.status_file = g('status_file').value
        self.goals = self._load_goals(g('goals_str').value, g('registry_file').value)

        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub = self.create_publisher(String, '~/status', 10)
        self.create_service(Trigger, '~/start', self._on_start)
        self.create_service(Trigger, '~/cancel', self._on_cancel)

        self.state = 'idle' if self.goals else 'no_route'
        self.lap = 0
        self.idx = 0
        self.results = []
        self.goal_handle = None
        self.cancel_requested = False
        self.run_start = None
        self.goal_start = None
        self.ready_since = None
        self.message = ''
        self.final_elapsed = 0.0
        self._timed_out = False
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"route_runner: env '{self.env}', {len(self.goals)} goals, {self.laps} lap(s), "
            f"auto_start={self.auto_start}. Start with: ros2 service call /route_runner/start std_srvs/srv/Trigger")

    # ------------------------------------------------------------------ route
    def _load_goals(self, goals_str, registry_file):
        if goals_str.strip():
            try:
                return [tuple(float(v) for v in pt.split(',')) for pt in goals_str.split(';') if pt.strip()]
            except ValueError:
                self.get_logger().error(f"bad goals_str '{goals_str}' (expected 'x,y;x,y')")
                return []
        if not self.env:
            return []
        path = registry_file
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('esdf_sim_bringup'), 'config', 'testing_envs.yaml')
        try:
            envs = yaml.safe_load(open(path))['envs']
            return [(float(x), float(y)) for x, y in envs[self.env]['goals']]
        except Exception as ex:
            self.get_logger().error(f'could not read goals of {self.env} from {path}: {ex}')
            return []

    def _pose(self, i):
        x, y = self.goals[i]
        if i + 1 < len(self.goals):
            nx, ny = self.goals[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        elif i > 0:
            px, py = self.goals[i - 1]
            yaw = math.atan2(y - py, x - px)
        else:
            yaw = 0.0
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return p

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ services
    def _on_start(self, req, res):
        if not self.goals:
            res.success, res.message = False, 'no route loaded (check the env name / registry)'
        elif self.state == 'running':
            res.success, res.message = False, 'already running'
        elif not self.ac.wait_for_server(timeout_sec=0.0):
            res.success, res.message = False, 'Nav2 navigate_to_pose is not available yet'
        else:
            self._begin()
            res.success, res.message = True, f'route started ({len(self.goals)} goals x {self.laps} lap(s))'
        self.get_logger().info(f'start requested: {res.message}')
        return res

    def _on_cancel(self, req, res):
        if self.state != 'running':
            res.success, res.message = False, 'not running'
        else:
            self.cancel_requested = True
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            res.success, res.message = True, 'cancelling'
        return res

    # ------------------------------------------------------------------ state machine (callback driven)
    def _begin(self):
        self.state, self.lap, self.idx = 'running', 0, 0
        self.results, self.cancel_requested = [], False
        self.run_start = self._now()
        self._send_current()

    def _send_current(self):
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(self.idx)
        self.goal_start = self._now()
        self.goal_handle = None
        self.ac.send_goal_async(goal).add_done_callback(self._on_goal_response)
        self.get_logger().info(f'lap {self.lap + 1}/{self.laps}, goal {self.idx + 1}/{len(self.goals)}: {self.goals[self.idx]}')

    def _on_goal_response(self, fut):
        h = fut.result()
        if not h.accepted:
            self._record('rejected')
            return
        self.goal_handle = h
        if self.cancel_requested:
            h.cancel_goal_async()
        h.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        st = fut.result().status
        name = 'timeout' if self._timed_out else STATUS_NAMES.get(st, str(st))
        self._timed_out = False
        self._record(name)

    def _record(self, status):
        self.results.append(dict(lap=self.lap + 1, goal=self.idx + 1, x=self.goals[self.idx][0], y=self.goals[self.idx][1],
                                 status=status, time_s=round(self._now() - self.goal_start, 2)))
        self.goal_handle = None
        if self.cancel_requested:
            self._finish('cancelled'); return
        if status != 'succeeded' and self.stop_on_failure:
            self._finish('failed'); return
        self.idx += 1
        if self.idx >= len(self.goals):
            self.idx, self.lap = 0, self.lap + 1
            if self.lap >= self.laps:
                self._finish('finished'); return
        self._send_current()

    def _finish(self, state):
        self.state = state
        self.final_elapsed = self._now() - self.run_start
        self.message = f'{state} after {self.final_elapsed:.0f} s (sim time)'
        self.get_logger().info(f'route {self.message}')

    # ------------------------------------------------------------------ periodic: auto-start, timeout, status
    def _tick(self):
        now = self._now()
        ready = self.ac.wait_for_server(timeout_sec=0.0)
        if self.state == 'idle' and ready and self.ready_since is None:
            self.ready_since = now
        if self.state == 'idle' and self.auto_start and ready and self.ready_since is not None \
                and now - self.ready_since >= self.auto_delay:
            self.get_logger().info('auto_start: Nav2 ready, starting the route')
            self._begin()
        if self.state == 'running' and self.goal_handle is not None and now - self.goal_start > self.goal_timeout \
                and not self._timed_out:
            self.get_logger().warn(f'goal {self.idx + 1} timed out after {self.goal_timeout:.0f} s, cancelling it')
            self._timed_out = True
            self.goal_handle.cancel_goal_async()
        self._publish(ready, now)

    def _publish(self, ready, now):
        s = dict(env=self.env, state=self.state, nav2_ready=bool(ready), laps=self.laps, lap=min(self.lap + 1, self.laps),
                 goals_total=len(self.goals), goal=self.idx + 1 if self.state == 'running' else None,
                 goal_xy=list(self.goals[self.idx]) if self.state == 'running' and self.goals else None,
                 elapsed_s=round(now - self.run_start, 1) if self.state == 'running' else round(self.final_elapsed, 1),
                 results=self.results, message=self.message)
        msg = json.dumps(s)
        self.pub.publish(String(data=msg))
        if self.status_file:
            try:
                tmp = self.status_file + '.tmp'
                with open(tmp, 'w') as f:
                    f.write(msg)
                os.replace(tmp, self.status_file)
            except OSError:
                pass


def main():
    rclpy.init()
    node = RouteRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
