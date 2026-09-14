"""Measured Go2 motion boundary for the shared asynchronous ESCAPE runtime."""
from __future__ import annotations

from collections import OrderedDict, deque
from dataclasses import replace
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from std_srvs.srv import SetBool
from s2e_vlm_msgs.action import Rotate
from s2e_vlm_msgs.msg import HabitatActionCommand, HabitatActionResult, StampedPose, NavigationTask, NodeStatus, SweepCommand, SweepAck, PointGoal

from .motion import MeasuredMotion, MotionLimits, MotionOutput, ROTATE_TIMEOUT_S_PER_DEG, wrap
from .sensor_geometry import odom_sample, stamp_ns, yaw_from_quaternion
from .sport_client import SportClient, SportClientError
from .body_look_control import BODY_COMMAND_TIMEOUT_S


class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__("go2_action_executor")
        defaults = {
            "command_socket_path": "/run/s2e-robot/commands.sock",
            "motion_enabled": False, "body_look_enabled": False, "forward_speed_mps": 0.50,
            "turn_speed_radps": 0.45, "minimum_rotate_speed_radps": 0.40,
            "rotate_proportional_gain": 1.2, "rotate_settle_samples": 2,
            "rotate_settle_timeout_s": 2.0,
            "rotate_settle_window_s": 0.0,
            "rotate_recoil_limit_deg": 0.0,
            "rotate_min_timeout_s": 0.0,
            "max_forward_distance_m": 0.25, "max_turn_angle_deg": 30.0,
            "max_rotate_goal_angle_deg": 180.0, "command_rate_hz": 20.0,
            "socket_timeout_s": 0.10, "odom_timeout_s": 0.5,
            "camera_timeout_s": 1.0,
            "command_max_age_s": 0.5, "macro_timeout_s": 6.0,
            "linear_acceleration_mps2": 1.0,
            "odometry_topic": "/s2e/robot/sensors/odom",
            "planner_status_node_name": "vlm_node",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._p = lambda name: self.get_parameter(name).value
        minimum_rotation_timeout = float(self._p("rotate_min_timeout_s"))
        if not math.isfinite(minimum_rotation_timeout) or not 0 <= minimum_rotation_timeout <= 60:
            raise ValueError("invalid rotate_min_timeout_s")
        self._limits = MotionLimits(linear_speed=self._positive("forward_speed_mps"),
            angular_speed=self._positive("turn_speed_radps"),
            minimum_turn_speed=self._positive("minimum_rotate_speed_radps"),
            turn_gain=self._positive("rotate_proportional_gain"),
            settle_samples=int(self._positive("rotate_settle_samples")),
            rotation_settle_timeout=self._positive("rotate_settle_timeout_s"),
            rotation_settle_window=float(self._p("rotate_settle_window_s")),
            rotation_recoil_limit=math.radians(float(self._p("rotate_recoil_limit_deg"))),
            acceleration=self._positive("linear_acceleration_mps2"),
            odom_timeout=self._positive("odom_timeout_s"))
        self._client = SportClient(str(self._p("command_socket_path")), self._positive("socket_timeout_s"))
        self._lock = threading.RLock()
        self._motion_enabled = False
        self._enabled_after = 0
        self._active = None
        self._body_command = None
        self._body_held = False
        self._body_returning = False
        self._body_action = ""
        self._body_after = None
        self._body_id = None
        self._body_deadline = 0.
        self._body_recovery_reason = ''
        self._command = None
        self._rotate_handle = None
        self._rotate_future = None
        self._rotate_reserved = False
        self._odom = None
        self._translation_reference = None
        self._odoms = deque(maxlen=250)
        self._last_pose = None
        self._camera_received = 0.0
        self._camera_stamp = 0
        self._pending_result = None
        self._health = {}
        self._episode_id = ""
        self._pointgoal = None
        self._results = OrderedDict()
        self._step_index = 0
        self._highest_generation = 0
        self._sweep = None
        self._early_pause_acks = deque(maxlen=8)
        self._executor_paused = False
        self._quiescent_acked = False
        self._last_fault = ""
        reliable = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        task_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._result_publisher = self.create_publisher(HabitatActionResult, "/s2e/sim/action_result", reliable)
        self._ack_publisher = self.create_publisher(SweepAck, "/s2e/sweep/ack", reliable)
        self._status_publisher = self.create_publisher(NodeStatus, "/s2e/controller/status", reliable)
        self.create_subscription(HabitatActionCommand, "/s2e/controller/habitat_action", self._on_command, reliable)
        self.create_subscription(NavigationTask, "/s2e/task", self._on_task, task_qos)
        self.create_subscription(PointGoal, "/s2e/sensors/pointgoal", self._on_pointgoal, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(self._p("odometry_topic")), self._on_odom, qos_profile_sensor_data)
        self.create_subscription(StampedPose, "/s2e/odometry/pose", self._on_pose, qos_profile_sensor_data)
        self.create_subscription(Image, "/s2e/sensors/camera/image", self._on_image, qos_profile_sensor_data)
        # Point clouds and aggregate SLAM sensor health are not policy or
        # actuation inputs. The pose and RGB streams are validated directly.
        planner = str(self._p('planner_status_node_name'))
        if planner not in ('vlm_node','direct_goal_pixnav'):
            raise ValueError('invalid planner_status_node_name')
        for topic in ("/s2e/status/"+planner, "/s2e/status/pixnav_runtime_node", "/s2e/status/sweep_scheduler_node", "/s2e/robot/observation_status"):
            self.create_subscription(NodeStatus, topic, self._on_health, reliable)
        self.create_subscription(SweepCommand, "/s2e/sweep/command", self._on_sweep_command, reliable)
        self.create_subscription(SweepAck, "/s2e/sweep/ack", self._on_sweep_ack, reliable)
        emergency = ReentrantCallbackGroup()
        self.create_service(SetBool, "/s2e/robot/set_motion_enabled", self._set_motion_enabled, callback_group=emergency)
        self._rotate_server = ActionServer(self, Rotate, "/s2e/controller/rotate",
            execute_callback=self._execute_rotate, goal_callback=self._accept_rotate_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=emergency)
        self.create_timer(1.0 / self._positive("command_rate_hz"), self._tick)
        self.create_timer(0.5, self._publish_status)
        # Startup must never restore physical authorization after a process restart.
        if self._p("motion_enabled"):
            raise ValueError("startup motion is disabled; use set_motion_enabled after sensor readiness")
        self.get_logger().info("Full ESCAPE motion boundary ready; motion disabled")

    def _positive(self, name):
        value = float(self._p(name))
        if not math.isfinite(value) or value <= 0:
            raise ValueError("invalid " + name)
        return value

    def _on_odom(self, message):
        with self._lock:
            try:
                value = odom_sample(message, time.monotonic())
                age = (self.get_clock().now().nanoseconds - value.stamp_ns) * 1e-9
                if age < -0.10 or age > self._limits.odom_timeout:
                    return
                if self._odom and value.stamp_ns <= self._odom.stamp_ns:
                    if value.stamp_ns < self._odom.stamp_ns:
                        self._fault("ODOM_CLOCK_RESET")
                    return
                # Some odometry publishers leave twist at zero. Derive a second
                # velocity estimate over ~100 ms so zero-filled twist cannot
                # falsely certify a stopped robot.
                older = next((entry for entry in reversed(self._odoms)
                              if 80_000_000 <= value.stamp_ns-entry.stamp_ns <= 200_000_000), None)
                if older is not None and older.frame == value.frame:
                    dt = (value.stamp_ns-older.stamp_ns)*1e-9
                    speed = math.hypot(value.x-older.x, value.y-older.y)/dt
                    yaw_rate = wrap(value.yaw-older.yaw)/dt
                    value = replace(value, speed=max(value.speed, speed),
                                    yaw_rate=max((value.yaw_rate, yaw_rate), key=abs))
                self._odom = value
                self._odoms.append(value)
            except ValueError:
                self._fault("INVALID_ODOMETRY")

    def _on_pose(self, message):
        with self._lock:
            age = (self.get_clock().now().nanoseconds-stamp_ns(message.header.stamp))*1e-9
            try:
                yaw_from_quaternion(message.pose.orientation)
                if (message.header.frame_id != "robot_origin" or message.status != "OK"
                        or not math.isfinite(message.pose.position.x) or not math.isfinite(message.pose.position.y)
                        or not -0.1 <= age <= self._limits.odom_timeout):
                    return
            except ValueError:
                return
            if self._last_pose is None or stamp_ns(message.header.stamp) > stamp_ns(self._last_pose.header.stamp):
                self._last_pose = message
                self._flush_result()

    def _on_image(self, message):
        stamp = stamp_ns(message.header.stamp)
        age = (self.get_clock().now().nanoseconds - stamp) * 1e-9
        with self._lock:
            if stamp > self._camera_stamp and -0.1 <= age < self._positive("camera_timeout_s"):
                self._camera_stamp = stamp
                self._camera_received = time.monotonic()

    def _on_health(self, message):
        age = (self.get_clock().now().nanoseconds-stamp_ns(message.header.stamp))*1e-9
        with self._lock:
            if -0.1 <= age < 3.0:
                self._health[message.node_name] = (time.monotonic(), message.is_healthy)

    def _admission_error(self, now):
        if self._odom is None or now-self._odom.received_s > self._limits.odom_timeout:
            return "ODOM_STALE"
        if self._last_pose is None or not -0.1 <= (self.get_clock().now().nanoseconds-stamp_ns(self._last_pose.header.stamp))*1e-9 <= self._limits.odom_timeout:
            return "LOCALIZED_POSE_STALE"
        if now-self._camera_received > self._positive("camera_timeout_s"):
            return "CAMERA_STALE"
        for name in (str(self._p('planner_status_node_name')), "pixnav_runtime_node", "sweep_scheduler_node", "rtab_pointgoal_adapter"):
            receipt, healthy = self._health.get(name, (0, False))
            if not healthy or now-receipt > 3.0:
                return "NODE_UNAVAILABLE:" + name
        return ""

    def _on_pointgoal(self, message):
        with self._lock:
            if (message.episode_id == self._episode_id
                    and math.isfinite(message.distance_m) and message.distance_m >= 0
                    and math.isfinite(message.bearing_rad)
                    and (self._pointgoal is None or stamp_ns(message.header.stamp) > stamp_ns(self._pointgoal.header.stamp))):
                self._pointgoal = message

    def _on_task(self, message):
        with self._lock:
            if message.episode_id == self._episode_id:
                return
            self._finish(MotionOutput(terminal="TASK_REPLACED"))
            self._flush_result(failure="TASK_REPLACED")
            self._translation_reference = None
            self._episode_id = message.episode_id
            self._pointgoal = None
            self._step_index = 0
            self._highest_generation = 0
            self._sweep = None
            self._executor_paused = self._quiescent_acked = False
            self._early_pause_acks.clear()

    def _on_command(self, message):
        with self._lock:
            key = (message.episode_id, message.command_id)
            if key in self._results:
                self._result_publisher.publish(self._results[key])
                return
            if self._pending_result and key == (self._pending_result[0].episode_id, self._pending_result[0].command_id):
                return
            if self._command is not None and key == (self._command.episode_id, self._command.command_id):
                return
            if any(c is not None and key == (c.episode_id, c.command_id)
                   for c in (self._body_command, self._body_after)):
                return
            if not message.command_id or message.episode_id != self._episode_id:
                return
            action = message.action
            if action == "stop" and self._body_held and not self._body_returning:
                self._begin_body_return(message)
                return
            if action == "stop":
                self._abort_body("BODY_LOOK_STOPPED")
                self._finish(MotionOutput(terminal="STOPPED"))
                error = self._stop() if self._motion_enabled else ""
                self._publish_result(message, executed=not error, status="STOPPED" if not error else error)
                return
            error = self._admission_error(time.monotonic())
            age = (self.get_clock().now().nanoseconds-stamp_ns(message.header.stamp))*1e-9
            if not self._motion_enabled:
                error = "MOTION_INHIBITED"
            elif self._body_recovery_reason:
                error = 'BODY_RECOVERY_IN_PROGRESS'
            elif not -0.05 <= age <= self._positive("command_max_age_s") or stamp_ns(message.header.stamp) < self._enabled_after:
                error = "STALE_COMMAND"
            elif (self._pointgoal is None or self._pointgoal.episode_id != message.episode_id
                    or stamp_ns(self._pointgoal.header.stamp) < self._enabled_after
                    or not -0.1 <= (self.get_clock().now().nanoseconds-stamp_ns(self._pointgoal.header.stamp))*1e-9 <= self._positive("camera_timeout_s")):
                error = "POINTGOAL_UNAVAILABLE_OR_STALE"
            elif not message.trajectory_id or message.generation < 1:
                error = "INVALID_POLICY_OWNER"
            elif message.generation < self._highest_generation:
                error = "STALE_POLICY_GENERATION"
            elif (self._active is not None or self._rotate_reserved or self._pending_result
                  or self._body_command is not None or self._body_returning):
                error = "MOTION_BUSY"
            elif self._sweep is not None and (self._sweep.command != SweepCommand.REQUEST_PAUSE or self._executor_paused):
                error = "SWEEP_OWNS_MOTION"
            if error:
                self._publish_result(message, executed=False, status=error)
                return
            if self._handle_body_command(message):
                return
            self._start_motion(message)

    def _start_motion(self, message):
        action = message.action
        try:
            if action in {"move_forward", "move_backward"}:
                amount = float(message.forward_distance_m)
                if not 0 < amount <= self._positive("max_forward_distance_m"):
                    raise ValueError("INVALID_FORWARD_DISTANCE")
                kind, target = "translate", amount * (1 if action == "move_forward" else -1)
            elif action in {"turn_left", "turn_right"}:
                amount = float(message.turn_angle_deg)
                if not 0 < amount <= self._positive("max_turn_angle_deg"):
                    raise ValueError("INVALID_TURN_ANGLE")
                kind, target = "rotate", math.radians(amount) * (1 if action == "turn_left" else -1)
            else:
                raise ValueError("UNSUPPORTED_FIXED_CAMERA_ACTION")
            timeout = self._positive("macro_timeout_s")
            if kind == "rotate":
                timeout = max(timeout, abs(math.degrees(target)) * ROTATE_TIMEOUT_S_PER_DEG,
                              float(self._p("rotate_min_timeout_s")))
            reference = self._translation_reference
            if reference is None or reference.frame != self._odom.frame:
                reference = self._odom
            self._active = MeasuredMotion(kind=kind, target=target, sample=self._odom,
                now=time.monotonic(), timeout=timeout, limits=self._limits,
                heading_yaw=reference.yaw if kind == "translate" else None)
            # VLM handoffs and stationary waits do not request a new
            # bearing. Only an actual turn or execution reset releases it.
            self._translation_reference = reference if kind == "translate" else None
            self._command = message
            self._highest_generation = max(self._highest_generation, message.generation)
        except ValueError as error:
            self._publish_result(message, executed=False, status=str(error))

    def _handle_body_command(self, message):
        if self._body_held:
            self._begin_body_return(message)
            return True
        if message.action not in ('look_up', 'look_down') or not self._p('body_look_enabled'):
            return False
        try:
            self._client.body_look(message.action)
            self._body_command = message
            self._body_action = message.action
            self._body_deadline = time.monotonic()+BODY_COMMAND_TIMEOUT_S
            self._body_id = None
            self._highest_generation = max(self._highest_generation, message.generation)
            self.get_logger().info('robot_body_look_started '+message.command_id)
        except SportClientError:
            self._publish_result(message, executed=False, status='BODY_LOOK_TRANSPORT_FAILED')
            self._fault('BODY_LOOK_TRANSPORT_FAILED')
        return True

    def _begin_body_return(self, after):
        self._body_after = after
        try:
            self._client.body_return()
            self._body_returning = True
            self._body_deadline = time.monotonic()+BODY_COMMAND_TIMEOUT_S
        except SportClientError:
            self._fault('BODY_RETURN_TRANSPORT_FAILED')

    def _abort_body(self, reason):
        commands = (self._body_command, self._body_after)
        owned = self._body_command is not None or self._body_held or self._body_returning
        self._body_command = self._body_after = None
        self._body_held = self._body_returning = False
        self._body_recovery_reason = ''
        self._body_id = None
        if owned:
            self._stop()
        for command in commands:
            if command is not None:
                self._publish_result(command, executed=False, status=reason)

    def _tick_body(self, now):
        if self._body_command is None and not self._body_held and not self._body_returning:
            return
        if not self._motion_enabled:
            self._abort_body('MOTION_INHIBITED')
            return
        try:
            status = self._client.body_status()  # Renews the same bounded command lease.
        except SportClientError:
            self._fault('BODY_STATUS_TRANSPORT_FAILED')
            return
        if self._body_id is None:
            self._body_id = status.look_id
        if status.look_id != self._body_id or not status.look_id:
            self._fault('BODY_LOOK_OWNER_CHANGED')
            return
        if status.state == 5:
            self._fault(status.error or 'BODY_LOOK_FAILED')
            return
        if now > self._body_deadline:
            self._fault('BODY_LOOK_EXECUTION_TIMEOUT')
            return
        if self._body_recovery_reason:
            # Planner failure does not invalidate healthy local posture
            # feedback. Finish a bounded neutral return before latching it.
            # Sensor/operator/transport failures still stop immediately.
            if status.state == 2 and not self._body_returning:
                try:
                    self._client.body_return()
                    self._body_returning = True
                    self._body_deadline = now+BODY_COMMAND_TIMEOUT_S
                except SportClientError:
                    self._fault('BODY_RETURN_TRANSPORT_FAILED')
            elif status.state == 4:
                reason = self._body_recovery_reason
                self._fault(reason)
            return
        if status.state == 2 and self._body_command is not None:
            if status.frame_stamp_ns <= 0:
                self._fault('BODY_LOOK_IMAGE_MISSING')
                return
            command, self._body_command = self._body_command, None
            self._body_held = True
            self._body_deadline = now+15.
            # A successful look acknowledges the tilted observation, while
            # the physical posture stays held for the next policy inference.
            self._pending_result = (command, MotionOutput(terminal='OK'),
                                    status.frame_stamp_ns, now+1.)
            self._flush_result()
        if status.state == 4 and self._body_returning:
            ready = (self._last_pose is not None and self._odom is not None and
                     stamp_ns(self._last_pose.header.stamp) >= status.frame_stamp_ns > 0 and
                     self._odom.stamp_ns >= status.frame_stamp_ns and
                     abs(self._odom.speed) <= self._limits.stop_speed and
                     abs(self._odom.yaw_rate) <= self._limits.stop_yaw_rate)
            if not ready:
                return
            after, self._body_after = self._body_after, None
            self._body_held = self._body_returning = False
            self._body_id = None
            if after is None:
                return
            if after.action == 'stop':
                self._publish_result(after, executed=True, status='STOPPED')
            elif after.action in ('look_up', 'look_down'):
                # An opposite look restores neutral. Repeated same-direction
                # increments exceed this single 15-degree validation; do not fake
                # another camera step or silently replay the same view.
                inverse = after.action != self._body_action
                self._publish_result(after, executed=inverse,
                    status='OK' if inverse else 'BODY_LOOK_INCREMENT_UNVALIDATED')
            else:
                self._start_motion(after)

    def _accept_rotate_goal(self, request):
        with self._lock:
            if self._body_command is not None or self._body_held or self._body_returning:
                return GoalResponse.REJECT
            values = (request.target_yaw_delta_deg, request.max_yaw_rate_deg_s, request.tolerance_deg, request.timeout_s)
            if (not all(math.isfinite(v) for v in values) or not 0 < abs(values[0]) <= self._positive("max_rotate_goal_angle_deg")
                or any(v <= 0 for v in values[1:]) or values[2] > 5.0 or values[3] > 60.0
                or self._active or self._pending_result or self._rotate_reserved or not self._motion_enabled or self._admission_error(time.monotonic())
                or (self._sweep is not None and self._sweep.command != SweepCommand.START_SWEEP)):
                return GoalResponse.REJECT
            self._rotate_reserved = request
            return GoalResponse.ACCEPT

    async def _execute_rotate(self, handle):
        with self._lock:
            request = handle.request
            future = Future()
            if (self._rotate_reserved is not request or not self._motion_enabled or self._active
                    or self._pending_result or self._admission_error(time.monotonic())
                    or handle.is_cancel_requested):
                self._rotate_reserved = False
                handle.abort()
                result = Rotate.Result()
                result.success = False
                result.result_code = "MOTION_ADMISSION_CHANGED"
                return result
            self._rotate_handle, self._rotate_future = handle, future
            timeout = max(float(request.timeout_s),
                          abs(request.target_yaw_delta_deg) * ROTATE_TIMEOUT_S_PER_DEG,
                          float(self._p("rotate_min_timeout_s")))
            self._active = MeasuredMotion(kind="rotate", target=math.radians(request.target_yaw_delta_deg),
                sample=self._odom, now=time.monotonic(), timeout=timeout, limits=self._limits,
                angular_limit=math.radians(request.max_yaw_rate_deg_s), tolerance=math.radians(request.tolerance_deg))
            self._translation_reference = None
        output = await future
        result = Rotate.Result()
        result.success = output.terminal == "OK"
        result.result_code = "SUCCESS" if result.success else output.terminal
        result.final_yaw_delta_deg = math.degrees(output.progress)
        result.message = "measured odometry terminal: " + output.terminal
        if handle.is_cancel_requested:
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        return result

    def _tick(self):
        with self._lock:
            now = time.monotonic()
            if self._motion_enabled:
                error = self._admission_error(now)
                if error:
                    planners = ('NODE_UNAVAILABLE:'+str(self._p('planner_status_node_name')),
                                'NODE_UNAVAILABLE:pixnav_runtime_node')
                    body_active = self._body_command is not None or self._body_held or self._body_returning
                    if error in planners and body_active and self._active is None:
                        if not self._body_recovery_reason:
                            self._body_recovery_reason = error
                            # Preserve the active action deadline. A newly
                            # requested return gets the usual bounded return
                            # budget exactly once in _tick_body.
                            self.get_logger().warning('body_recovery_before_inhibit '+error)
                    else:
                        self._fault(error)
                        return
            self._tick_body(now)
            # Return completion can construct a new macro after a socket poll.
            # Never step that macro with the timer's earlier timestamp.
            now = time.monotonic()
            active = self._active
            if active is not None:
                if self._rotate_handle and self._rotate_handle.is_cancel_requested:
                    self._finish(active.finish("CANCELED"))
                    return
                output = active.step(self._odom, now=now, permitted=self._motion_enabled)
                if output.terminal:
                    self._finish(output)
                else:
                    try:
                        if output.linear == 0 and output.angular == 0:
                            if self._stop():
                                self._fault("SPORT_STOP_FAILED")
                                return
                        else:
                            self._client.move(output.linear, output.angular)
                        if active.kind == "translate":
                            self.get_logger().info("robot_motion_event " + json.dumps({
                                "event": "translate_command", "episode_id": self._episode_id,
                                "command_id": self._command.command_id if self._command else "",
                                "target": active.target, "measured_progress": active.progress,
                                "elapsed_s": now-active.started,
                                "reference_yaw_rad": active.heading_yaw,
                                "measured_yaw_rad": self._odom.yaw,
                                "heading_error_rad": wrap(active.heading_yaw-self._odom.yaw),
                                "linear_command_mps": output.linear,
                                "angular_command_radps": output.angular,
                                "measured_yaw_rate_radps": self._odom.yaw_rate,
                                "transport_accepted": True,
                                "odom_stamp_ns": self._odom.stamp_ns,
                            }, separators=(",", ":")))
                        if active.kind == "rotate":
                            self.get_logger().info("robot_motion_event " + json.dumps({
                                "event": "rotate_command", "episode_id": self._episode_id,
                                "target": active.target, "measured_progress": active.progress,
                                "elapsed_s": now-active.started,
                                "tracking_budget_s": active.tracking_deadline-active.started,
                                "total_budget_s": active.deadline-active.started,
                                "angular_command_radps": output.angular,
                                "phase": "settling" if active.rotation_settling else "tracking",
                                "measured_yaw_rate_radps": self._odom.yaw_rate,
                                "odom_stamp_ns": self._odom.stamp_ns,
                            }, separators=(",", ":")))
                        if self._rotate_handle:
                            feedback = Rotate.Feedback()
                            feedback.current_yaw_delta_deg = math.degrees(output.progress)
                            feedback.remaining_deg = math.degrees(active.target-output.progress)
                            feedback.controller_state = "MEASURED_ROTATION"
                            self._rotate_handle.publish_feedback(feedback)
                    except SportClientError as error:
                        self._fault("COMMAND_TRANSPORT_FAILED")
                        self.get_logger().error(str(error))
            self._flush_result()
            self._pause_if_ready()

    def _stop(self):
        try:
            self._client.stop()
            return ""
        except SportClientError:
            return "SPORT_STOP_FAILED"

    def _finish(self, output):
        if output.terminal in {"TASK_REPLACED", "SHUTDOWN", "MOTION_INHIBITED"}:
            self._abort_body(output.terminal)
        active, command = self._active, self._command
        future = self._rotate_future
        self._active = self._command = self._rotate_handle = self._rotate_future = None
        self._rotate_reserved = False
        if active is None:
            return
        error = self._stop()
        output = replace(output, progress=active.progress)
        self.get_logger().info("robot_motion_event " + json.dumps({
            "event": "terminal", "episode_id": self._episode_id,
            "command_id": command.command_id if command else "rotate",
            "kind": active.kind, "target": active.target, "measured_progress": active.progress,
            "elapsed_s": time.monotonic()-active.started, "status": error or output.terminal,
            "collision_feedback_available": False, "odom_stamp_ns": self._odom.stamp_ns if self._odom else 0,
        }, separators=(",", ":")))
        if error:
            output = MotionOutput(terminal=error, progress=output.progress)
        if output.terminal not in {"OK", "STOPPED", "CANCELED", "TASK_REPLACED"}:
            self._motion_enabled = False
            self._last_fault = output.terminal
            self._translation_reference = None
        if command:
            if output.terminal == "OK":
                self._pending_result = (command, output, self._odom.stamp_ns, time.monotonic()+1.0)
                self._flush_result()
            else:
                self._publish_result(command, executed=False, status=output.terminal)
        if future and not future.done():
            future.set_result(output)

    def _fault(self, reason):
        self._abort_body(reason)
        was_enabled = self._motion_enabled
        self._motion_enabled = False
        self._translation_reference = None
        self._last_fault = reason
        self._finish(MotionOutput(terminal=reason, progress=self._active.progress if self._active else 0))
        self._flush_result(failure=reason)
        if was_enabled:
            self._stop()
            self.get_logger().error("Motion disabled: " + reason)
            if self._sweep:
                self._ack(SweepAck.FAILED, reason)

    def _set_motion_enabled(self, request, response):
        with self._lock:
            if request.data:
                error = ('BODY_RECOVERY_IN_PROGRESS' if self._body_recovery_reason
                         else self._admission_error(time.monotonic()))
                if not error:
                    try:
                        self._client.ping()
                    except SportClientError:
                        error = "COMMAND_TRANSPORT_UNAVAILABLE"
                response.success = not error
                response.message = error or "Motion enabled"
                if not error:
                    if not self._motion_enabled:
                        self._translation_reference = None
                    self._enabled_after = self.get_clock().now().nanoseconds
                    self._motion_enabled = True
                    self._last_fault = ""
            else:
                self._motion_enabled = False
                self._translation_reference = None
                self._finish(MotionOutput(terminal="MOTION_INHIBITED"))
                self._flush_result(failure="MOTION_INHIBITED")
                error = self._stop()
                response.success = not error
                response.message = error or "Motion inhibited and StopMove published"
            return response

    def _flush_result(self, failure=""):
        pending = self._pending_result
        if pending is None:
            return
        command, output, completed_stamp, deadline = pending
        # Causal result pose is from a TF-resolved post-action sample. Older
        # poses must not falsely certify progress or seed asynchronous warping.
        ready = (self._last_pose is not None
                 and stamp_ns(self._last_pose.header.stamp) >= completed_stamp
                 and self._odom is not None and self._odom.stamp_ns >= completed_stamp
                 and abs(self._odom.speed) <= self._limits.stop_speed
                 and abs(self._odom.yaw_rate) <= self._limits.stop_yaw_rate)
        if not failure and not ready and time.monotonic() < deadline:
            return
        self._pending_result = None
        if not failure and not ready:
            failure = "POST_ACTION_POSE_OR_SETTLE_TIMEOUT"
        if failure:
            self._motion_enabled = False
            self._last_fault = failure
            self._translation_reference = None
        self._publish_result(command, executed=not failure, status=failure or output.terminal)

    def _publish_result(self, command, *, executed, status):
        result = HabitatActionResult()
        result.header.stamp = self.get_clock().now().to_msg()
        result.header.frame_id = "robot_origin"
        result.episode_id, result.command_id = command.episode_id, command.command_id
        result.action_result_id = "result-" + command.command_id
        result.trajectory_id, result.generation = command.trajectory_id, command.generation
        self._step_index += 1
        result.habitat_step_index, result.action = self._step_index, command.action
        if self._last_pose:
            # The common action-result contract uses one measurement stamp for
            # both the result and its post-action pose. Preserve the causal pose
            # time instead of tagging it with the later publication time.
            result.header = self._last_pose.header
            result.pose_after.header = self._last_pose.header
            result.pose_after.pose = self._last_pose.pose
        else:
            result.pose_after.header = result.header
        result.executed = executed
        # This robot adapter has no physical collision feedback source. False
        # means no collision was reported, not a claim of contact-free motion.
        result.collided = False
        # Controller terminal reasons are not the shared wire-level status enum.
        # Failed attempts retain the measured controller terminal reason.
        result.status = "EXECUTED" if executed else "FAILED"
        result.error_code = "" if executed else status
        self._results[(command.episode_id, command.command_id)] = result
        while len(self._results) > 256:
            self._results.popitem(last=False)
        self._result_publisher.publish(result)

    @staticmethod
    def _sweep_identity(message):
        return (message.episode_id, message.sweep_id, message.source_decision_id,
                message.trajectory_id, message.generation, message.command_sequence)

    def _on_sweep_ack(self, message):
        with self._lock:
            if message.ack != SweepAck.EXECUTOR_PAUSED or message.producer != "pixnav_runtime_node" or message.episode_id != self._episode_id:
                return
            identity = self._sweep_identity(message)
            if self._sweep and identity == self._sweep_identity(self._sweep):
                self._executor_paused = True
            else:
                self._early_pause_acks.append(identity)
            self._pause_if_ready()

    def _on_sweep_command(self, message):
        with self._lock:
            if message.episode_id != self._episode_id or message.navigation_backend != "pixnav":
                return
            previous = self._sweep
            if previous and (message.sweep_id != previous.sweep_id or message.command_sequence <= previous.command_sequence):
                return
            kind = message.command
            if kind == SweepCommand.REQUEST_PAUSE:
                if previous:
                    return
                self._sweep = message
                self._executor_paused = self._sweep_identity(message) in self._early_pause_acks
                self._quiescent_acked = False
                self._pause_if_ready()
            elif previous is not None:
                allowed = {
                    SweepCommand.START_SWEEP: SweepCommand.REQUEST_PAUSE,
                    SweepCommand.RESUME: SweepCommand.START_SWEEP,
                    SweepCommand.APPLY_RESUME: SweepCommand.RESUME,
                }
                if kind != SweepCommand.CANCEL and allowed.get(kind) != previous.command:
                    return
                if kind == SweepCommand.START_SWEEP and not self._quiescent_acked:
                    return
                self._sweep = message
                if kind == SweepCommand.RESUME:
                    if (self._body_command or self._body_held or self._body_returning or
                            self._active or self._pending_result or self._rotate_reserved or self._stop()):
                        self._ack(SweepAck.FAILED, "CONTROLLER_NOT_STOPPED")
                    else:
                        self._ack(SweepAck.CONTROLLER_RELEASED)
                elif kind == SweepCommand.APPLY_RESUME:
                    self._sweep = None
                elif kind == SweepCommand.CANCEL:
                    # A failed Rotate result cancels its containing sweep. Keep
                    # that original fault visible instead of replacing it with
                    # the cancellation that followed it.
                    self._fault(self._last_fault or "SWEEP_CANCELLED")
                    self._sweep = None

    def _pause_if_ready(self):
        if not self._sweep or self._sweep.command != SweepCommand.REQUEST_PAUSE or not self._executor_paused or self._quiescent_acked:
            return
        if self._body_held and not self._body_returning:
            self._begin_body_return(None)
        if self._body_command is not None or self._body_returning:
            return
        if self._active or self._pending_result or self._rotate_reserved or self._odom is None:
            return
        if abs(self._odom.speed) > self._limits.stop_speed or abs(self._odom.yaw_rate) > self._limits.stop_yaw_rate:
            return
        error = self._stop()
        if error:
            self._fault(error)
            return
        self._quiescent_acked = True
        self._ack(SweepAck.CONTROLLER_QUIESCENT)

    def _ack(self, kind, detail=""):
        command = self._sweep
        if command is None:
            return
        ack = SweepAck()
        ack.header.stamp = self.get_clock().now().to_msg()
        for name in ("episode_id", "sweep_id", "source_decision_id", "trajectory_id", "generation", "command_sequence"):
            setattr(ack, name, getattr(command, name))
        # This node implements the controller role in the shared sweep protocol.
        ack.producer, ack.ack, ack.detail = "controller_node", kind, detail
        if self._last_pose:
            ack.anchor_pose.header, ack.anchor_pose.pose = self._last_pose.header, self._last_pose.pose
        self._ack_publisher.publish(ack)

    def _publish_status(self):
        status = NodeStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.node_name = "go2_action_executor"
        status.state = "ACTIVE" if self._motion_enabled else "INHIBITED"
        status.active_mode = ("BODY_RECOVERY" if self._body_recovery_reason else "BODY_LOOK" if self._body_command or self._body_held or self._body_returning
                              else "MOVING" if self._active else "IDLE")
        status.is_healthy = not bool(self._last_fault)
        status.is_motion_critical = True
        status.error_code = self._last_fault
        self._status_publisher.publish(status)

    def destroy_node(self):
        with self._lock:
            was_enabled = self._motion_enabled
            self._motion_enabled = False
            self._finish(MotionOutput(terminal="SHUTDOWN"))
            self._flush_result(failure="SHUTDOWN")
            if was_enabled:
                self._stop()
        self._rotate_server.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutorNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
