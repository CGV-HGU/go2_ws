"""Measured macro execution. No ROS, sockets, clocks, or actuator side effects."""

from dataclasses import dataclass
import math


ROTATE_TIMEOUT_S_PER_DEG = 15.0 / 60.0


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class MotionSample:
    stamp_ns: int
    received_s: float
    x: float
    y: float
    yaw: float
    speed: float
    yaw_rate: float
    frame: str = "odom"

    def __post_init__(self):
        if self.stamp_ns <= 0 or not self.frame or not all(math.isfinite(v) for v in (
            self.received_s, self.x, self.y, self.yaw, self.speed, self.yaw_rate
        )):
            raise ValueError("INVALID_ODOMETRY")


@dataclass(frozen=True)
class MotionLimits:
    linear_speed: float = 0.50
    angular_speed: float = 0.45
    minimum_turn_speed: float = 0.40
    turn_gain: float = 1.2
    acceleration: float = 1.0
    odom_timeout: float = 0.5
    distance_tolerance: float = 0.015
    angle_tolerance: float = math.radians(3.0)
    settle_samples: int = 2
    rotation_settle_timeout: float = 2.0
    rotation_settle_window: float = 0.0
    rotation_recoil_limit: float = 0.0  # Zero retains the original tolerance-sized cap.
    stop_speed: float = 0.06
    stop_yaw_rate: float = 0.10
    stall_timeout: float = 1.5

    def __post_init__(self):
        for name, value in vars(self).items():
            invalid = value < 0 if name in {"rotation_settle_window", "rotation_recoil_limit"} else value <= 0
            if not math.isfinite(value) or invalid:
                raise ValueError("INVALID_MOTION_LIMIT:" + name)
        if self.rotation_recoil_limit > math.radians(10):
            raise ValueError("rotation recoil limit exceeds 10 degrees")
        if self.minimum_turn_speed > self.angular_speed:
            raise ValueError("minimum turn speed exceeds maximum")
        if not isinstance(self.settle_samples, int):
            raise ValueError("settle_samples must be an integer")


@dataclass(frozen=True)
class MotionOutput:
    linear: float = 0.0
    angular: float = 0.0
    terminal: str = ""
    progress: float = 0.0


class MeasuredMotion:
    """One displacement/rotation closes only on distinct post-command odometry."""

    def __init__(self, *, kind: str, target: float, sample: MotionSample,
                 now: float, timeout: float, limits: MotionLimits,
                 angular_limit: float | None = None, tolerance: float | None = None,
                 heading_yaw: float | None = None):
        if kind not in {"translate", "rotate"} or not math.isfinite(target) or target == 0:
            raise ValueError("INVALID_MOTION_TARGET")
        if not math.isfinite(timeout) or timeout <= 0 or now < sample.received_s:
            raise ValueError("INVALID_MOTION_TIMING")
        if angular_limit is not None and (not math.isfinite(angular_limit) or angular_limit <= 0):
            raise ValueError("INVALID_ANGULAR_LIMIT")
        if heading_yaw is not None and (kind != "translate" or not math.isfinite(heading_yaw)):
            raise ValueError("INVALID_TRANSLATION_HEADING")
        self.kind, self.target, self.start = kind, target, sample
        # A run of translations may retain its intended bearing across macro
        # boundaries. Displacement and deviation remain measured per macro.
        self.heading_yaw = sample.yaw if heading_yaw is None else wrap(heading_yaw)
        self.limits = limits
        self.angular_limit = min(angular_limit or limits.angular_speed, limits.angular_speed)
        self.tolerance = tolerance or (limits.distance_tolerance if kind == "translate" else limits.angle_tolerance)
        if not math.isfinite(self.tolerance) or self.tolerance <= 0:
            raise ValueError("INVALID_TOLERANCE")
        self.started = self.last_tick = self.last_progress_at = now
        self.tracking_deadline = now + timeout
        self.deadline = self.tracking_deadline + (
            limits.rotation_settle_timeout if kind == "rotate" else 0.0
        )
        self.last_sample = sample
        self.yaw_progress = self.progress = self.progress_mark = 0.0
        self.settled = 0
        self.rotation_settling = False
        self._rotation_stop_progress = 0.0
        self._rotation_brake_offset = 0.0
        self._rotation_rest_window = []
        self.translation_settling = False
        self._last_rotation_error = target
        self.linear = 0.0
        self.finished: MotionOutput | None = None

    def finish(self, reason: str) -> MotionOutput:
        if self.finished is None:
            self.finished = MotionOutput(terminal=reason, progress=self.progress)
        return self.finished

    def step(self, sample: MotionSample | None, *, now: float,
             permitted: bool = True) -> MotionOutput:
        if self.finished:
            return self.finished
        if not permitted:
            return self.finish("MOTION_INHIBITED")
        if sample is None or now - sample.received_s > self.limits.odom_timeout:
            return self.finish("ODOM_STALE")
        if now < self.last_tick or sample.received_s > now or sample.frame != self.start.frame:
            return self.finish("ODOM_RESET")
        if sample.stamp_ns < self.last_sample.stamp_ns:
            return self.finish("ODOM_RESET")
        fresh = sample.stamp_ns > self.last_sample.stamp_ns
        if fresh:
            elapsed = (sample.stamp_ns - self.last_sample.stamp_ns) * 1e-9
            translation = math.hypot(sample.x - self.last_sample.x, sample.y - self.last_sample.y)
            yaw_delta = wrap(sample.yaw - self.last_sample.yaw)
            if translation > 0.10 + 1.5 * elapsed or abs(yaw_delta) > 0.15 + 2.0 * elapsed:
                return self.finish("ODOM_JUMP")
            self.yaw_progress += yaw_delta
            self.last_sample = sample
        dx, dy = sample.x - self.start.x, sample.y - self.start.y
        self.progress = (dx * math.cos(self.start.yaw) + dy * math.sin(self.start.yaw)
                         if self.kind == "translate" else self.yaw_progress)
        if now >= self.deadline:
            return self.finish("MOTION_TIMEOUT")
        dt = min(max(now - self.last_tick, 0), 0.1)
        self.last_tick = now
        if self.kind == "translate":
            cross_track = -dx * math.sin(self.start.yaw) + dy * math.cos(self.start.yaw)
            if abs(cross_track) > 0.12 or abs(wrap(sample.yaw - self.start.yaw)) > math.radians(25):
                return self.finish("PATH_DEVIATION")
            remaining = abs(self.target) - math.copysign(1, self.target) * self.progress
            if remaining < -0.07:
                return self.finish("DISTANCE_OVERSHOOT")
            at_target = remaining <= self.tolerance
        else:
            remaining = self.target - self.progress
            at_target = abs(remaining) <= self.tolerance
        stationary = abs(sample.speed) <= self.limits.stop_speed and abs(sample.yaw_rate) <= self.limits.stop_yaw_rate
        if self.kind == "rotate":
            tracking_expired = now >= self.tracking_deadline
            if tracking_expired and not (at_target or self.rotation_settling):
                return self.finish("MOTION_TIMEOUT")
            brake_remaining = remaining + self._rotation_brake_offset
            crossed_target = brake_remaining * self._last_rotation_error < 0
            self._last_rotation_error = brake_remaining
            # Aim inside the requested tolerance before stopping, leaving room
            # for body settling. On overshoot, stop before reversing direction.
            if not self.rotation_settling and (
                abs(brake_remaining) <= self.tolerance / 2 or crossed_target
                or (tracking_expired and at_target)
            ):
                self.rotation_settling = True
                self._rotation_stop_progress = self.progress
                self._rotation_rest_window.clear()
                self.settled = 0
            if self.rotation_settling:
                if fresh:
                    self.settled = self.settled + 1 if stationary else 0
                    if stationary:
                        if (self.limits.rotation_settle_window > 0 and self._rotation_rest_window
                                and now-self._rotation_rest_window[-1][0] > .1):
                            self._rotation_rest_window.clear()
                        self._rotation_rest_window.append((now, self.progress))
                        cutoff = now-self.limits.rotation_settle_window
                        while len(self._rotation_rest_window) > 2 and self._rotation_rest_window[1][0] <= cutoff:
                            self._rotation_rest_window.pop(0)
                    else:
                        self._rotation_rest_window.clear()
                window = self._rotation_rest_window
                stable_window = self.limits.rotation_settle_window == 0 or (
                    len(window) >= 2
                    and window[-1][0]-window[0][0] + 1e-9 >= self.limits.rotation_settle_window
                    and max(p for _, p in window)-min(p for _, p in window)
                        <= min(self.tolerance/2, math.radians(.25))
                )
                if self.settled < self.limits.settle_samples or not stable_window:
                    # A noisy angle crossing must not restart the gait while
                    # the preceding StopMove is still bringing the body to rest.
                    return MotionOutput(progress=self.progress)
                # Leave part of the acceptance band for residual settling after
                # the ACK. At 0.5 rad/s the recorded 27.15-degree right turn
                # recoiled to 26.92 after a 30-degree / 3-degree request.
                # This is an internal completion margin, not a wider goal band.
                completion_margin = (
                    min(self.tolerance / 4, math.radians(.5))
                    if self.limits.rotation_settle_window > 0 else 0.0
                )
                if abs(remaining) <= self.tolerance - completion_margin:
                    return self.finish("OK")
                if tracking_expired:
                    # The extra budget is only for stopping and verification;
                    # it never authorizes another corrective turn.
                    return self.finish("MOTION_TIMEOUT")
                # Correct only the error measured after the robot has stopped.
                # Tight centre alignment can repeatedly brake at the same
                # angle and recoil outside its band. Compensate the measured
                # recoil on the next correction. A configured field limit is
                # separate from the acceptance band and at most half this turn.
                # Acceptance always uses the original target and tolerance.
                recoil = self._rotation_stop_progress - self.progress
                recoil_cap = (min(self.limits.rotation_recoil_limit, abs(self.target)/2)
                              if self.limits.rotation_recoil_limit else self.tolerance)
                self._rotation_brake_offset = (
                    math.copysign(min(abs(recoil), recoil_cap), remaining)
                    if ((self.tolerance < self.limits.angle_tolerance or self.limits.rotation_settle_window > 0)
                        and (self.limits.rotation_recoil_limit > 0 or remaining * self.target > 0)
                        and recoil * remaining > 0)
                    else 0.0
                )
                self._last_rotation_error = remaining + self._rotation_brake_offset
                self.rotation_settling = False
                self._rotation_rest_window.clear()
                self.settled = 0
                self.progress_mark, self.last_progress_at = self.progress, now
        else:
            # Brake inside the acceptance band, leaving room for body recoil.
            # Once StopMove starts, do not restart the gait on a noisy crossing
            # of the distance boundary before fresh odometry confirms rest.
            if not self.translation_settling and remaining <= self.tolerance / 2:
                self.translation_settling = True
                self.settled = 0
            if fresh:
                self.settled = self.settled + 1 if stationary and (
                    at_target or self.translation_settling
                ) else 0
            if self.settled >= self.limits.settle_samples:
                if at_target:
                    return self.finish("OK")
                self.translation_settling = False
                self.settled = 0
                self.progress_mark, self.last_progress_at = self.progress, now
            if self.translation_settling:
                self.linear = 0.0
                return MotionOutput(progress=self.progress)
        if abs(self.progress - self.progress_mark) > (0.008 if self.kind == "translate" else 0.015):
            self.progress_mark, self.last_progress_at = self.progress, now
        if not at_target and now - self.last_progress_at > self.limits.stall_timeout:
            return self.finish("NO_MOTION_PROGRESS")
        if self.kind == "translate":
            # Brake according to remaining displacement; never infer distance from elapsed time.
            desired = min(self.limits.linear_speed, math.sqrt(2 * self.limits.acceleration * max(remaining - self.tolerance / 2, 0)))
            self.linear = min(desired, self.linear + self.limits.acceleration * dt)
            yaw_error = wrap(self.heading_yaw - sample.yaw)
            return MotionOutput(math.copysign(self.linear, self.target), max(-0.25, min(0.25, yaw_error * 1.2)), progress=self.progress)
        # Go2 stopped progressing as commands fell to ~0.274 rad/s in the
        # 20260907-165034 run. Keep tracking commands in the previously working
        # range; the latched settling phase above controls the final stop.
        # A caller's lower speed ceiling still takes precedence over this floor.
        brake_remaining = remaining + self._rotation_brake_offset
        rate = min(self.angular_limit, max(self.limits.minimum_turn_speed,
                                          abs(brake_remaining) * self.limits.turn_gain))
        return MotionOutput(angular=math.copysign(rate, brake_remaining), progress=self.progress)
