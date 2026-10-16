"""
Measurements for the PyBullet front-ends: how fast the robot really walks, and
how hard the firmware drives the joints to get there.

The firmware's odometry integrates the twist the gait executes, i.e. what the
robot would do if its feet never slipped. PyBullet knows where the body actually
is, so the gap between the two speeds is slip. Both are averaged over one gait
cycle (the body surges and stalls within a cycle), and the best sustained value
is kept as the top speed.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Mapping, Optional, Sequence

# Reference: tripod at full command on the reconstructed Chica server
# (EternalNitrous/chica-server), MODE_STANDARD and MODE_RACE (speed factor 2)
CHICA_STANDARD = 240.0  # mm/s
CHICA_RACE = 480.0      # mm/s

# Datasheet no-load speed of an MG996R at 6 V (0.14 s / 60 deg)
MG996R_DEG_S = 430.0


def gait_limits(config: Mapping, gait: str) -> dict:
    """Top speeds the config allows for a gait: the firmware's command clamp
    (safety.lin_vel_max) and the stride cap max_stride / (duty * cycle_time).
    The lower of the two is the most the gait can be asked for."""
    g = config["gaits"]
    params = g[gait]
    cycle = float(g["cycle_time"])
    stride_cap = params["max_stride"] / (params["duty_factor"] * cycle)
    lin_max = float(config["safety"]["lin_vel_max"])
    return {
        "cycle_time": cycle,
        "duty": float(params["duty_factor"]),
        "max_stride": float(params["max_stride"]),
        "stride_cap": stride_cap,
        "lin_vel_max": lin_max,
        "top": min(lin_max, stride_cap),
    }


class _WindowSpeed:
    """Planar speed = displacement over the last `window` seconds / window."""

    def __init__(self, window: float) -> None:
        self.window = window
        self._buf: deque = deque()  # (t, x_mm, y_mm)

    def push(self, t: float, x: float, y: float) -> Optional[float]:
        """Add a sample; return the speed (mm/s) once a full window is covered."""
        buf = self._buf
        buf.append((t, x, y))
        while len(buf) > 1 and t - buf[1][0] >= self.window:
            buf.popleft()
        t0, x0, y0 = buf[0]
        span = t - t0
        if span < self.window * 0.999:
            return None
        return math.hypot(x - x0, y - y0) / span

    def reset(self) -> None:
        self._buf.clear()


class Speedometer:
    """Real (PyBullet) and gait (odometry) speed of the body, one-cycle averages.

    Straight-line speed: while turning, the chord over a window is a little
    shorter than the arc actually walked."""

    def __init__(self, physics, robot_id: int, window: float) -> None:
        self.p = physics
        self.robot = robot_id
        self.t = 0.0
        self.real = 0.0   # mm/s, ground truth
        self.gait = 0.0   # mm/s, what the firmware executes (odometry)
        self.best = 0.0   # mm/s, best full-window ground-truth speed
        self._real = _WindowSpeed(window)
        self._gait = _WindowSpeed(window)

    def update(self, dt: float, telemetry=None) -> None:
        """Call once per control tick; pass telemetry whenever you have it."""
        self.t += dt
        (x, y, _), _ = self.p.getBasePositionAndOrientation(self.robot)
        v = self._real.push(self.t, x * 1000.0, y * 1000.0)
        if v is not None:
            self.real = v
            self.best = max(self.best, v)
        if telemetry is not None:
            v = self._gait.push(self.t, telemetry.odom_x, telemetry.odom_y)
            if v is not None:
                self.gait = v

    def position_mm(self) -> tuple:
        (x, y, _), _ = self.p.getBasePositionAndOrientation(self.robot)
        return x * 1000.0, y * 1000.0

    def lines(self, commanded: Optional[float] = None) -> list:
        cmd = f", cmd {commanded:.0f}" if commanded is not None else ""
        grip = f"  grip {100 * self.real / self.gait:3.0f}%" if self.gait > 1.0 else ""
        return [
            f"speed {self.real:4.0f} mm/s   best {self.best:4.0f} mm/s",
            f"gait {self.gait:4.0f} mm/s{cmd}{grip}",
            f"Chica: {CHICA_STANDARD:.0f} standard / {CHICA_RACE:.0f} race",
        ]


class JointRate:
    """Peak commanded joint speed (deg/s) across the 18 servos, the same yardstick
    the gait comparison uses: what the firmware asks of the servos per tick."""

    def __init__(self) -> None:
        self._prev: Optional[Sequence[float]] = None
        self.peak = 0.0

    def update(self, servo_deg: Sequence[float], dt: float) -> None:
        if self._prev is not None:
            rate = max(abs(a - b) for a, b in zip(servo_deg, self._prev)) / dt
            self.peak = max(self.peak, rate)
        self._prev = list(servo_deg)

    def restart(self) -> None:
        """Start a new peak (keeps the previous sample, so no spurious jump)."""
        self.peak = 0.0


class Hud:
    """Lines of debug text floating above the robot, following it."""

    def __init__(self, physics, robot_id: int, size: float = 1.1) -> None:
        self.p = physics
        self.robot = robot_id
        self.size = size
        self._ids: list = []

    def show(self, lines: Sequence[str]) -> None:
        (x, y, z), _ = self.p.getBasePositionAndOrientation(self.robot)
        for i, line in enumerate(lines):
            pos = [x, y, z + 0.22 - 0.03 * i]
            if i < len(self._ids):
                self._ids[i] = self.p.addUserDebugText(
                    line, pos, textColorRGB=[1, 1, 1], textSize=self.size,
                    replaceItemUniqueId=self._ids[i])
            else:
                self._ids.append(self.p.addUserDebugText(
                    line, pos, textColorRGB=[1, 1, 1], textSize=self.size))


def follow_camera(physics, robot_id: int) -> None:
    """Keep the camera on the robot as it walks off, preserving whatever
    distance/yaw/pitch was set with the mouse."""
    cam = physics.getDebugVisualizerCamera()
    yaw, pitch, dist = cam[8], cam[9], cam[10]
    (x, y, _), _ = physics.getBasePositionAndOrientation(robot_id)
    physics.resetDebugVisualizerCamera(dist, yaw, pitch, [x, y, 0.0])
