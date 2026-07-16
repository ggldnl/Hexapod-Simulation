"""
Viser interface: apply the firmware's servo commands to the URDF joints.

Viser is a pure visualizer (no physics), so this just poses the model. The
firmware emits calibrated servo-space degrees; the URDF's revolute joints are
named leg_{1..6}_{coxa,femur,tibia}, and cfg::servo_channel(leg, joint) is
leg*3 + joint, so we map each servo channel to its joint by name and push the
radians straight in.
"""
from __future__ import annotations

import math
from typing import List

_JOINTS = ("coxa", "femur", "tibia")


class ViserInterface:
    """Poses a ViserUrdf from the firmware's 18 servo angles."""

    def __init__(self, urdf) -> None:
        self.urdf = urdf
        # Names we expect to drive; warn early if the URDF disagrees.
        expected = {f"leg_{leg + 1}_{j}" for leg in range(6) for j in _JOINTS}
        actuated = set(urdf.get_actuated_joint_names())
        missing = expected - actuated
        if missing:
            raise RuntimeError(f"URDF is missing expected joints: {sorted(missing)}")

    def apply(self, servo_deg: List[float], powered: bool = True) -> None:
        """Pose the model to the commanded servo angles. `powered` is accepted
        for parity with the PyBullet interface but ignored (no dynamics here)."""
        cfg = {
            f"leg_{leg + 1}_{j}": math.radians(servo_deg[leg * 3 + i])
            for leg in range(6) for i, j in enumerate(_JOINTS)
        }
        self.urdf.update_cfg(cfg)

    # No dynamics, so teleport and apply are the same thing.
    teleport = apply
