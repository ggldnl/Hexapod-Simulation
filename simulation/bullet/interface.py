"""
PyBullet interface: apply the firmware's servo commands to the URDF joints.

The firmware already emits calibrated servo-space degrees, and the URDF's 18
revolute joints are in leg-major order (leg*3 + joint), the same order as
cfg::servo_channel, so we apply radians(servo_deg) straight to the joints 
with no extra mapping.
"""
from __future__ import annotations

import math
from typing import List


class BulletInterface:
    # MG996R: 11 kgf.cm @ 6V ~= 1.08 N.m.
    def __init__(self, physics, robot_id, force: float = 1.08) -> None:
        self.p = physics
        self.robot = robot_id
        self.force = force

        revolute = [j for j in range(physics.getNumJoints(robot_id))
                    if physics.getJointInfo(robot_id, j)[2] == physics.JOINT_REVOLUTE]
        if len(revolute) != 18:
            raise RuntimeError(f"expected 18 revolute joints, found {len(revolute)}")
        # index i == cfg::servo_channel(leg, joint) == leg*3 + joint
        self.joints: List[int] = revolute

    def apply(self, servo_deg: List[float], powered: bool = True) -> None:
        """Drive the joints toward the commanded servo angles (position control).
        When de-energized (OFF/FAULT), let the legs go limp."""
        for i, joint in enumerate(self.joints):
            self.p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=joint,
                controlMode=self.p.POSITION_CONTROL,
                targetPosition=math.radians(servo_deg[i]),
                force=self.force if powered else 0.0,
            )

    def teleport(self, servo_deg: List[float]) -> None:
        """Hard-set the joints (no dynamics), handy to seed a starting pose."""
        for i, joint in enumerate(self.joints):
            self.p.resetJointState(self.robot, joint, math.radians(servo_deg[i]))
