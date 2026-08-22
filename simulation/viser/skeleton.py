"""
Kinematic-skeleton overlay for the Viser view.

What the browser draws is the CAD model; what the firmware walks on is a much
simpler thing: three link lengths, six mount poses and a stance, all typed by
hand into config.yml. Nothing checks that the two agree. This module draws that
kinematic model over the mesh -- the leg chains, the ground plane the firmware
believes it stands on, and where the feet touch it -- and reports the stance
numbers, so a wrong link length or mount shows up as a skeleton drifting off the
mesh instead of as a robot that limps on the real hardware.

Frames
    The kinematic model is body-centred, in mm: +x forward, +y left, and z = 0
    is the plane the mounts measure from -- the coxa-femur joints, on this robot
    -- which stands `standing_height` above the ground. The
    URDF's base frame is that same frame rotated +90 deg about z (its +y is
    forward), in metres, so `to_scene()` is the whole conversion. `z_offset`
    slides the kinematic origin along the URDF's z; where it belongs is the
    plane of the coxa-femur joints, which the CAD's base origin does not sit on
    (see `UrdfGeometry.femur_plane`).

Ground
    Viser has no physics and never moves the body, so here the body pose moves
    the GROUND instead: standing, the ground plane sits `standing_height` below
    the model's origin, and it rises to meet the body as the robot sits down --
    until the chassis is on it, which is as far as a real robot gets (see
    `measure`). Foot targets are world-frame in the firmware, so contacts are
    computed there and mapped back into the body frame for drawing.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Optional, Sequence, Union

import numpy as np
import yaml

from hexapod.client import DEFAULT_CONFIG, JOINTS, LEGS

N_LEGS = len(LEGS)
N_JOINTS = len(JOINTS)

MM = 1e-3          # the scene is in metres, the model in mm
CONTACT_EPS = 2.0  # mm, a foot within this of the ground counts as down

# Short tags for the readout: FR, MR, RR, RL, ML, FL
TAGS = tuple("".join(w[0] for w in leg.split("_")).upper() for leg in LEGS)

ConfigLike = Union[Mapping, str, Path, None]

COLOR = {
    "link": (255, 165, 0),
    "joint": (255, 110, 40),
    "body": (90, 160, 255),
    "foot": (255, 60, 60),
    "drop": (150, 150, 150),
    "contact": (0, 220, 130),
    "support": (0, 220, 130),
    "center": (255, 230, 0),
    "target": (120, 120, 255),
    "ground": (120, 120, 120),
}


# Frames

def to_scene(pts_mm, z_offset: float = 0.0) -> np.ndarray:
    """Kinematic body frame (mm) -> Viser/URDF frame (m), for any array shaped
    (..., 3). A +90 deg turn about z (the URDF's +y is forward), then the shift."""
    p = np.asarray(pts_mm, dtype=float)
    out = np.empty_like(p)
    out[..., 0] = -p[..., 1]
    out[..., 1] = p[..., 0]
    out[..., 2] = p[..., 2] + z_offset
    return out * MM


def from_scene(pts_m, z_offset: float = 0.0) -> np.ndarray:
    """Viser/URDF frame (m) -> kinematic body frame (mm). Inverse of to_scene."""
    p = np.asarray(pts_m, dtype=float) / MM
    out = np.empty_like(p)
    out[..., 0] = p[..., 1]
    out[..., 1] = -p[..., 0]
    out[..., 2] = p[..., 2] - z_offset
    return out


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) Ry(pitch) Rx(roll) from degrees; the firmware's math::rpy_matrix."""
    r, p, y = np.radians([roll, pitch, yaw])
    cr, sr, cp, sp, cy, sy = (np.cos(r), np.sin(r), np.cos(p),
                              np.sin(p), np.cos(y), np.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])


# The model as configured

def load_config(config: ConfigLike = None) -> Mapping:
    """The config as a mapping: passed through, read from a path, or the one
    packaged with the client (the same file the board is provisioned from)."""
    if isinstance(config, Mapping):
        return config
    with open(Path(config) if config is not None else DEFAULT_CONFIG) as f:
        return yaml.safe_load(f)


@dataclass(frozen=True)
class KinematicConfig:
    """The kinematic half of config.yml, as arrays in canonical leg order."""

    coxa: float             # mm, radial: coxa axis -> coxa-femur joint
    coxa_offset: float      # mm, how far that joint sits to the side (+ left)
    femur: float            # mm
    tibia: float            # mm
    standing_height: float  # mm, origin plane above the ground when standing
    stance_radius: float    # mm, neutral foot distance from its mount
    mount: np.ndarray       # (6, 3) mm, body frame; z is the coxa-femur joint's
                            # height, 0 when the body origin is on that plane
    mount_yaw: np.ndarray   # (6,) rad
    trim: np.ndarray        # (6, 3) deg
    direction: np.ndarray   # (6, 3), +-1

    @classmethod
    def load(cls, config: ConfigLike = None) -> "KinematicConfig":
        cfg = load_config(config)
        try:
            kin = cfg["kinematics"]
            legs, mounts = kin["legs"], kin["mounts"]
            hw = cfg.get("hardware", {})
            trim = hw.get("trim", {})
            direction = hw.get("direction", {})
            return cls(
                coxa=float(legs["coxa"]),
                coxa_offset=float(legs["coxa_offset"]),
                femur=float(legs["femur"]),
                tibia=float(legs["tibia"]),
                standing_height=float(kin["standing_height"]),
                stance_radius=float(kin["stance_radius"]),
                mount=np.array([mounts[leg]["position"] for leg in LEGS], float),
                mount_yaw=np.radians([mounts[leg]["orientation"][2] for leg in LEGS]),
                trim=np.array([trim.get(leg, (0.0,) * N_JOINTS) for leg in LEGS], float),
                direction=np.array([direction.get(leg, (1.0,) * N_JOINTS)
                                    for leg in LEGS], float),
            )
        except (KeyError, TypeError, ValueError) as e:
            raise ValueError(f"config is missing kinematics the overlay needs: {e}") from e

    def kin_angles(self, servo_deg: Sequence[float]) -> np.ndarray:
        """Servo-space degrees (18, leg-major) -> kinematic radians, (6, 3). The
        inverse of the firmware's servo map, kin = (servo - trim) / direction."""
        servo = np.asarray(servo_deg, float).reshape(N_LEGS, N_JOINTS)
        return np.radians((servo - self.trim) / self.direction)

    def chain(self, servo_deg: Sequence[float]) -> np.ndarray:
        """Body-frame joint positions, (6, 4, 3) mm: coxa axis (the mount), femur
        axis, tibia axis, foot. This is kin::leg_forward kept in pieces instead
        of collapsed onto the foot."""
        coxa, femur, tibia = self.kin_angles(servo_deg).T
        # Walk out the leg's own vertical plane: distance along it and height,
        # one column per joint. Only the coxa axis is on the plane's edge -- the
        # rest of the leg is carried COXA_OFFSET to the side of it
        radial = np.stack([
            np.zeros(N_LEGS),
            np.full(N_LEGS, self.coxa),
            self.coxa + self.femur * np.cos(femur),
            self.coxa + self.femur * np.cos(femur) + self.tibia * np.cos(femur + tibia),
        ], axis=1)
        lateral = np.full_like(radial, self.coxa_offset)
        lateral[:, 0] = 0.0
        height = np.stack([
            np.zeros(N_LEGS),
            np.zeros(N_LEGS),
            self.femur * np.sin(femur),
            self.femur * np.sin(femur) + self.tibia * np.sin(femur + tibia),
        ], axis=1)
        # The coxa yaw stacks on the mount yaw, then the mount translates
        ang = (self.mount_yaw + coxa)[:, None]
        cos, sin = np.cos(ang), np.sin(ang)
        pts = np.stack([radial * cos - lateral * sin,
                        radial * sin + lateral * cos, height], axis=2)
        return pts + self.mount[:, None, :]

    def neutral_feet(self) -> np.ndarray:
        """Where the gait parks the feet, (6, 3) mm in the ground frame: the mount
        pushed out by stance_radius along its yaw. Mirrors gait::compute_neutral."""
        return np.stack([
            self.mount[:, 0] + self.stance_radius * np.cos(self.mount_yaw),
            self.mount[:, 1] + self.stance_radius * np.sin(self.mount_yaw),
            np.zeros(N_LEGS),
        ], axis=1)


# The stance, measured

@dataclass
class Stance:
    """Where the legs actually are, all in the ground (world) frame, mm."""

    body_height: float    # coxa plane above the ground
    foot: np.ndarray      # (6, 3)
    contact: np.ndarray   # (6, 3), each foot dropped onto the ground
    mount: np.ndarray     # (6, 3)
    height: np.ndarray    # (6,) foot above the ground
    radius: np.ndarray    # (6,) mount -> foot, in the ground plane
    grounded: np.ndarray  # (6,) bool, foot within CONTACT_EPS of the ground
    center: np.ndarray    # (3,) the body origin dropped onto the ground
    support: np.ndarray   # (M, 2) the grounded contacts, hulled
    margin: float         # body centre to the support polygon edge, + inside
    commanded: float      # body height the core asked for, before the chassis
    resting: bool         # the chassis is down: body_height is the floor, not
                          # the height the core asked for


def measure(cfg: KinematicConfig, chain: np.ndarray, pose,
            min_height: float = -np.inf) -> Stance:
    """Turn a body-frame chain plus the live body pose into ground-frame numbers.
    `pose` is the client's BodyPose: x/y/z mm (z relative to standing), rpy deg.

    `min_height` is how close the body origin can get to the ground before the
    chassis is resting on it. The core models the body as a point and sits down
    to z = 0, which would bury half the robot; the real chassis stops it, so the
    ground stops there too instead of rising through the meshes."""
    R = rpy_matrix(pose.roll, pose.pitch, pose.yaw)
    height = cfg.standing_height + pose.z
    resting = height < min_height
    t = np.array([pose.x, pose.y, max(height, min_height)])

    foot = chain[:, 3] @ R.T + t
    mount = chain[:, 0] @ R.T + t
    contact = np.column_stack([foot[:, :2], np.zeros(N_LEGS)])
    clearance = foot[:, 2]
    radius = np.linalg.norm(foot[:, :2] - mount[:, :2], axis=1)
    grounded = clearance < CONTACT_EPS
    center = np.array([t[0], t[1], 0.0])
    support = convex_hull(contact[grounded, :2])

    return Stance(body_height=float(t[2]), foot=foot, contact=contact, mount=mount,
                  height=clearance, radius=radius, grounded=grounded, center=center,
                  support=support, margin=polygon_margin(support, center[:2]),
                  commanded=float(height), resting=resting)


def convex_hull(xy: np.ndarray) -> np.ndarray:
    """Counter-clockwise convex hull of a handful of 2D points (monotone chain).
    Fewer than three points come back as they are: there is no polygon to make."""
    pts = np.unique(np.asarray(xy, float).round(6), axis=0)
    if len(pts) < 3:
        return pts

    def turn(o, a, b):  # 2D cross product: > 0 is a left turn
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def half(ordered):
        out: list = []
        for p in ordered:
            while len(out) >= 2 and turn(out[-2], out[-1], p) <= 0:
                out.pop()
            out.append(p)
        return out

    lower, upper = half(pts), half(pts[::-1])
    return np.array(lower[:-1] + upper[:-1])


def polygon_margin(hull: np.ndarray, point: np.ndarray) -> float:
    """Signed distance from `point` to a CCW convex polygon's edge, + inside.
    NaN when there is no polygon (fewer than three feet down)."""
    if len(hull) < 3:
        return float("nan")
    edge = np.roll(hull, -1, axis=0) - hull
    inward = np.column_stack([-edge[:, 1], edge[:, 0]])
    inward /= np.linalg.norm(inward, axis=1, keepdims=True)
    return float(np.min(np.sum((point - hull) * inward, axis=1)))


# Scene overlay

def _segments(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Pair up two (N, 3) point sets into viser's (N, 2, 3) segment array."""
    return np.stack([np.atleast_2d(a), np.atleast_2d(b)], axis=1)


def _loop(points: np.ndarray) -> np.ndarray:
    """Segments closing a polygon through `points` (N, 3)."""
    return _segments(points, np.roll(points, -1, axis=0))


def _grid(half: float, cell: float) -> np.ndarray:
    """A flat square grid centred on the origin, as segments (ground frame)."""
    ticks = np.arange(-half, half + cell, cell)
    z = np.zeros_like(ticks)
    a = np.concatenate([np.column_stack([np.full_like(ticks, -half), ticks, z]),
                        np.column_stack([ticks, np.full_like(ticks, -half), z])])
    b = np.concatenate([np.column_stack([np.full_like(ticks, half), ticks, z]),
                        np.column_stack([ticks, np.full_like(ticks, half), z])])
    return _segments(a, b)


class SkeletonOverlay:
    """The configured kinematic model, drawn over the URDF and kept in sync.

    Two groups, each toggled on its own:
        skeleton  the leg chains, their joint centres and the mount polygon
        stance    every foot dropped onto the ground, the support polygon over
                  the grounded ones, the body's own ground projection, and the
                  neutral foot targets config.yml asks for

    The ground plane is always drawn: it is the reference everything else is
    read against. Scene nodes are made once and their geometry rewritten in
    place; a hidden group is not recomputed.
    """

    def __init__(self, server, cfg: KinematicConfig, *, z_offset: float = 0.0,
                 chassis_floor: Optional[float] = None, root: str = "/kinematics",
                 visible: Optional[Mapping[str, bool]] = None) -> None:
        self.server = server
        self.cfg = cfg
        self.z_offset = float(z_offset)
        # Lowest point of the CAD's chassis, in the URDF's frame: how far the
        # body really extends below the point the core sits down onto
        self.chassis_floor = chassis_floor
        self.stance: Optional[Stance] = None

        self.visible = {"skeleton": True, "stance": True}
        self.visible.update(visible or {})

        seg, pts = np.zeros((1, 2, 3)), np.zeros((1, 3))
        add_seg = server.scene.add_line_segments
        add_pts = server.scene.add_point_cloud

        # skeleton: the chains, their joints, the feet, and the mount polygon
        self.links = add_seg(f"{root}/links", seg, COLOR["link"], line_width=3.0)
        self.joints = add_pts(f"{root}/joints", pts, COLOR["joint"],
                              point_size=0.003, point_shape="circle")
        self.feet = add_pts(f"{root}/feet", pts, COLOR["foot"],
                            point_size=0.004, point_shape="circle")
        self.body = add_seg(f"{root}/body", seg, COLOR["body"], line_width=2.0)

        # stance: the drop under each foot, the contact points, the support
        # polygon, the body centre's projection, and the neutral targets
        self.drops = add_seg(f"{root}/drops", seg, COLOR["drop"], line_width=1.0)
        self.contacts = add_pts(f"{root}/contacts", pts, COLOR["contact"],
                                point_size=0.004, point_shape="circle")
        self.support = add_seg(f"{root}/support", seg, COLOR["support"], line_width=2.0)
        self.center = add_pts(f"{root}/center", pts, COLOR["center"],
                              point_size=0.005, point_shape="diamond")
        self.neutral = add_pts(f"{root}/neutral", pts, COLOR["target"],
                               point_size=0.005, point_shape="diamond")

        # ground: the plane the firmware is standing on
        self.ground = add_seg(f"{root}/ground", seg, COLOR["ground"], line_width=1.0)

        self._groups = {
            "skeleton": (self.links, self.joints, self.feet, self.body),
            "stance": (self.drops, self.contacts, self.support, self.center,
                       self.neutral),
        }
        for group, on in self.visible.items():
            self._show(group, on)

    # Visibility

    def set_visible(self, **groups: bool) -> None:
        """Show or hide whole groups by name; unchanged groups cost nothing."""
        for group, on in groups.items():
            if group not in self._groups:
                raise KeyError(f"unknown overlay group: {group}")
            if self.visible[group] != bool(on):
                self.visible[group] = bool(on)
                self._show(group, on)

    def _show(self, group: str, on: bool) -> None:
        for node in self._groups[group]:
            node.visible = bool(on)

    # Geometry

    def update(self, servo_deg: Sequence[float], pose) -> Stance:
        """Redraw from the firmware's servo angles and its live body pose, and
        return the stance numbers for the readout."""
        cfg = self.cfg
        chain = cfg.chain(servo_deg)
        st = measure(cfg, chain, pose, self.min_height)
        self.stance = st

        # World -> body, so ground-frame geometry can be drawn around the body,
        # which is the only frame Viser ever draws in
        R = rpy_matrix(pose.roll, pose.pitch, pose.yaw)
        t = np.array([pose.x, pose.y, st.body_height])

        def to_body(world):
            return (np.asarray(world, float) - t) @ R

        if self.visible["skeleton"]:
            self.links.points = self._scene(
                _segments(chain[:, :3].reshape(-1, 3), chain[:, 1:].reshape(-1, 3)))
            self.joints.points = self._scene(chain[:, :3].reshape(-1, 3))
            self.feet.points = self._scene(chain[:, 3])
            self.body.points = self._scene(_loop(chain[:, 0]))

        if self.visible["stance"]:
            contact_body = to_body(st.contact)
            self.drops.points = self._scene(_segments(chain[:, 3], contact_body))
            self.contacts.points = self._scene(contact_body)
            self.center.points = self._scene(to_body(st.center)[None, :])
            # Where the gait parks each foot: the mount pushed out along its
            # yaw, coxa at zero. World-fixed, so a body shift slides them
            self.neutral.points = self._scene(to_body(cfg.neutral_feet()))
            hull = st.support
            if len(hull) >= 3:
                flat = np.column_stack([hull, np.zeros(len(hull))])
                self.support.points = self._scene(_loop(to_body(flat)))
            self.support.visible = len(hull) >= 3

        grid = _grid(400.0, 50.0)
        self.ground.points = self._scene(
            to_body(grid.reshape(-1, 3)).reshape(-1, 2, 3))
        return st

    @property
    def min_height(self) -> float:
        """How close the body origin can get to the ground before the chassis is
        resting on it (mm), from wherever the overlay's origin currently sits."""
        if self.chassis_floor is None:
            return -np.inf
        return self.z_offset - self.chassis_floor

    def _scene(self, pts_mm) -> np.ndarray:
        return to_scene(pts_mm, self.z_offset)


# What the CAD actually is

@dataclass
class UrdfGeometry:
    """The CAD's leg geometry, measured off the URDF at zero joint angles and
    expressed in the KINEMATIC frame (mm). Only axis positions are read, never
    angles: the URDF's zero is servo zero, which is not the kinematic zero.

    Read the axes IN THE LEG PLANE only -- radial distance and height. A joint
    frame's remaining coordinate slides freely along its own axis and is
    wherever the CAD author dropped it, usually a servo horn: the coxa frames
    sit 22 mm above the joints they turn, the femur and tibia frames 22 mm to
    one side. Neither means anything. Where the leg actually is comes from the
    meshes, which is what `leg_plane` and `tibia_tip` measure."""

    coxa_axis: np.ndarray   # (6, 3), the real mount points (x, y; z is arbitrary)
    femur_axis: np.ndarray  # (6, 3), radial and z only
    tibia_axis: np.ndarray  # (6, 3), radial and z only
    tibia_tip: np.ndarray   # (6,) mm, tibia axis -> the tip of its mesh, in the
                            # leg plane: the foot (NaN if the meshes are absent)
    leg_plane: np.ndarray   # (6,) mm, how far the limb runs to the side of the
                            # coxa axis: the CAD's own coxa_offset
    chassis_floor: Optional[float] = None
    # Lowest point of everything that is not a leg (mm over the URDF's base
    # origin): the battery, on this robot. The core models the body as a point
    # and sits down onto z = 0, so this is how deep the real robot would be
    # buried if the ground were drawn where the core puts it. None: no meshes.

    @property
    def femur_plane(self) -> float:
        """Height of the CAD's coxa-femur joints over the URDF's base origin
        (mm), and the overlay's z offset.

        This is the robot's z = 0: the plane the mounts sit on, with mount z
        measured from it. The coxa axes cannot define it -- they are vertical
        lines, so `coxa_axis`'s own z is wherever the CAD happened to put the
        joint frame along them and means nothing. The femur axes are horizontal,
        so their height is real, and lining the model's origin up with it is
        what puts the model's feet on the CAD's feet."""
        return float(np.mean(self.femur_axis[:, 2]))


def measure_urdf(model, cfg: KinematicConfig) -> UrdfGeometry:
    """Measure a loaded yourdfpy model. The joint axes come from the pose the URDF
    ships in; the foot, the leg plane and the chassis floor from the meshes.

    `cfg` supplies the servo map, which is what says where the tibia is pointing
    in that shipped pose -- without it the tip cannot be told from any other
    corner of the mesh."""
    joints = {j.name: j for j in model.robot.joints}
    axes = {}
    for name in JOINTS:
        origins = [model.get_transform(joints[f"leg_{leg + 1}_{name}"].child, "base")[:3, 3]
                   for leg in range(N_LEGS)]
        axes[name] = np.array(origins)  # metres, URDF frame

    tip = np.full(N_LEGS, np.nan)
    plane = np.full(N_LEGS, np.nan)
    floor = None
    try:  # the meshes are optional: the axis rows work without them
        scene = model.scene
        # Leg angles the shipped pose corresponds to: servo zero through the map
        kin0 = cfg.kin_angles(np.zeros(N_LEGS * N_JOINTS))
        for leg in range(N_LEGS):
            # The leg's own directions, in the URDF frame (+90 deg from the model)
            yaw = cfg.mount_yaw[leg] + np.pi / 2
            fwd = np.array([np.cos(yaw), np.sin(yaw)])
            side = np.array([-fwd[1], fwd[0]])

            node = f"leg_{leg + 1}_tibia_visual"
            T = scene.graph.get(node)[0]
            v = (np.asarray(scene.geometry[node].vertices) @ T[:3, :3].T + T[:3, 3]) / MM
            rel = v - axes["tibia"][leg] / MM

            # Walk out along the tibia to its far end, then average across the
            # rim so the point lands on the leg's centreline, not on a corner
            along_dir = kin0[leg, 1] + kin0[leg, 2]
            along = ((rel[:, :2] @ fwd) * np.cos(along_dir) +
                     rel[:, 2] * np.sin(along_dir))
            rim = v[along > along.max() - 1.0].mean(axis=0)

            foot = rim - axes["tibia"][leg] / MM
            tip[leg] = np.hypot(foot[:2] @ fwd, foot[2])
            plane[leg] = (rim[:2] - axes["coxa"][leg, :2] / MM) @ side

        lows = []
        for node in scene.geometry:
            if node.startswith("leg_"):
                continue  # the legs swing; the chassis is what the body rests on
            T = scene.graph.get(node)[0]
            v = np.asarray(scene.geometry[node].vertices) @ T[:3, :3].T + T[:3, 3]
            lows.append(v[:, 2].min() / MM)
        floor = min(lows) if lows else None
    except Exception:
        pass

    return UrdfGeometry(coxa_axis=from_scene(axes["coxa"]),
                        femur_axis=from_scene(axes["femur"]),
                        tibia_axis=from_scene(axes["tibia"]),
                        tibia_tip=tip, leg_plane=plane, chassis_floor=floor)


def compare(cfg: KinematicConfig, urdf: UrdfGeometry) -> str:
    """Config against CAD, as markdown: the numbers behind "does the skeleton
    match the model". The coxa axis -> coxa-femur joint step is split along the
    configured mount yaw into its radial part (the coxa link) and its sideways
    part (the offset that carries the leg plane off the axis)."""
    fwd = np.column_stack([np.cos(cfg.mount_yaw), np.sin(cfg.mount_yaw)])
    d = urdf.femur_axis - urdf.coxa_axis
    radial = (d[:, :2] * fwd).sum(1)
    femur = np.linalg.norm(urdf.tibia_axis - urdf.femur_axis, axis=1)
    mount_err = np.linalg.norm(urdf.coxa_axis[:, :2] - cfg.mount[:, :2], axis=1)
    tip = float(np.nanmean(urdf.tibia_tip))
    lateral = float(np.nanmean(urdf.leg_plane))

    def row(name, want, got, note=""):
        return f"| {name} | {want:.1f} | {got:.1f} | {got - want:+.1f} | {note} |"

    lines = [
        "**config.yml vs URDF** (mm, kinematic frame)",
        "",
        "| what | config | CAD | delta | |",
        "| --- | --- | --- | --- | --- |",
        row("coxa", cfg.coxa, float(radial.mean()), "radial"),
        row("coxa offset", cfg.coxa_offset, lateral, "leg plane, from the meshes")
        if np.isfinite(lateral) else f"| coxa offset | {cfg.coxa_offset:.1f} | - | - | no mesh |",
        row("femur", cfg.femur, float(femur.mean())),
        row("tibia", cfg.tibia, tip, "to the mesh tip") if np.isfinite(tip)
        else f"| tibia | {cfg.tibia:.1f} | - | - | no mesh |",
        row("mount error", 0.0, float(mount_err.max()), "worst leg, x/y"),
        row("mount z", float(cfg.mount[:, 2].mean()), 0.0,
            "the mounts define z = 0"),
        "",
        "| leg | config x, y | CAD x, y | delta |",
        "| --- | --- | --- | --- |",
    ]
    for leg in range(N_LEGS):
        lines.append(f"| {TAGS[leg]} "
                     f"| {cfg.mount[leg, 0]:.1f}, {cfg.mount[leg, 1]:.1f} "
                     f"| {urdf.coxa_axis[leg, 0]:.1f}, {urdf.coxa_axis[leg, 1]:.1f} "
                     f"| {mount_err[leg]:.1f} |")
    return "\n".join(lines)


def report(stance: Stance, cfg: KinematicConfig) -> str:
    """The live stance as markdown: is the robot standing where config says?"""
    down = int(stance.grounded.sum())
    radius = stance.radius[stance.grounded] if down else stance.radius
    margin = "-" if not np.isfinite(stance.margin) else f"{stance.margin:.0f} mm"
    body = (f"**body** {stance.body_height:.1f} mm (config {cfg.standing_height:.1f}, "
            f"{stance.body_height - cfg.standing_height:+.1f})")
    if stance.resting:  # the core wants the body lower than the chassis allows
        body = (f"**body** {stance.body_height:.1f} mm, chassis down "
                f"(core asks {stance.commanded:.1f})")
    lines = [
        body,
        "",
        f"**stance** {radius.mean():.1f} mm mean (config {cfg.stance_radius:.1f}, "
        f"{radius.mean() - cfg.stance_radius:+.1f}), "
        f"spread {radius.max() - radius.min():.1f}",
        "",
        f"**support** {down}/{N_LEGS} down, margin {margin}",
        "",
        "| leg | radius | delta | height |",
        "| --- | --- | --- | --- |",
    ]
    for leg in range(N_LEGS):
        lifted = "" if stance.grounded[leg] else " ^"
        lines.append(f"| {TAGS[leg]}{lifted} | {stance.radius[leg]:.1f} "
                     f"| {stance.radius[leg] - cfg.stance_radius:+.1f} "
                     f"| {stance.height[leg]:+.1f} |")
    return "\n".join(lines)
