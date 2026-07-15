"""
PyBullet physics simulation of the Hexapod.

Runs the REAL C++ firmware core in-process (via the ctypes bridge) and drives it
with the ordinary Pi-side HexapodClient, the same client you would use over a
serial link to the board. PyBullet is only the "hardware": it renders the URDF
and applies the servo commands the firmware produces.

    HexapodClient -> SimTransport -> Firmware (C++ core) -> servo deg -> PyBullet

This is a scripted demo. For live control use `simulation.bullet.teleop` or 
`simulation.viser.main`.

./bridge/build.sh  # build the bridge first  
python3 -m simulation.bullet.main [--urdf PATH] [--gait tripod]
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc

from simulation import Firmware, SimTransport, paths
from simulation.bullet.interface import BulletInterface
from hexapod import GaitId, HexapodClient


GAITS = {"tripod": GaitId.TRIPOD, "wave": GaitId.WAVE, "ripple": GaitId.RIPPLE}


def main() -> None:
    ap = argparse.ArgumentParser(description="Hexapod-Reimagined PyBullet demo")
    ap.add_argument("--urdf", default=str(paths.default_urdf()),
                    help="path to hexapod.urdf (from the Hexapod-Hardware submodule)")
    ap.add_argument("--gait", "-g", default="tripod", choices=list(GAITS))
    ap.add_argument("--vx", "-x", type=float, default=120.0, help="forward mm/s")
    ap.add_argument("--vy", "-y", type=float, default=0.0, help="strafe mm/s")
    ap.add_argument("--yaw", "-v", type=float, default=20.0, help="yaw deg/s")
    ap.add_argument("--control-rate", "-c", type=float, default=50.0,
                    help="Hz (matches cfg::CONTROL_RATE_HZ)")
    ap.add_argument("--sim-rate", "-s", type=float, default=240.0, help="physics Hz")
    ap.add_argument("--width", "-w", type=int, default=1280)
    ap.add_argument("--height", "-e", type=int, default=720)
    ap.add_argument("--video", type=str, default=None,
                    help="if set, record the run to this .mp4 path")
    args = ap.parse_args()

    if not Path(args.urdf).exists():
        raise SystemExit(f"URDF not found: {args.urdf}\n"
                         "Add the Hexapod-Hardware submodule or pass --urdf.")

    # Firmware core + client (same client as on the real robot)
    fw = Firmware()
    bot = HexapodClient(SimTransport(fw))

    # PyBullet world
    phys = bc.BulletClient(connection_mode=p.GUI,
                           options=f"--width={args.width} --height={args.height}")
    phys.configureDebugVisualizer(phys.COV_ENABLE_GUI, 0)
    phys.configureDebugVisualizer(phys.COV_ENABLE_SHADOWS, 0)
    phys.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=180,
                                    cameraPitch=-35, cameraTargetPosition=[0, 0, 0])
    phys.setAdditionalSearchPath(pybullet_data.getDataPath())
    phys.setGravity(0, 0, -9.81)
    phys.loadURDF("plane.urdf")
    robot = phys.loadURDF(args.urdf, [0, 0, 0.20],
                          phys.getQuaternionFromEuler([0, 0, 0]),
                          flags=phys.URDF_USE_INERTIA_FROM_FILE)
    iface = BulletInterface(phys, robot)

    if args.video:
        Path(args.video).parent.mkdir(parents=True, exist_ok=True)
        phys.startStateLogging(phys.STATE_LOGGING_VIDEO_MP4, args.video)

    # scripted timeline (seconds -> action)
    def do_enable():   bot.enable(); bot.set_gait(GAITS[args.gait])
    def do_walk():     bot.set_velocity(args.vx, args.vy, 0.0)
    def do_turn():     bot.set_velocity(args.vx, args.vy, args.yaw)
    def do_lean():     bot.set_body_pose(roll=5.0, pitch=-5.0)
    def do_stop():     bot.set_velocity(0.0, 0.0, 0.0); bot.set_body_pose()
    def do_shutdown(): bot.shutdown()

    timeline = [(0.5, do_enable), (5.0, do_walk), (12.0, do_turn),
                (20.0, do_lean), (26.0, do_stop), (29.0, do_shutdown), (34.0, None)]
    ti = 0

    control_dt = 1.0 / args.control_rate
    sim_steps = max(1, round(args.sim_rate / args.control_rate))

    t = 0.0
    last = time.perf_counter()
    try:
        while phys.isConnected():
            # fire the next scheduled action(s)
            while ti < len(timeline) and t >= timeline[ti][0]:
                action = timeline[ti][1]
                if action is None:
                    return
                action()
                ti += 1

            bot.heartbeat()                  # keep the command watchdog fed
            fw.update(control_dt)            # advance the real firmware
            iface.apply(fw.servos(), powered=fw.powered())

            for _ in range(sim_steps):
                phys.stepSimulation()

            t += control_dt
            last += control_dt               # pace to wall clock
            time.sleep(max(0.0, last - time.perf_counter()))
    finally:
        if args.video:
            time.sleep(0.1)
            phys.stopStateLogging(phys.STATE_LOGGING_VIDEO_MP4)
        try:
            print("final telemetry:", bot.get_telemetry())
        except Exception:
            pass
        if phys.isConnected():
            phys.disconnect()


if __name__ == "__main__":
    main()
