"""
PyBullet physics simulation of the Hexapod.

Runs the REAL C++ firmware core in-process (via the ctypes bridge) and drives it
with the ordinary Pi-side HexapodClient, the same client you would use over a
serial link to the board. PyBullet is only the "hardware": it renders the URDF
and applies the servo commands the firmware produces.

This is a scripted demo. For live control use `simulation.bullet.teleop` or
`simulation.viser.main`.

    ./bridge/build.sh                                   # build the bridge first
    python3 -m simulation.bullet.main                   # [--urdf PATH] [--gait tripod]
    python3 -m simulation.bullet.main --speed-test      # top-speed benchmark
    python3 -m simulation.bullet.main --speed-test --headless   # no window, faster than real time

The board is provisioned from config.yml at startup, exactly as the Pi does over
the serial link (--config PATH, or --no-provision to keep the firmware's baked
defaults).

--speed-test walks straight at the config's top speed, stops, restarts and stops
again, then prints the speed actually reached (PyBullet ground truth) next to the
speed the gait executes and Chica's, and the peak joint speed the firmware
commanded while walking, stopping and restarting.
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import pybullet as p
import pybullet_data
import pybullet_utils.bullet_client as bc
import yaml

from simulation import Firmware, SimTransport, paths
from simulation.bullet.interface import BulletInterface
from simulation.bullet.metrics import (CHICA_RACE, CHICA_STANDARD, MG996R_DEG_S,
                                       Hud, JointRate, Speedometer, follow_camera,
                                       gait_limits)
from hexapod import GaitId, HexapodClient
from hexapod.client import DEFAULT_CONFIG


GAITS = {"tripod": GaitId.TRIPOD, "wave": GaitId.WAVE, "ripple": GaitId.RIPPLE}


def main() -> None:

    ap = argparse.ArgumentParser(description="Hexapod PyBullet demo")
    ap.add_argument("--urdf", default=str(paths.default_urdf()),
                    help="path to hexapod.urdf")
    ap.add_argument("--config", default=None,
                    help="path to config.yml (default: the one in the client package)")
    ap.add_argument("--no-provision", action="store_true",
                    help="leave the core on its baked defaults instead of config.yml")
    ap.add_argument("--gait", "-g", default="tripod", choices=list(GAITS))
    ap.add_argument("--vx", "-x", type=float, default=None,
                    help="forward mm/s (default 120, or the gait's top speed with --speed-test)")
    ap.add_argument("--vy", "-y", type=float, default=0.0, help="strafe mm/s")
    ap.add_argument("--yaw", "-v", type=float, default=20.0, help="yaw deg/s")
    ap.add_argument("--speed-test", action="store_true",
                    help="benchmark: walk straight at top speed, stop, restart, report")
    ap.add_argument("--hold", type=float, default=10.0,
                    help="seconds of straight walking in --speed-test")
    ap.add_argument("--headless", action="store_true",
                    help="no window, run as fast as possible (pairs with --speed-test)")
    ap.add_argument("--no-follow", action="store_true",
                    help="keep the camera still instead of following the robot")
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

    config_path = Path(args.config) if args.config else DEFAULT_CONFIG
    with open(config_path) as f:
        config = yaml.safe_load(f)
    limits = gait_limits(config, args.gait)

    # Firmware core + client (same client as on the real robot), provisioned from
    # the config before anything is energized, as the Pi does at connect
    fw = Firmware()
    bot = HexapodClient(SimTransport(fw))
    if not args.no_provision:
        try:
            bot.provision(config)
        except Exception as e:  # a bad config should not cost you the sim
            print(f"warning: provisioning failed ({e}); the core keeps its baked "
                  "defaults. If a section was rejected for its length, the bridge "
                  "is older than the protocol: rebuild it with ./bridge/build.sh")
    bot.set_gait(GAITS[args.gait])

    # PyBullet world
    gui = not args.headless
    if gui:
        phys = bc.BulletClient(connection_mode=p.GUI,
                               options=f"--width={args.width} --height={args.height}")
        phys.configureDebugVisualizer(phys.COV_ENABLE_GUI, 0)
        phys.configureDebugVisualizer(phys.COV_ENABLE_SHADOWS, 0)
        phys.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=180,
                                        cameraPitch=-35, cameraTargetPosition=[0, 0, 0])
    else:
        phys = bc.BulletClient(connection_mode=p.DIRECT)
    phys.setAdditionalSearchPath(pybullet_data.getDataPath())
    phys.setGravity(0, 0, -9.81)
    phys.loadURDF("plane.urdf")
    robot = phys.loadURDF(args.urdf, [0, 0, 0.20],
                          phys.getQuaternionFromEuler([0, 0, 0]),
                          flags=phys.URDF_USE_INERTIA_FROM_FILE)
    iface = BulletInterface(phys, robot)

    if args.video and gui:
        Path(args.video).parent.mkdir(parents=True, exist_ok=True)
        phys.startStateLogging(phys.STATE_LOGGING_VIDEO_MP4, args.video)

    control_dt = 1.0 / args.control_rate
    sim_steps = max(1, round(args.sim_rate / args.control_rate))
    # Physics must advance exactly one control tick per tick: with PyBullet's
    # default 1/240 s step, 5 steps would be 20.8 ms against the firmware's 20 ms,
    # and the world would run 4% faster than the robot thinks (speeds read high)
    phys.setTimeStep(control_dt / sim_steps)
    speedo = Speedometer(phys, robot, window=limits["cycle_time"])
    joints = JointRate()
    hud = Hud(phys, robot) if gui else None

    def step(commanded=None) -> None:
        """One control tick: firmware, physics, measurements, HUD, pacing."""
        nonlocal last, hud_t
        bot.heartbeat()                  # keep the command watchdog fed
        fw.update(control_dt)            # advance the real firmware
        servos = fw.servos()
        iface.apply(servos, powered=fw.powered())
        for _ in range(sim_steps):
            phys.stepSimulation()
        joints.update(servos, control_dt)
        speedo.update(control_dt, bot.get_telemetry())
        if gui:
            if not args.no_follow:
                follow_camera(phys, robot)
            hud_t += control_dt
            if hud_t >= 0.2:             # debug text is slow to redraw: ~5 Hz
                hud_t = 0.0
                hud.show(speedo.lines(commanded))
            last += control_dt           # pace to wall clock
            time.sleep(max(0.0, last - time.perf_counter()))

    last = time.perf_counter()
    hud_t = 0.0
    try:
        if args.speed_test:
            vx = args.vx if args.vx is not None else limits["top"]
            run_speed_test(args, vx, limits, bot, step, speedo, joints, control_dt)
        else:
            vx = args.vx if args.vx is not None else 120.0
            run_demo(args, vx, bot, step, control_dt)
    finally:
        if args.video and gui:
            time.sleep(0.1)
            phys.stopStateLogging(phys.STATE_LOGGING_VIDEO_MP4)
        try:
            print("final telemetry:", bot.get_telemetry())
        except Exception:
            pass
        if phys.isConnected():
            phys.disconnect()


def run_demo(args, vx, bot, step, control_dt) -> None:
    """The scripted demo: stand, look around, walk, walk + turn, stop, sit."""
    timeline = [
        (0.5,  lambda: bot.enable()),
        (5.0,  lambda: bot.set_body_pose(roll=5.0, pitch=-5.0, yaw=5.0)),
        (6.0,  lambda: bot.set_body_pose(roll=-5.0, pitch=-5.0, yaw=-5.0)),
        (7.0,  lambda: bot.set_body_pose(roll=0.0, pitch=0.0, yaw=0.0)),
        (8.0,  lambda: bot.set_velocity(vx, args.vy, 0.0)),
        (12.0, lambda: bot.set_velocity(vx, args.vy, args.yaw)),
        (18.0, lambda: bot.set_velocity(0.0, 0.0, 0.0)),
        (22.0, lambda: bot.shutdown()),
        (25.0, None)
    ]
    ti = 0
    t = 0.0
    while True:
        # fire the next scheduled action(s)
        while ti < len(timeline) and t >= timeline[ti][0]:
            action = timeline[ti][1]
            if action is None:
                return
            action()
            ti += 1
        step()
        t += control_dt


def run_speed_test(args, vx, limits, bot, step, speedo, joints, control_dt) -> None:
    """Walk straight at vx, stop, restart, stop, sit, then print the report.

    The velocity is re-sent every tick of a walking window, so the test does not
    depend on exactly when the stand-up finishes. The speed is measured after a
    settling period, as the average over the rest of the walk."""
    t_walk = 4.5                       # enable at 0.5; the stand-up is done by ~3.5
    t_stop = t_walk + args.hold
    t_restart = t_stop + 2.0
    t_stop2 = t_restart + 2.0
    t_sit = t_stop2 + 2.0
    t_end = t_sit + 3.0
    settle = min(2.0, args.hold / 2)   # accelerate + one cycle before measuring

    print(f"speed test: {args.gait}, {vx:.0f} mm/s straight for {args.hold:.0f} s "
          f"(config allows {limits['top']:.0f})")
    t = 0.0
    enabled = sat = False
    start = None                       # (t, real xy, odom xy) when measuring starts
    end = None                         # same, at the stop command
    stopped_after = None               # s from the stop command to leaving WALK
    peaks = {"walk": 0.0, "stop": 0.0, "restart": 0.0}
    phase = None

    while t < t_end:
        tel = bot.get_telemetry()
        if start is None and t >= t_walk + settle:
            start = (t, speedo.position_mm(), (tel.odom_x, tel.odom_y))
            if tel.state.name != "WALK":
                print(f"warning: not walking yet at t={t:.1f} s (state {tel.state.name}); "
                      "the stand-up may be slower than expected, raise --hold")
        if end is None and t >= t_stop:
            end = (t, speedo.position_mm(), (tel.odom_x, tel.odom_y))
        if end is not None and stopped_after is None and t > end[0] \
                and tel.state.name != "WALK":
            stopped_after = t - end[0]

        if not enabled and t >= 0.5:
            bot.enable()
            enabled = True

        walking = t_walk <= t < t_stop or t_restart <= t < t_stop2
        commanded = vx if walking else 0.0
        if t >= t_sit:
            if not sat:
                bot.shutdown()
                sat = True
        elif t >= t_walk:
            bot.set_velocity(commanded, 0.0, 0.0)

        # which joint-speed bucket this tick belongs to
        if t_walk + settle <= t < t_stop:
            now = "walk"
        elif t_stop <= t < t_restart or t_stop2 <= t < t_sit:
            now = "stop"
        elif t_restart <= t < t_restart + 1.0:
            now = "restart"
        elif t >= t_sit:
            now = "sit"
        else:
            now = None
        if now != phase:
            if phase in peaks:
                peaks[phase] = max(peaks[phase], joints.peak)
            joints.restart()
            phase = now

        step(commanded if t >= t_walk else None)
        t += control_dt

    if start is None or end is None:
        print("speed test did not complete")
        return

    dt = end[0] - start[0]
    real = ((end[1][0] - start[1][0]) ** 2 + (end[1][1] - start[1][1]) ** 2) ** 0.5 / dt
    gait = ((end[2][0] - start[2][0]) ** 2 + (end[2][1] - start[2][1]) ** 2) ** 0.5 / dt
    grip = f"  (grip {100 * real / gait:.0f}%)" if gait > 1 else ""
    print()
    print(f"  config limits  lin_vel_max {limits['lin_vel_max']:.0f} mm/s | stride cap "
          f"{limits['stride_cap']:.0f} mm/s = {limits['max_stride']:.0f} mm / "
          f"({limits['duty']:.2f} x {limits['cycle_time']:.2f} s)")
    print(f"  gait           {gait:5.0f} mm/s  what the firmware executes (odometry)")
    print(f"  measured       {real:5.0f} mm/s  PyBullet ground truth, mean over "
          f"{dt:.1f} s{grip}")
    print(f"  best cycle     {speedo.best:5.0f} mm/s  best one-cycle average")
    print(f"  Chica          {CHICA_STANDARD:5.0f} mm/s standard, {CHICA_RACE:.0f} race"
          f"  -> you are at {100 * real / CHICA_STANDARD:.0f}% of standard")
    print(f"  joint speed    walk {peaks['walk']:.0f} | stop {peaks['stop']:.0f} | "
          f"restart {peaks['restart']:.0f} deg/s peak commanded "
          f"(MG996R ~{MG996R_DEG_S:.0f} no-load)")
    if stopped_after is not None:
        print(f"  stopping       feet down, IDLE {stopped_after:.2f} s after the stop command")
    print()


if __name__ == "__main__":
    main()
