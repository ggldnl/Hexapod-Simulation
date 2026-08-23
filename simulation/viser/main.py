"""
Viser visualization of the Hexapod stack, with live gait controls.

Runs the REAL C++ firmware core in-process (via the ctypes bridge) and drives it
with the ordinary Pi-side HexapodClient. Viser renders the URDF in the browser and
gives you a control panel to drive the gait directly.

Viser is a pure visualizer (no physics), so the body stays put and the legs cycle
in place. It is meant to "show the robot walking" and to poke at the gait live.

Controls (in the browser panel)
    Lifecycle   Enable / Shutdown / Stop, plus the reported state
    Gait        tripod / wave / ripple
    Velocity    vx, vy (mm/s) and yaw (deg/s)
    Body pose   height, roll, pitch, yaw offsets from standing
    Kinematics  the config.yml skeleton drawn over the mesh, the ground plane
                and the foot contacts, plus the stance numbers (see skeleton.py).
                Turn the meshes off to read the skeleton on its own

The board is provisioned from config.yml at startup, exactly as the Pi does over
the serial link, so the firmware and the overlay are reading the same numbers.

Build the bridge first:  ./bridge/build.sh
Run:                     python3 -m simulation.viser.main   [--urdf PATH] [--port 8080]
Then open the printed URL (default http://localhost:8080).
"""
from __future__ import annotations

import argparse
import time
from collections import deque
from pathlib import Path

import viser
import yourdfpy
from viser.extras import ViserUrdf

from simulation import Firmware, SimTransport, paths
from simulation.viser.interface import ViserInterface
from simulation.viser import skeleton, utils
from hexapod import GaitId, HexapodClient


GAIT_ID = {"tripod": GaitId.TRIPOD, "wave": GaitId.WAVE, "ripple": GaitId.RIPPLE}


def main() -> None:

    ap = argparse.ArgumentParser(description="Hexapod Viser viz + controls")
    ap.add_argument("--urdf", default=str(paths.default_urdf()),
                    help="path to hexapod.urdf")
    ap.add_argument("--gait", "-g", default="tripod", choices=list(GAIT_ID))
    ap.add_argument("--port", "-p", type=int, default=8080, help="Viser server port")
    ap.add_argument("--control-rate", "-c", type=float, default=50.0,
                    help="Hz (matches cfg::CONTROL_RATE_HZ)")
    ap.add_argument("--config", default=None,
                    help="path to config.yml (default: the one in the client package)")
    ap.add_argument("--no-provision", action="store_true",
                    help="leave the core on its baked defaults instead of config.yml")
    args = ap.parse_args()

    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        raise SystemExit(f"URDF not found: {urdf_path}\n"
                         "Add the Hexapod-Hardware submodule or pass --urdf.")

    # One config for both the core and the overlay, so the skeleton on screen is
    # always the model the firmware is actually walking on
    config = skeleton.load_config(args.config)
    kin_cfg = skeleton.KinematicConfig.load(config)

    # Firmware core + client (same client as on the real robot)
    fw = Firmware()
    bot = HexapodClient(SimTransport(fw))
    if not args.no_provision:
        try:
            bot.provision(config)
        except Exception as e:  # a bad config should not cost you the viewer
            print(f"warning: provisioning failed ({e}); the core keeps its baked "
                  "defaults. If a section was rejected for its length, the bridge "
                  "is older than the protocol: rebuild it with ./bridge/build.sh")

    # Viser scene
    server = viser.ViserServer(port=args.port)
    robot_model = yourdfpy.URDF.load(str(urdf_path), mesh_dir=str(urdf_path.parent))
    # Left on the default (GLB) path on purpose: it is the only one that carries
    # the URDF's own materials and exported normals. Fading the meshes would mean
    # viser's flat-shaded add_mesh_simple instead, which does not look like the
    # robot, so the panel hides them outright rather than making them translucent
    urdf = ViserUrdf(server, robot_model, root_node_name="/robot")
    iface = ViserInterface(urdf)

    # The config's kinematic model, drawn over the mesh, with its origin on the
    # CAD's femur axes -- the plane the model's foot heights are really measured
    # from -- and its ground stopped by the CAD's own chassis (see skeleton.py)
    urdf_geom = skeleton.measure_urdf(robot_model, kin_cfg)
    overlay = skeleton.SkeletonOverlay(server, kin_cfg, z_offset=urdf_geom.femur_plane,
                                       chassis_floor=urdf_geom.chassis_floor)
    # model_check = skeleton.compare(kin_cfg, urdf_geom)
    # print(model_check)

    # Control panel
    # Discrete actions (lifecycle / gait) are queued from GUI callbacks and drained
    # by the loop, so ONLY the loop ever touches the client (no cross-thread races)
    actions: "deque[tuple]" = deque()

    with server.gui.add_folder("Lifecycle"):
        enable_btn = server.gui.add_button("Enable (stand up)")
        shutdown_btn = server.gui.add_button("Shutdown (sit down)")
        stop_btn = server.gui.add_button("Stop")
        state_txt = server.gui.add_text("State", initial_value="OFF", disabled=True)

    with server.gui.add_folder("Gait"):
        gait_dd = server.gui.add_dropdown("Pattern", ("tripod", "wave", "ripple"),
                                          initial_value=args.gait)

    with server.gui.add_folder("Velocity"):
        vx_sl = server.gui.add_slider("vx (mm/s)", -200.0, 200.0, 5.0, 0.0)
        vy_sl = server.gui.add_slider("vy (mm/s)", -200.0, 200.0, 5.0, 0.0)
        yaw_sl = server.gui.add_slider("yaw (deg/s)", -60.0, 60.0, 1.0, 0.0)
        zero_btn = server.gui.add_button("Zero velocity")

    with server.gui.add_folder("Body pose"):
        height_sl = server.gui.add_slider("height (mm)", -40.0, 40.0, 1.0, 0.0)
        roll_sl = server.gui.add_slider("roll (deg)", -15.0, 15.0, 0.5, 0.0)
        pitch_sl = server.gui.add_slider("pitch (deg)", -15.0, 15.0, 0.5, 0.0)
        poseyaw_sl = server.gui.add_slider("yaw (deg)", -15.0, 15.0, 0.5, 0.0)

    with server.gui.add_folder("Kinematic model"):
        mesh_cb = server.gui.add_checkbox("Meshes", True)
        skel_cb = server.gui.add_checkbox("Skeleton", True)
        stance_cb = server.gui.add_checkbox("Contacts + targets", True)
        zoff_nb = server.gui.add_number("z offset (mm)", initial_value=overlay.z_offset,
                                        step=0.5)

    with server.gui.add_folder("Telemetry"):
        odom_txt = server.gui.add_text("Odometry", initial_value="-", disabled=True)
        power_txt = server.gui.add_text("Power", initial_value="-", disabled=True)

    enable_btn.on_click(lambda _: actions.append(("enable",)))
    shutdown_btn.on_click(lambda _: actions.append(("shutdown",)))
    stop_btn.on_click(lambda _: actions.append(("stop",)))
    zero_btn.on_click(lambda _: actions.append(("zero",)))
    gait_dd.on_update(lambda _: actions.append(("gait", gait_dd.value)))

    # Control loop
    control_dt = 1.0 / args.control_rate
    telemetry_accum = 0.0
    last = time.perf_counter()
    print(f"Viser running at http://localhost:{args.port}  (Ctrl+C to quit)")

    try:
        while True:
            # Drain queued GUI actions (client touched only here)
            while actions:
                act = actions.popleft()
                if act[0] == "enable":
                    bot.enable()
                    bot.set_gait(GAIT_ID[gait_dd.value])
                elif act[0] == "shutdown":
                    bot.shutdown()
                elif act[0] == "stop":
                    bot.stop()
                    vx_sl.value = vy_sl.value = yaw_sl.value = 0.0
                elif act[0] == "zero":
                    vx_sl.value = vy_sl.value = yaw_sl.value = 0.0
                elif act[0] == "gait":
                    bot.set_gait(GAIT_ID[act[1]])

            # Stream setpoints every tick (also pets the command watchdog)
            bot.set_velocity(vx_sl.value, vy_sl.value, yaw_sl.value)
            bot.set_body_pose(z=height_sl.value, roll=roll_sl.value,
                              pitch=pitch_sl.value, yaw=poseyaw_sl.value)

            # Advance the firmware and pose the model
            fw.update(control_dt)
            servos = fw.servos()
            iface.apply(servos, powered=fw.powered())

            # Redraw the kinematic overlay from those same servo angles, in the
            # body pose the core is actually holding (mid-slew, not the target)
            urdf.show_visual = mesh_cb.value
            overlay.set_visible(skeleton=skel_cb.value, stance=stance_cb.value)
            overlay.z_offset = zoff_nb.value
            stance = overlay.update(servos, bot.get_body_pose())

            # A little ground arrow showing the commanded ground velocity, on
            # the same ground plane the overlay draws
            walking = vx_sl.value or vy_sl.value or yaw_sl.value
            if walking:
                _draw_velocity_arrow(server, vx_sl.value, vy_sl.value,
                                     z=overlay.z_offset - stance.body_height)

            # Telemetry readout (~5 Hz)
            telemetry_accum += control_dt
            if telemetry_accum >= 0.2:
                telemetry_accum = 0.0
                try:
                    tel = bot.get_telemetry()
                    state_txt.value = tel.state.name
                    odom_txt.value = (f"x={tel.odom_x:+.0f} y={tel.odom_y:+.0f} mm  "
                                      f"yaw={tel.odom_yaw:+.0f} deg")
                    power_txt.value = (f"{'ON' if fw.powered() else 'off'}  "
                                       f"{tel.voltage:.1f} V  {tel.current:.2f} A")
                except Exception:
                    pass

            last += control_dt
            time.sleep(max(0.0, last - time.perf_counter()))
    except KeyboardInterrupt:
        print("\nViser stopped")


def _draw_velocity_arrow(server, vx: float, vy: float, z: float = 0) -> None:
    """Draw (replace) a ground-plane line for the commanded velocity, in meters.
    Re-adding with the same name replaces the node."""
    scale = 1.0 / 1000.0  # mm/s -> a length in meters
    if abs(vx) < 1e-3 and abs(vy) < 1e-3:
        start, end = (0, 0, z * scale), (0, 0, z * scale + 0.01)  # tiny upright tick when stopped
    else:
        start, end = (0, 0, z * scale), (-vy * scale, vx * scale, z * scale)  # different frame convention
    try:
        utils.add_line(server, name="/cmd_vel", start=start, end=end,
                       line_width=4.0, colors=(0, 200, 0))
    except Exception:
        pass


if __name__ == "__main__":
    main()
