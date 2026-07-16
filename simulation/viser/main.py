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
from simulation.viser import utils
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
    args = ap.parse_args()

    urdf_path = Path(args.urdf)
    if not urdf_path.exists():
        raise SystemExit(f"URDF not found: {urdf_path}\n"
                         "Add the Hexapod-Hardware submodule or pass --urdf.")

    # Firmware core + client (same client as on the real robot)
    fw = Firmware()
    bot = HexapodClient(SimTransport(fw))

    # Store ground height
    ground = bot.get_body_pose().z

    # Viser scene
    server = viser.ViserServer(port=args.port)
    robot_model = yourdfpy.URDF.load(str(urdf_path), mesh_dir=str(urdf_path.parent))
    urdf = ViserUrdf(server, robot_model, root_node_name="/robot")
    iface = ViserInterface(urdf)

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
            iface.apply(fw.servos(), powered=fw.powered())

            # A little ground arrow showing the commanded ground velocity
            walking = vx_sl.value or vy_sl.value or yaw_sl.value
            if walking:
                _draw_velocity_arrow(server, vx_sl.value, vy_sl.value, z=ground)

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
