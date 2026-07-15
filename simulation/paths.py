"""
Path helpers tying the simulation to its submodules at the repo root:
  Hexapod-Controller   the Pi-side `hexapod` client package (installed editable)
  Hexapod-Firmware     the C++ core, compiled by bridge/build.sh into the .so
  Hexapod-Hardware     the URDF + STL meshes the sims render
"""
from __future__ import annotations

from pathlib import Path

# Repo root == Hexapod-Simulation (the parent of the `simulation` package).
SIM_ROOT = Path(__file__).resolve().parent.parent
# The submodules live at the repo root, next to the bridge.
CONTROLLER_ROOT = SIM_ROOT / "Hexapod-Controller"
HARDWARE_ROOT = SIM_ROOT / "Hexapod-Hardware"
BRIDGE_ROOT = SIM_ROOT / "bridge"


def get_controller_path(*parts) -> Path:
    """A path inside the Hexapod-Controller repo."""
    return CONTROLLER_ROOT.joinpath(*parts)


def get_hardware_path(*parts) -> Path:
    """A path inside the Hexapod-Hardware repo."""
    return HARDWARE_ROOT.joinpath(*parts)


def get_bridge_lib() -> Path:
    """The compiled firmware core (built by bridge/build.sh)."""
    return BRIDGE_ROOT / "libhexapod_fw.so"


def default_urdf() -> Path:
    """The hexapod URDF shipped by the Hexapod-Hardware repo."""
    return HARDWARE_ROOT / "hexapod.urdf"
