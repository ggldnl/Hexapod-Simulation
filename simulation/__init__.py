"""Host simulation for Hexapod-Reimagined.

Runs the real C++ firmware core in-process (via the ctypes bridge) and drives it
with the ordinary Pi-side HexapodClient. Two front-ends render what the firmware
commands:

    simulation.viser   browser visualization with live gait controls
    simulation.bullet  PyBullet physics (scripted demo + interactive teleop)

Both share the same data flow:

    HexapodClient -> SimTransport -> Firmware (C++ core) -> servo deg -> renderer
"""
from . import paths
from .firmware import Firmware, SimTransport

__all__ = ["Firmware", "SimTransport", "paths"]
