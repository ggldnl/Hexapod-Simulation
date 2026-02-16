"""
Bridges simulation repo with hardware and controller repos
"""

from pathlib import Path

# Get the controller and hardware paths relative to this file
CONTROLLER_ROOT = Path(__file__).parent.parent / "Hexapod-Controller"
HARDWARE_ROOT = Path(__file__).parent.parent / "Hexapod-Hardware"

def get_controller_path(*parts):
    """Helper to get paths within controller directory"""
    return CONTROLLER_ROOT / Path(*parts)

def get_hardware_path(*parts):
    """Helper to get paths within hardware directory"""
    return HARDWARE_ROOT / Path(*parts)