"""
Python interfacing layer over the C++ firmware core (via ctypes).

`Firmware` wraps libhexapod_fw.so (the real robot::Robot + router + gait + IK,
compiled for the host). `SimTransport` presents that firmware as a byte transport
so the ordinary `HexapodClient` drives it exactly as it would a serial link. The
simulation ticks `Firmware.update(dt)` and reads `Firmware.servos()`.
"""
from __future__ import annotations

import ctypes
import os
from typing import List

from . import paths
from hexapod.transport import Transport  # noqa: E402


class Firmware:
    """In-process instance of the C++ firmware core."""

    def __init__(self, lib_path: str | None = None) -> None:
        lib_path = lib_path or str(paths.get_bridge_lib())
        self._lib = ctypes.CDLL(os.path.abspath(lib_path))
        self._bind()
        self._h = self._lib.fw_new()
        self.num_servos = self._lib.fw_num_servos()
        self._servo_buf = (ctypes.c_float * self.num_servos)()

    def _bind(self) -> None:
        L = self._lib
        L.fw_new.restype = ctypes.c_void_p
        L.fw_free.argtypes = [ctypes.c_void_p]
        L.fw_update.argtypes = [ctypes.c_void_p, ctypes.c_float]
        L.fw_feed.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int,
                              ctypes.c_char_p, ctypes.c_int]
        L.fw_feed.restype = ctypes.c_int
        L.fw_servos.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_float)]
        L.fw_powered.argtypes = [ctypes.c_void_p]
        L.fw_powered.restype = ctypes.c_int
        L.fw_set_current.argtypes = [ctypes.c_void_p, ctypes.c_float]
        L.fw_set_voltage.argtypes = [ctypes.c_void_p, ctypes.c_float]
        L.fw_num_servos.restype = ctypes.c_int

    def feed(self, data: bytes) -> bytes:
        """Feed wire bytes; return any reply frame bytes."""
        out = ctypes.create_string_buffer(cap := 256)
        n = self._lib.fw_feed(self._h, bytes(data), len(data), out, cap)
        return out.raw[:n]

    def update(self, dt: float) -> None:
        self._lib.fw_update(self._h, ctypes.c_float(dt))

    def servos(self) -> List[float]:
        """Latest 18 servo angles (servo-space degrees)."""
        self._lib.fw_servos(self._h, self._servo_buf)
        return list(self._servo_buf)

    def powered(self) -> bool:
        return bool(self._lib.fw_powered(self._h))

    def set_current(self, amps: float) -> None:
        self._lib.fw_set_current(self._h, ctypes.c_float(amps))

    def set_voltage(self, volts: float) -> None:
        self._lib.fw_set_voltage(self._h, ctypes.c_float(volts))

    def __del__(self) -> None:
        try:
            self._lib.fw_free(self._h)
        except Exception:
            pass


class SimTransport(Transport):
    """A Transport backed by an in-process Firmware: the client's writes are fed
    straight in, and reply frames are buffered for read()."""

    def __init__(self, firmware: Firmware) -> None:
        self._fw = firmware
        self._rx = bytearray()

    def write(self, data: bytes) -> None:
        self._rx += self._fw.feed(data)

    def read(self, n: int) -> bytes:
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out
