"""USB serial transport for a Marlin-driven scanner motion controller.

The OpenScan host keeps ownership of the camera and lights. Rotor and
turntable moves are delegated to Marlin over one shared serial connection.
"""

import logging
import re
from threading import Condition, RLock
from time import sleep, time

logger = logging.getLogger(__name__)

_AXIS_RE = re.compile(r"^[XY]$")
_POSITION_RE = r"(?:^|\s){axis}:\s*(-?\d+(?:\.\d+)?)"
_SHARED_LOCK = RLock()
_SHARED = {}


def marlin_feedrate(steps_per_rotation: int, max_speed: float, max_feedrate: float) -> float:
    degrees_per_second = max_speed * 360.0 / max(int(steps_per_rotation), 1)
    return min(float(max_feedrate), max(30.0, degrees_per_second * 60.0))


def marlin_move_timeout(steps: int, steps_per_rotation: int, feedrate: float) -> float:
    degrees = abs(int(steps)) * 360.0 / max(int(steps_per_rotation), 1)
    move_seconds = degrees * 60.0 / max(float(feedrate), 1.0)
    return min(180.0, max(30.0, move_seconds * 3.0 + 15.0))


class MarlinMotion:
    """Serialize scanner moves over a single acknowledged Marlin connection."""

    def __init__(self, serial_factory=None, sleep_fn=sleep):
        self._serial_factory = serial_factory
        self._sleep = sleep_fn
        self._serial = None
        self._port = None
        self._baudrate = None
        self._move_lock = RLock()
        self._write_lock = RLock()
        self._pending_condition = Condition()
        self._pending = 0
        self._stop_requested = False
        self._users = 0
        self._shared_key = None

    @classmethod
    def shared(cls, port, baudrate):
        """Return the shared transport for a port so all motors use one link."""
        key = (str(port), int(baudrate))
        with _SHARED_LOCK:
            motion = _SHARED.get(key)
            if motion is None:
                motion = cls()
                _SHARED[key] = motion
            motion._users += 1
            motion._shared_key = key
        return motion

    def release(self):
        with _SHARED_LOCK:
            if self._shared_key is None:
                self.close()
                return
            self._users -= 1
            if self._users > 0:
                return
            _SHARED.pop(self._shared_key, None)
            self._shared_key = None
            self.close()

    @property
    def connected(self):
        return self._serial is not None and getattr(self._serial, "is_open", True)

    def _ensure_connection(self, port, baudrate):
        if self.connected and self._port == port and self._baudrate == baudrate:
            return
        self.close()
        started = time()

        if self._serial_factory is None:
            import serial

            serial_factory = serial.Serial
        else:
            serial_factory = self._serial_factory

        # Keep DTR low so opening the USB bridge does not reset the board.
        connection = serial_factory(
            port=None, baudrate=baudrate, timeout=0.2, write_timeout=3
        )
        connection.dtr = False
        connection.port = port
        connection.open()
        self._serial = connection
        self._port = port
        self._baudrate = baudrate
        logger.debug("Marlin port %s opened after %.2fs", port, time() - started)

        self._sleep(2)
        if hasattr(connection, "reset_input_buffer"):
            connection.reset_input_buffer()
        logger.debug("Marlin port %s ready after %.2fs", port, time() - started)

        self._send("M115")
        try:
            self._wait_for_ok(timeout=8)
        except Exception:
            self.close()
            raise
        logger.debug("Marlin handshake done after %.2fs", time() - started)

    def _send(self, command):
        if self._serial is None:
            raise RuntimeError("Marlin serial connection is not open")
        with self._write_lock:
            with self._pending_condition:
                self._pending += 1
            try:
                self._serial.write((command + "\n").encode("ascii"))
                self._serial.flush()
            except Exception:
                with self._pending_condition:
                    self._pending = max(0, self._pending - 1)
                    self._pending_condition.notify_all()
                raise

    def _wait_for_ok(self, timeout):
        deadline = time() + timeout
        response = []
        while True:
            with self._pending_condition:
                if self._pending == 0:
                    return response
            if time() >= deadline:
                with self._pending_condition:
                    self._pending = 0
                    self._pending_condition.notify_all()
                raise TimeoutError("Timed out waiting for Marlin response")
            raw = self._serial.readline()
            if not raw:
                continue
            for line in raw.decode("ascii", errors="replace").splitlines():
                line = line.strip()
                if not line:
                    continue
                response.append(line)
                lowered = line.lower()
                if lowered.startswith("error:") or lowered.startswith("!!"):
                    with self._pending_condition:
                        self._pending = 0
                        self._pending_condition.notify_all()
                    raise RuntimeError("Marlin rejected the move: {}".format(line))
                if lowered == "ok" or lowered.startswith("ok "):
                    with self._pending_condition:
                        self._pending = max(0, self._pending - 1)
                        self._pending_condition.notify_all()

    def _command(self, command, timeout):
        self._send(command)
        return self._wait_for_ok(timeout)

    def request_stop(self):
        self._stop_requested = True
        if self._serial is None:
            return
        try:
            self._send("M410")
        except Exception:
            pass

    def status(self, port, baudrate):
        with self._move_lock:
            self._ensure_connection(port, baudrate)
            return self._command("M115", timeout=8)

    def move_steps(
        self, axis, steps, steps_per_rotation, feedrate, port, baudrate, timeout=180
    ):
        axis = str(axis).upper()
        if not _AXIS_RE.match(axis):
            raise ValueError("Marlin scanner axes must be X or Y")
        steps = int(steps)
        steps_per_rotation = int(steps_per_rotation)
        feedrate = float(feedrate)
        if steps_per_rotation <= 0:
            raise ValueError("steps_per_rotation must be positive")
        if feedrate <= 0:
            raise ValueError("feedrate must be positive")
        if steps == 0:
            return 0

        angle = steps * 360.0 / steps_per_rotation
        steps_per_degree = steps_per_rotation / 360.0

        with self._move_lock:
            self._ensure_connection(port, baudrate)
            self._stop_requested = False
            self._command(
                "M92 {} {:.5f}".format(axis, steps_per_degree), timeout=5
            )
            self._command("G90", timeout=5)
            self._command("G92 {}0".format(axis), timeout=5)
            self._command("G91", timeout=5)
            self._command(
                "G1 {}{:.5f} F{:.1f}".format(axis, angle, feedrate), timeout=10
            )
            self._command("M400", timeout=timeout)
            executed_steps = abs(steps)
            if self._stop_requested:
                executed_steps = self._read_executed_steps(axis, steps_per_rotation)
            try:
                self._command("G90", timeout=5)
                self._command("G92 {}0".format(axis), timeout=5)
            except Exception:
                logger.warning(
                    "Failed to reset Marlin axis %s coordinates after move",
                    axis,
                    exc_info=True,
                )
            return executed_steps

    def _read_executed_steps(self, axis, steps_per_rotation):
        response = self._command("M114", timeout=5)
        match = re.search(
            _POSITION_RE.format(axis=axis), " ".join(response), re.IGNORECASE
        )
        if match is None:
            return 0
        angle = float(match.group(1))
        return int(round(angle * steps_per_rotation / 360.0))

    def close(self):
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
                self._port = None
                self._baudrate = None
                with self._pending_condition:
                    self._pending = 0
                    self._pending_condition.notify_all()
