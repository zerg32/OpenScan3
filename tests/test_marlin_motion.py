import sys
import unittest
from collections import deque
from pathlib import Path
from unittest.mock import patch

sys.path.insert(
    0,
    str(
        Path(__file__).resolve().parents[1]
        / "openscan_firmware"
        / "controllers"
        / "hardware"
    ),
)

from marlin_motion import MarlinMotion, marlin_feedrate, marlin_move_timeout


class FakeSerial:
    def __init__(self, **kwargs):
        self.commands = []
        self.is_open = False
        self.dtr = True
        self.port = kwargs.get("port")
        self._lines = deque()
        self.on_write = kwargs.get("on_write")

    def open(self):
        self.is_open = True

    def write(self, data):
        command = data.decode("ascii").strip()
        self.commands.append(command)
        if self.on_write is not None:
            self.on_write(command, self)
        else:
            self.respond(command)

    def respond(self, command):
        if command == "M114":
            self._lines.append(b"X:45.000 Y:0.000 Z:0.000 E:0.000\n")
        self._lines.append(b"ok\n")

    def flush(self):
        pass

    def readline(self):
        if self._lines:
            return self._lines.popleft()
        return b""

    def reset_input_buffer(self):
        self._lines.clear()

    def close(self):
        self.is_open = False


class MarlinMotionTests(unittest.TestCase):
    def make_controller(self, on_write=None):
        self.serial = None

        def serial_factory(**kwargs):
            self.serial = FakeSerial(**kwargs)
            self.serial.on_write = on_write
            return self.serial

        with patch("marlin_motion.sleep"):
            controller = MarlinMotion(
                serial_factory=serial_factory, sleep_fn=lambda _: None
            )
        return controller

    @patch("marlin_motion.sleep")
    def test_move_uses_relative_gcode_and_waits_for_completion(self, _sleep):
        controller = self.make_controller()

        executed = controller.move_steps(
            "X", 800, 3200, 600.0, "/dev/ttyUSB0", 115200
        )

        self.assertEqual(executed, 800)
        self.assertEqual(
            self.serial.commands,
            [
                "M115",
                "M92 X 8.88889",
                "G90",
                "G92 X0",
                "G91",
                "G1 X90.00000 F600.0",
                "M400",
                "G90",
                "G92 X0",
            ],
        )

    @patch("marlin_motion.sleep")
    def test_connection_is_reused_for_multiple_moves(self, _sleep):
        controller = self.make_controller()

        controller.move_steps("X", 1, 3200, 600.0, "/dev/ttyUSB0", 115200)
        connection = self.serial
        controller.move_steps("Y", 1, 3200, 600.0, "/dev/ttyUSB0", 115200)

        self.assertIs(controller._serial, connection)
        self.assertEqual(self.serial.commands.count("M115"), 1)

    @patch("marlin_motion.sleep")
    def test_stop_sends_quickstop_and_reads_executed_position(self, _sleep):
        controller = None

        def on_write(command, serial):
            if command == "M400":
                controller.request_stop()
            serial.respond(command)

        controller = self.make_controller(on_write=on_write)
        executed = controller.move_steps(
            "X", 800, 3200, 600.0, "/dev/ttyUSB0", 115200
        )

        self.assertEqual(executed, 400)
        self.assertIn("M410", self.serial.commands)
        self.assertIn("M114", self.serial.commands)

    def test_rejects_non_scanner_axis(self):
        controller = self.make_controller()
        with self.assertRaises(ValueError):
            controller.move_steps("Z", 1, 3200, 600.0, "/dev/ttyUSB0", 115200)

    def test_shared_transport_reused_for_same_port(self):
        first = MarlinMotion.shared("/dev/ttyUSB9", 115200)
        second = MarlinMotion.shared("/dev/ttyUSB9", 115200)
        self.assertIs(first, second)

        first.release()
        second.release()
        third = MarlinMotion.shared("/dev/ttyUSB9", 115200)
        self.assertIsNot(first, third)
        third.release()

    @patch("marlin_motion.sleep")
    def test_timeout_keeps_connection_and_recovers(self, _sleep):
        state = {"skip_next_m400": True}

        def on_write(command, serial):
            if command == "M400" and state["skip_next_m400"]:
                state["skip_next_m400"] = False
                return
            serial.respond(command)

        controller = self.make_controller(on_write=on_write)

        with self.assertRaises(TimeoutError):
            controller.move_steps(
                "X", 100, 3200, 600.0, "/dev/ttyUSB0", 115200, timeout=0.2
            )

        connection = self.serial
        self.assertTrue(controller.connected)
        self.assertIs(controller._serial, connection)
        self.assertEqual(controller._pending, 0)

        executed = controller.move_steps(
            "X", 100, 3200, 600.0, "/dev/ttyUSB0", 115200
        )
        self.assertEqual(executed, 100)
        self.assertIs(controller._serial, connection)


class MarlinTimingTests(unittest.TestCase):
    def test_feedrate_is_capped_by_marlin_limit(self):
        self.assertEqual(marlin_feedrate(3200, 5000, 600.0), 600.0)

    def test_timeout_uses_commanded_feedrate(self):
        timeout = marlin_move_timeout(1467, 3200, 600.0)
        self.assertAlmostEqual(timeout, 64.51125, places=4)
        self.assertGreater(timeout, 15.0)

    def test_short_move_has_minimum_timeout(self):
        self.assertEqual(marlin_move_timeout(10, 3200, 600.0), 30.0)


if __name__ == "__main__":
    unittest.main()
