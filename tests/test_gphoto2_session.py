import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from openscan_firmware.controllers.hardware.cameras.gphoto2.session import GPhoto2Session


class FakeCameraFile:
    def __init__(self, payload: bytes):
        self._payload = payload

    def get_data_and_size(self):
        return self._payload


class FakeCamera:
    def __init__(self, capture_error=None, trigger_error=None, payload=b"jpeg"):
        self.capture_error = capture_error
        self.trigger_error = trigger_error
        self.payload = payload
        self.trigger_calls = 0
        self.exit_calls = 0

    def capture(self, capture_type):
        if self.capture_error is not None:
            raise self.capture_error
        return SimpleNamespace(folder="store_00010001/DCIM/100CANON", name="IMG_0001.JPG")

    def trigger_capture(self):
        self.trigger_calls += 1
        if self.trigger_error is not None:
            raise self.trigger_error

    def file_get(self, folder, name, file_type):
        return FakeCameraFile(self.payload)

    def exit(self):
        self.exit_calls += 1


class GPhoto2SessionCaptureTests(unittest.TestCase):
    def test_capture_reconnects_after_unspecified_error(self):
        session = GPhoto2Session(camera_path="usb:001,005")
        failing = FakeCamera(
            capture_error=RuntimeError("[-1] Unspecified error"),
            trigger_error=RuntimeError("[-1] Unspecified error"),
        )
        recovered = FakeCamera(payload=b"recovered-jpeg")
        session._camera = failing
        session._capture_retry_delay_s = 0

        fallback = MagicMock(side_effect=RuntimeError("[-1] Unspecified error"))
        with patch.object(
            session,
            "ensure_connected",
            side_effect=[failing, recovered],
        ), patch.object(
            session,
            "_trigger_capture_and_wait_for_file",
            fallback,
        ):
            payload, metadata = session.capture_image()

        self.assertEqual(payload, b"recovered-jpeg")
        self.assertEqual(metadata["capture_name"], "IMG_0001.JPG")
        self.assertEqual(fallback.call_count, 1)
        self.assertEqual(failing.exit_calls, 1)

    def test_capture_reraises_after_all_attempts_fail(self):
        session = GPhoto2Session(camera_path="usb:001,005")
        failing = FakeCamera(capture_error=RuntimeError("[-1] Unspecified error"))
        session._camera = failing
        session._capture_retry_attempts = 2
        session._capture_retry_delay_s = 0

        with patch.object(
            session,
            "ensure_connected",
            return_value=failing,
        ), patch.object(
            session,
            "_trigger_capture_and_wait_for_file",
            side_effect=RuntimeError("[-1] Unspecified error"),
        ):
            with self.assertRaises(RuntimeError):
                session.capture_image()

        self.assertEqual(failing.exit_calls, 1)


if __name__ == "__main__":
    unittest.main()
