import sys
import unittest
from pathlib import Path

import cv2
import numpy as np
import piexif

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from openscan_firmware.config.camera import CameraSettings
from openscan_firmware.controllers.hardware.cameras.gphoto2.controller import GPhoto2Controller


class GPhoto2ControllerOrientationTests(unittest.TestCase):
    def test_embed_orientation_flag_writes_exif_to_memory(self):
        _, encoded = cv2.imencode(".jpg", np.zeros((16, 16, 3), dtype=np.uint8))
        jpeg_bytes = encoded.tobytes()
        controller = object.__new__(GPhoto2Controller)
        controller.settings = CameraSettings(orientation_flag=1)

        result = controller._embed_orientation_flag(jpeg_bytes)

        exif = piexif.load(result)
        self.assertEqual(exif["0th"][piexif.ImageIFD.Orientation], 1)


if __name__ == "__main__":
    unittest.main()
