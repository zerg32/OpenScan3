import asyncio
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import AsyncMock, Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from openscan_firmware.controllers.services.tasks.core.scan_task import ScanTask
from openscan_firmware.models.paths import PolarPoint3D
from openscan_firmware.models.task import TaskStatus


class ScanTaskSkipTests(unittest.IsolatedAsyncioTestCase):
    async def test_capture_error_is_skipped_and_scan_completes(self):
        task = object.__new__(ScanTask)
        task._task_model = SimpleNamespace(result=None)
        task._failed_captures = []
        task.is_cancelled = Mock(return_value=False)
        task.wait_for_pause = AsyncMock()

        points = [
            PolarPoint3D(theta=1.0, fi=1.0, r=1.0),
            PolarPoint3D(theta=2.0, fi=2.0, r=1.0),
        ]
        task._ctx = SimpleNamespace(
            path_dict={points[0]: 0, points[1]: 1},
            scan=SimpleNamespace(
                index=1,
                project_name="test",
                status=None,
                duration=0.0,
                current_step=0,
                settings=SimpleNamespace(pause_before_capture_ms=0),
            ),
            focus_context=None,
            camera_controller=SimpleNamespace(),
            project_manager=SimpleNamespace(save_scan_state=AsyncMock()),
        )

        capture = AsyncMock(side_effect=[RuntimeError("camera failed"), None])
        task._capture_photos_at_position = capture

        with patch(
            "openscan_firmware.controllers.hardware.motors.move_to_point",
            new=AsyncMock(),
        ):
            progress = [update async for update in task._execute_scan_loop(0, 2)]

        self.assertEqual(capture.await_count, 2)
        self.assertEqual(task._failed_captures, [0])
        self.assertEqual(task._ctx.scan.status, TaskStatus.COMPLETED)
        self.assertIn("1 skipped photo(s)", task._task_model.result)
        self.assertEqual(len(progress), 3)
        self.assertIn("1 skipped photo(s)", progress[-1].message)


if __name__ == "__main__":
    unittest.main()
