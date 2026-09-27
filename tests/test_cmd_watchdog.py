"""Unit tests for the /cmd_vel watchdog. Plain Python -- no Isaac Sim, no ROS 2.

    python -m unittest discover -s tests          (any Python 3.8+)
    C:\\isaacsim\\python.bat -m unittest discover -s tests
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from warehouse_amr.cmd_watchdog import CmdVelWatchdog  # noqa: E402


def make(**kw):
    kw.setdefault("log", lambda *_: None)
    return CmdVelWatchdog(timeout=0.5, max_linear=1.0, max_angular=1.5, **kw)


class Watchdog(unittest.TestCase):
    def test_starts_stopped(self):
        wd = make()
        self.assertEqual(wd.output(0.0), (0.0, 0.0))
        self.assertTrue(wd.tripped)
        self.assertEqual(wd.trips, 0)

    def test_passes_fresh_commands(self):
        wd = make()
        wd.on_command(0.3, 0.3, now=1.0)
        self.assertEqual(wd.output(1.2), (0.3, 0.3))
        self.assertFalse(wd.tripped)

    def test_stops_after_timeout_and_counts_once(self):
        wd = make()
        wd.on_command(0.3, 0.0, now=1.0)
        self.assertEqual(wd.output(1.5), (0.3, 0.0))      # exactly at the limit: still fresh
        self.assertEqual(wd.output(1.51), (0.0, 0.0))
        self.assertEqual(wd.output(5.0), (0.0, 0.0))
        self.assertEqual(wd.trips, 1)

    def test_steady_stream_never_trips(self):
        # 20 Hz publisher and 30 Hz control loop on one merged timeline.
        wd = make()
        pubs = [k * 0.05 for k in range(200)]
        ctrl = [k / 30 for k in range(1, 300)]
        events = sorted([(t, "pub") for t in pubs] + [(t, "ctl") for t in ctrl])
        for t, kind in events:
            if kind == "pub":
                wd.on_command(0.5, 0.0, t)
            else:
                self.assertEqual(wd.output(t), (0.5, 0.0))
        self.assertEqual(wd.trips, 0)

    def test_resumes_on_new_message(self):
        wd = make()
        wd.on_command(0.3, 0.0, 1.0)
        wd.output(2.0)
        wd.on_command(0.2, 0.0, 2.1)
        self.assertEqual(wd.output(2.1), (0.2, 0.0))

    def test_rearm_requires_zero(self):
        wd = make(require_zero_to_rearm=True)
        wd.on_command(0.3, 0.0, 1.0)
        wd.output(2.0)                                     # trip
        self.assertFalse(wd.on_command(0.3, 0.0, 2.1))     # stale intent is ignored
        self.assertEqual(wd.output(2.1), (0.0, 0.0))
        self.assertTrue(wd.on_command(0.0, 0.0, 2.2))      # explicit stop re-arms
        self.assertTrue(wd.on_command(0.3, 0.0, 2.3))
        self.assertEqual(wd.output(2.3), (0.3, 0.0))

    def test_rejects_non_finite_without_refreshing(self):
        wd = make()
        wd.on_command(0.3, 0.0, 1.0)
        self.assertFalse(wd.on_command(math.nan, 0.0, 1.4))
        self.assertFalse(wd.on_command(0.0, math.inf, 1.4))
        self.assertEqual(wd.output(1.45), (0.3, 0.0))
        self.assertEqual(wd.output(1.6), (0.0, 0.0))       # garbage did not keep it alive

    def test_clamps(self):
        wd = make()
        wd.on_command(9.0, -9.0, 0.0)
        self.assertEqual(wd.output(0.0), (1.0, -1.5))

    def test_clock_reset_does_not_revive_old_command(self):
        wd = make()
        wd.on_command(0.3, 0.0, 100.0)
        self.assertEqual(wd.output(100.1), (0.3, 0.0))
        self.assertEqual(wd.output(0.0), (0.0, 0.0))       # Stop -> Play: sim time back to 0
        self.assertEqual(wd.output(0.2), (0.0, 0.0))
        wd.on_command(0.1, 0.0, 0.3)
        self.assertEqual(wd.output(0.3), (0.1, 0.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
