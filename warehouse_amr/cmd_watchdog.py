"""Receiver-side /cmd_vel watchdog: stop the robot when commands stop arriving.

Two things in the simulator *remember* the last command. A ROS 2 subscriber
keeps its last message, and a PhysX joint drive keeps its last target velocity
until something overwrites it. So if the node sending /cmd_vel dies without
saying goodbye -- a crash, ``taskkill /F``, a dropped network -- the robot
carries on at the last speed forever. Every real robot guards against this
with a command timeout in its base controller; this is that guard.

The rules it follows, each of which is tested in ``tests/test_cmd_watchdog.py``:

* It lives on the *receiving* side. Only the receiver can notice silence.
* It times the last *valid message received*, not changes in value: a stream
  of identical commands is still a heartbeat.
* Silence means an active zero, because nothing else will write one.
* Fail-safe start: until the first command arrives, the output is zero.
* A clock that runs backwards (timeline Stop -> Play resets sim time) drops
  the stored command instead of treating it as fresh.
* NaN/inf commands are rejected and do not keep the watchdog alive.

It is pure Python with no Isaac or ROS imports. The caller passes in the
time, which keeps it deterministic and testable without a simulator.
"""

from __future__ import annotations

import math


class CmdVelWatchdog:
    def __init__(self, timeout, max_linear, max_angular, require_zero_to_rearm=False, log=print):
        self.timeout = timeout
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.require_zero_to_rearm = require_zero_to_rearm
        self.log = log
        self._cmd = (0.0, 0.0)
        self._last_rx = None        # never heard anything -> stopped (fail-safe start)
        self._armed = True
        self.tripped = True
        self.trips = 0

    def on_command(self, v, w, now) -> bool:
        """Call from the subscription callback. Only valid messages refresh the timer."""
        if not (math.isfinite(v) and math.isfinite(w)):
            return False
        v = max(-self.max_linear, min(self.max_linear, v))
        w = max(-self.max_angular, min(self.max_angular, w))
        if not self._armed:
            if v != 0.0 or w != 0.0:
                return False        # latched: ignore motion until a stop command arrives
            self._armed = True
            self.log("[watchdog] re-armed by a zero command")
        self._cmd = (v, w)
        self._last_rx = now
        return True

    def output(self, now):
        """Call once per control step; returns the (v, w) that is safe to apply."""
        if self._last_rx is not None:
            age = now - self._last_rx
            # age < 0: the clock went backwards (timeline Stop -> Play resets
            # sim time), so the stored command belongs to a previous run.
            if age < 0.0 or age > self.timeout:
                self._last_rx = None            # drop it for good: the trip fires once
                self._cmd = (0.0, 0.0)
                self.trips += 1
                if self.require_zero_to_rearm:
                    self._armed = False
                self.log("[watchdog] no fresh /cmd_vel -> STOP (age %.2fs)" % age)
        if self._last_rx is None:
            self.tripped = True
            return (0.0, 0.0)
        if self.tripped:
            self.log("[watchdog] receiving /cmd_vel")
        self.tripped = False
        return self._cmd

