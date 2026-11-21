"""The cube-hold recorder, run against a stand-in SDK and a stand-in clock. Authored-by: CPCS.

No robot, no network. It checks that hardware/hold_cube.py asks for no more than reserve control, renews the
cube connection inside the SDK subscription's 60 s, reads both batteries on schedule, records cube
events, lets the cube go at the end, and stops cleanly. It says nothing about a real robot or cube.
"""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional
from unittest import mock

import _common  # noqa: F401  (puts the skill on the path)
from test_recorder import FakeClock

SCRIPT = Path(__file__).resolve().parent.parent / "hardware" / "hold_cube.py"
ENV_KEYS = ("VECTOR_SERIAL", "VECTOR_IP", "HOLD_S", "RENEW_S", "BATTERY_S", "NOTE")
SERIAL = "0a1b2c3d"
EVENT_NAMES = ("object_tapped", "object_moved", "object_stopped_moving", "object_up_axis_changed",
               "object_connection_state")


class HookClock(FakeClock):
    """A clock that runs a callback the first time it passes a given time."""

    def __init__(self) -> None:
        super().__init__()
        self.hooks: List[List[Any]] = []

    def sleep(self, seconds: float) -> None:
        super().sleep(seconds)
        for hook in self.hooks:
            if not hook[2] and self.now >= hook[0]:
                hook[2] = True
                hook[1]()


class FakeRobot:
    def __init__(self, clock: HookClock, renew_error_at: Optional[int] = None, volts: float = 4.08) -> None:
        self.clock, self.calls, self.handlers, self.volts = clock, [], {}, volts
        self.renewals, self.renew_error_at = 0, renew_error_at
        self.events = types.SimpleNamespace(subscribe=self._subscribe, unsubscribe=self._unsubscribe)
        self.world = types.SimpleNamespace(connect_cube=self._connect_cube,
                                           disconnect_cube=lambda: self.calls.append(("release", clock.now)))

    def _subscribe(self, handler: Callable, event: str) -> None:
        self.handlers[event] = handler

    def _unsubscribe(self, handler: Callable, event: str) -> None:
        self.handlers.pop(event, None)

    def _connect_cube(self) -> Any:
        self.renewals += 1
        self.calls.append(("renew", self.clock.now))
        if self.renew_error_at == self.renewals:
            raise TimeoutError("no answer")
        return types.SimpleNamespace(success=True, factory_id="02:00:00:00:00:01")

    def connect(self, timeout: float = 0.0) -> None:
        self.calls.append(("connect", self.clock.now))

    def disconnect(self) -> None:
        self.calls.append(("disconnect", self.clock.now))

    def get_battery_state(self) -> Any:
        self.calls.append(("battery", self.clock.now))
        cube = types.SimpleNamespace(level=1, factory_id="02:00:00:00:00:01", battery_volts=1.55,
                                     time_since_last_reading_sec=12.0)
        return types.SimpleNamespace(battery_volts=self.volts, is_charging=True, is_on_charger_platform=True, cube_battery=cube)

    def tap(self) -> None:
        self.handlers["object_tapped"](self, "object_tapped", types.SimpleNamespace(object_id=1, timestamp=12345))


def run(env: Dict[str, str], robot: FakeRobot, lock_held: bool = False,
        stop_at: Optional[float] = None) -> Dict[str, Any]:
    made: Dict[str, Any] = {}

    def make(*args: Any, **kwargs: Any) -> FakeRobot:
        made["args"], made["kwargs"] = args, kwargs
        return robot

    top = types.ModuleType("anki_vector")
    top.Robot = make
    events = types.ModuleType("anki_vector.events")
    events.Events = types.SimpleNamespace(**{name: name for name in EVENT_NAMES})
    conn = types.ModuleType("anki_vector.connection")
    conn.ControlPriorityLevel = types.SimpleNamespace(RESERVE_CONTROL="reserve", DEFAULT_PRIORITY="default",
                                                      OVERRIDE_BEHAVIORS_PRIORITY="override")
    out = io.StringIO()
    with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(os.environ, env), \
            mock.patch.dict(sys.modules, {"anki_vector": top, "anki_vector.events": events, "anki_vector.connection": conn}):
        for key in ENV_KEYS:
            if key not in env:
                os.environ.pop(key, None)
        spec = importlib.util.spec_from_file_location("hold_cube_under_test", SCRIPT)
        assert spec is not None and spec.loader is not None, "cannot load the script"
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.time, module.T0, module.LOCK = robot.clock, 0.0, str(Path(tmp) / "lock")
        if lock_held:
            Path(module.LOCK).write_text("held")
        if stop_at is not None:
            robot.clock.hooks.append([stop_at, lambda: module.STATE.update(stop="SIGTERM"), False])
        with contextlib.redirect_stdout(out):
            code = module.main()
        lock_left = Path(module.LOCK).exists() and not lock_held
    rows = [json.loads(line) for line in out.getvalue().splitlines()]
    return {"rows": rows, "code": code, "made": made, "lock_left": lock_left}


def kinds(rows: List[Dict[str, Any]], kind: str) -> List[Dict[str, Any]]:
    return [row for row in rows if row["k"] == kind]


class HoldCube(unittest.TestCase):
    def test_reserve_control_renews_inside_the_expiry_reads_batteries_and_lets_the_cube_go(self) -> None:
        robot = FakeRobot(HookClock())
        got = run({"VECTOR_SERIAL": SERIAL, "HOLD_S": "600", "RENEW_S": "30", "BATTERY_S": "60"}, robot)
        self.assertEqual(got["code"], 0)
        self.assertEqual(got["made"]["kwargs"]["behavior_control_level"], "reserve")
        self.assertEqual(sorted(robot.handlers), [], "every event handler is removed at the end")
        renew_times = [t for name, t in robot.calls if name == "renew"]
        self.assertEqual(len(renew_times), 20)
        gaps = [b - a for a, b in zip(renew_times, renew_times[1:])]
        self.assertLessEqual(max(gaps), 30.5)
        self.assertLess(max(gaps), 60.0, "a renewal must come before the 60 s subscription expires")
        batteries = kinds(got["rows"], "b")
        self.assertEqual(len(batteries), 11)  # ten in the hold, one at the end
        self.assertEqual(batteries[0]["cube_volts"], 1.55)
        self.assertEqual(batteries[0]["cube_id"], "02:00:00:00:00:01")
        names = [name for name, _ in robot.calls]
        self.assertIn("release", names, "the cube was never let go")
        self.assertLess(names.index("release"), names.index("disconnect"))
        self.assertFalse(got["lock_left"])

    def test_it_subscribes_to_every_cube_event_and_records_a_tap(self) -> None:
        clock = HookClock()
        robot = FakeRobot(clock)
        clock.hooks.append([100.0, robot.tap, False])
        got = run({"VECTOR_SERIAL": SERIAL, "HOLD_S": "120"}, robot)
        events = kinds(got["rows"], "e")
        self.assertEqual(len(events), 1)
        self.assertEqual((events[0]["event"], events[0]["object_id"], events[0]["timestamp"]), ("object_tapped", 1, 12345))
        self.assertAlmostEqual(events[0]["t"], 100.0, delta=0.6)

    def test_a_failed_renewal_is_recorded_and_the_hold_goes_on(self) -> None:
        robot = FakeRobot(HookClock(), renew_error_at=2)
        got = run({"VECTOR_SERIAL": SERIAL, "HOLD_S": "120", "RENEW_S": "30"}, robot)
        renewals = kinds(got["rows"], "r")
        self.assertEqual(len(renewals), 4)
        self.assertIn("TimeoutError", renewals[1].get("error", ""))
        self.assertTrue(renewals[2].get("success"))

    def test_a_stop_signal_ends_the_hold_early_and_still_releases_the_cube(self) -> None:
        robot = FakeRobot(HookClock())
        got = run({"VECTOR_SERIAL": SERIAL, "HOLD_S": "3600"}, robot, stop_at=95.0)
        held = [r for r in got["rows"] if r.get("name") == "held"][0]
        self.assertEqual(held["stopped_by"], "SIGTERM")
        self.assertLess(held["seconds"], 100.0)
        self.assertIn("release", [name for name, _ in robot.calls])

    def test_a_low_robot_battery_ends_the_hold_and_lets_the_cube_go(self) -> None:
        robot = FakeRobot(HookClock(), volts=3.45)
        got = run({"VECTOR_SERIAL": SERIAL, "HOLD_S": "3600"}, robot)
        held = [r for r in got["rows"] if r.get("name") == "held"][0]
        self.assertEqual(held["stopped_by"], "low_battery")
        self.assertLess(held["seconds"], 5.0)
        self.assertIn("release", [name for name, _ in robot.calls])

    def test_a_held_lock_means_no_connection(self) -> None:
        robot = FakeRobot(HookClock())
        got = run({"VECTOR_SERIAL": SERIAL}, robot, lock_held=True)
        self.assertEqual(got["code"], 3)
        self.assertEqual(robot.calls, [])

    def test_settings_out_of_range_are_refused_before_anything_happens(self) -> None:
        for env in ({"VECTOR_SERIAL": SERIAL, "HOLD_S": "30"}, {"VECTOR_SERIAL": SERIAL, "HOLD_S": "30000"},
                    {"VECTOR_SERIAL": SERIAL, "RENEW_S": "55"}, {"VECTOR_SERIAL": SERIAL, "RENEW_S": "5"},
                    {"VECTOR_SERIAL": SERIAL, "BATTERY_S": "10"}, {"VECTOR_SERIAL": "nope"}):
            robot = FakeRobot(HookClock())
            with self.assertRaises(AssertionError, msg=str(env)):
                run(env, robot)
            self.assertEqual(robot.calls, [], str(env))


if __name__ == "__main__":
    unittest.main()
