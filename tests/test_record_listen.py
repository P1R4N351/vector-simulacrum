"""The listen-only recorder, run against a stand-in SDK, a stand-in pod and a stand-in clock..

No robot, no network, no MuJoCo. It checks that hardware/record_listen.py never asks for behaviour
control and never commands anything, that it records for the time it is given, and that it lifts the
pod's quiet mode only if it was on and always puts it back. It says nothing about a real robot.
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
from typing import Any, Dict, List, Optional
from unittest import mock

import _common  # noqa: F401  (puts the skill on the path)
from test_recorder import FakeClock

SCRIPT = Path(__file__).resolve().parent.parent / "hardware" / "record_listen.py"
ENV_KEYS = ("VECTOR_SERIAL", "VECTOR_IP", "LISTEN_S", "QUIET_LIFT", "POD_URL", "NOTE")
SERIAL = "0a1b2c3d"


class FakeRobot:
    """An SDK robot that only streams state; any attribute a command would need is missing."""

    def __init__(self, clock: FakeClock, connect_error: Optional[Exception] = None) -> None:
        self.clock, self.calls, self.connect_error = clock, [], connect_error
        self.events = types.SimpleNamespace(subscribe=lambda *a: self.calls.append("subscribe"),
                                            unsubscribe=lambda *a: self.calls.append("unsubscribe"))

    def connect(self, timeout: float = 0.0) -> None:
        self.calls.append("connect")
        if self.connect_error is not None:
            raise self.connect_error

    def disconnect(self) -> None:
        self.calls.append("disconnect")

    def get_battery_state(self) -> Any:
        self.calls.append("battery")
        return types.SimpleNamespace(battery_volts=4.07, battery_level=2, is_charging=False, is_on_charger_platform=False)


class FakePod:
    """wire-pod's quiet_mode endpoint: remembers the setting and every request."""

    def __init__(self, enabled: bool) -> None:
        self.enabled, self.requests = enabled, []

    def urlopen(self, req: Any, timeout: float = 0.0) -> Any:
        url, method = req.full_url, req.get_method()
        self.requests.append((method, url))
        if method == "POST":
            self.enabled = "enable=true" in url
        body = json.dumps({"enabled": self.enabled, "active": self.enabled, "holding_reserve": self.enabled,
                           "muted": self.enabled}).encode()
        return contextlib.nullcontext(types.SimpleNamespace(read=lambda: body))


def run(env: Dict[str, str], robot: FakeRobot, pod: FakePod, lock_held: bool = False) -> Dict[str, Any]:
    made: Dict[str, Any] = {}

    def make(*args: Any, **kwargs: Any) -> FakeRobot:
        made["args"], made["kwargs"] = args, kwargs
        return robot

    top = types.ModuleType("anki_vector")
    top.Robot = make
    events = types.ModuleType("anki_vector.events")
    events.Events = types.SimpleNamespace(robot_state="robot_state")
    out = io.StringIO()
    with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(os.environ, env), \
            mock.patch.dict(sys.modules, {"anki_vector": top, "anki_vector.events": events}):
        for key in ENV_KEYS:
            if key not in env:
                os.environ.pop(key, None)
        spec = importlib.util.spec_from_file_location("record_listen_under_test", SCRIPT)
        assert spec is not None and spec.loader is not None, "cannot load the script"
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.time, module.T0, module.LOCK = robot.clock, 0.0, str(Path(tmp) / "lock")
        if lock_held:
            Path(module.LOCK).write_text("held")
        module.urllib.request.urlopen = pod.urlopen
        error = None
        with contextlib.redirect_stdout(out):
            try:
                code = module.main()
            except Exception as exc:  # the connect error a test injects; the rows written before it still count
                code, error = None, exc
        lock_left = Path(module.LOCK).exists() and not lock_held
    rows = [json.loads(line) for line in out.getvalue().splitlines()]
    return {"rows": rows, "code": code, "error": error, "made": made, "lock_left": lock_left}


def named(rows: List[Dict[str, Any]], name: str) -> List[Dict[str, Any]]:
    return [row for row in rows if row.get("name") == name]


class ListenOnly(unittest.TestCase):
    def test_it_asks_for_no_control_records_for_the_time_given_and_commands_nothing(self) -> None:
        robot = FakeRobot(FakeClock())
        got = run({"VECTOR_SERIAL": SERIAL, "LISTEN_S": "120"}, robot, FakePod(enabled=False))
        self.assertEqual(got["code"], 0)
        self.assertIsNone(got["made"]["kwargs"]["behavior_control_level"])
        self.assertEqual(got["made"]["args"], (SERIAL,))
        self.assertEqual(robot.calls, ["connect", "subscribe", "battery", "battery", "unsubscribe", "disconnect"])
        start, stop = named(got["rows"], "listening")[0]["t"], named(got["rows"], "listened")[0]["t"]
        self.assertAlmostEqual(stop - start, 120.0, delta=0.25)
        self.assertEqual(len([r for r in got["rows"] if r["k"] == "b"]), 2)
        self.assertFalse(got["lock_left"])

    def test_quiet_mode_on_is_lifted_then_put_back(self) -> None:
        pod = FakePod(enabled=True)
        got = run({"VECTOR_SERIAL": SERIAL, "LISTEN_S": "10"}, FakeRobot(FakeClock()), pod)
        self.assertEqual([m for m, _ in pod.requests], ["GET", "POST", "POST"])
        self.assertIn("enable=false", pod.requests[1][1])
        self.assertIn("enable=true", pod.requests[2][1])
        self.assertTrue(pod.enabled)
        lifted_at = named(got["rows"], "quiet")[1]["t"]
        self.assertLessEqual(lifted_at, named(got["rows"], "connect_to")[0]["t"])

    def test_quiet_mode_off_is_left_alone(self) -> None:
        pod = FakePod(enabled=False)
        run({"VECTOR_SERIAL": SERIAL, "LISTEN_S": "10"}, FakeRobot(FakeClock()), pod)
        self.assertEqual([m for m, _ in pod.requests], ["GET"])
        self.assertFalse(pod.enabled)

    def test_quiet_mode_is_put_back_and_the_lock_released_when_the_connection_fails(self) -> None:
        pod = FakePod(enabled=True)
        got = run({"VECTOR_SERIAL": SERIAL, "LISTEN_S": "10"}, FakeRobot(FakeClock(), connect_error=OSError("refused")), pod)
        self.assertIsInstance(got["error"], OSError)
        self.assertTrue(pod.enabled)
        self.assertFalse(got["lock_left"])

    def test_quiet_lift_off_never_calls_the_pod(self) -> None:
        pod = FakePod(enabled=True)
        run({"VECTOR_SERIAL": SERIAL, "LISTEN_S": "10", "QUIET_LIFT": "0"}, FakeRobot(FakeClock()), pod)
        self.assertEqual(pod.requests, [])

    def test_a_held_lock_means_no_connection_and_no_pod_call(self) -> None:
        robot, pod = FakeRobot(FakeClock()), FakePod(enabled=True)
        got = run({"VECTOR_SERIAL": SERIAL, "LISTEN_S": "10"}, robot, pod, lock_held=True)
        self.assertEqual(got["code"], 3)
        self.assertEqual(robot.calls, [])
        self.assertEqual(pod.requests, [])

    def test_settings_out_of_range_are_refused_before_anything_happens(self) -> None:
        for env in ({"VECTOR_SERIAL": SERIAL, "LISTEN_S": "5"}, {"VECTOR_SERIAL": SERIAL, "LISTEN_S": "301"},
                    {"VECTOR_SERIAL": "xyz"}, {"VECTOR_SERIAL": SERIAL, "QUIET_LIFT": "yes"},
                    {"VECTOR_SERIAL": SERIAL, "VECTOR_IP": "10.3.142"}):
            robot, pod = FakeRobot(FakeClock()), FakePod(enabled=True)
            with self.assertRaises(AssertionError, msg=str(env)):
                run(env, robot, pod)
            self.assertEqual((robot.calls, pod.requests), ([], []), str(env))


if __name__ == "__main__":
    unittest.main()
