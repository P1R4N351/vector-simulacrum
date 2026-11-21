"""The peer-capture script and its saver, against a stand-in SDK, pod and clock. Authored-by: CPCS (cpcs-66ffc23).

No robot, no network. It checks that hardware/capture_peers.py holds behaviour control and commands
nothing, lifts quiet mode only while control is held and always puts it back first, never connects while
Cryosleep is on or not known to be off, stops for each reason it names, and that hardware/save_capture.py
keeps exactly the bytes the robot side hashed. It says nothing about a real robot's camera.
"""

from __future__ import annotations

import base64
import contextlib
import hashlib
import importlib.util
import io
import json
import os
import sys
import tempfile
import types
import unittest
import urllib.error
from pathlib import Path
from typing import Any, Dict, List, Optional
from unittest import mock

import _common  # noqa: F401  (puts the skill on the path)
from test_recorder import FakeClock

HERE = Path(__file__).resolve().parent.parent
SCRIPT = HERE / "hardware" / "capture_peers.py"
SAVER = HERE / "hardware" / "save_capture.py"
ENV_KEYS = ("VECTOR_SERIAL", "VECTOR_IP", "IN_VIEW", "SCENE", "CAPTURE_S", "FPS", "STALL_S", "JPEG_QUALITY",
            "QUIET_LIFT", "POD_URL")
SERIAL = "0a1b2c3d"
BASE = {"VECTOR_SERIAL": SERIAL, "IN_VIEW": "none", "CAPTURE_S": "20", "FPS": "1"}


class FakeImage:
    """A PIL image stand-in: 640x360, and JPEG bytes that depend on the image id."""

    def __init__(self, image_id: int) -> None:
        self.size, self.image_id = (640, 360), image_id

    def save(self, buf: io.BytesIO, format: str = "", quality: int = 0) -> None:
        assert format == "JPEG", "only JPEG is asked for"
        buf.write(b"\xff\xd8" + ("frame %d q%d" % (self.image_id, quality)).encode() + b"\xff\xd9")


class FakeRobot:
    """A robot that streams a camera at 15 fps from t=0 until stall_at, holds control from granted_at
    until lost_at, and has no attribute a motion command would need."""

    def __init__(self, clock: FakeClock, log: List[str], granted_at: Optional[float] = 0.2,
                 lost_at: Optional[float] = None, stall_at: Optional[float] = None) -> None:
        self.clock, self.log, self.calls = clock, log, []
        self.granted_at, self.lost_at, self.stall_at = granted_at, lost_at, stall_at
        self.head_angle_rad, self.lift_height_mm = 0.05, 32.0
        self.status = types.SimpleNamespace(is_on_charger=True)
        granted = types.SimpleNamespace(is_set=self._granted)
        self.conn = types.SimpleNamespace(control_granted_event=granted)
        robot = self
        camera = type("FakeCamera", (), {"latest_image": property(lambda _: robot._latest()),
                                         "init_camera_feed": lambda _: robot._call("camera_on"),
                                         "close_camera_feed": lambda _: robot._call("camera_off")})
        self.camera = camera()

    def _call(self, name: str) -> None:
        self.calls.append(name)
        self.log.append("robot:" + name)

    def _granted(self) -> bool:
        now = self.clock.now
        if self.granted_at is None or now < self.granted_at:
            return False
        return self.lost_at is None or now < self.lost_at

    def _latest(self) -> Optional[Any]:
        if "camera_on" not in self.calls:
            return None
        now = self.clock.now if self.stall_at is None else min(self.clock.now, self.stall_at)
        image_id = int(now * 15)
        return types.SimpleNamespace(image_id=image_id, raw_image=FakeImage(image_id))

    def connect(self, timeout: float = 0.0) -> None:
        self._call("connect")

    def disconnect(self) -> None:
        self._call("disconnect")

    def get_battery_state(self) -> Any:
        self.calls.append("battery")
        return types.SimpleNamespace(battery_volts=4.07, battery_level=2, is_charging=True, is_on_charger_platform=True)


class FakePod:
    """wire-pod's quiet_mode and cryo_mute endpoints. cryo is "off", "on", "404" or "down"."""

    def __init__(self, log: List[str], quiet: bool = False, cryo: str = "off") -> None:
        self.log, self.quiet, self.cryo, self.requests = log, quiet, cryo, []

    def urlopen(self, req: Any, timeout: float = 0.0) -> Any:
        url, method = req.full_url, req.get_method()
        self.requests.append((method, url))
        if "/cryo_mute" in url:
            self.log.append("pod:cryo")
            if self.cryo == "404":
                raise urllib.error.HTTPError(url, 404, "not found", {}, None)
            if self.cryo == "down":
                raise urllib.error.URLError("connection refused")
            body = {"muted": self.cryo == "on", "pod_gated": self.cryo == "on", "robot_muted": self.cryo == "on"}
        else:
            if method == "POST":
                self.quiet = "enable=true" in url
                self.log.append("pod:quiet_" + ("on" if self.quiet else "off"))
            body = {"enabled": self.quiet, "active": self.quiet, "holding_reserve": self.quiet, "muted": self.quiet}
        data = json.dumps(body).encode()
        return contextlib.nullcontext(types.SimpleNamespace(read=lambda: data))


class StopAt(FakeClock):
    """A clock that raises the script's stop flag once time passes `at`."""

    def __init__(self, at: float) -> None:
        super().__init__()
        self.at, self.state = at, None

    def sleep(self, seconds: float) -> None:
        super().sleep(seconds)
        if self.state is not None and self.now >= self.at:
            self.state["stop"] = "SIGTERM"


def load(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None, "cannot load " + str(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(env: Dict[str, str], robot: FakeRobot, pod: FakePod, lock_held: bool = False) -> Dict[str, Any]:
    made: Dict[str, Any] = {}

    def make(*args: Any, **kwargs: Any) -> FakeRobot:
        made["args"], made["kwargs"] = args, kwargs
        return robot

    top = types.ModuleType("anki_vector")
    top.Robot = make
    conn = types.ModuleType("anki_vector.connection")
    conn.ControlPriorityLevel = types.SimpleNamespace(DEFAULT_PRIORITY="DEFAULT", RESERVE_CONTROL="RESERVE")
    out = io.StringIO()
    with tempfile.TemporaryDirectory() as tmp, mock.patch.dict(os.environ, env), \
            mock.patch.dict(sys.modules, {"anki_vector": top, "anki_vector.connection": conn}):
        for key in ENV_KEYS:
            if key not in env:
                os.environ.pop(key, None)
        module = load(SCRIPT, "capture_peers_under_test")
        module.time, module.T0, module.LOCK = robot.clock, 0.0, str(Path(tmp) / "lock")
        if isinstance(robot.clock, StopAt):
            robot.clock.state = module.STATE
        if lock_held:
            Path(module.LOCK).write_text("held")
        module.urllib.request.urlopen = pod.urlopen
        with contextlib.redirect_stdout(out):
            code = module.main()
        lock_left = Path(module.LOCK).exists() and not lock_held
    rows = [json.loads(line) for line in out.getvalue().splitlines()]
    return {"rows": rows, "code": code, "made": made, "lock_left": lock_left, "stream": out.getvalue()}


def new_robot(clock: Optional[FakeClock] = None, log: Optional[List[str]] = None, **kw: Any) -> FakeRobot:
    return FakeRobot(clock or FakeClock(), log if log is not None else [], **kw)


def named(rows: List[Dict[str, Any]], name: str) -> List[Dict[str, Any]]:
    return [row for row in rows if row.get("name") == name]


def frames(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return [row for row in rows if row["k"] == "f"]


class Capture(unittest.TestCase):
    def test_it_holds_default_control_commands_nothing_and_streams_one_new_frame_per_tick(self) -> None:
        robot = new_robot()
        got = run(BASE, robot, FakePod([]))
        self.assertEqual(got["code"], 0)
        self.assertEqual(got["made"]["kwargs"]["behavior_control_level"], "DEFAULT")
        self.assertEqual(robot.calls, ["connect", "camera_on", "battery", "battery", "camera_off", "disconnect"])
        shot = frames(got["rows"])
        self.assertEqual(len(shot), 20)
        self.assertEqual(named(got["rows"], "end")[0]["why"], "done")
        ids = [row["id"] for row in shot]
        self.assertEqual(ids, sorted(set(ids)))
        for row in shot:
            data = base64.b64decode(row["jpeg"])
            self.assertEqual(hashlib.sha256(data).hexdigest(), row["sha256"])
            self.assertEqual((row["w"], row["h"], row["on_charger"]), (640, 360, True))
        self.assertEqual(named(got["rows"], "begin")[0]["in_view"], ["none"])
        self.assertFalse(got["lock_left"])

    def test_quiet_mode_on_is_lifted_only_while_control_is_held_and_put_back_before_release(self) -> None:
        log: List[str] = []
        pod = FakePod(log, quiet=True)
        got = run(BASE, new_robot(log=log), pod)
        self.assertEqual(got["code"], 0)
        self.assertTrue(pod.quiet)
        order = [e for e in log if e in ("pod:cryo", "robot:connect", "pod:quiet_off", "robot:camera_on",
                                          "robot:camera_off", "pod:quiet_on", "robot:disconnect")]
        self.assertEqual(order, ["pod:cryo", "robot:connect", "pod:quiet_off", "robot:camera_on",
                                 "robot:camera_off", "pod:quiet_on", "robot:disconnect"])

    def test_quiet_mode_off_is_left_alone(self) -> None:
        pod = FakePod([], quiet=False)
        run(BASE, new_robot(), pod)
        self.assertEqual([m for m, u in pod.requests if "quiet_mode" in u], ["GET"])
        self.assertFalse(pod.quiet)

    def test_quiet_lift_off_never_asks_about_quiet_mode(self) -> None:
        pod = FakePod([], quiet=True)
        run(dict(BASE, QUIET_LIFT="0"), new_robot(), pod)
        self.assertEqual([u for _, u in pod.requests if "quiet_mode" in u], [])

    def test_cryosleep_on_means_no_connection_and_no_quiet_change(self) -> None:
        robot, pod = new_robot(), FakePod([], quiet=True, cryo="on")
        got = run(BASE, robot, pod)
        self.assertEqual(got["code"], 6)
        self.assertEqual(robot.calls, [])
        self.assertEqual(len(pod.requests), 1)
        self.assertTrue(pod.quiet)
        self.assertFalse(got["lock_left"])

    def test_cryosleep_unknown_means_no_connection_but_a_pod_without_the_endpoint_goes_ahead(self) -> None:
        robot = new_robot()
        self.assertEqual(run(BASE, robot, FakePod([], cryo="down"))["code"], 6)
        self.assertEqual(robot.calls, [])
        robot = new_robot()
        self.assertEqual(run(BASE, robot, FakePod([], cryo="404"))["code"], 0)
        self.assertIn("connect", robot.calls)

    def test_control_never_granted_ends_without_lifting_quiet_or_starting_the_camera(self) -> None:
        robot, pod = new_robot(granted_at=None), FakePod([], quiet=True)
        got = run(BASE, robot, pod)
        self.assertEqual(got["code"], 5)
        self.assertEqual(robot.calls, ["connect", "disconnect"])
        self.assertTrue(pod.quiet)
        self.assertEqual([m for m, u in pod.requests if "quiet_mode" in u], [])

    def test_lost_control_stops_the_capture_and_quiet_mode_still_goes_back(self) -> None:
        pod = FakePod([], quiet=True)
        got = run(BASE, new_robot(lost_at=5.5), pod)
        self.assertEqual(named(got["rows"], "end")[0]["why"], "control_lost")
        self.assertLessEqual(len(frames(got["rows"])), 6)
        self.assertTrue(pod.quiet)

    def test_a_camera_that_stops_producing_new_frames_stops_the_capture(self) -> None:
        got = run(dict(BASE, STALL_S="4"), new_robot(stall_at=3.0), FakePod([]))
        end = named(got["rows"], "end")[0]
        self.assertEqual(end["why"], "stall")
        self.assertEqual(end["frames"], 3)
        self.assertLess(end["t"], 10.0)

    def test_a_stop_signal_stops_the_capture(self) -> None:
        got = run(BASE, new_robot(clock=StopAt(7.5)), FakePod([]))
        self.assertEqual(named(got["rows"], "end")[0]["why"], "signal:SIGTERM")
        self.assertEqual(len(frames(got["rows"])), 7)

    def test_a_held_lock_means_no_pod_call_and_no_connection(self) -> None:
        robot, pod = new_robot(), FakePod([])
        got = run(BASE, robot, pod, lock_held=True)
        self.assertEqual(got["code"], 3)
        self.assertEqual((robot.calls, pod.requests), ([], []))

    def test_settings_out_of_range_are_refused_before_anything_happens(self) -> None:
        bad = ({"IN_VIEW": ""}, {"IN_VIEW": "R2D2"}, {"IN_VIEW": "Unit 2,unit 2"}, {"IN_VIEW": "none,unit 2"},
               {"CAPTURE_S": "5"}, {"CAPTURE_S": "601"}, {"FPS": "0.1"}, {"FPS": "6"}, {"STALL_S": "1"},
               {"JPEG_QUALITY": "40"}, {"VECTOR_SERIAL": "xyz"}, {"VECTOR_IP": "10.3.142"},
               {"QUIET_LIFT": "yes"}, {"SCENE": "x" * 401})
        for change in bad:
            robot, pod = new_robot(), FakePod([])
            with self.assertRaises(AssertionError, msg=str(change)):
                run(dict(BASE, **change), robot, pod)
            self.assertEqual((robot.calls, pod.requests), ([], []), str(change))


class Saver(unittest.TestCase):
    def save(self, stream: str, out: Path) -> Dict[str, Any]:
        module = load(SAVER, "save_capture_under_test")
        buf = io.StringIO()
        with mock.patch.object(sys, "stdin", io.StringIO(stream)), contextlib.redirect_stdout(buf):
            code = module.main(["save_capture.py", str(out)])
        return {"code": code, "summary": json.loads(buf.getvalue().splitlines()[-1])}

    def stream(self) -> str:
        return run(BASE, new_robot(), FakePod([]))["stream"]

    def test_it_keeps_exactly_the_bytes_the_robot_side_hashed(self) -> None:
        stream = self.stream()
        with tempfile.TemporaryDirectory() as tmp:
            got = self.save(stream, Path(tmp) / "cap")
            self.assertEqual(got["code"], 0)
            self.assertEqual((got["summary"]["frames"], got["summary"]["end"]), (20, "done"))
            rows = [json.loads(l) for l in (Path(tmp) / "cap" / "capture.jsonl").read_text().splitlines()]
            kept = [r for r in rows if r["k"] == "f"]
            self.assertEqual(len(kept), 20)
            for row in kept:
                self.assertNotIn("jpeg", row)
                data = (Path(tmp) / "cap" / row["file"]).read_bytes()
                self.assertEqual(hashlib.sha256(data).hexdigest(), row["sha256"])

    def test_a_frame_whose_bytes_do_not_match_its_hash_stops_the_save(self) -> None:
        lines = self.stream().splitlines()
        i = next(n for n, l in enumerate(lines) if json.loads(l)["k"] == "f")
        row = json.loads(lines[i])
        row["jpeg"] = base64.b64encode(b"\xff\xd8tampered").decode()
        lines[i] = json.dumps(row)
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(self.save("\n".join(lines) + "\n", Path(tmp) / "cap")["code"], 4)

    def test_frames_before_begin_or_a_repeated_image_id_stop_the_save(self) -> None:
        lines = self.stream().splitlines()
        first = next(l for l in lines if json.loads(l)["k"] == "f")
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(self.save(first + "\n" + "\n".join(lines) + "\n", Path(tmp) / "a")["code"], 4)
            self.assertEqual(self.save("\n".join(lines) + "\n" + first + "\n", Path(tmp) / "b")["code"], 4)

    def test_it_refuses_an_output_folder_that_already_holds_anything(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            (Path(tmp) / "keep.txt").write_text("x")
            self.assertEqual(self.save(self.stream(), Path(tmp))["code"], 2)
            self.assertTrue((Path(tmp) / "keep.txt").exists())


if __name__ == "__main__":
    unittest.main()
