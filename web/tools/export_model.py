#!/usr/bin/env python3
"""Export the simulacrum's model for the browser..

Writes, from the simulacrum one directory up (the single source of every number):

  public/robot/vector.xml   the MJCF the browser compiles
  public/robot/model.json   the parameter values and the derived quantities
  test/fixtures/drive.json  traces of the Python tread encoder and speed loop
  test/fixtures/hazards.json  traces of the Python hazard flags
  test/fixtures/prox.json   readings of the Python time-of-flight sensor
  test/fixtures/robot.json  runs of the Python VectorSim (needs the mujoco
                            package; skipped with a message when it is absent)

The JavaScript in src/sim/ is a port of simulacrum/{drive,hazards,robot}.py.
The fixtures are what the port is checked against: `npm test` replays the
same inputs through the JavaScript and compares.

With --check nothing is written: the files on disk are compared with what
the simulacrum would export now, so a change to the parameter table that was
not re-exported is caught. vector.xml must match exactly; the JSON files to
1e-9 (math-library results can differ in the last digit between machines).
The VectorSim fixture is not re-run by --check: it is tied to one MuJoCo
version, and test/robot.test.js already refuses a model it was not made with.

Usage: python3 tools/export_model.py [--simulacrum DIR] [--no-robot] [--check]
Exit: 0 written (or, with --check, up to date), 1 --check found a difference,
2 bad arguments, 3 simulacrum not found.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import random
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence

HERE = Path(__file__).resolve().parent.parent
DEFAULT_SIMULACRUM = HERE.parent
# One static block on the table, in front of the start pose: (cx, cy, cz, sx, sy, sz) in mm.
SCENE_OBSTACLES = ((190.0, 60.0, 22.0, 44.0, 44.0, 44.0),)
FIXTURE_SEED = 20261118
ROBOT_MODEL = "vector_1"
# The head camera looks 4 degrees below the head's forward axis in the engine the robot runs.
# Fallback for --simulacrum pointing to the older, level-camera MJCF generator.
CAMERA_PITCH_DOWN_DEG = 4.0
CAMERA_PITCH_DOWN_SOURCE = ("the robot's engine, engine/fullRobotPose.cpp in kercre123/victor, branch snowboy "
                            "(sha256 2c7df61741fd0751d0cc835c7558edec24c26974ccdcdfcdfbcfb7aa04d9ac3d): "
                            "kDefaultHeadCamRotation, '4-degree look down'; its optical axis works out to 4.002 degrees "
                            "below the head's forward axis. Not measured on a robot.")
CAMERA_NEAR_MM = 5.0  # nearest distance drawn; a drawing choice, not a property of the lens
CALIBRATION_FILE = HERE / "camera_calibration.json"
# What documents say, kept beside the real calibration for comparison only.
CAMERA_DOCUMENTS = {"anki_simulator_horizontal_deg": 94.22,
                    "anki_simulator_source": "simulator/protos/CozmoCamera.proto, fieldOfView 1.6445 rad at 640 x 360"}
ROBOT_SAMPLE_TICKS = 10
# Heading at which the block is straight ahead of the range sensor (which sits 10 mm ahead of the origin).
BLOCK_BEARING_DEG = 18.0


def _jsonable(value: Any) -> Any:
    """Tuples to lists, recursively; everything else unchanged."""
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    assert value is None or isinstance(value, (bool, int, float, str)), "unexpected type in the table"
    return value


def _provenance_counts(table_bytes: bytes) -> Dict[str, int]:
    """How many parameters the table marks measured, fitted, manual, sdk, assumed, chosen."""
    entries = json.loads(table_bytes)["parameters"]
    counts: Dict[str, int] = {}
    for entry in entries.values():
        counts[entry["provenance"]] = counts.get(entry["provenance"], 0) + 1
    assert sum(counts.values()) == len(entries) > 0, "provenance count does not cover the table"
    return counts


def _calibration(image_size) -> Dict[str, Any]:
    """The real robot's camera calibration, checked against the field of view the robot reported with it."""
    path = HERE / "camera_calibration_vector2.json" if tuple(image_size) == (800, 600) else CALIBRATION_FILE
    cal = json.loads(path.read_text(encoding="utf-8"))
    width, height, fx, fy = cal["width_px"], cal["height_px"], cal["fx_px"], cal["fy_px"]
    assert width > 0 and height > 0 and fx > 0.0 and fy > 0.0, "bad camera calibration"
    assert 0.0 < cal["cx_px"] < width and 0.0 < cal["cy_px"] < height, "optical centre outside the image"
    # The robot reports its field of view as twice the angle from the image centre to its edge.
    for size, focal, reported in ((width, fx, cal["reported_fov_x_deg"]), (height, fy, cal["reported_fov_y_deg"])):
        implied = 2.0 * math.degrees(math.atan(0.5 * size / focal))
        assert abs(implied - reported) < 0.01, f"calibration is not self-consistent: {implied} against {reported}"
    return cal


def camera_block(v: Dict[str, Any]) -> Dict[str, Any]:
    """The head camera for the page's camera view: where it is, where it looks, and its optics.

    Optics: the real robot's factory calibration (camera_calibration.json), a pinhole with
    its own focal lengths and optical centre. The manual's and Anki's simulator's field of
    view are carried along for comparison and used for nothing.
    Position: the table's head_camera_position_mm (the firmware's, relative to the neck).
    Tilt: the engine's 4 degrees down, until the table carries it.
    """
    cal = _calibration(v["camera_resolution_px"])
    width, height = cal["width_px"], cal["height_px"]
    assert [width, height] == [int(x) for x in v["camera_resolution_px"]], "calibration is for another image size"
    edges = {"left": math.atan(cal["cx_px"] / cal["fx_px"]), "right": math.atan((width - cal["cx_px"]) / cal["fx_px"]),
             "up": math.atan(cal["cy_px"] / cal["fy_px"]), "down": math.atan((height - cal["cy_px"]) / cal["fy_px"])}
    in_table = "head_camera_pitch_down_deg" in v
    pitch = float(v["head_camera_pitch_down_deg"]) if in_table else CAMERA_PITCH_DOWN_DEG
    assert 0.0 <= pitch < 45.0, "camera pitch out of range"
    return {
        "name": "head_cam", "width_px": width, "height_px": height,
        "fx_px": cal["fx_px"], "fy_px": cal["fy_px"], "cx_px": cal["cx_px"], "cy_px": cal["cy_px"],
        "fov_horizontal_deg": math.degrees(edges["left"] + edges["right"]),
        "fov_vertical_deg": math.degrees(edges["up"] + edges["down"]),
        "edge_angles_deg": {k: math.degrees(a) for k, a in edges.items()},
        "optics_source": cal["what"] + ". " + cal["read"], "optics_limits": cal["limits"],
        "near_mm": CAMERA_NEAR_MM, "position_from_neck_mm": _jsonable(v["head_camera_position_mm"]),
        "pitch_down_deg": pitch,
        # False: the MJCF's camera looks straight ahead and the page adds the tilt itself.
        "pitch_down_in_model": in_table,
        "pitch_down_source": "parameter table" if in_table else CAMERA_PITCH_DOWN_SOURCE,
        "documents": dict(CAMERA_DOCUMENTS, manual_deg=[v["camera_fov_horizontal_deg"], v["camera_fov_vertical_deg"]],
                          model_fovy_deg=v["camera_fov_vertical_deg"]),
    }


def _write(path: Path, text: str) -> None:
    assert text, "refusing to write an empty file"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    print(f"wrote {path.relative_to(HERE)}  {len(text)} bytes")


def _same(a: Any, b: Any) -> bool:
    """Equal, with numbers compared to a relative 1e-9."""
    if isinstance(a, bool) or isinstance(b, bool) or a is None or b is None or isinstance(a, str) or isinstance(b, str):
        return type(a) is type(b) and a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12)
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(_same(x, y) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return a.keys() == b.keys() and all(_same(a[k], b[k]) for k in a)
    return False


def _check(outputs: Dict[str, str]) -> int:
    """Compare what would be exported with the files on disk. Returns the exit code."""
    assert outputs, "nothing to check"
    stale = []
    for relative, text in outputs.items():
        path = HERE / relative
        if not path.is_file():
            stale.append(f"{relative}: missing")
        elif relative.endswith(".xml"):
            if path.read_text(encoding="utf-8") != text:
                stale.append(f"{relative}: differs")
        elif relative.endswith("model.json"):
            on_disk, fresh = json.loads(path.read_text(encoding="utf-8")), json.loads(text)
            hashes = on_disk.get("source") == fresh["source"]
            on_disk.pop("source", None), fresh.pop("source")
            if not (hashes and _same(on_disk, fresh)):
                stale.append(f"{relative}: differs")
        elif not _same(json.loads(path.read_text(encoding="utf-8")), json.loads(text)):
            stale.append(f"{relative}: differs")
    for line in stale:
        print(f"STALE {line}", file=sys.stderr)
    print(f"checked {len(outputs)} exported files against the simulacrum: "
          + ("up to date" if not stale else f"{len(stale)} out of date; run tools/export_model.py"))
    return 1 if stale else 0


def drive_fixture(v: Dict[str, Any], drive: Any) -> Dict[str, Any]:
    """Seeded inputs through TreadEncoder and TreadController, with every output."""
    rng = random.Random(FIXTURE_SEED)
    encoder, travel, speed = drive.TreadEncoder(v), 0.0, 0.0
    enc_rows: List[List[float]] = []
    for tick in range(1200):
        if tick % 150 == 0:
            speed = rng.choice([0.0, 0.0, 25.0, -40.0, 100.0, 220.0, -180.0, 3.0])
        travel += speed * v["control_period_ms"] * 0.001 * (1.0 + 0.05 * rng.uniform(-1.0, 1.0))
        out = encoder.update(travel)
        enc_rows.append([travel, encoder.counts, out])
    controller = drive.TreadController(v)
    ctl_rows: List[List[float]] = []
    desired = 0.0
    for tick in range(1200):
        if tick % 100 == 0:
            desired = rng.choice([0.0, 0.5, 20.0, -20.0, 60.0, 150.0, -220.0, 220.0])
        measured = float(int(desired * rng.uniform(0.2, 1.3)))
        power = controller.update(desired, measured)
        ctl_rows.append([desired, measured, power, controller.error_sum])
    assert len(enc_rows) == 1200 and len(ctl_rows) == 1200, "fixture length changed"
    return {"encoder": enc_rows, "controller": ctl_rows,
            "feedforward": [[s, drive.TreadController(v).feedforward(s)] for s in (-220.0, -5.0, 0.0, 1.0, 5.0, 30.0, 220.0)],
            "whole_mmps": [[s, drive.whole_mmps(s)] for s in (-3.7, -0.2, 0.0, 0.9, 41.99)]}


def _prox_rows(values: Dict[str, Any], prox_mod: Any, seed: int) -> List[List[Any]]:
    """Seeded fans of hits, lift heights and pitches through one ProxSensor, with every reading."""
    rng = random.Random(seed)
    sensor = prox_mod.ProxSensor(values, 0)
    count = len(sensor.rays)
    rows: List[List[Any]] = []
    lift, pitch = 80.0, 0.0
    for step in range(900):
        if step % 9 == 0:
            lift = rng.choice([32.0, 33.9, 34.0, 45.0, 56.0, 56.1, 80.0, 80.0, 80.0])
        if step % 31 == 0:
            pitch = math.radians(rng.choice([0.0, 0.0, 0.0, 4.9, 5.1, -5.1]))
        base = rng.choice([-1.0, 12.0, 35.0, 80.0, 150.0, 260.0, 380.0, 420.0, 900.0, 1300.0])
        filled = rng.randint(0, count)
        hits = [(base * rng.uniform(0.9, 1.1) if base > 0.0 and k < filled else -1.0) for k in range(count)]
        r = sensor.read(hits, lift, pitch)
        rows.append([hits, lift, pitch, r.distance_mm, r.signal_quality, r.unobstructed, r.found_object, r.is_lift_in_fov])
    return rows


def prox_fixture(v: Dict[str, Any], prox_mod: Any) -> Dict[str, Any]:
    """Readings of the Python sensor for the port to reproduce. The sensor's noise is set to zero:
    its random stream is not the same in the two languages. A second set has the signal made
    strong, so that far hits register and the upper report limit is reached."""
    quiet = dict(v, tof_range_std_fraction=0.0, tof_weak_distance_sd_mm=0.0)
    strong = dict(quiet, tof_quality_at_100mm=50.0)
    rows, strong_rows = _prox_rows(quiet, prox_mod, FIXTURE_SEED + 2), _prox_rows(strong, prox_mod, FIXTURE_SEED + 3)
    for index, label in ((5, "unobstructed"), (6, "found"), (7, "lift in view")):
        assert any(row[index] for row in rows) and not all(row[index] for row in rows), f"fixture never varies {label}"
    assert any(row[3] == v["tof_report_min_mm"] for row in rows), "the lower report limit is never reached"
    assert any(row[3] == v["tof_report_max_mm"] for row in strong_rows), "the upper report limit is never reached"
    return {"values": _jsonable(quiet), "strong_values": _jsonable(strong), "rays": _jsonable(prox_mod.ray_directions(quiet)),
            "rows": rows, "strong_rows": strong_rows}


def hazards_fixture(v: Dict[str, Any], hazards_mod: Any) -> Dict[str, Any]:
    """Seeded cliff ranges and accelerations through Hazards, with every flag."""
    rng = random.Random(FIXTURE_SEED + 1)
    hz = hazards_mod.Hazards(v)
    g = v["gravity_mmps2"]
    rows: List[List[Any]] = []
    ranges = [v["cliff_sensor_height_mm"]] * 4
    accel = g
    for tick in range(1500):
        if tick % 37 == 0:
            ranges = [rng.choice([v["cliff_sensor_height_mm"], -1.0, 300.0, v["cliff_detect_range_mm"] + 0.1,
                                  v["cliff_detect_range_mm"] - 0.1]) for _ in range(4)]
        if tick % 23 == 0:
            accel = rng.choice([g, 0.0, 0.3 * g, 2.0 * g, v["fall_accel_low_mmps2"] - 1.0, v["fall_accel_high_mmps2"] + 1.0])
        hz.update(ranges, accel)
        rows.append([list(ranges), accel, list(hz.cliff), hz.cliff_any, hz.cliff_new, hz.falling, hz.picked_up])
    assert any(r[5] for r in rows) and any(r[6] for r in rows), "fixture never raises falling or picked_up"
    return {"rows": rows}


def _robot_row(sim: Any) -> List[Any]:
    truth = sim.sim_truth()
    x, y, z = sim.pose_mm()
    rep = sim.reported_now()
    return [sim.tick, x, y, z, sim.heading_total_rad(), rep["wl"], rep["wr"], truth["motor_power"][0],
            truth["motor_power"][1], rep["head_deg"], rep["lift_mm"], list(truth["cliff_flags"]),
            sim.has_control, sim.reactions, truth["fallen"], sim.hazards.picked_up, sim.hazards.falling,
            sim.battery_volts, sim.tread_heat[0], sim.tread_heat[1], sim.state.prox_distance_mm,
            sim.state.prox_signal_quality, sim.state.prox_found_object, sim.state.prox_unobstructed,
            sim.state.prox_is_lift_in_fov]


def _run_script(robot_mod: Any, v: Dict[str, Any], level: str, script: Sequence[Sequence[Any]], ticks: int) -> List[List[Any]]:
    """Run timed commands (tick, name, args...) and sample the robot every ROBOT_SAMPLE_TICKS."""
    heading = BLOCK_BEARING_DEG if any(c[1] == "turn_toward_block" for c in script) else 0.0
    sim = robot_mod.VectorSim(values=v, control_level=level, obstacles=SCENE_OBSTACLES, heading_deg=heading)
    pending = sorted((c for c in script if c[1] != "turn_toward_block"), key=lambda c: c[0])
    rows = [_robot_row(sim)]
    at = 0
    for tick in range(ticks):
        while at < len(pending) and pending[at][0] == tick:
            getattr(sim, pending[at][1])(*pending[at][2:])
            at += 1
        sim.step(1)
        if sim.tick % ROBOT_SAMPLE_TICKS == 0:
            rows.append(_robot_row(sim))
    assert at == len(pending), "a scripted command was never issued"
    return rows


def _action_runs(robot_mod: Any, quiet: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Blocking SDK actions, one after another on one robot per list, with each result."""
    plans = {
        "default": [("drive_straight", 120.0, 80.0), ("turn_in_place", 90.0), ("set_head_angle", 25.0),
                    ("set_lift_height", 80.0), ("drive_straight", -60.0, 50.0), ("turn_in_place", -135.0, 60.0),
                    ("drive_straight", 400.0, 100.0), ("drive_straight", 50.0, 50.0)],
        "override": [("turn_in_place", 180.0), ("drive_straight", 500.0, 120.0)],
    }
    out = []
    for level, plan in plans.items():
        sim = robot_mod.VectorSim(values=quiet, control_level=level, obstacles=SCENE_OBSTACLES)
        steps = []
        for name, *args in plan:
            res = getattr(sim, name)(*args)
            x, y, z = sim.pose_mm()
            steps.append({"call": [name, *args], "result": res.result, "moved_mm": res.moved_mm,
                          "turned_deg": res.turned_deg, "ticks": res.ticks, "tick": sim.tick,
                          "pose_mm": [x, y, z], "heading_total_rad": sim.heading_total_rad()})
        out.append({"level": level, "steps": steps})
    results = [s["result"] for run in out for s in run["steps"]]
    assert {"completed", "cancelled_cliff", "no_control", "aborted_off_ground"} <= set(results), \
        f"the action fixture does not reach every result it should: {sorted(set(results))}"
    return out


def _low_battery_run(robot_mod: Any, quiet: Dict[str, Any]) -> Dict[str, Any]:
    """A run on a robot with a weaker, sagging battery. In the table the battery rests at the
    reference voltage and does not sag, so the voltage scaling does nothing there; these
    overrides make it act. Both stay below the reference, so the MJCF is the same model."""
    overrides = {"battery_rest_voltage_v": 3.6, "battery_sag_v_per_power": 0.25}
    weak = dict(quiet, **overrides)
    assert weak["battery_rest_voltage_v"] < weak["motor_reference_voltage_v"], "the override must not change the MJCF"
    assert mjcf_same(robot_mod, quiet, weak), "the low-battery values compile to a different model"
    script = [(0, "set_wheel_motors", 140.0, 90.0), (300, "set_wheel_motors", -150.0, 150.0), (600, "stop_all_motors")]
    rows = _run_script(robot_mod, weak, "default", script, 700)
    assert min(r[17] for r in rows) < 3.4, "the battery never sagged in the low-battery run"
    assert rows[-1][12] is True and rows[-1][13] == 0, "the low-battery run must stay on the table and keep control"
    return {"overrides": overrides, "level": "default", "ticks": 700, "script": [list(c) for c in script], "rows": rows}


def mjcf_same(robot_mod: Any, a: Dict[str, Any], b: Dict[str, Any]) -> bool:
    """Whether two value sets generate the same model text."""
    return robot_mod.mjcf.build_mjcf(a, SCENE_OBSTACLES) == robot_mod.mjcf.build_mjcf(b, SCENE_OBSTACLES)


def robot_fixture(v: Dict[str, Any], robot_mod: Any, mujoco_version: str) -> Dict[str, Any]:
    """VectorSim runs with the sensor noise set to zero, so the port can be compared exactly."""
    quiet = dict(v)
    quiet.update({"accel_noise_rms_mg": 0.0, "gyro_noise_rms_dps": 0.0, "tof_range_std_fraction": 0.0,
                  "tof_weak_distance_sd_mm": 0.0})
    scripts = {
        "drive_and_turn": ("default", 900, [
            (0, "set_wheel_motors", 80.0, 80.0), (200, "set_wheel_motors", 60.0, -60.0),
            (350, "command_head_angle", 30.0), (350, "command_lift_height", 70.0),
            (500, "set_wheel_motors", -50.0, -50.0, 200.0, 200.0), (700, "stop_all_motors")]),
        "edge_default": ("default", 1500, [(0, "set_wheel_motors", 100.0, 100.0)]),
        "edge_override": ("override", 1100, [(0, "set_wheel_motors", 100.0, 100.0)]),
        # Faster than wheel_speed_max_mmps, which direct drive does not apply; long enough for the treads to warm.
        "fast_spin": ("default", 1400, [(0, "set_wheel_motors", 300.0, -300.0), (1200, "stop_all_motors")]),
        # Start facing the block, pass the lift through the range sensor's beam and out again, then turn away.
        "look_at_block": ("default", 1000, [(0, "turn_toward_block",), (200, "command_lift_height", 45.0),
                                            (500, "command_lift_height", 85.0), (750, "set_wheel_motors", 60.0, -60.0)]),
    }
    runs = {}
    for name, (level, ticks, script) in scripts.items():
        rows = _run_script(robot_mod, quiet, level, script, ticks)
        runs[name] = {"level": level, "ticks": ticks, "rows": rows,
                      "heading_deg": BLOCK_BEARING_DEG if name == "look_at_block" else 0.0,
                      "script": [list(c) for c in script if c[1] != "turn_toward_block"]}
    look = runs["look_at_block"]["rows"]
    assert any(r[22] for r in look) and any(r[24] for r in look) and any(r[23] for r in look), \
        "the range sensor run must find the block, lose it behind the lift, and see a clear way"
    assert runs["edge_default"]["rows"][-1][13] >= 1, "the default-level run never reacted to the edge"
    assert runs["edge_override"]["rows"][-1][14] is True, "the override-level run never fell"
    return {"values": _jsonable(quiet), "obstacles": _jsonable(SCENE_OBSTACLES), "mujoco_version": mujoco_version,
            "actions": _action_runs(robot_mod, quiet), "low_battery": _low_battery_run(robot_mod, quiet),
            "sample_ticks": ROBOT_SAMPLE_TICKS,
            "columns": ["tick", "x_mm", "y_mm", "z_mm", "heading_total_rad", "wl", "wr", "power_l", "power_r",
                        "head_deg", "lift_mm", "cliff_flags", "has_control", "reactions", "fallen", "picked_up",
                        "falling", "battery_v", "heat_l", "heat_r", "prox_mm", "prox_quality", "prox_found",
                        "prox_unobstructed", "prox_lift_in_view"],
            "runs": runs}


def main(argv: Sequence[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--simulacrum", type=Path, default=DEFAULT_SIMULACRUM)
    parser.add_argument("--no-robot", action="store_true", help="skip the fixture that needs the mujoco package")
    parser.add_argument("--check", action="store_true", help="write nothing; exit 1 if the exported files are out of date")
    args = parser.parse_args(list(argv))
    root = args.simulacrum.resolve()
    if not (root / "simulacrum" / "mjcf.py").is_file():
        print(f"simulacrum not found at {root}", file=sys.stderr)
        return 3
    sys.path.insert(0, str(root))
    from simulacrum import drive, hazards, mjcf, params, prox  # noqa: E402  (path set above)

    from contact_trial import contact_model

    table_bytes = (root / "parameters.json").read_bytes()
    # The base table, which the simulacrum calls vector_1. The recorded robot is a Vector 1.0
    # (the table's REC and REC2 sources, on its word).
    v = params.values(params.load(model=ROBOT_MODEL))
    d = mjcf.derive(v)
    xml = mjcf.build_mjcf(v, SCENE_OBSTACLES)
    assert all(math.isfinite(x) for x in d["chassis_inertia"]), "derived inertia is not finite"
    trial_xml = contact_model(xml, v)
    model = {"source": {"trial_xml_sha256": hashlib.sha256(trial_xml.encode()).hexdigest(),
                        "table_sha256": hashlib.sha256(table_bytes).hexdigest(),
                        "xml_sha256": hashlib.sha256(xml.encode("utf-8")).hexdigest()},
             "robot_model": ROBOT_MODEL,
             "values": _jsonable(v), "derived": _jsonable(d), "cliff_names": list(mjcf.CLIFF_NAMES),
             "provenance_counts": _provenance_counts(table_bytes), "camera": camera_block(v),
             "obstacles": _jsonable(SCENE_OBSTACLES)}
    outputs = {
        "public/robot/vector.xml": xml,
        "public/robot/vector-contact42.xml": trial_xml,
        "public/robot/model.json": json.dumps(model, indent=1, sort_keys=True) + "\n",
        "test/fixtures/drive.json": json.dumps(drive_fixture(v, drive)) + "\n",
        "test/fixtures/hazards.json": json.dumps(hazards_fixture(v, hazards)) + "\n",
        "test/fixtures/prox.json": json.dumps(prox_fixture(v, prox)) + "\n",
    }
    free_cube = mjcf.build_mjcf(v, SCENE_OBSTACLES, cube="free")
    for name, scene in [("vector-free", free_cube), ("vector-contact42-free", contact_model(free_cube, v))]:
        outputs[f"public/robot/{name}.xml"] = scene
        model["source"][name + "_sha256"] = hashlib.sha256(scene.encode()).hexdigest()
    outputs["public/robot/model.json"] = json.dumps(model, indent=1, sort_keys=True) + "\n"
    if args.check:
        return _check(outputs)
    for relative, text in outputs.items():
        _write(HERE / relative, text)
    if args.no_robot:
        print("skipped test/fixtures/robot.json (--no-robot)")
        return 0
    try:
        import mujoco  # noqa: F401
    except ImportError:
        print("skipped test/fixtures/robot.json: the mujoco package is not installed here", file=sys.stderr)
        return 0
    from simulacrum import robot as robot_mod  # noqa: E402
    _write(HERE / "test" / "fixtures" / "robot.json",
           json.dumps(robot_fixture(v, robot_mod, mujoco.__version__)) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
