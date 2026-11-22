"""Wheelstands: the balance geometry, reading tilt from the accelerometer, and a whole constructed session.

Needs no MuJoCo except the last class, which checks the balance angle against the physics..
"""

from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import _common
from simulacrum import params, wheelstand

V = params.values(params.load())
G = wheelstand.geometry(V)
DT = 0.06
SCALE = 1.039  # the accelerometer read 1.039 g at rest; a common scale must cancel


def reading(pitch_deg: float, roll_deg: float, head_rad: float) -> List[float]:
    """What the head's accelerometer reads at rest for a body tilt, with the table's offsets and pitch."""
    th, ps = math.radians(pitch_deg), math.radians(roll_deg)
    ux, uy, uz = math.sin(th) * math.cos(ps), -math.sin(ps), math.cos(th) * math.cos(ps)
    a = head_rad + math.radians(V["imu_pitch_offset_deg"])
    g, mg = SCALE * V["gravity_mmps2"], 0.001 * V["gravity_mmps2"]
    raw = (g * (ux * math.cos(a) + uz * math.sin(a)), g * uy, g * (-ux * math.sin(a) + uz * math.cos(a)))
    return [raw[i] + mg * V["accel_bias_mg"][i] for i in range(3)]


class Session:
    """A constructed recording: holds, releases and hand moves, at 60 ms per row."""

    def __init__(self) -> None:
        self.rows: List[Dict[str, Any]] = [{"k": "c", "t": 0.0, "name": "begin", "phases": ["wheelstand"]}]
        self.t = 0.0

    def hold(self, pitch: float, roll: float, seconds: float, head_deg: float = 0.0, lift_mm: float = 32.0) -> None:
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            self.rows.append({"k": "s", "t": round(self.t, 4), "accel": reading(pitch, roll, math.radians(head_deg)),
                              "gyro": [0.002, -0.001, 0.001], "head": math.radians(head_deg), "lift": lift_mm,
                              "wl": 0.0, "wr": 0.0, "status": 0})

    def move(self, seconds: float, rate_dps: float) -> None:
        for _ in range(int(round(seconds / DT))):
            self.t += DT
            self.rows.append({"k": "s", "t": round(self.t, 4), "accel": [3000.0, 0.0, 6000.0],
                              "gyro": [0.0, math.radians(rate_dps), 0.0], "head": 0.0, "lift": 32.0,
                              "wl": 0.0, "wr": 0.0, "status": 0})

    def release(self, kind: str, angle: float, falls_over: bool) -> None:
        """Hold at an angle, let go, and come to rest where it fell."""
        pitch, roll = (angle, 0.0) if kind == "rear" else (0.0, angle)
        self.move(0.6, 25.0)
        self.hold(pitch, roll, 1.2)
        self.move(0.36, 240.0)
        if not falls_over:
            self.hold(0.0, 0.0, 1.0)
        elif kind == "rear":
            self.hold(100.0, 0.0, 1.0)
        else:
            self.hold(0.0, 88.0, 1.0)
        self.move(1.5, 30.0)
        self.hold(0.0, 0.0, 1.0)

    def write(self, path: Path) -> Path:
        path.write_text("".join(json.dumps(r) + "\n" for r in self.rows), encoding="utf-8")
        return path


def bisect_balance(dx: float, dz: float) -> float:
    """Where a point (dx forward, dz up of a pivot) passes over the pivot as the body tips back, by bisection."""
    low, high = 0.0, 89.9
    for _ in range(80):
        mid = 0.5 * (low + high)
        th = math.radians(mid)
        if dx * math.cos(th) - dz * math.sin(th) > 0.0:
            low = mid
        else:
            high = mid
    return 0.5 * (low + high)


class Geometry(unittest.TestCase):
    def test_closed_form_matches_a_bisection_of_the_rotated_centre_of_mass(self) -> None:
        for com_x, com_z in ((-24.0, 22.0), (-30.0, 15.0), (-14.0, 30.0)):
            expected = wheelstand.expected_deg(com_x, com_z, G)
            rear = bisect_balance(com_x + G["wheel_base"], com_z - G["rear_radius"])
            side = bisect_balance(G["edge_y"], com_z)
            self.assertAlmostEqual(expected["rear"], rear, delta=1e-6)
            self.assertAlmostEqual(expected["side"], side, delta=1e-6)

    def test_solve_inverts_the_balance_angles(self) -> None:
        angles = wheelstand.expected_deg(-19.5, 26.0, G)
        com_x, com_z = wheelstand.solve(angles["rear"], angles["side"], G)
        self.assertAlmostEqual(com_x, -19.5, delta=1e-9)
        self.assertAlmostEqual(com_z, 26.0, delta=1e-9)

    def test_the_tables_robot_balances_at_65_and_52_degrees(self) -> None:
        angles = wheelstand.expected_deg(V["com_x_mm"], V["com_z_mm"], G)
        self.assertAlmostEqual(angles["rear"], 64.885, delta=0.001)
        self.assertAlmostEqual(angles["side"], 53.471, delta=0.001)  # tread edge at 29.7 mm (tread_center_spacing_mm, tread_width_mm)

    def test_a_ruler_width_sets_the_side_edge_and_narrows_the_interval(self) -> None:
        width = V["tread_center_spacing_mm"] + V["tread_width_mm"]  # a ruler reading the table's own width
        self.assertAlmostEqual(wheelstand.geometry(V, width)["edge_y"], 0.5 * width, places=9)
        angles = wheelstand.expected_deg(-24.0, 22.0, G)
        table = wheelstand.solve_interval((angles["rear"], 0.5), (angles["side"], 0.5), V)
        ruler = wheelstand.solve_interval((angles["rear"], 0.5), (angles["side"], 0.5), V, width)
        alone = wheelstand.solve_interval((angles["rear"], 0.5), (angles["side"], 0.5), V, width, angles_only=True)
        width = lambda iv: iv["com_x_mm"][1] - iv["com_x_mm"][0]
        self.assertGreater(width(table), width(ruler))
        self.assertGreater(width(ruler), width(alone))
        for interval in (table, ruler, alone):
            self.assertTrue(interval["com_x_mm"][0] < -24.0 < interval["com_x_mm"][1])
            self.assertTrue(interval["com_z_mm"][0] < 22.0 < interval["com_z_mm"][1])


class Tilt(unittest.TestCase):
    def test_tilt_is_read_through_the_head_angle_offsets_and_scale(self) -> None:
        for pitch, roll, head in ((64.9, 0.0, 0.0), (0.0, 51.8, 0.0), (30.0, 0.0, 0.35), (0.0, -40.0, -0.2),
                                  (100.0, 0.0, 0.0), (0.0, 88.0, 0.1)):
            row = {"accel": reading(pitch, roll, head), "head": head}
            got = wheelstand.tilt_deg(row, V)
            self.assertAlmostEqual(got[0], pitch, delta=1e-6)
            self.assertAlmostEqual(got[1], roll, delta=1e-6)

    def test_an_x_axis_scale_unlike_the_others_is_undone_before_the_tilt(self) -> None:
        scaled = dict(V, accel_scale=[1.05, 1.0, 1.0])
        mg = 0.001 * V["gravity_mmps2"]
        for pitch, head in ((64.9, 0.0), (30.0, 0.35)):
            raw = reading(pitch, 0.0, head)
            row = {"accel": [(raw[0] - mg * V["accel_bias_mg"][0]) * 1.05 + mg * V["accel_bias_mg"][0], raw[1], raw[2]],
                   "head": head}
            self.assertAlmostEqual(wheelstand.tilt_deg(row, scaled)[0], pitch, delta=1e-6)
            self.assertGreater(abs(wheelstand.tilt_deg(row, V)[0] - pitch), 0.3)


def a_session(true_x: float, true_z: float, step: float) -> Tuple[Session, Dict[str, float]]:
    """Releases on both sides of each balance angle, as a person would make them, plus one hand move and one off-pose hold."""
    truth = wheelstand.expected_deg(true_x, true_z, G)
    session = Session()
    session.hold(0.0, 0.0, 2.0)
    for kind in ("rear", "side"):
        for offset in (-2.0, -1.0, 1.0, 2.0):
            session.release(kind, truth[kind] + offset * step, falls_over=offset > 0)
    session.hold(truth["rear"] + 3.0 * step, 0.0, 1.2)
    session.move(1.2, 20.0)
    session.hold(0.0, 0.0, 1.0)
    session.hold(truth["rear"] - 5.0 * step, 0.0, 1.2, head_deg=15.0)
    session.move(0.36, 240.0)
    session.hold(0.0, 0.0, 1.0)
    return session, truth


class Session_(unittest.TestCase):
    def test_a_constructed_session_brackets_both_angles_and_recovers_the_centre_of_mass(self) -> None:
        session, truth = a_session(-21.0, 24.0, 0.4)
        with tempfile.TemporaryDirectory() as tmp:
            out = wheelstand.describe(session.write(Path(tmp) / "ws.jsonl"))
        rear, side = out["stands"]["rear"], out["stands"]["side"]
        self.assertEqual((rear["method"], side["method"]), ("bracket", "bracket"))
        self.assertEqual((rear["fell_back_onto_treads"], rear["fell_over"], rear["held"], rear["off_pose"]), (2, 2, 1, 1))
        self.assertEqual((side["fell_back_onto_treads"], side["fell_over"]), (2, 2))
        self.assertAlmostEqual(rear["estimate_deg"], truth["rear"], delta=1e-6)
        self.assertAlmostEqual(rear["half_width_deg"], 0.4, delta=1e-6)
        self.assertAlmostEqual(side["estimate_deg"], truth["side"], delta=1e-6)
        com = out["com_mm"]
        self.assertAlmostEqual(com["com_x_mm"], -21.0, delta=1e-6)
        self.assertAlmostEqual(com["com_z_mm"], 24.0, delta=1e-6)
        alone = com["interval_from_angles_alone"]
        self.assertTrue(alone["com_x_mm"][0] < -21.0 < alone["com_x_mm"][1])
        self.assertEqual(com["side_edge_from"], "table ranges")

    def test_a_slow_drift_under_the_gyro_threshold_is_cut_into_short_stretches(self) -> None:
        session = Session()
        session.hold(0.0, 0.0, 1.0)
        for step in range(40):
            session.hold(40.0 + 0.25 * step, 0.0, DT)
        with tempfile.TemporaryDirectory() as tmp:
            out = wheelstand.describe(session.write(Path(tmp) / "drift.jsonl"))
        tilted = [t for t in out["trials"] if t["kind"] == "rear"]
        self.assertGreaterEqual(len(tilted), 2)
        for trial in tilted:
            self.assertLessEqual(trial["rows"] * 0.25, wheelstand.STRETCH_DRIFT_DEG + 0.25)

    def test_holds_alone_give_a_held_estimate_and_one_stand_alone_gives_no_centre_of_mass(self) -> None:
        session = Session()
        session.hold(0.0, 0.0, 1.0)
        for angle in (64.0, 65.0, 66.0):
            session.hold(angle, 0.0, 1.5)
            session.move(2.0, 15.0)
            session.hold(0.0, 0.0, 1.0)
        with tempfile.TemporaryDirectory() as tmp:
            out = wheelstand.describe(session.write(Path(tmp) / "held.jsonl"))
        rear = out["stands"]["rear"]
        self.assertEqual((rear["method"], rear["held"]), ("held", 3))
        self.assertAlmostEqual(rear["estimate_deg"], 65.0, delta=1e-6)
        self.assertGreaterEqual(rear["half_width_deg"], 1.0)
        self.assertEqual(out["stands"]["side"]["method"], "none")
        self.assertIn("not_computable", out["com_mm"])


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Physics(unittest.TestCase):
    def test_the_model_balances_where_the_closed_form_says(self) -> None:
        import wheelstand_check
        result = wheelstand_check.check(V)
        self.assertLess(abs(result["rear"]["sim_deg"] - result["rear"]["closed_form_deg"]), 0.5, result)
        self.assertLess(abs(result["side"]["sim_deg"] - result["side"]["closed_form_deg"]), 0.5, result)
        self.assertEqual(result["rear"]["other_contacts_at_balance"], [], result)
        self.assertEqual(result["side"]["other_contacts_at_balance"], [], result)
        self.assertLess(abs(result["rear"]["accel_read_deg"] - result["rear"]["placed_deg"]), 0.3, result)
        self.assertLess(abs(result["side"]["accel_read_deg"] - result["side"]["placed_deg"]), 0.3, result)


if __name__ == "__main__":
    unittest.main()
