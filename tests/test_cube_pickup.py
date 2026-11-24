"""The fork's barbed hooks, the cube's lift pockets and the lift's torque, and a pickup that uses them.

The stock fork carries a barbed hook on each upright and the light cube a pocket behind each
vertical edge's notch, both measured from Anki's animation models (evidence/cube-pickup). Docking
puts the hooks in the notches; raising the lift drops each pocket's lip into its hook's valley and
lifts the cube. The geometry checks need no MuJoCo; the torque and pickup checks do.
"""

from __future__ import annotations

import importlib.util
import math
import unittest
import xml.etree.ElementTree as ET
from typing import List

import numpy as np

import _common
from simulacrum import mjcf, params
from test_cube_surface import d4_asymmetry_count, load_boxes, max_pairwise_overlap_mm, union_mask_at

ROOT = _common.ROOT
TRIAL = ROOT / "evidence" / "cube-pickup" / "trial.py"
HOLDING = ROOT / "evidence" / "cube-pickup" / "holding_torque.py"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _geoms(root: ET.Element, prefix: str) -> List[ET.Element]:
    return [g for g in root.iter("geom") if g.get("name", "").startswith(prefix)]


def _floats(text) -> List[float]:
    return [float(x) for x in text.split()]


class HookGeometry(unittest.TestCase):
    def setUp(self) -> None:
        self.v = params.values(params.load())
        self.root = ET.fromstring(mjcf.build_mjcf(self.v))

    def test_each_upright_carries_the_tables_hook_and_upright(self) -> None:
        y_in, y_out = self.v["lift_fork_hook_y_mm"]
        boxes = [("fork_upright", self.v["lift_fork_hook_upright_mm"])] + [
            (f"fork_hook_{k}", s) for k, s in enumerate(self.v["lift_fork_hook_slices_mm"])]
        for stem, (x0, x1, z0, z1) in boxes:
            for side, sign in (("left", 1.0), ("right", -1.0)):
                found = [g for g in self.root.iter("geom") if g.get("name") == f"{stem}_{side}"]
                self.assertEqual(len(found), 1, f"{stem}_{side} missing or doubled")
                pos, size = _floats(found[0].get("pos")), _floats(found[0].get("size"))
                want_pos = [0.5 * (x0 + x1), sign * 0.5 * (y_in + y_out), 0.5 * (z0 + z1)]
                want_size = [0.5 * (x1 - x0), 0.5 * (y_out - y_in), 0.5 * (z1 - z0)]
                np.testing.assert_allclose(np.array(pos) / mjcf.MM, want_pos, atol=1e-6)
                np.testing.assert_allclose(np.array(size) / mjcf.MM, want_size, atol=1e-6)
        self.assertEqual(len(_geoms(self.root, "fork_hook_")), 2 * len(self.v["lift_fork_hook_slices_mm"]))

    def test_the_hook_dips_into_a_valley_before_its_tip(self) -> None:
        """The cube's lip sits in the valley: its top must lie well below the tip's, and nearer the upright."""
        slices = self.v["lift_fork_hook_slices_mm"]
        tops = [s[3] for s in slices]
        valley = int(np.argmin(tops))
        self.assertLess(valley, len(slices) // 2, "the valley is not next to the upright")
        self.assertGreater(tops[-1] - tops[valley], 3.0, "the hook's tip does not rise above its valley")
        self.assertGreater(self.v["lift_fork_hook_upright_mm"][3], tops[valley] + 3.0,
                           "the upright does not stand above the valley")


class CubePockets(unittest.TestCase):
    def setUp(self) -> None:
        self.v = params.values(params.load())
        self.rows = mjcf.cube_boxes(self.v)
        self.as_tuples = [(m, np.array(c), np.array(h)) for m, c, h in self.rows]

    def _solid(self, points) -> np.ndarray:
        return union_mask_at(np.array(points, dtype=float), self.as_tuples)

    def test_the_pocket_is_hollow_and_the_lips_stay(self) -> None:
        back, lip = self.v["cube_notch_pocket_xy_mm"]
        top = self.v["cube_notch_pocket_half_z_mm"]
        mid, z_lip = 0.5 * (back + lip), 0.5 * (9.5 + top)
        for sx in (1, -1):
            for sy in (1, -1):
                for sz in (1, -1):
                    pocket = [sx * mid, sy * mid, sz * z_lip]
                    x_lip = [sx * 0.5 * (lip + 22.0), sy * mid, sz * z_lip]
                    y_lip = [sx * mid, sy * 0.5 * (lip + 22.0), sz * z_lip]
                    above = [sx * mid, sy * mid, sz * 0.5 * (top + 22.0)]
                    self.assertEqual(list(self._solid([pocket, x_lip, y_lip, above])), [False, True, True, True],
                                     f"edge {sx:+d}{sy:+d}, end {sz:+d}: pocket, lips or cap wrong")

    def test_carving_removes_only_the_pockets(self) -> None:
        back, lip = self.v["cube_notch_pocket_xy_mm"]
        top = self.v["cube_notch_pocket_half_z_mm"]
        source = load_boxes()
        # Half a step off the 0.5 mm grid, so no sample lies on a face that carving can split.
        axis = np.linspace(-21.75, 21.75, 88)
        pts = np.stack(np.meshgrid(axis, axis, axis, indexing="ij"), axis=-1).reshape(-1, 3)
        before, after = union_mask_at(pts, source), self._solid(pts)
        a = np.abs(pts)
        in_pocket = (a[:, 0] > back) & (a[:, 0] < lip) & (a[:, 1] > back) & (a[:, 1] < lip) & (a[:, 2] < top)
        self.assertTrue(np.array_equal(before[~in_pocket], after[~in_pocket]), "material moved outside the pockets")
        self.assertFalse(after[in_pocket].any(), "material left inside a pocket")
        self.assertGreater(int(before[in_pocket].sum()), 0, "the SDK shell had nothing to carve")

    def test_the_carved_cube_stays_disjoint_and_d4_symmetric(self) -> None:
        self.assertLessEqual(max_pairwise_overlap_mm(self.as_tuples), 1e-6)
        self.assertEqual(d4_asymmetry_count([r for r in self.as_tuples if r[0] == "shell"]), 0)
        self.assertEqual(d4_asymmetry_count([r for r in self.as_tuples if r[0] == "rubber"]), 0)
        self.assertEqual(sum(1 for r in self.rows if r[0] == "rubber"), 8)

    def test_the_free_cube_is_built_from_the_carved_boxes(self) -> None:
        root = ET.fromstring(mjcf.build_mjcf(self.v, (), "free"))
        geoms = _geoms(root, "cube_")
        self.assertEqual(len(geoms), len(self.rows))
        scale = self.v["cube_outer_mm"] / 44.0
        for geom, (material, centre, half) in zip(geoms, self.rows):
            self.assertTrue(geom.get("name").startswith(f"cube_{material}_"))
            np.testing.assert_allclose(np.array(_floats(geom.get("pos"))) / mjcf.MM, np.array(centre) * scale, atol=1e-6)
            np.testing.assert_allclose(np.array(_floats(geom.get("size"))) / mjcf.MM, np.array(half) * scale, atol=1e-6)


class LiftTorque(unittest.TestCase):
    def setUp(self) -> None:
        self.v = params.values(params.load())
        self.root = ET.fromstring(mjcf.build_mjcf(self.v))

    def _limit(self, name: str) -> float:
        found = [a for a in self.root.iter("position") if a.get("name") == name]
        self.assertEqual(len(found), 1)
        return _floats(found[0].get("forcerange"))[1] * 1000.0

    def test_the_lift_and_head_take_their_own_limits(self) -> None:
        self.assertAlmostEqual(self._limit("lift"), self.v["lift_servo_torque_max_mNm"], places=6)
        self.assertAlmostEqual(self._limit("head"), self.v["servo_torque_max_mNm"], places=6)
        changed = ET.fromstring(mjcf.build_mjcf(params.values(params.load(), {"lift_servo_torque_max_mNm": 77.0})))
        lift = [a for a in changed.iter("position") if a.get("name") == "lift"][0]
        self.assertAlmostEqual(_floats(lift.get("forcerange"))[1] * 1000.0, 77.0, places=6)

    @unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
    def test_the_lift_can_hold_the_heaviest_cube_the_table_allows(self) -> None:
        holding = _load(HOLDING, "holding_torque")
        table = params.load()
        heaviest = dict(self.v, cube_mass_g=float(table["parameters"]["cube_mass_g"]["high"]))
        worst = max(t for _, t in holding.holding(heaviest, "lift"))
        self.assertGreater(worst, 40.0, "the holding torque looks too small to be the cube's")
        self.assertLess(worst, table["parameters"]["lift_servo_torque_max_mNm"]["low"])


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class Pickup(unittest.TestCase):
    def test_the_robot_picks_up_and_carries_a_cube_it_docks_with(self) -> None:
        trial = _load(TRIAL, "cube_pickup_trial")
        v = params.values(params.load())
        top = mjcf.lift_angle_of_height(v, v["lift_height_max_mm"])
        for offset in (-1.0, 0.0, 1.0):
            row = trial.trial(v, offset)
            self.assertEqual(row["drive_result"], "completed", row)
            # Carrying the cube the lift still reaches its target (the integral term, test_lift_integral).
            self.assertEqual(row["lift_result"], "completed", row)
            self.assertLess(top - mjcf.lift_angle_of_height(v, row["lift_height_mm"]), math.radians(v["lift_angle_tol_deg"]), row)
            self.assertGreater(row["cube_rise_mm"], trial.PICKUP_RISE_MM, row)
            self.assertGreater(row["rise_after_backing_mm"], trial.PICKUP_RISE_MM, row)
            self.assertLess(row["tilt_after_backing_deg"], 10.0, row)

    def test_free_cube_shell_contacts_use_shell_time_constant_in_mujoco(self) -> None:
        import mujoco

        v = params.values(params.load())
        tc = v["contact_time_constant_ms"] * 0.001
        model = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(v, (), "free"))
        data = mujoco.MjData(model)
        cq = int(model.joint("cube_root").qposadr[0])
        data.qpos[cq : cq + 3] = [0.042, 0.0, 0.0220]
        mujoco.mj_forward(model, data)
        seen = {}
        for i in range(data.ncon):
            con = data.contact[i]
            pair = tuple(sorted((model.geom(con.geom[0]).name, model.geom(con.geom[1]).name)))
            seen[pair] = float(con.solref[0])
        table_shell = [sr for (a, b), sr in seen.items() if ("table" in (a, b)) and any(n.startswith("cube_shell_") for n in (a, b))]
        table_rubber = [sr for (a, b), sr in seen.items() if ("table" in (a, b)) and any(n.startswith("cube_rubber_") for n in (a, b))]
        fork_shell = [sr for (a, b), sr in seen.items() if any("fork" in n for n in (a, b)) and any(n.startswith("cube_shell_") for n in (a, b))]
        self.assertGreater(len(table_shell), 0)
        self.assertGreater(len(table_rubber), 0)
        self.assertGreater(len(fork_shell), 0)
        for sr in table_shell + fork_shell:
            self.assertAlmostEqual(sr, tc, places=9)
        for sr in table_rubber:
            self.assertAlmostEqual(sr, 0.008, places=9)


if __name__ == "__main__":
    unittest.main()
