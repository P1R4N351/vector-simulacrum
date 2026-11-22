"""The rig-shaped body: geometry helpers, solids, the fidelity measure and the mesh-mode MJCF.

Needs numpy and scipy, not the rig: every shape here is made up in the test. The MuJoCo cases
compile a body built from boxes and skip without MuJoCo."""

from __future__ import annotations

import importlib.util
import json
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import _common
from simulacrum import body_mesh, mjcf, params

HAVE_SCIPY = importlib.util.find_spec("scipy") is not None
SCIPY_SKIP = "scipy is not installed here; run in a GPU host"


def box_mesh(lo, hi):
    """A closed, outward box as (vertices, triangles)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    v = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    t = np.array([q for a, b, c, d in quads for q in ((a, b, c), (a, c, d))], dtype=np.int64)
    if body_mesh.mesh_volume(v, t) < 0.0:
        t = t[:, ::-1].copy()
    return v, t


class Geometry(unittest.TestCase):
    def test_circle_fit_recovers_a_circle(self) -> None:
        rng = np.random.default_rng(1)
        a = rng.uniform(0.0, 2.0 * math.pi, 200)
        xy = np.column_stack((3.0 + 7.5 * np.cos(a), -2.0 + 7.5 * np.sin(a))) + rng.normal(0.0, 0.01, (200, 2))
        centre, radius, rms = body_mesh.circle_fit(xy)
        self.assertLess(np.abs(centre - [3.0, -2.0]).max(), 0.01)
        self.assertAlmostEqual(radius, 7.5, delta=0.01)
        self.assertLess(rms, 0.02)

    def test_end_cap_finds_both_pivots_of_a_link(self) -> None:
        # A 66 mm link 5 mm wide with rounded ends round (0, 0) and (66, -12): the lift's own shape.
        angle = math.atan2(-12.0, 66.0)
        ends = np.array([[0.0, 0.0], [66.0, -12.0]])
        t = np.linspace(-math.pi / 2, math.pi / 2, 60)
        d = np.array([math.cos(angle), math.sin(angle)])
        n = np.array([-d[1], d[0]])
        front = ends[1] + 2.5 * (np.outer(np.cos(t), d) + np.outer(np.sin(t), n))
        rear = ends[0] - 2.5 * (np.outer(np.cos(t), d) + np.outer(np.sin(t), n))
        s = np.linspace(0.0, 1.0, 80)[:, None]
        sides = np.concatenate([ends[0] + s * (ends[1] - ends[0]) + k * 2.5 * n for k in (-1, 1)])
        xz = np.concatenate((front, rear, sides))
        got_front = body_mesh.end_cap(xz, front=True)["centre_xz"]
        got_rear = body_mesh.end_cap(xz, front=False)["centre_xz"]
        self.assertLess(np.abs(np.array(got_front) - ends[1]).max(), 0.05)
        self.assertLess(np.abs(np.array(got_rear) - ends[0]).max(), 0.05)

    def test_hinge_turn_raises_the_front_and_undoes(self) -> None:
        p = np.array([[10.0, 1.0, 0.0]])
        up = body_mesh.rot_y_up(p, math.radians(30.0))
        self.assertAlmostEqual(float(up[0, 2]), 5.0, places=9)  # positive angle lifts +x towards +z
        np.testing.assert_allclose(body_mesh.rot_y_up(up, math.radians(-30.0)), p, atol=1e-12)

    def test_touchdown_has_the_known_answer(self) -> None:
        # A point 20 mm behind an axle 10 mm up, on a 10 mm wheel, reaches the table when sin(pitch) = 0.5.
        got = body_mesh.touchdown_deg(np.array([[-20.0, 0.0, 10.0]]), 0.0, 10.0, 10.0)
        # The first search step at or past the true angle, so up to one step late.
        self.assertGreaterEqual(got, 30.0 - 1e-9)
        self.assertLessEqual(got, 30.0 + body_mesh.TOUCHDOWN_STEP_DEG + 1e-9)
        self.assertEqual(body_mesh.touchdown_deg(np.array([[-5.0, 0.0, 8.0]]), 0.0, 10.0, 10.0), math.inf)

    def test_table_check_names_a_drifted_feature(self) -> None:
        v = params.values(params.load())
        neck, base = v["neck_joint_position_mm"], v["lift_base_position_mm"]
        rad = lambda x, z: {"centre_xz": [x, z]}
        feats = {"front_rims": rad(0.0, v["wheel_front_radius_mm"]),
                 "rear_rims": rad(-v["wheel_base_mm"], v["wheel_rear_radius_mm"]),
                 "head_side_frames": rad(neck[0], neck[2]),
                 "upper_link_rear": rad(base[0], base[2]),
                 "lower_link_rear": rad(base[0], base[2] - 12.0),
                 "lower_link_front": rad(base[0] + v["lift_arm_length_mm"], base[2] - 12.0)}
        self.assertEqual(body_mesh.check_against_table(v, feats), [])
        feats["head_side_frames"] = rad(neck[0], neck[2] + 2.0)
        drift = body_mesh.check_against_table(v, feats)
        self.assertEqual(len(drift), 1)
        self.assertIn("neck z", drift[0])


@unittest.skipUnless(HAVE_SCIPY, SCIPY_SKIP)
class Solids(unittest.TestCase):
    def test_voxel_surface_is_closed_outward_and_exact(self) -> None:
        solid = np.zeros((6, 6, 6), dtype=bool)
        solid[1:5, 1:5, 1:5] = True
        v, t = body_mesh.voxel_surface(solid, np.zeros(3))
        edges = {}
        for a, b, c in t:
            for e in ((a, b), (b, c), (c, a)):
                key = tuple(sorted(e))
                edges[key] = edges.get(key, 0) + 1
        self.assertEqual(set(edges.values()), {2})
        self.assertAlmostEqual(body_mesh.mesh_volume(v, t), 64 * body_mesh.VOXEL_MM ** 3, places=9)

    def test_fill_closes_a_shell_with_a_large_hole(self) -> None:
        mark = np.zeros((24, 24, 24), dtype=bool)
        mark[2:22, 2:22, 2:22] = True
        mark[3:21, 3:21, 3:21] = False   # a hollow box ...
        mark[2, 8:16, 8:16] = False      # ... with an 8-voxel hole in one wall, far wider than the closing
        filled = body_mesh.fill(mark)
        self.assertGreaterEqual(int(filled[2:22, 2:22, 2:22].sum()), int(0.98 * 20 ** 3))

    def test_fill_leaves_a_pocket_open_on_two_sides_empty(self) -> None:
        mark = np.zeros((30, 30, 30), dtype=bool)
        mark[2:28, 2:28, 2:5] = True     # floor
        mark[2:28, 2:5, 2:28] = True     # two walls facing each other, 20 voxels (10 mm) apart:
        mark[2:28, 25:28, 2:28] = True
        mark[2:5, 2:28, 2:28] = True     # and one end wall: open at the top and at the other end only
        self.assertEqual(int(body_mesh.ray_votes(mark)[15, 15, 15]), 4)
        filled = body_mesh.fill(mark)
        self.assertFalse(filled[15, 15, 15])

    def test_clean_keeps_the_body_and_drops_scraps(self) -> None:
        solid = np.zeros((40, 40, 40), dtype=bool)
        solid[5:30, 5:30, 5:30] = True    # the body
        solid[35:37, 35:37, 35:37] = True  # a loose 8-voxel speck
        solid[2, 5:30, 5:30] = True       # a one-voxel sheet beside the body, not touching it
        kept = body_mesh.clean(solid)
        self.assertFalse(kept[36, 36, 36])
        self.assertFalse(kept[2, 15, 15])
        self.assertTrue(kept[15, 15, 15])
        self.assertGreater(int(kept.sum()), int(0.95 * 25 ** 3))

    def test_tread_hull_is_shrunk_inside_the_tip_supports(self) -> None:
        a = np.linspace(0.0, 2.0 * math.pi, 64, endpoint=False)
        ring = np.column_stack((14.5 * np.cos(a), np.zeros_like(a), 14.5 + 14.5 * np.sin(a)))
        v, _ = body_mesh.clipped_hull(np.concatenate((ring, ring + [0.0, 10.0, 0.0])), 0.3, 0.5)
        # 29 less the inset; a 64-sided ring's apex drops by inset / cos(2.8 deg) = 0.5006.
        self.assertAlmostEqual(float(v[:, 2].max()), 29.0 - 0.5 / math.cos(math.pi / 64), places=6)
        self.assertLess(float(np.hypot(v[:, 0], v[:, 2] - 14.5).max()), 14.5 - 0.3)  # inside a tip support

    def test_clip_keeps_only_what_lies_inside(self) -> None:
        v, _ = box_mesh((0, 0, 0), (20, 20, 20))
        cut = body_mesh.clip_to(v, np.array([[0.0, 0.0, 1.0, -5.0]]))  # z <= 5
        self.assertIsNotNone(cut)
        self.assertAlmostEqual(body_mesh.mesh_volume(*cut), 20 * 20 * 5, places=6)
        self.assertIsNone(body_mesh.clip_to(v, np.array([[0.0, 0.0, 1.0, 5.0]])))  # z <= -5: nothing left

    def test_clipped_hull_stands_on_its_floor(self) -> None:
        a = np.linspace(0.0, 2.0 * math.pi, 64, endpoint=False)
        ring = np.column_stack((14.5 * np.cos(a), np.zeros_like(a), 14.5 + 14.5 * np.sin(a)))
        pts = np.concatenate((ring, ring + [0.0, 10.0, 0.0]))
        v, t = body_mesh.clipped_hull(pts, 0.3)
        self.assertAlmostEqual(float(v[:, 2].min()), 0.3, places=9)
        self.assertAlmostEqual(float(v[:, 2].max()), 29.0, places=6)
        self.assertGreater(body_mesh.mesh_volume(v, t), 0.0)


@unittest.skipUnless(HAVE_SCIPY, SCIPY_SKIP)
class Fidelity(unittest.TestCase):
    def setUp(self) -> None:
        self.ref = box_mesh((0, 0, 0), (20, 20, 20))

    def test_a_shape_scores_zero_against_itself(self) -> None:
        f = body_mesh.fidelity(*self.ref, [(*self.ref, "chassis")])
        self.assertEqual(f["protrusion_mm"]["max"], 0.0)
        # Two independent samplings of one surface: nearest-sample gaps run to about one spacing.
        self.assertLess(f["excess_mm"]["p95"], 1.5 * body_mesh.SAMPLE_SPACING_MM)

    def test_a_grown_piece_shows_as_excess(self) -> None:
        grown = box_mesh((-2, -2, -2), (22, 22, 22))
        f = body_mesh.fidelity(*self.ref, [(*grown, "chassis")])
        self.assertGreater(f["excess_mm"]["p50"], 1.9)
        self.assertEqual(f["protrusion_mm"]["max"], 0.0)

    def test_a_missing_piece_shows_as_protrusion(self) -> None:
        half = box_mesh((0, 0, 0), (20, 20, 10))
        f = body_mesh.fidelity(*self.ref, [(*half, "chassis")])
        self.assertGreater(f["protrusion_mm"]["max"], 9.0)
        self.assertGreater(f["outside_fraction"], 0.3)

    def test_two_pieces_that_tile_the_shape_score_zero(self) -> None:
        low, high = box_mesh((0, 0, 0), (20, 20, 10)), box_mesh((0, 0, 10), (20, 20, 20))
        f = body_mesh.fidelity(*self.ref, [(*low, "chassis"), (*high, "chassis")])
        self.assertEqual(f["protrusion_mm"]["max"], 0.0)
        self.assertLess(f["excess_mm"]["p99"], 1.5 * body_mesh.SAMPLE_SPACING_MM)


def boxes_body(folder: Path) -> dict:
    """A stand-in for a built body: one box per part, written where mjcf will look."""
    v = params.values(params.load())
    frames = {"chassis": {"origin_mm": [0.0, 0.0, 0.0], "rig_angle_rad": 0.0},
              "head": {"origin_mm": list(v["neck_joint_position_mm"]), "rig_angle_rad": 0.0},
              "lift_upper": {"origin_mm": list(v["lift_base_position_mm"]), "rig_angle_rad": -0.19},
              "lift_lower": {"origin_mm": [v["lift_base_position_mm"][0], 0.0, v["lift_base_position_mm"][2] - 12.0],
                             "rig_angle_rad": -0.19},
              "fork": {"origin_mm": [23.7, 0.0, 32.6], "rig_angle_rad": 0.0}}
    parts = {}
    for name in body_mesh.PARTS:
        bv, bt = box_mesh((-5, -5, 2), (5, 5, 12)) if name == "chassis" else box_mesh((-3, -3, -3), (3, 3, 3))
        sha_c = body_mesh.write_obj(folder / f"{name}_c00.obj", bv, bt)
        sha_v = body_mesh.write_obj(folder / f"{name}_visual.obj", bv, bt)
        parts[name] = {"collision": [{"file": f"{name}_c00.obj", "friction": "chassis", "sha256": sha_c}],
                       "visual": [{"file": f"{name}_visual.obj", "sha256": sha_v}]}
    parts["chassis"]["collision"].append(dict(parts["chassis"]["collision"][0], friction="tread"))
    body = {"rig": dict(body_mesh.RIG), "frames": frames, "parts": parts}
    (folder / body_mesh.MANIFEST).write_text(json.dumps(body))
    return dict(body, dir=str(folder))


class MeshModeModel(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.folder = Path(self.tmp.name)
        self.v = params.values(params.load())
        self.body = boxes_body(self.folder)
        self.xml = mjcf.build_mjcf(self.v, body=self.body)
        self.root = ET.fromstring(self.xml)

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_default_model_is_unchanged(self) -> None:
        self.assertEqual(mjcf.build_mjcf(self.v), mjcf.build_mjcf(self.v, body=None))
        self.assertNotIn("<mesh", mjcf.build_mjcf(self.v))

    def test_load_refuses_a_changed_file(self) -> None:
        self.assertEqual(body_mesh.load(self.folder)["dir"], str(self.folder.resolve()))
        (self.folder / "fork_c00.obj").write_text("v 0 0 0\n")
        with self.assertRaises(RuntimeError):
            body_mesh.load(self.folder)

    def test_lower_links_are_a_body_held_to_the_lift(self) -> None:
        names = {b.get("name") for b in self.root.iter("body")}
        self.assertIn("lift_lower", names)
        follow = [e for e in self.root.iter("joint") if e.get("name") == "lift_lower_follow"]
        self.assertEqual(len(follow), 1)
        self.assertEqual((follow[0].get("joint1"), follow[0].get("joint2")), ("lift_lower_joint", "lift_joint"))
        self.assertEqual(follow[0].get("polycoef"), "0 1 0 0 0")

    def test_primitives_keep_mass_but_lose_contact(self) -> None:
        geoms = {g.get("name"): g for g in self.root.iter("geom")}
        for name in ("chassis_geom", "head_geom", "lift_arm_geom", "lift_fork_geom", "lift_lower_geom"):
            self.assertEqual((geoms[name].get("contype"), geoms[name].get("conaffinity")), ("0", "0"), name)
        self.assertNotIn("fork_side_left", geoms)
        arm = float(geoms["lift_arm_geom"].get("mass")) + float(geoms["lift_lower_geom"].get("mass"))
        self.assertAlmostEqual(arm, mjcf.derive(self.v)["masses_g"]["arm"] * mjcf.GRAM, places=12)

    def test_pieces_carry_the_table_friction(self) -> None:
        geoms = {g.get("name"): g for g in self.root.iter("geom")}
        self.assertEqual(float(geoms["chassis_hull_0"].get("friction").split()[0]), self.v["chassis_friction"])
        self.assertEqual(float(geoms["chassis_hull_1"].get("friction").split()[0]), self.v["tread_friction"])
        for part in body_mesh.PARTS:
            self.assertEqual(geoms[f"{part}_hull_0"].get("contype"), "1")
            self.assertEqual(geoms[f"{part}_visual"].get("contype"), "0")

    @unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
    def test_compiles_with_the_same_mass(self) -> None:
        import mujoco

        mesh = mujoco.MjModel.from_xml_string(self.xml)
        prim = mujoco.MjModel.from_xml_string(mjcf.build_mjcf(self.v))
        self.assertAlmostEqual(float(mesh.body_subtreemass[1]), float(prim.body_subtreemass[1]), places=12)

    @unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
    def test_the_real_body_keeps_the_tables_centre_of_mass(self) -> None:
        import mujoco

        m = mujoco.MjModel.from_xml_string(self.xml)
        d = mujoco.MjData(m)
        low = mjcf.derive(self.v)["lift_angle_min"]
        for name, angle in (("lift_joint", low), ("wrist_joint", -low), ("lift_lower_joint", low)):
            d.joint(name).qpos[0] = angle
        mujoco.mj_forward(m, d)
        com = d.subtree_com[m.body("chassis").id] / mjcf.MM
        self.assertAlmostEqual(float(com[0]), self.v["com_x_mm"], delta=0.05)
        self.assertAlmostEqual(float(com[2]), self.v["com_z_mm"], delta=0.05)

    @unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
    def test_lower_links_follow_the_lift(self) -> None:
        import mujoco
        from simulacrum.robot import VectorSim

        sim = VectorSim(body=self.body)
        sim.set_lift_height(self.v["lift_height_max_mm"])
        lower = sim.model.jnt_qposadr[mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_JOINT, "lift_lower_joint")]
        self.assertAlmostEqual(float(sim.data.qpos[lower]), float(sim.data.qpos[sim._q["lift"]]), delta=1e-3)
        self.assertGreater(sim.lift_height_mm(), self.v["lift_height_max_mm"] - 2.0)


if __name__ == "__main__":
    unittest.main()
