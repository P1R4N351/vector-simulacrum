"""Printed replacement parts change only what they replace, by the numbers their meshes give..

Synthetic meshes (built here, with known answers) test the machinery everywhere; the registered parts,
which are fetched from their sources and never kept in the repository, are tested where they are cached
(an optional input). Tolerances:
  mass, centre, inertia of a box     1e-9 relative (closed forms)
  open-loop travel against radius    0.3 % (the treads are a coupled pair of cylinders on a soft contact)
  resting height against extra tread 0.05 mm
"""

from __future__ import annotations

import hashlib
import math
import os
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import _common
from simulacrum import mjcf, params, printed_parts as pp

if _common.HAVE_MUJOCO:
    import mujoco
    from simulacrum import robot

CACHED_SKIP = (_common.OPTIONAL_SKIP_PREFIX + "the registered printed parts are not cached here "
               "(python -c 'from simulacrum import printed_parts as p; [p.fetch_part(n) for n in p.PARTS]')")
TRAVEL_TOL = 0.003
REST_TOL_MM = 0.05
STL_TOL_MM = 1e-5  # STL stores float32: about 2e-6 mm at 15 mm


def box(lo, hi) -> np.ndarray:
    """A closed box, faces wound outward."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    v = np.array([[(lo, hi)[i][0], (lo, hi)[j][1], (lo, hi)[k][2]] for i in (0, 1) for j in (0, 1) for k in (0, 1)])
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return v[np.array(faces)]


def annulus(inner: float, outer: float, width: float, segments: int = 120) -> np.ndarray:
    """A closed ring about z, from z = 0 to width, faces wound outward."""
    a = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    ring = lambda r, z: np.stack((r * np.cos(a), r * np.sin(a), np.full_like(a, z)), axis=1)
    o0, o1, i0, i1 = ring(outer, 0.0), ring(outer, width), ring(inner, 0.0), ring(inner, width)
    tris = []
    for k in range(segments):
        n = (k + 1) % segments
        tris += [(o0[k], o0[n], o1[n]), (o0[k], o1[n], o1[k]),      # outside
                 (i0[k], i1[n], i0[n]), (i0[k], i1[k], i1[n]),      # inside
                 (o1[k], o1[n], i1[n]), (o1[k], i1[n], i1[k]),      # top
                 (o0[k], i0[n], o0[n]), (o0[k], i0[k], i0[n])]      # bottom
    return np.array(tris)


# Where forks7.stl lies on its print bed (measured: x -72.005..-24.005, y from -36.792, z -27.821..-15.613).
BED = np.array([-72.0, -36.79, -27.82])


def fork_on_the_bed() -> np.ndarray:
    """A U fork laid out like forks7.stl: crossbar 48 wide (x) and 8.5 high (y, the table's crossbar height),
    12 deep (z); two side plates 3 wide rising from 2 mm above it, 4 deep at the front."""
    parts = [box((0, 0, 0), (48, 8.5, 12)), box((0, 10.5, 8), (3, 30.5, 12)), box((45, 10.5, 8), (48, 30.5, 12))]
    return np.concatenate(parts) + BED


def to_stl(tris: np.ndarray) -> bytes:
    record = np.zeros(len(tris), dtype=[("normal", "<f4", 3), ("triangle", "<f4", (3, 3)), ("attribute", "<u2")])
    record["triangle"] = tris
    return b"\0" * 80 + len(tris).to_bytes(4, "little") + record.tobytes()


def to_ascii_stl(tris: np.ndarray) -> bytes:
    lines = ["solid test"]
    for t in tris:
        lines += ["facet normal 0 0 0", "outer loop"] + [f"vertex {float(p[0])!r} {float(p[1])!r} {float(p[2])!r}" for p in t] + ["endloop", "endfacet"]
    return ("\n".join(lines + ["endsolid test"]) + "\n").encode()


TREAD = (26.0, 28.5, 11.0)   # synthetic band: inner radius, outer radius, width (mm)
THIN_TREAD = (26.0, 26.8, 11.0)   # 0.8 mm thick: rolls on a smaller radius than the 1.5 mm stock tread
RIM_FRONT, RIM_REAR = (12.0, 13.2, 8.0), (8.0, 9.4, 8.0)
SYNTHETIC = {
    "fork:synthetic": ("fork", {"fork": fork_on_the_bed()}, {"registration": "bed"}),
    "treads:synthetic": ("treads", {"left": annulus(*TREAD), "right": annulus(*TREAD)}, {}),
    "treads:synthetic-thin": ("treads", {"left": annulus(*THIN_TREAD), "right": annulus(*THIN_TREAD)}, {}),
    "rims:synthetic": ("rims", {"front": annulus(*RIM_FRONT), "rear": annulus(*RIM_REAR)}, {}),
}


class WithSyntheticParts(unittest.TestCase):
    """Registers three synthetic parts and caches them in a temporary directory."""

    def setUp(self) -> None:
        self.tmp = tempfile.mkdtemp()
        self.old_env = os.environ.get("VECTOR_PRINT_PARTS")
        os.environ["VECTOR_PRINT_PARTS"] = self.tmp
        for name, (kind, meshes, extra) in SYNTHETIC.items():
            files = {}
            for role, tris in meshes.items():
                data = to_stl(tris)
                path = Path(self.tmp) / name.replace(":", "_") / f"{role}.stl"
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
                files[role] = ("synthetic", hashlib.sha256(data).hexdigest())
            pp.PARTS[name] = {"kind": kind, "files": files, "licence": "test", "page": "test",
                              "source": {"kind": "test"}, **extra}
        self.v = params.values(params.load())
        self.stock = mjcf.build_mjcf(self.v)

    def tearDown(self) -> None:
        for name in SYNTHETIC:
            pp.PARTS.pop(name, None)
        if self.old_env is None:
            os.environ.pop("VECTOR_PRINT_PARTS", None)
        else:
            os.environ["VECTOR_PRINT_PARTS"] = self.old_env
        shutil.rmtree(self.tmp)


def _find(root: ET.Element, tag: str, name: str) -> ET.Element:
    for elem in root.iter(tag):
        if elem.get("name") == name:
            return elem
    raise AssertionError(f"no <{tag} name={name!r}>")


def _floats(text: str) -> list:
    return [float(x) for x in text.split()]


class Meshes(unittest.TestCase):
    def test_box_mass_centre_and_inertia_match_the_closed_forms(self) -> None:
        props = pp.mass_properties(box((5, -3, 2), (15, 17, 32)), density_g_cm3=1.0)
        m = 6.0  # 10 x 20 x 30 mm at 1 g/cm^3
        self.assertAlmostEqual(props["mass_g"], m, places=9)
        np.testing.assert_allclose(props["com_mm"], (10, 7, 17), rtol=1e-12)
        expected = np.diag([m * (20 ** 2 + 30 ** 2) / 12, m * (10 ** 2 + 30 ** 2) / 12, m * (10 ** 2 + 20 ** 2) / 12])
        np.testing.assert_allclose(props["inertia_gmm2"], expected, rtol=1e-9, atol=1e-9)

    def test_density_and_fill_scale_the_mass_and_not_the_centre(self) -> None:
        a = pp.mass_properties(box((0, 0, 0), (10, 10, 10)), 1.24, 1.0)
        b = pp.mass_properties(box((0, 0, 0), (10, 10, 10)), 1.24, 0.5)
        self.assertAlmostEqual(a["mass_g"], 1.24, places=9)
        self.assertAlmostEqual(b["mass_g"], 0.62, places=9)
        np.testing.assert_allclose(a["com_mm"], b["com_mm"])

    def test_an_inside_out_mesh_is_refused(self) -> None:
        with self.assertRaises(AssertionError):
            pp.mass_properties(box((0, 0, 0), (1, 1, 1))[:, ::-1], 1.0)

    def test_binary_and_ascii_stl_read_the_same(self) -> None:
        tris = fork_on_the_bed()
        np.testing.assert_allclose(pp.read_stl(to_stl(tris)), tris, atol=1e-5)
        np.testing.assert_allclose(pp.read_stl(to_ascii_stl(tris)), tris, atol=1e-12)

    def test_voxels_fill_a_box_and_the_boxes_cover_them_exactly(self) -> None:
        occ, corner = pp.voxelise(box((0, 0, 0), (10, 6, 4)), 1.0)
        self.assertEqual(int(occ.sum()), 240)
        boxes = pp.boxes_from_grid(occ, corner, 1.0)
        self.assertEqual(len(boxes), 1)
        np.testing.assert_allclose(boxes[0][0], (5, 3, 2))
        np.testing.assert_allclose(boxes[0][1], (5, 3, 2))

    def test_the_fork_opening_stays_open(self) -> None:
        occ, corner = pp.voxelise(fork_on_the_bed(), 1.0)
        boxes = pp.boxes_from_grid(occ, corner, 1.0)
        self.assertAlmostEqual(sum(8 * float(np.prod(h)) for _, h in boxes), float(occ.sum()), places=9)
        mid = BED + (24.0, 20.0, 10.0)
        self.assertFalse(any(np.all(np.abs(mid - c) < h) for c, h in boxes), "a box closes the fork's opening")
        side = BED + (1.5, 20.0, 10.0)
        self.assertTrue(any(np.all(np.abs(side - c) < h) for c, h in boxes), "a side plate went missing")

    def test_a_ring_reports_its_radii_and_width(self) -> None:
        shape = pp.ring(annulus(*TREAD))
        self.assertEqual(shape["axis"], 2)
        self.assertAlmostEqual(shape["inner_mm"], TREAD[0], places=9)
        self.assertAlmostEqual(shape["outer_mm"], TREAD[1], places=9)
        self.assertAlmostEqual(shape["width_mm"], TREAD[2], places=9)


class DefaultUnchanged(unittest.TestCase):
    def test_no_parts_hands_back_the_same_string(self) -> None:
        v = params.values(params.load())
        for cube in (None, "lift", "free"):
            xml = mjcf.build_mjcf(v) if cube is None else mjcf.build_mjcf(v, (), cube)
            self.assertIs(pp.apply(xml, v, ()), xml)
            self.assertEqual(pp.build_mjcf(v, (), cube=cube), xml)


class Registry(unittest.TestCase):
    def test_every_part_names_its_source_licence_and_file_hashes(self) -> None:
        for name, part in pp.PARTS.items():
            self.assertIn(part["kind"], pp.KINDS, name)
            self.assertTrue(name.startswith(part["kind"] + ":"), name)
            self.assertTrue(part["licence"] and part["page"].startswith("https://"), name)
            self.assertIn(part["source"]["kind"], ("archive.org", "printables"), name)
            roles = {"fork": {"fork"}, "rims": {"front", "rear"}, "treads": {"left", "right"}}[part["kind"]]
            self.assertEqual(set(part["files"]), roles, name)
            for ref, sha in part["files"].values():
                self.assertRegex(sha, r"^[0-9a-f]{64}$", name)

    def test_one_part_of_each_kind_and_no_unknown_parts(self) -> None:
        with self.assertRaises(AssertionError):
            pp.resolve(("fork:dauler", "fork:10zz"))
        with self.assertRaises(AssertionError):
            pp.resolve(("fork:nobody",))
        self.assertEqual(pp.resolve(("fork:dauler", "treads:madizmo")), {"fork": "fork:dauler", "treads": "treads:madizmo"})


class SyntheticParts(WithSyntheticParts):
    def test_a_cached_file_with_another_hash_is_refused(self) -> None:
        path = Path(self.tmp) / "treads_synthetic" / "left.stl"
        path.write_bytes(to_stl(annulus(26.0, 29.0, 11.0)))
        with self.assertRaises(ValueError):
            pp.load("treads:synthetic")

    def test_the_printed_fork_takes_over_contact_and_mass(self) -> None:
        report = pp.part_report(self.v, ("fork:synthetic",))["fork"]
        root = ET.fromstring(pp.apply(self.stock, self.v, ("fork:synthetic",)))
        fork = _find(root, "body", "lift_fork")
        for geom in fork.findall("geom"):
            name = geom.get("name")
            if name.startswith(("lift_fork_geom", "fork_side_")):
                self.assertEqual((geom.get("contype"), geom.get("conaffinity"), geom.get("mass")), ("0", "0", "0"), name)
        printed = [g for g in fork.findall("geom") if g.get("name").startswith("printed_fork_")]
        self.assertEqual(len(printed), len(report["boxes"]))
        self.assertTrue(all(g.get("contype") == "1" and g.get("mass") == "0" for g in printed))
        inertial = fork.find("inertial")
        self.assertAlmostEqual(float(inertial.get("mass")) / mjcf.GRAM, 1.24 * (48 * 8.5 * 12 + 2 * 3 * 20 * 4) * 1e-3, places=6)
        np.testing.assert_allclose(_floats(inertial.get("pos")), report["com_mm"] * mjcf.MM, atol=1e-9)

    def test_the_fork_lands_where_forks7_is_registered(self) -> None:
        crossbar = pp.fork_in_wrist(self.v, fork_on_the_bed()[:12])
        lo, hi = crossbar.reshape(-1, 3).min(0), crossbar.reshape(-1, 3).max(0)
        # Centred across the robot, and its bottom and top where the table puts the crossbar's.
        self.assertAlmostEqual(lo[1] + hi[1], 0.0, delta=0.05)
        self.assertAlmostEqual(hi[1] - lo[1], 48.0, places=9)
        top = self.v["lift_xbar_top_wrt_wrist_mm"]
        self.assertAlmostEqual(lo[2], top - self.v["lift_xbar_height_mm"], delta=0.05)
        self.assertAlmostEqual(hi[2], top, delta=0.05)

    def test_treads_set_rolling_radius_width_and_the_rear_gear(self) -> None:
        root = ET.fromstring(pp.apply(self.stock, self.v, ("treads:synthetic",)))
        thick = TREAD[1] - TREAD[0]
        rims = (0.5 * pp.STOCK_RIM_DIAMETER_MM[0], 0.5 * pp.STOCK_RIM_DIAMETER_MM[1])
        for name, rim in (("wheel_fl", rims[0]), ("wheel_fr", rims[0]), ("wheel_rl", rims[1]), ("wheel_rr", rims[1])):
            geom = _find(root, "body", name).find("geom")
            self.assertAlmostEqual(float(geom.get("size")) / mjcf.MM, rim + thick, delta=STL_TOL_MM)
            ends = _floats(geom.get("fromto"))
            self.assertAlmostEqual((ends[4] - ends[1]) / mjcf.MM, TREAD[2], places=6)
        for tread in ("left_tread", "right_tread"):
            gear = _floats(_find(root, "joint", tread).get("polycoef"))[1]
            self.assertAlmostEqual(gear, (rims[0] + thick) / (rims[1] + thick), delta=1e-6)
        gap = self.v["tip_support_clearance_mm"]
        self.assertAlmostEqual(float(_find(root, "geom", "tip_front_l").get("size")) / mjcf.MM, rims[0] + thick - gap, delta=STL_TOL_MM)

    def test_tread_mass_moves_from_the_chassis_to_the_wheels_and_runs(self) -> None:
        report = pp.part_report(self.v, ("treads:synthetic",))
        root = ET.fromstring(pp.apply(self.stock, self.v, ("treads:synthetic",)))
        stock_root = ET.fromstring(self.stock)
        chassis = lambda r: float(_find(r, "body", "chassis").find("inertial").get("mass")) / mjcf.GRAM
        runs = sum(report["treads"][s]["share_g"]["runs"] for s in ("left", "right"))
        self.assertAlmostEqual(chassis(root), chassis(stock_root) - 2 * report["stock_tread"]["mass_g"] + runs, places=6)
        front = float(_find(root, "body", "wheel_fl").find("inertial").get("mass")) / mjcf.GRAM
        self.assertAlmostEqual(front, self.v["wheel_mass_g"] + report["treads"]["left"]["share_g"]["front"], places=6)

    def _body_mode(self) -> str:
        """The stock model marked the way the body-mesh mode marks its contact pieces."""
        root = ET.fromstring(self.stock)
        ET.SubElement(_find(root, "body", "lift_fork"), "geom", name="fork_hull_0", type="box", size="0.001 0.001 0.001")
        ET.SubElement(_find(root, "body", "chassis"), "geom", name="chassis_hull_0", type="box", size="0.001 0.001 0.001")
        return ET.tostring(root, encoding="unicode")

    def test_on_a_body_mesh_model_a_fork_and_smaller_wheels_are_refused(self) -> None:
        body_mode = self._body_mode()
        with self.assertRaises(AssertionError):
            pp.apply(body_mode, self.v, ("fork:synthetic",))
        with self.assertRaises(AssertionError):
            pp.apply(body_mode, self.v, ("treads:synthetic-thin",))
        pp.apply(body_mode, self.v, ("treads:synthetic",))           # larger: the hulls stay clear
        pp.apply(body_mode, self.v, ("rims:synthetic",))
        pp.apply(self.stock, self.v, ("treads:synthetic-thin",))      # the stock model has no hulls to land

    def test_tread_masses_follow_the_wheel_bodies_not_the_table(self) -> None:
        moved = ET.fromstring(self.stock)
        dy = 0.95  # the refit moves the treads from 23.0 to 23.95 mm off the middle
        for name, sign in (("wheel_fl", 1), ("wheel_rl", 1), ("wheel_fr", -1), ("wheel_rr", -1)):
            body = _find(moved, "body", name)
            x, y, z = _floats(body.get("pos"))
            body.set("pos", mjcf.vec((x, y + sign * dy * mjcf.MM, z)))
        chassis = lambda xml: _find(ET.fromstring(pp.apply(xml, self.v, ("treads:synthetic",))), "body", "chassis").find("inertial")
        here, there = chassis(self.stock), chassis(ET.tostring(moved, encoding="unicode"))
        report = pp.part_report(self.v, ("treads:synthetic",))
        stock, runs = report["stock_tread"]["share_g"], report["treads"]["left"]["share_g"]["runs"]
        m = 2 * (runs - stock["front"] - stock["rear"] - stock["runs"])  # both sides, g
        y0 = 0.5 * self.v["tread_spacing_mm"]
        expected = m * ((y0 + dy) ** 2 - y0 ** 2) * mjcf.GRAM * mjcf.MM ** 2
        ixx = lambda e: _floats(e.get("diaginertia"))[0]
        self.assertAlmostEqual(ixx(there) - ixx(here), expected, delta=1e-12)
        self.assertAlmostEqual(_floats(there.get("pos"))[1], 0.0, delta=1e-12)

    def test_rims_set_the_wheel_mass_and_radius(self) -> None:
        report = pp.part_report(self.v, ("rims:synthetic",))["rims"]
        root = ET.fromstring(pp.apply(self.stock, self.v, ("rims:synthetic",)))
        body = _find(root, "body", "wheel_rl")
        self.assertAlmostEqual(float(body.find("inertial").get("mass")) / mjcf.GRAM, report["rear"]["mass_g"], places=9)
        stock_thick = self.v["wheel_rear_radius_mm"] - 0.5 * pp.STOCK_RIM_DIAMETER_MM[1]
        self.assertAlmostEqual(float(body.find("geom").get("size")) / mjcf.MM, RIM_REAR[1] + stock_thick, places=6)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class SyntheticPartsInMuJoCo(WithSyntheticParts):
    def test_vector_sim_puts_the_stock_builder_back_even_when_construction_fails(self) -> None:
        stock = mjcf.build_mjcf
        with self.assertRaises(AssertionError):
            pp.vector_sim(parts=("treads:synthetic",), control_level="no such level")
        self.assertIs(mjcf.build_mjcf, stock)
        sim = pp.vector_sim(parts=("treads:synthetic",))
        self.assertIs(mjcf.build_mjcf, stock)
        self.assertEqual(sim.parts, ("treads:synthetic",))
        self.assertIsInstance(sim, robot.VectorSim)

    def test_the_compiled_mass_is_the_stock_less_its_treads_plus_the_printed_ones(self) -> None:
        report = pp.part_report(self.v, ("treads:synthetic", "fork:synthetic"))
        model = mujoco.MjModel.from_xml_string(pp.apply(self.stock, self.v, ("treads:synthetic", "fork:synthetic")))
        expected = (self.v["total_mass_g"] - 2 * report["stock_tread"]["mass_g"]
                    + report["treads"]["left"]["mass_g"] + report["treads"]["right"]["mass_g"]
                    - report["fork"]["replaces_mass_g"] + report["fork"]["mass_g"])
        self.assertAlmostEqual(float(model.body_mass.sum()) / mjcf.GRAM, expected, places=6)

    def _open_loop(self, parts) -> tuple:
        sim = pp.vector_sim(parts=parts)
        x0, _, z0 = sim.pose_mm()
        sim.set_wheel_motors(60.0, 60.0)
        sim.advance(3.0)
        return sim.pose_mm()[0] - x0, z0

    def test_open_loop_travel_scales_with_the_rolling_radius(self) -> None:
        stock, z_stock = self._open_loop(())
        printed, z_printed = self._open_loop(("treads:synthetic",))
        radius = 0.5 * pp.STOCK_RIM_DIAMETER_MM[0] + TREAD[1] - TREAD[0]
        self.assertAlmostEqual(printed / stock, radius / self.v["wheel_front_radius_mm"], delta=TRAVEL_TOL)
        self.assertAlmostEqual(z_printed - z_stock, radius - self.v["wheel_front_radius_mm"], delta=REST_TOL_MM)

    def _lift_sag_mm(self, parts) -> float:
        sim = pp.vector_sim(parts=parts)
        sim.set_lift_height(self.v["lift_height_max_mm"])
        sim.advance(1.0)
        return self.v["lift_height_max_mm"] - sim.lift_height_mm()

    def test_the_lift_sags_more_under_a_heavier_fork_and_less_under_a_lighter_one(self) -> None:
        stock = self._lift_sag_mm(())
        heavy = pp.part_report(self.v, ("fork:synthetic",))["fork"]["mass_g"]
        self.assertGreater(heavy, pp.mjcf.derive(self.v)["masses_g"]["fork"], "fixture must be heavier than stock")
        self.assertGreater(self._lift_sag_mm(("fork:synthetic",)), stock)
        saved = pp.DEFAULT_MATERIAL["fork"]
        try:
            pp.DEFAULT_MATERIAL["fork"] = ("PLA", 0.25)
            self.assertLess(self._lift_sag_mm(("fork:synthetic",)), stock)
        finally:
            pp.DEFAULT_MATERIAL["fork"] = saved


def _cached() -> bool:
    return all(pp._cached(n, r).is_file() for n, p in pp.PARTS.items() if p["source"]["kind"] != "test" for r in p["files"])


@unittest.skipUnless(_cached(), CACHED_SKIP)
class RegisteredParts(unittest.TestCase):
    def setUp(self) -> None:
        self.v = params.values(params.load())

    def test_the_stock_shaped_rims_measure_the_recorded_diameters(self) -> None:
        for name in ("rims:dauler-stock", "rims:10zz"):
            report = pp.part_report(self.v, (name,))["rims"]
            self.assertAlmostEqual(2 * report["front"]["outer_mm"], pp.STOCK_RIM_DIAMETER_MM[0], delta=0.05, msg=name)
            self.assertAlmostEqual(2 * report["rear"]["outer_mm"], pp.STOCK_RIM_DIAMETER_MM[1], delta=0.05, msg=name)

    def test_the_printed_bands_are_wider_and_thicker_than_the_stock_model(self) -> None:
        for name in ("treads:dauler-obar", "treads:madizmo"):
            band = pp.part_report(self.v, (name,))["treads"]["left"]
            self.assertAlmostEqual(band["width_mm"], 11.6, delta=0.05, msg=name)
            self.assertGreater(band["thickness_mm"], self.v["wheel_front_radius_mm"] - 0.5 * pp.STOCK_RIM_DIAMETER_MM[0])

    def test_the_10zz_hook_has_one_clear_orientation(self) -> None:
        fork = pp.part_report(self.v, ("fork:10zz",))["fork"]
        self.assertGreaterEqual(fork["registration_overlap"], pp.MATCH_MARGIN * fork["registration_runner_up"])

    @unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
    def test_every_registered_part_loads_and_keeps_the_robot_on_its_wheels(self) -> None:
        for name in pp.PARTS:
            sim = pp.vector_sim(parts=(name,))
            self.assertFalse(sim.sim_truth()["fallen"], name)
            self.assertGreater(sim.pose_mm()[2], -0.1, name)


if __name__ == "__main__":
    unittest.main()
