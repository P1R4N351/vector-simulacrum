"""Tests for the Vector dock (charger) surface model, MJCF scene, and physics."""

from __future__ import annotations

import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

import _common
from simulacrum import dock, mjcf, params

ROOT = _common.ROOT
HP_OBJ = ROOT / "web" / "public" / "robot" / "sdk" / "Vic_charger_hp_geo.obj"
MAX_OBJ_LINES = 200_000


def _quat_to_rot(quat: tuple[float, float, float, float]) -> np.ndarray:
    assert len(quat) == 4, "quaternion must have 4 components"
    w, x, y, z = quat
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    assert abs(norm - 1.0) < 1e-4, "quaternion must be unit length"
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)


def _obb_overlap_mm(box_a: dock.DockBox, box_b: dock.DockBox,
                    offset_b: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> float:
    ca = np.array(box_a.center_mm, dtype=float)
    cb = np.array(box_b.center_mm, dtype=float) + np.array(offset_b, dtype=float)
    ha, hb = np.array(box_a.half_mm, dtype=float), np.array(box_b.half_mm, dtype=float)
    ra, rb = _quat_to_rot(box_a.quat), _quat_to_rot(box_b.quat)
    assert ha.min() > 0.0 and hb.min() > 0.0, "half extents must be positive"
    axes = [ra[:, i] for i in range(3)] + [rb[:, j] for j in range(3)]
    for i in range(3):
        for j in range(3):
            axes.append(np.cross(ra[:, i], rb[:, j]))
    min_pen = 1e9
    delta = cb - ca
    for axis in axes:
        length = float(np.linalg.norm(axis))
        if length < 1e-9:
            continue
        unit = axis / length
        proj = float(np.sum(np.abs(unit @ ra) * ha) + np.sum(np.abs(unit @ rb) * hb))
        gap = abs(float(delta @ unit)) - proj
        if gap >= -1e-6:
            return 0.0
        min_pen = min(min_pen, -gap)
    return float(min_pen)


def _box_corners_mm(box: dock.DockBox) -> np.ndarray:
    rot = _quat_to_rot(box.quat)
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    assert signs.shape == (8, 3), "corner template must be 8x3"
    local = signs * np.array(box.half_mm, dtype=float)
    return np.array(box.center_mm, dtype=float) + (rot @ local.T).T


def _load_hp_obj_tris(obj_path: Path) -> tuple[np.ndarray, np.ndarray]:
    assert obj_path.is_file(), f"missing obj file: {obj_path}"
    verts: list[list[float]] = []
    tris: list[list[int]] = []
    lines = obj_path.read_text(encoding="utf-8").splitlines()
    assert len(lines) <= MAX_OBJ_LINES, "OBJ line limit exceeded"
    for line in lines[:MAX_OBJ_LINES]:
        if line.startswith("v "):
            _, xs, ys, zs = line.split()[:4]
            verts.append([float(xs), float(ys), float(zs)])
        elif line.startswith("f "):
            idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:]]
            for k in range(1, len(idx) - 1):
                tris.append([idx[0], idx[k], idx[k + 1]])
    charger_pts = dock.obj_to_charger_mm(np.array(verts, dtype=float))
    return charger_pts, charger_pts[np.array(tris, dtype=int)]


def _z_ramp_box_at(ramp_boxes: list[dock.DockBox], x_mm: float, y_mm: float) -> float:
    best = -1e9
    for box in ramp_boxes:
        rot = _quat_to_rot(box.quat)
        ex, _, ez = rot[:, 0]
        nx, _, nz = rot[:, 2]
        assert abs(ex) > 1e-6, f"vertical ramp x-axis on {box.part}"
        u = (x_mm - box.center_mm[0] - box.half_mm[2] * nx) / ex
        if abs(u) <= box.half_mm[0] and abs(y_mm - box.center_mm[1]) <= box.half_mm[1]:
            best = max(best, box.center_mm[2] + u * ez + box.half_mm[2] * nz)
    return best


def _tread_lane_mae_mm(ramp_boxes: list[dock.DockBox], tri_verts: np.ndarray) -> tuple[int, float, float]:
    mask = ((tri_verts[:, :, 0].max(axis=1) >= 2.0) & (tri_verts[:, :, 0].min(axis=1) <= 78.0)
            & (np.abs(tri_verts[:, :, 1]).min(axis=1) <= 31.0) & (tri_verts[:, :, 2].max(axis=1) <= 18.0))
    sub_tris = tri_verts[mask]
    assert len(sub_tris) > 0, "no ramp triangles matched"
    a, b, d = sub_tris[:, 0, :2], sub_tris[:, 1, :2], sub_tris[:, 2, :2]
    v0, v1 = d - a, b - a
    d00, d01, d11 = (v0 * v0).sum(axis=1), (v0 * v1).sum(axis=1), (v1 * v1).sum(axis=1)
    denom = d00 * d11 - d01 * d01
    ok = np.abs(denom) > 1e-12
    inv = np.where(ok, 1.0 / np.where(ok, denom, 1.0), 0.0)
    errs: list[float] = []
    for x in np.linspace(5.0, 74.0, 36):
        for y in (-24.0, -20.0, 20.0, 24.0):
            v2 = np.array([x, y]) - a
            d02, d12 = (v0 * v2).sum(axis=1), (v1 * v2).sum(axis=1)
            u, v = (d11 * d02 - d01 * d12) * inv, (d00 * d12 - d01 * d02) * inv
            hit = ok & (u >= -1e-6) & (v >= -1e-6) & (u + v <= 1.0 + 1e-6)
            if not hit.any():
                continue
            zs = ((1.0 - u[hit] - v[hit]) * sub_tris[hit, 0, 2]
                  + v[hit] * sub_tris[hit, 1, 2] + u[hit] * sub_tris[hit, 2, 2])
            errs.append(abs(float(zs.max()) - _z_ramp_box_at(ramp_boxes, float(x), y)))
    return len(errs), float(np.mean(errs)), float(np.max(errs))


class DockSurfaceCsv(unittest.TestCase):
    def test_schema_envelope_and_bilateral_symmetry(self) -> None:
        boxes = dock.load_surface_boxes()
        self.assertEqual(len(boxes), 13)
        roles: dict[str, int] = {}
        by_part: dict[str, dock.DockBox] = {}
        for box in boxes:
            roles[box.role] = roles.get(box.role, 0) + 1
            by_part[box.part] = box
            corners = _box_corners_mm(box)
            self.assertGreaterEqual(float(corners[:, 0].min()), -2.0)
            self.assertLessEqual(float(corners[:, 0].max()), 114.5)
            self.assertLessEqual(float(np.abs(corners[:, 1]).max()), 44.5)
            self.assertGreaterEqual(float(corners[:, 2].min()), -0.1)
            self.assertLessEqual(float(corners[:, 2].max()), 79.0)
        self.assertEqual(roles, {"ramp": 3, "guide": 6, "shell": 2, "pin": 2})
        for stem in ("funnel", "rail", "pin"):
            left, right = by_part[f"{stem}_l"], by_part[f"{stem}_r"]
            self.assertAlmostEqual(left.center_mm[0], right.center_mm[0], places=6)
            self.assertAlmostEqual(left.center_mm[1], -right.center_mm[1], places=6)
            self.assertAlmostEqual(left.center_mm[2], right.center_mm[2], places=6)
            self.assertEqual(left.half_mm, right.half_mm)
            lw, lx, ly, lz = left.quat
            for a, b in zip((lw, -lx, ly, -lz), right.quat):
                self.assertAlmostEqual(a, b, places=6)

    def test_non_pin_zero_overlap_and_perturbation_guard(self) -> None:
        boxes = dock.load_surface_boxes()
        non_pin = [b for b in boxes if b.role != "pin"]
        self.assertEqual(len(non_pin), 11)
        by_part = {b.part: b for b in boxes}
        for i, box_a in enumerate(non_pin):
            for box_b in non_pin[i + 1:]:
                ov = _obb_overlap_mm(box_a, box_b)
                self.assertLessEqual(ov, 1e-6, f"overlap {box_a.part} vs {box_b.part}: {ov}")
        shifted = _obb_overlap_mm(by_part["bed"], by_part["funnel_l"], offset_b=(18.0, -30.0, -5.0))
        self.assertGreater(shifted, 1.0)


class DockMjcfAndCli(unittest.TestCase):
    def test_build_mjcf_mount_and_cli(self) -> None:
        values = params.values(params.load())
        plain_xml = mjcf.build_mjcf(values)
        self.assertNotIn('<body name="dock"', plain_xml)
        dock_xml = mjcf.build_mjcf(values, dock="scene")
        self.assertIn('<body name="dock" pos="-0.12 0 0" quat="', dock_xml)
        for box in dock.load_surface_boxes():
            self.assertIn(f'name="dock_{box.part}"', dock_xml)
        with tempfile.TemporaryDirectory() as tmpdir:
            out_path = Path(tmpdir) / "dock_scene.xml"
            proc = subprocess.run(
                [sys.executable, str(ROOT / "build_mjcf.py"), "--dock", "scene", "--out", str(out_path)],
                cwd=str(ROOT), capture_output=True, text=True, check=False,
            )
            self.assertEqual(proc.returncode, 0, proc.stderr)
            self.assertEqual(out_path.read_text(encoding="utf-8"), dock_xml)


class DockContactGeometry(unittest.TestCase):
    def test_nominal_offset_and_inverted_bounds(self) -> None:
        # Chassis at (-150, 0, 6.3) mm pitched nose-down (+8.2 deg) on dock at (-120, 0, 0) yaw 180 deg.
        dock_pos = (-0.12, 0.0, 0.0)
        dock_mat = (-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0)
        th = math.radians(dock.DOCKED_START_PITCH_DEG)
        c_rot = (math.cos(th), 0.0, math.sin(th), 0.0, 1.0, 0.0, -math.sin(th), 0.0, math.cos(th))
        nom = dock.contact_state((-0.150, 0.0, 0.0063), c_rot, dock_pos, dock_mat)
        self.assertTrue(nom["on_charger"])
        self.assertTrue(nom["on_charger_platform"])
        shifted = dock.contact_state((-0.150, 0.008, 0.0063), c_rot, dock_pos, dock_mat)
        self.assertFalse(shifted["on_charger"])
        self.assertTrue(shifted["on_charger_platform"])
        flipped = (math.cos(th), 0.0, math.sin(th), 0.0, -1.0, 0.0, math.sin(th), 0.0, -math.cos(th))
        inv = dock.contact_state((-0.150, 0.0, 0.0063), flipped, dock_pos, dock_mat)
        self.assertFalse(inv["on_charger"])
        self.assertFalse(inv["on_charger_platform"])
        off = dock.contact_state((0.0, 0.0, 0.0), c_rot, dock_pos, dock_mat)
        self.assertFalse(off["on_charger"])
        self.assertFalse(off["on_charger_platform"])


class OptionalObjFidelity(unittest.TestCase):
    def test_hp_obj_dimensions_and_ramp_fit(self) -> None:
        if not HP_OBJ.is_file():
            self.skipTest(f"{_common.OPTIONAL_SKIP_PREFIX}missing {HP_OBJ.relative_to(ROOT)}")
        pts, tri_verts = _load_hp_obj_tris(HP_OBJ)
        cmin, cmax = pts.min(axis=0), pts.max(axis=0)
        self.assertTrue(-0.15 <= cmin[0] <= 0.15 and 112.8 <= cmax[0] <= 113.2)
        self.assertAlmostEqual(float(cmin[1]), -43.731, delta=0.1)
        self.assertAlmostEqual(float(cmax[1]), 43.731, delta=0.1)
        self.assertTrue(-0.1 <= cmin[2] <= 0.1 and 77.4 <= cmax[2] <= 77.8)
        lpin = pts[(np.abs(pts[:, 0] - 59.71) < 5.0) & (pts[:, 1] < -6.0) & (pts[:, 1] > -13.5) & (pts[:, 2] < 16.0)]
        rpin = pts[(np.abs(pts[:, 0] - 59.71) < 5.0) & (pts[:, 1] > 6.0) & (pts[:, 1] < 13.5) & (pts[:, 2] < 16.0)]
        self.assertAlmostEqual(float(lpin[:, 2].max()), dock.PIN_LEFT_MM[2], delta=0.12)
        self.assertAlmostEqual(float(rpin[:, 2].max()), dock.PIN_RIGHT_MM[2], delta=0.12)
        wall = pts[(np.abs(pts[:, 1]) < 20.0) & (pts[:, 2] >= 30.0) & (pts[:, 2] <= 65.0) & (pts[:, 0] > 88.0) & (pts[:, 0] < 97.0)]
        self.assertAlmostEqual(float(wall[:, 0].min()), dock.MARKER_WALL_X_MM, delta=0.25)
        samples, mae_mm, max_err_mm = _tread_lane_mae_mm([b for b in dock.load_surface_boxes() if b.role == "ramp"], tri_verts)
        self.assertGreaterEqual(samples, 100)
        self.assertLess(mae_mm, 0.15)
        self.assertLess(max_err_mm, 0.35)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class DockPhysics(unittest.TestCase):
    def test_docked_start_voltage_and_undock(self) -> None:
        from simulacrum.robot import VectorSim

        sim = VectorSim(params.values(params.load()), dock="docked", noise_seed=1)
        self.assertTrue(sim.state.on_charger)
        self.assertTrue(sim.state.on_charger_platform)
        self.assertEqual(sim.battery_volts, dock.CHARGER_SUPPLY_VOLTS)
        pitch_deg = math.degrees(sim._pitch())
        self.assertTrue(-9.5 <= pitch_deg <= -6.5, f"unexpected docked pitch {pitch_deg:.3f}")
        res = sim.drive_straight(120.0, 80.0)
        self.assertEqual(res.result, "completed")
        self.assertFalse(sim.state.on_charger)
        self.assertFalse(sim.state.on_charger_platform)
        self.assertLess(sim.battery_volts, 4.10)

    def test_skewed_backup_self_centering_and_tof(self) -> None:
        import contact_trial
        from simulacrum.robot import VectorSim

        values = params.values(params.load())
        orig_build = mjcf.build_mjcf
        with mock.patch("simulacrum.mjcf.build_mjcf",
                        side_effect=lambda v, obstacles=(), cube=None, dock=None:
                        contact_trial.contact_model(orig_build(v, obstacles, cube=cube, dock=dock), v)):
            sim = VectorSim(values, dock="scene", start_xy_mm=(-55.0, 3.5), heading_deg=-3.0, lift_mm=45.0, noise_seed=1)
        sim.set_wheel_motors(-45.0, -45.0)
        contact_ticks = 0
        for _ in range(720):
            sim.step(1)
            if sim.state.on_charger:
                contact_ticks += 1
                if contact_ticks >= 35:
                    sim.stop_all_motors()
                    break
        sim.step(40)
        self.assertTrue(sim.state.on_charger)
        self.assertEqual(sim.battery_volts, dock.CHARGER_SUPPLY_VOLTS)
        self.assertEqual(sim.reactions, 0)
        self.assertLess(abs(math.degrees(sim.heading_total_rad())), 2.0)
        self.assertTrue(-153.0 <= sim.pose_mm()[0] <= -147.0)
        tof_sim = VectorSim(values, dock="scene", start_xy_mm=(-30.0, 0.0), heading_deg=180.0, noise_seed=1)
        tof_sim.step(5)
        self.assertTrue(tof_sim.state.prox_found_object)
        self.assertTrue(150 <= tof_sim.state.prox_distance_mm <= 195)


if __name__ == "__main__":
    unittest.main()
