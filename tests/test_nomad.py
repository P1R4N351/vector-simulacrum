"""Unit and MuJoCo/ONNX verification tests for NoMaD navigation and exploration."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from typing import Tuple

import numpy as np

import _common  # noqa: F401
from simulacrum import autonomy, nomad, nomad_cam, nomad_onnx, params

REPO_ROOT = Path(__file__).resolve().parent.parent


def _fake_obs(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    return rng.integers(0, 255, size=(nomad_cam.IMG_H, nomad_cam.IMG_W, 3), dtype=np.uint8)


def _export_temp_policy(out_dir: Path) -> Tuple[object, object, nomad_onnx.OnnxNoMaD]:
    from simulacrum import nomad_model

    enc = nomad_model.NoMaDEncoder().eval()
    den = nomad_model.NoMaDDenoiser().eval()
    nomad_model.export_onnx(enc, den, out_dir)
    nomad_model.export_web_weights(enc, den, out_dir, {"test": True})
    return enc, den, nomad_onnx.OnnxNoMaD(out_dir)


class TopologicalGraphUnit(unittest.TestCase):
    """Verify TopologicalGraph insertion, shortest-path routing, and snapshots."""

    def test_shortest_path_subgoal_and_snapshot(self) -> None:
        graph = nomad.TopologicalGraph()

        def metric_dist(obs_batch: np.ndarray, goal_batch: np.ndarray) -> np.ndarray:
            diffs = np.mean(np.abs(obs_batch[:, -3:] - goal_batch), axis=(1, 2, 3))
            return (diffs * 18.0).astype(np.float32)

        f0, f1, f2 = _fake_obs(1), _fake_obs(2), _fake_obs(3)
        for idx, (frame, pose) in enumerate((
            (f0, (0.0, 0.0, 0.0)),
            (f1, (100.0, 0.0, 0.0)),
            (f2, (200.0, 0.0, 0.0)),
        )):
            obs = nomad_onnx.stack_obs_frames([frame])
            nid = graph.record_step(obs, frame, pose, metric_dist, tick=idx * 25, label=f"n{idx}")
            self.assertEqual(nid, idx)
        graph.edges[0].clear()
        graph.edges[2].pop(0, None)
        graph.add_edge(0, 1, 1.5)
        graph.add_edge(1, 2, 1.5)
        graph.add_edge(0, 2, 10.0)
        self.assertEqual(graph.shortest_path(0, 2), [0, 1, 2])
        obs0 = nomad_onnx.stack_obs_frames([f0])
        sub_far, _, d_far = graph.choose_subgoal(obs0, 2, lambda o, g: np.array([0.1, 3.0, 6.0], dtype=np.float32))
        self.assertEqual(sub_far, 1)
        self.assertGreater(d_far, 3.5)
        sub_near, _, _ = graph.choose_subgoal(obs0, 2, lambda o, g: np.array([0.1, 2.0, 1.2], dtype=np.float32))
        self.assertEqual(sub_near, 2)
        snap = graph.snapshot()
        self.assertEqual(len(snap["nodes"]), 3)
        self.assertEqual(len(snap["edges"]), 3)


class NomadWaypointUnit(unittest.TestCase):
    """Verify safe trajectory selection and differential-drive waypoint tracking."""

    def test_select_safe_trajectory_penalizes_edge_and_obstacle(self) -> None:
        v = params.values(params.load())
        state = SimpleNamespace(
            x_mm=240.0, y_mm=0.0, pose_angle_rad=0.0,
            prox_found_object=True, prox_distance_mm=95.0, cliff_detected=False,
        )
        unsafe_edge = np.tile(np.array([[40.0, 0.0]], dtype=np.float32), (nomad_onnx.HORIZON, 1))
        safe_turn = np.tile(np.array([[-15.0, 55.0]], dtype=np.float32), (nomad_onnx.HORIZON, 1))
        cands = np.stack([unsafe_edge, safe_turn, unsafe_edge, unsafe_edge], axis=0)
        best, best_idx = nomad.select_safe_trajectory(cands, state, v)
        self.assertEqual(best_idx, 1)
        self.assertLess(float(best[2, 0]), 0.0)

    def test_waypoint_to_wheels_hazards_and_steering(self) -> None:
        v = params.values(params.load())
        fwd = [[15.0 * i, 0.0] for i in range(nomad_onnx.HORIZON)]
        self.assertEqual(nomad.waypoint_to_wheels(fwd, v, cliff_any=True), (0.0, 0.0))
        self.assertEqual(nomad.waypoint_to_wheels(fwd, v, prox_found=True, prox_mm=60.0), (0.0, 0.0))
        l_fwd, r_fwd = nomad.waypoint_to_wheels(fwd, v)
        self.assertGreater(l_fwd, 15.0)
        self.assertAlmostEqual(l_fwd, r_fwd, places=3)
        left_traj = [[15.0 * i, 25.0 * i] for i in range(nomad_onnx.HORIZON)]
        l_turn, r_turn = nomad.waypoint_to_wheels(left_traj, v)
        self.assertLess(l_turn, r_turn)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class NomadCameraAndOnnx(unittest.TestCase):
    """Verify MuJoCo head_cam renderer and PyTorch/ONNX NoMaD invariants."""

    def test_head_cam_sees_cube_and_lift_occlusion(self) -> None:
        sim_cube = autonomy.make_dock_sim(start_xy_mm=(-30.0, 0.0), heading_deg=0.0, head_deg=4.0, lift_mm=32.0)
        img_cube = nomad_cam.render_head_cam_rgb(sim_cube)
        sim_away = autonomy.make_dock_sim(start_xy_mm=(160.0, 120.0), heading_deg=0.0, head_deg=4.0, lift_mm=32.0)
        img_away = nomad_cam.render_head_cam_rgb(sim_away)
        sim_lift = autonomy.make_dock_sim(start_xy_mm=(-30.0, 0.0), heading_deg=0.0, head_deg=4.0, lift_mm=50.0)
        img_lift = nomad_cam.render_head_cam_rgb(sim_lift)
        self.assertEqual(img_cube.shape, (nomad_cam.IMG_H, nomad_cam.IMG_W, 3))
        self.assertEqual(img_cube.dtype, np.uint8)
        self.assertGreater(float(np.mean(np.abs(img_cube.astype(float) - img_away.astype(float)))), 1.5)
        self.assertTrue(np.all(img_lift[-7:, 8:-8, :] == np.array([52, 56, 64], dtype=np.uint8)))

    @unittest.skipUnless(_common.HAVE_TRAINING, _common.TRAINING_SKIP_REASON)
    def test_goal_mask_invariance_and_torch_onnx_parity(self) -> None:
        import torch
        from simulacrum import nomad_model

        with tempfile.TemporaryDirectory() as tmp:
            enc, den, runner = _export_temp_policy(Path(tmp))
            rng = np.random.default_rng(11)
            obs_np = rng.uniform(0.0, 1.0, size=(2, 12, nomad_cam.IMG_H, nomad_cam.IMG_W)).astype(np.float32)
            goal_a = rng.uniform(0.0, 1.0, size=(2, 3, nomad_cam.IMG_H, nomad_cam.IMG_W)).astype(np.float32)
            goal_b = np.ones_like(goal_a, dtype=np.float32) - goal_a
            mask_on, mask_off = np.ones((2, 1), dtype=np.float32), np.zeros((2, 1), dtype=np.float32)
            noise = rng.standard_normal((2, nomad_model.HORIZON, nomad_model.ACT_DIM)).astype(np.float32)
            ts = np.array([12.0, 4.0], dtype=np.float32)
            with torch.inference_mode():
                t_ctx_a, t_dist_a = enc(torch.from_numpy(obs_np), torch.from_numpy(goal_a), torch.from_numpy(mask_on))
                t_ctx_b, t_dist_b = enc(torch.from_numpy(obs_np), torch.from_numpy(goal_b), torch.from_numpy(mask_on))
                t_ctx0, t_dist0 = enc(torch.from_numpy(obs_np), torch.from_numpy(goal_a), torch.from_numpy(mask_off))
                t_eps = den(torch.from_numpy(noise), t_ctx0, torch.from_numpy(ts))
            self.assertLess(float(torch.max(torch.abs(t_ctx_a - t_ctx_b))), 1e-6)
            self.assertLess(float(torch.max(torch.abs(t_dist_a - t_dist_b))), 1e-6)
            o_ctx_a, o_dist_a = runner.encode(obs_np, goal_a, mask_on)
            o_ctx_b, o_dist_b = runner.encode(obs_np, goal_b, mask_on)
            o_ctx0, o_dist0 = runner.encode(obs_np, goal_a, mask_off)
            o_eps = runner.denoise(noise, o_ctx0, ts)
            self.assertLess(float(np.max(np.abs(o_ctx_a - o_ctx_b))), 1e-6)
            self.assertLess(float(np.max(np.abs(o_dist_a - o_dist_b))), 1e-6)
            self.assertLess(float(np.max(np.abs(o_ctx0 - t_ctx0.numpy()))), 1e-4)
            self.assertLess(float(np.max(np.abs(o_dist0 - t_dist0.numpy()))), 1e-4)
            self.assertLess(float(np.max(np.abs(o_eps - t_eps.numpy()))), 1e-4)

    @unittest.skipUnless(_common.HAVE_TRAINING, _common.TRAINING_SKIP_REASON)
    def test_torch_free_runner_and_navigator(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            out_dir = Path(tmp)
            _export_temp_policy(out_dir)
            runner = nomad_onnx.OnnxNoMaD(out_dir)
            probe = subprocess.run(
                [sys.executable, "-c",
                 "import sys; from simulacrum import nomad_onnx; print('torch' in sys.modules)"],
                cwd=str(REPO_ROOT), capture_output=True, text=True, check=True,
            )
            self.assertEqual(probe.stdout.strip(), "False")
            sim = autonomy.make_dock_sim(start_xy_mm=(-30.0, 0.0), heading_deg=0.0, head_deg=4.0, lift_mm=32.0)
            nav = nomad.NoMaDNavigator(sim, runner, seed=3)
            gid = nav.capture_goal("start")
            ex = nav.explore(2.0)
            nv = nav.navigate_to(max_s=2.0, goal_node_id=gid)
            self.assertFalse(ex["fallen"])
            self.assertFalse(nv["fallen"])
            self.assertGreaterEqual(ex["nodes"], 1)
            self.assertEqual(len(json.loads((out_dir / "nomad.json").read_text())["alphas_cumprod"]), 16)
