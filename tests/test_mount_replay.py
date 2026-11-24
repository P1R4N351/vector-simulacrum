"""Regression tests for evidence/unit3-spin-cube/mount_replay.py."""

from __future__ import annotations

import importlib.util
import math
import shutil
import unittest

import numpy as np

import _common

SCRIPT = _common.ROOT / "evidence" / "unit3-spin-cube" / "mount_replay.py"


def _load_mount_replay():
    """Import mount_replay.py without altering global sys.modules permanently."""
    assert SCRIPT.is_file(), f"missing replay script: {SCRIPT}"
    spec = importlib.util.spec_from_file_location("mount_replay_under_test", SCRIPT)
    assert spec is not None and spec.loader is not None, "failed to create spec"
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    assert len(a) == 4 and len(b) == 4, "quaternions must have 4 elements"
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def _angvel_between(q_prev: np.ndarray, q_next: np.ndarray, dt: float, body_frame: bool) -> np.ndarray:
    assert dt > 0.0 and len(q_prev) == 4 and len(q_next) == 4, "bad angvel inputs"
    q_inv = np.array([q_prev[0], -q_prev[1], -q_prev[2], -q_prev[3]])
    dq = _quat_mul(q_inv, q_next) if body_frame else _quat_mul(q_next, q_inv)
    if dq[0] < 0.0:
        dq = -dq
    sin_half = float(np.linalg.norm(dq[1:4]))
    if sin_half < 1e-12:
        return np.zeros(3)
    return (2.0 * math.atan2(sin_half, float(dq[0])) / dt) * (dq[1:4] / sin_half)


@unittest.skipUnless(_common.HAVE_MUJOCO, _common.SKIP_REASON)
class MountReplay(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.mr = _load_mount_replay()
        cls.v, cls.derived, cls.source = cls.mr._load_model_config()
        cls.path = cls.mr._load_structured("path.csv.gz")
        cls.obs = cls.mr._load_structured("observations.csv")

    def _w_case(self, trial: str, obs_name: str):
        robot = self.path[self.path["trial"] == trial]
        obs_rows = [row for row in self.obs if row["trial"] == trial and row["observation"] == obs_name]
        assert len(obs_rows) == 1, f"missing observation {trial}/{obs_name}"
        return robot, obs_rows[0]

    def test_freejoint_velocity_uses_body_frame_not_world_frame(self) -> None:
        import mujoco as mj

        model = self.mr.build_model(self.source, self.v, self.derived, drive="freejoint", contact="fork")
        dt = float(model.opt.timestep)
        robot, _ = self._w_case("W35", "target")
        traj = self.mr.compute_trajectory(robot, self.v, dt)
        idx = 2850
        pose0 = np.concatenate([traj["fork_pos"][idx], traj["quat"][idx]])
        pose1 = np.concatenate([traj["fork_pos"][idx + 1], traj["quat"][idx + 1]])
        bid = model.body("fork").id
        jid = int(model.body_jntadr[bid])
        qvel = self.mr.freejoint_qvel_for_body(model, int(model.jnt_qposadr[jid]), int(model.jnt_dofadr[jid]), pose0, pose1, dt)
        expected_body = _angvel_between(traj["quat"][idx], traj["quat"][idx + 1], dt, body_frame=True)
        wrong_world = _angvel_between(traj["quat"][idx], traj["quat"][idx + 1], dt, body_frame=False)
        np.testing.assert_allclose(qvel[3:6], expected_body, atol=1e-6)
        self.assertGreater(float(np.linalg.norm(qvel[3:6] - wrong_world)), 1.0)

    def test_freejoint_fork_imparts_nonzero_contact_velocity_unlike_mocap(self) -> None:
        robot, obs = self._w_case("W33", "target")
        self.assertEqual(self.mr.DEFAULT_DRIVE, "freejoint")
        m_mocap = self.mr.build_model(self.source, self.v, self.derived, drive="mocap", contact="fork")
        traj = self.mr.compute_trajectory(robot, self.v, float(m_mocap.opt.timestep))
        mocap_res = self.mr.simulate_trial(m_mocap, obs, traj, drive="mocap")
        self.assertEqual(mocap_res["max_cvel_lin"], 0.0)
        self.assertEqual(mocap_res["max_cvel_ang"], 0.0)

        m_default = self.mr.build_model(self.source, self.v, self.derived, drive=self.mr.DEFAULT_DRIVE, contact="fork")
        free_res = self.mr.simulate_trial(m_default, obs, traj, drive=self.mr.DEFAULT_DRIVE)
        self.assertGreater(free_res["max_cvel_lin"], 0.05)
        self.assertGreater(free_res["max_cvel_ang"], 1.0)
        self.assertIsNotNone(free_res["first_fork_contact_s"])
        self.assertTrue(-0.21 <= free_res["first_fork_contact_s"] <= -0.19)
        self.assertGreater(free_res["sim_raw_peak_g"], 1.8)

    def test_full_contact_option_adds_chassis_and_wheels(self) -> None:
        import mujoco as mj

        m_fork = self.mr.build_model(self.source, self.v, self.derived, drive="freejoint", contact="fork")
        self.assertEqual(mj.mj_name2id(m_fork, mj.mjtObj.mjOBJ_BODY, "robot_chassis"), -1)
        m_full = self.mr.build_model(self.source, self.v, self.derived, drive="freejoint", contact="full")
        self.assertGreaterEqual(mj.mj_name2id(m_full, mj.mjtObj.mjOBJ_BODY, "robot_chassis"), 0)
        for gname in ("chassis_geom", "wheel_fl_geom", "wheel_rr_geom"):
            self.assertGreaterEqual(mj.mj_name2id(m_full, mj.mjtObj.mjOBJ_GEOM, gname), 0)

    def test_five_phase_sensor_scoring_on_w33_target(self) -> None:
        if shutil.which("node") is None or not (_common.ROOT / "web" / "node_modules").exists():
            self.skipTest(_common.OPTIONAL_SKIP_PREFIX + "node or web/node_modules absent")
        robot, obs = self._w_case("W33", "target")
        model = self.mr.build_model(self.source, self.v, self.derived, drive=self.mr.DEFAULT_DRIVE, contact="fork")
        traj = self.mr.compute_trajectory(robot, self.v, float(model.opt.timestep))
        sim = self.mr.simulate_trial(model, obs, traj, drive=self.mr.DEFAULT_DRIVE)
        pkt = self.mr.score_sensor_phases(sim["dt"], float(sim["times"][0]), sim["forces"])
        self.assertGreater(pkt["sim_phase_max_g"], 1.0)
        self.assertLessEqual(pkt["sim_phase_min_g"], pkt["sim_peak_g"])
        self.assertLessEqual(pkt["sim_peak_g"], pkt["sim_phase_max_g"])
        self.assertEqual(pkt["sim_phase_taps_nonzero"], 5)


if __name__ == "__main__":
    unittest.main()
