"""PPO training, ONNX export and the torch-free policy runner..

Needs MuJoCo, torch, rsl_rl and onnxruntime. One tiny training run (3
iterations, 4 environments, 16 steps) is shared by the tests: it proves the
pipeline, not the policy.
"""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import _common

if _common.HAVE_TRAINING:
    import numpy as np
    import onnx
    import torch
    from simulacrum import onnx_policy, task, training

ROOT = _common.ROOT
ITERATIONS, ENVS, STEPS, SEED = 3, 4, 16, 3
ACTION_TOL = 1.0e-5


def _tiny_run(out_dir: Path, seed: int) -> tuple:
    env = training.VectorRslEnv(ENVS, seed, workers=0)
    try:
        runner = training.make_runner(env, STEPS, seed)
        curve = training.train(env, runner, ITERATIONS)
        training.save_run(runner, out_dir, curve, {"seed": seed, "note": "test run"})
    finally:
        env.close()
    return runner, curve


def _run_policy(*args: str) -> tuple:
    done = subprocess.run([sys.executable, str(ROOT / "run_policy.py"), *args], capture_output=True, text=True,
                          timeout=300, cwd=str(ROOT))
    lines = [line for line in done.stdout.splitlines() if line.startswith("REPORT ")]
    report = json.loads(lines[0][len("REPORT "):]) if len(lines) == 1 else {}
    return done.returncode, report, done.stderr


@unittest.skipUnless(_common.HAVE_TRAINING, _common.TRAINING_SKIP_REASON)
class Pipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.work = Path(tempfile.mkdtemp(prefix="vector-simulacrum-train-"))
        cls.run_dir = cls.work / "run"
        cls.runner, cls.curve = _tiny_run(cls.run_dir, SEED)
        rng = np.random.default_rng(0)
        cls.obs = rng.uniform(-1.0, 1.0, (64, task.OBS_DIM)).astype(np.float32)

    @classmethod
    def tearDownClass(cls) -> None:
        shutil.rmtree(cls.work, ignore_errors=True)

    def test_run_writes_checkpoint_curve_and_a_single_file_policy(self) -> None:
        names = sorted(p.name for p in self.run_dir.iterdir())
        self.assertEqual(names, ["curve.json", "model.pt", "parity.npz", "policy.json", "policy.onnx"])
        onnx.checker.check_model(str(self.run_dir / "policy.onnx"))
        graph = onnx.load(str(self.run_dir / "policy.onnx")).graph
        self.assertEqual([i.name for i in graph.input], ["obs"])
        self.assertEqual([o.name for o in graph.output], ["actions"])

    def test_curve_has_one_record_per_iteration_with_its_step_count(self) -> None:
        self.assertEqual(len(self.curve), ITERATIONS)
        self.assertEqual([row["iteration"] for row in self.curve], list(range(ITERATIONS)))
        self.assertTrue(all(row["env_steps"] == ENVS * STEPS for row in self.curve))
        saved = json.loads((self.run_dir / "curve.json").read_text())["curve"]
        self.assertEqual(saved, self.curve)

    def test_training_changes_the_policy(self) -> None:
        env = training.VectorRslEnv(ENVS, SEED, workers=0)
        try:
            runner = training.make_runner(env, STEPS, SEED)
            before = training.deterministic_actions(runner, self.obs)
            training.train(env, runner, 2)
            after = training.deterministic_actions(runner, self.obs)
        finally:
            env.close()
        self.assertGreater(float(np.max(np.abs(after - before))), 1.0e-4)

    def test_onnx_gives_the_torch_actions_on_the_same_observations(self) -> None:
        policy = onnx_policy.OnnxPolicy(self.run_dir / "policy.onnx")
        from_torch = training.deterministic_actions(self.runner, self.obs)
        from_onnx = policy.act_rows(self.obs)
        self.assertEqual(from_onnx.shape, (64, task.ACT_DIM))
        self.assertLess(float(np.max(np.abs(from_onnx - from_torch))), ACTION_TOL)
        self.assertGreater(float(np.max(np.abs(from_torch))), 1.0e-3, "a policy that outputs zeros proves nothing")

    def test_onnx_runs_without_torch_and_reproduces_the_recorded_closed_loop(self) -> None:
        code, report, stderr = _run_policy("--policy", str(self.run_dir / "policy.onnx"),
                                           "--parity", str(self.run_dir / "parity.npz"), "--seconds", "3.0")
        self.assertEqual(code, 0, stderr[-400:])
        self.assertFalse(report["torch_imported"])
        self.assertFalse(report["parity"]["torch_imported"])
        self.assertGreaterEqual(report["parity"]["rows"], 50)
        self.assertLess(report["parity"]["max_action_difference"], ACTION_TOL)
        self.assertGreaterEqual(report["parity"]["closed_loop_steps_identical"], 1)
        self.assertEqual(report["steps"], 50)

    def test_parity_check_fails_for_a_different_policy(self) -> None:
        other = self.work / "other"
        _tiny_run(other, SEED + 1)
        code, report, _ = _run_policy("--policy", str(other / "policy.onnx"),
                                      "--parity", str(self.run_dir / "parity.npz"), "--seconds", "0.3")
        self.assertEqual(code, 7)
        self.assertGreater(report["parity"]["max_action_difference"], ACTION_TOL)

    def test_policy_description_matches_the_task(self) -> None:
        meta = json.loads((self.run_dir / "policy.json").read_text())
        self.assertEqual(tuple(meta["obs_names"]), task.OBS_NAMES)
        self.assertEqual((meta["obs_dim"], meta["act_dim"]), (task.OBS_DIM, task.ACT_DIM))
        self.assertEqual(meta["policy_period_ms"], 60.0)
        self.assertEqual(meta["wheel_speed_max_mmps"], 220.0)

    def test_a_policy_for_another_observation_layout_is_refused(self) -> None:
        copy = self.work / "wrong-layout"
        shutil.copytree(self.run_dir, copy)
        meta = json.loads((copy / "policy.json").read_text())
        meta["obs_names"] = list(reversed(meta["obs_names"]))
        (copy / "policy.json").write_text(json.dumps(meta))
        with self.assertRaises(ValueError):
            onnx_policy.OnnxPolicy(copy / "policy.onnx")

    def test_same_seed_gives_the_same_trained_policy(self) -> None:
        again = self.work / "again"
        runner, _ = _tiny_run(again, SEED)
        first = training.deterministic_actions(self.runner, self.obs)
        second = training.deterministic_actions(runner, self.obs)
        self.assertLess(float(np.max(np.abs(first - second))), 1.0e-6)


@unittest.skipUnless(_common.HAVE_TRAINING, _common.TRAINING_SKIP_REASON)
class Adapter(unittest.TestCase):
    def test_environment_has_the_shape_rsl_rl_expects(self) -> None:
        cfg = task.TaskConfig(randomise=False, override_fraction=0.0, episode_s=0.18)
        env = training.VectorRslEnv(3, 1, workers=0, config=cfg)
        try:
            obs = env.get_observations()
            self.assertEqual(tuple(obs["policy"].shape), (3, task.OBS_DIM))
            self.assertEqual((env.num_envs, env.num_actions, env.max_episode_length), (3, 2, 3))
            for _ in range(3):
                obs, reward, done, extras = env.step(torch.zeros(3, 2))
            self.assertEqual(tuple(reward.shape), (3,))
            self.assertEqual(done.tolist(), [1, 1, 1])
            self.assertEqual(extras["time_outs"].tolist(), [True, True, True], "a time limit is a time-out")
            self.assertEqual(env.episode_length_buf.tolist(), [0, 0, 0])
            stats = env.pop_stats()
            self.assertEqual((stats["env_steps"], stats["episodes"], stats["time_limits"], stats["falls"]), (9, 3, 3, 0))
        finally:
            env.close()

    def test_a_fall_is_not_reported_as_a_time_out(self) -> None:
        cfg = task.TaskConfig(randomise=False, override_fraction=1.0, command_resample_s=100.0)
        env = training.VectorRslEnv(1, 5, workers=0, config=cfg)
        try:
            fell = None
            for _ in range(env.max_episode_length + 1):
                _, _, done, extras = env.step(torch.full((1, 2), 0.5))
                if int(done[0]):
                    fell = extras["time_outs"].tolist()
                    break
            self.assertEqual(fell, [False])
            self.assertEqual(env.pop_stats()["falls"], 1)
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
