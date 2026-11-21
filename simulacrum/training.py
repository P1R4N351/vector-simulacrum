"""PPO training on the task with rsl_rl, and ONNX export..

Needs torch, tensordict and rsl_rl (all in a GPU host's environment). The
environment adapter gives rsl_rl one observation group, "policy", used by
both actor and critic, and keeps its own record of every finished episode so
the learning curve does not depend on rsl_rl's logger.

Import order matters in a GPU host's environment (mujoco 3.11, tensordict 0.15,
torch 2.15): importing mujoco before tensordict ends in a segmentation fault
when tensordict is imported (measured 2026-11-14: "import mujoco, tensordict"
crashes, "import tensordict, mujoco" does not). So this module must be
imported before anything that imports mujoco, and it refuses the other order
with an error instead of letting the interpreter crash.
"""

from __future__ import annotations

import contextlib
import io
import json
import os
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

if "mujoco" in sys.modules and "tensordict" not in sys.modules:
    raise ImportError("import simulacrum.training (or tensordict) before mujoco: the other order crashes "
                      "the interpreter in this environment")

import numpy as np
import onnx
import torch
from rsl_rl.env import VecEnv
from rsl_rl.runners import OnPolicyRunner
from tensordict import TensorDict

from . import params, task
from .vec_env import TaskPool

TORCH_THREADS = 4
ONNX_NAME = "policy.onnx"
META_NAME = "policy.json"
CHECKPOINT_NAME = "model.pt"
CURVE_NAME = "curve.json"
PARITY_NAME = "parity.npz"


def train_config(steps_per_env: int) -> Dict[str, Any]:
    """rsl_rl 5.x runner configuration: a small MLP actor and critic, PPO with its usual settings."""
    assert steps_per_env >= 2, "a rollout needs at least two steps"
    model = {"class_name": "MLPModel", "hidden_dims": [64, 64], "activation": "elu", "obs_normalization": True}
    actor = dict(model, distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.5,
                                          "std_type": "scalar"})
    algorithm = {
        "class_name": "PPO", "num_learning_epochs": 5, "num_mini_batches": 4, "learning_rate": 1.0e-3,
        "schedule": "adaptive", "gamma": 0.99, "lam": 0.95, "entropy_coef": 0.005, "desired_kl": 0.01,
        "max_grad_norm": 1.0, "value_loss_coef": 1.0, "use_clipped_value_loss": True, "clip_param": 0.2,
        "rnd_cfg": None, "symmetry_cfg": None,
    }
    cfg = {"num_steps_per_env": steps_per_env, "save_interval": 10 ** 9, "check_for_nan": True,
           "obs_groups": {"actor": ["policy"], "critic": ["policy"]},
           "actor": actor, "critic": dict(model), "algorithm": algorithm}
    assert set(cfg["obs_groups"]) == {"actor", "critic"}, "actor and critic both need observations"
    return cfg


class VectorRslEnv(VecEnv):
    """The task pool, in the shape rsl_rl's runner expects."""

    def __init__(self, num_envs: int, seed: int, workers: int = 0,
                 config: Optional[task.TaskConfig] = None) -> None:
        self.pool = TaskPool(num_envs, seed, workers, config)
        self.num_envs, self.num_actions = num_envs, task.ACT_DIM
        nominal = params.values(params.load())
        cfg = config if config is not None else task.TaskConfig()
        self.max_episode_length = int(round(cfg.episode_s * 1000.0 / nominal["state_period_ms"]))
        self.episode_length_buf = torch.zeros(num_envs, dtype=torch.long)
        self.device = "cpu"
        self.cfg = {"task": "vector speed and turn following", "num_envs": num_envs, "seed": seed}
        self.finished: List[tuple] = []
        self.errors: List[tuple] = []
        assert self.pool.obs.shape == (num_envs, task.OBS_DIM), "pool observation shape"

    def get_observations(self) -> TensorDict:
        obs = torch.from_numpy(np.ascontiguousarray(self.pool.obs)).float()
        return TensorDict({"policy": obs}, batch_size=[self.num_envs])

    def step(self, actions: torch.Tensor) -> tuple:
        out = self.pool.step(actions.detach().cpu().numpy())
        done = out["terminated"] | out["truncated"]
        self.episode_length_buf += 1
        self.episode_length_buf[torch.from_numpy(done)] = 0
        self.finished.extend(out["episodes"])
        self.errors.append((float(out["forward_error"].mean()), float(out["turn_error"].mean()),
                            float(out["reward"].mean())))
        extras = {"time_outs": torch.from_numpy(out["truncated"].copy()), "log": {}}
        assert out["reward"].shape == (self.num_envs,), "one reward per environment"
        return (self.get_observations(), torch.from_numpy(out["reward"].copy()).float(),
                torch.from_numpy(done.copy()).long(), extras)

    def pop_stats(self) -> Dict[str, Any]:
        """Summarise and clear everything recorded since the last call."""
        steps = len(self.errors)
        assert steps > 0, "no steps recorded"
        reasons = [record[0] for record in self.finished]
        stats = {
            "env_steps": steps * self.num_envs,
            "reward_per_step": sum(e[2] for e in self.errors) / steps,
            "forward_error_mmps": sum(e[0] for e in self.errors) / steps,
            "turn_error_dps": sum(e[1] for e in self.errors) / steps,
            "episodes": len(self.finished),
            "falls": reasons.count(task.END_FALL),
            "cliff_reactions": reasons.count(task.END_REACTION),
            "time_limits": reasons.count(task.END_TIME),
            "override_episodes": sum(1 for record in self.finished if record[3] == "override"),
        }
        self.finished, self.errors = [], []
        return stats

    def close(self) -> None:
        self.pool.close()


def make_runner(env: VectorRslEnv, steps_per_env: int, seed: int) -> OnPolicyRunner:
    """Build the PPO runner. rsl_rl prints its model summary; that goes nowhere."""
    assert seed >= 0, "seed must not be negative"
    torch.set_num_threads(TORCH_THREADS)
    torch.manual_seed(seed)
    with contextlib.redirect_stdout(io.StringIO()):
        runner = OnPolicyRunner(env, train_config(steps_per_env), log_dir=None, device="cpu")
    assert runner.alg is not None, "runner has no algorithm"
    return runner


def train(env: VectorRslEnv, runner: OnPolicyRunner, iterations: int) -> List[Dict[str, Any]]:
    """Run PPO and return one record per iteration: the learning curve."""
    assert 1 <= iterations <= 100000, "iteration count out of range"
    curve: List[Dict[str, Any]] = []
    for index in range(iterations):
        started = time.perf_counter()
        with contextlib.redirect_stdout(io.StringIO()):
            runner.learn(1)
        record = env.pop_stats()
        record.update(iteration=index, wall_s=round(time.perf_counter() - started, 3),
                      action_std=float(runner.alg.get_policy().output_std.detach().mean()))
        curve.append(record)
    assert len(curve) == iterations, "an iteration left no record"
    return curve


def deterministic_actions(runner: OnPolicyRunner, obs: np.ndarray) -> np.ndarray:
    """The torch policy's mean action for each observation row, through rsl_rl's own export wrapper."""
    assert obs.ndim == 2 and obs.shape[1] == task.OBS_DIM, "observations are rows of OBS_DIM"
    module = runner.alg.get_policy().as_onnx(verbose=False)
    module.eval()
    with torch.inference_mode():
        out = module(torch.from_numpy(np.ascontiguousarray(obs)).float())
    assert out.shape == (obs.shape[0], task.ACT_DIM), "one action per observation"
    return out.numpy().astype(np.float32)


def _single_file(path: Path) -> None:
    """The exporter may put the weights in a side file; fold them back so the policy is one file."""
    side = path.with_name(path.name + ".data")
    model = onnx.load(str(path), load_external_data=True)
    onnx.save_model(model, str(path), save_as_external_data=False)
    if side.exists():
        side.unlink()
    onnx.checker.check_model(str(path))
    assert not side.exists(), "ONNX weights are still in a side file"


def export_policy(runner: OnPolicyRunner, out_dir: Path, meta: Dict[str, Any]) -> Path:
    """Write policy.onnx with rsl_rl's exporter, and policy.json describing its inputs and outputs."""
    out_dir.mkdir(parents=True, exist_ok=True)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        runner.export_policy_to_onnx(str(out_dir), ONNX_NAME)
    _single_file(out_dir / ONNX_NAME)
    nominal = params.values(params.load())
    described = dict(meta)
    described.update({
        "obs_names": list(task.OBS_NAMES), "obs_dim": task.OBS_DIM, "act_dim": task.ACT_DIM,
        "action_meaning": "left and right wheel speed as a fraction of wheel_speed_max_mmps, clipped to -1..1",
        "wheel_speed_max_mmps": nominal["wheel_speed_max_mmps"], "policy_period_ms": nominal["state_period_ms"],
        "observation_scales": task.observation_scales(nominal),
    })
    (out_dir / META_NAME).write_text(json.dumps(described, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    path = out_dir / ONNX_NAME
    assert path.is_file() and path.stat().st_size > 0, "ONNX file was not written"
    return path


def record_parity(runner: OnPolicyRunner, out_dir: Path, seed: int, steps: int) -> Path:
    """Drive one nominal robot with the torch policy and save what it saw and what it did.

    The file lets a process without torch replay the same observations through the ONNX
    file and compare actions, and repeat the closed loop from the same seed.
    """
    assert 1 <= steps <= 5000, "parity step count out of range"
    config = task.TaskConfig(randomise=False, override_fraction=0.0)
    env = task.VectorTask(seed, config)
    obs_rows, act_rows = [], []
    for _ in range(steps):
        obs = np.array([env.observation()], dtype=np.float32)
        action = deterministic_actions(runner, obs)[0]
        obs_rows.append(obs[0])
        act_rows.append(action)
        result = env.step(action)
        if result.terminated or result.truncated:
            break
    path = out_dir / PARITY_NAME
    np.savez(path, obs=np.array(obs_rows, dtype=np.float32), actions=np.array(act_rows, dtype=np.float32),
             seed=np.array([seed]), steps=np.array([len(obs_rows)]))
    assert path.is_file(), "parity file was not written"
    return path


def save_run(runner: OnPolicyRunner, out_dir: Path, curve: List[Dict[str, Any]], meta: Dict[str, Any]) -> None:
    """Checkpoint, learning curve, ONNX policy and parity recording, all in one directory."""
    out_dir.mkdir(parents=True, exist_ok=True)
    runner.save(os.path.join(str(out_dir), CHECKPOINT_NAME))
    (out_dir / CURVE_NAME).write_text(json.dumps({"meta": meta, "curve": curve}, indent=1) + "\n", encoding="utf-8")
    export_policy(runner, out_dir, meta)
    record_parity(runner, out_dir, seed=int(meta.get("seed", 0)), steps=120)
    for name in (CHECKPOINT_NAME, CURVE_NAME, ONNX_NAME, META_NAME, PARITY_NAME):
        assert (out_dir / name).is_file(), f"{name} missing from the run directory"
