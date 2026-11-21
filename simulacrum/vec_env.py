"""Many tasks stepped together, in this process or in worker processes..

No torch. Each environment has its own seed and is independent of the
others, so the result does not depend on how many workers carry them.
An environment that ends is reset at once; the observation returned for it
is the first one of its next episode.

a GPU host sits on a shared hypervisor: the pool refuses more than MAX_WORKERS
worker processes.
"""

from __future__ import annotations

import multiprocessing
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .task import ACT_DIM, OBS_DIM, TaskConfig, VectorTask

MAX_WORKERS = 4
MAX_ENVS = 1024
MAX_MESSAGES = 10 ** 9  # bound on one worker's message loop
SEED_STRIDE = 7919  # a prime, so neighbouring environments get unrelated seeds


def env_seed(base_seed: int, index: int) -> int:
    assert base_seed >= 0 and index >= 0, "seeds and indices are not negative"
    return base_seed * SEED_STRIDE + index


def _step_tasks(tasks: List[VectorTask], actions: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, list]:
    """Step each task once. Returns observations, a per-env numeric block and finished-episode records."""
    assert actions.shape == (len(tasks), ACT_DIM), "one action per environment"
    obs = np.zeros((len(tasks), OBS_DIM), dtype=np.float32)
    nums = np.zeros((len(tasks), 5), dtype=np.float32)  # reward, terminated, truncated, forward err, turn err
    final_obs = np.zeros((len(tasks), OBS_DIM), dtype=np.float32)
    episodes = []
    for i, task in enumerate(tasks):
        result = task.step(actions[i])
        nums[i] = (result.reward, float(result.terminated), float(result.truncated),
                   result.forward_error_mmps, result.turn_error_dps)
        final_obs[i] = result.observation
        if result.terminated or result.truncated:
            episodes.append((result.end_reason, task.episode_return, task.steps, task.sim.control_level))
            obs[i] = task.reset()
        else:
            obs[i] = result.observation
    return obs, nums, final_obs, episodes


def _worker(conn: Any, seeds: List[int], config: Optional[TaskConfig]) -> None:
    """Message loop of one worker process: ('step', actions) or ('close', None)."""
    tasks = [VectorTask(seed, config) for seed in seeds]
    conn.send(np.array([task.observation() for task in tasks], dtype=np.float32))
    for _ in range(MAX_MESSAGES):
        kind, payload = conn.recv()
        if kind == "close":
            break
        assert kind == "step", "unknown message to an environment worker"
        conn.send(_step_tasks(tasks, payload))
    conn.close()


class TaskPool:
    """num_envs tasks behind one step call."""

    def __init__(self, num_envs: int, seed: int, workers: int = 0, config: Optional[TaskConfig] = None) -> None:
        assert 1 <= num_envs <= MAX_ENVS, "environment count out of range"
        assert 0 <= workers <= MAX_WORKERS, f"at most {MAX_WORKERS} worker processes on the shared node"
        self.num_envs, self.workers = num_envs, min(workers, num_envs)
        self.seeds = [env_seed(seed, i) for i in range(num_envs)]
        self._tasks: List[VectorTask] = []
        self._conns: List[Any] = []
        self._procs: List[Any] = []
        self._slices: List[Tuple[int, int]] = []
        self.obs = np.zeros((num_envs, OBS_DIM), dtype=np.float32)
        if self.workers == 0:
            self._tasks = [VectorTask(s, config) for s in self.seeds]
            self.obs[:] = [task.observation() for task in self._tasks]
        else:
            self._start_workers(config)
        self.closed = False

    def _start_workers(self, config: Optional[TaskConfig]) -> None:
        ctx = multiprocessing.get_context("spawn")
        bounds = np.linspace(0, self.num_envs, self.workers + 1).astype(int)
        for w in range(self.workers):
            lo, hi = int(bounds[w]), int(bounds[w + 1])
            parent, child = ctx.Pipe()
            proc = ctx.Process(target=_worker, args=(child, self.seeds[lo:hi], config), daemon=True)
            proc.start()
            child.close()
            self._conns.append(parent)
            self._procs.append(proc)
            self._slices.append((lo, hi))
        for conn, (lo, hi) in zip(self._conns, self._slices):
            self.obs[lo:hi] = conn.recv()
        assert len(self._procs) == self.workers, "a worker did not start"

    def step(self, actions: np.ndarray) -> Dict[str, Any]:
        """Step every environment. Keys: obs, reward, terminated, truncated, forward_error, turn_error, episodes."""
        assert not self.closed, "pool is closed"
        actions = np.asarray(actions, dtype=np.float32)
        assert actions.shape == (self.num_envs, ACT_DIM), "one action per environment"
        if self.workers == 0:
            parts = [_step_tasks(self._tasks, actions)]
        else:
            for conn, (lo, hi) in zip(self._conns, self._slices):
                conn.send(("step", actions[lo:hi]))
            parts = [conn.recv() for conn in self._conns]
        self.obs = np.concatenate([p[0] for p in parts])
        nums = np.concatenate([p[1] for p in parts])
        episodes = [record for p in parts for record in p[3]]
        return {"obs": self.obs, "reward": nums[:, 0], "terminated": nums[:, 1] > 0.5,
                "truncated": nums[:, 2] > 0.5, "forward_error": nums[:, 3], "turn_error": nums[:, 4],
                "final_obs": np.concatenate([p[2] for p in parts]), "episodes": episodes}

    def close(self) -> None:
        """Stop the workers and wait for them."""
        if self.closed:
            return
        for conn in self._conns:
            conn.send(("close", None))
        for proc in self._procs:
            proc.join(timeout=10.0)
            if proc.is_alive():
                proc.terminate()
        self.closed = True
        assert all(not proc.is_alive() for proc in self._procs) or not self._procs, "a worker is still alive"


def rollout(pool: TaskPool, policy: Any, steps: int) -> Dict[str, float]:
    """Run a policy (observations -> actions, both arrays) and summarise what happened."""
    assert 1 <= steps <= 10 ** 6, "step count out of range"
    total, falls, reactions, timeouts = 0.0, 0, 0, 0
    err_f = err_t = 0.0
    for _ in range(steps):
        out = pool.step(policy(pool.obs))
        total += float(out["reward"].sum())
        err_f += float(out["forward_error"].mean())
        err_t += float(out["turn_error"].mean())
        for reason, _, _, _ in out["episodes"]:
            falls += reason == "fell"
            reactions += reason == "cliff_reaction"
            timeouts += reason == "time_limit"
    env_steps = steps * pool.num_envs
    return {"env_steps": env_steps, "reward_per_step": total / env_steps, "forward_error_mmps": err_f / steps,
            "turn_error_dps": err_t / steps, "falls": falls, "cliff_reactions": reactions, "time_limits": timeouts}
