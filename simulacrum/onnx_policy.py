"""Run an exported policy with onnxruntime only..

No torch is imported here or by anything this module imports. This is the
deployment side: the same observation code as training (simulacrum.task),
an ONNX file, and two wheel speeds out every state period.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import numpy as np
import onnxruntime

from . import task

META_NAME = "policy.json"


class OnnxPolicy:
    """An exported policy: observation row in, two normalised wheel speeds out."""

    def __init__(self, onnx_path: Path) -> None:
        onnx_path = Path(onnx_path)
        options = onnxruntime.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.session = onnxruntime.InferenceSession(str(onnx_path), sess_options=options,
                                                    providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        self.meta = json.loads((onnx_path.parent / META_NAME).read_text(encoding="utf-8"))
        if tuple(self.meta["obs_names"]) != task.OBS_NAMES:
            raise ValueError("the policy was exported for a different observation layout")
        assert self.meta["act_dim"] == task.ACT_DIM, "the policy has a different action size"

    def act(self, observation: Sequence[float]) -> Tuple[float, float]:
        """Mean action for one observation, as the exported graph computes it."""
        assert len(observation) == task.OBS_DIM, "observation has the wrong length"
        row = np.asarray([observation], dtype=np.float32)
        out = self.session.run(None, {self.input_name: row})[0]
        assert out.shape == (1, task.ACT_DIM), "policy output has the wrong shape"
        return float(out[0][0]), float(out[0][1])

    def act_rows(self, observations: np.ndarray) -> np.ndarray:
        """Actions for many observation rows, one run per row (the graph is exported for one row)."""
        assert observations.ndim == 2 and observations.shape[1] == task.OBS_DIM, "rows of OBS_DIM"
        out = np.zeros((observations.shape[0], task.ACT_DIM), dtype=np.float32)
        for i in range(observations.shape[0]):
            out[i] = self.session.run(None, {self.input_name: observations[i:i + 1].astype(np.float32)})[0][0]
        return out


def closed_loop(policy: OnnxPolicy, seed: int, steps: int, randomise: bool = False,
                override_fraction: float = 0.0) -> Dict[str, Any]:
    """Drive one simulated robot with the policy and report what happened."""
    assert 1 <= steps <= 100000, "step count out of range"
    env = task.VectorTask(seed, task.TaskConfig(randomise=randomise, override_fraction=override_fraction))
    obs_rows: List[Tuple[float, ...]] = []
    act_rows: List[Tuple[float, float]] = []
    reward = err_f = err_t = 0.0
    reason = None
    for _ in range(steps):
        obs = env.observation()
        action = policy.act(obs)
        obs_rows.append(obs)
        act_rows.append(action)
        result = env.step(action)
        reward, err_f, err_t = reward + result.reward, err_f + result.forward_error_mmps, err_t + result.turn_error_dps
        if result.terminated or result.truncated:
            reason = result.end_reason
            break
    count = len(obs_rows)
    return {"steps": count, "end_reason": reason, "reward_per_step": reward / count,
            "forward_error_mmps": err_f / count, "turn_error_dps": err_t / count,
            "obs": np.array(obs_rows, dtype=np.float32), "actions": np.array(act_rows, dtype=np.float32)}


def parity(policy: OnnxPolicy, parity_path: Path) -> Dict[str, Any]:
    """Compare this process's ONNX actions with the torch actions recorded at training time.

    The test is the first comparison: the observations the torch policy saw in its closed
    loop, put through the ONNX graph, must give the same actions. The closed loop is then
    repeated from the same seed with the ONNX policy in charge, and the number of steps
    that stay identical is reported. It is not a pass criterion: measured in this model,
    shifting every action by 1e-7 is enough to make two runs part within ten steps, so two
    loops that agree to rounding error still drift apart.
    """
    recorded = np.load(parity_path)
    obs, torch_actions = recorded["obs"], recorded["actions"]
    assert obs.shape[0] == torch_actions.shape[0] and obs.shape[0] > 0, "parity file is empty or ragged"
    onnx_actions = policy.act_rows(obs)
    loop = closed_loop(policy, int(recorded["seed"][0]), int(recorded["steps"][0]))
    shared = min(loop["obs"].shape[0], obs.shape[0])
    apart = np.abs(loop["obs"][:shared] - obs[:shared]).max(axis=1) > 1.0e-4
    identical = int(np.argmax(apart)) if apart.any() else shared
    return {
        "rows": int(obs.shape[0]),
        "max_action_difference": float(np.max(np.abs(onnx_actions - torch_actions))),
        "closed_loop_steps": int(loop["steps"]), "closed_loop_steps_identical": identical,
        "torch_imported": "torch" in sys.modules,
    }
