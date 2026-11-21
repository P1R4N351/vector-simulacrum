"""Score an exported policy against two fixed baselines, without torch..

    the GPU host python evaluate.py --policy /root/vector-runs/smoke/policy.onnx

Runs the ONNX policy, a robot that stands still, and a hand-written no-slip
controller on the same randomised robots (same seeds, same commands, same
tables) and prints one row each, with the number of environment steps and
episodes behind every figure. The seeds are not the training seeds.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Callable, Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from simulacrum import onnx_policy, params, task  # noqa: E402
from simulacrum.vec_env import MAX_WORKERS, TaskPool, rollout  # noqa: E402

EXIT_OK, EXIT_USAGE = 0, 2
FORWARD_AT, TURN_AT = task.OBS_NAMES.index("command_forward"), task.OBS_NAMES.index("command_turn")


def baselines(nominal: Dict[str, Any]) -> Dict[str, Callable[[np.ndarray], np.ndarray]]:
    """Two policies that need no training: stand still, and the no-slip wheel speeds for the command."""
    fwd, turn = nominal["directive_forward_speed_max_mmps"], nominal["directive_turn_speed_max_dps"]

    def still(obs: np.ndarray) -> np.ndarray:
        return np.zeros((obs.shape[0], task.ACT_DIM), dtype=np.float32)

    def no_slip(obs: np.ndarray) -> np.ndarray:
        rows = [task.kinematic_action((row[FORWARD_AT] * fwd, row[TURN_AT] * turn), nominal) for row in obs]
        return np.array(rows, dtype=np.float32)

    assert fwd > 0.0 and turn > 0.0, "command limits must be positive"
    return {"stand still": still, "no-slip controller": no_slip}


def score(policy: Callable[[np.ndarray], np.ndarray], envs: int, steps: int, seed: int, workers: int) -> Dict[str, Any]:
    pool = TaskPool(envs, seed, workers)
    try:
        out = rollout(pool, policy, steps)
    finally:
        pool.close()
    out["episodes"] = out["falls"] + out["cliff_reactions"] + out["time_limits"]
    assert out["env_steps"] == envs * steps, "step count differs from the request"
    return out


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Compare an ONNX policy with two baselines on randomised robots.")
    parser.add_argument("--policy", required=True)
    parser.add_argument("--envs", type=int, default=16)
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--seed", type=int, default=1000)
    parser.add_argument("--workers", type=int, default=3)
    args = parser.parse_args(argv)
    if not Path(args.policy).is_file() or not 0 <= args.workers <= MAX_WORKERS or args.envs < 1 or args.steps < 1:
        print(f"error: --policy must be a file, workers 0..{MAX_WORKERS}, envs and steps at least 1", file=sys.stderr)
        return EXIT_USAGE
    learned = onnx_policy.OnnxPolicy(Path(args.policy))
    contenders = {"onnx policy": learned.act_rows}
    contenders.update(baselines(params.values(params.load())))
    rows = {name: score(policy, args.envs, args.steps, args.seed, args.workers) for name, policy in contenders.items()}
    print(f"{args.envs} randomised robots x {args.steps} policy steps = {args.envs * args.steps} environment steps "
          f"each, seed {args.seed}")
    print("policy               reward/step  fwd err mm/s  turn err deg/s  episodes  falls  reactions  time limits")
    for name, row in rows.items():
        print(f"{name:20s} {row['reward_per_step']:11.3f}  {row['forward_error_mmps']:12.1f}  "
              f"{row['turn_error_dps']:14.1f}  {row['episodes']:8d}  {row['falls']:5d}  "
              f"{row['cliff_reactions']:9d}  {row['time_limits']:11d}")
    print("EVALUATION " + json.dumps({"rows": rows, "torch_imported": "torch" in sys.modules}, sort_keys=True))
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
