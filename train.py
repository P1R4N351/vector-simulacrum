"""Train a wheel-speed policy with PPO and export it to ONNX..

    the GPU host python train.py --out /root/vector-runs/smoke --iterations 100
    the GPU host python train.py --out /root/vector-runs/tiny --iterations 2 --envs 4 --steps 16 --workers 0

Writes model.pt, curve.json, policy.onnx, policy.json and parity.npz to --out
and prints the learning curve. The defaults are a smoke run for a GPU host (CPU
only, shared node): they prove the pipeline, they do not train a usable
policy. --workers is capped at 4 by the environment pool.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import training  # noqa: E402
from simulacrum.vec_env import MAX_WORKERS  # noqa: E402

EXIT_OK, EXIT_USAGE = 0, 2
PRINT_ROWS = 12


def _mean(curve: List[Dict[str, Any]], key: str) -> float:
    assert len(curve) > 0, "empty curve slice"
    return sum(float(row[key]) for row in curve) / len(curve)


def summarise(curve: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Compare the first and last tenth of the run. Counts carry their denominators."""
    assert len(curve) >= 2, "need at least two iterations to compare"
    span = max(1, len(curve) // 10)
    first, last = curve[:span], curve[-span:]
    out = {"iterations": len(curve), "iterations_compared": span,
           "env_steps_total": sum(row["env_steps"] for row in curve),
           "episodes_total": sum(row["episodes"] for row in curve),
           "falls_total": sum(row["falls"] for row in curve),
           "override_episodes_total": sum(row["override_episodes"] for row in curve),
           "cliff_reactions_total": sum(row["cliff_reactions"] for row in curve),
           "time_limits_total": sum(row["time_limits"] for row in curve)}
    for key in ("reward_per_step", "forward_error_mmps", "turn_error_dps"):
        out[key + "_first"] = round(_mean(first, key), 4)
        out[key + "_last"] = round(_mean(last, key), 4)
    return out


def print_curve(curve: List[Dict[str, Any]]) -> None:
    stride = max(1, len(curve) // PRINT_ROWS)
    assert stride >= 1, "stride must be positive"
    print("iter  reward/step  fwd err mm/s  turn err deg/s  episodes  falls  reactions  action std  wall s")
    for row in curve[::stride] + ([curve[-1]] if (len(curve) - 1) % stride else []):
        print(f"{row['iteration']:4d}  {row['reward_per_step']:11.3f}  {row['forward_error_mmps']:12.1f}  "
              f"{row['turn_error_dps']:14.1f}  {row['episodes']:8d}  {row['falls']:5d}  "
              f"{row['cliff_reactions']:9d}  {row['action_std']:10.3f}  {row['wall_s']:6.2f}")


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="PPO training of the Vector speed-and-turn task.")
    parser.add_argument("--out", required=True, help="directory for the run's files")
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--envs", type=int, default=32)
    parser.add_argument("--steps", type=int, default=32, help="policy steps per environment per iteration")
    parser.add_argument("--workers", type=int, default=3, help=f"environment worker processes, 0..{MAX_WORKERS}")
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args(argv)
    if not (0 <= args.workers <= MAX_WORKERS and args.iterations >= 2 and args.envs >= 1 and args.steps >= 2):
        print(f"error: workers 0..{MAX_WORKERS}, iterations >= 2, envs >= 1, steps >= 2", file=sys.stderr)
        return EXIT_USAGE
    started = time.perf_counter()
    env = training.VectorRslEnv(args.envs, args.seed, args.workers)
    try:
        runner = training.make_runner(env, args.steps, args.seed)
        curve = training.train(env, runner, args.iterations)
        meta = {"seed": args.seed, "envs": args.envs, "steps_per_env": args.steps, "workers": args.workers,
                "iterations": args.iterations, "wall_s": round(time.perf_counter() - started, 1),
                "summary": summarise(curve)}
        training.save_run(runner, Path(args.out), curve, meta)
    finally:
        env.close()
    print_curve(curve)
    print("summary:", meta["summary"])
    print(f"wall time {meta['wall_s']} s; files in {args.out}")
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
