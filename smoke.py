"""The whole pipeline once, small: train, export, check without torch, compare..

    ./run_in_ct.sh smoke.py                       about five minutes in a GPU host
    ./run_in_ct.sh smoke.py --iterations 20       shorter

Three separate processes, so that the second and third hold no torch:
  1. train.py        PPO with rsl_rl, writes model.pt, curve.json, policy.onnx, parity.npz
  2. run_policy.py   onnxruntime only: same actions as torch, closed loop repeated
  3. evaluate.py     onnxruntime only: the policy against two baselines on unseen robots

This is a smoke run. It shows that every stage works and what the learning
curve looks like after a few minutes on a CPU. It does not produce a policy
to put on a robot.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path
from typing import List

HERE = Path(__file__).resolve().parent
EXIT_OK, EXIT_RED = 0, 7
STAGE_TIMEOUT_S = 3600


def stage(name: str, script: str, args: List[str]) -> int:
    """Run one stage as its own process, echo its output, return its exit code."""
    assert (HERE / script).is_file(), f"missing {script}"
    started = time.perf_counter()
    print(f"--- {name}", flush=True)
    done = subprocess.run([sys.executable, str(HERE / script), *args], cwd=str(HERE), timeout=STAGE_TIMEOUT_S)
    print(f"--- {name}: exit {done.returncode}, {time.perf_counter() - started:.1f} s", flush=True)
    assert isinstance(done.returncode, int), "stage returned no exit code"
    return done.returncode


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Smoke run of the Vector training pipeline.")
    parser.add_argument("--out", default="/root/vector-runs/smoke")
    parser.add_argument("--iterations", type=int, default=150)
    parser.add_argument("--envs", type=int, default=32)
    parser.add_argument("--steps", type=int, default=32)
    parser.add_argument("--workers", type=int, default=3)
    parser.add_argument("--seed", type=int, default=1)
    args = parser.parse_args(argv)
    out = Path(args.out)
    codes = [stage("train (PPO, rsl_rl)", "train.py", [
        "--out", str(out), "--iterations", str(args.iterations), "--envs", str(args.envs),
        "--steps", str(args.steps), "--workers", str(args.workers), "--seed", str(args.seed)])]
    if codes[0] == 0:
        codes.append(stage("run the ONNX policy without torch, compare with torch", "run_policy.py", [
            "--policy", str(out / "policy.onnx"), "--parity", str(out / "parity.npz")]))
        codes.append(stage("compare with baselines on unseen randomised robots", "evaluate.py", [
            "--policy", str(out / "policy.onnx"), "--workers", str(args.workers)]))
    ok = len(codes) == 3 and all(code == 0 for code in codes)
    print(f"smoke run: {'all three stages passed' if ok else 'FAILED'}; exit codes {codes}")
    return EXIT_OK if ok else EXIT_RED


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
