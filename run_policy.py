"""Run an exported ONNX policy on the simulacrum without torch..

    the GPU host python run_policy.py --policy /root/vector-runs/smoke/policy.onnx
    the GPU host python run_policy.py --policy .../policy.onnx --parity .../parity.npz

The first form drives one simulated robot closed-loop and prints how well the
command was followed. The second also checks that this process, which has no
torch in it, gets the same actions the torch policy gave on the observations
of its own closed loop.
The last line is one JSON object. Exit codes: 0 ok, 7 the parity check
failed, 2 bad arguments. This never talks to a real robot.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import List

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import onnx_policy  # noqa: E402

EXIT_OK, EXIT_USAGE, EXIT_RED = 0, 2, 7
ACTION_TOL = 1.0e-5


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Run an exported Vector policy with onnxruntime only.")
    parser.add_argument("--policy", required=True, help="path to policy.onnx (policy.json beside it)")
    parser.add_argument("--parity", help="parity.npz written by train.py")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--randomise", action="store_true", help="draw the robot from the parameter ranges")
    args = parser.parse_args(argv)
    if not Path(args.policy).is_file() or not 0.0 < args.seconds <= 600.0:
        print("error: --policy must be a file and --seconds inside 0..600", file=sys.stderr)
        return EXIT_USAGE
    policy = onnx_policy.OnnxPolicy(Path(args.policy))
    steps = int(round(args.seconds * 1000.0 / policy.meta["policy_period_ms"]))
    loop = onnx_policy.closed_loop(policy, args.seed, steps, randomise=args.randomise)
    report = {"steps": loop["steps"], "end_reason": loop["end_reason"],
              "reward_per_step": round(loop["reward_per_step"], 4),
              "forward_error_mmps": round(loop["forward_error_mmps"], 2),
              "turn_error_dps": round(loop["turn_error_dps"], 2), "torch_imported": "torch" in sys.modules}
    ok = not report["torch_imported"]
    if args.parity:
        check = onnx_policy.parity(policy, Path(args.parity))
        report["parity"] = check
        ok = (ok and check["max_action_difference"] <= ACTION_TOL and check["closed_loop_steps_identical"] >= 1
              and not check["torch_imported"])
    print(f"closed loop: {loop['steps']} steps, reward {report['reward_per_step']} per step, forward error "
          f"{report['forward_error_mmps']} mm/s, turn error {report['turn_error_dps']} deg/s, "
          f"ended: {loop['end_reason']}")
    print("REPORT " + json.dumps(report, sort_keys=True))
    return EXIT_OK if ok else EXIT_RED


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
