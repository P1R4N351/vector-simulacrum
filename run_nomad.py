"""Run an exported ONNX NoMaD policy without torch and verify parity + navigation."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import autonomy, nomad, nomad_onnx  # noqa: E402

EXIT_OK, EXIT_FAIL = 0, 7
TOLERANCE = 1.0e-4


def check_parity(policy: nomad_onnx.OnnxNoMaD, parity_path: Path) -> Dict[str, Any]:
    """Compare ONNXRuntime outputs against recorded PyTorch parity arrays."""
    assert parity_path.is_file(), f"missing {parity_path}"
    rec = np.load(parity_path)
    obs_exact = np.rint(rec["obs_stack"] * 255.0).astype(np.float32) / 255.0
    goal_exact = np.rint(rec["goal_img"] * 255.0).astype(np.float32) / 255.0
    ctx, dist = policy.encode(obs_exact, goal_exact, rec["goal_mask"])
    noise = policy.denoise(rec["init_noise"], rec["context"], np.array([15.0, 15.0], dtype=np.float32))
    raw_ctx, raw_dist = policy.encode(rec["obs_stack"], rec["goal_img"], rec["goal_mask"])
    raw_noise = policy.denoise(rec["init_noise"], rec["context"], np.array([15.0, 15.0], dtype=np.float32))
    return {
        "max_context_diff": float(np.max(np.abs(raw_ctx - rec["context"]))),
        "max_dist_diff": float(np.max(np.abs(raw_dist - rec["pred_dist"]))),
        "max_noise_diff": float(np.max(np.abs(raw_noise - rec["pred_noise"]))),
        "quant_context_diff": float(np.max(np.abs(ctx - rec["context"]))),
        "quant_dist_diff": float(np.max(np.abs(dist - rec["pred_dist"]))),
        "quant_noise_diff": float(np.max(np.abs(noise - rec["pred_noise"]))),
        "torch_imported": "torch" in sys.modules,
    }


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--model-dir", required=True, help="directory with nomad_encoder.onnx + nomad.json")
    parser.add_argument("--parity", default="", help="optional parity.npz to verify against")
    parser.add_argument("--explore-s", type=float, default=6.0)
    parser.add_argument("--navigate-s", type=float, default=16.0)
    args = parser.parse_args(argv)
    policy = nomad_onnx.OnnxNoMaD(Path(args.model_dir))
    sim = autonomy.make_dock_sim(start_xy_mm=(-30.0, 0.0), heading_deg=0.0, head_deg=4.0, lift_mm=32.0)
    nav = nomad.NoMaDNavigator(sim, policy, seed=1)
    goal_id = nav.capture_goal("cube_view")
    explore_res = nav.explore(args.explore_s)
    nav_res = nav.navigate_to(max_s=args.navigate_s, goal_node_id=goal_id)
    report: Dict[str, Any] = {
        "torch_imported": "torch" in sys.modules,
        "explore": explore_res,
        "navigate": nav_res,
    }
    ok = (not report["torch_imported"]) and (not explore_res["fallen"]) and (not nav_res["fallen"])
    if args.parity:
        p = check_parity(policy, Path(args.parity))
        report["parity"] = p
        ok = ok and max(p["max_context_diff"], p["max_dist_diff"], p["max_noise_diff"]) < TOLERANCE
    print("REPORT " + json.dumps(report, sort_keys=True))
    return EXIT_OK if ok else EXIT_FAIL


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
