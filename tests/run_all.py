"""Run the simulacrum's tests and say exactly how many ran. Authored-by: CPCS.

    python3 tests/run_all.py                      everything that can run here
    cauldron python tests/run_all.py --require-mujoco
    python3 tests/run_all.py --only test_sim.Cliff.test_reaction_on_stops_and_backs_away

Exit codes: 0 all passed, 7 a test failed or errored (or was skipped under
--require-mujoco for anything but an optional input, _common.OPTIONAL_SKIP_PREFIX), 3 MuJoCo or the training packages were required and are
missing, 2 bad arguments. The last line is one JSON object, for the
mutation check to read.
"""

from __future__ import annotations

import argparse
import json
import sys
import unittest
from pathlib import Path
from typing import List, Tuple

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import _common  # noqa: E402

MODULES = ("test_params", "test_mjcf", "test_drive", "test_recording", "test_imu_fit", "test_recorder", "test_sim",
           "test_replay", "test_task", "test_training", "test_rank", "test_mcp",
           "test_wheelstand", "test_motor_heat", "test_prox", "test_spin_fit", "test_camera_spins",
           "test_imu_filter", "test_record_listen",
           "test_hold_cube", "test_cube_accel", "test_cube_model", "test_cube_firmware", "test_capture_peers",
           "test_run_all")
EXIT_OK, EXIT_USAGE, EXIT_ENV, EXIT_RED = 0, 2, 3, 7


def _load(only: List[str]) -> unittest.TestSuite:
    loader = unittest.TestLoader()
    names = only if only else list(MODULES)
    assert len(names) > 0, "nothing to run"
    suite = loader.loadTestsFromNames(names)
    assert not loader.errors, "test modules failed to load: " + "; ".join(loader.errors)[:400]
    return suite


def _ids(pairs: list) -> List[str]:
    return sorted(test.id() for test, _ in pairs)


def verdict(result: unittest.TestResult, declared: int, require_mujoco: bool) -> Tuple[bool, int]:
    """(green, skips for an optional input). Under --require-mujoco any other skip is red."""
    optional = [s for s in result.skipped if str(s[1]).startswith(_common.OPTIONAL_SKIP_PREFIX)]
    green = result.wasSuccessful() and result.testsRun == declared
    return green and not (require_mujoco and len(result.skipped) > len(optional)), len(optional)


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Run the Vector simulacrum tests.")
    parser.add_argument("--only", action="append", default=[], help="dotted test name; may repeat")
    parser.add_argument("--require-mujoco", action="store_true",
                        help="fail if MuJoCo or the training packages are missing, or if any test is skipped")
    args = parser.parse_args(argv)
    if args.require_mujoco and not _common.HAVE_TRAINING:
        print("MuJoCo, torch, rsl_rl and onnxruntime are required and at least one is not installed here.")
        return EXIT_ENV
    suite = _load(args.only)
    declared = suite.countTestCases()
    result = unittest.TextTestRunner(verbosity=1, stream=sys.stdout).run(suite)
    passed = result.testsRun - len(result.failures) - len(result.errors) - len(result.skipped)
    summary = {
        "declared": declared, "ran": result.testsRun, "passed": passed,
        "failed": _ids(result.failures), "errored": _ids(result.errors),
        "skipped": len(result.skipped), "mujoco": _common.HAVE_MUJOCO, "training": _common.HAVE_TRAINING,
    }
    green, optional = verdict(result, declared, args.require_mujoco)
    summary["skipped_optional"] = optional
    print(f"declared {declared}, ran {result.testsRun}, passed {passed}, failed {len(result.failures)}, "
          f"errored {len(result.errors)}, skipped {len(result.skipped)} ({optional} for an optional input)")
    print("SUMMARY " + json.dumps(summary, sort_keys=True))
    return EXIT_OK if green else EXIT_RED


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
