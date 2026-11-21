"""Shared set-up for the tests.."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

HAVE_MUJOCO = importlib.util.find_spec("mujoco") is not None
HAVE_TRAINING = HAVE_MUJOCO and all(
    importlib.util.find_spec(name) is not None for name in ("torch", "rsl_rl", "tensordict", "onnx", "onnxruntime"))
if HAVE_TRAINING:
    # Importing mujoco before tensordict crashes the interpreter in a GPU host's environment
    # (see simulacrum/training.py), so tensordict is loaded here, before any test imports mujoco.
    import tensordict  # noqa: F401
TRAINING_SKIP_REASON = "torch, rsl_rl or onnxruntime is not installed here; run in a GPU host"
SKIP_REASON = "MuJoCo is not installed here; run in a GPU host with ./run_in_ct.sh tests/run_all.py"
