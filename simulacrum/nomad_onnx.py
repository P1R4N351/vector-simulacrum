"""Torch-free ONNXRuntime inference engine and DDIM sampler for NoMaD."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

ENCODER_ONNX = "nomad_encoder.onnx"
DENOISER_ONNX = "nomad_denoiser.onnx"
META_NAME = "nomad.json"
IMG_H, IMG_W, CONTEXT_FRAMES = 36, 64, 4
HORIZON, ACT_DIM, EMBED_DIM = 8, 2, 64


def rgb_to_chw(img_u8: np.ndarray) -> np.ndarray:
    """Convert (36, 64, 3) uint8 RGB image to normalized (3, 36, 64) float32."""
    arr = np.asarray(img_u8, dtype=np.float32)
    assert arr.shape == (IMG_H, IMG_W, 3), f"expected {(IMG_H, IMG_W, 3)}, got {arr.shape}"
    return np.ascontiguousarray(np.transpose(arr, (2, 0, 1)) / 255.0, dtype=np.float32)


def stack_obs_frames(frames_u8: Sequence[np.ndarray]) -> np.ndarray:
    """Build (12, 36, 64) float32 temporal stack from 1..4 uint8 RGB frames."""
    assert len(frames_u8) >= 1, "at least one frame required"
    kept = list(frames_u8[-CONTEXT_FRAMES:])
    for _ in range(CONTEXT_FRAMES):
        if len(kept) >= CONTEXT_FRAMES:
            break
        kept.insert(0, kept[0])
    chw = [rgb_to_chw(frame) for frame in kept]
    out = np.concatenate(chw, axis=0)
    assert out.shape == (CONTEXT_FRAMES * 3, IMG_H, IMG_W), "bad stacked frame shape"
    return out


class OnnxNoMaD:
    """NoMaD encoder + DDIM diffusion trajectory sampler backed by ONNXRuntime (no torch)."""

    def __init__(self, model_dir: Path) -> None:
        self.model_dir = Path(model_dir)
        meta_path = self.model_dir / META_NAME
        assert meta_path.is_file(), f"missing {meta_path}"
        self.meta: Dict[str, object] = json.loads(meta_path.read_text(encoding="utf-8"))
        self.alphas_cumprod = [float(x) for x in self.meta["alphas_cumprod"]]  # type: ignore[index]
        self.diffusion_steps = int(self.meta["diffusion_steps"])  # type: ignore[arg-type]
        self.ddim_steps = int(self.meta["ddim_steps"])  # type: ignore[arg-type]
        self.waypoint_scale_mm = float(self.meta["waypoint_scale_mm"])  # type: ignore[arg-type]
        import onnxruntime

        opts = onnxruntime.SessionOptions()
        opts.intra_op_num_threads = 1
        opts.inter_op_num_threads = 1
        self.encoder = onnxruntime.InferenceSession(str(self.model_dir / ENCODER_ONNX), sess_options=opts,
                                                    providers=["CPUExecutionProvider"])
        self.denoiser = onnxruntime.InferenceSession(str(self.model_dir / DENOISER_ONNX), sess_options=opts,
                                                     providers=["CPUExecutionProvider"])

    def encode(self, obs_stack: np.ndarray, goal_img: np.ndarray,
               goal_mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Run encoder on batch of observation stacks, goals, and mask flags."""
        obs = np.ascontiguousarray(obs_stack, dtype=np.float32)
        goal = np.ascontiguousarray(goal_img, dtype=np.float32)
        mask = np.ascontiguousarray(goal_mask, dtype=np.float32).reshape(-1, 1)
        assert obs.ndim == 4 and obs.shape[1:] == (CONTEXT_FRAMES * 3, IMG_H, IMG_W), "bad obs_stack shape"
        assert goal.ndim == 4 and goal.shape[1:] == (3, IMG_H, IMG_W), "bad goal_img shape"
        ctx, pred_dist = self.encoder.run(None, {"obs_stack": obs, "goal_img": goal, "goal_mask": mask})
        return np.asarray(ctx, dtype=np.float32), np.asarray(pred_dist, dtype=np.float32)

    def denoise(self, noisy_actions: np.ndarray, context: np.ndarray, timestep: np.ndarray) -> np.ndarray:
        """Predict diffusion noise eps(x_t, c, t) for a batch of trajectory samples."""
        act = np.ascontiguousarray(noisy_actions, dtype=np.float32)
        ctx = np.ascontiguousarray(context, dtype=np.float32)
        t_arr = np.ascontiguousarray(timestep, dtype=np.float32).reshape(-1)
        assert act.ndim == 3 and act.shape[1:] == (HORIZON, ACT_DIM), "bad noisy_actions shape"
        out = self.denoiser.run(None, {"noisy_actions": act, "context": ctx, "timestep": t_arr})[0]
        return np.asarray(out, dtype=np.float32)

    def ddim_sample(self, context: np.ndarray, num_samples: int = 4, ddim_steps: Optional[int] = None,
                    init_noise: Optional[np.ndarray] = None, seed: int = 0) -> np.ndarray:
        """Sample (num_samples, 8, 2) waypoint trajectories in mm using deterministic DDIM."""
        steps = int(ddim_steps if ddim_steps is not None else self.ddim_steps)
        assert 1 <= num_samples <= 32 and 1 <= steps <= self.diffusion_steps, "sample params out of range"
        ctx_one = np.asarray(context, dtype=np.float32).reshape(1, EMBED_DIM)
        ctx_batch = np.repeat(ctx_one, num_samples, axis=0)
        if init_noise is not None:
            x = np.asarray(init_noise, dtype=np.float32).copy()
        else:
            x = np.random.default_rng(seed).standard_normal((num_samples, HORIZON, ACT_DIM)).astype(np.float32)
        assert x.shape == (num_samples, HORIZON, ACT_DIM), "init_noise shape mismatch"
        stride = max(1, self.diffusion_steps // steps)
        t_list: List[int] = [self.diffusion_steps - 1 - i * stride for i in range(steps) if (self.diffusion_steps - 1 - i * stride) >= 0]
        if t_list[-1] != 0:
            t_list[-1] = 0
        for idx, t in enumerate(t_list):
            t_prev = t_list[idx + 1] if idx + 1 < len(t_list) else -1
            eps = self.denoise(x, ctx_batch, np.full((num_samples,), float(t), dtype=np.float32))
            ab_t = self.alphas_cumprod[t]
            ab_prev = self.alphas_cumprod[t_prev] if t_prev >= 0 else 1.0
            x0 = np.clip((x - math.sqrt(1.0 - ab_t) * eps) / math.sqrt(ab_t), -1.8, 1.8)
            x = math.sqrt(ab_prev) * x0 + math.sqrt(max(0.0, 1.0 - ab_prev)) * eps
        return x * self.waypoint_scale_mm
