"""PyTorch NoMaD model (Goal-Masked Transformer Encoder + FiLM 1D U-Net Denoiser) and ONNX/Web exporters."""

from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
from typing import Any, Dict, Tuple

if importlib.util.find_spec("tensordict") is not None:
    import tensordict  # noqa: F401  # must load before mujoco on Linux hosts (see simulacrum/training.py)

import numpy as np
import onnx
import torch
import torch.nn as nn
import torch.nn.functional as F

IMG_H, IMG_W, CONTEXT_FRAMES = 36, 64, 4
HORIZON, ACT_DIM, EMBED_DIM = 8, 2, 64
NUM_HEADS, FF_DIM = 4, 128
DIFFUSION_STEPS, DDIM_STEPS = 16, 8
WAYPOINT_SCALE_MM, STEP_PERIOD_S = 120.0, 0.25
MAX_DIST_STEPS = 16.0
ENCODER_ONNX = "nomad_encoder.onnx"
DENOISER_ONNX = "nomad_denoiser.onnx"
WEB_META_NAME = "nomad.json"
WEB_WEIGHTS_NAME = "nomad_weights.bin"


def diffusion_schedule(steps: int = DIFFUSION_STEPS) -> Dict[str, np.ndarray]:
    """Cosine-like quadratic beta schedule returning float32 diffusion arrays."""
    assert 4 <= steps <= 256, "diffusion steps out of bounds"
    betas = np.linspace(0.002, 0.14, steps, dtype=np.float32)
    alphas = 1.0 - betas
    alphas_cumprod = np.cumprod(alphas, axis=0).astype(np.float32)
    return {"betas": betas, "alphas": alphas.astype(np.float32), "alphas_cumprod": alphas_cumprod}


class ConvStem(nn.Module):
    """Lightweight CNN tokenizer mapping (B, C, 36, 64) to (B, 64)."""

    def __init__(self, in_ch: int, embed_dim: int = EMBED_DIM) -> None:
        super().__init__()
        self.conv1 = nn.Conv2d(in_ch, 16, kernel_size=3, stride=2, padding=1)
        self.conv2 = nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1)
        self.conv3 = nn.Conv2d(32, 48, kernel_size=3, stride=1, padding=1)
        self.pool = nn.AvgPool2d(kernel_size=(3, 4), stride=(3, 4))
        self.proj = nn.Linear(48 * 3 * 4, embed_dim)
        self.norm = nn.LayerNorm(embed_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        h = F.silu(self.conv1(x))
        h = F.silu(self.conv2(h))
        h = F.silu(self.conv3(h))
        h = self.pool(h).reshape(x.shape[0], 48 * 3 * 4)
        return self.norm(self.proj(h))


class TransformerBlock(nn.Module):
    """Explicit pre-norm multi-head self-attention block for deterministic ONNX/JS export."""

    def __init__(self, d_model: int = EMBED_DIM, n_heads: int = NUM_HEADS, ff_dim: int = FF_DIM) -> None:
        super().__init__()
        assert d_model % n_heads == 0, "embed dim must divide heads"
        self.n_heads, self.head_dim = n_heads, d_model // n_heads
        self.ln1 = nn.LayerNorm(d_model)
        self.q_proj = nn.Linear(d_model, d_model)
        self.k_proj = nn.Linear(d_model, d_model)
        self.v_proj = nn.Linear(d_model, d_model)
        self.out_proj = nn.Linear(d_model, d_model)
        self.ln2 = nn.LayerNorm(d_model)
        self.ff1 = nn.Linear(d_model, ff_dim)
        self.ff2 = nn.Linear(ff_dim, d_model)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        bsz, seq_len, dim = x.shape
        h = self.ln1(x)
        q = self.q_proj(h).reshape(bsz, seq_len, self.n_heads, self.head_dim).permute(0, 2, 1, 3)
        k = self.k_proj(h).reshape(bsz, seq_len, self.n_heads, self.head_dim).permute(0, 2, 1, 3)
        v = self.v_proj(h).reshape(bsz, seq_len, self.n_heads, self.head_dim).permute(0, 2, 1, 3)
        attn = torch.softmax((q @ k.transpose(-2, -1)) * (1.0 / math.sqrt(self.head_dim)), dim=-1)
        out = (attn @ v).permute(0, 2, 1, 3).reshape(bsz, seq_len, dim)
        x = x + self.out_proj(out)
        h2 = self.ln2(x)
        return x + self.ff2(F.silu(self.ff1(h2)))


class NoMaDEncoder(nn.Module):
    """Goal-masked visual Transformer producing diffusion context and temporal distance."""

    def __init__(self) -> None:
        super().__init__()
        self.obs_stem = ConvStem(3, EMBED_DIM)
        self.goal_stem = ConvStem(6, EMBED_DIM)
        self.mask_token = nn.Parameter(torch.randn(1, EMBED_DIM) * 0.02)
        self.pos_embed = nn.Parameter(torch.randn(1, CONTEXT_FRAMES + 1, EMBED_DIM) * 0.02)
        self.blocks = nn.ModuleList([TransformerBlock() for _ in range(2)])
        self.final_norm = nn.LayerNorm(EMBED_DIM)
        self.ctx_proj = nn.Linear(EMBED_DIM * 2, EMBED_DIM)
        self.ctx_norm = nn.LayerNorm(EMBED_DIM)
        self.dist_fc1 = nn.Linear(EMBED_DIM, EMBED_DIM)
        self.dist_fc2 = nn.Linear(EMBED_DIM, 1)

    def forward(self, obs_stack: torch.Tensor, goal_img: torch.Tensor,
                goal_mask: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        bsz = obs_stack.shape[0]
        obs_frames = obs_stack.reshape(bsz * CONTEXT_FRAMES, 3, IMG_H, IMG_W)
        obs_toks = self.obs_stem(obs_frames).reshape(bsz, CONTEXT_FRAMES, EMBED_DIM)
        cur_frame = obs_stack[:, (CONTEXT_FRAMES - 1) * 3:CONTEXT_FRAMES * 3]
        goal_tok = self.goal_stem(torch.cat([cur_frame, goal_img], dim=1))
        mask_val = goal_mask.reshape(bsz, 1)
        masked_goal = (1.0 - mask_val) * goal_tok + mask_val * self.mask_token.expand(bsz, EMBED_DIM)
        tokens = torch.cat([obs_toks, masked_goal.unsqueeze(1)], dim=1) + self.pos_embed
        for blk in self.blocks:
            tokens = blk(tokens)
        tokens = self.final_norm(tokens)
        pooled = torch.cat([tokens[:, CONTEXT_FRAMES - 1], tokens[:, CONTEXT_FRAMES]], dim=-1)
        context = self.ctx_norm(self.ctx_proj(pooled))
        pred_dist = self.dist_fc2(F.silu(self.dist_fc1(context)))
        return context, pred_dist


class FiLMResBlock1d(nn.Module):
    """1D residual convolution block conditioned via scale-and-shift FiLM."""

    def __init__(self, in_ch: int, out_ch: int, cond_dim: int = EMBED_DIM) -> None:
        super().__init__()
        self.conv1 = nn.Conv1d(in_ch, out_ch, kernel_size=3, padding=1)
        self.norm1 = nn.GroupNorm(4, out_ch)
        self.film = nn.Linear(cond_dim, out_ch * 2)
        self.conv2 = nn.Conv1d(out_ch, out_ch, kernel_size=3, padding=1)
        self.norm2 = nn.GroupNorm(4, out_ch)
        self.skip = nn.Conv1d(in_ch, out_ch, kernel_size=1) if in_ch != out_ch else nn.Identity()

    def forward(self, x: torch.Tensor, cond: torch.Tensor) -> torch.Tensor:
        h = self.norm1(self.conv1(x))
        gb = self.film(cond).unsqueeze(-1)
        scale, shift = gb[:, :h.shape[1]], gb[:, h.shape[1]:]
        h = F.silu((1.0 + scale) * h + shift)
        h = F.silu(self.norm2(self.conv2(h)))
        return h + self.skip(x)


class NoMaDDenoiser(nn.Module):
    """FiLM-conditioned 1D U-Net predicting diffusion noise over 8 waypoints."""

    inv_freq: torch.Tensor

    def __init__(self) -> None:
        super().__init__()
        half = EMBED_DIM // 2
        freqs = torch.exp(-math.log(100.0) * torch.arange(half, dtype=torch.float32) / half)
        self.register_buffer("inv_freq", freqs.unsqueeze(0))
        self.time_fc = nn.Linear(EMBED_DIM, EMBED_DIM)
        self.ctx_fc = nn.Linear(EMBED_DIM, EMBED_DIM)
        self.down1 = FiLMResBlock1d(ACT_DIM, 32, EMBED_DIM)
        self.down_pool = nn.Conv1d(32, 32, kernel_size=3, stride=2, padding=1)
        self.mid = FiLMResBlock1d(32, 64, EMBED_DIM)
        self.up_conv = nn.Conv1d(64, 32, kernel_size=3, padding=1)
        self.up1 = FiLMResBlock1d(64, 32, EMBED_DIM)
        self.out_conv = nn.Conv1d(32, ACT_DIM, kernel_size=1)

    def forward(self, noisy_actions: torch.Tensor, context: torch.Tensor, timestep: torch.Tensor) -> torch.Tensor:
        ang = timestep.reshape(-1, 1) * self.inv_freq
        t_emb = torch.cat([torch.sin(ang), torch.cos(ang)], dim=-1)
        cond = F.silu(self.time_fc(t_emb) + self.ctx_fc(context))
        x = noisy_actions.permute(0, 2, 1)
        h1 = self.down1(x, cond)
        h_mid = self.mid(self.down_pool(h1), cond)
        up = h_mid.unsqueeze(-1).expand(-1, -1, -1, 2).reshape(x.shape[0], 64, HORIZON)
        h_up = self.up1(torch.cat([self.up_conv(up), h1], dim=1), cond)
        return self.out_conv(h_up).permute(0, 2, 1)


def _fold_single_file(path: Path) -> None:
    side = path.with_name(path.name + ".data")
    model = onnx.load(str(path), load_external_data=True)
    onnx.save_model(model, str(path), save_as_external_data=False)
    if side.exists():
        side.unlink()
    onnx.checker.check_model(str(path))
    assert not side.exists(), "external ONNX data file left behind"


def export_onnx(encoder: NoMaDEncoder, denoiser: NoMaDDenoiser, out_dir: Path) -> Tuple[Path, Path]:
    """Export single-file encoder and denoiser ONNX graphs with dynamic batch size."""
    assert str(out_dir).strip() != "", "out_dir must not be empty"
    out_dir.mkdir(parents=True, exist_ok=True)
    enc_cpu = encoder.eval().cpu()
    den_cpu = denoiser.eval().cpu()
    dummy_obs = torch.zeros(1, CONTEXT_FRAMES * 3, IMG_H, IMG_W, dtype=torch.float32)
    dummy_goal = torch.zeros(1, 3, IMG_H, IMG_W, dtype=torch.float32)
    dummy_mask = torch.zeros(1, 1, dtype=torch.float32)
    dummy_act = torch.zeros(1, HORIZON, ACT_DIM, dtype=torch.float32)
    dummy_ctx = torch.zeros(1, EMBED_DIM, dtype=torch.float32)
    dummy_t = torch.zeros(1, dtype=torch.float32)
    enc_path = out_dir / ENCODER_ONNX
    den_path = out_dir / DENOISER_ONNX
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        torch.onnx.export(enc_cpu, (dummy_obs, dummy_goal, dummy_mask), str(enc_path),
                          input_names=["obs_stack", "goal_img", "goal_mask"],
                          output_names=["context", "pred_dist"],
                          dynamic_axes={"obs_stack": {0: "batch"}, "goal_img": {0: "batch"},
                                        "goal_mask": {0: "batch"}, "context": {0: "batch"}, "pred_dist": {0: "batch"}},
                          opset_version=17, dynamo=False)
        torch.onnx.export(den_cpu, (dummy_act, dummy_ctx, dummy_t), str(den_path),
                          input_names=["noisy_actions", "context", "timestep"],
                          output_names=["pred_noise"],
                          dynamic_axes={"noisy_actions": {0: "batch"}, "context": {0: "batch"},
                                        "timestep": {0: "batch"}, "pred_noise": {0: "batch"}},
                          opset_version=17, dynamo=False)
    _fold_single_file(enc_path)
    _fold_single_file(den_path)
    assert enc_path.is_file() and den_path.is_file(), "ONNX export missing output"
    return enc_path, den_path


def export_web_weights(encoder: NoMaDEncoder, denoiser: NoMaDDenoiser, out_dir: Path,
                       meta: Dict[str, Any]) -> Tuple[Path, Path]:
    """Pack encoder + denoiser Float32 weights into nomad_weights.bin + nomad.json."""
    assert out_dir.exists(), "output directory must exist before exporting web weights"
    tensors, chunks = [], []
    offset = 0
    for prefix, mod in (("encoder", encoder.eval().cpu()), ("denoiser", denoiser.eval().cpu())):
        for name, param in mod.state_dict().items():
            arr = np.ascontiguousarray(param.detach().cpu().numpy(), dtype=np.float32)
            flat = arr.reshape(-1)
            tensors.append({"name": f"{prefix}.{name}", "shape": list(arr.shape), "offset": offset, "length": int(flat.size)})
            chunks.append(flat)
            offset += int(flat.size)
    blob = np.concatenate(chunks).astype("<f4").tobytes()
    weights_path = out_dir / WEB_WEIGHTS_NAME
    weights_path.write_bytes(blob)
    sched = diffusion_schedule(DIFFUSION_STEPS)
    spec = {
        "img_h": IMG_H, "img_w": IMG_W, "context_frames": CONTEXT_FRAMES, "horizon": HORIZON,
        "act_dim": ACT_DIM, "embed_dim": EMBED_DIM, "num_heads": NUM_HEADS,
        "waypoint_scale_mm": WAYPOINT_SCALE_MM, "step_period_s": STEP_PERIOD_S,
        "diffusion_steps": DIFFUSION_STEPS, "ddim_steps": DDIM_STEPS,
        "betas": [float(x) for x in sched["betas"]],
        "alphas_cumprod": [float(x) for x in sched["alphas_cumprod"]],
        "weights_sha256": hashlib.sha256(blob).hexdigest(),
        "weights_floats": offset,
        "tensors": tensors,
        "meta": meta,
    }
    meta_path = out_dir / WEB_META_NAME
    meta_path.write_text(json.dumps(spec, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    assert weights_path.is_file() and meta_path.is_file(), "web weights were not written"
    return meta_path, weights_path
