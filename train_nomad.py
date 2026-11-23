"""Collect MuJoCo head_cam trajectories, train NoMaD, and export ONNX + Web weights."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple

if importlib.util.find_spec("tensordict") is not None:
    import tensordict  # noqa: F401  # must load before mujoco on Linux hosts (see simulacrum/training.py)

import numpy as np
import torch
import torch.nn.functional as F

sys.path.insert(0, str(Path(__file__).resolve().parent))

from simulacrum import autonomy, nomad_cam, nomad_model, nomad_onnx  # noqa: E402

EXIT_OK, EXIT_USAGE = 0, 2
CURVE_NAME = "curve.json"
PARITY_NPZ = "parity.npz"
PARITY_JSON = "nomad_parity.json"
STEP_TICKS = 50


def _wrap(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def _clamp(val: float, low: float, high: float) -> float:
    assert low <= high, "invalid clamp bounds"
    return min(max(val, low), high)


def _pick_goal_xy(idx: int, rng: np.random.Generator) -> Tuple[Tuple[float, float], float, Tuple[float, float]]:
    if idx % 3 == 0:
        tgt = (100.0, 0.0)
        ang = float(rng.choice([0.0, 0.75, -0.75, 1.5, -1.5, math.pi]))
        sx, sy = tgt[0] + float(rng.uniform(160.0, 260.0)) * math.cos(ang), tgt[1] + float(rng.uniform(160.0, 260.0)) * math.sin(ang)
    elif idx % 3 == 1:
        tgt = (-120.0, 0.0)
        ang = float(rng.uniform(-0.85, 0.85))
        sx, sy = tgt[0] + float(rng.uniform(180.0, 310.0)) * math.cos(ang), float(rng.uniform(180.0, 310.0)) * math.sin(ang)
    else:
        tgt = (float(rng.choice([-60.0, 220.0])), float(rng.choice([-190.0, 190.0])))
        sx, sy = float(rng.uniform(-160.0, 140.0)), float(rng.uniform(-140.0, 140.0))
    hd = math.degrees(_wrap(math.atan2(tgt[1] - sy, tgt[0] - sx) + float(rng.uniform(-1.3, 1.3))))
    return (_clamp(sx, -210.0, 210.0), _clamp(sy, -210.0, 210.0)), hd, tgt


def _explore_wheels(sim: Any, rng: np.random.Generator, base_turn: float, step_i: int) -> Tuple[float, float, float]:
    s = sim.state
    yaw = float(s.pose_angle_rad)
    near_edge = abs(s.x_mm) > 200.0 or abs(s.y_mm) > 200.0
    near_cube = math.hypot(s.x_mm - 100.0, s.y_mm) < 105.0
    near_dock = s.x_mm < -60.0 and abs(s.y_mm) < 75.0
    if sim.hazards.cliff_any or near_edge or near_cube or near_dock or (s.prox_found_object and s.prox_distance_mm < 115.0):
        to_center = math.atan2(-s.y_mm, -s.x_mm)
        err = _wrap(to_center - yaw) if near_edge else (_wrap(yaw + math.pi - yaw) if s.prox_found_object else 1.2)
        turn = 1.6 if err >= 0.0 else -1.6
        fwd = 0.0 if (abs(err) > 0.8 or sim.hazards.cliff_any) else 65.0
    else:
        if step_i % 8 == 0:
            base_turn = float(rng.uniform(-0.85, 0.85))
        fwd, turn = float(rng.uniform(75.0, 100.0)), base_turn
    half = 0.5 * float(sim.v["tread_spacing_mm"])
    return _clamp(fwd - turn * half, -150.0, 150.0), _clamp(fwd + turn * half, -150.0, 150.0), base_turn


def _approach_wheels(sim: Any, tgt: Tuple[float, float]) -> Tuple[float, float]:
    s = sim.state
    dx, dy = tgt[0] - float(s.x_mm), tgt[1] - float(s.y_mm)
    dist = math.hypot(dx, dy)
    if dist < 68.0 or abs(s.x_mm) > 235.0 or abs(s.y_mm) > 235.0:
        return 0.0, 0.0
    err = _wrap(math.atan2(dy, dx) - float(s.pose_angle_rad))
    half = 0.5 * float(sim.v["tread_spacing_mm"])
    if abs(err) > 0.70:
        omega = (1.0 if err >= 0.0 else -1.0) * _clamp(abs(err) * 1.4, 0.7, 1.8)
        return -omega * half, omega * half
    lin = _clamp(dist * 0.75, 30.0, 100.0)
    omega = _clamp(2.2 * err, -1.5, 1.5)
    return _clamp(lin - omega * half, -150.0, 150.0), _clamp(lin + omega * half, -150.0, 150.0)


def _rollout_episode(ep_idx: int, steps: int, seed: int) -> Tuple[List[np.ndarray], List[Tuple[float, float, float]]]:
    rng = np.random.default_rng(seed + ep_idx * 97)
    is_goal_ep = (ep_idx % 2) == 1
    if is_goal_ep:
        start_xy, hd, tgt = _pick_goal_xy(ep_idx, rng)
    else:
        start_xy = (float(rng.uniform(-160.0, 160.0)), float(rng.uniform(-160.0, 160.0)))
        if math.hypot(start_xy[0] - 100.0, start_xy[1]) < 85.0:
            start_xy = (0.0, float(rng.choice([-110.0, 110.0])))
        hd, tgt = float(rng.uniform(-180.0, 180.0)), (0.0, 0.0)
    sim = autonomy.make_dock_sim(start_xy_mm=start_xy, heading_deg=hd, head_deg=4.0, lift_mm=32.0, noise_seed=seed + ep_idx)
    sim._in_action = True
    sim.control_level = "override"
    frames: List[np.ndarray] = []
    poses: List[Tuple[float, float, float]] = []
    base_turn = float(rng.uniform(-0.7, 0.7))
    for step_i in range(steps):
        frames.append(nomad_cam.render_head_cam_rgb(sim))
        px, py, _ = sim.pose_mm()
        poses.append((px, py, float(sim._yaw())))
        if is_goal_ep:
            wl, wr = _approach_wheels(sim, tgt)
        else:
            wl, wr, base_turn = _explore_wheels(sim, rng, base_turn, step_i)
        sim._wheel_target = [wl, wr]
        sim.step(STEP_TICKS)
        sim._publish_state()
    return frames, poses


def _build_samples(episodes: List[Tuple[List[np.ndarray], List[Tuple[float, float, float]]]]) -> Dict[str, np.ndarray]:
    assert len(episodes) >= 2, "need at least 2 episodes for positive and negative goals"
    frames_arr = np.stack([np.stack(ep[0], axis=0) for ep in episodes], axis=0)
    ep_count, ep_steps = frames_arr.shape[0], frames_arr.shape[1]
    horizon = nomad_model.HORIZON
    scale = nomad_model.WAYPOINT_SCALE_MM
    anchors = ep_steps - horizon - 1
    assert anchors >= 2, "episodes too short for horizon"
    total = ep_count * anchors
    ep_ids = np.repeat(np.arange(ep_count, dtype=np.int32), anchors)
    step_ids = np.tile(np.arange(anchors, dtype=np.int32), ep_count)
    way = np.zeros((total, horizon, 2), dtype=np.float32)
    for row, (ei, t) in enumerate(zip(ep_ids.tolist(), step_ids.tolist())):
        x0, y0, yaw0 = episodes[ei][1][t]
        cy, sy = math.cos(yaw0), math.sin(yaw0)
        for k in range(1, horizon + 1):
            xj, yj, _ = episodes[ei][1][min(t + k, ep_steps - 1)]
            dx, dy = xj - x0, yj - y0
            way[row, k - 1, 0] = (cy * dx + sy * dy) / scale
            way[row, k - 1, 1] = (-sy * dx + cy * dy) / scale
    return {"frames": frames_arr, "ep_ids": ep_ids, "step_ids": step_ids, "waypoints": way}


def _sample_batch(ds: Dict[str, np.ndarray], idx: np.ndarray, rng: np.random.Generator) -> Tuple[torch.Tensor, ...]:
    frames = ds["frames"]
    ep_ids, step_ids = ds["ep_ids"][idx], ds["step_ids"][idx]
    bsz, ep_count, ep_steps = len(idx), frames.shape[0], frames.shape[1]
    obs_u8 = np.zeros((bsz, nomad_model.CONTEXT_FRAMES, nomad_model.IMG_H, nomad_model.IMG_W, 3), dtype=np.uint8)
    goal_u8 = np.zeros((bsz, nomad_model.IMG_H, nomad_model.IMG_W, 3), dtype=np.uint8)
    dist = np.zeros((bsz, 1), dtype=np.float32)
    mask = (rng.random((bsz, 1)) < 0.45).astype(np.float32)
    neg = rng.random((bsz, 1)) < 0.20
    for i in range(bsz):
        ei, t = int(ep_ids[i]), int(step_ids[i])
        for f_i, back in enumerate((3, 2, 1, 0)):
            obs_u8[i, f_i] = frames[ei, max(0, t - back)]
        if neg[i, 0] and mask[i, 0] < 0.5:
            other_ei = (ei + int(rng.integers(1, ep_count))) % ep_count
            other_t = int(rng.integers(0, ep_steps))
            goal_u8[i] = frames[other_ei, other_t]
            dist[i, 0] = nomad_model.MAX_DIST_STEPS
        else:
            d_step = int(rng.integers(1, min(15, ep_steps - t)))
            goal_u8[i] = frames[ei, t + d_step]
            dist[i, 0] = float(d_step)
    obs_t = torch.from_numpy(np.transpose(obs_u8, (0, 1, 4, 2, 3)).reshape(bsz, 12, 36, 64).astype(np.float32) / 255.0)
    goal_t = torch.from_numpy(np.transpose(goal_u8, (0, 3, 1, 2)).astype(np.float32) / 255.0)
    way_t = torch.from_numpy(ds["waypoints"][idx])
    mask_t = torch.from_numpy(mask)
    dist_t = torch.from_numpy(dist)
    act_weight = torch.from_numpy((1.0 - neg.astype(np.float32)))
    return obs_t, goal_t, mask_t, way_t, dist_t, act_weight


def train_nomad_models(ds: Dict[str, np.ndarray], epochs: int, batch_size: int,
                       seed: int) -> Tuple[nomad_model.NoMaDEncoder, nomad_model.NoMaDDenoiser, List[Dict[str, Any]]]:
    """Train encoder + denoiser with DDPM noise objective and goal-distance loss."""
    assert 1 <= epochs <= 200 and batch_size >= 4, "invalid training hyperparameters"
    torch.set_num_threads(4)
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed + 11)
    enc = nomad_model.NoMaDEncoder()
    den = nomad_model.NoMaDDenoiser()
    opt = torch.optim.AdamW(list(enc.parameters()) + list(den.parameters()), lr=1.5e-3, weight_decay=1e-4)
    sched_opt = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=max(1, epochs))
    ab = torch.from_numpy(nomad_model.diffusion_schedule()["alphas_cumprod"])
    total_n = len(ds["ep_ids"])
    curve: List[Dict[str, Any]] = []
    for epoch in range(epochs):
        t0 = time.perf_counter()
        perm = rng.permutation(total_n)
        d_sum = dist_sum = 0.0
        batches = max(1, (total_n + batch_size - 1) // batch_size)
        for b in range(batches):
            sl = perm[b * batch_size:min(total_n, (b + 1) * batch_size)]
            obs_t, goal_t, mask_t, way_t, dist_t, act_w = _sample_batch(ds, sl, rng)
            t_idx = torch.randint(0, nomad_model.DIFFUSION_STEPS, (len(sl),))
            eps = torch.randn_like(way_t)
            ab_t = ab[t_idx].reshape(-1, 1, 1)
            noisy = torch.sqrt(ab_t) * way_t + torch.sqrt(1.0 - ab_t) * eps
            ctx, pred_d = enc(obs_t, goal_t, mask_t)
            pred_eps = den(noisy, ctx, t_idx.float())
            loss_diff = (((pred_eps - eps) ** 2).mean(dim=(1, 2)) * act_w.squeeze(1)).mean()
            loss_dist = ((1.0 - mask_t) * (pred_d - dist_t) ** 2).mean()
            loss = loss_diff + 0.08 * loss_dist
            opt.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(list(enc.parameters()) + list(den.parameters()), 2.0)
            opt.step()
            d_sum += float(loss_diff.item())
            dist_sum += float(loss_dist.item())
        sched_opt.step()
        curve.append({"epoch": epoch, "diff_loss": round(d_sum / batches, 5),
                      "dist_loss": round(dist_sum / batches, 4), "wall_s": round(time.perf_counter() - t0, 3)})
    return enc, den, curve


def write_parity_artifacts(out_dir: Path, ds: Dict[str, np.ndarray],
                           enc: nomad_model.NoMaDEncoder, den: nomad_model.NoMaDDenoiser) -> None:
    """Save deterministic PyTorch parity samples (parity.npz and web test nomad_parity.json)."""
    assert out_dir.is_dir(), "out_dir must exist"
    rng = np.random.default_rng(123)
    obs_t, goal_t, _, _, _, _ = _sample_batch(ds, np.array([0, min(5, len(ds["ep_ids"]) - 1)], dtype=np.int32), rng)
    enc.eval()
    den.eval()
    with torch.inference_mode():
        m0 = torch.tensor([[0.0], [1.0]], dtype=torch.float32)
        ctx, dist = enc(obs_t, goal_t, m0)
        init_noise = rng.standard_normal((2, nomad_model.HORIZON, nomad_model.ACT_DIM)).astype(np.float32)
        ts = torch.tensor([15.0, 15.0], dtype=torch.float32)
        noise_out = den(torch.from_numpy(init_noise), ctx, ts)
    runner = nomad_onnx.OnnxNoMaD(out_dir)
    trajs0 = runner.ddim_sample(ctx[0].numpy(), num_samples=2, init_noise=init_noise)
    np.savez(out_dir / PARITY_NPZ, obs_stack=obs_t.numpy(), goal_img=goal_t.numpy(), goal_mask=m0.numpy(),
             context=ctx.numpy(), pred_dist=dist.numpy(), init_noise=init_noise, pred_noise=noise_out.numpy(),
             trajectories=trajs0)
    fixture = {
        "obs_stack": np.rint(obs_t[0].numpy() * 255.0).astype(int).reshape(-1).tolist(),
        "goal_img": np.rint(goal_t[0].numpy() * 255.0).astype(int).reshape(-1).tolist(),
        "alt_goal_img": np.rint(goal_t[1].numpy() * 255.0).astype(int).reshape(-1).tolist(),
        "context_unmasked": [round(float(x), 7) for x in ctx[0].numpy().tolist()],
        "pred_dist_unmasked": round(float(dist[0, 0].item()), 7),
        "init_noise": [round(float(x), 7) for x in init_noise.reshape(-1).tolist()],
        "pred_noise_t15": [round(float(x), 7) for x in noise_out[0].numpy().reshape(-1).tolist()],
        "ddim_trajectories": [[[round(float(c), 5) for c in wp] for wp in tr] for tr in trajs0.tolist()],
    }
    (out_dir / PARITY_JSON).write_text(json.dumps(fixture, separators=(",", ":")) + "\n", encoding="utf-8")


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", required=True, help="output directory for ONNX + Web artifacts")
    parser.add_argument("--episodes", type=int, default=80)
    parser.add_argument("--steps", type=int, default=38)
    parser.add_argument("--epochs", type=int, default=45)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args(argv)
    if args.episodes < 2 or args.steps < 12 or args.epochs < 1 or args.batch_size < 4:
        return EXIT_USAGE
    started = time.perf_counter()
    episodes = [_rollout_episode(i, args.steps, args.seed) for i in range(args.episodes)]
    ds = _build_samples(episodes)
    enc, den, curve = train_nomad_models(ds, args.epochs, args.batch_size, args.seed)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    meta = {"episodes": args.episodes, "steps": args.steps, "epochs": args.epochs,
            "samples": int(len(ds["ep_ids"])), "seed": args.seed,
            "final_diff_loss": curve[-1]["diff_loss"], "final_dist_loss": curve[-1]["dist_loss"],
            "wall_s": round(time.perf_counter() - started, 2)}
    (out_dir / CURVE_NAME).write_text(json.dumps({"meta": meta, "curve": curve}, indent=1) + "\n", encoding="utf-8")
    nomad_model.export_onnx(enc, den, out_dir)
    nomad_model.export_web_weights(enc, den, out_dir, meta)
    write_parity_artifacts(out_dir, ds, enc, den)
    print(json.dumps({"ok": True, **meta}))
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
