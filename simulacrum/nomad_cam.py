"""Fast calibrated 64x36 RGB head_cam renderer for NoMaD visual navigation."""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from . import autonomy

IMG_W = 64
IMG_H = 36
IMG_C = 3
MM = 0.001
NEAR_M = 0.005


def _camera_rays_world(sim: Any, spec: Dict[str, Any]) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    import mujoco
    cam_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, spec["name"])
    if cam_id < 0:
        return None
    origin = np.asarray(sim.data.cam_xpos[cam_id], dtype=float).copy()
    raw_r = np.asarray(sim.data.cam_xmat[cam_id], dtype=float).reshape(3, 3)
    mount_down = 0.0 if spec.get("pitch_down_in_model") else float(spec["pitch_down_deg"])
    tilt = math.radians(float(spec.get("rgb_pitch_up_deg", 0.0)) - mount_down)
    ct, st = math.cos(tilt), math.sin(tilt)
    r_cam_w = raw_r @ np.array([[1.0, 0.0, 0.0], [0.0, ct, -st], [0.0, st, ct]], dtype=float)
    us = (np.arange(IMG_W, dtype=float) + 0.5) * (float(spec["width_px"]) / IMG_W)
    vs = (np.arange(IMG_H, dtype=float) + 0.5) * (float(spec["height_px"]) / IMG_H)
    gx, gy = np.meshgrid((us - float(spec["cx_px"])) / float(spec["fx_px"]),
                         (float(spec["cy_px"]) - vs) / float(spec["fy_px"]))
    dirs_c = np.stack((gx, gy, -np.ones_like(gx)), axis=-1).reshape(-1, 3)
    norms = np.linalg.norm(dirs_c, axis=1, keepdims=True)
    dirs_w = (dirs_c / norms) @ r_cam_w.T
    assert dirs_w.shape == (IMG_W * IMG_H, 3), "ray grid shape mismatch"
    return origin, dirs_w


def _ray_obb_batch(origin: np.ndarray, dirs_w: np.ndarray, pos: np.ndarray, rot: np.ndarray,
                   bmin: np.ndarray, bmax: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    o_loc = rot.T @ (origin - pos)
    d_loc = dirs_w @ rot
    parallel = np.abs(d_loc) <= 1e-8
    inside = (o_loc[None, :] >= bmin[None, :]) & (o_loc[None, :] <= bmax[None, :])
    safe_d = np.where(parallel, 1.0, d_loc)
    raw0 = (bmin[None, :] - o_loc[None, :]) / safe_d
    raw1 = (bmax[None, :] - o_loc[None, :]) / safe_d
    t0 = np.where(parallel, np.where(inside, -np.inf, np.inf), np.minimum(raw0, raw1))
    t1 = np.where(parallel, np.where(inside, np.inf, -np.inf), np.maximum(raw0, raw1))
    tmin = np.max(t0, axis=1)
    tmax = np.min(t1, axis=1)
    valid = (tmax >= tmin) & (tmin > NEAR_M)
    t_hit = np.where(valid, tmin, np.inf)
    hit_loc = o_loc[None, :] + t_hit[:, None] * d_loc
    return valid, t_hit, hit_loc


def _shade_backdrop(dirs_w: np.ndarray) -> np.ndarray:
    dz = dirs_w[:, 2]
    up = np.clip(dz * 2.5, 0.0, 1.0)[:, None]
    sky = np.array([[28.0, 34.0, 46.0]]) * (1.0 - up) + np.array([[56.0, 68.0, 88.0]]) * up
    floor = np.array([[18.0, 22.0, 28.0]])
    return np.where((dz >= 0.0)[:, None], sky, floor)


def _shade_table(origin: np.ndarray, dirs_w: np.ndarray, sim: Any,
                 t_best: np.ndarray, rgb: np.ndarray) -> None:
    half_l = 0.5 * float(sim.v["table_length_mm"]) * MM
    half_w = 0.5 * float(sim.v["table_width_mm"]) * MM
    dz = dirs_w[:, 2]
    t_plane = np.where(dz < -1e-6, -float(origin[2]) / np.where(dz < -1e-6, dz, -1.0), np.inf)
    hx = origin[0] + t_plane * dirs_w[:, 0]
    hy = origin[1] + t_plane * dirs_w[:, 1]
    hit = (t_plane > NEAR_M) & (t_plane < t_best) & (np.abs(hx) <= half_l) & (np.abs(hy) <= half_w)
    if not np.any(hit):
        return
    edge_dist = np.minimum(half_l - np.abs(hx), half_w - np.abs(hy))
    is_border = edge_dist < 0.022
    fade = np.clip(1.12 - 0.55 * t_plane, 0.55, 1.0)[:, None]
    wood = np.array([[148.0, 118.0, 86.0]]) * fade
    border = np.array([[224.0, 184.0, 118.0]]) * fade
    t_best[hit] = t_plane[hit]
    rgb[hit] = np.where(is_border[hit, None], border[hit], wood[hit])


def _shade_obstacles(origin: np.ndarray, dirs_w: np.ndarray, sim: Any,
                     t_best: np.ndarray, rgb: np.ndarray) -> None:
    import mujoco
    for g in range(int(sim.model.ngeom)):
        if int(sim.model.geom_type[g]) != int(mujoco.mjtGeom.mjGEOM_BOX):
            continue
        name = mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if not name.startswith("obstacle_"):
            continue
        pos = np.asarray(sim.data.geom_xpos[g], dtype=float)
        rot = np.asarray(sim.data.geom_xmat[g], dtype=float).reshape(3, 3)
        ext = np.asarray(sim.model.geom_size[g], dtype=float)
        valid, th, _ = _ray_obb_batch(origin, dirs_w, pos, rot, -ext, ext)
        closer = valid & (th < t_best)
        if np.any(closer):
            t_best[closer] = th[closer]
            rgb[closer] = np.array([182.0, 190.0, 202.0])


def _shade_cube(origin: np.ndarray, dirs_w: np.ndarray, sim: Any,
                t_best: np.ndarray, rgb: np.ndarray) -> None:
    import mujoco
    cid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    if cid < 0:
        return
    pos = np.asarray(sim.data.xpos[cid], dtype=float)
    rot = np.asarray(sim.data.xmat[cid], dtype=float).reshape(3, 3)
    ext = np.array([0.022, 0.022, 0.022], dtype=float)
    valid, th, hl = _ray_obb_batch(origin, dirs_w, pos, rot, -ext, ext)
    closer = valid & (th < t_best)
    if not np.any(closer):
        return
    ax, ay, az = np.abs(hl[:, 0]), np.abs(hl[:, 1]), np.abs(hl[:, 2])
    on_x, on_y, on_z = ax >= 0.0216, ay >= 0.0216, az >= 0.0216
    u = np.where(on_x, ay, np.where(on_y, ax, ax))
    v = np.where(on_z, ay, az)
    in_sq = (on_x | on_y) & (~on_z) & (u < 0.014) & (v < 0.014)
    white_frame = in_sq & ((np.maximum(u, v) > 0.009) | (np.maximum(u, v) < 0.0045))
    top_led = on_z & (np.maximum(ax, ay) > 0.008) & (np.maximum(ax, ay) < 0.015)
    col = np.full((len(th), 3), [48.0, 52.0, 60.0])
    col[in_sq] = [24.0, 28.0, 34.0]
    col[white_frame] = [236.0, 238.0, 242.0]
    col[top_led] = [60.0, 205.0, 235.0]
    t_best[closer] = th[closer]
    rgb[closer] = col[closer]


def _shade_dock(origin: np.ndarray, dirs_w: np.ndarray, sim: Any,
                t_best: np.ndarray, rgb: np.ndarray) -> None:
    import mujoco
    did = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "dock")
    if did < 0:
        return
    pos = np.asarray(sim.data.xpos[did], dtype=float)
    rot = np.asarray(sim.data.xmat[did], dtype=float).reshape(3, 3)
    for bmin, bmax, is_tower in (
        (np.array([0.0, -0.028, 0.0]), np.array([0.092, 0.028, 0.016]), False),
        (np.array([0.092, -0.028, 0.014]), np.array([0.116, 0.028, 0.082]), True),
    ):
        valid, th, hl = _ray_obb_batch(origin, dirs_w, pos, rot, bmin, bmax)
        closer = valid & (th < t_best)
        if not np.any(closer):
            continue
        col = np.full((len(th), 3), [38.0, 42.0, 50.0] if not is_tower else [44.0, 48.0, 58.0])
        if is_tower:
            u, v = np.abs(hl[:, 1]), np.abs(hl[:, 2] - 0.0485)
            face = (hl[:, 0] < 0.0925) & (u < 0.023) & (v < 0.023)
            bright = face & ((np.maximum(u, v) > 0.015) | ((u < 0.009) & (v < 0.009)))
            col[face] = [20.0, 24.0, 30.0]
            col[bright] = [242.0, 244.0, 248.0]
        t_best[closer] = th[closer]
        rgb[closer] = col[closer]


def render_head_cam_rgb(sim: Any, spec: Optional[Dict[str, Any]] = None) -> np.ndarray:
    """Render calibrated 64x36 uint8 RGB observation from Vector head_cam."""
    assert sim is not None, "sim must not be None"
    cam_spec = spec or autonomy.load_camera_spec()
    rays = _camera_rays_world(sim, cam_spec)
    assert rays is not None, "head_cam missing from model"
    origin, dirs_w = rays
    rgb = _shade_backdrop(dirs_w)
    t_best = np.full(len(dirs_w), np.inf, dtype=float)
    _shade_table(origin, dirs_w, sim, t_best, rgb)
    _shade_obstacles(origin, dirs_w, sim, t_best, rgb)
    _shade_dock(origin, dirs_w, sim, t_best, rgb)
    _shade_cube(origin, dirs_w, sim, t_best, rgb)
    img = np.clip(np.rint(rgb), 0, 255).astype(np.uint8).reshape(IMG_H, IMG_W, IMG_C)
    lift_mm = float(sim.lift_height_mm())
    if 38.0 < lift_mm < 62.0:
        img[-7:, 8:-8, :] = np.array([52, 56, 64], dtype=np.uint8)
    assert img.shape == (IMG_H, IMG_W, IMG_C), "unexpected camera image shape"
    return img
