"""Numerical experiments for unit 3 hub spacing, FCC ruler photogrammetry, and held-out mic ports."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

import audit_geom as ag

HUB_CROPS_REL: Dict[str, List[Optional[List[List[float]]]]] = {
    "Bm-15": [[[153, 279], [220, 335]], [[100, 177], [136, 211]], [[240, 257], [398, 300]]],
    "Bp0": [None, [[164, 220], [104, 196]], [[475, 303], [302, 298]]],
    "Am-15": [None, [[69, 153], [91, 177]], [[55, 217], [193, 255]]],
}
CROP_OFFSETS = {
    "main": np.array([90.0, 238.0]),
    "ultra": np.array([250.0, 300.0]),
    "see1": np.array([230.0, 252.0]),
}
PHONE_LIPS_PX = [np.array([562.0, 518.0]), np.array([528.0, 470.0])]
PHONE_SCREWS_PX = [
    np.array([[211, 132], [211, 232], [607, 131], [607, 246]], float) / 1.55 + [445.0, 280.0],
    np.array([[142, 149], [142, 202], [360, 156], [360, 219]], float) / 1.55 + [445.0, 280.0],
]
SEE_SCREWS_PX = np.array([[29, 84], [35, 178], [438, 91], [434, 187]], float) / 2.3 + [760.0, 515.0]
CAMS = ("main", "ultra", "see1")


def load_observations(raw_dir: Path, survey_json: Path) -> Dict[str, Any]:
    """Load Bm-15 phone dock detections and survey.json landmarks without mutation."""
    assert raw_dir.is_dir() and survey_json.is_file(), "missing raw_dir or survey_json"
    im_m = cv2.imread(str(raw_dir / "Bm-15-main.jpg"))
    im_u = cv2.imread(str(raw_dir / "Bm-15-ultra.jpg"))
    assert im_m is not None and im_u is not None, "failed loading Bm-15 phone images"
    q_m = ag.detect_glyph_quad(im_m, is_ultra=False)
    q_u = ag.detect_glyph_quad(im_u, is_ultra=True)
    survey = json.loads(survey_json.read_text(encoding="utf-8"))
    hubs_full = {
        label: [
            None if HUB_CROPS_REL[label][i] is None else (np.asarray(HUB_CROPS_REL[label][i], float) + CROP_OFFSETS[CAMS[i]])
            for i in range(3)
        ]
        for label in ("Bm-15", "Bp0", "Am-15")
    }
    return {"q": (q_m, q_u), "hubs": hubs_full, "survey": survey}


def fit_spot_b_calibration(obs: Dict[str, Any], see_lens: np.ndarray, opts: Dict[str, Any]) -> Dict[str, Any]:
    """Fit 3-camera Spot-B calibration using explicit structural ablation options."""
    assert len(see_lens) == 3 and "q" in obs and "hubs" in obs, "invalid see lens or obs"
    glyph_mm = float(opts.get("glyph_mm", 46.0))
    lip_y_mm = float(opts.get("lip_y_mm", -94.0))
    joint_see = bool(opts.get("joint_see", True))
    circular_wh = bool(opts.get("circular_wh", False))
    world = ag.scaled_world(glyph_mm, lip_y_mm)
    see0 = ag.fit_see_pose_only(see_lens, glyph_mm=glyph_mm, use_shelf=True, fprior=580.0)
    pm0 = ag.seed_phone_pose(obs["q"][0], False, PHONE_LIPS_PX[0])
    pu0 = ag.seed_phone_pose(obs["q"][1], True, PHONE_LIPS_PX[1])
    wh_see = _see_horiz_hubs(obs["hubs"], see0, see_lens)
    s0 = _seed_screws(see0, see_lens, 22.0)
    h0 = np.vstack([wh_see["Bm-15"], wh_see["Bp0"]])
    res_fn = _build_ba_residual(
        obs, see_lens, world, see0, wh_see,
        joint_see=joint_see, use_shelf=bool(opts.get("use_shelf", False)),
        use_see_lip=bool(opts.get("use_see_lip", True)), use_phone_lip=bool(opts.get("use_phone_lip", False)),
        circular_wh=circular_wh, f_see_prior=opts.get("f_see_prior", None),
    )
    x0 = np.r_[pm0, pu0, *([see0] if joint_see else []), s0.ravel(), *([] if circular_wh else [h0.ravel()])]
    sol = least_squares(res_fn, x0, loss="soft_l1", max_nfev=6000)
    pm, pu = sol.x[:7], sol.x[7:14]
    ps = sol.x[14:21] if joint_see else see0
    screws3d = sol.x[(21 if joint_see else 14) : (33 if joint_see else 26)].reshape(4, 3)
    assert np.all(np.isfinite(sol.x)) and pm[6] > 0.0 and pu[6] > 0.0, "Spot-B solve failed"
    return {"pm": pm, "pu": pu, "ps": ps, "see_lens": see_lens, "world": world, "screws3d": screws3d, "sol": sol}


def _see_horiz_hubs(hubs: Dict[str, Any], ps: np.ndarray, lens: np.ndarray) -> Dict[str, np.ndarray]:
    assert len(ps) == 7 and len(lens) == 3, "invalid see pose or lens"
    out = {}
    for label in ("Bm-15", "Bp0", "Am-15"):
        pts = [
            ag.intersect_plane_z(*_see_crop_ray(uv, ps, lens), z_mm)
            for uv, z_mm in zip(hubs[label][2], ag.HUB_Z_NOMINAL)
        ]
        out[label] = np.asarray(pts, dtype=float)
    return out


def _seed_screws(ps: np.ndarray, lens: np.ndarray, depth_y: float) -> np.ndarray:
    assert len(ps) == 7 and depth_y > 0.0, "invalid wall-screw seed depth"
    out = []
    for uv in SEE_SCREWS_PX:
        xy = (ag.rectify_uv(uv, lens) - lens[1:]) / float(ps[6])
        inv = Rotation.from_rotvec(-ps[:3])
        ori, dirv = inv.apply(-ps[3:6]), inv.apply(np.r_[xy, 1.0])
        dirv = dirv / np.linalg.norm(dirv)
        out.append(ori + dirv * ((depth_y - ori[1]) / dirv[1]))
    return np.asarray(out, dtype=float)


def _see_crop_ray(uv: np.ndarray, ps: np.ndarray, lens: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    assert len(ps) == 7 and len(lens) == 3, "invalid see ray parameters"
    return ag.camera_ray(np.asarray(uv, float) + ag.SEE_CROP_OFFSET, ps, lens[1:])


def _see_crop_proj(xyz: np.ndarray, ps: np.ndarray, lens: np.ndarray) -> np.ndarray:
    assert len(ps) == 7 and len(lens) == 3, "invalid see proj parameters"
    return ag.project_pinhole(xyz, ps, lens[1:]) - ag.SEE_CROP_OFFSET


def _build_ba_residual(
    obs: Dict[str, Any], see_lens: np.ndarray, world: np.ndarray, see0: np.ndarray, wh_see: Dict[str, np.ndarray],
    *, joint_see: bool, use_shelf: bool, use_see_lip: bool, use_phone_lip: bool,
    circular_wh: bool, f_see_prior: Optional[float],
):
    assert len(world) == 5 and len(see0) == 7, "expected 5 dock points and 7-param seed"

    def _residual(x: np.ndarray) -> np.ndarray:
        assert len(x) >= 26 and np.all(np.isfinite(x)), "non-finite BA parameter vector"
        pm, pu = x[:7], x[7:14]
        idx = 14
        ps = x[14:21] if joint_see else see0
        if joint_see:
            idx = 21
        s3 = x[idx : idx + 12].reshape(4, 3)
        idx += 12
        h3 = None if circular_wh else x[idx : idx + 12].reshape(4, 3)
        out: List[np.ndarray] = []
        _append_glyph_terms(out, obs, world, pm, pu, ps, see_lens, joint_see, use_shelf, use_see_lip, use_phone_lip, f_see_prior)
        _append_screw_and_hub_terms(out, obs, pm, pu, ps, see_lens, s3, h3, wh_see)
        res_vec = np.concatenate(out)
        assert len(res_vec) >= 30, "unexpectedly small BA residual vector"
        return res_vec

    return _residual


def _append_glyph_terms(
    out: List[np.ndarray], obs: Dict[str, Any], world: np.ndarray, pm: np.ndarray, pu: np.ndarray,
    ps: np.ndarray, see_lens: np.ndarray, joint_see: bool, use_shelf: bool, use_see_lip: bool,
    use_phone_lip: bool, f_see_prior: Optional[float],
) -> None:
    assert len(out) == 0 and len(world) == 5, "expected empty residual list"
    w4, w_lip = world[:4], world[4:5]
    out.append(((ag.project_pinhole(w4, pm, (500.0, 375.0)) - obs["q"][0]) / 1.0).ravel())
    out.append(((ag.project_pinhole(w4, pu, (500.0, 375.0)) - obs["q"][1]) / 1.0).ravel())
    if joint_see:
        out.append(((ag.project_see_raw(w4, ps, see_lens) - ag.SEE_GLYPH_PX[:4]) / 1.0).ravel())
        if use_see_lip:
            out.append(((ag.project_see_raw(w_lip, ps, see_lens) - ag.SEE_GLYPH_PX[4:5]) / 1.5).ravel())
        if f_see_prior is not None:
            out.append(np.array([(ps[6] - float(f_see_prior)) / 150.0]))
        if use_shelf:
            out.append(_shelf_perp_term(ps, see_lens))
    if use_phone_lip:
        out.append(((ag.project_pinhole(w_lip, pm, (500.0, 375.0)) - PHONE_LIPS_PX[0]) / 4.0).ravel())
        out.append(((ag.project_pinhole(w_lip, pu, (500.0, 375.0)) - PHONE_LIPS_PX[1]) / 4.0).ravel())


def _shelf_perp_term(ps: np.ndarray, lens: np.ndarray) -> np.ndarray:
    assert len(ps) == 7 and len(lens) == 3, "invalid pose or lens for shelf term"
    rays = (ag.rectify_uv(ag.SEE_SHELF_PX, lens) - lens[1:]) / float(ps[6])
    inv = Rotation.from_rotvec(-ps[:3])
    ori, dirs = inv.apply(-ps[3:6]), inv.apply(np.c_[rays, np.ones(4)])
    a, b, c_pt, d_pt = (ori + (-ori[2]) * dirs / dirs[:, 2:3])[:, :2]
    across = (b - a) / np.linalg.norm(b - a)
    return np.array([((c_pt - a) @ across) / 5.0, ((d_pt - b) @ across) / 5.0])


def _append_screw_and_hub_terms(
    out: List[np.ndarray], obs: Dict[str, Any], pm: np.ndarray, pu: np.ndarray, ps: np.ndarray,
    see_lens: np.ndarray, s3: np.ndarray, h3: Optional[np.ndarray], wh_see: Dict[str, np.ndarray],
) -> None:
    assert s3.shape == (4, 3), "expected 4 wall screw points"
    out.append(((ag.project_pinhole(s3, pm, (500.0, 375.0)) - PHONE_SCREWS_PX[0]) / 1.2).ravel())
    out.append(((ag.project_pinhole(s3, pu, (500.0, 375.0)) - PHONE_SCREWS_PX[1]) / 1.2).ravel())
    out.append(((ag.project_see_raw(s3, ps, see_lens) - SEE_SCREWS_PX) / 1.2).ravel())
    out.append(((s3[:, 1] - s3[:, 1].mean()) / 1.5).ravel())
    for k_pose, label in enumerate(("Bm-15", "Bp0")):
        pts = wh_see[label] if h3 is None else h3[2 * k_pose : 2 * k_pose + 2]
        if obs["hubs"][label][0] is not None:
            out.append(((ag.project_pinhole(pts, pm, (500.0, 375.0)) - obs["hubs"][label][0]) / 1.2).ravel())
        if obs["hubs"][label][1] is not None:
            out.append(((ag.project_pinhole(pts, pu, (500.0, 375.0)) - obs["hubs"][label][1]) / 1.2).ravel())
        if h3 is not None:
            out.append(((_see_crop_proj(pts, ps, see_lens) - obs["hubs"][label][2]) / 1.2).ravel())
    out.append(np.array([
        (pm[6] - ag.PHONE_F_PRIORS[0]) / (0.03 * ag.PHONE_F_PRIORS[0]),
        (pu[6] - ag.PHONE_F_PRIORS[1]) / (0.03 * ag.PHONE_F_PRIORS[1]),
        (float(np.linalg.norm(ag.camera_center(pm) - ag.camera_center(pu))) - ag.PHONE_BASELINE_MM) / 0.5,
    ]))


def evaluate_calibration(calib: Dict[str, Any], obs: Dict[str, Any]) -> Dict[str, Any]:
    """Evaluate frozen Spot-B calibration on hubs, per-camera planes, and held-out A mic ports."""
    assert "pm" in calib and "hubs" in obs, "invalid evaluation inputs"
    pm, pu, ps, lens = calib["pm"], calib["pu"], calib["ps"], calib["see_lens"]
    hub_res = _evaluate_all_hubs(obs["hubs"], pm, pu, ps, lens)
    mic_res = _evaluate_all_mics(obs["survey"]["microphone_ports"], pm, pu, ps, lens)
    dock_rms = _dock_rms_dict(obs["q"], calib["world"], pm, pu, ps, lens)
    assert "Bm-15" in hub_res and "S-Am-15" in mic_res, "incomplete evaluation outputs"
    return {
        "focals_px": {"main": round(float(pm[6]), 2), "ultra": round(float(pu[6]), 2), "see1": round(float(ps[6]), 2)},
        "phone_baseline_mm": round(float(np.linalg.norm(ag.camera_center(pm) - ag.camera_center(pu))), 3),
        "dock_rms_px": dock_rms,
        "hubs": hub_res,
        "mics": mic_res,
    }


def _cam_ray(i: int, uv: Sequence[float], pm: np.ndarray, pu: np.ndarray, ps: np.ndarray, lens: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    assert i in (0, 1, 2), f"invalid camera index: {i}"
    if i == 0:
        return ag.camera_ray(uv, pm, (500.0, 375.0))
    if i == 1:
        return ag.camera_ray(uv, pu, (500.0, 375.0))
    return _see_crop_ray(uv, ps, lens)


def _cam_proj(i: int, xyz: np.ndarray, pm: np.ndarray, pu: np.ndarray, ps: np.ndarray, lens: np.ndarray) -> np.ndarray:
    assert i in (0, 1, 2), f"invalid camera index: {i}"
    if i == 0:
        return ag.project_pinhole(xyz, pm, (500.0, 375.0))[0]
    if i == 1:
        return ag.project_pinhole(xyz, pu, (500.0, 375.0))[0]
    return _see_crop_proj(xyz, ps, lens)[0]


def _dock_rms_dict(
    q_phone: Tuple[np.ndarray, np.ndarray], world: np.ndarray, pm: np.ndarray, pu: np.ndarray, ps: np.ndarray, lens: np.ndarray
) -> Dict[str, float]:
    assert len(world) == 5 and len(q_phone) == 2, "expected 5 dock points and 2 phone quads"
    r_m = math.sqrt(float(np.mean(np.sum((ag.project_pinhole(world[:4], pm, (500.0, 375.0)) - q_phone[0]) ** 2, axis=1))))
    r_u = math.sqrt(float(np.mean(np.sum((ag.project_pinhole(world[:4], pu, (500.0, 375.0)) - q_phone[1]) ** 2, axis=1))))
    r_s = math.sqrt(float(np.mean(np.sum((ag.project_see_raw(world[:4], ps, lens) - ag.SEE_GLYPH_PX[:4]) ** 2, axis=1))))
    return {"main": round(r_m, 3), "ultra": round(r_u, 3), "see1": round(r_s, 3)}


def _evaluate_all_hubs(
    hubs: Dict[str, Any], pm: np.ndarray, pu: np.ndarray, ps: np.ndarray, lens: np.ndarray
) -> Dict[str, Any]:
    assert "Bm-15" in hubs and "Am-15" in hubs, "missing expected poses in hubs"
    out: Dict[str, Any] = {}
    for label in ("Bm-15", "Bp0", "Am-15"):
        uvs = hubs[label]
        pts3 = np.array([ag.triangulate_rays([_cam_ray(i, uvs[i][j], pm, pu, ps, lens) for i in range(3) if uvs[i] is not None]) for j in range(2)])
        hdir = pts3[0, :2] - pts3[1, :2]
        hdir = hdir / np.linalg.norm(hdir)
        nside, p0 = np.array([-hdir[1], hdir[0], 0.0]), pts3.mean(axis=0)
        per_z, per_side = {}, {}
        for i, cname in enumerate(CAMS):
            if uvs[i] is None:
                continue
            pz = np.array([ag.intersect_plane_z(*_cam_ray(i, uvs[i][j], pm, pu, ps, lens), ag.HUB_Z_NOMINAL[j]) for j in range(2)])
            psv = np.array([ag.intersect_vertical_plane(*_cam_ray(i, uvs[i][j], pm, pu, ps, lens), p0, nside) for j in range(2)])
            per_z[cname] = {"d_xy_mm": round(float(np.linalg.norm(pz[0, :2] - pz[1, :2])), 2), "d_3d_mm": round(float(np.linalg.norm(pz[0] - pz[1])), 2)}
            per_side[cname] = {"d_xy_mm": round(float(np.linalg.norm(psv[0, :2] - psv[1, :2])), 2), "d_3d_mm": round(float(np.linalg.norm(psv[0] - psv[1])), 2)}
        reproj = [[None if uvs[i] is None else round(float(np.linalg.norm(_cam_proj(i, pts3[j], pm, pu, ps, lens) - uvs[i][j])), 2) for i in range(3)] for j in range(2)]
        out[label] = {
            "tri_xy_mm": round(float(np.linalg.norm(pts3[0, :2] - pts3[1, :2])), 2),
            "tri_3d_mm": round(float(np.linalg.norm(pts3[0] - pts3[1])), 2),
            "z_mm": [round(float(pts3[0, 2]), 2), round(float(pts3[1, 2]), 2)],
            "reproj_px": reproj, "per_cam_horiz_z": per_z, "per_cam_side_plane": per_side,
        }
    return out


def _evaluate_all_mics(
    mics: Dict[str, Any], pm: np.ndarray, pu: np.ndarray, ps: np.ndarray, lens: np.ndarray
) -> Dict[str, Any]:
    assert "Bm-15" in mics and "S-Am-15" in mics, "missing expected microphone port entries"
    out: Dict[str, Any] = {}
    for label, obs_m in mics.items():
        pts3, reproj, omitted = [], [], []
        for j in range(2):
            used = [(i, np.asarray(obs_m[i][j], float), _cam_ray(i, obs_m[i][j], pm, pu, ps, lens)) for i in range(3) if obs_m[i] is not None]
            pt = ag.triangulate_rays([u[2] for u in used])
            pts3.append(pt)
            reproj.append([round(float(np.linalg.norm(_cam_proj(i, pt, pm, pu, ps, lens) - uv)), 2) for i, uv, _ in used])
            if len(used) == 3:
                omitted.append([round(float(np.linalg.norm(_cam_proj(used[k][0], ag.triangulate_rays([u[2] for idx, u in enumerate(used) if idx != k]), pm, pu, ps, lens) - used[k][1])), 2) for k in range(3)])
        flat_rp = [val for row in reproj for val in row]
        out[label] = {
            "dist_mm": round(float(np.linalg.norm(pts3[1] - pts3[0])), 3),
            "z_mm": [round(float(pts3[0][2]), 2), round(float(pts3[1][2]), 2)],
            "rms_px": round(math.sqrt(float(np.mean(np.square(flat_rp)))), 2),
            "reproj_px": reproj, "omit_cam_px": omitted,
        }
    return out


def fit_purple_hubcaps_50mp(raw_dir: Path, calib: Dict[str, Any]) -> Dict[str, Any]:
    """Fit full-resolution 8160x6120 convex-hull ellipses to Bm-15 purple wheel hubcaps."""
    im_path = raw_dir / "Bm-15-main.jpg"
    assert im_path.is_file(), f"missing {im_path}"
    im = cv2.imread(str(im_path))
    crop = im[3750:5050, 1650:2850]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = cv2.morphologyEx(cv2.inRange(hsv, (115, 42, 35), (165, 255, 255)), cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    nl, lab, st, cen = cv2.connectedComponentsWithStats(mask)
    top2 = sorted(sorted(range(1, nl), key=lambda k: st[k, cv2.CC_STAT_AREA], reverse=True)[:2], key=lambda k: cen[k][0])
    assert len(top2) == 2, "failed detecting both purple hubcaps"
    hulls = [cv2.convexHull(max(cv2.findContours((lab == cid).astype(np.uint8) * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0], key=cv2.contourArea))[:, 0, :].astype(float) / 8.16 + [1650.0 / 8.16, 3750.0 / 8.16] for cid in top2]
    return _project_caps_to_side_plane(hulls, calib)


def _project_caps_to_side_plane(hulls_1k: List[np.ndarray], calib: Dict[str, Any]) -> Dict[str, Any]:
    assert len(hulls_1k) == 2 and "pm" in calib, "expected 2 hulls and calibration"
    pm, pu, ps, lens = calib["pm"], calib["pu"], calib["ps"], calib["see_lens"]
    bm_pts = np.array([ag.triangulate_rays([_cam_ray(i, (np.asarray(HUB_CROPS_REL["Bm-15"][i][j], float) + CROP_OFFSETS[CAMS[i]]), pm, pu, ps, lens) for i in range(3)]) for j in range(2)])
    hdir = bm_pts[0, :2] - bm_pts[1, :2]
    hdir /= np.linalg.norm(hdir)
    nside, p0 = np.array([-hdir[1], hdir[0], 0.0]), bm_pts.mean(axis=0)
    caps = []
    for hull in hulls_1k:
        p3 = np.array([ag.intersect_vertical_plane(*ag.camera_ray(uv, pm, (500.0, 375.0)), p0, nside) for uv in hull])
        xy = np.column_stack([(p3 - p0) @ np.r_[hdir, 0.0], p3[:, 2]])
        c = np.linalg.lstsq(np.column_stack([2.0 * xy[:, 0], 2.0 * xy[:, 1], np.ones(len(xy))]), xy[:, 0] ** 2 + xy[:, 1] ** 2, rcond=None)[0]
        caps.append((c[:2], math.sqrt(float(c[2] + c[0] ** 2 + c[1] ** 2))))
    d3d = float(np.linalg.norm(caps[0][0] - caps[1][0]))
    assert d3d > 20.0 and caps[0][1] > caps[1][1], "implausible fitted purple hubcap radii"
    return {"front_r_mm": round(caps[0][1], 2), "rear_r_mm": round(caps[1][1], 2), "d_3d_mm": round(d3d, 2)}


def analyze_fcc_rulers(pdf_path: Path) -> Dict[str, Any]:
    """Measure DDL Vector 2.0 FCC ruler scales and solve 3D differential-depth axle spacing."""
    jpegs = ag.extract_fcc_jpegs(pdf_path)
    j2_hom = _solve_j2_upright_z0(jpegs[2])
    j1_diff = _solve_j1_differential_depth(jpegs[1])
    return {"jpeg_count": len(jpegs), "j2_upright_z0": j2_hom, "j1_inverted": j1_diff}


def _solve_j2_upright_z0(im2: np.ndarray) -> Dict[str, Any]:
    assert im2.shape[:2] == (900, 1200), "expected 1200x900 FCC image"
    vl = np.array([[60, 137.5, 898], [70, 124.5, 832], [80, 112, 768.5], [90, 100, 708], [100, 88.5, 649.5], [110, 77, 593.5], [120, 66, 539.5], [130, 55.5, 487.5], [140, 45.5, 437.5], [150, 36, 389.5]], float)
    vr = np.array([[50.8, 293, 882], [76.2, 254, 725], [101.6, 218, 581.5], [127.0, 185, 450.5], [152.4, 154.5, 330]], float)
    ht = np.array([[30, 540.5, 896], [40, 604.5, 850.5], [50, 666, 806.5], [60, 725.5, 764], [70, 782.5, 723.5], [80, 837.5, 684], [90, 890.5, 646], [100, 941.5, 609.5], [110, 991, 574], [120, 1038.5, 540], [130, 1084.5, 507]], float)
    pts0 = np.vstack([np.column_stack([np.zeros(len(vl)), vl[:, 0]]), np.column_stack([np.full(len(vr), 25.4), vr[:, 0]]), np.column_stack([31.92 + ht[:, 0] * math.cos(math.radians(8.99)), 31.04 + ht[:, 0] * math.sin(math.radians(8.99))])])
    obs0 = np.vstack([vl[:, 1:], vr[:, 1:], ht[:, 1:]])
    h_mat, _ = cv2.findHomography(pts0, obs0)
    assert h_mat is not None and h_mat.shape == (3, 3), "J2 homography fit failed"
    proj = (h_mat @ np.column_stack([pts0, np.ones(len(pts0))]).T).T
    rms = math.sqrt(float(np.mean(np.sum((proj[:, :2] / proj[:, 2:3] - obs0) ** 2, axis=1))))
    h_inv = np.linalg.inv(h_mat)
    p_l, p_r = h_inv @ np.array([555.0, 795.0, 1.0]), h_inv @ np.array([815.0, 650.0, 1.0])
    plow_w = float(np.linalg.norm(p_l[:2] / p_l[2] - p_r[:2] / p_r[2]))
    return {"homography_rms_px": round(rms, 2), "plow_span_z0_mm": round(plow_w, 2)}


def _solve_j1_differential_depth(im1: np.ndarray) -> Dict[str, Any]:
    assert im1.shape[:2] == (900, 1200), "expected 1200x900 FCC image"
    scale_z0 = 4.450
    w_outer_px, l_tread_px = 389.1, 438.5
    naive_z0_len = l_tread_px / scale_z0
    naive_z0_ax = naive_z0_len - (ag.HUB_Z_NOMINAL[0] + ag.HUB_Z_NOMINAL[1])
    rows = []
    for w_out in (56.0, 58.0, 60.0):
        ax = solve_cylinder_horizon_wheelbase(w_out, 66.0, scale_z0, w_outer_px, 360.5, 799.0)
        rows.append({"assumed_w_outer_mm": w_out, "h_cam_mm": round(ax["h_cam_mm"], 1), "flat_plane_ax_mm": round(l_tread_px / (w_outer_px / w_out) - 25.25, 2), "diff_depth_ax_mm": round(ax["wheelbase_mm"], 2)})
    assert len(rows) == 3 and naive_z0_len > 80.0, "unexpected J1 sweep output"
    return {"scale_z0_px_per_mm": scale_z0, "naive_z0_total_mm": round(naive_z0_len, 2), "naive_z0_axle_mm": round(naive_z0_ax, 2), "depth_corrected_sweep": rows}


def solve_cylinder_horizon_wheelbase(
    w_outer_mm: float, z_flat_mm: float, scale_z0: float, w_outer_px: float, u_rear_px: float, u_front_px: float
) -> Dict[str, float]:
    """Recover fore-aft axle spacing using flat-tread Z_flat vs wheel equator cylinder horizons."""
    assert w_outer_mm > 40.0 and z_flat_mm > 30.0 and scale_z0 > 1.0, "invalid physical geometry"
    m_flat = (w_outer_px / w_outer_mm) / scale_z0
    assert m_flat > 1.02, "expected inverted treads closer to camera than Z=0 ruler"
    h_cam = z_flat_mm * m_flat / (m_flat - 1.0)
    focal = scale_z0 * h_cam

    def _sil_u(xc: float, zc: float, rad: float, sgn: float) -> float:
        d = math.hypot(xc, zc - h_cam)
        ang = math.atan2(xc, h_cam - zc) + sgn * math.asin(rad / d)
        return 600.0 + focal * math.tan(ang)

    xf = float(least_squares(lambda v: _sil_u(float(v[0]), z_flat_mm - 14.5, 14.5, +1.0) - u_front_px, [24.0]).x[0])
    xr = float(least_squares(lambda v: _sil_u(float(v[0]), z_flat_mm - 10.75, 10.75, -1.0) - u_rear_px, [-24.0]).x[0])
    return {"h_cam_mm": h_cam, "focal_px": focal, "wheelbase_mm": xf - xr}
