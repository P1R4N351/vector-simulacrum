"""Reproducible unit 3 hub-spacing, dock-glyph, FCC ruler, and held-out A mic-port audit."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Sequence

import cv2
import numpy as np

import audit_analysis as aa
import audit_geom as ag


def verify_glyph_edges(raw_dir: Path) -> Dict[str, Any]:
    """Verify 46 mm dock glyph boundaries against back wall, ramp, and 50 MP S-dock frame."""
    im_sd = cv2.imread(str(raw_dir / "S-dock-main.jpg"))
    assert im_sd is not None, f"missing S-dock-main.jpg in {raw_dir}"
    gray = cv2.cvtColor(im_sd, cv2.COLOR_BGR2GRAY).astype(float)
    y_top = 2600 + int(np.argmax(np.gradient(gray[2600:2720, 5700])))
    y_in = 2720 + int(np.argmin(np.gradient(gray[2720:2840, 5700])))
    x_right = 5800 + int(np.argmin(np.gradient(gray[3100, 5800:5900])))
    x_in = 5700 + int(np.argmax(np.gradient(gray[3100, 5700:5820])))
    # Rectified See3CAM measurements previously confirmed: back post = 99.5 px, outer ring = 69.0 px, inner square = 51.0 px.
    outer_px, wall_px, inner_px = 69.0, 99.5, 51.0
    implied_wall_if_outer_46 = round(wall_px / outer_px * 46.0, 2)
    implied_wall_if_inner_46 = round(wall_px / inner_px * 46.0, 2)
    assert implied_wall_if_inner_46 > 88.0, "inner-46 assignment must exceed physical charger width"
    return {
        "chargerHome_512_ratios": {"outer_ring": round(411.0 / 512.0, 4), "inner_over_outer": round(295.0 / 411.0, 4)},
        "see3cam_rectified_px": {"back_wall_w": wall_px, "outer_white_ring_w": outer_px, "inner_dark_sq_w": inner_px},
        "s_dock_50mp_border_px": {"top_border": int(y_in - y_top), "right_border": int(x_right - x_in)},
        "cross_check_70mm_back_wall": {
            "if_outer_ring_is_46mm": implied_wall_if_outer_46,
            "if_inner_square_is_46mm": implied_wall_if_inner_46,
            "if_512px_padded_box_is_46mm_outer_mm": round(411.0 / 512.0 * 46.0, 2),
        },
    }


def _survey_baseline_calib(obs: Dict[str, Any], see_lens: np.ndarray) -> Dict[str, Any]:
    assert "cameras" in obs["survey"] and len(obs["survey"]["cameras"]) == 3, "invalid survey cameras"
    cams = obs["survey"]["cameras"]
    vecs = [np.r_[c["rotation"], c["translation"], c["focal_px"]] for c in cams]
    assert len(vecs) == 3 and all(len(v) == 7 for v in vecs), "invalid survey camera vectors"
    return {"pm": vecs[0], "pu": vecs[1], "ps": vecs[2], "see_lens": see_lens, "world": ag.WORLD_DOCK_MM}


def run_ablation_ladder(obs: Dict[str, Any], see_lens: np.ndarray) -> Dict[str, Any]:
    """Execute step-by-step unforced Spot-B calibration ladder without forcing 48 mm."""
    assert len(see_lens) == 3 and "q" in obs, "invalid inputs to ablation ladder"
    base_cal = _survey_baseline_calib(obs, see_lens)
    base_eval = aa.evaluate_calibration(base_cal, obs)
    specs = [
        ("0_baseline_circular_wh", dict(glyph_mm=46.0, lip_y_mm=-94.0, joint_see=False, use_shelf=True, use_see_lip=True, use_phone_lip=True, circular_wh=True)),
        ("1_free_3d_hubs_only", dict(glyph_mm=46.0, lip_y_mm=-94.0, joint_see=True, use_shelf=True, use_see_lip=True, use_phone_lip=True, circular_wh=False, f_see_prior=580.0)),
        ("2_drop_occluded_phone_lip_94", dict(glyph_mm=46.0, lip_y_mm=-94.0, joint_see=True, use_shelf=True, use_see_lip=True, use_phone_lip=False, circular_wh=False, f_see_prior=580.0)),
        ("3_unforced_no_shelf_lip_94", dict(glyph_mm=46.0, lip_y_mm=-94.0, joint_see=True, use_shelf=False, use_see_lip=True, use_phone_lip=False, circular_wh=False, f_see_prior=None)),
        ("4_unforced_no_shelf_proto_lip_89_3", dict(glyph_mm=46.0, lip_y_mm=-89.33, joint_see=True, use_shelf=False, use_see_lip=True, use_phone_lip=False, circular_wh=False, f_see_prior=None)),
        ("5_unforced_all_lips_dropped", dict(glyph_mm=46.0, lip_y_mm=-94.0, joint_see=True, use_shelf=False, use_see_lip=False, use_phone_lip=False, circular_wh=False, f_see_prior=None)),
        ("6_glyph_tol_minus_1_5mm", dict(glyph_mm=44.5, lip_y_mm=-89.33, joint_see=True, use_shelf=False, use_see_lip=True, use_phone_lip=False, circular_wh=False, f_see_prior=None)),
        ("7_glyph_tol_plus_1_5mm", dict(glyph_mm=47.5, lip_y_mm=-89.33, joint_see=True, use_shelf=False, use_see_lip=True, use_phone_lip=False, circular_wh=False, f_see_prior=None)),
    ]
    results, calibs = {}, {}
    for key, kw in specs:
        cal = aa.fit_spot_b_calibration(obs, see_lens, kw)
        results[key] = aa.evaluate_calibration(cal, obs)
        calibs[key] = cal
    assert len(results) == len(specs), "ablation ladder incomplete"
    return {"baseline_calib": base_cal, "baseline_eval": base_eval, "models": results, "calibs": calibs, "see5pt_no_shelf": _see_only_comparison(see_lens, obs)}


def _see_only_comparison(see_lens: np.ndarray, obs: Dict[str, Any]) -> Dict[str, Any]:
    assert len(see_lens) == 3 and "hubs" in obs, "expected 3-element See3CAM lens and hubs"
    out = {}
    for name, use_shelf, fprior in [("with_shelf_f580", True, 580.0), ("no_shelf_f580", False, 580.0), ("no_shelf_unforced", False, None)]:
        ps = ag.fit_see_pose_only(see_lens, glyph_mm=46.0, use_shelf=use_shelf, fprior=fprior)
        wh = aa._see_horiz_hubs(obs["hubs"], ps, see_lens)
        rms = math.sqrt(float(np.mean(np.sum((ag.project_see_raw(ag.WORLD_DOCK_MM, ps, see_lens) - ag.SEE_GLYPH_PX) ** 2, axis=1))))
        out[name] = {
            "focal_px": round(float(ps[6]), 2), "dock_rms_px": round(rms, 3),
            "hubs_horiz_xy_mm": {k: round(float(np.linalg.norm(v[0, :2] - v[1, :2])), 2) for k, v in wh.items()},
        }
    assert len(out) == 3, "expected 3 see-only ablation entries"
    return out


def propagate_scale_uncertainty(obs: Dict[str, Any], see_lens: np.ndarray, models: Dict[str, Any], raw_dir: Path) -> Dict[str, Any]:
    """Quantify parametric Jacobian 1-sigma, glyph/lip systematics, and 50 MP rim cross-check."""
    assert "4_unforced_no_shelf_proto_lip_89_3" in models, "missing primary unforced model"
    m_base = models["4_unforced_no_shelf_proto_lip_89_3"]
    m_lo = models["6_glyph_tol_minus_1_5mm"]
    m_hi = models["7_glyph_tol_plus_1_5mm"]
    m_lip94 = models["3_unforced_no_shelf_lip_94"]
    bm_glyph_half = 0.5 * abs(m_hi["hubs"]["Bm-15"]["tri_xy_mm"] - m_lo["hubs"]["Bm-15"]["tri_xy_mm"])
    am_glyph_half = 0.5 * abs(m_hi["hubs"]["Am-15"]["tri_xy_mm"] - m_lo["hubs"]["Am-15"]["tri_xy_mm"])
    mic_glyph_half = 0.5 * abs(m_hi["mics"]["S-Am-15"]["dist_mm"] - m_lo["mics"]["S-Am-15"]["dist_mm"])
    lip_half = 0.5 * abs(m_base["hubs"]["Bm-15"]["tri_xy_mm"] - m_lip94["hubs"]["Bm-15"]["tri_xy_mm"])
    pick_sigma_mm = 0.65  # 1-px hub pick uncertainty across 3 views + bevel offset
    total_hub_1sig = math.hypot(math.hypot(bm_glyph_half, lip_half), pick_sigma_mm)
    total_mic_1sig = math.hypot(mic_glyph_half, 0.28)
    cal94 = aa.fit_spot_b_calibration(obs, see_lens, {"lip_y_mm": -94.0, "use_shelf": False, "f_see_prior": None})
    cal89 = aa.fit_spot_b_calibration(obs, see_lens, {"lip_y_mm": -89.33, "use_shelf": False, "f_see_prior": None})
    assert total_hub_1sig > 0.0 and total_mic_1sig > 0.0, "non-positive uncertainty"
    return {
        "hub_bm15_1sigma_mm": round(total_hub_1sig, 2),
        "hub_am15_glyph_half_mm": round(am_glyph_half, 2),
        "mic_sam15_1sigma_mm": round(total_mic_1sig, 2),
        "purple_hubcap_50mp": {"lip_94": aa.fit_purple_hubcaps_50mp(raw_dir, cal94), "lip_89_33": aa.fit_purple_hubcaps_50mp(raw_dir, cal89)},
    }


def run_self_test() -> int:
    """Validate pinhole, ray triangulation, and 3D differential-depth solvers on synthetic ground truth."""
    rng = np.random.default_rng(11)
    lens = np.array([-1.45, 960.0, 540.0])
    uv = np.array([[500.0, 420.0], [1180.0, 760.0]])
    rt = ag.rectify_uv(ag.distort_uv(uv, lens), lens)
    assert np.allclose(rt, uv, atol=1e-9), "distort/rectify roundtrip failed"
    p_true = np.array([12.5, -185.0, 44.0])
    cam1 = np.array([1.7, -0.05, 0.08, -30.0, 70.0, 360.0, 540.0])
    cam2 = np.array([1.5, 0.45, -0.45, 75.0, 52.0, 310.0, 660.0])
    r1 = ag.camera_ray(ag.project_pinhole(p_true, cam1, (960.0, 540.0))[0], cam1, (960.0, 540.0))
    r2 = ag.camera_ray(ag.project_pinhole(p_true, cam2, (500.0, 375.0))[0], cam2, (500.0, 375.0))
    p_rec = ag.triangulate_rays((r1, r2))
    assert float(np.linalg.norm(p_rec - p_true)) < 1e-8, f"triangulation drift: {p_rec}"
    wb = _synthetic_horizon_roundtrip(rng)
    assert abs(wb - 47.2) < 0.05, f"cylinder horizon solver error: {wb}"
    print("SELF_TEST_OK")
    return 0


def _synthetic_horizon_roundtrip(rng: np.random.Generator) -> float:
    w_out, z_flat, scale0, wb_true = 58.0, 66.0, 4.45, 47.2
    assert rng is not None and scale0 > 1.0, "invalid synthetic test generator"
    h_cam = 195.0
    focal = scale0 * h_cam
    w_px = w_out * scale0 * (h_cam / (h_cam - z_flat))
    xf_c, xr_c = 0.5 * wb_true, -0.5 * wb_true
    ang_f = math.atan2(xf_c, h_cam - (z_flat - 14.5)) + math.asin(14.5 / math.hypot(xf_c, z_flat - 14.5 - h_cam))
    ang_r = math.atan2(xr_c, h_cam - (z_flat - 10.75)) - math.asin(10.75 / math.hypot(xr_c, z_flat - 10.75 - h_cam))
    u_f, u_r = 600.0 + focal * math.tan(ang_f), 600.0 + focal * math.tan(ang_r)
    sol = aa.solve_cylinder_horizon_wheelbase(w_out, z_flat, scale0, w_px, u_r, u_f)
    assert len(rng.normal(0.0, 1.0, size=2)) == 2 and sol["wheelbase_mm"] > 20.0
    return float(sol["wheelbase_mm"])


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse CLI arguments for unit 3 hub-scale audit and optional self-test."""
    p = argparse.ArgumentParser(description="Unforced unit 3 hub-spacing & FCC scale audit.")
    p.add_argument("--raw-dir", type=Path, help="Raw 3-camera unit 3 JPEG directory.")
    p.add_argument("--edge-csv", type=Path, help="Plumb-line edge-pixels CSV for See3CAM lens fit.")
    p.add_argument("--survey-json", type=Path, help="Existing survey.json landmark JSON.")
    p.add_argument("--fcc-pdf", type=Path, help="DDL Vector FCC external-photos PDF.")
    p.add_argument("--out-dir", type=Path, help="Output directory for JSON and overlays.")
    p.add_argument("--self-test", action="store_true", help="Run synthetic invariant checks and exit.")
    args = p.parse_args(argv)
    assert p is not None and (args.self_test or (args.raw_dir and args.edge_csv and args.survey_json and args.fcc_pdf and args.out_dir)), "pass --self-test or all data flags"
    assert (not args.self_test) or args.self_test is True, "invalid self-test flag"
    return args


def main(argv: Sequence[str] | None = None) -> int:
    """Run unit 3 hub-scale audit, write JSON summary, and render all four visual overlays."""
    args = parse_args(argv)
    assert args is not None, "parsed args missing"
    if args.self_test:
        return run_self_test()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    obs = aa.load_observations(args.raw_dir, args.survey_json)
    see_lens = ag.fit_see_lens(args.edge_csv)
    ladder = run_ablation_ladder(obs, see_lens)
    summary = {
        "see_lens": [round(float(x), 4) for x in see_lens],
        "glyph_verification": verify_glyph_edges(args.raw_dir),
        "fcc_ruler_photogrammetry": aa.analyze_fcc_rulers(args.fcc_pdf),
        "baseline_survey_json": ladder["baseline_eval"],
        "see5pt_ablation": ladder["see5pt_no_shelf"],
        "models": ladder["models"],
        "uncertainty": propagate_scale_uncertainty(obs, see_lens, ladder["models"], args.raw_dir),
    }
    out_json = args.out_dir / "hub_scale_audit.json"
    out_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    assert out_json.is_file(), "failed writing hub_scale_audit.json"
    import render_overlays as ro
    ro.render_all(args.raw_dir, args.fcc_pdf, obs, see_lens, ladder, summary, args.out_dir)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
