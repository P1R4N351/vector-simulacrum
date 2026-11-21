"""Render four before/after diagnostic overlays for the unit 3 hub-spacing & FCC scale audit."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

import audit_analysis as aa
import audit_geom as ag


def _rect_see_crop(img_path: Path, lens: np.ndarray) -> np.ndarray:
    im = cv2.rotate(cv2.imread(str(img_path)), cv2.ROTATE_180)
    assert im is not None and len(lens) == 3, f"failed reading {img_path}"
    yy, xx = np.indices((780, 1200))
    uv = ag.distort_uv(np.stack((xx + 310.0, yy + 395.0), axis=2), lens).astype("float32")
    return cv2.cvtColor(cv2.remap(im, uv[:, :, 0], uv[:, :, 1], cv2.INTER_LINEAR), cv2.COLOR_BGR2RGB)


def _rect_see_full(img_path: Path, lens: np.ndarray) -> np.ndarray:
    im = cv2.rotate(cv2.imread(str(img_path)), cv2.ROTATE_180)
    assert im is not None and len(lens) == 3, f"failed reading {img_path}"
    yy, xx = np.indices((1080, 1920))
    uv = ag.distort_uv(np.stack((xx.astype(float), yy.astype(float)), axis=2), lens).astype("float32")
    return cv2.cvtColor(cv2.remap(im, uv[:, :, 0], uv[:, :, 1], cv2.INTER_LINEAR), cv2.COLOR_BGR2RGB)


def _load_rgb_1000(img_path: Path) -> np.ndarray:
    im = cv2.imread(str(img_path))
    assert im is not None, f"missing image {img_path}"
    return cv2.cvtColor(cv2.resize(im, (1000, 750)), cv2.COLOR_BGR2RGB)


def render_glyph_overlay(
    raw_dir: Path, obs: Dict[str, Any], see_lens: np.ndarray, ladder: Dict[str, Any], summary: Dict[str, Any], out_path: Path
) -> None:
    """Render Panel 1: Dock glyph edges, occluded phone lips, and See3CAM shelf orthogonality test."""
    assert out_path.parent.is_dir() and "models" in ladder, "invalid glyph overlay inputs"
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 9.5), dpi=160)
    _draw_glyph_see_panel(axes[0, 0], raw_dir, see_lens, ladder, summary)
    _draw_glyph_50mp_panel(axes[0, 1], raw_dir, summary)
    _draw_lip_occlusion_panel(axes[1, 0], raw_dir, ladder)
    _draw_shelf_ablation_bar(axes[1, 1], ladder, summary)
    fig.suptitle("Dock Glyph Edge Audit, Occluded Phone Lip Picks & Shelf Orthogonality Test", fontsize=13.5, fontweight="bold")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def _draw_glyph_see_panel(ax: Any, raw_dir: Path, see_lens: np.ndarray, ladder: Dict[str, Any], summary: Dict[str, Any]) -> None:
    im = _rect_see_full(raw_dir / "dockpair-see1.jpg", see_lens)
    assert im.shape[:2] == (1080, 1920), "unexpected rectified see1 dimensions"
    assert "4_unforced_no_shelf_proto_lip_89_3" in ladder["calibs"], "missing unforced calibration"
    ax.imshow(im[495:745, 735:985], extent=(735, 985, 745, 495))
    poly = np.vstack([ag.SEE_GLYPH_PX[:4], ag.SEE_GLYPH_PX[0:1]])
    ax.plot(poly[:, 0], poly[:, 1], "c-", lw=2.0, label="46 mm outer ring")
    cal_aft = ladder["calibs"]["4_unforced_no_shelf_proto_lip_89_3"]
    pr_a = ag.project_see_raw(ag.scaled_world(46.0, -89.33), cal_aft["ps"], see_lens)
    ax.plot(pr_a[:4, 0], pr_a[:4, 1], "o", mfc="none", mec="lime", ms=7, mew=1.8, label=f"Unforced reproj ({ladder['models']['4_unforced_no_shelf_proto_lip_89_3']['dock_rms_px']['see1']:.2f} px)")
    ax.plot(ag.SEE_GLYPH_PX[4, 0], ag.SEE_GLYPH_PX[4, 1], "r+", ms=12, mew=2.0, label="Unoccluded See3CAM lip")
    gv = summary["glyph_verification"]["cross_check_70mm_back_wall"]
    ax.set_title(f"See3CAM Rectified Charger (Outer=69px, Wall=99.5px -> {gv['if_outer_ring_is_46mm']} vs 70mm)", fontsize=9.8)
    ax.legend(loc="lower right", fontsize=7.6, framealpha=0.88)
    ax.axis("off")


def _draw_glyph_50mp_panel(ax: Any, raw_dir: Path, summary: Dict[str, Any]) -> None:
    im = cv2.cvtColor(cv2.imread(str(raw_dir / "S-dock-main.jpg")), cv2.COLOR_BGR2RGB)
    assert im.shape[0] > 4000, "expected 50 MP S-dock-main.jpg"
    assert "s_dock_50mp_border_px" in summary["glyph_verification"], "missing border summary"
    ax.imshow(im[2420:4120, 4950:6250], extent=(4950, 6250, 4120, 2420))
    sb = summary["glyph_verification"]["s_dock_50mp_border_px"]
    ax.axhline(2679, color="cyan", lw=1.4, ls="--")
    ax.axhline(2760, color="orange", lw=1.4, ls="--")
    ax.axvline(5873, color="cyan", lw=1.4, ls="--", label="Outer white edge (46 mm)")
    ax.axvline(5808, color="orange", lw=1.4, ls="--", label="Inner sharp sq (~33-35 mm)")
    ax.set_title(f"50 MP S-dock-main: Border Top={sb['top_border']}px, Right={sb['right_border']}px", fontsize=9.8)
    ax.legend(loc="upper left", fontsize=7.8, framealpha=0.88)
    ax.axis("off")


def _draw_lip_occlusion_panel(ax: Any, raw_dir: Path, ladder: Dict[str, Any]) -> None:
    im = _load_rgb_1000(raw_dir / "Bm-15-ultra.jpg")
    assert im.shape[:2] == (750, 1000), "expected 1000x750 resized Bm-15-ultra"
    assert "4_unforced_no_shelf_proto_lip_89_3" in ladder["calibs"], "missing unforced calibration"
    ax.imshow(im[340:540, 420:670], extent=(420, 670, 540, 340))
    ax.plot(aa.PHONE_LIPS_PX[1][0], aa.PHONE_LIPS_PX[1][1], "rx", ms=12, mew=2.5, label="BEFORE phone lip pick [528,470] (on tire!)")
    cal_aft = ladder["calibs"]["4_unforced_no_shelf_proto_lip_89_3"]
    lip_pr = ag.project_pinhole(np.array([[0.0, -89.33, 0.0]]), cal_aft["pu"], (500.0, 375.0))[0]
    ax.plot(lip_pr[0], lip_pr[1], "o", mfc="none", mec="lime", ms=10, mew=2.2, label=f"AFTER unforced [0,-89.3,0] ({lip_pr[0]:.0f},{lip_pr[1]:.0f})")
    ax.set_title("Bm-15-ultra: Why Removing Occluded Phone Lip Fixes Scale & Residuals", fontsize=9.8)
    ax.legend(loc="lower right", fontsize=7.6, framealpha=0.88)
    ax.axis("off")


def _draw_shelf_ablation_bar(ax: Any, ladder: Dict[str, Any], summary: Dict[str, Any]) -> None:
    assert "with_shelf_f580" in summary["see5pt_ablation"], "missing 5pt ablation"
    s5 = summary["see5pt_ablation"]
    assert "4_unforced_no_shelf_proto_lip_89_3" in ladder["models"], "missing primary unforced model"
    labels = ["With Shelf Orthog\n(BEFORE, f=473px)", "Drop Shelf (fprior=580)\n(a reviewer 5pt, f=561px)", "Drop Shelf Unforced\n(Zero fprior, f=526px)", "Unforced 3-Cam BA\n(lip=-89.3, f=502px)"]
    vals = [s5["with_shelf_f580"]["hubs_horiz_xy_mm"]["Bm-15"], s5["no_shelf_f580"]["hubs_horiz_xy_mm"]["Bm-15"], s5["no_shelf_unforced"]["hubs_horiz_xy_mm"]["Bm-15"], ladder["models"]["4_unforced_no_shelf_proto_lip_89_3"]["hubs"]["Bm-15"]["tri_xy_mm"]]
    cols = ["#c0392b", "#e67e22", "#2980b9", "#27ae60"]
    bars = ax.bar(range(len(labels)), vals, color=cols, width=0.58)
    ax.axhline(48.0, color="#222222", ls="--", lw=1.4, label="DDL CozmoBot2.proto 48.0 mm")
    for bar, v in zip(bars, vals):
        y_txt = v - 1.15 if abs(v - 48.0) < 0.9 else v + 0.3
        col_txt = "white" if abs(v - 48.0) < 0.9 else "black"
        ax.text(bar.get_x() + bar.get_width() / 2.0, y_txt, f"{v:.2f} mm", ha="center", va="bottom", color=col_txt, fontsize=8.5, fontweight="bold")
    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, fontsize=7.6)
    ax.set_ylim(38.0, 52.0)
    ax.set_ylabel("Bm-15 Fore-Aft Hub Spacing (mm)", fontsize=9)
    ax.set_title("Effect of False SHELF Orthogonality vs Unforced 3-Cam Fit", fontsize=9.8)
    ax.legend(loc="lower right", fontsize=8)
    ax.grid(axis="y", alpha=0.25)


def render_fcc_overlay(fcc_pdf: Path, summary: Dict[str, Any], out_path: Path) -> None:
    """Render Panel 2: DDL Vector 2.0 FCC ruler Z=0 vs 3D differential-depth photogrammetry."""
    jpegs = ag.extract_fcc_jpegs(fcc_pdf)
    assert len(jpegs) == 4 and out_path.parent.is_dir(), "expected 4 FCC JPEGs and valid output dir"
    assert "fcc_ruler_photogrammetry" in summary, "missing FCC analysis section"
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 9.5), dpi=160)
    _draw_fcc_j1_panel(axes[0, 0], jpegs[1], summary)
    _draw_fcc_j3_panel(axes[0, 1], jpegs[3])
    _draw_fcc_j2_panel(axes[1, 0], jpegs[2], summary)
    _draw_fcc_curve_panel(axes[1, 1], summary)
    fig.suptitle("DDL Vector 2.0 FCC Steel-Ruler Photogrammetry: Perspective & Z-Plane Offset", fontsize=13.5, fontweight="bold")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def _draw_fcc_j1_panel(ax: Any, im1: np.ndarray, summary: Dict[str, Any]) -> None:
    rgb = cv2.cvtColor(im1, cv2.COLOR_BGR2RGB)
    assert rgb.shape[:2] == (900, 1200), "expected 1200x900 FCC JPEG 1"
    assert "j1_inverted" in summary["fcc_ruler_photogrammetry"], "missing j1 metrics"
    ax.imshow(rgb)
    ax.plot([360.5, 799.0], [414, 414], "r-", lw=2.2, label="Naive @ Z=0 (4.45 px/mm): 98.5 mm tot")
    ax.plot([360.5, 360.5], [220, 608], "c--", lw=1.6)
    ax.plot([799.0, 799.0], [220, 608], "c--", lw=1.6, label="Outer W_img=389.1px @ Z_flat=66mm")
    j1 = summary["fcc_ruler_photogrammetry"]["j1_inverted"]
    ax.set_title(f"FCC JPEG 1 Inverted: Z=0 Ruler 4.45 px/mm -> Depth-Corrected 43.7-47.6 mm", fontsize=9.6)
    ax.legend(loc="upper right", fontsize=7.6, framealpha=0.88)
    ax.axis("off")


def _draw_fcc_j3_panel(ax: Any, im3: np.ndarray) -> None:
    rgb = cv2.cvtColor(im3, cv2.COLOR_BGR2RGB)
    assert rgb.shape[:2] == (900, 1200), "expected 1200x900 FCC JPEG 3"
    ax.imshow(rgb[140:640, 260:860], extent=(260, 860, 640, 140))
    ax.plot(362.0, 285.5, "go", ms=9, mfc="none", mew=2.2, label="Front hub center (Z_eq = Z_flat-14.5)")
    ax.plot(514.0, 531.0, "o", mec="orange", ms=9, mfc="none", mew=2.2, label="Rear hub center (Z_eq = Z_flat-10.75)")
    ax.plot([362.0, 514.0], [285.5, 531.0], "y--", lw=1.8, label="288.8 px tilted side-plane chord")
    ax.set_title("FCC JPEG 3 Oblique Side View: Fore/Aft Wheel Equators Deeper Than Belly", fontsize=9.6)
    ax.legend(loc="lower left", fontsize=7.6, framealpha=0.88)
    ax.axis("off")


def _draw_fcc_j2_panel(ax: Any, im2: np.ndarray, summary: Dict[str, Any]) -> None:
    rgb = cv2.cvtColor(im2, cv2.COLOR_BGR2RGB)
    assert rgb.shape[:2] == (900, 1200), "expected 1200x900 FCC JPEG 2"
    ax.imshow(rgb)
    j2 = summary["fcc_ruler_photogrammetry"]["j2_upright_z0"]
    ax.plot([555.0, 815.0], [795.0, 650.0], "lime", lw=2.2, label=f"Z=0 plow span = {j2['plow_span_z0_mm']:.2f} mm")
    ax.set_title(f"FCC JPEG 2 Upright Homography on Z=0 Mat (RMS {j2['homography_rms_px']:.2f} px)", fontsize=9.6)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.88)
    ax.axis("off")


def _draw_fcc_curve_panel(ax: Any, summary: Dict[str, Any]) -> None:
    sw = summary["fcc_ruler_photogrammetry"]["j1_inverted"]["depth_corrected_sweep"]
    assert len(sw) == 3, "expected 3 W_outer sweep points"
    ws = [r["assumed_w_outer_mm"] for r in sw]
    assert all(w > 40.0 for w in ws), "invalid assumed W_outer width"
    flat_v = [r["flat_plane_ax_mm"] for r in sw]
    diff_v = [r["diff_depth_ax_mm"] for r in sw]
    ax.plot(ws, flat_v, "s--", color="#d35400", lw=2.0, label="AFTER naive single-Z_flat scale")
    ax.plot(ws, diff_v, "o-", color="#27ae60", lw=2.4, label="AFTER 3D differential-depth (Z_flat vs Z_eq)")
    ax.axhline(48.0, color="#222222", ls="--", lw=1.5, label="DDL CozmoBot2.proto 48.0 mm")
    ax.axhspan(46.08, 47.73, color="#27ae60", alpha=0.14, label="Unit 3 unforced 3-cam range (46.1-47.7 mm)")
    for w, fv, dv in zip(ws, flat_v, diff_v):
        ax.text(w, dv + 0.45, f"{dv:.1f}", ha="center", fontsize=8.5, fontweight="bold", color="#1e8449")
        ax.text(w, fv - 1.1, f"{fv:.1f}", ha="center", fontsize=8.2, color="#d35400")
    ax.set_xlabel("Physical Outer Tread Width W_outer (mm, bounded <64 mm charger)", fontsize=9)
    ax.set_ylabel("Implied Fore-Aft Hub Spacing (mm)", fontsize=9)
    ax.set_ylim(34.0, 52.0)
    ax.set_title("FCC JPEG 1: Single-Z Underestimate vs 3D Cylinder Horizon Recovery", fontsize=9.6)
    ax.legend(loc="lower right", fontsize=7.6)
    ax.grid(alpha=0.28)


def render_unit3_hubs_overlay(
    raw_dir: Path, obs: Dict[str, Any], see_lens: np.ndarray, ladder: Dict[str, Any], summary: Dict[str, Any], out_path: Path
) -> None:
    """Render Panel 3: Per-camera Bm-15/Bp0/Am-15 hub projections, 50 MP ellipses & unforced waterfall."""
    assert out_path.parent.is_dir() and "models" in ladder, "invalid unit 3 hub overlay inputs"
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 9.5), dpi=160)
    caps = summary["uncertainty"]["purple_hubcap_50mp"]["lip_89_33"]
    m4 = ladder["models"]["4_unforced_no_shelf_proto_lip_89_3"]["hubs"]
    bm_h = m4["Bm-15"]["per_cam_horiz_z"]["ultra"]["d_xy_mm"]
    _draw_cam_crop_hubs(axes[0, 0], _load_rgb_1000(raw_dir / "Bm-15-main.jpg"), 0, "Bm-15", obs, ladder, [180, 360, 460, 620], f"Bm-15 main (50 MP caps: R_f={caps['front_r_mm']}mm, d3d={caps['d_3d_mm']}mm)")
    _draw_cam_crop_hubs(axes[0, 1], _load_rgb_1000(raw_dir / "Bm-15-ultra.jpg"), 1, "Bm-15", obs, ladder, [300, 440, 420, 560], f"Bm-15 ultra (grazing horiz-Z {bm_h} vs unforced {m4['Bm-15']['tri_xy_mm']} mm)")
    _draw_cam_crop_hubs(axes[1, 0], _rect_see_crop(raw_dir / "Am-15-see1.jpg", see_lens), 2, "Am-15", obs, ladder, [215, 485, 415, 575], f"Held-out Am-15 see1 (Spot-B Frozen Unforced = {m4['Am-15']['tri_xy_mm']} mm)")
    _draw_waterfall_panel(axes[1, 1], obs, ladder, summary)
    fig.suptitle("Unit 3 Multi-Camera Hub Projections & Strictly Unforced Waterfall (Zero 48 mm Prior)", fontsize=13.5, fontweight="bold")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def _draw_cam_crop_hubs(
    ax: Any, img_rgb: np.ndarray, cam_idx: int, pose_label: str, obs: Dict[str, Any], ladder: Dict[str, Any], box: List[int], title: str
) -> None:
    x0, x1, y0, y1 = box
    assert img_rgb.ndim == 3 and x1 > x0 and y1 > y0, "invalid crop box"
    ax.imshow(img_rgb[y0:y1, x0:x1], extent=(x0, x1, y1, y0))
    uv_obs = np.asarray(aa.HUB_CROPS_REL[pose_label][cam_idx], float) + aa.CROP_OFFSETS[aa.CAMS[cam_idx]]
    ax.plot(uv_obs[:, 0], uv_obs[:, 1], "c+", ms=12, mew=2.2, label="Picked hub center")
    for cal, sty, lbl in [
        (ladder["baseline_calib"], "ro", "BEFORE (survey.json)"),
        (ladder["calibs"]["4_unforced_no_shelf_proto_lip_89_3"], "gs", "AFTER unforced (no 48mm prior)"),
    ]:
        used = [i for i in range(3) if aa.HUB_CROPS_REL[pose_label][i] is not None]
        pts = [ag.triangulate_rays([aa._cam_ray(i, (np.asarray(aa.HUB_CROPS_REL[pose_label][i], float) + aa.CROP_OFFSETS[aa.CAMS[i]])[j], cal["pm"], cal["pu"], cal["ps"], cal["see_lens"]) for i in used]) for j in (0, 1)]
        pr = np.array([aa._cam_proj(cam_idx, p, cal["pm"], cal["pu"], cal["ps"], cal["see_lens"]) for p in pts])
        ax.plot(pr[:, 0], pr[:, 1], sty, mfc="none", ms=9, mew=2.0, label=lbl)
    ax.set_title(title, fontsize=9.3)
    ax.legend(loc="lower right", fontsize=7.2, facecolor="#1a1d24", labelcolor="white", framealpha=0.88)
    ax.axis("off")


def _draw_waterfall_panel(ax: Any, obs: Dict[str, Any], ladder: Dict[str, Any], summary: Dict[str, Any]) -> None:
    keys = ["1_free_3d_hubs_only", "2_drop_occluded_phone_lip_94", "3_unforced_no_shelf_lip_94", "4_unforced_no_shelf_proto_lip_89_3"]
    names = ["0. BEFORE\n(survey.json)", "1. Free 3D\nhubs only", "2. Drop phone\nocl. lip", "3. Unforced\nno shelf -94", "4. Unforced\nlip -89.3"]
    b0 = summary["baseline_survey_json"]["hubs"]
    bm_v = [b0["Bm-15"]["tri_xy_mm"]] + [ladder["models"][k]["hubs"]["Bm-15"]["tri_xy_mm"] for k in keys]
    am_v = [b0["Am-15"]["tri_xy_mm"]] + [ladder["models"][k]["hubs"]["Am-15"]["tri_xy_mm"] for k in keys]
    assert len(bm_v) == 5 and len(am_v) == 5, "waterfall length mismatch"
    assert "uncertainty" in summary and summary["uncertainty"]["hub_bm15_1sigma_mm"] > 0.0, "missing uncertainty"
    x = np.arange(len(names))
    ax.bar(x - 0.18, bm_v, width=0.35, color="#3498db", label="Fit Spot B (Bm-15)")
    ax.bar(x + 0.18, am_v, width=0.35, color="#2ecc71", label="Held-out Spot A (Am-15)")
    sig = summary["uncertainty"]["hub_bm15_1sigma_mm"]
    ax.errorbar([4.18], [am_v[-1]], yerr=[[sig], [sig]], fmt="none", ecolor="black", elinewidth=2, capsize=5, label=f"1-sigma = +/-{sig} mm")
    ax.axhline(48.0, color="#222222", ls="--", lw=1.4, label="DDL 48.0 mm")
    for idx, (bv, av) in enumerate(zip(bm_v, am_v)):
        yb = bv - 1.0 if abs(bv - 48.0) < 0.9 else bv + 0.2
        cb = "white" if abs(bv - 48.0) < 0.9 else "black"
        ax.text(idx - 0.18, yb, f"{bv:.1f}", ha="center", color=cb, fontsize=7.5, fontweight="bold")
        ya = av + sig + 0.25 if idx == 4 else (av - 1.0 if abs(av - 48.0) < 0.9 else av + 0.2)
        ca = "black" if (idx == 4 or abs(av - 48.0) >= 0.9) else "white"
        ax.text(idx + 0.18, ya, f"{av:.1f}", ha="center", color=ca, fontsize=7.5, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=7.6)
    ax.set_ylim(38.0, 52.2)
    ax.set_ylabel("Triangulated Fore-Aft Hub Distance (mm)", fontsize=8.8)
    ax.set_title("Unforced Explanation of ~6 mm Gap (No 48 mm Forced Target)", fontsize=9.6)
    ax.legend(loc="lower right", fontsize=7.2)
    ax.grid(axis="y", alpha=0.25)


def render_mic_overlay(
    raw_dir: Path, obs: Dict[str, Any], see_lens: np.ndarray, ladder: Dict[str, Any], summary: Dict[str, Any], out_path: Path
) -> None:
    """Render Panel 4: Frozen Spot-B calibration evaluated on Bm-15 & held-out S-Am-15 mic ports."""
    assert out_path.parent.is_dir() and "survey" in obs, "invalid mic overlay inputs"
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 9.5), dpi=160)
    m0_rms = summary["baseline_survey_json"]["mics"]["S-Am-15"]["rms_px"]
    m4_rms = ladder["models"]["4_unforced_no_shelf_proto_lip_89_3"]["mics"]["S-Am-15"]["rms_px"]
    _draw_mic_view(axes[0, 0], _load_rgb_1000(raw_dir / "S-Am-15-main.jpg"), 0, "S-Am-15", obs, ladder, [270, 420, 270, 400], "Held-out S-Am-15 main (BEFORE vs Unforced AFTER)")
    _draw_mic_view(axes[0, 1], _load_rgb_1000(raw_dir / "S-Am-15-ultra.jpg"), 1, "S-Am-15", obs, ladder, [350, 460, 325, 425], "Held-out S-Am-15 ultra (Unforced AFTER on target)")
    _draw_mic_view(axes[1, 0], _rect_see_crop(raw_dir / "S-Am-15-see1.jpg", see_lens), 2, "S-Am-15", obs, ladder, [425, 595, 280, 390], f"Held-out S-Am-15 see1 (3-cam RMS {m0_rms} -> {m4_rms} px)")
    _draw_mic_summary_chart(axes[1, 1], ladder, summary)
    fig.suptitle("Frozen Spot-B Calibration Tested on Held-Out Spot-A Backpack Microphone Ports", fontsize=13.5, fontweight="bold")
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def _draw_mic_view(
    ax: Any, img_rgb: np.ndarray, cam_idx: int, label: str, obs: Dict[str, Any], ladder: Dict[str, Any], box: List[int], title: str
) -> None:
    x0, x1, y0, y1 = box
    assert img_rgb.ndim == 3 and x1 > x0 and y1 > y0, "invalid mic crop box"
    ax.imshow(img_rgb[y0:y1, x0:x1], extent=(x0, x1, y1, y0))
    picks = np.asarray(obs["survey"]["microphone_ports"][label][cam_idx], float)
    ax.plot(picks[:, 0], picks[:, 1], "c+", ms=12, mew=2.2, label="Picked mic ports (0, 1)")
    for cal, sty, lbl in [
        (ladder["baseline_calib"], "ro", "BEFORE (survey.json)"),
        (ladder["calibs"]["4_unforced_no_shelf_proto_lip_89_3"], "gs", "AFTER (frozen-B unforced)"),
    ]:
        raw_obs = obs["survey"]["microphone_ports"][label]
        pts = [ag.triangulate_rays([aa._cam_ray(i, raw_obs[i][j], cal["pm"], cal["pu"], cal["ps"], cal["see_lens"]) for i in range(3) if raw_obs[i] is not None]) for j in (0, 1)]
        pr = np.array([aa._cam_proj(cam_idx, p, cal["pm"], cal["pu"], cal["ps"], cal["see_lens"]) for p in pts])
        ax.plot(pr[:, 0], pr[:, 1], sty, mfc="none", ms=9, mew=2.0, label=lbl)
    ax.set_title(title, fontsize=9.3)
    ax.legend(loc="lower right", fontsize=7.2, facecolor="#1a1d24", labelcolor="white", framealpha=0.88)
    ax.axis("off")


def _draw_mic_summary_chart(ax: Any, ladder: Dict[str, Any], summary: Dict[str, Any]) -> None:
    poses = ("Bm-15", "S-Am-15", "Am-15")
    names = ("Spot B Fit\n(Bm-15 3-cam)", "Held-out A1\n(S-Am-15 3-cam)", "Held-out A2\n(Am-15 2-cam)")
    m0 = summary["baseline_survey_json"]["mics"]
    m3 = ladder["models"]["3_unforced_no_shelf_lip_94"]["mics"]
    m4 = ladder["models"]["4_unforced_no_shelf_proto_lip_89_3"]["mics"]
    sig = summary["uncertainty"]["mic_sam15_1sigma_mm"]
    assert all(p in m4 for p in poses), "missing mic pose entry"
    assert sig > 0.0 and all(p in m0 for p in poses), "invalid baseline mic metrics"
    x = np.arange(3)
    b0 = [m0[p]["dist_mm"] for p in poses]
    b3 = [m3[p]["dist_mm"] for p in poses]
    b4 = [m4[p]["dist_mm"] for p in poses]
    ax.bar(x - 0.25, b0, width=0.24, color="#c0392b", label="BEFORE (survey.json: 19.22 / 18.68 mm)")
    ax.bar(x, b3, width=0.24, color="#2980b9", label="Unforced lip=-94 (20.90 / 20.39 mm)")
    ax.bar(x + 0.25, b4, width=0.24, color="#27ae60", yerr=sig, capsize=4, label=f"Unforced lip=-89.3 (+/-{sig} mm)")
    for idx in range(3):
        ax.text(idx - 0.25, b0[idx] + 0.2, f"{b0[idx]:.2f}", ha="center", fontsize=7.4)
        ax.text(idx, b3[idx] + 0.2, f"{b3[idx]:.2f}", ha="center", fontsize=7.4)
        ax.text(idx + 0.25, b4[idx] + sig + 0.25, f"{b4[idx]:.2f}", ha="center", fontsize=7.4, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(names, fontsize=8.0)
    ax.set_ylim(16.0, 23.0)
    ax.set_ylabel("3D Mic-Port Chord Separation (mm)", fontsize=8.8)
    m0_rms = m0["Bm-15"]["rms_px"]
    m4_rms = m4["Bm-15"]["rms_px"]
    ax.set_title(f"Frozen-B Mic Chord & Bm-15 RMS Drop ({m0_rms} -> {m4_rms} px)", fontsize=9.4)
    ax.legend(loc="lower right", fontsize=7.2)
    ax.grid(axis="y", alpha=0.25)


def render_all(
    raw_dir: Path, fcc_pdf: Path, obs: Dict[str, Any], see_lens: np.ndarray, ladder: Dict[str, Any], summary: Dict[str, Any], out_dir: Path
) -> None:
    """Generate all four diagnostic PNG overlays in out_dir."""
    assert raw_dir.is_dir() and fcc_pdf.is_file(), "missing input raw_dir or fcc_pdf"
    out_dir.mkdir(parents=True, exist_ok=True)
    render_glyph_overlay(raw_dir, obs, see_lens, ladder, summary, out_dir / "glyph-scale-before-after.png")
    render_fcc_overlay(fcc_pdf, summary, out_dir / "fcc-ruler-hubs-before-after.png")
    render_unit3_hubs_overlay(raw_dir, obs, see_lens, ladder, summary, out_dir / "unit3-hubs-before-after.png")
    render_mic_overlay(raw_dir, obs, see_lens, ladder, summary, out_dir / "heldout-a-mic-ports-before-after.png")
    assert (out_dir / "heldout-a-mic-ports-before-after.png").is_file(), "overlay output missing"
