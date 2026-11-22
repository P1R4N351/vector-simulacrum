"""Render a 4-panel body-shape dimensional, joint, and CoACD fidelity summary without proprietary meshes.

Usage:
    python3 evidence/body-shape/plot_breakdown.py
"""

from __future__ import annotations

import csv
import math
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulacrum import mjcf, params  # noqa: E402

HERE = Path(__file__).resolve().parent
CSV_PATH = HERE / "measurements.csv"
PNG_PATH = HERE / "envelope_and_joints.png"
MAX_ROWS = 200


def load_rows(path: Path) -> Dict[Tuple[str, str], Dict[str, str]]:
    """Load structured measurement rows indexed by (section, item)."""
    assert path.is_file(), f"missing CSV: {path}"
    out: Dict[Tuple[str, str], Dict[str, str]] = {}
    with path.open(newline="") as fh:
        reader = csv.DictReader(fh)
        for idx, row in enumerate(reader):
            assert idx < MAX_ROWS, "measurement table exceeded bounded row cap"
            out[(row["section"], row["item"])] = row
    assert len(out) > 0, "measurement table is empty"
    return out


def draw_side_panel(ax: plt.Axes, v: Dict, rows: Dict[Tuple[str, str], Dict[str, str]]) -> None:
    """Plot side profile (x-z): parametric primitives vs retail/rig outer bounds and joint pivots."""
    c_len = float(v["chassis_length_mm"])
    c_cx = float(v["chassis_center_x_mm"])
    c_z0 = float(v["chassis_ground_clearance_mm"])
    c_z1 = float(v["chassis_top_z_mm"])
    ax.add_patch(patches.Rectangle((c_cx - 0.5 * c_len, c_z0), c_len, c_z1 - c_z0,
                                   fc="#b0c4de", alpha=0.45, ec="#2b5b84", lw=1.5, label="Parametric boxes/wheels"))
    nx, _, nz = [float(x) for x in v["neck_joint_position_mm"]]
    hl, _, hh = [float(x) for x in v["head_box_mm"]]
    ax.add_patch(patches.Rectangle((nx - 0.5 * hl, nz - 0.5 * hh), hl, hh, fc="#b0c4de", alpha=0.45, ec="#2b5b84", lw=1.5))
    ax.add_patch(patches.Circle((0.0, float(v["wheel_front_radius_mm"])), float(v["wheel_front_radius_mm"]),
                                fc="none", ec="#2b5b84", ls="--", lw=1.4))
    ax.add_patch(patches.Circle((-float(v["wheel_base_mm"]), float(v["wheel_rear_radius_mm"])),
                                float(v["wheel_rear_radius_mm"]), fc="none", ec="#2b5b84", ls="--", lw=1.4))
    rig_h = float(rows[("envelope", "height")]["rig_value"])
    rig_l = float(rows[("envelope", "length")]["rig_value"])
    ret_l = float(rows[("envelope", "length")]["reference_value"])
    ret_h = float(rows[("envelope", "height")]["reference_value"])
    ax.add_patch(patches.Rectangle((-66.7, 0.0), rig_l, rig_h, fc="none", ec="#d95f02", lw=2.0,
                                   label="Rig collision box (100.9 x 69.7 mm)"))
    ax.add_patch(patches.Rectangle((-66.7, 0.0), ret_l, ret_h, fc="none", ec="#1b9e77", ls=":", lw=1.8,
                                   label="Retail envelope (99.8 x 69.3 mm)"))
    pivots = [("front_axle", 0.0, float(rows[("joint", "front_axle_z")]["rig_value"])),
              ("rear_axle", float(rows[("joint", "rear_axle_x")]["rig_value"]), float(rows[("joint", "rear_axle_z")]["rig_value"])),
              ("neck", float(rows[("joint", "neck_x")]["rig_value"]), float(rows[("joint", "neck_z")]["rig_value"])),
              ("lift", float(rows[("joint", "upper_lift_pivot_x")]["rig_value"]),
               float(rows[("joint", "upper_lift_pivot_z")]["rig_value"]))]
    for name, px, pz in pivots:
        ax.plot(px, pz, "ro", ms=5)
        ax.annotate(name, (px, pz), xytext=(3, 3), textcoords="offset points", fontsize=8, color="#8b0000")
    ax.axhline(0.0, color="#333333", lw=1.0)
    ax.set(xlim=(-74, 46), ylim=(-4, 88), aspect="equal", xlabel="Robot X (mm)", ylabel="Z (mm)",
           title="Side profile: parametric (86.1x66.0) vs rig (100.9x69.7) & pivots")
    ax.legend(loc="upper left", fontsize=7.5)
    ax.grid(alpha=0.25)


def draw_top_panel(ax: plt.Axes, v: Dict, rows: Dict[Tuple[str, str], Dict[str, str]]) -> None:
    """Plot top view (x-y): parametric tread spacing/width vs rig treads, head, and fork width."""
    c_len = float(v["chassis_length_mm"])
    c_wid = float(v["chassis_width_mm"])
    c_cx = float(v["chassis_center_x_mm"])
    ax.add_patch(patches.Rectangle((c_cx - 0.5 * c_len, -0.5 * c_wid), c_len, c_wid,
                                   fc="#b0c4de", alpha=0.45, ec="#2b5b84", lw=1.5, label="Parametric chassis/treads"))
    ts_p = float(v["tread_spacing_mm"])
    tw_p = float(v["tread_width_mm"])
    fx_front, fx_rear = mjcf.wheel_x_mm(v)
    tl_x0 = fx_rear - float(v["wheel_rear_radius_mm"])
    tl_len = (fx_front + float(v["wheel_front_radius_mm"])) - tl_x0
    for sgn in (-1.0, 1.0):
        ax.add_patch(patches.Rectangle((tl_x0, sgn * 0.5 * ts_p - 0.5 * tw_p), tl_len, tw_p,
                                       fc="#b0c4de", alpha=0.5, ec="#2b5b84", lw=1.2))
        ax.add_patch(patches.Rectangle((-62.5, sgn * 23.94 - 5.75), 77.0, 11.5,
                                       fc="none", ec="#d95f02", lw=1.8, label="Rig treads (47.9 sep, 11.5 wide)" if sgn > 0 else None))
    rig_w = float(rows[("envelope", "width")]["rig_value"])
    rig_l = float(rows[("envelope", "length")]["rig_value"])
    ax.add_patch(patches.Rectangle((-66.7, -0.5 * rig_w), rig_l, rig_w,
                                   fc="none", ec="#1b9e77", ls="--", lw=1.8, label="Rig envelope (100.9 x 60.9 mm)"))
    ax.set(xlim=(-74, 46), ylim=(-42, 46), aspect="equal", xlabel="Robot X (mm)", ylabel="Y left (mm)",
           title="Top footprint: treads (46 vs 47.9) & width (56.0 vs 60.9 mm)")
    ax.legend(loc="upper left", fontsize=7.5)
    ax.grid(alpha=0.25)


def draw_dimensions_panel(ax: plt.Axes, rows: Dict[Tuple[str, str], Dict[str, str]]) -> None:
    """Plot bar comparison of key body dimensions across parametric, rig, and retail/printed references."""
    specs = [("length", "envelope", "Length"), ("width", "envelope", "Width"),
             ("height", "envelope", "Height"), ("tread_spacing_as_built", "dimension", "Tread sep"),
             ("tread_width", "printed", "Tread w"), ("fork_width", "printed", "Fork w")]
    labels, param_v, rig_v, ref_v = [], [], [], []
    for item, sec, lbl in specs:
        row = rows[(sec, item)]
        labels.append(lbl)
        param_v.append(float(row["parametric_value"]))
        rig_v.append(float(row["rig_value"]))
        ref_v.append(float(row["reference_value"]))
    idx = list(range(len(specs)))
    w = 0.26
    ax.bar([i - w for i in idx], param_v, width=w, color="#6baed6", label="Parametric")
    ax.bar(idx, rig_v, width=w, color="#fd8d3c", label="Animation rig")
    ax.bar([i + w for i in idx], ref_v, width=w, color="#74c476", label="Retail / printed ref")
    ax.set_xticks(idx)
    ax.set_xticklabels(labels, fontsize=8.5)
    ax.set_ylabel("Millimetres")
    ax.set_title("Body & printed-part dimensional agreement (mm)")
    ax.legend(fontsize=7.5)
    ax.grid(axis="y", alpha=0.25)


def draw_fidelity_panel(ax: plt.Axes, rows: Dict[Tuple[str, str], Dict[str, str]]) -> None:
    """Plot CoACD convex piece fidelity (protrusion p99 and excess p95) against acceptance limits."""
    parts = ["chassis", "head", "lift_upper", "lift_lower", "fork"]
    pieces = [int(rows[("fidelity", p)]["parametric_value"]) for p in parts]
    protr = [float(rows[("fidelity", p)]["rig_value"]) for p in parts]
    excess = [float(rows[("fidelity", p)]["reference_value"]) for p in parts]
    idx = list(range(len(parts)))
    w = 0.34
    ax.bar([i - 0.5 * w for i in idx], protr, width=w, color="#3182bd", label="Protrusion p99 (limit 1.0)")
    ax.bar([i + 0.5 * w for i in idx], excess, width=w, color="#e6550d", label="Excess p95 (limit 2.0)")
    ax.axhline(1.0, color="#3182bd", ls="--", lw=1.2)
    ax.axhline(2.0, color="#e6550d", ls="--", lw=1.2)
    ax.set_xticks(idx)
    ax.set_xticklabels([f"{p}\n({c} pcs)" for p, c in zip(parts, pieces)], fontsize=8)
    ax.set_ylim(0, 2.35)
    ax.set_ylabel("Distance error (mm)")
    ax.set_title("CoACD 64-piece whole-robot surface fidelity (all <= limits)")
    ax.legend(loc="upper left", fontsize=7.5)
    ax.grid(axis="y", alpha=0.25)


def main() -> int:
    v = params.values(params.load())
    rows = load_rows(CSV_PATH)
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.2), constrained_layout=True)
    draw_side_panel(axes[0, 0], v, rows)
    draw_top_panel(axes[0, 1], v, rows)
    draw_dimensions_panel(axes[1, 0], rows)
    draw_fidelity_panel(axes[1, 1], rows)
    fig.suptitle("Vector Real Body Shape Breakdown: Parametric vs Rig vs Retail/Printed References", fontsize=12)
    fig.savefig(PNG_PATH, dpi=150)
    assert PNG_PATH.is_file() and PNG_PATH.stat().st_size > 10_000, "diagram write failed"
    print(f"Wrote {PNG_PATH} ({PNG_PATH.stat().st_size} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
