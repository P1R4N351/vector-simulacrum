"""Measure the hooks on the stock fork's two uprights from the animation rig.

    python evidence/cube-pickup/measure_hook.py [--rig-cache DIR] [--out FILE]

Each upright of the stock fork carries a barbed hook that sticks out ahead of the upright's front
face. Its upper surface dips into a valley just ahead of the upright and rises again to the tip, so
the cube's thin front lip over each edge notch sits in the valley once the lift takes the cube, and
the cube cannot slide off forward without climbing over the tip. This script cuts the fork out of the
pinned rig (simulacrum/body_mesh.py, the same registration and frames the mesh body uses), samples
its surface, and reports, per side, in the fork's frame (mm, origin at the wrist, x forward, z up,
the fork level):

- the upright's front face, measured where the upright carries no hook;
- the hook's lateral span;
- the upper and lower surface of the hook in 0.1 mm columns, from the upright to the tip;
- the hook as slices SLICE_MM wide, each a box from the highest lower surface to the lowest upper
  surface over its width, so every box lies inside the measured hook and the valley stays open;
- the upright behind the valley as one box, from UPRIGHT_BACK_MM to the first slice and from the
  window's floor (the upright runs on down into the crossbar) to the lowest point of its upper surface.

The two sides are measured separately and must agree; the values written are those both sides
contain. The output is the source for lift_fork_hook_* in parameters.json.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import numpy as np  # noqa: E402

from simulacrum import body_mesh, params  # noqa: E402

SAMPLE_SPACING_MM = 0.1
SIDE_MIN_Y_MM = 10.0          # beyond this |y| a point belongs to an upright, not the crossbar
UPRIGHT_BAND_Z_MM = (-20.0, -17.0)  # an upright's front where it carries no hook
HOOK_CLEAR_MM = 0.4           # a hook point stands this far ahead of the upright's front
HOOK_CORE_MM = 1.0            # the lateral span is read this far ahead, clear of the root's fillet
WINDOW_Z_MM = (-20.0, 4.0)    # the upright and hook, above the crossbar
COLUMN_MM = 0.1
SLICE_MM = 0.5
UPRIGHT_BACK_MM = 1.0         # the upright box's back face; the real upright runs back to x -3.2
SIDE_AGREE_MM = 0.3
DEFAULT_CACHE = Path.home() / ".cache" / "vector-simulacrum-rig"


def fork_samples(rig: Path, v) -> np.ndarray:
    """Surface points of the fork, in the fork's own frame (mm)."""
    verts_cm, groups = body_mesh.read_obj(rig)
    reg = body_mesh.register(verts_cm, groups)
    v_mm = body_mesh.to_robot(verts_cm, reg)
    feats = body_mesh.measure(v_mm, groups)
    drift = body_mesh.check_against_table(v, feats)
    assert not drift, "rig disagrees with the parameter table: " + "; ".join(drift)
    parts = body_mesh.split_parts(v_mm, groups, body_mesh.part_frames(v, feats))
    entries = parts["fork"]["visual"]
    assert entries, "the rig has no fork"
    verts, tris = body_mesh.merge(entries)
    return body_mesh.surface_samples(verts, tris, SAMPLE_SPACING_MM, np.random.default_rng(0))


def side(points: np.ndarray, sign: float) -> dict:
    """One upright's front, hook span and surface columns, with y mirrored so both sides read positive."""
    p = points[sign * points[:, 1] > SIDE_MIN_Y_MM].copy()
    p[:, 1] *= sign
    assert len(p) > 1000, "too few points on this upright"
    band = p[(p[:, 2] >= UPRIGHT_BAND_Z_MM[0]) & (p[:, 2] <= UPRIGHT_BAND_Z_MM[1])]
    assert len(band) > 50, "no points where the upright has no hook"
    front = float(band[:, 0].max())
    hook = p[p[:, 0] > front + HOOK_CLEAR_MM]
    assert len(hook) > 100, "no hook ahead of the upright"
    core = hook[hook[:, 0] > front + HOOK_CORE_MM]
    y_in, y_out = float(core[:, 1].min()), float(core[:, 1].max())
    span = p[(p[:, 1] >= y_in) & (p[:, 1] <= y_out) & (p[:, 2] > WINDOW_Z_MM[0]) & (p[:, 2] < WINDOW_Z_MM[1])]
    tip = float(span[:, 0].max())
    columns = []
    for x in np.arange(UPRIGHT_BACK_MM, tip, COLUMN_MM):
        cell = span[(span[:, 0] >= x) & (span[:, 0] < x + COLUMN_MM)]
        if len(cell):
            columns.append([round(float(x), 2), round(float(cell[:, 2].min()), 2), round(float(cell[:, 2].max()), 2)])
    return {"upright_front_x_mm": front, "hook_y_mm": [y_in, y_out], "hook_tip_x_mm": tip, "columns": columns}


def _inscribed(columns, x0: float, x1: float):
    """(bottom, top) of the box inside every column of [x0, x1): the highest bottom and the lowest top."""
    inside = [(b, t) for x, b, t in columns if x0 - 1e-9 <= x < x1 - 1e-9]
    assert inside, f"no surface columns in [{x0}, {x1})"
    return max(b for b, _ in inside), min(t for _, t in inside)


def common(left: dict, right: dict) -> dict:
    """What both sides contain: the span rounded inward to 0.1 mm, slices from the upright's front
    (rounded down to SLICE_MM) to the last whole slice before the tip, and the upright box."""
    for key in ("upright_front_x_mm", "hook_tip_x_mm"):
        assert abs(left[key] - right[key]) < SIDE_AGREE_MM, f"the two sides disagree on {key}"
    y = [math.ceil(max(left["hook_y_mm"][0], right["hook_y_mm"][0]) * 10) / 10,
         math.floor(min(left["hook_y_mm"][1], right["hook_y_mm"][1]) * 10) / 10]
    start = math.floor(min(left["upright_front_x_mm"], right["upright_front_x_mm"]) / SLICE_MM) * SLICE_MM
    tip = min(left["hook_tip_x_mm"], right["hook_tip_x_mm"])
    both = left["columns"] + right["columns"]
    slices = []
    x = start
    while x + SLICE_MM <= tip + 1e-9 and len(slices) < 64:
        bottom, top = _inscribed(both, x, x + SLICE_MM)
        assert bottom < top, f"the hook is empty over [{x}, {x + SLICE_MM})"
        slices.append([round(x, 2), round(x + SLICE_MM, 2), math.ceil(bottom * 10) / 10, math.floor(top * 10) / 10])
        x += SLICE_MM
    assert slices, "no hook slice survives"
    # Behind its front face the upright is solid down into the crossbar, so a column there shows only
    # the upright's upper surface: its top is the lowest of those, and its bottom is the window's floor,
    # where the front face is seen to reach (UPRIGHT_BAND_Z_MM).
    _, top = _inscribed(both, UPRIGHT_BACK_MM, start)
    upright = [UPRIGHT_BACK_MM, round(start, 2), WINDOW_Z_MM[0], math.floor(top * 10) / 10]
    assert upright[2] < upright[3], "the upright box is empty"
    return {"hook_y_mm": y, "hook_slices_mm": slices, "hook_upright_mm": upright}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rig-cache", type=Path, default=DEFAULT_CACHE)
    ap.add_argument("--out", type=Path, default=Path(__file__).with_name("hook.json"))
    args = ap.parse_args(argv)
    v = params.values(params.load())
    rig = body_mesh.fetch(args.rig_cache)
    pts = fork_samples(rig, v)
    left, right = side(pts, 1.0), side(pts, -1.0)
    result = {"rig": body_mesh.RIG, "frame": "fork, mm, origin at the wrist, x forward, z up, level",
              "samples": int(len(pts)), "spacing_mm": SAMPLE_SPACING_MM, "slice_mm": SLICE_MM,
              "left": left, "right": right, **common(left, right)}
    args.out.write_text(json.dumps(result, indent=1) + "\n")
    for name, s in (("left", left), ("right", right)):
        print(f"{name}: upright front x {s['upright_front_x_mm']:.2f}, hook y {s['hook_y_mm'][0]:.2f}..{s['hook_y_mm'][1]:.2f}, "
              f"tip x {s['hook_tip_x_mm']:.2f}")
    print("common: hook y", result["hook_y_mm"], "upright [x0, x1, z_bottom, z_top]", result["hook_upright_mm"])
    print("slices [x0, x1, z_bottom, z_top]:", json.dumps(result["hook_slices_mm"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
