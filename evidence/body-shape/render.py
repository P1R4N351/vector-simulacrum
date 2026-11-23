"""Side, top and front views of the rig-shaped robot at rest: collision pieces (colour per part, filled
silhouettes) under the rig's own surface (grey points). python3 evidence/body-shape/render.py BODY_DIR OUT.png"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simulacrum import body_mesh  # noqa: E402

COLOURS = {"chassis": (70, 150, 90), "head": (80, 110, 200), "lift_upper": (230, 140, 30),
           "lift_lower": (190, 100, 20), "fork": (210, 60, 60)}
SCALE, PAD = 4.0, 16


def main(body_dir, out_png):
    body = body_mesh.load(Path(body_dir))
    verts_cm, groups = body_mesh.read_obj(Path(body["dir"]) / body_mesh.RIG_FILE)
    refs = body_mesh.reference_surfaces(body_mesh.to_robot(verts_cm, body["registration"]), groups)
    rng = np.random.default_rng(0)
    rig = np.concatenate([body_mesh.surface_samples(*refs[p], 0.8, rng) for p in body_mesh.PARTS])
    pieces = []
    for part in body_mesh.PARTS:
        for item in body["parts"][part]["collision"]:
            pv, _ = body_mesh.read_obj(Path(body["dir"]) / item["file"])
            pieces.append((part, body_mesh.to_robot_frame(pv, body["frames"][part])))
    views = (("side: x forward to the right, z up", 0, 2, 1), ("top: x forward to the right, y left up", 0, 1, 1),
             ("front: robot's left on the right, z up", 1, 2, 1))
    panels = []
    for title, a, b, _ in views:
        lo, hi = rig[:, [a, b]].min(0) - 3, rig[:, [a, b]].max(0) + 3
        w, h = int((hi[0] - lo[0]) * SCALE) + 2 * PAD, int((hi[1] - lo[1]) * SCALE) + 2 * PAD + 14
        im = Image.new("RGB", (w, h), "white")
        draw = ImageDraw.Draw(im, "RGBA")
        px = lambda u, v: (PAD + (u - lo[0]) * SCALE, h - PAD - (v - lo[1]) * SCALE)
        for part, pv in pieces:
            q = pv[:, [a, b]]
            if np.ptp(q[:, 0]) < 1e-6 or np.ptp(q[:, 1]) < 1e-6:
                continue
            ring = q[ConvexHull(q).vertices]
            draw.polygon([px(u, v) for u, v in ring], fill=COLOURS[part] + (70,), outline=COLOURS[part] + (200,))
        for p in rig[:: max(1, len(rig) // 25000)]:
            draw.point(px(p[a], p[b]), fill=(60, 60, 60, 140))
        draw.text((4, 2), title, fill="black")
        panels.append(im)
    width = sum(p.width for p in panels) + 12 * (len(panels) - 1)
    sheet = Image.new("RGB", (width, max(p.height for p in panels)), "white")
    x = 0
    for p in panels:
        sheet.paste(p, (x, 0))
        x += p.width + 12
    sheet.save(out_png)


if __name__ == "__main__":
    assert len(sys.argv) == 3, __doc__
    main(sys.argv[1], sys.argv[2])
