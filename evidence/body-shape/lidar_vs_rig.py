"""Unit 3's iPhone LiDAR point clouds against the rig's surface, both in robot mm.

Run from the repository root: python3 evidence/body-shape/lidar_vs_rig.py RIG.obj
Prints, per scan and per region, the distance from each scan point to the nearest rig surface
sample: as registered by the scan's own fit, and after a translation-only nearest-point
alignment (at most 30 rounds), which separates shape error from the scan's pose error.
Regions leave out what unit 3 does not share with a stock robot: its printed lift and hubcaps.
"""
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from simulacrum import body_mesh  # noqa: E402

HERE = Path(__file__).resolve().parent
SCANS = ("level-scan.ply", "liftup-scan.ply", "headup-scan.ply")
ALIGN_ROUNDS = 30
INLIER_MM = 8.0


def core(p):
    """Body and head only: inside the treads and behind the fork."""
    return (np.abs(p[:, 1]) < 17.0) & (p[:, 0] < 12.0)


# (name, predicate on scan points) - all in robot mm at the rig's pose.
REGIONS = (
    ("all points", lambda p: np.ones(len(p), bool)),
    ("body and head, |y| < 17, x < 12 mm (no treads, hubcaps, lift)", lambda p: core(p)),
    ("rear body, x < -30 mm, |y| < 17 mm", lambda p: (p[:, 0] < -30.0) & (np.abs(p[:, 1]) < 17.0)),
    ("head top, z > 55 mm, x > -35 mm", lambda p: (p[:, 2] > 55.0) & (p[:, 0] > -35.0)),
)


def read_ply(path):
    raw = path.read_bytes()
    head, body = raw.split(b"end_header\n", 1)
    count = int([l for l in head.decode().splitlines() if l.startswith("element vertex")][0].split()[2])
    rec = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1"), ("s", "u1")])
    arr = np.frombuffer(body[: count * rec.itemsize], dtype=rec)
    return np.column_stack((arr["x"], arr["y"], arr["z"])).astype(float)


def rig_surface(rig_path):
    verts_cm, groups = body_mesh.read_obj(rig_path)
    reg = body_mesh.register(verts_cm, groups)
    v_mm = body_mesh.to_robot(verts_cm, reg)
    keep = [(n, t) for n, t in groups if body_mesh.piece_of(n) in ("body", "tread", "wheel", "head", "face")]
    tris = np.concatenate([t for _, t in keep])
    return body_mesh.surface_samples(v_mm, tris, 0.5, np.random.default_rng(0))


def align(points, tree):
    shift = np.zeros(3)
    for _ in range(ALIGN_ROUNDS):
        d, i = tree.query(points + shift)
        inl = d < INLIER_MM
        if inl.sum() < 50:
            break
        step = (tree.data[i[inl]] - (points[inl] + shift)).mean(0)
        shift += step
        if np.linalg.norm(step) < 0.01:
            break
    return shift


def box_surface(centre, size, spacing=0.7):
    """Points on the faces of an axis-aligned box (mm)."""
    c, h = np.asarray(centre, float), 0.5 * np.asarray(size, float)
    out = []
    for axis in range(3):
        u, w = [a for a in range(3) if a != axis]
        gu = np.arange(-h[u], h[u] + 1e-9, spacing)
        gw = np.arange(-h[w], h[w] + 1e-9, spacing)
        U, W = np.meshgrid(gu, gw)
        for sign in (-1.0, 1.0):
            q = np.zeros((U.size, 3))
            q[:, u], q[:, w], q[:, axis] = U.ravel(), W.ravel(), sign * h[axis]
            out.append(q + c)
    return np.concatenate(out)


def cylinder_surface(x, y, z, r, width, spacing=0.7):
    """Points on a cylinder with its axis along y (mm), caps included."""
    ang = np.arange(0.0, 2 * np.pi, spacing / r)
    ys = np.arange(-0.5 * width, 0.5 * width + 1e-9, spacing)
    A, Y = np.meshgrid(ang, ys)
    side = np.column_stack((x + r * np.cos(A.ravel()), y + Y.ravel(), z + r * np.sin(A.ravel())))
    rr = np.arange(0.0, r, spacing)
    R, B = np.meshgrid(rr, ang)
    caps = [np.column_stack((x + R.ravel() * np.cos(B.ravel()), np.full(R.size, y + s * 0.5 * width),
                             z + R.ravel() * np.sin(B.ravel()))) for s in (-1, 1)]
    return np.concatenate([side] + caps)


def parametric_surface(v):
    """The simulacrum's own primitive body (mjcf.py), lift left out like the scan regions do."""
    from simulacrum import mjcf

    height = v["chassis_top_z_mm"] - v["chassis_ground_clearance_mm"]
    parts = [box_surface((v["chassis_center_x_mm"], 0.0, v["chassis_ground_clearance_mm"] + 0.5 * height),
                         (v["chassis_length_mm"], v["chassis_width_mm"], height)),
             box_surface(v["neck_joint_position_mm"], v["head_box_mm"])]
    half = 0.5 * v["tread_spacing_mm"]
    front, rear = mjcf.wheel_x_mm(v)
    for x, r in ((front, v["wheel_front_radius_mm"]), (rear, v["wheel_rear_radius_mm"])):
        for y in (half, -half):
            parts.append(cylinder_surface(x, y, r, r, v["tread_width_mm"]))
    return np.concatenate(parts)


def score(label, surface, scans):
    """Median and p90 distance of each scan's core points to a surface, as registered (no alignment)."""
    tree = cKDTree(surface)
    cells = []
    for name, pts in scans:
        d = tree.query(pts[core(pts)])[0]
        cells.append(f"{name.split('-')[0]} {np.percentile(d, 50):.1f}/{np.percentile(d, 90):.1f}")
    print(f"   {label:44s} " + "   ".join(cells))


def main(rig_path):
    from simulacrum import params

    rig = rig_surface(Path(rig_path))
    tree = cKDTree(rig)
    scan_dir = HERE.parent / "iphone-lidar-orbits"
    scans = [(scan, read_ply(scan_dir / scan)) for scan in SCANS]
    for scan, pts in scans:
        core_sel = core(pts)
        shift = align(pts[core_sel], tree)
        print(f"{scan}: {len(pts)} points; translation-only alignment of the core region: "
              f"{np.round(shift, 2).tolist()} mm")
        for name, pred in REGIONS:
            sel = pred(pts)
            if sel.sum() < 20:
                continue
            raw = tree.query(pts[sel])[0]
            moved = tree.query(pts[sel] + shift)[0]
            q = lambda d: "/".join(f"{np.percentile(d, p):.1f}" for p in (50, 90))
            print(f"   {name:55s} n={sel.sum():5d}  median/p90 as registered {q(raw)} mm, aligned {q(moved)} mm")
    print("Controls, core region (body and head), median/p90 mm as registered:")
    score("rig (Anki's shape)", rig, scans)
    score("rig scaled x1.10 about the front axle", rig * 1.10, scans)
    score("rig scaled x0.90 about the front axle", rig * 0.90, scans)
    score("rig moved 5 mm back", rig - [5.0, 0.0, 0.0], scans)
    score("simulacrum's primitive body (mjcf.py)", parametric_surface(params.values(params.load())), scans)


if __name__ == "__main__":
    assert len(sys.argv) == 2, __doc__
    main(sys.argv[1])
