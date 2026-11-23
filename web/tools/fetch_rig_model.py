#!/usr/bin/env python3
"""Slice Victor_rig_01.obj into joint-framed OBJ parts for the web simulacrum."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
from typing import Any, Dict, List, Optional, Sequence, Tuple
import urllib.error
import urllib.request

import numpy as np

HERE = Path(__file__).resolve().parent.parent
SDK_DEST = HERE / "public" / "robot" / "sdk"
RIG_DEST = SDK_DEST / "rig"
MODEL_JSON = HERE / "public" / "robot" / "model.json"

RIG_META = {
    "repo": "kercre123/vector-animations-raw",
    "commit": "59b198d4e6fb7394ae4c3d832dea2c4d28866ab0",
    "path": "assets/rigs/Base/obj/Victor_rig_01.obj",
    "sha256": "59890fafc1c122726d73f49fab66913d45d87426143107d9359e6ca81e7fad60",
}
RIG_URL = f"https://raw.githubusercontent.com/{RIG_META['repo']}/{RIG_META['commit']}/{RIG_META['path']}"
DDL_URL = f"https://raw.githubusercontent.com/{RIG_META['repo']}/main/LICENSE"
DDL_SHA256 = "6573b8f79ac25f8626b795b3322aa7420e87e21b0520d20ad90ebca7ba4e06f0"

MAX_OBJ_LINES = 2_000_000
CM_TO_MM = 10.0
FEATURE_TOLERANCE_MM = 1.0
DOWNLOAD_TIMEOUT_S = 240

PIECE_RULES = (
    ("LP_", None),
    ("low_res_grp", None),
    ("Eye_rig", "face"),
    ("overscan", "face"),
    ("head_grp", "head"),
    ("Upper_Hand", "arm_upper"),
    ("Lower_Hand", "arm_lower"),
    ("Front_Mid_Part", "fork"),
    ("Back_Mid_Part", "fork"),
    ("Bottom_Mid_Part", "fork"),
    ("Rubber_Chain", "tread"),
    ("Wheels", "wheel"),
    ("Tooth", "wheel"),
    ("body_grp", "body"),
    ("backpack", "body"),
)
MATERIAL_LOOK = {
    "pianoBlack_matSG": "gloss_black",
    "blinn1SG": "dark_grey",
    "blinn2SG": "gunmetal",
    "lambert5SG2": "rubber",
    "anisotropic1SG3": "screen",
    "lambert2SG3": "screen",
    "lambert2SG4": "screen",
    "pasted__lambert8SG1": "screen",
    "lambert4SG": "light_diffuser",
    "backpackMidSG": "light_diffuser",
    "backpackBttmSG": "light_diffuser",
    "lambert4SG2": "light_diffuser",
    "shadingMap1SG3": "eye",
    "pasted__eye_l_mat2SG": "eye",
    "shadingMap1SG2": "eye_glow",
    "eye_r_glowSG": "eye_glow",
}
TARGETS = (
    "chassis",
    "head_shell",
    "head_visor",
    "eye_l",
    "eye_r",
    "lift_upper",
    "lift_lower",
    "fork",
    "wheel_fl",
    "wheel_fr",
    "wheel_bl",
    "wheel_br",
)


def sha256_bytes(data: bytes) -> str:
    """Return lowercase SHA-256 hex digest for in-memory bytes."""
    assert isinstance(data, (bytes, bytearray)), "bytes required"
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    """Stream-hash a file without loading large inputs at once."""
    assert path.is_file(), f"missing file to hash: {path}"
    digest = hashlib.sha256()
    with path.open("rb") as src:
        for _ in range(1 << 20):
            block = src.read(1 << 20)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def piece_of(group: str) -> Optional[str]:
    """Classify one Victor_rig_01 OBJ group name into a semantic robot piece."""
    assert group, "empty group name"
    if group.endswith("L_Wheels L_Tooth"):
        return "tread"
    for key, piece in PIECE_RULES:
        if key in group:
            return piece
    raise ValueError(f"rig group not classified: {group!r}")


def read_obj(path: Path) -> Tuple[np.ndarray, List[Tuple[str, np.ndarray, List[str]]]]:
    """Parse vertices (n, 3) and triangulated per-group triangles + look labels."""
    verts: List[List[float]] = []
    groups: Dict[str, List[List[int]]] = {}
    looks: Dict[str, List[str]] = {}
    cur, mat = "default", None
    with path.open(errors="replace") as src:
        for count, line in enumerate(src):
            assert count < MAX_OBJ_LINES, "OBJ longer than pinned rig"
            if line.startswith("v "):
                verts.append([float(t) for t in line.split()[1:4]])
            elif line.startswith(("g ", "o ")):
                cur = line[2:].strip() or "default"
            elif line.startswith("usemtl "):
                mat = line.split(None, 1)[1].strip()
            elif line.startswith("f "):
                if mat not in MATERIAL_LOOK:
                    raise ValueError(f"rig material without look: {mat!r}")
                idx = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
                assert len(idx) >= 3 and min(idx) >= 0, "face needs 3+ indices"
                tris = [[idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1)]
                groups.setdefault(cur, []).extend(tris)
                looks.setdefault(cur, []).extend([MATERIAL_LOOK[mat]] * len(tris))
    v = np.asarray(verts, dtype=float)
    assert v.ndim == 2 and v.shape[1] == 3 and len(v) > 0, "empty rig OBJ"
    return v, [(n, np.asarray(t, dtype=np.int64), looks[n]) for n, t in groups.items()]


def circle_fit(xy: np.ndarray) -> Tuple[np.ndarray, float, float]:
    """Least-squares circle fit returning (centre, radius, rms residual)."""
    assert xy.ndim == 2 and xy.shape[1] == 2 and len(xy) >= 3, "need >=3 pts"
    a = np.column_stack((2.0 * xy[:, 0], 2.0 * xy[:, 1], np.ones(len(xy))))
    sol, *_ = np.linalg.lstsq(a, (xy ** 2).sum(1), rcond=None)
    centre = sol[:2]
    radius = float(math.sqrt(max(sol[2] + centre @ centre, 0.0)))
    resid = np.hypot(*(xy - centre).T) - radius
    return centre, radius, float(np.sqrt((resid ** 2).mean()))


def end_cap(xz: np.ndarray, front: bool, rounds: int = 8) -> Dict[str, Any]:
    """Fit a rounded lift-link end cap center iteratively in robot XZ."""
    assert xz.ndim == 2 and xz.shape[1] == 2 and len(xz) >= 12, "invalid cap points"
    edge = xz[:, 0].max() if front else xz[:, 0].min()
    sel = (xz[:, 0] > edge - 5.0) if front else (xz[:, 0] < edge + 5.0)
    centre, radius, rms = circle_fit(xz[sel])
    for _ in range(rounds):
        beyond = (xz[:, 0] >= centre[0]) if front else (xz[:, 0] <= centre[0])
        sel = beyond & (np.abs(np.hypot(*(xz - centre).T) - radius) < 0.6)
        assert sel.sum() >= 12, "end cap lost its points"
        centre, radius, rms = circle_fit(xz[sel])
    return {"centre_xz": centre.tolist(), "radius": radius, "rms": rms}


def piece_points(v: np.ndarray, groups: Sequence[Tuple[str, np.ndarray, Any]], keys: Tuple[str, ...]) -> np.ndarray:
    """Gather unique vertices across groups whose names contain any substring in keys."""
    assert len(keys) > 0, "keys required"
    ids = np.unique(np.concatenate([t.ravel() for name, t, _ in groups if any(k in name for k in keys)]))
    assert len(ids) > 0, f"no rig group matches {keys}"
    return v[ids]


def register_rig(v: np.ndarray, groups: Sequence[Tuple[str, np.ndarray, Any]]) -> Dict[str, float]:
    """Compute ground plane, front axle center, and midline symmetry in Maya cm."""
    tread = np.concatenate([v[np.unique(t)] for name, t, _ in groups if piece_of(name) == "tread"])
    ground_y = float(tread[:, 1].min())
    front = piece_points(v, groups, ("R_Large_Ri", "L_Tooth21"))
    centre, _, _ = circle_fit(front[:, [2, 1]])
    rims_r = piece_points(v, groups, ("R_Large_Ri", "R_Small_Ri"))
    rims_l = piece_points(v, groups, ("L_Tooth21", "L_Tooth20"))
    mid = 0.25 * (rims_r[:, 0].min() + rims_r[:, 0].max() + rims_l[:, 0].min() + rims_l[:, 0].max())
    return {"ground_y_cm": ground_y, "front_axle_z_cm": float(centre[0]), "centre_x_cm": float(mid)}


def to_robot(p_cm: np.ndarray, reg: Dict[str, float]) -> np.ndarray:
    """Convert Maya cm (+Z fwd, +X left, +Y up) into robot mm (+X fwd, +Y left, +Z up)."""
    assert p_cm.ndim == 2 and p_cm.shape[1] == 3, "points must be (n, 3)"
    return np.column_stack((
        (p_cm[:, 2] - reg["front_axle_z_cm"]) * CM_TO_MM,
        (p_cm[:, 0] - reg["centre_x_cm"]) * CM_TO_MM,
        (p_cm[:, 1] - reg["ground_y_cm"]) * CM_TO_MM,
    ))


def measure_rig(v_mm: np.ndarray, groups: Sequence[Tuple[str, np.ndarray, Any]]) -> Dict[str, Any]:
    """Measure joint axes, wheel centers, and tread contact widths in robot mm."""
    out: Dict[str, Any] = {}
    for label, keys in (
        ("front_rims", ("R_Large_Ri", "L_Tooth21")),
        ("rear_rims", ("R_Small_Ri", "L_Tooth20")),
        ("head_side_frames", ("R_Large_Frame", "L_Large_Frame")),
    ):
        c, r, e = circle_fit(piece_points(v_mm, groups, keys)[:, [0, 2]])
        out[label] = {"centre_xz": c.tolist(), "radius": r, "rms": e}
    for label, key in (("upper", "_Upper_Hand"), ("lower", "_Lower_Hand")):
        xz = piece_points(v_mm, groups, (key,))[:, [0, 2]]
        out[f"{label}_link_rear"] = end_cap(xz, front=False)
        out[f"{label}_link_front"] = end_cap(xz, front=True)
    tread = np.concatenate([v_mm[np.unique(t)] for n, t, _ in groups if piece_of(n) == "tread"])
    for side, sign in (("left", 1.0), ("right", -1.0)):
        band = tread[(np.sign(tread[:, 1]) == sign) & (tread[:, 2] < 1.0)]
        assert len(band) > 0, f"missing {side} tread contact band"
        out[f"tread_contact_{side}_y"] = [float(band[:, 1].min()), float(band[:, 1].max())]
    return out


def rig_lift_angle(feats: Dict[str, Any]) -> float:
    """Measure the lift linkage angle present in Victor_rig_01.obj."""
    angles: List[float] = []
    for which in ("upper", "lower"):
        rear = feats[f"{which}_link_rear"]["centre_xz"]
        front = feats[f"{which}_link_front"]["centre_xz"]
        angles.append(math.atan2(front[1] - rear[1], front[0] - rear[0]))
    assert abs(angles[0] - angles[1]) < math.radians(2.0), "non-parallel lift links"
    return 0.5 * (angles[0] + angles[1])


def check_against_table(v: Dict[str, Any], feats: Dict[str, Any]) -> None:
    """Fail closed if any rig joint drifts more than FEATURE_TOLERANCE_MM from model.json."""
    neck, base = v["neck_joint_position_mm"], v["lift_base_position_mm"]
    lu_r, lu_f = feats["lower_link_rear"]["centre_xz"], feats["lower_link_front"]["centre_xz"]
    l_len = math.hypot(lu_f[0] - lu_r[0], lu_f[1] - lu_r[1])
    pairs = (
        ("front axle height", feats["front_rims"]["centre_xz"][1], v["wheel_front_radius_mm"]),
        ("rear axle x", feats["rear_rims"]["centre_xz"][0], -v["wheel_base_mm"]),
        ("rear axle height", feats["rear_rims"]["centre_xz"][1], v["wheel_rear_radius_mm"]),
        ("neck x", feats["head_side_frames"]["centre_xz"][0], neck[0]),
        ("neck z", feats["head_side_frames"]["centre_xz"][1], neck[2]),
        ("lift pivot x", feats["upper_link_rear"]["centre_xz"][0], base[0]),
        ("lift pivot z", feats["upper_link_rear"]["centre_xz"][1], base[2]),
        ("lift link length", l_len, v["lift_arm_length_mm"]),
    )
    drift = [f"{n}: rig {g:.2f} vs table {w:.2f}" for n, g, w in pairs if abs(g - w) > FEATURE_TOLERANCE_MM]
    if drift:
        raise ValueError("rig disagrees with model.json: " + "; ".join(drift))


def compute_frames(v: Dict[str, Any], feats: Dict[str, Any]) -> Dict[str, Any]:
    """Return moving-part joint origins in robot mm and rig rest lift angle."""
    base = [float(x) for x in v["lift_base_position_mm"]]
    a0 = rig_lift_angle(feats)
    ru, rl = feats["upper_link_rear"]["centre_xz"], feats["lower_link_rear"]["centre_xz"]
    lower = [base[0] + rl[0] - ru[0], base[1], base[2] + rl[1] - ru[1]]
    arm = float(v["lift_arm_length_mm"])
    wrist = [base[0] + arm * math.cos(a0), base[1], base[2] + arm * math.sin(a0)]
    return {
        "neck_mm": [float(x) for x in v["neck_joint_position_mm"]],
        "upper_origin_mm": base,
        "lower_origin_mm": lower,
        "wrist_mm": wrist,
        "wheel_front_mm": [0.0, 0.0, float(v["wheel_front_radius_mm"])],
        "wheel_rear_mm": [-float(v["wheel_base_mm"]), 0.0, float(v["wheel_rear_radius_mm"])],
        "rig_lift_angle_rad": float(a0),
    }


def rot_y_up(points: np.ndarray, angle: float) -> np.ndarray:
    """Rotate points about +Y (left) by angle: positive turns +X toward +Z."""
    assert points.ndim == 2 and points.shape[1] == 3, "points must be (n, 3)"
    c, s = math.cos(angle), math.sin(angle)
    return np.column_stack((c * points[:, 0] - s * points[:, 2], points[:, 1], s * points[:, 0] + c * points[:, 2]))


def classify_wheel_group(pts: np.ndarray) -> str:
    """Separate bogie carrier rails from front and rear rotating wheels."""
    min_x, max_x = float(pts[:, 0].min()), float(pts[:, 0].max())
    side = "l" if float(pts[:, 1].mean()) > 0.0 else "r"
    if max_x < -38.0:
        return f"wheel_b{side}"
    if min_x < -15.0:
        return "chassis"
    assert min_x >= -15.0 and max_x <= 14.0, f"unexpected wheel x range: {min_x}..{max_x}"
    return f"wheel_f{side}"


def classify_group(name: str, piece: Optional[str], v_mm: np.ndarray, tris: np.ndarray) -> Optional[str]:
    """Map rig group + geometry into one sliced target key or None when dropped."""
    if piece is None or piece == "tread":
        return None
    if piece == "head":
        return "head_visor" if "Face head_grp" in name else "head_shell"
    if piece == "face":
        if "eye_L" in name:
            return "eye_l"
        return "eye_r" if "eye_R" in name else "head_shell"
    if piece == "body":
        return "chassis"
    if piece in ("arm_upper", "arm_lower", "fork"):
        return {"arm_upper": "lift_upper", "arm_lower": "lift_lower", "fork": "fork"}[piece]
    assert piece == "wheel", f"unexpected piece {piece!r}"
    return classify_wheel_group(v_mm[np.unique(tris)])


def material_tag(target: str, group_name: str, look: str) -> str:
    """Choose semantic usemtl tag allowing v1/v2 shell trims and visor glass styling."""
    assert target in TARGETS and look, "invalid material tag input"
    if target == "head_visor":
        return "visor"
    if target == "head_shell" and ("L_Part" in group_name or "R_Part" in group_name):
        return "ear_ring"
    if target == "chassis":
        if "Back_Part_02" in group_name:
            return "backpack_trim"
        if "R_Bottom_Part" in group_name or "L_Bottom_Part" in group_name:
            return "contact_pad"
        if "backpackLight" in group_name or "backpack_light" in group_name:
            return "light_diffuser"
    return look


def to_target_frame(pts_mm: np.ndarray, target: str, frames: Dict[str, Any]) -> np.ndarray:
    """Transform robot-mm vertices into the sliced OBJ's local joint frame."""
    assert pts_mm.ndim == 2 and pts_mm.shape[1] == 3, "invalid points"
    if target in ("head_shell", "head_visor", "eye_l", "eye_r"):
        return pts_mm - np.asarray(frames["neck_mm"])
    if target == "lift_upper":
        return rot_y_up(pts_mm - np.asarray(frames["upper_origin_mm"]), -frames["rig_lift_angle_rad"])
    if target == "lift_lower":
        return rot_y_up(pts_mm - np.asarray(frames["lower_origin_mm"]), -frames["rig_lift_angle_rad"])
    if target == "fork":
        return pts_mm - np.asarray(frames["wrist_mm"])
    if target in ("wheel_fl", "wheel_fr"):
        return pts_mm - np.asarray(frames["wheel_front_mm"])
    if target in ("wheel_bl", "wheel_br"):
        return pts_mm - np.asarray(frames["wheel_rear_mm"])
    return pts_mm.copy()


def collect_targets(
    v_mm: np.ndarray, raw_groups: Sequence[Tuple[str, np.ndarray, List[str]]], frames: Dict[str, Any]
) -> Dict[str, Dict[str, List[Tuple[np.ndarray, np.ndarray]]]]:
    """Bucket triangles per target and material tag in local joint frames."""
    buckets: Dict[str, Dict[str, List[Tuple[np.ndarray, np.ndarray]]]] = {t: {} for t in TARGETS}
    for name, tris, tri_looks in raw_groups:
        target = classify_group(name, piece_of(name), v_mm, tris)
        if target is None:
            continue
        ids, local_tris = np.unique(tris, return_inverse=True)
        local_pts = to_target_frame(v_mm[ids], target, frames)
        local_tris = local_tris.reshape(-1, 3)
        look_arr = np.asarray(tri_looks)
        for look in sorted(set(tri_looks)):
            sel = local_tris[look_arr == look]
            u_ids, u_tris = np.unique(sel, return_inverse=True)
            tag = material_tag(target, name, look)
            buckets[target].setdefault(tag, []).append((local_pts[u_ids], u_tris.reshape(-1, 3)))
    assert all(buckets[t] for t in TARGETS), "at least one rig target is empty"
    return buckets


def write_tagged_obj(
    out_path: Path, tag_map: Dict[str, List[Tuple[np.ndarray, np.ndarray]]], recenter: bool = False
) -> Tuple[str, int, List[float]]:
    """Write single-file multi-object OBJ partitioned by usemtl tag; return hash, tris, offset."""
    assert tag_map, f"empty tag map for {out_path.name}"
    all_pts = np.concatenate([v for chunks in tag_map.values() for v, _ in chunks])
    offset = (0.5 * (all_pts.min(0) + all_pts.max(0))).tolist() if recenter else [0.0, 0.0, 0.0]
    lines: List[str] = [f"# {out_path.name} sliced in mm"]
    v_base, total_tris = 1, 0
    for tag in sorted(tag_map):
        lines.append(f"o {out_path.stem}_{tag}")
        lines.append(f"usemtl {tag}")
        for verts, tris in tag_map[tag]:
            shifted = verts - np.asarray(offset)
            lines.extend(f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in shifted)
            lines.extend(f"f {a + v_base} {b + v_base} {c + v_base}" for a, b, c in tris)
            v_base += len(verts)
            total_tris += len(tris)
    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return sha256_file(out_path), total_tris, offset


def ensure_ddl_license(ddl_license: Optional[Path]) -> None:
    """Install and SHA-256-verify DDL-ASSET-LICENSE.txt in public/robot/sdk/."""
    SDK_DEST.mkdir(parents=True, exist_ok=True)
    target = SDK_DEST / "DDL-ASSET-LICENSE.txt"
    if target.is_file() and sha256_file(target) == DDL_SHA256:
        return
    if ddl_license is not None:
        assert ddl_license.is_file(), f"missing license file: {ddl_license}"
        data = ddl_license.read_bytes()
    else:
        with urllib.request.urlopen(DDL_URL, timeout=60) as resp:
            data = resp.read()
    if sha256_bytes(data) != DDL_SHA256:
        raise ValueError("DDL-ASSET-LICENSE.txt hash differs from pinned")
    target.write_bytes(data)


def download_rig_to(dest_file: Path) -> None:
    """Download Victor_rig_01.obj at pinned commit and verify SHA-256."""
    assert dest_file.parent.is_dir(), "temp parent missing"
    with urllib.request.urlopen(RIG_URL, timeout=DOWNLOAD_TIMEOUT_S) as resp, dest_file.open("wb") as sink:
        for _ in range(1 << 16):
            block = resp.read(1 << 20)
            if not block:
                break
            sink.write(block)
    if sha256_file(dest_file) != RIG_META["sha256"]:
        raise ValueError("downloaded Victor_rig_01.obj SHA-256 mismatch")


def build_rig_assets(rig_path: Path, ddl_license: Optional[Path]) -> int:
    """Slice rig_path into DEST and write rig_manifest.json."""
    assert MODEL_JSON.is_file(), f"missing {MODEL_JSON}"
    if not rig_path.is_file():
        print(f"rig OBJ not found: {rig_path}", file=sys.stderr)
        return 3
    if sha256_file(rig_path) != RIG_META["sha256"]:
        print("SHA-256 mismatch for Victor_rig_01.obj", file=sys.stderr)
        return 4
    ensure_ddl_license(ddl_license)
    values = json.loads(MODEL_JSON.read_text(encoding="utf-8"))["values"]
    verts_cm, raw_groups = read_obj(rig_path)
    reg = register_rig(verts_cm, raw_groups)
    v_mm = to_robot(verts_cm, reg)
    feats = measure_rig(v_mm, raw_groups)
    check_against_table(values, feats)
    frames = compute_frames(values, feats)
    buckets = collect_targets(v_mm, raw_groups, frames)
    RIG_DEST.mkdir(parents=True, exist_ok=True)
    files_meta: Dict[str, Any] = {}
    for tgt in TARGETS:
        digest, ntris, off = write_tagged_obj(RIG_DEST / f"{tgt}.obj", buckets[tgt], recenter=tgt in ("eye_l", "eye_r"))
        files_meta[f"{tgt}.obj"] = {"sha256": digest, "triangles": ntris}
        if tgt in ("eye_l", "eye_r"):
            frames[f"{tgt}_offset_mm"] = off
    frames["tread_contact_left_y_mm"] = feats["tread_contact_left_y"]
    frames["tread_contact_right_y_mm"] = feats["tread_contact_right_y"]
    manifest = {
        "source": f"{RIG_META['repo']} {Path(RIG_META['path']).name}@{RIG_META['commit'][:12]}",
        "license": "Digital Dream Labs Software Asset License v1.0 (../DDL-ASSET-LICENSE.txt)",
        "rig_sha256": RIG_META["sha256"],
        "files": files_meta,
        "frames": frames,
    }
    (RIG_DEST / "rig_manifest.json").write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {len(TARGETS) + 1} rig files to {RIG_DEST.relative_to(HERE)}")
    return 0


def remove_rig_assets() -> int:
    """Remove generated rig OBJ parts and rig_manifest.json."""
    removed = 0
    for name in [*(f"{t}.obj" for t in TARGETS), "rig_manifest.json"]:
        p = RIG_DEST / name
        if p.is_file():
            p.unlink()
            removed += 1
    if RIG_DEST.is_dir() and not any(RIG_DEST.iterdir()):
        RIG_DEST.rmdir()
    print(f"removed {removed} rig files")
    return 0


def main(argv: Sequence[str]) -> int:
    """CLI entrypoint for fetching/slicing or removing rig 3D assets."""
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--rig", type=Path, help="Local path to Victor_rig_01.obj")
    mode.add_argument("--download", action="store_true", help="Download Victor_rig_01.obj cleanly")
    mode.add_argument("--remove", action="store_true", help="Remove public/robot/sdk/rig")
    p.add_argument("--ddl-license", type=Path, default=None, help="Optional local DDL-ASSET-LICENSE.txt")
    args = p.parse_args(list(argv))
    if args.remove:
        return remove_rig_assets()
    if args.download:
        with tempfile.TemporaryDirectory(prefix="vector-rig-") as td:
            rig_tmp = Path(td) / "Victor_rig_01.obj"
            try:
                download_rig_to(rig_tmp)
            except (OSError, urllib.error.URLError, ValueError) as err:
                print(f"rig download failed: {err}", file=sys.stderr)
                return 5
            return build_rig_assets(rig_tmp, args.ddl_license)
    assert args.rig is not None, "--rig required"
    return build_rig_assets(args.rig, args.ddl_license)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
