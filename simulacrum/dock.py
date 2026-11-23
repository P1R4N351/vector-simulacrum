"""Vector dock (charger) geometry, coordinate transforms, and pin-contact physics..

Zero MuJoCo dependency: reads simulacrum/dock_surface.csv (extracted from
kercre123/vector-animations-raw Vic_charger_hp_geo.obj) and evaluates pin/pad
engagement from body world poses.

Charger frame: origin on ground (z = 0) at the front lip centre, +x inward
toward the marker tower, +y left when facing the ramp, +z up. Lengths in mm.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

CSV_PATH = Path(__file__).resolve().with_name("dock_surface.csv")
DOCK_MOUNTS = (None, "scene", "docked")
DEFAULT_DOCK_POSE_MM = (-120.0, 0.0, 180.0)
OBJ_LIP_Z_MM = -81.2401
OBJ_BOTTOM_Y_MM = -0.5586
MARKER_WALL_X_MM = 93.205
MARKER_POS_MM = (93.205, 0.0, 48.5)
PIN_LEFT_MM = (59.710, -9.850, 13.671)
PIN_RIGHT_MM = (59.710, 9.850, 13.671)
ROBOT_PAD_X_MM = -35.5
ROBOT_PAD_Y_MM = 9.90
ROBOT_PAD_Z_MM = 1.0
PIN_TOL_X_MM = 9.0
PIN_TOL_Y_MM = 3.8
PIN_TOL_Z_LOW_MM = -5.0
PIN_TOL_Z_HIGH_MM = 4.0
CHARGER_SUPPLY_VOLTS = 4.20
DOCKED_START_DIST_MM = 30.0
DOCKED_START_Z_MM = 6.3
DOCKED_START_PITCH_DEG = 8.2
MAX_SURFACE_BOXES = 64


@dataclass(frozen=True)
class DockBox:
    """One oriented box of the charger collision/surface decomposition (mm)."""

    part: str
    role: str
    center_mm: Tuple[float, float, float]
    half_mm: Tuple[float, float, float]
    quat: Tuple[float, float, float, float]


def obj_to_charger_mm(points_maya: np.ndarray) -> np.ndarray:
    """Convert Maya (X_m, Y_m, Z_m) mm to canonical charger (x_c, y_c, z_c) mm (det = +1)."""
    pts = np.asarray(points_maya, dtype=float)
    assert pts.ndim == 2 and pts.shape[1] == 3, "points must be Nx3"
    assert len(pts) > 0, "empty point array"
    return np.column_stack((pts[:, 2] - OBJ_LIP_Z_MM, pts[:, 0], pts[:, 1] - OBJ_BOTTOM_Y_MM))


def load_surface_boxes(path: Optional[Path] = None) -> List[DockBox]:
    """Read OBB records from simulacrum/dock_surface.csv."""
    target = Path(path) if path is not None else CSV_PATH
    assert target.is_file(), f"missing dock surface file: {target}"
    out: List[DockBox] = []
    with target.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            assert len(out) < MAX_SURFACE_BOXES, "dock surface box limit exceeded"
            center = (float(row["x_mm"]), float(row["y_mm"]), float(row["z_mm"]))
            half = (float(row["half_x_mm"]), float(row["half_y_mm"]), float(row["half_z_mm"]))
            quat = (float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"]))
            assert all(h > 0.0 for h in half), f"non-positive half extent on {row['part']}"
            out.append(DockBox(row["part"], row["role"], center, half, quat))
    assert len(out) > 0, f"no boxes read from {target}"
    return out


def resolve_dock_pose(dock: Any) -> Optional[Tuple[float, float, float]]:
    """Return (x_mm, y_mm, yaw_deg) for a dock mode or explicit 3-tuple, else None."""
    if dock is None:
        return None
    if dock in ("scene", "docked"):
        return DEFAULT_DOCK_POSE_MM
    assert isinstance(dock, (tuple, list)) and len(dock) == 3, "dock must be None, 'scene', 'docked', or (x, y, yaw_deg)"
    pose = tuple(float(c) for c in dock)
    assert all(math.isfinite(c) for c in pose), "non-finite dock pose"
    return pose


def docked_robot_pose(dock: Any = "docked", dist_mm: float = DOCKED_START_DIST_MM) -> Tuple[Tuple[float, float], float, float, float]:
    """Return ((x_mm, y_mm), heading_deg, z_mm, nose_down_pitch_deg) for starting on the charger."""
    pose = resolve_dock_pose("docked" if dock is None else dock)
    assert pose is not None and math.isfinite(dist_mm), "docked pose requires valid dock and distance"
    dx_mm, dy_mm, dock_yaw_deg = pose
    yaw = math.radians(dock_yaw_deg)
    rx = dx_mm + dist_mm * math.cos(yaw)
    ry = dy_mm + dist_mm * math.sin(yaw)
    return (rx, ry), dock_yaw_deg - 180.0, DOCKED_START_Z_MM, DOCKED_START_PITCH_DEG


def _in_pin_box(delta_mm: np.ndarray) -> bool:
    return (abs(float(delta_mm[0])) <= PIN_TOL_X_MM and abs(float(delta_mm[1])) <= PIN_TOL_Y_MM
            and PIN_TOL_Z_LOW_MM <= float(delta_mm[2]) <= PIN_TOL_Z_HIGH_MM)


def contact_state(chassis_pos_m: Sequence[float], chassis_mat: Sequence[float],
                  dock_pos_m: Sequence[float], dock_mat: Sequence[float]) -> Dict[str, Any]:
    """Evaluate platform occupancy and dual brass spring-pin contact in the charger frame."""
    c_pos = np.asarray(chassis_pos_m, dtype=float)
    c_rot = np.asarray(chassis_mat, dtype=float).reshape(3, 3)
    d_pos = np.asarray(dock_pos_m, dtype=float)
    d_rot = np.asarray(dock_mat, dtype=float).reshape(3, 3)
    assert c_pos.shape == (3,) and d_pos.shape == (3,), "positions must be 3-vectors"
    assert abs(float(np.linalg.det(d_rot)) - 1.0) < 1e-3, "dock matrix must be proper orthonormal"
    pad_m = np.array([[ROBOT_PAD_X_MM, ROBOT_PAD_Y_MM, ROBOT_PAD_Z_MM],
                      [ROBOT_PAD_X_MM, -ROBOT_PAD_Y_MM, ROBOT_PAD_Z_MM]], dtype=float) * 0.001
    pads_dock_mm = ((c_pos + (c_rot @ pad_m.T).T) - d_pos) @ d_rot * 1000.0
    body_center_mm = ((c_pos - d_pos) @ d_rot) * 1000.0
    # Pair by lateral sign so left/right pins cannot cross-match when centered.
    ordered = sorted(pads_dock_mm, key=lambda p: float(p[1]))
    d_left = np.asarray(ordered[0], dtype=float) - np.asarray(PIN_LEFT_MM, dtype=float)
    d_right = np.asarray(ordered[1], dtype=float) - np.asarray(PIN_RIGHT_MM, dtype=float)
    up_dot = float((d_rot.T @ c_rot[:, 2])[2])
    on_platform = (up_dot > 0.85 and -30.0 <= float(body_center_mm[0]) <= 65.0
                   and abs(float(body_center_mm[1])) <= 32.0 and -2.0 <= float(body_center_mm[2]) <= 35.0)
    on_charger = on_platform and _in_pin_box(d_left) and _in_pin_box(d_right)
    return {
        "on_charger": bool(on_charger),
        "on_charger_platform": bool(on_platform),
        "pin_error_mm": (d_left.tolist(), d_right.tolist()),
    }
